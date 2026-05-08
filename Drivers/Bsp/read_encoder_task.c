#include "read_encoder_task.h"


volatile uint8_t g_task_alive_flags;
// --------------------------------

// 放电时间全局变量
uint32_t discharge_samples[BMS_SAMPLE_BUFFER_SIZE];
uint8_t discharge_idx = 0;
uint8_t discharge_count = 0;
// 充电时间全局变量
uint32_t charge_samples[BMS_SAMPLE_BUFFER_SIZE];
uint8_t charge_idx = 0;
uint8_t charge_count = 0;

extern UART_HandleTypeDef huart7;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart8;
extern RTC_HandleTypeDef hrtc;
// Slave全局变量
static modbusHandler_t encoder_forward_server;
#define REGS_TOTAL_NUM 256
uint16_t modbus_registers[REGS_TOTAL_NUM] = {0};
uint16_t modbus_input_registers[REGS_TOTAL_NUM] = {0};
uint16_t now_volume;      // 假设音量寄存器地址为103
uint16_t last_volume = 0; // 确保开机第一次循环必定能进if分支

void RFID_master_thread(void *argument);
void RFID_OnFrame(RFIDClient *c, const uint8_t *frm, uint16_t len);
void RFID_CheckOffline(RFIDClient *c);
void RFID_WriteToModbusRegs(RFIDClient *c);

osThreadId_t ai_safy_slave_handle;
const osThreadAttr_t ai_safy_slave_attributes = {
    .name = "AISafySlave",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t ai_safy_master_handle;
const osThreadAttr_t ai_safy_master_attributes = {
    .name = "AISafyMaster",
    .stack_size = 1024 * 6,
    .priority = (osPriority_t)osPriorityNormal1,
};
// RFID
osThreadId_t RFID_master_handle;
const osThreadAttr_t RFID_master_attributes = {
    .name = "RFIDMaster",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};
// gps待机线程
osThreadId_t gps_standby_handle;
const osThreadAttr_t gps_standby_attributes = {
    .name = "GPSStandby",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};
// 心跳检测任务
osThreadId_t relay_heartbeat_handle;
const osThreadAttr_t relay_heartbeat_attributes = {
    .name = "RelayHeartbeat",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

uint8_t first_findVolume = 1;
EventGroupHandle_t eg = NULL; // 初始化事件组为NULL
void EventGroupCreate_Init(void)
{
    if (eg == NULL)
    {
        eg = xEventGroupCreate();
    }
}
// ========== 内部函数：大端拼接 ==========
static uint32_t be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           ((uint32_t)p[3] << 0);
}

// ========== 内部函数：解析RFID帧 ==========
static int parse_frame(const uint8_t *frm, uint16_t len,
                       uint8_t *out_rssi, uint8_t *out_rfid_battery, uint32_t *out_uid)
{
    if (len < 14)
        return 0;
    if (frm[0] != 0x1B || frm[1] != 0x39 || frm[2] != 0x01)
        return 0;

    *out_rssi = frm[7];
    *out_rfid_battery = frm[9];
    *out_uid = be_u32(&frm[10]);

    return 1;
}

// ========== 内部函数：查找UID ==========
static int find_uid(RFIDClient *c, uint32_t uid)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) && c->tags[i].uid == uid)
        {
            return i;
        }
    }
    return -1;
}

// ========== 内部函数：分配空闲槽位 ==========
static int alloc_slot(RFIDClient *c)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) == 0)
        {
            return i;
        }
    }
    return -1;
}

// rfid相关3个函数的实现
// 1.写入Modbus寄存器
void RFID_WriteToModbusRegs(RFIDClient *c)
{
    // 更新位图到 modbus_registers[3] 的低8位
    modbus_registers[REG_RFID_VALID] =
        (modbus_registers[REG_RFID_VALID] & 0xFF00) | c->valid_bitmap;

    // 写8组数据到 modbus_registers[4~27]
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        uint16_t base = REG_RFID_BASE + i * 3;

        if (c->valid_bitmap & (1U << i))
        {
            // 标签有效时写入数据
            modbus_registers[base + 0] = (uint16_t)(c->tags[i].uid >> 16);
            modbus_registers[base + 1] = (uint16_t)(c->tags[i].uid & 0xFFFF);
            modbus_registers[base + 2] = ((uint16_t)c->tags[i].rssi << 8) | c->tags[i].rfid_battery;
        }
        else
        {
            // 标签无效时清零
            modbus_registers[base + 0] = 0;
            modbus_registers[base + 1] = 0;
            modbus_registers[base + 2] = 0;
        }
    }
}
// 2.检查离线
void RFID_CheckOffline(RFIDClient *c)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t timeout = pdMS_TO_TICKS(RFID_OFFLINE_MS);

    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) == 0)
            continue;

        if ((uint32_t)(now - c->tags[i].last_seen_tick) > timeout)
        {
            c->valid_bitmap &= ~(1U << i); // 标记标签无效
            LOGI("RFID offline: idx=%d, UID=0x%08X\n",
                 i, (unsigned int)c->tags[i].uid);
            memset(&c->tags[i], 0, sizeof(c->tags[i])); // 清除标签结构体数据
        }
    }
}
// 3. 收到帧后更新
void RFID_OnFrame(RFIDClient *c, const uint8_t *frm, uint16_t len)
{
    uint8_t rssi, rfid_battery;
    uint32_t uid;

    if (!parse_frame(frm, len, &rssi, &rfid_battery, &uid))
    {
        LOGE("RFID parse fail\n");
        return;
    }

    uint32_t now = xTaskGetTickCount();

    int idx = find_uid(c, uid);
    if (idx < 0)
    {
        idx = alloc_slot(c);
        if (idx < 0)
        {
            LOGE("RFID slots full, UID=0x%08X\n", (unsigned int)uid);
            return;
        }
        c->valid_bitmap |= (1U << idx);
        c->tags[idx].uid = uid;
        LOGI("RFID new tag: idx=%d, UID=0x%08X\n", idx, (unsigned int)uid);
    }

    c->tags[idx].rssi = rssi;
    c->tags[idx].rfid_battery = rfid_battery;
    c->tags[idx].last_seen_tick = now;

    LOGD("RFID update: idx=%d, UID=0x%08X, RSSI=%d, rfid_battery=%d\n",
         idx, (unsigned int)uid, rssi, rfid_battery);
}

void init_ai_safy_slave(void)
{
    // 初始化寄存器数组
    memset(modbus_registers, 0, sizeof(modbus_registers));
    
    // HACK 设置系统信息
    modbus_registers[REG_ERROR_CODE] = 0x0000; // 清零错误码/在线状态
    // 配置MODBUS Slave处理器
    encoder_forward_server.uModbusType = MB_SLAVE;
    encoder_forward_server.u8id = FORWARD_SLAVE_ADDR; // Slave ID=3
    encoder_forward_server.port = &huart7;
    encoder_forward_server.EN_Port = NULL; // 无RS485控制引脚
    encoder_forward_server.EN_Pin = 0;
    encoder_forward_server.u16regs = modbus_registers;            // 保持寄存器
    encoder_forward_server.u16inputregs = modbus_input_registers; // 输入寄存器
    encoder_forward_server.u16regsize = REGS_TOTAL_NUM;
    encoder_forward_server.u16timeOut = 1000; // 1秒超时
    encoder_forward_server.xTypeHW = USART_HW;
    LOGI("MODBUS-RTU Slave ID: %d\r\n", FORWARD_SLAVE_ADDR);
    
    // 初始化MODBUS Slave
    ModbusInit(&encoder_forward_server);
    ModbusStart(&encoder_forward_server);

    LOGI("Register range: 0x0000-0x%04X (%d registers)\r\n",
         REGS_TOTAL_NUM - 1, REGS_TOTAL_NUM);
}

modbusHandler_t bms_sound_light_app;
modbusHandler_t rfid_app;

uint16_t bms_results[2] = {0};
uint16_t remainDischargeTime_results[2] = {0};

uint16_t firstVolume_results[2] = {0};
modbus_t telegram[8];

modbus_t telegram2[3];

// HACK
uint16_t modbus_master_buf[128] = {0};
uint16_t modbus_master_buf2[128] = {0};

// 喇叭逻辑处理函数
void buzzer_logic(void)
{
    // 1. 获取目标模式：3m 优先于 7m，0 为关闭。
    uint16_t raw_target_mode = 0;
    if (modbus_registers[CMD_BUZZER_3M] == 1)
    {
        raw_target_mode = 2;
    }
    else if (modbus_registers[CMD_BUZZER_7M] == 1)
    {
        raw_target_mode = 1;
    }

    // 定义状态机状态
    typedef enum {
        STATE_IDLE = 0,
        STATE_BUSY
    } buzzer_state_t;

    static buzzer_state_t state = STATE_IDLE;
    static TickType_t busy_start_tick = 0;
    
    // 语音单次播放的最长耗时（单位：毫秒）
    const uint32_t VOICE_PLAY_TIME_MS = 5300; 

    // ================= 状态机处理 =================

    // 如果当前是 BUSY 状态，判断是否需要退出
    if (state == STATE_BUSY)
    {
        if ((xTaskGetTickCount() - busy_start_tick) >= pdMS_TO_TICKS(VOICE_PLAY_TIME_MS))
        {
            state = STATE_IDLE;
        }
        else
        {
            return;
        }
    }

    // 当前处于 IDLE 状态，判断是否需要下发指令
    uint16_t current_mode = modbus_registers[STATUS_BUZZER];

    // 如果没检测到人，且喇叭状态也已经是关闭的，那就什么都不做
    if (raw_target_mode == 0 && current_mode == 0)
    {
        return;
    }

    uint32_t err = 0;

    // 根据检测结果发送动作
    if (raw_target_mode == 2) 
    {
        // 触发 3M 报警
        telegram[1].u16RegAdd = REG_SOUND_LIGHT_CTRL;
        telegram[1].u16reg[0] = CMD_SOUND_3M;
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        
        if (err != OP_OK_QUERY) {
            LOGE("BUZZER_3M write fail, retrying...\n");
        } else {
            LOGI("BUZZER_3M write success. Entering BUSY.\n");
            modbus_registers[STATUS_BUZZER] = 2;
            
            // 进入 BUSY 状态，并记录起始时间戳
            state = STATE_BUSY;
            busy_start_tick = xTaskGetTickCount();
        }
    }
    else if (raw_target_mode == 1) 
    {
        // 触发 7M 报警
        telegram[1].u16RegAdd = REG_SOUND_LIGHT_CTRL;
        telegram[1].u16reg[0] = CMD_SOUND_7M;
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        
        if (err != OP_OK_QUERY) {
            LOGE("BUZZER_7M write fail, retrying...\n");
        } else {
            LOGI("BUZZER_7M write success. Entering BUSY.\n");
            modbus_registers[STATUS_BUZZER] = 1;
            
            // 进入 BUSY 状态，并记录起始时间戳 
            state = STATE_BUSY;
            busy_start_tick = xTaskGetTickCount();
        }
    }
    else 
    {
        // 发送关闭命令（或者停止指令）收尾，确保寄存器状态归零
        telegram[1].u16RegAdd = REG_SOUND_STOP;
        telegram[1].u16reg[0] = CMD_SOUND_STOP;
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        
        if (err != OP_OK_QUERY) {
            LOGE("BUZZER_SOUND_STOP write fail, retrying...\n");
        } else {
            LOGI("BUZZER_SOUND_STOP write success. System cleared.\n");
            modbus_registers[STATUS_BUZZER] = 0;
        }
    }
}

void modbus_TxData_logic(void)
{
    uint16_t cmd_led_switch = modbus_registers[CMD_LED_SWITCH];
    // 读取寄存器0的值，例如
    // 灯打开或关闭命令
    if (cmd_led_switch == 1 && modbus_registers[STATUS_LED_SWITCH] == 0)
    {
        LOGI(" LED on \n");
        telegram[1].u16RegAdd = REG_LED_CTRL;
        telegram[1].u16reg[0] = CMD_LED_SLOW_FLASH;// 慢闪，爆闪改为61
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        if (err == OP_OK_QUERY)
        {
            LOGI("LED on write success \n");
            // 更新状态寄存器regs[100]且清除命令寄存器regs[0]
            modbus_registers[STATUS_LED_SWITCH] = 1;
        }
        else
        {
            LOGE("LED on write fail : %d \n", err);
        }
    }

    if (cmd_led_switch == 0 && modbus_registers[STATUS_LED_SWITCH] == 1)
    {
        LOGI(" LED off \n");
        telegram[1].u16RegAdd = REG_LED_CTRL;
        telegram[1].u16reg[0] = CMD_LED_OFF;
        ModbusQuery(&bms_sound_light_app, telegram[1]);
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        if (err == OP_OK_QUERY)
        {
            LOGI("LED off write success : %d \n", err);
            modbus_registers[STATUS_LED_SWITCH] = 0;
        }
        else
        {
            LOGE("LED off write fail : %d \n", err);
        }
    }

    // 喇叭逻辑处理
    buzzer_logic();
}

void ai_safy_master_thread(void *argument)
{
    memset(modbus_master_buf, 0, sizeof(modbus_master_buf));

    bms_sound_light_app.uModbusType = MB_MASTER;
    bms_sound_light_app.port = &huart8;
    bms_sound_light_app.u8id = 0;
    bms_sound_light_app.u16timeOut = 500;
    bms_sound_light_app.EN_Port = NULL;
    bms_sound_light_app.EN_Pin = 0;
    bms_sound_light_app.u16regs = modbus_master_buf;
    bms_sound_light_app.u16regsize = sizeof(modbus_master_buf) / sizeof(modbus_master_buf[0]);
    bms_sound_light_app.xTypeHW = USART_HW;
    ModbusInit(&bms_sound_light_app);
    ModbusStart(&bms_sound_light_app);

    LOGI("bms sound light modbus master start \n");

    telegram[0].u8id = SLAVE_BMS_ID;
    telegram[0].u8fct = MB_FC_READ_REGISTERS;
    telegram[0].u16RegAdd = REG_BATTERY_LEVEL;
    telegram[0].u16CoilsNo = 1;
    telegram[0].u16reg = bms_results;

    telegram[1].u8id = SLAVE_LED_ID;
    telegram[1].u8fct = MB_FC_WRITE_REGISTER;
    telegram[1].u16CoilsNo = 1;

    telegram[3].u8id = SLAVE_BMS_ID;
    telegram[3].u8fct = MB_FC_READ_REGISTERS;
    telegram[3].u16RegAdd = REG_REMAIN_DISCHARGE;
    telegram[3].u16CoilsNo = 1;
    telegram[3].u16reg = remainDischargeTime_results;

    telegram[4].u8id = SLAVE_BMS_ID;
    telegram[4].u8fct = MB_FC_READ_REGISTERS;
    telegram[4].u16RegAdd = REG_TOTAL_VOLTAGE;
    telegram[4].u16CoilsNo = 1;

    telegram[5].u8id = SLAVE_BMS_ID;
    telegram[5].u8fct = MB_FC_READ_REGISTERS;
    telegram[5].u16RegAdd = REG_IS_CHARGING;
    telegram[5].u16CoilsNo = 1;

    telegram[6].u8id = SLAVE_BMS_ID;
    telegram[6].u8fct = MB_FC_READ_REGISTERS;
    telegram[6].u16RegAdd = REG_REMAIN_CHARGE;
    telegram[6].u16CoilsNo = 1;

    telegram[7].u8id = SLAVE_BMS_ID;
    telegram[7].u8fct = MB_FC_READ_REGISTERS;
    telegram[7].u16RegAdd = REG_TOTAL_CURRENT;
    telegram[7].u16CoilsNo = 1;

    TickType_t last_10s = xTaskGetTickCount();
    TickType_t last_500ms = xTaskGetTickCount();
    TickType_t last_5s = xTaskGetTickCount();
    // 心跳机制计时器
    TickType_t last_heartbeat = xTaskGetTickCount();

    for (;;)
    {
        g_task_alive_flags |= TASK_AI_SAFY_ALIVE;
        // 设置音量
        now_volume = modbus_registers[CMD_VOLUME];

        // 读取音量寄存器
        if (now_volume != last_volume)
        { // 如果音量有变化
            telegram[1].u16RegAdd = REG_VOLUME_CTRL;
            telegram[1].u16reg[0] = now_volume;
            ModbusQuery(&bms_sound_light_app, telegram[1]); // 调整喇叭的实际输出音量。
            uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
            if (err == OP_OK_QUERY)
            {
                LOGI("BUZZER_VOLUME write success, lastvolume=%d , nowVolume=%d,modbusReg[CMD_VOLUME]:%d\n", last_volume, now_volume, modbus_registers[CMD_VOLUME]);
                last_volume = now_volume; // 更新上一次的音量记录以供下次比较使用。
            }
            else
            {
                LOGE("BUZZER_VOLUME write fail : %d \n", err);
            }
        }

        // 每 500ms 轮询一次led & buzzer
        if (xTaskGetTickCount() - last_500ms >= pdMS_TO_TICKS(500))
        {
            last_500ms += pdMS_TO_TICKS(500);
            modbus_TxData_logic();

            // 每500ms采样一次放电时间
            ModbusQuery(&bms_sound_light_app, telegram[3]);
            int err1 = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
            if (err1 != OP_OK_QUERY)
            {
                LOGE("bms dischange time modbus master read fail : %d \n", err1);
            }
            else
            {
                uint16_t val = telegram[3].u16reg[0];
                if (val != 0xFFFF)
                {
                    discharge_samples[discharge_idx % BMS_SAMPLE_VALID_COUNT] = val; 
                    discharge_idx++;
                    if (discharge_count < BMS_SAMPLE_VALID_COUNT)
                        discharge_count++;
                }
            }

            // 每500ms采样一次充电时间
            ModbusQuery(&bms_sound_light_app, telegram[6]);
            int err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
            if (err != OP_OK_QUERY)
            {
                LOGE("bms charge time modbus master read fail : %d \n", err);
            }
            else
            {
                uint16_t val = telegram[6].u16reg[0]; 
                if (val != 0xFFFF)
                {
                    charge_samples[charge_idx % BMS_SAMPLE_VALID_COUNT] = val; 
                    charge_idx++;
                    if (charge_count < BMS_SAMPLE_VALID_COUNT)
                        charge_count++;
                }
            }
        }

        // 每 10s 执行一次（读取电量/电流） &&   每10s执行一次平均值放入寄存器（剩余放电时间）
        if (xTaskGetTickCount() - last_10s >= pdMS_TO_TICKS(10000))
        {
            last_10s += pdMS_TO_TICKS(10000);
            ModbusQuery(&bms_sound_light_app, telegram[0]);
            int err1 = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
            if (err1 != OP_OK_QUERY)
            {
                LOGE("bms led sound modbus master read fail : %d \n", err1);
            }
            else
            {
                modbus_registers[STATUS_BMS_BATTERY] = telegram[0].u16reg[0];
                LOGD("bms led sound modbus master read success,Battery = %d\n", telegram[0].u16reg[0]);
            }
            // 是否在充电状态：0-否，1-是

            // 总电压读取
            ModbusQuery(&bms_sound_light_app, telegram[4]);
            int err2 = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
            if (err2 != OP_OK_QUERY)
            {
                LOGE("bms total voltage modbus master read fail : %d \n", err2);
            }
            else
            {
                modbus_registers[STATUS_BMS_TOTAL_VOLTAGE] = telegram[4].u16reg[0];
                LOGD("bms total voltage = %d\n", telegram[4].u16reg[0]);
            }

            // //总电流读取 && 判断是否充电状态0-否，1-是
            ModbusQuery(&bms_sound_light_app, telegram[7]);
            int err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
            if (err != OP_OK_QUERY)
            {
                LOGE("bms total current modbus master read fail : %d \n", err);
            }
            else
            {
                int16_t val = (int16_t)telegram[7].u16reg[0];
                // 判断是否充电状态
                if (val < 0)
                {
                    modbus_registers[STATUS_BMS_TOTAL_CURRENT] = (uint16_t)(-val);
                    modbus_registers[STATUS_BMS_IS_charge] = 0; // 放电状态
                }
                else if (val > 0)
                {
                    modbus_registers[STATUS_BMS_TOTAL_CURRENT] = (uint16_t)val;
                    modbus_registers[STATUS_BMS_IS_charge] = 1; // 充电状态
                }
                else
                {
                    modbus_registers[STATUS_BMS_IS_charge] = 2; // 无充电或放电状态
                }
                LOGD("bms charge status = %d\n", modbus_registers[STATUS_BMS_IS_charge]);
                LOGD("bms total current = %d===%d\n", modbus_registers[STATUS_BMS_TOTAL_CURRENT], val);
            }

            // 每500ms采样一次放电时间，每10s执行一次平均值放入寄存器（剩余放电时间）
            if (discharge_count > 0)
            {
                uint32_t sum = 0;
                for (uint8_t i = 0; i < discharge_count; i++)
                {
                    sum += discharge_samples[i];
                }
                uint16_t avg = (uint16_t)(sum / discharge_count);
                modbus_registers[STATUS_BMS_REMAIN_DISCHARGE_TIME] = avg;
                LOGD("remain discharge time avg = %d min\n", modbus_registers[STATUS_BMS_REMAIN_DISCHARGE_TIME]);
            }
            else
            {
                modbus_registers[STATUS_BMS_REMAIN_DISCHARGE_TIME] = 0xFFFF;
                LOGE("remain discharge time: no valid sample\n");
            }

            // 清空缓冲
            memset(discharge_samples, 0, sizeof(discharge_samples));
            discharge_idx = 0;
            discharge_count = 0;

            // 每500ms采样一次充电时间，每10s执行一次平均值放入寄存器（剩余充电时间）
            if (charge_count > 0)
            {
                uint32_t sum = 0;
                for (uint8_t i = 0; i < charge_count; i++)
                {
                    sum += charge_samples[i];
                }
                uint16_t avg = (uint16_t)(sum / charge_count);
                modbus_registers[STATUS_BMS_REMAIN_CHARGE_TIME] = avg;
                LOGD("remain charge time avg = %d min\n", modbus_registers[STATUS_BMS_REMAIN_CHARGE_TIME]);
            }
            else
            {
                modbus_registers[STATUS_BMS_REMAIN_CHARGE_TIME] = 0xFFFF;
                LOGE("remain charge time: no valid sample\n");
            }

            // 清空缓冲
            memset(charge_samples, 0, sizeof(charge_samples));
            charge_idx = 0;
            charge_count = 0;
        }

        // 判断工作状态
        if ((modbus_registers[STATUS_LED_SWITCH] == 1 && modbus_registers[STATUS_BUZZER] == 1) ||
            (modbus_registers[STATUS_LED_SWITCH] == 1 && modbus_registers[STATUS_BUZZER] == 0) ||
            (modbus_registers[STATUS_LED_SWITCH] == 0 && modbus_registers[STATUS_BUZZER] == 1))
        {
            if (modbus_registers[STATUS_BMS_BATTERY] <= LOW_BATTERY_THRESHOLD)
            {
                modbus_registers[STATUS_WORK_MODE] = 2; // 工作状态为2，即低电量警告模式
            }
            else
            {
                modbus_registers[STATUS_WORK_MODE] = 1; // 工作状态为1，即正常工作模式
            }
        }
        else
        {
            if (modbus_registers[STATUS_BMS_BATTERY] <= LOW_BATTERY_THRESHOLD)
            {
                modbus_registers[STATUS_WORK_MODE] = 2; // 工作状态为2，即低电量警告模式
            }
            else
            {

                modbus_registers[STATUS_WORK_MODE] = 0; // 工作状态为0，即非正常工作模式
            }
        }

        osDelay(500);
    }
}

RFIDClient RFID_client;
void RFID_master_thread(void *argument)
{
    
    TickType_t last_check = xTaskGetTickCount();
    for (;;)
    {
        g_task_alive_flags |= TASK_RFID_ALIVE;
        EventBits_t uxBits = xEventGroupWaitBits(
            eg,            // 事件组
            EVENT_RFID_RX, // 等待这个事件
            pdTRUE,        // 自动清除标志
            pdFALSE,       // 不需要等待所有位
            pdMS_TO_TICKS(500)   // 有看门狗的超时时间，这里是500ms
        );
        if ((uxBits & EVENT_RFID_RX) != 0)
        {
            LOGD("Recevice rfid: ");
            LOGD("modbus_reg[3] = %04X (", modbus_registers[3]);
            for (int i = 15; i >= 0; i--)
            {
                LOGD("%d", (modbus_registers[3] >> i) & 1);
                if (i % 4 == 0 && i != 0)
                    LOGD(" ");
            }
            RFID_OnFrame(&RFID_client,
                         RFID_client.Rx_RFID_buf,
                         RFID_client.Rx_RFID_len);

            // 写入Modbus寄存器
            RFID_WriteToModbusRegs(&RFID_client);
        }

        if ((TickType_t)(xTaskGetTickCount() - last_check) >= pdMS_TO_TICKS(5000))
        {
            last_check = xTaskGetTickCount();
            RFID_CheckOffline(&RFID_client);
            RFID_WriteToModbusRegs(&RFID_client);
        }

        osDelay(1000);
    }
}
// GPS待机线程
void gps_standby_thread(void *argument)
{
    config_gps_app();
    rtc_power_init();

    for (;;)
    {
        g_task_alive_flags |= TASK_GPS_ALIVE;
        update_gps_app();
        rtc_power_schedule_check();
        HAL_GPIO_TogglePin(GPIOD, H_B_LED_Pin);
        osDelay(1000);
    }
}
// 继电器心跳检测线程
void relay_heartbeat_thread(void *argument)
{
    // 初始化上次心跳值和接收时间
    uint16_t last_heartbeat_val = modbus_registers[STATUS_HEART_BEAT];
    TickType_t recv_heartbeat_time = xTaskGetTickCount();
    // 刚开机时默认是有电状态，1代表有电，0代表断电
    uint8_t relay_is_on = 1;
    for (;;)
    {
        g_task_alive_flags |= TASK_RELAY_ALIVE;
        uint16_t current_heartbeat = modbus_registers[STATUS_HEART_BEAT];

        // 1. 检查心跳是否有变化
        if (current_heartbeat != last_heartbeat_val)
        {
            last_heartbeat_val = current_heartbeat;
            recv_heartbeat_time = xTaskGetTickCount(); // 更新最后一次收到心跳的时间
            if (relay_is_on == 0)
            {
                // 收到心跳，继电器引脚置为1 (上电)
                HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_SET);
                last_volume = 0xFFFF; // 音量更新,主循环会更新
                relay_is_on = 1;      // 标记为有电状态
                modbus_registers[STATUS_LED_SWITCH]=0;// 灯关闭,如果指令寄存器=1，主循环会处理的

            }
            else
            {
                // 收到心跳，继电器引脚置为1 (上电)
                HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_SET);
                LOGI("[Heartbeat] Host active. Relay 2 SET to 1. Reg[104]=%d\r\n", current_heartbeat);
            }
        }
        else
        {
            // 如果没有变化，检查是否超时 1 分钟 (60000 毫秒)
            if ((xTaskGetTickCount() - recv_heartbeat_time) > pdMS_TO_TICKS(HEARTBEAT_TIMEOUT_MS) && relay_is_on == 1)
            {
                modbus_registers[CMD_LED_SWITCH] = 0;
                modbus_registers[CMD_BUZZER_7M] = 0;
                modbus_registers[CMD_BUZZER_3M] = 0;
                modbus_registers[STATUS_LED_SWITCH] = 0;
                modbus_registers[STATUS_BUZZER] = 0;

                relay_is_on = 0; 

                HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_RESET);
                LOGE("[Heartbeat] Timeout (>1 min). Relay 2 SET to 0.\r\n");
            }
        }
        osDelay(1000);
    }
}

void init_read_encoder_task()
{
    EventGroupCreate_Init();
    init_ai_safy_slave();
    init_uart_manage();

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RFID_client.rx_buf, (uint16_t)sizeof(RFID_client.rx_buf));
    modbus_registers[CMD_VOLUME] = DEFAULT_VOLUME;

    HAL_GPIO_WritePin(RELAY_1_PIN_GPIO_Port, RELAY_1_PIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_SET);

    relay_heartbeat_handle = osThreadNew(relay_heartbeat_thread, NULL, &relay_heartbeat_attributes);
    ai_safy_master_handle = osThreadNew(ai_safy_master_thread, NULL, &ai_safy_master_attributes);
    RFID_master_handle = osThreadNew(RFID_master_thread, NULL, &RFID_master_attributes);
    gps_standby_handle = osThreadNew(gps_standby_thread, NULL, &gps_standby_attributes);
}