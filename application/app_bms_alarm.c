#define MODULE_LOG_ENABLE LOG_SWITCH_ALARM
#include "app_bms_alarm.h"
#include "modbus_rtu_server_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Modbus.h"
#include "usart.h" // 包含 huart8 等串口

extern volatile uint8_t g_task_alive_flags;
extern volatile uint16_t is_soft_standby;
// 放电时间全局变量
uint32_t discharge_samples[BMS_SAMPLE_BUFFER_SIZE];
uint8_t discharge_idx = 0;
uint8_t discharge_count = 0;

// 充电时间全局变量
uint32_t charge_samples[BMS_SAMPLE_BUFFER_SIZE];
uint8_t charge_idx = 0;
uint8_t charge_count = 0;

uint16_t now_volume;
uint16_t last_volume = 0;

// 【修改点：拆分 Modbus Handler】
modbusHandler_t bms_app;         // 电池 BMS (USART6)
modbusHandler_t sound_light_app; // 声光报警灯 (USART7)

uint16_t bms_results[2] = {0};
uint16_t remainDischargeTime_results[2] = {0};
modbus_t telegram[8];

// 【修改点：分别为两个串口分配独立的缓冲区】
uint16_t bms_modbus_master_buf[128] = {0};
uint16_t sound_light_modbus_master_buf[128] = {0};

// =========================================================
// 1. 发送/控制 专用指令配置
// =========================================================
uint16_t cmd_payload = 0;
modbus_t cmd_telegram = {
    .u8id = SLAVE_LED_ID,
    .u8fct = MB_FC_WRITE_REGISTER,
    .u16CoilsNo = 1,
    .u16reg = &cmd_payload // 绑定数据缓存

};

// =========================================================
// 2. 读取 专用枚举与静态路由表
// =========================================================
typedef enum
{
    READ_BATT_LEVEL = 0,
    READ_REMAIN_DISCHARGE,
    READ_TOTAL_VOLTAGE,
    READ_IS_CHARGING,
    READ_REMAIN_CHARGE,
    READ_TOTAL_CURRENT,
    READ_PROTECT_STATUS,
    READ_MSG_COUNT // 自动计算要读取的指标数量
} BmsReadMsgIdx_t;

// 为每个读取项分配独立的数据接收缓存
uint16_t bms_read_results[READ_MSG_COUNT] = {0};

// BMS 数据读取配置表
static modbus_t bms_read_telegrams[READ_MSG_COUNT] = {
    [READ_BATT_LEVEL] = {.u8id = SLAVE_BMS_ID, .u8fct = MB_FC_READ_REGISTERS, .u16RegAdd = REG_BATTERY_LEVEL, .u16CoilsNo = 1, .u16reg = &bms_read_results[READ_BATT_LEVEL]},
    [READ_REMAIN_DISCHARGE] = {.u8id = SLAVE_BMS_ID, .u8fct = MB_FC_READ_REGISTERS, .u16RegAdd = REG_REMAIN_DISCHARGE, .u16CoilsNo = 1, .u16reg = &bms_read_results[READ_REMAIN_DISCHARGE]},
    [READ_TOTAL_VOLTAGE] = {.u8id = SLAVE_BMS_ID, .u8fct = MB_FC_READ_REGISTERS, .u16RegAdd = REG_TOTAL_VOLTAGE, .u16CoilsNo = 1, .u16reg = &bms_read_results[READ_TOTAL_VOLTAGE]},
    [READ_IS_CHARGING] = {.u8id = SLAVE_BMS_ID, .u8fct = MB_FC_READ_REGISTERS, .u16RegAdd = REG_IS_CHARGING, .u16CoilsNo = 1, .u16reg = &bms_read_results[READ_IS_CHARGING]},
    [READ_REMAIN_CHARGE] = {.u8id = SLAVE_BMS_ID, .u8fct = MB_FC_READ_REGISTERS, .u16RegAdd = REG_REMAIN_CHARGE, .u16CoilsNo = 1, .u16reg = &bms_read_results[READ_REMAIN_CHARGE]},
    [READ_TOTAL_CURRENT] = {.u8id = SLAVE_BMS_ID, .u8fct = MB_FC_READ_REGISTERS, .u16RegAdd = REG_TOTAL_CURRENT, .u16CoilsNo = 1, .u16reg = &bms_read_results[READ_TOTAL_CURRENT]},
    [READ_PROTECT_STATUS] = {.u8id = SLAVE_BMS_ID, .u8fct = MB_FC_READ_REGISTERS, .u16RegAdd = REG_PROTECT_STATUS, .u16CoilsNo = 1, .u16reg = &bms_read_results[READ_PROTECT_STATUS]}};

// 1.喇叭逻辑处理函数 ==========
void buzzer_logic(void)
{
    // 1. 获取目标模式：3m 优先于 7m，0 为关闭。
    uint16_t raw_target_mode = 0;
    if (MB_Reg_Get(CMD_BUZZER_3M) == 1)
    {
        raw_target_mode = 2;
    }
    else if (MB_Reg_Get(CMD_BUZZER_7M) == 1)
    {
        raw_target_mode = 1;
    }

    // 定义状态机状态
    typedef enum
    {
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
    uint16_t current_mode = MB_Reg_Get(STATUS_BUZZER);

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
        cmd_telegram.u16RegAdd = REG_SOUND_LIGHT_CTRL;
        cmd_payload = CMD_SOUND_3M;
        ModbusQuery(&sound_light_app, cmd_telegram); // 【修改：使用 sound_light_app】
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));

        if (err != OP_OK_QUERY)
        {
            LOGE("BUZZER_3M write fail, retrying...\n");
        }
        else
        {
            LOGI("BUZZER_3M write success. Entering BUSY.\n");
            MB_Reg_Set(STATUS_BUZZER, 2);

            // 进入 BUSY 状态，并记录起始时间戳
            state = STATE_BUSY;
            busy_start_tick = xTaskGetTickCount();
        }
    }
    else if (raw_target_mode == 1)
    {
        // 触发 7M 报警
        cmd_telegram.u16RegAdd = REG_SOUND_LIGHT_CTRL;
        cmd_payload = CMD_SOUND_7M;
        ModbusQuery(&sound_light_app, cmd_telegram); // 【修改：使用 sound_light_app】
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));

        if (err != OP_OK_QUERY)
        {
            LOGE("BUZZER_7M write fail, retrying...\n");
        }
        else
        {
            LOGI("BUZZER_7M write success. Entering BUSY.\n");
            MB_Reg_Set(STATUS_BUZZER, 1);

            // 进入 BUSY 状态，并记录起始时间戳
            state = STATE_BUSY;
            busy_start_tick = xTaskGetTickCount();
        }
    }
    else
    {
        cmd_telegram.u16RegAdd = REG_SOUND_STOP;
        cmd_payload = CMD_SOUND_STOP;
        ModbusQuery(&sound_light_app, cmd_telegram); // 【修改：使用 sound_light_app】
        err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));

        if (err != OP_OK_QUERY)
        {
            LOGE("BUZZER_SOUND_STOP write fail, retrying...\n");
        }
        else
        {
            LOGI("BUZZER_SOUND_STOP write success. System cleared.\n");
            MB_Reg_Set(STATUS_BUZZER, 0);
        }
    }
}

// ========== 灯光通信处理逻辑 ==========
/*void led_logic(void)
{
    static uint8_t led_timeout_count = 0; 

    uint16_t cmd_led_switch = MB_Reg_Get(CMD_LED_SWITCH);
    uint16_t status_led_switch = MB_Reg_Get(STATUS_LED_SWITCH);
    uint16_t current_err = MB_Reg_Get(REG_ERROR_CODE);

    if ((cmd_led_switch != status_led_switch) || (current_err & ERR_LED_OFFLINE))
    {
        cmd_telegram.u16RegAdd = REG_LED_CTRL;
       
        // 根据主机想要的状态打包数据
        if (cmd_led_switch == 1)
        {
            cmd_payload = CMD_LED_SLOW_FLASH;
        }
        else
        {
            cmd_payload = CMD_LED_OFF;
        }
        
        ModbusQuery(&sound_light_app, cmd_telegram); // 【修改：使用 sound_light_app】
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));

        if (err == OP_OK_QUERY)
        {
            LOGI("LED write success\n");
            MB_Reg_Set(STATUS_LED_SWITCH, cmd_led_switch);
            // 如果之前是掉线状态，说明现在重新接好了，执行恢复大清洗
            if (current_err & ERR_LED_OFFLINE)
            {
                LOGI("LED Reconnected! Clearing offline error.\n");
                taskENTER_CRITICAL();
                uint16_t err_led = MB_Reg_Get(REG_ERROR_CODE);
                MB_Reg_Set(REG_ERROR_CODE, err_led & ~ERR_LED_OFFLINE);
                taskEXIT_CRITICAL();
            }
            led_timeout_count = 0;
        }
        else
        {
            if (led_timeout_count < 3)
            {
                led_timeout_count++;
                LOGE("LED write fail, timeout count = %d\n", led_timeout_count);
            }
        }
    }

    // ===== 掉线异常叠加判定 =====
    if (led_timeout_count >= 3)
    {
        uint8_t need_log = 0;
        taskENTER_CRITICAL();
        uint16_t err_code = MB_Reg_Get(REG_ERROR_CODE);
        // 只有当错误码还没写进去时，才写一次（防止疯狂打印）
        if ((err_code & ERR_LED_OFFLINE) == 0)
        {
            MB_Reg_Set(REG_ERROR_CODE, err_code | ERR_LED_OFFLINE);
            need_log = 1;
        }
        taskEXIT_CRITICAL();
        if (need_log) // 只在真正状态改变时打印一次
        {
            LOGE("LED OFFLINE ERROR! Timeout >= 3 times.\n");
        }
    }
}*/
void led_logic(void)
{
    static uint8_t led_timeout_count = 0; 
    static TickType_t last_step_tick = 0;
    static uint32_t led_step_counter = 0;
    static uint8_t current_physical_state = 0xFF; 

    uint16_t cmd_led_switch = MB_Reg_Get(CMD_LED_SWITCH);
    uint16_t status_buzzer  = MB_Reg_Get(STATUS_BUZZER);
    uint16_t current_err    = MB_Reg_Get(REG_ERROR_CODE);

    /* PWD_LED follows the Modbus flash-light control register, not each flash phase. */
    HAL_GPIO_WritePin(PWD_LED_GPIO_Port,
                      PWD_LED_Pin,
                      (cmd_led_switch == 1U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    //自定义闪烁参数配置 (单位：250ms/步)
    const uint8_t ON_STEPS  = 8;  // 亮 3步 = 750ms
    const uint8_t OFF_STEPS = 4;  // 灭 1步 = 250ms
    const uint8_t CYCLE_STEPS = ON_STEPS + OFF_STEPS;

    uint8_t need_flash = (cmd_led_switch == 1) || (status_buzzer != 0);

    if (need_flash)
    {
        if (xTaskGetTickCount() - last_step_tick >= pdMS_TO_TICKS(250))
        {
            last_step_tick = xTaskGetTickCount();
            uint8_t current_pos = led_step_counter % CYCLE_STEPS;
            led_step_counter++;

            uint8_t desired_state = (current_pos < ON_STEPS) ? 1 : 0;

            if (desired_state != current_physical_state || (current_err & ERR_LED_OFFLINE))
            {
                cmd_telegram.u16RegAdd = REG_LED_CTRL;
                cmd_payload = desired_state ? CMD_LED_ON : CMD_LED_OFF; 
                
                ModbusQuery(&sound_light_app, cmd_telegram);
                uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));

                if (err == OP_OK_QUERY)
                {
                    LOGI("LED write success: %s\n", desired_state ? "ON" : "OFF");
                    current_physical_state = desired_state;
                    MB_Reg_Set(STATUS_LED_SWITCH, 1); 
                    
                    if (current_err & ERR_LED_OFFLINE)
                    {
                        LOGI("LED Reconnected! Clearing offline error.\n");
                        taskENTER_CRITICAL();
                        uint16_t err_led = MB_Reg_Get(REG_ERROR_CODE);
                        MB_Reg_Set(REG_ERROR_CODE, err_led & ~ERR_LED_OFFLINE);
                        taskEXIT_CRITICAL();
                    }
                    led_timeout_count = 0;
                }
                else
                {
                    if (led_timeout_count < 3)
                    {
                        led_timeout_count++;
                        LOGE("LED write fail, timeout count = %d\n", led_timeout_count);
                    }
                }
            }
        }
    }
    else
    {
        // 彻底关闭的逻辑：确保灯已经灭掉，且停止步进计数
        if (current_physical_state != 0 || (current_err & ERR_LED_OFFLINE))
        {
            cmd_telegram.u16RegAdd = REG_LED_CTRL;
            cmd_payload = CMD_LED_OFF;

            ModbusQuery(&sound_light_app, cmd_telegram);
            uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));

            if (err == OP_OK_QUERY)
            {
                LOGI("LED write success: ALL OFF (System cleared)\n");
                current_physical_state = 0;
                led_step_counter = 0;             // 归零计数器
                MB_Reg_Set(STATUS_LED_SWITCH, 0); // 彻底关闭
                
                if (current_err & ERR_LED_OFFLINE)
                {
                    taskENTER_CRITICAL();
                    uint16_t err_led = MB_Reg_Get(REG_ERROR_CODE);
                    MB_Reg_Set(REG_ERROR_CODE, err_led & ~ERR_LED_OFFLINE);
                    taskEXIT_CRITICAL();
                }
                led_timeout_count = 0;
            }
            else
            {
                if (led_timeout_count < 3)
                {
                    led_timeout_count++;
                    LOGE("LED OFF write fail, timeout count = %d\n", led_timeout_count);
                }
            }
        }
    }

    // ===== 掉线异常叠加判定 =====
    if (led_timeout_count >= 3)
    {
        uint8_t need_log = 0;
        taskENTER_CRITICAL();
        uint16_t err_code = MB_Reg_Get(REG_ERROR_CODE);
        if ((err_code & ERR_LED_OFFLINE) == 0)
        {
            MB_Reg_Set(REG_ERROR_CODE, err_code | ERR_LED_OFFLINE);
            need_log = 1;
        }
        taskEXIT_CRITICAL();
        if (need_log)
        {
            LOGE("LED OFFLINE ERROR! Timeout >= 3 times.\n");
        }
    }
}

void init_bms_alarm_module(void)
{
    // 【修改点：引用 UART6(电池) 和 UART7(灯) 】
    extern UART_HandleTypeDef huart6;
    extern UART_HandleTypeDef huart7;

    init_modbus_master(
        &bms_app,
        &huart6,
        bms_modbus_master_buf,
        sizeof(bms_modbus_master_buf) / sizeof(bms_modbus_master_buf[0]));
    LOGI("BMS modbus master (USART6) start \n");

    init_modbus_master(
        &sound_light_app,
        &huart7,
        sound_light_modbus_master_buf,
        sizeof(sound_light_modbus_master_buf) / sizeof(sound_light_modbus_master_buf[0]));
    LOGI("Sound light modbus master (USART7) start \n");
}

void modbus_alarm_handle(void)
{
    if (is_soft_standby == 1)
    {
        return;
    }
    static TickType_t last_500ms = 0;

    now_volume = MB_Reg_Get(CMD_VOLUME);

    // 音量同步逻辑
    if (now_volume != last_volume)
    {
        cmd_telegram.u16RegAdd = REG_VOLUME_CTRL;
        cmd_payload = now_volume;
        ModbusQuery(&sound_light_app, cmd_telegram); // 【修改：使用 sound_light_app】
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        if (err == OP_OK_QUERY)
        {
            LOGI("BUZZER_VOLUME write success, lastvolume=%d , nowVolume=%d,modbusReg[CMD_VOLUME]:%d\n", last_volume, now_volume, MB_Reg_Get(CMD_VOLUME));
            last_volume = now_volume;
        }
        else
        {
            LOGE("BUZZER_VOLUME write fail : %d \n", err);
        }
    }
    led_logic();    
    // 2. 灯光与喇叭控制 (500ms周期)
    if (xTaskGetTickCount() - last_500ms >= pdMS_TO_TICKS(500))
    {
        last_500ms = xTaskGetTickCount();
        buzzer_logic(); 
    }
}

void modbus_bms_handle(void)
{
    static TickType_t last_500ms = 0;
    static TickType_t last_10s = 0;
    // 每 500ms 轮询
    if (xTaskGetTickCount() - last_500ms >= pdMS_TO_TICKS(500))
    {
        last_500ms += pdMS_TO_TICKS(500);
        // 采样放电时间
        ModbusQuery(&bms_app, bms_read_telegrams[READ_REMAIN_DISCHARGE]);
        int err1 = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        if (err1 == OP_OK_QUERY)
        {
            uint16_t val = bms_read_results[READ_REMAIN_DISCHARGE];
            if (val != 0xFFFF)
            {
                discharge_samples[discharge_idx % BMS_SAMPLE_VALID_COUNT] = val;
                discharge_idx++;
                if (discharge_count < BMS_SAMPLE_VALID_COUNT)
                    discharge_count++;
            }
        }
        else
        {
            LOGE("READ_REMAIN_DISCHARGE read fail : %d \n", err1);
        }        ModbusQuery(&bms_app, bms_read_telegrams[READ_PROTECT_STATUS]);
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS)) == OP_OK_QUERY)
        {
            MB_Reg_Set(STATUS_BMS_PROTECT_STATUS, bms_read_results[READ_PROTECT_STATUS]);
            LOGD("bms protect status = 0x%04X\n", bms_read_results[READ_PROTECT_STATUS]);
        }
        else
        {
            LOGE("bms protect status modbus master read fail\n");
        }
        // 采样充电时间
        ModbusQuery(&bms_app, bms_read_telegrams[READ_REMAIN_CHARGE]);
        int err2 = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        if (err2 == OP_OK_QUERY)
        {
            uint16_t val = bms_read_results[READ_REMAIN_CHARGE];
            if (val != 0xFFFF)
            {
                charge_samples[charge_idx % BMS_SAMPLE_VALID_COUNT] = val;
                charge_idx++;
                if (charge_count < BMS_SAMPLE_VALID_COUNT)
                    charge_count++;
            }
        }
        else
        {
            LOGE("bms charge time modbus master read fail %d \n", err2);
        }
    }

    // 每 10s 执行一次
    if (xTaskGetTickCount() - last_10s >= pdMS_TO_TICKS(10000))
    {
        last_10s += pdMS_TO_TICKS(10000);

        ModbusQuery(&bms_app, bms_read_telegrams[READ_BATT_LEVEL]);
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS)) == OP_OK_QUERY)
        {
            MB_Reg_Set(STATUS_BMS_BATTERY, bms_read_results[READ_BATT_LEVEL]);
            LOGD("bms read success,Battery = %d\n", bms_read_results[READ_BATT_LEVEL]);
            taskENTER_CRITICAL();
            uint16_t err_bms = MB_Reg_Get(REG_ERROR_CODE);
            MB_Reg_Set(REG_ERROR_CODE, err_bms & ~ERR_BMS_READ_FAIL);
            taskEXIT_CRITICAL();
        }
        else
        {
            LOGE("bms read fail  \n");
            MB_Reg_Set(STATUS_BMS_BATTERY, 0);
            // 读取失败，设置系统错误码为BMS读取出错
            taskENTER_CRITICAL();
            uint16_t err_bms = MB_Reg_Get(REG_ERROR_CODE);
            MB_Reg_Set(REG_ERROR_CODE, err_bms | ERR_BMS_READ_FAIL);
            taskEXIT_CRITICAL();
        }

        ModbusQuery(&bms_app, bms_read_telegrams[READ_TOTAL_VOLTAGE]);
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS)) == OP_OK_QUERY)
        {
            MB_Reg_Set(STATUS_BMS_TOTAL_VOLTAGE, bms_read_results[READ_TOTAL_VOLTAGE]);
            LOGD("bms total voltage = %d\n", bms_read_results[READ_TOTAL_VOLTAGE]);
        }
        else
        {
            LOGE("bms total voltage modbus master read fail  \n");
        }

        ModbusQuery(&bms_app, bms_read_telegrams[READ_TOTAL_CURRENT]);
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS)) == OP_OK_QUERY)
        {
            int16_t val = (int16_t)bms_read_results[READ_TOTAL_CURRENT];
            // 判断是否充电状态
            if (val < 0)
            {
                MB_Reg_Set(STATUS_BMS_TOTAL_CURRENT, (uint16_t)(-val));
                MB_Reg_Set(STATUS_BMS_IS_charge, 0); // 放电状态
            }
            else if (val > 0)
            {
                MB_Reg_Set(STATUS_BMS_TOTAL_CURRENT, (uint16_t)val);
                MB_Reg_Set(STATUS_BMS_IS_charge, 1); // 充电状态
            }
            else
            {
                MB_Reg_Set(STATUS_BMS_IS_charge, 2); // 静止状态
            }
            LOGD("bms charge status = %d\n", MB_Reg_Get(STATUS_BMS_IS_charge));
            LOGD("bms total current = %d===%d\n", MB_Reg_Get(STATUS_BMS_TOTAL_CURRENT), val);
        }
        else
        {
            LOGE("bms total current modbus master read fail  \n");
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

            MB_Reg_Set(STATUS_BMS_REMAIN_DISCHARGE_TIME, avg);
            LOGD("remain discharge time avg = %d min\n", avg);
        }
        else
        {
            MB_Reg_Set(STATUS_BMS_REMAIN_DISCHARGE_TIME, 0xFFFF);
            LOGE("remain discharge time fail\n");
        }
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
            MB_Reg_Set(STATUS_BMS_REMAIN_CHARGE_TIME, (uint16_t)(sum / charge_count));
            LOGD("remain charge time avg = %d min\n", (uint16_t)(sum / charge_count));
        }
        else
        {
            MB_Reg_Set(STATUS_BMS_REMAIN_CHARGE_TIME, 0xFFFF);
            LOGE("remain charge time fail\n");
        }
        memset(charge_samples, 0, sizeof(charge_samples));
        charge_idx = 0;
        charge_count = 0;
    }
}

// =================================================================
// 设备上电自检程序 (POST: Power-On Self-Test)
// =================================================================
void power_on_self_test(void)
{
    LOGI("[POST] System Power-On Self-Test started...\n");
    osDelay(2500);

    cmd_telegram.u16RegAdd = 0x2303;
    cmd_payload = 0x0001; // 数据位：播放物理顺序第 1 曲

    ModbusQuery(&sound_light_app, cmd_telegram); // 【修改：使用 sound_light_app】
    uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

    if (err == OP_OK_QUERY)
    {
        LOGI("[POST] Sound & Light test command sent successfully!\n");
    }
    else
    {
        LOGE("[POST] Sound & Light test command FAILED!\n");
    }
    //bug修改：上电自检结束时“清洗”硬件状态
    osDelay(5500); 
    cmd_telegram.u16RegAdd = 0x6003; 
    cmd_payload = 0x0000;            
    ModbusQuery(&sound_light_app, cmd_telegram); // 【修改：使用 sound_light_app】
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    // 4. 同步 Modbus 状态机寄存器，防止后续逻辑误判
    MB_Reg_Set(STATUS_LED_SWITCH, 0);
    MB_Reg_Set(STATUS_BUZZER, 0);

    LOGI("[POST] Power-On Self-Test completed!\n");
}
