#include "app_bms_alarm.h"
#include "modbus_rtu_server_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Modbus.h"
#include "usart.h" // 包含 huart8 等串口

extern volatile uint8_t g_task_alive_flags;

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

modbusHandler_t bms_sound_light_app;
uint16_t bms_results[2] = {0};
uint16_t remainDischargeTime_results[2] = {0};
modbus_t telegram[8];
uint16_t modbus_master_buf[128] = {0};

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
    [READ_TOTAL_CURRENT] = {.u8id = SLAVE_BMS_ID, .u8fct = MB_FC_READ_REGISTERS, .u16RegAdd = REG_TOTAL_CURRENT, .u16CoilsNo = 1, .u16reg = &bms_read_results[READ_TOTAL_CURRENT]}};

// 1.喇叭逻辑处理函数 ==========
void buzzer_logic(void)
{
   // 1. 获取目标模式：3m 优先于 7m，0 为关闭。
    uint16_t raw_target_mode = 0;
    // 【修改点 1】替换为 MB_Reg_Get
    if (MB_Reg_Get(CMD_BUZZER_3M) == 1) {
        raw_target_mode = 2;
    } else if (MB_Reg_Get(CMD_BUZZER_7M) == 1) {
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
        ModbusQuery(&bms_sound_light_app, cmd_telegram);
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
        ModbusQuery(&bms_sound_light_app, cmd_telegram);
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
        // 发送关闭命令（或者停止指令）收尾，确保寄存器状态归零
        cmd_telegram.u16RegAdd = REG_SOUND_STOP;
        cmd_payload = CMD_SOUND_STOP;
        ModbusQuery(&bms_sound_light_app, cmd_telegram);
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
void led_logic(void)
{
    uint16_t cmd_led_switch = MB_Reg_Get(CMD_LED_SWITCH);

    if (cmd_led_switch == 1 && MB_Reg_Get(STATUS_LED_SWITCH) == 0)
    {
        LOGI(" LED on \n");
        cmd_telegram.u16RegAdd = REG_LED_CTRL;
        cmd_payload = CMD_LED_SLOW_FLASH;
        ModbusQuery(&bms_sound_light_app, cmd_telegram);
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        if (err == OP_OK_QUERY)
        {
            LOGI("LED on write success \n");
            MB_Reg_Set(STATUS_LED_SWITCH, 1);
        }
        else
        {
            LOGE("LED on write fail : %d \n", err);
        }
    }

    if (cmd_led_switch == 0 && MB_Reg_Get(STATUS_LED_SWITCH) == 1)
    {
        LOGI(" LED off \n");
        cmd_telegram.u16RegAdd = REG_LED_CTRL;
        cmd_payload = CMD_LED_OFF;
        ModbusQuery(&bms_sound_light_app, cmd_telegram);
        uint32_t err = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
        if (err == OP_OK_QUERY)
        {
            LOGI("LED off write success : %d \n", err);
            MB_Reg_Set(STATUS_LED_SWITCH, 0);
        }
        else
        {
            LOGE("LED off write fail : %d \n", err);
        }
    }
}
void init_bms_alarm_module(void)
{
    extern UART_HandleTypeDef huart8;
    init_modbus_master(
        &bms_sound_light_app,
        &huart8,
        modbus_master_buf,
        sizeof(modbus_master_buf) / sizeof(modbus_master_buf[0]));
    LOGI("bms sound light modbus master start \n");
}

void modbus_alarm_handle(void)
{
    static TickType_t last_500ms = 0;
    
   now_volume = MB_Reg_Get(CMD_VOLUME);

        // 音量同步逻辑
        if (now_volume != last_volume)
        {
            cmd_telegram.u16RegAdd = REG_VOLUME_CTRL;
            cmd_payload = now_volume;
            ModbusQuery(&bms_sound_light_app, cmd_telegram);
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
    // 2. 灯光与喇叭控制 (500ms周期)
    if (xTaskGetTickCount() - last_500ms >= pdMS_TO_TICKS(500))
    {
        last_500ms = xTaskGetTickCount();
        buzzer_logic(); // 调用内部的喇叭状态机
        led_logic();    // 调用内部的LED处理
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
            ModbusQuery(&bms_sound_light_app, bms_read_telegrams[READ_REMAIN_DISCHARGE]);
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
            }

            // 采样充电时间
            ModbusQuery(&bms_sound_light_app, bms_read_telegrams[READ_REMAIN_CHARGE]);
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

            // 读取电量
            ModbusQuery(&bms_sound_light_app, bms_read_telegrams[READ_BATT_LEVEL]);
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS)) == OP_OK_QUERY)
            {
                MB_Reg_Set(STATUS_BMS_BATTERY, bms_read_results[READ_BATT_LEVEL]);
                LOGD("bms led sound modbus master read success,Battery = %d\n", bms_read_results[READ_BATT_LEVEL]);
                //读取成功，清除系统错误码
                taskENTER_CRITICAL();
                uint16_t err_bms = MB_Reg_Get(REG_ERROR_CODE);
                MB_Reg_Set(REG_ERROR_CODE, err_bms & ~ERR_BMS_READ_FAIL);
                taskEXIT_CRITICAL();
            }
            else
            {
                LOGE("bms led sound modbus master read fail  \n");
                //读取失败，设置系统错误码为BMS读取出错
                taskENTER_CRITICAL();
                uint16_t err_bms = MB_Reg_Get(REG_ERROR_CODE);
                MB_Reg_Set(REG_ERROR_CODE, err_bms | ERR_BMS_READ_FAIL);
                taskEXIT_CRITICAL();
            }

            // 读取总电压
            ModbusQuery(&bms_sound_light_app, bms_read_telegrams[READ_TOTAL_VOLTAGE]);
            if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS)) == OP_OK_QUERY)
            {
                MB_Reg_Set(STATUS_BMS_TOTAL_VOLTAGE, bms_read_results[READ_TOTAL_VOLTAGE]);
                LOGD("bms total voltage = %d\n", bms_read_results[READ_TOTAL_VOLTAGE]);
            }
            else
            {
                LOGE("bms total voltage modbus master read fail  \n");
            }

            // 读取总电流及充放电状态
            ModbusQuery(&bms_sound_light_app, bms_read_telegrams[READ_TOTAL_CURRENT]);
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

