#define MODULE_LOG_ENABLE LOG_SWITCH_RELAY
#include "app_relay.h"
#include "modbus_rtu_server_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"

extern volatile uint8_t g_task_alive_flags;
extern uint16_t last_volume;
extern volatile uint16_t is_soft_standby; // 软休眠状态标志（爆闪灯断电）
static uint16_t last_heartbeat_val = 0;
static TickType_t recv_heartbeat_time = 0;
static uint8_t relay_is_on = 1;
void relay_app_init(void)
{
    // 初始化引脚状态
        HAL_GPIO_WritePin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin, GPIO_PIN_SET);
    // 初始化心跳记录（使用真实的运行时间）
    last_heartbeat_val = MB_Reg_Get(STATUS_HEART_BEAT);
    recv_heartbeat_time = xTaskGetTickCount();
    relay_is_on = 1;
}
void process_relay_logic(void)
{
    static TickType_t last_1000ms = 0;

    // 内部控制：每 1000ms 执行一次核心逻辑
    if (xTaskGetTickCount() - last_1000ms >= pdMS_TO_TICKS(1000))
    {
        last_1000ms = xTaskGetTickCount();

        // 检测心跳使能没有打开
        if (MB_Reg_Get(HEARTBEAT_ENABLE) == 0)
        {
            // 如果心跳功能被禁用，确保继电器保持在默认状态（上电）
            if (relay_is_on == 0)
            {
                HAL_GPIO_WritePin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(PWD_LED_GPIO_Port, PWD_LED_Pin, GPIO_PIN_SET);

                relay_is_on = 1;
                LOGI("[Heartbeat] Disabled. Relay 2 SET to 1.\r\n");
            }
            // 即使关闭心跳，也要后台“喂狗”，防止以后重新打开时瞬间触发超时
            recv_heartbeat_time = xTaskGetTickCount();
            last_heartbeat_val = MB_Reg_Get(STATUS_HEART_BEAT);
            return; // 心跳功能关闭，跳过后续逻辑
        }

        if (is_soft_standby == 1)
        {
            recv_heartbeat_time = xTaskGetTickCount();          // 不断“喂狗”，更新接收时间
            last_heartbeat_val = MB_Reg_Get(STATUS_HEART_BEAT); // 同步最新值
            relay_is_on = 0;                                    // 同步当前继电器真实状态
            return;                                             // 提前退出，不执行后续的心跳判断
        }

        uint16_t current_heartbeat = MB_Reg_Get(STATUS_HEART_BEAT);

        // 1. 检查心跳是否有变化
        if (current_heartbeat != last_heartbeat_val)
        {
            last_heartbeat_val = current_heartbeat;
            recv_heartbeat_time = xTaskGetTickCount(); // 更新最后一次收到心跳的时间

            if (relay_is_on == 0)
            {
                // 收到心跳，继电器引脚置为1 (上电)
                HAL_GPIO_WritePin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(PWD_LED_GPIO_Port, PWD_LED_Pin, GPIO_PIN_SET);
                LOGI("[Heartbeat] Heartbeat received. Relay 2 SET to 1.\r\n");
                last_volume = 0xFFFF;             // 音量更新,主循环会更新
                relay_is_on = 1;                  // 标记为有电状态
                MB_Reg_Set(STATUS_LED_SWITCH, 0); // 灯关闭
            }
            else
            {
                // 收到心跳，继电器引脚置为1 (上电)
                HAL_GPIO_WritePin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(PWD_LED_GPIO_Port, PWD_LED_Pin, GPIO_PIN_SET);
                LOGI("[Heartbeat] Host active. Relay 2 SET to 1. Reg[104]=%d\r\n", current_heartbeat);
            }
            taskENTER_CRITICAL();
            uint16_t err_heart = MB_Reg_Get(REG_ERROR_CODE);
            if (err_heart & ERR_HEARTBEAT_TIMEOUT) // 如果之前存在心跳超时错误，收到心跳后清除该错误
            {
                MB_Reg_Set(REG_ERROR_CODE, err_heart & ~ERR_HEARTBEAT_TIMEOUT);
            }
            taskEXIT_CRITICAL();
        }
        else
        {
            // 是否超过静默时间
            if ((xTaskGetTickCount() - recv_heartbeat_time) > pdMS_TO_TICKS(HEARTBEAT_SILENCE_MS))
            {
                MB_Reg_Set(CMD_LED_SWITCH, 0);
                MB_Reg_Set(CMD_BUZZER_7M, 0);
                MB_Reg_Set(CMD_BUZZER_3M, 0);

                // LOGE("[Heartbeat] Silence Timeout! Lights & Buzzer OFF \r\n");
            }
            // 如果没有变化，检查是否超时 1 分钟 (60000 毫秒)
            if ((xTaskGetTickCount() - recv_heartbeat_time) > pdMS_TO_TICKS(HEARTBEAT_TIMEOUT_MS) && relay_is_on == 1)
            {
                MB_Reg_Set(CMD_LED_SWITCH, 0);
                MB_Reg_Set(CMD_BUZZER_7M, 0);
                MB_Reg_Set(CMD_BUZZER_3M, 0);

                relay_is_on = 0;

                HAL_GPIO_WritePin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(PWD_LED_GPIO_Port, PWD_LED_Pin, GPIO_PIN_RESET);
                taskENTER_CRITICAL();
                uint16_t err_heart = MB_Reg_Get(REG_ERROR_CODE);
                MB_Reg_Set(REG_ERROR_CODE, err_heart | ERR_HEARTBEAT_TIMEOUT); // 叠加心跳超时错误
                taskEXIT_CRITICAL();
                LOGE("[Heartbeat] Timeout (>1 min). Relay 2 SET to 0.\r\n");
            }
        }
    }
}
