#include "app_relay.h"
#include "modbus_rtu_server_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board.h" 

extern volatile uint8_t g_task_alive_flags;
extern uint16_t last_volume; 

static uint16_t last_heartbeat_val = 0;
static TickType_t recv_heartbeat_time = 0;
static uint8_t relay_is_on = 1;

void relay_app_init(void)
{
    // 初始化引脚状态
    HAL_GPIO_WritePin(RELAY_1_PIN_GPIO_Port, RELAY_1_PIN_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_SET);

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

        uint16_t current_heartbeat = MB_Reg_Get(STATUS_HEART_BEAT);

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
                MB_Reg_Set(STATUS_LED_SWITCH, 0); // 灯关闭
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
                MB_Reg_Set(CMD_LED_SWITCH, 0);
                MB_Reg_Set(CMD_BUZZER_7M, 0);
                MB_Reg_Set(CMD_BUZZER_3M, 0);
                MB_Reg_Set(STATUS_LED_SWITCH, 0);
                MB_Reg_Set(STATUS_BUZZER, 0);

                relay_is_on = 0; 

                HAL_GPIO_WritePin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin, GPIO_PIN_RESET);
                LOGE("[Heartbeat] Timeout (>1 min). Relay 2 SET to 0.\r\n");
            }
        }
    }
}


