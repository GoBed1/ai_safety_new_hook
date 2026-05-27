#include "app_heart_led_status.h"
#include "cmsis_os.h"
#include "modbus_rtu_server_interface.h"
#include "main.h"
static uint8_t cycle_step = 0;

void sys_supervisor_process(void)
{

    // 1. 获取当前系统错误码
    SystemErrorCode_t current_error = (SystemErrorCode_t)MB_Reg_Get(REG_ERROR_CODE);

    // 2. 状态机逻辑处理
    if (current_error != ERR_NONE)
    {
        // 错误>=2：多重错误，每秒闪2次
        if ((current_error & (current_error - 1)) != 0)
        {
            // 多重错误：均匀爆闪，无停顿 (亮200ms -> 灭200ms，无限循环)
            if ((cycle_step % 4) < 2)
            {
                HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_SET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
            }

            cycle_step++;
            if (cycle_step >= 4)
                cycle_step = 0;
        }
        else // 单个错误处理逻辑
        {
            uint8_t blink_count = 0; // 记录需要闪烁的次数

            // 【查表】：根据具体的错误码，赋予不同的闪烁次数
            if (current_error == ERR_HEARTBEAT_TIMEOUT)
            {
                blink_count = 1;
            }
            else if (current_error == ERR_LED_OFFLINE)
            {
                blink_count = 2;
            }
            else if (current_error == ERR_BMS_READ_FAIL)
            {
                blink_count = 3;
            }

            if (blink_count > 0)
            {
                if (cycle_step < (blink_count * 4))
                {
                    // 用 % 4 来切分 400ms 的闪烁动作：前 200ms 亮，后 200ms 灭
                    if ((cycle_step % 4) < 2)
                        HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_SET);
                    else
                        HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
                }
                else
                {
                    // 闪够次数后，剩下的时间全部处于熄灭冷却区
                    HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
                }

                cycle_step++;
                if (cycle_step >= 30)
                    cycle_step = 0;
            }
            else
            {
                // 兜底保护：如果发生了未定义的单一错误，直接强制灯熄灭，防止状态卡死
                HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
                cycle_step = 0;
            }
        }
    }
    else
    {

        // 正常模式：1s 翻转一次
        if (cycle_step % 10 == 0)
        {
            HAL_GPIO_TogglePin(GPIOD, H_B_LED_Pin);
        }

        cycle_step++;
        if (cycle_step >= 20)
            cycle_step = 0;
    }
}

// 工作状态模式判定逻辑
void work_mode_logic(void)
{
    uint16_t current_error = MB_Reg_Get(REG_ERROR_CODE);
    uint16_t current_battery = MB_Reg_Get(STATUS_BMS_BATTERY);

    GPIO_PinState relay2_state = HAL_GPIO_ReadPin(RELAY_2_PIN_GPIO_Port, RELAY_2_PIN_Pin);

    WorkMode_t target_mode = MODE_DEVICE_STANDBY;

    // 异常判断
    if (current_error != ERR_NONE)
    {
        target_mode = DEVICE_ERROR;
    }
    // 软待机（爆闪灯断电）
    else if (relay2_state == GPIO_PIN_RESET)
    {
        target_mode = MODE_DEVICE_STANDBY;
    }
    // 低电量判断
    else if (current_battery <= LOW_BATTERY_THRESHOLD)
    {
        target_mode = MODE_LOW_BATTERY;
    }
    // 正常工作
    else
    {
        target_mode = MODE_COM_WORKING;
    }
    MB_Reg_Set(STATUS_WORK_MODE, (uint16_t)target_mode);
}