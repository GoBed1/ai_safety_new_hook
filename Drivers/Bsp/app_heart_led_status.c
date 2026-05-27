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
            // 多重错误：1秒1个周期 (10步)，快闪2次 (亮-灭-亮-灭-停)
            if (cycle_step < 4)
            {
                if (cycle_step % 2 == 0)
                    HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_SET);
                else
                    HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
            }
            else
            {
                HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET); // 后面0.6秒纯灭
            }
            cycle_step++;
            if (cycle_step >= 10)
                cycle_step = 0;
        }
        else
        {
            // 错误1：接收心跳超时，每3秒一个周期，前0.5秒点亮，后面2.5秒熄灭
            if (current_error == ERR_HEARTBEAT_TIMEOUT)
            {
                if (cycle_step < 5)
                {
                    // 前 0.5 秒（0~4步）点亮
                    HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_SET);
                }
                else
                {
                    // 第 0.5 秒之后直到第 3 秒结束（5~29步），全部熄灭
                    HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
                }

                cycle_step++;
                if (cycle_step >= 30)
                    cycle_step = 0;
            }

            // 错误3：bms读取错误，每3秒一个周期，第一秒闪3次，后面2秒熄灭冷却
            else if (current_error == ERR_BMS_READ_FAIL)
            {
                // 错误模式：3秒一周期（30步）
                uint8_t blink_count = 3;

                if (cycle_step < 10) // 前 1s 闪烁区
                {
                    if (cycle_step < (blink_count * 2))
                    {
                        // 偶数步亮，奇数步灭
                        if (cycle_step % 2 == 0)
                            HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_SET);
                        else
                            HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
                    }
                    else
                    {
                        HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET); // 闪完灭
                    }
                }
                else // 后 2s 熄灭冷却区
                {
                    HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
                }

                cycle_step++;
                if (cycle_step >= 30)
                    cycle_step = 0;
            }
            else 
            {
                // 如果发生了未定义的单一错误，直接强制灯熄灭，防止状态卡死
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
    uint16_t current_led = MB_Reg_Get(STATUS_LED_SWITCH);
    uint16_t current_buzzer = MB_Reg_Get(STATUS_BUZZER);
    uint16_t current_battery = MB_Reg_Get(STATUS_BMS_BATTERY);

    WorkMode_t target_mode = MODE_COM_WORKING;

    // 优先级 1：异常判断 (有任何错误码，模式设为 3)
    if (current_error != ERR_NONE)
    {
        target_mode = DEVICE_ERROR;
    }
    // 优先级 2：低电量判断 (电量低于阈值，模式设为 2)
    else if (current_battery <= LOW_BATTERY_THRESHOLD)
    {
        target_mode = MODE_LOW_BATTERY;
    }
    // 工作/报警状态 (声或光开启，模式设为 1)
    else if ((current_led == 1 && current_buzzer == 1) ||
             (current_led == 1 && current_buzzer == 0) ||
             (current_led == 0 && current_buzzer == 1))
    {
        target_mode = MODE_COM_WORKING;
    }
    // 优先级 4：待机状态 (模式设为 0)
    else
    {
        target_mode = MODE_DEVICE_STANDBY;
    }
    MB_Reg_Set(STATUS_WORK_MODE, (uint16_t)target_mode);
}