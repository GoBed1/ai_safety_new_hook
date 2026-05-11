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
            // 错误模式：3秒一周期（30步） 
            uint8_t blink_count = (current_error > 5) ? 5 : (uint8_t)current_error;
            
            if (cycle_step < 10) // 前 1s 闪烁区
            {
                if (cycle_step < (blink_count * 2))
                {
                    // 偶数步亮，奇数步灭
                    if (cycle_step % 2 == 0) HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_SET);
                    else HAL_GPIO_WritePin(GPIOD, H_B_LED_Pin, GPIO_PIN_RESET);
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
            if (cycle_step >= 30) cycle_step = 0;
        }
        else
        {
            // 正常模式：1s 翻转一次 
            if (cycle_step % 10 == 0) 
            {
                HAL_GPIO_TogglePin(GPIOD, H_B_LED_Pin);
            }
            
            cycle_step++;
            if (cycle_step >= 20) cycle_step = 0;
        }
    }
