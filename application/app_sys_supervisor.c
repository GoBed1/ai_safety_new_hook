#define MODULE_LOG_ENABLE LOG_SWITCH_SYS_SUPERVISOR
#include "app_sys_supervisor.h"
#include "cmsis_os.h"
#include "modbus_rtu_server_interface.h"
#include "main.h"
static LedAnimCtx_t heart_led = {
    .port = HEART_LED_GPIO_Port,
    .pin  = HEART_LED_Pin,
    .step_period_ms = 200, //单次亮/灭的时长200ms
    .cycle_step = 0, 
    .last_mode = LED_MODE_NORMAL, 
    .last_count = 0
};

void sys_supervisor_process(void)
{

  // 获取当前系统错误码
    SystemErrorCode_t current_error = (SystemErrorCode_t)MB_Reg_Get(REG_ERROR_CODE);

    LedAnimMode_t target_mode = LED_MODE_NORMAL;
    uint8_t target_blink_count = 0;

    // 错误码翻译成动作指令
    if (current_error != ERR_NONE)
    {
        if ((current_error & (current_error - 1)) != 0)
        {
            // 多重错误
            target_mode = LED_MODE_BURST;
        }
        else
        {
            // 单个错误
            target_mode = LED_MODE_BLINK_PAUSE;
            
            // 查表赋予闪烁次数
            if (current_error == ERR_HEARTBEAT_TIMEOUT)
                target_blink_count = 1;
            else if (current_error == ERR_LED_OFFLINE)
                target_blink_count = 2;
            else if (current_error == ERR_BMS_READ_FAIL)
                target_blink_count = 3;
        }
    }

    // 执行错误
    LED_Animation_Step(&heart_led, target_mode, target_blink_count);
}

// 工作状态模式判定逻辑
void work_mode_logic(void)
{
    uint16_t current_error = MB_Reg_Get(REG_ERROR_CODE);
    uint16_t current_battery = MB_Reg_Get(STATUS_BMS_BATTERY);

    GPIO_PinState relay2_state = HAL_GPIO_ReadPin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin);


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




void LED_Animation_Step(LedAnimCtx_t *ctx, LedAnimMode_t target_mode, uint8_t blink_count)
{
    // 防御性编程：防止除以0
    if (ctx->step_period_ms == 0) ctx->step_period_ms = 100; 

    // 1. 状态改变时重置动画
    if (ctx->last_mode != target_mode || ctx->last_count != blink_count)
    {
        ctx->cycle_step = 0;
        ctx->last_mode = target_mode;
        ctx->last_count = blink_count;
    }

    // 2. 状态机执行
    switch (target_mode)
    {
        case LED_MODE_NORMAL:
        {
            // 正常模式：维持 1秒亮，1秒灭 
            // (1秒需要的步数 = 1000 / 周期)
            uint16_t steps_1s = 1000 / ctx->step_period_ms;
            
            if (ctx->cycle_step < steps_1s)
                HAL_GPIO_WritePin(ctx->port, ctx->pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(ctx->port, ctx->pin, GPIO_PIN_RESET);
            
            if (++ctx->cycle_step >= (steps_1s * 2)) ctx->cycle_step = 0;
            break;
        }

        case LED_MODE_BURST:
        {
            // 爆闪模式：1步亮，1步灭 (无限循环)
            // 占用 2 个 step 为一个周期
            if ((ctx->cycle_step % 2) == 0)
                HAL_GPIO_WritePin(ctx->port, ctx->pin, GPIO_PIN_SET);
            else
                HAL_GPIO_WritePin(ctx->port, ctx->pin, GPIO_PIN_RESET);
            
            if (++ctx->cycle_step >= 2) ctx->cycle_step = 0;
            break;
        }

        case LED_MODE_BLINK_PAUSE:
        {
            if (blink_count > 0)
            {
                // 【核心算法】
                // 1次闪烁 = 1步亮 + 1步灭 = 2个step
                uint16_t blink_phase_steps = blink_count * 2; 
                
                // 停顿 2000ms = 2000 / 周期 对应的步数
                uint16_t pause_phase_steps = 2000 / ctx->step_period_ms; 
                
                // 总周期步数
                uint16_t total_steps = blink_phase_steps + pause_phase_steps; 

                if (ctx->cycle_step < blink_phase_steps)
                {
                    // 闪烁阶段：偶数步亮，奇数步灭
                    if ((ctx->cycle_step % 2) == 0)
                        HAL_GPIO_WritePin(ctx->port, ctx->pin, GPIO_PIN_SET);
                    else
                        HAL_GPIO_WritePin(ctx->port, ctx->pin, GPIO_PIN_RESET);
                }
                else
                {
                    // 停顿冷却区：全灭
                    HAL_GPIO_WritePin(ctx->port, ctx->pin, GPIO_PIN_RESET);
                }

                if (++ctx->cycle_step >= total_steps) ctx->cycle_step = 0;
            }
            else
            {
                HAL_GPIO_WritePin(ctx->port, ctx->pin, GPIO_PIN_RESET);
                ctx->cycle_step = 0;
            }
            break;
        }
    }
}
