#ifndef APP_SYS_SUPERVISOR_H
#define APP_SYS_SUPERVISOR_H
#include "stm32h743xx.h"
#include "system_def.h"
typedef enum
{
    LED_MODE_NORMAL = 0, // 正常模式：1s亮, 1s灭
    LED_MODE_BURST,      // 爆闪模式：无限循环 1步亮, 1步灭
    LED_MODE_BLINK_PAUSE // 闪烁停顿模式：闪烁N次(每次1步亮1步灭)后停顿2s时间
} LedAnimMode_t;

// LED 控制器
typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    uint16_t step_period_ms;  // 外部调用这个函数的真实周期（如 50ms, 100ms）
    uint16_t cycle_step;     // 当前运行步数 (1 step = 100ms)
    LedAnimMode_t last_mode; // 追踪上一次的模式
    uint8_t last_count;      // 追踪上一次的闪烁次数
} LedAnimCtx_t;


void LED_Animation_Step(LedAnimCtx_t *ctx, LedAnimMode_t target_mode, uint8_t blink_count);

void sys_supervisor_process(void);

void work_mode_logic(void);

#endif // APP_SYS_SUPERVISOR_H
