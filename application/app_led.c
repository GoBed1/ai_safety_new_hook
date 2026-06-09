#include "app_led.h"
#include "system_def.h"
#include "modbus_rtu_server_interface.h"
#include "FreeRTOS.h"
#include "task.h"

// 引入外部 Modbus 底层依赖
extern modbusHandler_t bms_sound_light_app;
extern modbus_t cmd_telegram;
extern uint16_t cmd_payload;

//全局变量LED 上下文对象
static led_context_t g_led_ctx = {
    .state = LED_IDLE,
    .retry_count = 0,
    .has_event = 0,
    .current_evt = { .type = LED_EVT_CMD_OFF },
    .target_cmd = { .type = LED_CMD_NONE }
};

// 内部状态机
static led_cmd_t led_fsm_engine(led_context_t *ctx, led_event_t evt)
{
    switch (ctx->state)
    {
        case LED_IDLE:
            if (evt.type == LED_EVT_CMD_ON) {
                ctx->state = LED_FLASHING;
                return (led_cmd_t){ .type = LED_CMD_SEND_ON };
            }
            if (evt.type == LED_EVT_LINK_DOWN) {
                ctx->state = LED_OFFLINE;
                return (led_cmd_t){ .type = LED_CMD_NONE };
            }
            break;

        case LED_FLASHING:
            if (evt.type == LED_EVT_CMD_OFF) {
                ctx->state = LED_IDLE;
                return (led_cmd_t){ .type = LED_CMD_SEND_OFF };
            }
            if (evt.type == LED_EVT_LINK_DOWN) {
                ctx->state = LED_OFFLINE;
                return (led_cmd_t){ .type = LED_CMD_NONE };
            }
            break;

        case LED_OFFLINE:
            if (evt.type == LED_EVT_LINK_UP) {
                ctx->state = LED_IDLE; 
                return (led_cmd_t){ .type = LED_CMD_NONE };
            }
            // 离线期间持续发送探测命令
            if (evt.type == LED_EVT_CMD_ON) return (led_cmd_t){ .type = LED_CMD_SEND_ON };
            if (evt.type == LED_EVT_CMD_OFF) return (led_cmd_t){ .type = LED_CMD_SEND_OFF };
            break;
    }
    return (led_cmd_t){ .type = LED_CMD_NONE };
}

//内部执行
static int led_hardware_execute(led_cmd_t cmd)
{
    if (cmd.type == LED_CMD_SEND_ON) {
        cmd_telegram.u16RegAdd = REG_LED_CTRL;
        cmd_payload = CMD_LED_SLOW_FLASH;
        ModbusQuery(&bms_sound_light_app, cmd_telegram);
        return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
    }
    else if (cmd.type == LED_CMD_SEND_OFF) {
        cmd_telegram.u16RegAdd = REG_LED_CTRL;
        cmd_payload = CMD_LED_OFF;
        ModbusQuery(&bms_sound_light_app, cmd_telegram);
        return ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MODBUS_WAIT_TIMEOUT_MS));
    }
    return -1;
}

//事件层
static void led_sense_event(led_context_t *ctx)
{
    ctx->has_event = 0; // 默认无事件
    uint16_t cmd_led_switch = MB_Reg_Get(CMD_LED_SWITCH);
    uint16_t status_led_switch = MB_Reg_Get(STATUS_LED_SWITCH);
    uint16_t current_err = MB_Reg_Get(REG_ERROR_CODE);

    if (current_err & ERR_LED_OFFLINE) 
    {
        if (ctx->state != LED_OFFLINE) {
            ctx->current_evt.type = LED_EVT_LINK_DOWN;
            ctx->has_event = 1;
        } else {
            ctx->current_evt.type = (cmd_led_switch == 1) ? LED_EVT_CMD_ON : LED_EVT_CMD_OFF;
            ctx->has_event = 1;
        }
    } 
    else if (cmd_led_switch != status_led_switch) 
    {
        ctx->current_evt.type = (cmd_led_switch == 1) ? LED_EVT_CMD_ON : LED_EVT_CMD_OFF;
        ctx->has_event = 1;
    }
}

//状态层
static void led_think_state(led_context_t *ctx)
{
    if (ctx->has_event) {
        ctx->target_cmd = led_fsm_engine(ctx, ctx->current_evt);
    } else {
        ctx->target_cmd.type = LED_CMD_NONE;
    }
}

//执行层
static void led_act_execute(led_context_t *ctx)
{
    if (ctx->target_cmd.type == LED_CMD_NONE) return;

    int err = led_hardware_execute(ctx->target_cmd);

    uint16_t cmd_led_switch = MB_Reg_Get(CMD_LED_SWITCH);
    uint16_t current_err = MB_Reg_Get(REG_ERROR_CODE);

    if (err == OP_OK_QUERY) 
    {
        LOGI("LED write success\n");
        MB_Reg_Set(STATUS_LED_SWITCH, cmd_led_switch);
        ctx->retry_count = 0;

        // 如果刚才还在掉线，说明探测成功，清洗错误码并恢复状态
        if (current_err & ERR_LED_OFFLINE) 
        {
            LOGI("LED Reconnected!\n");
            taskENTER_CRITICAL();
            MB_Reg_Set(REG_ERROR_CODE, current_err & ~ERR_LED_OFFLINE);
            taskEXIT_CRITICAL();
            
            // 补充投递一次 LINK_UP，让状态机回 IDLE
            led_fsm_engine(ctx, (led_event_t){.type = LED_EVT_LINK_UP});
        }
    } 
    else 
    {
        if (ctx->retry_count < 3) ctx->retry_count++;
        
        if (ctx->retry_count >= 3 && !(current_err & ERR_LED_OFFLINE)) 
        {
            taskENTER_CRITICAL();
            MB_Reg_Set(REG_ERROR_CODE, current_err | ERR_LED_OFFLINE);
            taskEXIT_CRITICAL();
            LOGE("LED OFFLINE ERROR!\n");
        }
    }
}


void led_logic_update(void)
{
    // 1. 事件层：查探发生了什么
    led_sense_event(&g_led_ctx);

    // 2. 状态层：送入状态机思考下一步指令
    led_think_state(&g_led_ctx);

    // 3. 执行层：执行发送与超时计数
    led_act_execute(&g_led_ctx);
}