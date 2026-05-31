#include "app_4g.h"
#include "uart_manage.h"
#include "modbus_rtu_server_interface.h"

// 业务处理函数声明
static void handle_light_cmd(ProtocolFrame_t *frame);
static void handle_bms_cmd(ProtocolFrame_t *frame);
static void handle_system_cmd(ProtocolFrame_t *frame);
static void handle_time_cmd(ProtocolFrame_t *frame);
static void handle_heartbeat_cmd(ProtocolFrame_t *frame);
// 全局解析上下文
ParserCtx_t g_parser_ctx;
// 发送序列号缓存
static uint8_t g_tx_seq = 0;

// 内容ID映射表
const IdHandlerMap_t handler_map[] = {
    {0x01, handle_light_cmd},     // 灯光警报业务
    {0x02, handle_bms_cmd},       // 电池读取业务
    {0x03, handle_system_cmd},    // 系统相关业务
    {0x04, handle_time_cmd},      // 时间相关业务
    {0x05, handle_heartbeat_cmd}, // 心跳包业务
    {0x00, NULL}                  // 结束标记
};

// CRC16-CCITT (poly 0x1021) initial 0x0000
static uint16_t crc16_ccitt(const uint8_t *buf, uint32_t len)
{
    uint16_t crc = 0;
    for (uint32_t i = 0; i < len; ++i)
    {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t j = 0; j < 8; ++j)
        {
            if (crc & 0x8000)
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            else
                crc <<= 1;
        }
    }
    return crc;
}

// 统一打包并发送响应帧
// is_query: 1代表这是问询应答(上报Topic2)，0代表控制应答(通知Topic1)
// payload_len：实际业务数据长度，不包含4G前缀和协议头
static void app_4G_send_ack(uint8_t ack_id, uint8_t *payload, uint8_t payload_len, uint8_t is_query)
{
    static uint8_t tx_buf[300];
    uint16_t tx_idx = 0;

    // 1. 组装4G分发前缀
    if (is_query)
    {
        tx_buf[tx_idx++] = '2'; // 问询帧上报Topic2：/ai_safety/ais001/info/upload/hook/device_manager
    }
    else
    {
        tx_buf[tx_idx++] = '1'; // 数据帧通知Topic1：/ai_safety/ais001/control/inform/hook/device_manager
    }
    tx_buf[tx_idx++] = ',';

    uint16_t frame_start_idx = tx_idx;
    tx_buf[tx_idx++] = FRAME_HEADER_MAGIC; // 帧头
    tx_buf[tx_idx++] = ack_id;             // 内容ID
    tx_buf[tx_idx++] = g_tx_seq++;         // 包序号循环递增
    tx_buf[tx_idx++] = payload_len;        // 数据长度

    if (payload_len > 0 && payload != NULL)
    {
        memcpy(&tx_buf[tx_idx], payload, payload_len);
        tx_idx += payload_len;
    }

    // CRC
    uint16_t crc_calc = crc16_ccitt(&tx_buf[frame_start_idx], tx_idx - frame_start_idx);
    tx_buf[tx_idx++] = (uint8_t)(crc_calc >> 8);
    tx_buf[tx_idx++] = (uint8_t)(crc_calc & 0xFF);

    uart_manage_dma_send_by_name("4g", tx_buf, tx_idx);
}

// ==========================================
//  业务分发函数
// ==========================================

// 内容ID: 0x01 灯光警报业务
static void handle_light_cmd(ProtocolFrame_t *frame)
{
    uint8_t is_query;
    if (frame->data_len == 0)
    {
        is_query = 1; // 长度为0，问询帧
    }
    else
    {
        is_query = 0; // 长度不为0，控制帧
    }

    // 如果是控制指令，解析并下发
    if (!is_query && frame->data_len >= 1)
    {
        uint8_t cmd_byte = frame->data[0];
        // Bit 0: 爆闪灯 (1开0关)
        MB_Reg_Set(CMD_LED_SWITCH, (cmd_byte & 0x01) ? 1 : 0);
        // Bit 1: 7m警报响 (1开0关)
        MB_Reg_Set(CMD_BUZZER_7M, (cmd_byte & 0x02) ? 1 : 0);
        // Bit 2: 3m警报响 (1开0关)
        MB_Reg_Set(CMD_BUZZER_3M, (cmd_byte & 0x04) ? 1 : 0);
        // Bit 3-7: 音量
        MB_Reg_Set(CMD_VOLUME, (cmd_byte >> 3) & 0x1F);
    }

    uint8_t status_byte = 0;
    if (MB_Reg_Get(STATUS_LED_SWITCH))
    {
        status_byte |= (1 << 0);
    }
    if (MB_Reg_Get(STATUS_BUZZER))
    {
        status_byte |= (1 << 1);
    }

    uint8_t vol = MB_Reg_Get(CMD_VOLUME) & 0x1F;
    status_byte |= (vol << 3);

    app_4G_send_ack(ACK_ID_LIGHT_CONTROL, &status_byte, 1, is_query);
}

// 内容ID: 0x02 电池上报问询（BMS仅有一问一答的问询指令）
static void handle_bms_cmd(ProtocolFrame_t *frame)
{
    uint8_t is_query = 1; // bms仅有问询帧
    uint8_t payload[10] = {0};

    uint16_t battery = MB_Reg_Get(STATUS_BMS_BATTERY);
    uint16_t voltage = MB_Reg_Get(STATUS_BMS_TOTAL_VOLTAGE);
    uint16_t current = MB_Reg_Get(STATUS_BMS_TOTAL_CURRENT);
    uint16_t dis_time = MB_Reg_Get(STATUS_BMS_REMAIN_DISCHARGE_TIME);
    uint16_t chg_time = MB_Reg_Get(STATUS_BMS_REMAIN_CHARGE_TIME);

    // 采用大端序组装
    payload[0] = (battery >> 8) & 0xFF;
    payload[1] = battery & 0xFF;

    payload[2] = (voltage >> 8) & 0xFF;
    payload[3] = voltage & 0xFF;

    payload[4] = (current >> 8) & 0xFF;
    payload[5] = current & 0xFF;

    payload[6] = (dis_time >> 8) & 0xFF;
    payload[7] = dis_time & 0xFF;

    payload[8] = (chg_time >> 8) & 0xFF;
    payload[9] = chg_time & 0xFF;

    app_4G_send_ack(ACK_ID_BMS_QUERY, payload, 10, is_query); // 发送应答, 10字节长度
}

// 内容ID: 0x03 系统相关：待机位/工作模式/错误码
static void handle_system_cmd(ProtocolFrame_t *frame)
{
    uint8_t is_query;
    if (frame->data_len == 0)
    {
        is_query = 1; // 长度为0，问询帧
    }
    else
    {
        is_query = 0; // 长度不为0，控制帧
    }

    if (!is_query && frame->data_len >= 3)
    {
        MB_Reg_Set(SOFT_STANDBY_ENABLE, frame->data[0]);
        MB_Reg_Set(STATUS_WORK_MODE, frame->data[1]);
    }

    uint8_t payload[3];
    payload[0] = MB_Reg_Get(SOFT_STANDBY_ENABLE) & 0xFF;
    payload[1] = MB_Reg_Get(STATUS_WORK_MODE) & 0xFF;
    payload[2] = MB_Reg_Get(REG_ERROR_CODE) & 0xFF; // 取低8位做简易错误码

    app_4G_send_ack(ACK_ID_SYSTEM_STATUS, payload, 3, is_query);
}

// 内容ID: 0x04 时间相关：关机/开机/stm32rtc
static void handle_time_cmd(ProtocolFrame_t *frame)
{
    uint8_t is_query;
    if (frame->data_len == 0)
    {
        is_query = 1; // 长度为0，问询帧
    }
    else
    {
        is_query = 0; // 长度不为0，控制帧
    }

    if (!is_query && frame->data_len >= 6)
    {
        // 大端序
        uint16_t off_time = (frame->data[0] << 8) | frame->data[1];
        uint16_t on_time = (frame->data[2] << 8) | frame->data[3];

        MB_Reg_Set(STATUS_POWER_OFF_TIME, off_time);
        MB_Reg_Set(STATUS_POWER_ON_TIME, on_time);
    }

    uint8_t payload[6];
    uint16_t off_time = MB_Reg_Get(STATUS_POWER_OFF_TIME);
    uint16_t on_time = MB_Reg_Get(STATUS_POWER_ON_TIME);
    uint16_t rtc_time = MB_Reg_Get(RTC_TIME);

    // 上报
    payload[0] = (off_time >> 8) & 0xFF;
    payload[1] = off_time & 0xFF;

    payload[2] = (on_time >> 8) & 0xFF;
    payload[3] = on_time & 0xFF;

    payload[4] = (rtc_time >> 8) & 0xFF;
    payload[5] = rtc_time & 0xFF;

    app_4G_send_ack(ACK_ID_TIME_SCHEDULE, payload, 6, is_query);
}

// 内容ID: 0x05 心跳包
static void handle_heartbeat_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len >= 1)
    {
        MB_Reg_Set(STATUS_HEART_BEAT, frame->data[0]);
    }

    uint8_t payload[1];
    payload[0] = MB_Reg_Get(STATUS_HEART_BEAT) & 0xFF;

    app_4G_send_ack(ACK_ID_HEARTBEAT, payload, 1, 0); // 心跳无问询，属于应答
}

// 帧解析完成后的分发函数
static void Parser_FrameComplete(ParserCtx_t *ctx)
{
    static ProtocolFrame_t frame;

    frame.header = ctx->header_buf[0];
    frame.content_id = ctx->header_buf[1];
    frame.seq = ctx->header_buf[2];
    frame.data_len = ctx->header_buf[3];
    if (frame.data_len > 0)
    {
        memcpy(frame.data, ctx->data_buf, frame.data_len);
    }

    // 根据 content_id 查找并执行对应函数
    for (int i = 0; handler_map[i].handler != NULL; i++)
    {
        if (handler_map[i].content_id == frame.content_id)
        {
            handler_map[i].handler(&frame);
            return;
        }
    }
    LOGE("Unknown Content ID: 0x%02X\n", frame.content_id);
}

// 逐字节解析状态机入口
void parser_process_byte(ParserCtx_t *ctx, uint8_t byte)
{
    switch (ctx->state)
    {
    case STATE_WAIT_START:
        if (byte == FRAME_HEADER_MAGIC)
        {
            ctx->header_buf[0] = byte;
            ctx->header_cnt = 1;
            ctx->state = STATE_READ_HEADER;
        }
        break;

    case STATE_READ_HEADER:
        ctx->header_buf[ctx->header_cnt++] = byte;
        // 解析帧头: Head(1) + ID(1) + Seq(1) + Len(1)
        if (ctx->header_cnt == 4)
        {
            ctx->data_len = ctx->header_buf[3];
            if (ctx->data_len == 0)
            {
                // 无数据段(问询帧)，直接读取CRC
                ctx->crc_cnt = 0;
                ctx->state = STATE_READ_CRC;
            }
            else
            {
                ctx->data_cnt = 0;
                ctx->state = STATE_READ_DATA;
            }
        }
        break;

    case STATE_READ_DATA:
        ctx->data_buf[ctx->data_cnt++] = byte;
        if (ctx->data_cnt == ctx->data_len)
        {
            // 数据段读取完成，开始读CRC
            ctx->crc_cnt = 0;
            ctx->state = STATE_READ_CRC;
        }
        break;

    case STATE_READ_CRC:
        ctx->crc_buf[ctx->crc_cnt++] = byte;
        if (ctx->crc_cnt == 2)
        {
            // 读取完整包，执行CRC计算比对
            // 计算范围: 帧头字节 + 内容ID + 包序号 + 数据长度 + 数据
            uint16_t rcv_crc = (ctx->crc_buf[0] << 8) | ctx->crc_buf[1];

            // 拼接出待算CRC的完整Buffer
            static uint8_t calc_buf[260];
            memcpy(calc_buf, ctx->header_buf, 4);
            if (ctx->data_len > 0)
            {
                memcpy(&calc_buf[4], ctx->data_buf, ctx->data_len);
            }

            uint16_t calc_crc = crc16_ccitt(calc_buf, 4 + ctx->data_len);

            if (calc_crc == rcv_crc)
            {
                // //打印查看接收到的完整帧内容
                uint16_t total_len = 4 + ctx->data_len;
                printf("[INFO] 4G Recv OK [%d Bytes]: ", total_len);
                for (uint16_t i = 0; i < total_len; i++)
                {
                    printf("%02X ", calc_buf[i]);
                }
                printf("| CRC:%04X\n", rcv_crc);

                Parser_FrameComplete(ctx);
            }
            else
            {
                LOGE("4G Frame CRC Error: calc=0x%04X, rcv=0x%04X\n", calc_crc, rcv_crc);
            }

            // 重置状态机，等待下一帧
            ctx->state = STATE_WAIT_START;
        }
        break;
    }
}

// 模块初始化
void app_4G_init(void)
{
    memset(&g_parser_ctx, 0, sizeof(ParserCtx_t));
    g_parser_ctx.state = STATE_WAIT_START;
}