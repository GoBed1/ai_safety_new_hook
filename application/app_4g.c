#define MODULE_LOG_ENABLE LOG_SWITCH_4G
#include "app_4g.h"
#include "uart_manage.h"
#include "modbus_rtu_server_interface.h"

// 业务处理函数声明
static void handle_light_cmd(ProtocolFrame_t *frame);
static void handle_bms_cmd(ProtocolFrame_t *frame);
static void handle_heartbeat_en_cmd(ProtocolFrame_t *frame);
static void handle_heartbeat_cmd(ProtocolFrame_t *frame);
static void handle_work_mode_cmd(ProtocolFrame_t *frame);
static void handle_sleep_en_cmd(ProtocolFrame_t *frame);
static void handle_sleep_time_cmd(ProtocolFrame_t *frame);
static void handle_time_cmd(ProtocolFrame_t *frame);
static void handle_sys_error_cmd(ProtocolFrame_t *frame);
// 全局解析上下文
ParserCtx_t g_parser_ctx;
// 发送序列号缓存
uint8_t g_tx_seq = 0;

// 内容ID映射表
const IdHandlerMap_t handler_map[] = {
    {CMD_ID_LIGHT, handle_light_cmd},               // 灯光控制命令
    {CMD_ID_BMS, handle_bms_cmd},                   // 电池管理系统命令
    {CMD_ID_HEARTBEAT_EN, handle_heartbeat_en_cmd}, // 是否使能心跳命令
    {CMD_ID_HEARTBEAT, handle_heartbeat_cmd},       // 工作心跳命令
    {CMD_ID_WORK_MODE, handle_work_mode_cmd},       // 工作模式命令
    {CMD_ID_SLEEP_EN, handle_sleep_en_cmd},         // 是否使能休眠命令
    {CMD_ID_SLEEP_TIME, handle_sleep_time_cmd},     // 休眠时间命令
    {CMD_ID_CURRENT_TIME, handle_time_cmd},         // 同步当前时间命令
    {CMD_ID_SYS_ERROR, handle_sys_error_cmd},       // 系统错误上报命令
    {0x00, NULL}                                    // 结束标记
};

void app_4g_update(void)
{
    // 获取 4G 接口对象
    uart_inferface_t *m_obj = uart_manage_get_obj_by_name("4g");
    if (m_obj == NULL) return;

    // 查看 RingBuffer 里有多少数据可以读
    lwrb_sz_t available = lwrb_get_full(&m_obj->process_ring_buffer);
    if (available == 0) return;

    uint8_t to_read_buffer[128];
    lwrb_sz_t to_read = (available > sizeof(to_read_buffer)) ? sizeof(to_read_buffer) : available;
    lwrb_sz_t read_size = lwrb_read(&m_obj->process_ring_buffer, to_read_buffer, to_read);

    if (read_size > 0)
    {

        for (lwrb_sz_t i = 0; i < read_size; i++)
        {
            parser_process_byte(&g_parser_ctx, to_read_buffer[i]);
        }
    }
}


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
static void app_4G_send_ack(uint8_t ack_id, uint8_t *payload, uint8_t payload_len)
{
    uint8_t tx_buf[300];
    uint16_t tx_idx = 0;

    // 前缀统一为通知/应答主题
    tx_buf[tx_idx++] = '3';
    tx_buf[tx_idx++] = ',';

    uint16_t frame_start_idx = tx_idx;
    tx_buf[tx_idx++] = FRAME_HEADER_MAGIC; // SOF
    tx_buf[tx_idx++] = g_tx_seq++;         // SEQ
    tx_buf[tx_idx++] = ack_id;             // CMD_ID
    tx_buf[tx_idx++] = payload_len;        // DATA-LEN

    if (payload_len > 0 && payload != NULL)
    {
        memcpy(&tx_buf[tx_idx], payload, payload_len);
        tx_idx += payload_len;
    }

    // CRC 小端序：低字节在前，高字节在后
    uint16_t crc_calc = crc16_ccitt(&tx_buf[frame_start_idx], tx_idx - frame_start_idx);
    tx_buf[tx_idx++] = (uint8_t)(crc_calc & 0xFF);
    tx_buf[tx_idx++] = (uint8_t)((crc_calc >> 8) & 0xFF);

    uart_manage_dma_send_by_name("4g", tx_buf, tx_idx);
}

// ==========================================
//  业务分发函数
// ==========================================

// 内容ID: 0x01 灯光警报业务
static void handle_light_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0)
    { // 读指令
        uint8_t status_byte = 0;
        if (MB_Reg_Get(STATUS_LED_SWITCH))
        {
            status_byte |= (1 << 0);
        }
        if (MB_Reg_Get(STATUS_BUZZER) == 1)
        {
            status_byte |= (1 << 1);
        }
        else if (MB_Reg_Get(STATUS_BUZZER) == 2)
        {
            status_byte |= (1 << 2);
        }
        uint8_t vol = MB_Reg_Get(CMD_VOLUME) & 0x1F;
        status_byte |= (vol << 3);

        app_4G_send_ack(CMD_ID_LIGHT, &status_byte, 1);
    }
    else
    { // 写指令
        uint8_t cmd_byte = frame->data[0];
        MB_Reg_Set(CMD_LED_SWITCH, (cmd_byte & 0x01) ? 1 : 0);
        MB_Reg_Set(CMD_BUZZER_7M, (cmd_byte & 0x02) ? 1 : 0);
        MB_Reg_Set(CMD_BUZZER_3M, (cmd_byte & 0x04) ? 1 : 0);
        MB_Reg_Set(CMD_VOLUME, (cmd_byte >> 3) & 0x1F);

        app_4G_send_ack(CMD_ID_LIGHT, NULL, 0); // 写操作返回空数据段
    }
}
// 0x02 电池读取
static void handle_bms_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0)
    { // 读指令
        uint8_t payload[12] = {0};
        uint16_t battery = MB_Reg_Get(STATUS_BMS_BATTERY);
        uint16_t voltage = MB_Reg_Get(STATUS_BMS_TOTAL_VOLTAGE);
        uint16_t current = MB_Reg_Get(STATUS_BMS_TOTAL_CURRENT);
        uint16_t dis_time = MB_Reg_Get(STATUS_BMS_REMAIN_DISCHARGE_TIME);
        uint16_t chg_time = MB_Reg_Get(STATUS_BMS_REMAIN_CHARGE_TIME);
        uint16_t protect = MB_Reg_Get(STATUS_BMS_PROTECT_STATUS);

        // 小端序组装
        payload[0] = battery & 0xFF;
        payload[1] = (battery >> 8) & 0xFF;
        payload[2] = voltage & 0xFF;
        payload[3] = (voltage >> 8) & 0xFF;
        payload[4] = current & 0xFF;
        payload[5] = (current >> 8) & 0xFF;
        payload[6] = dis_time & 0xFF;
        payload[7] = (dis_time >> 8) & 0xFF;
        payload[8] = chg_time & 0xFF;
        payload[9] = (chg_time >> 8) & 0xFF;
        payload[10] = protect & 0xFF;
        payload[11] = (protect >> 8) & 0xFF;
        app_4G_send_ack(CMD_ID_BMS, payload, 12);
    }
    else
    { // 应对异常写指令兜底
        uint8_t err = ERR_CODE_R_W;
        app_4G_send_ack(CMD_ID_BMS | 0x80, &err, 1);
    }
}

// 0x04 工作心跳使能
static void handle_heartbeat_en_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0) // 读指令
    {
        uint8_t heartbeat_en = MB_Reg_Get(HEARTBEAT_ENABLE) & 0xFF;
        app_4G_send_ack(CMD_ID_HEARTBEAT_EN, &heartbeat_en, 1);
    }
    else
    { // 写指令
        MB_Reg_Set(HEARTBEAT_ENABLE, frame->data[0]);
        app_4G_send_ack(CMD_ID_HEARTBEAT_EN, NULL, 0);
    }
}
// 0x05 工作心跳
static void handle_heartbeat_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0)
    { // 读指令
        uint8_t hb = MB_Reg_Get(STATUS_HEART_BEAT) & 0xFF;
        app_4G_send_ack(CMD_ID_HEARTBEAT, &hb, 1);
    }
    else
    { // 写指令，不做应答
        MB_Reg_Set(STATUS_HEART_BEAT, frame->data[0]);
    }
}
// 0x06 工作模式
static void handle_work_mode_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0) // 读指令
    {
        uint8_t mode = MB_Reg_Get(STATUS_WORK_MODE) & 0xFF;
        app_4G_send_ack(CMD_ID_WORK_MODE, &mode, 1);
    }
    else // 写指令，工作模式为只读，返回错误码
    {
        uint8_t err = ERR_CODE_LEN_ERROR;
        app_4G_send_ack(CMD_ID_WORK_MODE | 0x80, &err, 1);
    }
}

// 0x07 是否使能睡眠
static void handle_sleep_en_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0)
    {
        uint8_t st = MB_Reg_Get(SOFT_STANDBY_ENABLE) & 0xFF;
        app_4G_send_ack(CMD_ID_SLEEP_EN, &st, 1);
    }
    else
    {
        MB_Reg_Set(SOFT_STANDBY_ENABLE, frame->data[0]);
        app_4G_send_ack(CMD_ID_SLEEP_EN, NULL, 0);
    }
}

// 0x08 睡眠时间
static void handle_sleep_time_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0)
    {
        uint8_t payload[4];
        uint16_t on_time = MB_Reg_Get(STATUS_POWER_ON_TIME);
        uint16_t off_time = MB_Reg_Get(STATUS_POWER_OFF_TIME);

        // 小端序
        payload[0] = on_time & 0xFF;
        payload[1] = (on_time >> 8) & 0xFF;
        payload[2] = off_time & 0xFF;
        payload[3] = (off_time >> 8) & 0xFF;

        app_4G_send_ack(CMD_ID_SLEEP_TIME, payload, 4);
    }
    else if (frame->data_len >= 4)
    {
        // 小端序解析写入
        uint16_t on_time = frame->data[0] | (frame->data[1] << 8);
        uint16_t off_time = frame->data[2] | (frame->data[3] << 8);
        MB_Reg_Set(STATUS_POWER_ON_TIME, on_time);
        MB_Reg_Set(STATUS_POWER_OFF_TIME, off_time);

        app_4G_send_ack(CMD_ID_SLEEP_TIME, NULL, 0);
    }
}

// 0x09 当前时间
static void handle_time_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0)
    {
        uint8_t payload[2];
        uint16_t rtc_time = MB_Reg_Get(RTC_TIME);

        payload[0] = rtc_time & 0xFF;
        payload[1] = (rtc_time >> 8) & 0xFF;

        app_4G_send_ack(CMD_ID_CURRENT_TIME, payload, 2);
    }
    else if (frame->data_len >= 2)
    {
        uint16_t rtc_time = frame->data[0] | (frame->data[1] << 8);
        MB_Reg_Set(RTC_TIME, rtc_time);
        app_4G_send_ack(CMD_ID_CURRENT_TIME, NULL, 0);
    }
}
// 0xF0 系统错误码上报
static void handle_sys_error_cmd(ProtocolFrame_t *frame)
{
    if (frame->data_len == 0)
    {
        uint8_t err = MB_Reg_Get(REG_ERROR_CODE) & 0xFF;
        app_4G_send_ack(CMD_ID_SYS_ERROR, &err, 1);
    }
}
// 帧解析完成后的分发函数
static void Parser_FrameComplete(ParserCtx_t *ctx)
{
    ProtocolFrame_t frame;

    frame.header = ctx->header_buf[0];
    frame.seq = ctx->header_buf[1];
    frame.content_id = ctx->header_buf[2];
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
    uint8_t err_code = ERR_CODE_UNKNOWN_ID;
    app_4G_send_ack(frame.content_id | 0x80, &err_code, 1);
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
        // 解析帧头: (Header + SEQ + ID + Len)
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
            // 读取完整包，小端序执行CRC计算比对
            // 计算范围: 帧头字节 + 内容ID + 包序号 + 数据长度 + 数据
            uint16_t rcv_crc = (ctx->crc_buf[0] | (ctx->crc_buf[1] << 8));

            // 拼接出待算CRC的完整Buffer
            uint8_t calc_buf[260];
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
                printf("4G Frame CRC Error: calc=0x%04X, rcv=0x%04X\n", calc_crc, rcv_crc);
                // CRC 校验失败
                uint8_t req_id = ctx->header_buf[2]; // 内容ID
                uint8_t err_code = ERR_CODE_CRC_FAIL;
                app_4G_send_ack(req_id | 0x80, &err_code, 1);
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