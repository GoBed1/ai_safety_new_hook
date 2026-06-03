#ifndef APP_4G_H
#define APP_4G_H

#include <stdint.h>
#include "system_def.h"

#define CMD_ID_LIGHT            0x01  // 声光报警控制
#define CMD_ID_BMS              0x02  // 电池信息
#define CMD_ID_HEARTBEAT_EN     0x04  // 是否使能工作心跳
#define CMD_ID_HEARTBEAT        0x05  // 工作心跳
#define CMD_ID_WORK_MODE        0x06  // 工作模式
#define CMD_ID_SLEEP_EN         0x07  // 是否使能睡眠模式
#define CMD_ID_SLEEP_TIME       0x08  // 睡眠时间
#define CMD_ID_CURRENT_TIME     0x09  // 当前时间
#define CMD_ID_SYS_ERROR        0xF0  // 系统错误码

#define ACK_ID_LIGHT_CONTROL      0x81  // 灯光警报业务应答/上报
#define ACK_ID_BMS_QUERY          0x82  // BMS 电池问询上报
#define ACK_ID_SYSTEM_STATUS      0x83  // 系统相关状态应答/上报
#define ACK_ID_TIME_SCHEDULE      0x84  // 时间相关计划应答/上报
#define ACK_ID_HEARTBEAT          0x85  // 继电器心跳应答
// 协议帧头
#define FRAME_HEADER_MAGIC 0xA5

// 错误码定义 (用于错误帧的 Data 段)
#define ERR_CODE_CRC_FAIL       0x01  // CRC校验失败
#define ERR_CODE_LEN_ERROR      0x02  // 帧长度不够/异常
#define ERR_CODE_UNKNOWN_ID     0x03  // 未知的指令ID
#define ERR_CODE_R_W            0x04  // 不可读取或写入
// 状态机枚举
typedef enum {
    STATE_WAIT_START = 0,
    STATE_READ_HEADER,
    STATE_READ_DATA,
    STATE_READ_CRC,
    STATE_FRAME_COMPLETE
} ParserState_t;

// 完整协议帧结构 (不含4G前缀) 
#pragma pack(1)
typedef struct {
    uint8_t header;       // 帧头 0xFC
    uint8_t content_id;   // 内容ID
    uint8_t seq;          // 包序号
    uint8_t data_len;     // 数据长度
    uint8_t data[256];    // 业务数据 (最大256字节) 
} ProtocolFrame_t;
#pragma pack()

// 状态机上下文
typedef struct {
    ParserState_t state;
    uint8_t header_buf[4]; // 存放 header, id, seq, len
    uint8_t header_cnt;
    
    uint8_t data_buf[256];
    uint16_t data_len;
    uint16_t data_cnt;
    
    uint8_t crc_buf[2];
    uint8_t crc_cnt;
} ParserCtx_t;

// 分发函数指针定义
typedef void (*CommandHandler)(ProtocolFrame_t *frame);

// ID到处理函数的映射表结构
typedef struct {
    uint8_t content_id;
    CommandHandler handler;
} IdHandlerMap_t;

// 外部接口
void app_4G_init(void);
void parser_process_byte(ParserCtx_t *ctx, uint8_t byte);

#endif // APP_4G_H