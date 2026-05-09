#ifndef APP_RFID_H
#define APP_RFID_H

#include <stdint.h>
#include "system_def.h"
#include "FreeRTOS.h"
#include "task.h"
// #include "portmacro.h"
//RFID相关参数定义
#define RFID_MAX_TAGS      8
#define RFID_OFFLINE_MS    10000U
#define REG_RFID_VALID      3       // 离线位图寄存器索引
#define REG_RFID_BASE       4       // 第1组起始寄存器
typedef struct {
    uint32_t uid;              // 11~14字节拼成的UID
    uint8_t  rssi;             // 第9字节
    uint8_t  rfid_battery;              // 第10字节
    TickType_t last_seen_tick; // 最近一次收到该UID的时间戳
} RFIDTag;
typedef struct RFIDClient
{
    // rx相关参数
    UART_HandleTypeDef *huart;
    uint8_t rx_buf[256];
    uint8_t Rx_RFID_buf[256];
    uint8_t Rx_RFID_len;

    // 解析数据相关参数
    RFIDTag tags[RFID_MAX_TAGS];
    uint32_t valid_bitmap;      // 离线码：bit=1 表示该index已被占用/参与离线检测
  

} RFIDClient;
extern RFIDClient RFID_client;
void RFID_OnFrame(RFIDClient *c, const uint8_t *frm, uint16_t len);
void RFID_CheckOffline(RFIDClient *c);
void RFID_WriteToModbusRegs(RFIDClient *c);

#endif // APP_RFID_H