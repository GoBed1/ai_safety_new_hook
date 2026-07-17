#ifndef APP_RFID_H
#define APP_RFID_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"

#define RFID_MAX_TAGS          8U
#define RFID_OFFLINE_MS        10000U
#define RFID_RX_BUFFER_SIZE    256U
#define RFID_RX_QUEUE_DEPTH    4U

/* Internal Modbus register map, kept compatible with ai_satefy_hook_mcu. */
#define REG_RFID_VALID         3U
#define REG_RFID_BASE          4U
#define RFID_REGS_PER_TAG      3U

typedef struct
{
    uint32_t uid;
    uint8_t rssi;
    uint8_t rfid_battery;
    TickType_t last_seen_tick;
} RFIDTag;

typedef struct
{
    UART_HandleTypeDef *huart;
    TaskHandle_t notify_task;

    /* UART DMA receive area and the ISR-to-task frame queue. */
    uint8_t rx_buf[RFID_RX_BUFFER_SIZE];
    uint8_t frame_queue[RFID_RX_QUEUE_DEPTH][RFID_RX_BUFFER_SIZE];
    uint16_t frame_len[RFID_RX_QUEUE_DEPTH];
    volatile uint8_t queue_head;
    volatile uint8_t queue_tail;
    volatile uint8_t queue_count;
    volatile uint8_t rx_restart_needed;
    volatile uint32_t dropped_frames;

    RFIDTag tags[RFID_MAX_TAGS];
    uint32_t valid_bitmap;
} RFIDClient;

extern RFIDClient RFID_client;

HAL_StatusTypeDef RFID_Init(RFIDClient *client,
                            UART_HandleTypeDef *huart,
                            TaskHandle_t notify_task);
bool RFID_OwnsUart(const UART_HandleTypeDef *huart);
void RFID_RxEventCallbackFromISR(UART_HandleTypeDef *huart,
                                 uint16_t size,
                                 BaseType_t *higher_priority_task_woken);
void RFID_UartErrorCallback(UART_HandleTypeDef *huart);
void RFID_Service(RFIDClient *client);
bool RFID_TakeFrame(RFIDClient *client,
                    uint8_t *frame,
                    uint16_t frame_capacity,
                    uint16_t *frame_length);

void RFID_OnFrame(RFIDClient *client, const uint8_t *frame, uint16_t length);
void RFID_CheckOffline(RFIDClient *client);
void RFID_WriteToModbusRegs(RFIDClient *client);
void RFID_PrintValidTags(const RFIDClient *client);

#endif /* APP_RFID_H */
