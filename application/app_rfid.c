#define MODULE_LOG_ENABLE 1

#include "app_rfid.h"

#include <string.h>

#include "modbus_rtu_server_interface.h"
#include "system_def.h"

#if defined(__GNUC__)
#define RFID_DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define RFID_DMA_BUFFER
#endif

RFIDClient RFID_client RFID_DMA_BUFFER;

static uint32_t rfid_read_be_u32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           (uint32_t)data[3];
}

static bool rfid_parse_frame(const uint8_t *frame,
                             uint16_t length,
                             uint8_t *rssi,
                             uint8_t *battery,
                             uint32_t *uid)
{
    if ((frame == NULL) || (rssi == NULL) || (battery == NULL) || (uid == NULL))
    {
        return false;
    }

    if ((length < 14U) ||
        (frame[0] != 0x1BU) ||
        (frame[1] != 0x39U) ||
        (frame[2] != 0x01U))
    {
        return false;
    }

    *rssi = frame[7];
    *battery = frame[9];
    *uid = rfid_read_be_u32(&frame[10]);
    return true;
}

static int32_t rfid_find_uid(const RFIDClient *client, uint32_t uid)
{
    for (uint32_t i = 0U; i < RFID_MAX_TAGS; ++i)
    {
        if (((client->valid_bitmap & (1UL << i)) != 0U) &&
            (client->tags[i].uid == uid))
        {
            return (int32_t)i;
        }
    }

    return -1;
}

static int32_t rfid_alloc_slot(const RFIDClient *client)
{
    for (uint32_t i = 0U; i < RFID_MAX_TAGS; ++i)
    {
        if ((client->valid_bitmap & (1UL << i)) == 0U)
        {
            return (int32_t)i;
        }
    }

    return -1;
}

static HAL_StatusTypeDef rfid_start_receive(RFIDClient *client)
{
    HAL_StatusTypeDef status;

    if ((client == NULL) || (client->huart == NULL) ||
        (client->huart->hdmarx == NULL))
    {
        return HAL_ERROR;
    }

    status = HAL_UARTEx_ReceiveToIdle_DMA(client->huart,
                                          client->rx_buf,
                                          (uint16_t)sizeof(client->rx_buf));
    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(client->huart->hdmarx, DMA_IT_HT);
        client->rx_restart_needed = 0U;
    }
    else
    {
        client->rx_restart_needed = 1U;
    }

    return status;
}

static bool rfid_is_valid_frame(const uint8_t *frame, uint16_t length)
{
    return (length >= 14U) &&
           (frame[0] == 0x1BU) &&
           (frame[1] == 0x39U) &&
           (frame[2] == 0x01U);
}

static void rfid_enqueue_from_isr(RFIDClient *client, uint16_t length)
{
    uint8_t index;

    if (!rfid_is_valid_frame(client->rx_buf, length))
    {
        return;
    }

    if (client->queue_count >= RFID_RX_QUEUE_DEPTH)
    {
        client->queue_tail = (uint8_t)((client->queue_tail + 1U) % RFID_RX_QUEUE_DEPTH);
        --client->queue_count;
        ++client->dropped_frames;
    }

    index = client->queue_head;
    memcpy(client->frame_queue[index], client->rx_buf, length);
    client->frame_len[index] = length;
    __DMB();

    client->queue_head = (uint8_t)((index + 1U) % RFID_RX_QUEUE_DEPTH);
    ++client->queue_count;
}

HAL_StatusTypeDef RFID_Init(RFIDClient *client,
                            UART_HandleTypeDef *huart,
                            TaskHandle_t notify_task)
{
    if ((client == NULL) || (huart == NULL) || (notify_task == NULL))
    {
        return HAL_ERROR;
    }

    memset(client, 0, sizeof(*client));
    client->huart = huart;
    client->notify_task = notify_task;
    if (client->huart->hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    /* The task starts DMA after the scheduler is running. */
    client->rx_restart_needed = 1U;
    RFID_WriteToModbusRegs(client);
    return HAL_OK;
}

bool RFID_OwnsUart(const UART_HandleTypeDef *huart)
{
    return (huart != NULL) &&
           (RFID_client.huart != NULL) &&
           (huart->Instance == RFID_client.huart->Instance);
}

void RFID_RxEventCallbackFromISR(UART_HandleTypeDef *huart,
                                 uint16_t size,
                                 BaseType_t *higher_priority_task_woken)
{
    uint16_t length = size;
    bool notify_task = false;

    if (!RFID_OwnsUart(huart))
    {
        return;
    }

    if (length > RFID_RX_BUFFER_SIZE)
    {
        length = RFID_RX_BUFFER_SIZE;
    }

    if (rfid_is_valid_frame(RFID_client.rx_buf, length))
    {
        rfid_enqueue_from_isr(&RFID_client, length);
        notify_task = true;
    }

    if (rfid_start_receive(&RFID_client) != HAL_OK)
    {
        notify_task = true;
    }

    if (notify_task && (RFID_client.notify_task != NULL))
    {
        vTaskNotifyGiveFromISR(RFID_client.notify_task, higher_priority_task_woken);
    }
}

void RFID_UartErrorCallback(UART_HandleTypeDef *huart)
{
    if (!RFID_OwnsUart(huart))
    {
        return;
    }

    (void)HAL_UART_DMAStop(huart);
    (void)rfid_start_receive(&RFID_client);
}

void RFID_Service(RFIDClient *client)
{
    if ((client != NULL) && (client->rx_restart_needed != 0U))
    {
        (void)HAL_UART_DMAStop(client->huart);
        (void)rfid_start_receive(client);
    }
}

bool RFID_TakeFrame(RFIDClient *client,
                    uint8_t *frame,
                    uint16_t frame_capacity,
                    uint16_t *frame_length)
{
    uint8_t index;
    uint16_t length;

    if ((client == NULL) || (frame == NULL) || (frame_length == NULL))
    {
        return false;
    }

    taskENTER_CRITICAL();

    if (client->queue_count == 0U)
    {
        taskEXIT_CRITICAL();
        return false;
    }

    index = client->queue_tail;
    length = client->frame_len[index];
    if (length > frame_capacity)
    {
        length = frame_capacity;
    }

    memcpy(frame, client->frame_queue[index], length);
    client->queue_tail = (uint8_t)((index + 1U) % RFID_RX_QUEUE_DEPTH);
    --client->queue_count;

    taskEXIT_CRITICAL();

    *frame_length = length;
    return true;
}

void RFID_WriteToModbusRegs(RFIDClient *client)
{
    if (client == NULL)
    {
        return;
    }

    MB_Reg_SetBits(REG_RFID_VALID, 0x00FFU, (uint16_t)client->valid_bitmap);

    for (uint32_t i = 0U; i < RFID_MAX_TAGS; ++i)
    {
        uint16_t base = (uint16_t)(REG_RFID_BASE + (i * RFID_REGS_PER_TAG));

        if ((client->valid_bitmap & (1UL << i)) != 0U)
        {
            MB_Reg_Set(base, (uint16_t)(client->tags[i].uid >> 16));
            MB_Reg_Set((uint16_t)(base + 1U), (uint16_t)client->tags[i].uid);
            MB_Reg_Set((uint16_t)(base + 2U),
                       ((uint16_t)client->tags[i].rssi << 8) |
                       client->tags[i].rfid_battery);
        }
        else
        {
            MB_Reg_Set(base, 0U);
            MB_Reg_Set((uint16_t)(base + 1U), 0U);
            MB_Reg_Set((uint16_t)(base + 2U), 0U);
        }
    }
}

void RFID_PrintValidTags(const RFIDClient *client)
{
    char valid_groups[64];
    size_t used = 0U;
    uint16_t valid_register;
    uint8_t valid_count = 0U;

    if (client == NULL)
    {
        return;
    }

    valid_groups[0] = '\0';
    valid_register = (uint16_t)(MB_Reg_Get(REG_RFID_VALID) & 0x00FFU);

    for (uint32_t i = 0U; i < RFID_MAX_TAGS; ++i)
    {
        if ((valid_register & (1U << i)) != 0U)
        {
            int written;

            ++valid_count;
            written = snprintf(&valid_groups[used],
                               sizeof(valid_groups) - used,
                               "%s%lu",
                               (used == 0U) ? "" : ", ",
                               (unsigned long)(i + 1U));
            if (written > 0)
            {
                size_t appended = (size_t)written;
                size_t remaining = sizeof(valid_groups) - used;
                used += (appended < remaining) ? appended : (remaining - 1U);
            }
        }
    }

    LOGI("RFID reg[3]=0x%04X, valid count=%u, valid groups=%s\r\n",
         (unsigned int)valid_register,
         (unsigned int)valid_count,
         (valid_count > 0U) ? valid_groups : "none");

    for (uint32_t i = 0U; i < RFID_MAX_TAGS; ++i)
    {
        if ((valid_register & (1U << i)) != 0U)
        {
            LOGI("RFID group %lu: UID=0x%08lX, RSSI=%u, battery=%u\r\n",
                 (unsigned long)(i + 1U),
                 (unsigned long)client->tags[i].uid,
                 (unsigned int)client->tags[i].rssi,
                 (unsigned int)client->tags[i].rfid_battery);
        }
    }
}

void RFID_CheckOffline(RFIDClient *client)
{
    TickType_t now;
    TickType_t timeout;

    if (client == NULL)
    {
        return;
    }

    now = xTaskGetTickCount();
    timeout = pdMS_TO_TICKS(RFID_OFFLINE_MS);

    for (uint32_t i = 0U; i < RFID_MAX_TAGS; ++i)
    {
        if ((client->valid_bitmap & (1UL << i)) == 0U)
        {
            continue;
        }

        if ((TickType_t)(now - client->tags[i].last_seen_tick) > timeout)
        {
            LOGI("RFID offline: idx=%lu, UID=0x%08lX\r\n",
                 (unsigned long)i,
                 (unsigned long)client->tags[i].uid);
            client->valid_bitmap &= ~(1UL << i);
            memset(&client->tags[i], 0, sizeof(client->tags[i]));
        }
    }
}

void RFID_OnFrame(RFIDClient *client, const uint8_t *frame, uint16_t length)
{
    uint8_t rssi;
    uint8_t battery;
    uint32_t uid;
    int32_t index;

    if ((client == NULL) ||
        !rfid_parse_frame(frame, length, &rssi, &battery, &uid))
    {
        LOGE("RFID parse failed\r\n");
        return;
    }

    index = rfid_find_uid(client, uid);
    if (index < 0)
    {
        index = rfid_alloc_slot(client);
        if (index < 0)
        {
            LOGE("RFID slots full, UID=0x%08lX\r\n", (unsigned long)uid);
            return;
        }

        client->valid_bitmap |= (1UL << (uint32_t)index);
        client->tags[index].uid = uid;
        LOGI("RFID new tag: idx=%ld, UID=0x%08lX\r\n",
             (long)index,
             (unsigned long)uid);
    }

    client->tags[index].rssi = rssi;
    client->tags[index].rfid_battery = battery;
    client->tags[index].last_seen_tick = xTaskGetTickCount();

    LOGD("RFID update: idx=%ld, UID=0x%08lX, RSSI=%u, battery=%u\r\n",
         (long)index,
         (unsigned long)uid,
         rssi,
         battery);
}
