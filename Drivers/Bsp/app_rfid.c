#include "app_rfid.h"
#include "modbus_rtu_server_interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "usart.h" // 包含 huart2 的声明

extern volatile uint8_t g_task_alive_flags;
extern EventGroupHandle_t eg;
RFIDClient RFID_client;

// ========== 内部函数：大端拼接 ==========
static uint32_t be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           ((uint32_t)p[3] << 0);
}

// ========== 内部函数：解析RFID帧 ==========
static int parse_frame(const uint8_t *frm, uint16_t len,
                       uint8_t *out_rssi, uint8_t *out_rfid_battery, uint32_t *out_uid)
{
    if (len < 14)
        return 0;
    if (frm[0] != 0x1B || frm[1] != 0x39 || frm[2] != 0x01)
        return 0;

    *out_rssi = frm[7];
    *out_rfid_battery = frm[9];
    *out_uid = be_u32(&frm[10]);

    return 1;
}

// ========== 内部函数：查找UID ==========
static int find_uid(RFIDClient *c, uint32_t uid)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) && c->tags[i].uid == uid)
        {
            return i;
        }
    }
    return -1;
}

// ========== 内部函数：分配空闲槽位 ==========
static int alloc_slot(RFIDClient *c)
{
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) == 0)
        {
            return i;
        }
    }
    return -1;
}

// ========== 1.写入Modbus寄存器 ==========
void RFID_WriteToModbusRegs(RFIDClient *c)
{
    // 更新位图到 modbus_registers[3] (REG_RFID_VALID) 的低8位
    MB_Reg_SetBits(REG_RFID_VALID, 0x00FF, c->valid_bitmap);

    // 写8组数据到 modbus_registers[4~27]
    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        uint16_t base = REG_RFID_BASE + i * 3;

        if (c->valid_bitmap & (1U << i))
        {
            MB_Reg_Set(base + 0, (uint16_t)(c->tags[i].uid >> 16));
            MB_Reg_Set(base + 1, (uint16_t)(c->tags[i].uid & 0xFFFF));
            MB_Reg_Set(base + 2, ((uint16_t)c->tags[i].rssi << 8) | c->tags[i].rfid_battery);
        }
        else
        {
            MB_Reg_Set(base + 0, 0);
            MB_Reg_Set(base + 1, 0);
            MB_Reg_Set(base + 2, 0);
        }
    }
}

// ========== 2.检查离线 ==========
void RFID_CheckOffline(RFIDClient *c)
{
    uint32_t now = xTaskGetTickCount();
    uint32_t timeout = pdMS_TO_TICKS(RFID_OFFLINE_MS);

    for (int i = 0; i < RFID_MAX_TAGS; i++)
    {
        if ((c->valid_bitmap & (1U << i)) == 0)
            continue;

        if ((uint32_t)(now - c->tags[i].last_seen_tick) > timeout)
        {
            c->valid_bitmap &= ~(1U << i); // 标记标签无效
            LOGI("RFID offline: idx=%d, UID=0x%08X\n",
                 i, (unsigned int)c->tags[i].uid);
            memset(&c->tags[i], 0, sizeof(c->tags[i])); // 清除标签结构体数据
        }
    }
}

// ========== 3.收到帧后更新 ==========
void RFID_OnFrame(RFIDClient *c, const uint8_t *frm, uint16_t len)
{
    uint8_t rssi, rfid_battery;
    uint32_t uid;

    if (!parse_frame(frm, len, &rssi, &rfid_battery, &uid))
    {
        LOGE("RFID parse fail\n");
        return;
    }

    uint32_t now = xTaskGetTickCount();

    int idx = find_uid(c, uid);
    if (idx < 0)
    {
        idx = alloc_slot(c);
        if (idx < 0)
        {
            LOGE("RFID slots full, UID=0x%08X\n", (unsigned int)uid);
            return;
        }
        c->valid_bitmap |= (1U << idx);
        c->tags[idx].uid = uid;
        LOGI("RFID new tag: idx=%d, UID=0x%08X\n", idx, (unsigned int)uid);
    }

    c->tags[idx].rssi = rssi;
    c->tags[idx].rfid_battery = rfid_battery;
    c->tags[idx].last_seen_tick = now;

    LOGD("RFID update: idx=%d, UID=0x%08X, RSSI=%d, rfid_battery=%d\n",
         idx, (unsigned int)uid, rssi, rfid_battery);
}

