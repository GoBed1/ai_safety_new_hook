#include "ota_manage_port.h"

#include <string.h>

#include "system_def.h"
#include "stm32h7xx_hal.h"

#if defined(__GNUC__)
#define OTA_ALIGNED_32 __attribute__((aligned(32)))
#else
#define OTA_ALIGNED_32
#endif

static uint32_t ota_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
  for (uint32_t i = 0U; i < len; ++i)
  {
    crc ^= data[i];
    for (uint32_t bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 1UL) != 0UL)
      {
        crc = (crc >> 1) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  return crc;
}

static uint32_t ota_crc32_finish(uint32_t crc)
{
  return ~crc;
}

static uint32_t ota_flash_meta_crc(const ota_flash_meta_t *meta)
{
  ota_flash_meta_t temp;

  if (meta == NULL)
  {
    return 0U;
  }

  temp = *meta;
  temp.meta_crc32 = 0U;
  return ota_crc32_finish(ota_crc32_update(0xFFFFFFFFUL,
                                           (const uint8_t *)&temp,
                                           (uint32_t)sizeof(temp)));
}

static uint8_t ota_slot_index(uint32_t slot)
{
  return (slot == OTA_FLASH_SLOT_B) ? 1U : 0U;
}

static uint32_t ota_slot_address(uint32_t slot)
{
  if (slot == OTA_FLASH_SLOT_A)
  {
    return OTA_FLASH_SLOT_A_ADDR;
  }

  if (slot == OTA_FLASH_SLOT_B)
  {
    return OTA_FLASH_SLOT_B_ADDR;
  }

  return 0U;
}

static uint32_t ota_other_slot(uint32_t slot)
{
  return (slot == OTA_FLASH_SLOT_A) ? OTA_FLASH_SLOT_B : OTA_FLASH_SLOT_A;
}

static uint32_t ota_detect_running_slot(void)
{
  uint32_t pc = ((uint32_t)(uintptr_t)&ota_boot_callback) & ~1UL;

  if ((pc >= OTA_FLASH_SLOT_A_ADDR) && (pc < (OTA_FLASH_SLOT_A_ADDR + OTA_FLASH_SLOT_SIZE)))
  {
    return OTA_FLASH_SLOT_A;
  }

  if ((pc >= OTA_FLASH_SLOT_B_ADDR) && (pc < (OTA_FLASH_SLOT_B_ADDR + OTA_FLASH_SLOT_SIZE)))
  {
    return OTA_FLASH_SLOT_B;
  }

  return OTA_FLASH_SLOT_A;
}

static void ota_flash_make_default_meta(ota_flash_meta_t *meta)
{
  uint32_t active_slot;

  if (meta == NULL)
  {
    return;
  }

  active_slot = ota_detect_running_slot();

  (void)memset(meta, 0, sizeof(*meta));
  meta->magic = OTA_FLASH_META_MAGIC;
  meta->struct_version = OTA_FLASH_META_STRUCT_VERSION;
  meta->ota_request = OTA_FLASH_OTA_REQUEST_NONE;
  meta->target_slot = OTA_FLASH_SLOT_NONE;
  meta->active_slot = active_slot;
  meta->boot_count = 0U;
  meta->image[0].address = OTA_FLASH_SLOT_A_ADDR;
  meta->image[0].state = (active_slot == OTA_FLASH_SLOT_A) ? OTA_FLASH_IMAGE_VALID : OTA_FLASH_IMAGE_EMPTY;
  meta->image[1].address = OTA_FLASH_SLOT_B_ADDR;
  meta->image[1].state = (active_slot == OTA_FLASH_SLOT_B) ? OTA_FLASH_IMAGE_VALID : OTA_FLASH_IMAGE_EMPTY;
  meta->meta_crc32 = ota_flash_meta_crc(meta);
}

static int ota_flash_meta_is_valid(const ota_flash_meta_t *meta)
{
  if (meta == NULL)
  {
    return 0;
  }

  if ((meta->magic != OTA_FLASH_META_MAGIC) ||
      (meta->struct_version != OTA_FLASH_META_STRUCT_VERSION) ||
      (meta->meta_crc32 != ota_flash_meta_crc(meta)))
  {
    return 0;
  }

  if ((meta->active_slot != OTA_FLASH_SLOT_A) && (meta->active_slot != OTA_FLASH_SLOT_B))
  {
    return 0;
  }

  if ((meta->ota_request != OTA_FLASH_OTA_REQUEST_NONE) &&
      (meta->ota_request != OTA_FLASH_OTA_REQUEST_UPDATE))
  {
    return 0;
  }

  return 1;
}

static int ota_flash_read_meta(ota_flash_meta_t *meta)
{
  if (meta == NULL)
  {
    return -1;
  }

  (void)memcpy(meta, (const void *)OTA_FLASH_META_ADDR, sizeof(*meta));
  return (ota_flash_meta_is_valid(meta) != 0) ? 0 : -1;
}

static int ota_flash_erase_meta(void)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t sector_error = 0U;

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = FLASH_BANK_1;
  erase.Sector = FLASH_SECTOR_2;
  erase.NbSectors = 2U;
  erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  return (HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK) ? 0 : -1;
}

static int ota_flash_program_flashword(uint32_t address, const uint8_t *data)
{
  uint8_t program_buf[OTA_FLASH_PROGRAM_UNIT] OTA_ALIGNED_32;

  if (data == NULL)
  {
    return -1;
  }

  (void)memcpy(program_buf, data, sizeof(program_buf));
  return (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD,
                            address,
                            (uint32_t)(uintptr_t)program_buf) == HAL_OK) ? 0 : -1;
}

static int ota_flash_write_meta(const ota_flash_meta_t *meta)
{
  ota_flash_meta_t writable;
  uint8_t buffer[OTA_FLASH_PROGRAM_UNIT] OTA_ALIGNED_32;
  uint32_t offset = 0U;

  if (meta == NULL)
  {
    return -1;
  }

  writable = *meta;
  writable.magic = OTA_FLASH_META_MAGIC;
  writable.struct_version = OTA_FLASH_META_STRUCT_VERSION;
  writable.meta_crc32 = ota_flash_meta_crc(&writable);

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return -1;
  }

  if (ota_flash_erase_meta() != 0)
  {
    (void)HAL_FLASH_Lock();
    return -1;
  }

  while (offset < sizeof(writable))
  {
    uint32_t copy_len = sizeof(writable) - offset;

    if (copy_len > OTA_FLASH_PROGRAM_UNIT)
    {
      copy_len = OTA_FLASH_PROGRAM_UNIT;
    }

    (void)memset(buffer, 0xFF, sizeof(buffer));
    (void)memcpy(buffer, ((const uint8_t *)&writable) + offset, copy_len);

    if (ota_flash_program_flashword(OTA_FLASH_META_ADDR + offset, buffer) != 0)
    {
      (void)HAL_FLASH_Lock();
      return -1;
    }

    offset += OTA_FLASH_PROGRAM_UNIT;
  }

  (void)HAL_FLASH_Lock();
  return 0;
}

static int ota_write_boot_request(void)
{
  ota_flash_meta_t meta;

  if (ota_flash_read_meta(&meta) != 0)
  {
    ota_flash_make_default_meta(&meta);
  }

  meta.active_slot = ota_detect_running_slot();
  meta.image[ota_slot_index(meta.active_slot)].address = ota_slot_address(meta.active_slot);
  meta.image[ota_slot_index(meta.active_slot)].state = OTA_FLASH_IMAGE_VALID;
  meta.ota_request = OTA_FLASH_OTA_REQUEST_UPDATE;
  meta.target_slot = ota_other_slot(meta.active_slot);
  meta.boot_count = 0U;
  meta.meta_crc32 = ota_flash_meta_crc(&meta);

  return ota_flash_write_meta(&meta);
}

static int ota_write_confirmed(void)
{
  ota_flash_meta_t meta;
  uint32_t active_slot;
  uint8_t active_index;

  if (ota_flash_read_meta(&meta) != 0)
  {
    ota_flash_make_default_meta(&meta);
  }

  active_slot = ota_detect_running_slot();
  active_index = ota_slot_index(active_slot);
  meta.active_slot = active_slot;
  meta.ota_request = OTA_FLASH_OTA_REQUEST_NONE;
  meta.target_slot = OTA_FLASH_SLOT_NONE;
  meta.boot_count = 0U;
  meta.image[active_index].address = ota_slot_address(active_slot);
  meta.image[active_index].state = OTA_FLASH_IMAGE_VALID;
  meta.meta_crc32 = ota_flash_meta_crc(&meta);

  return ota_flash_write_meta(&meta);
}

static int ota_clear_boot_request(void)
{
  ota_flash_meta_t meta;

  if (ota_flash_read_meta(&meta) != 0)
  {
    ota_flash_make_default_meta(&meta);
  }

  meta.ota_request = OTA_FLASH_OTA_REQUEST_NONE;
  meta.target_slot = OTA_FLASH_SLOT_NONE;
  meta.boot_count = 0U;
  meta.meta_crc32 = ota_flash_meta_crc(&meta);

  return ota_flash_write_meta(&meta);
}

int ota_boot_callback(void)
{
  if (ota_write_boot_request() != 0)
  {
    LOGE("ota boot request save failed\r\n");
    return -1;
  }

  LOGI("ota boot request saved, reset\r\n");
  NVIC_SystemReset();
  return 0;
}

int ota_request_callback(void)
{
  if (ota_write_boot_request() != 0)
  {
    LOGE("ota request save failed\r\n");
    return -1;
  }

  LOGI("ota request saved\r\n");
  return 0;
}

int ota_cancel_callback(void)
{
  if (ota_clear_boot_request() != 0)
  {
    LOGE("ota request cancel failed\r\n");
    return -1;
  }

  LOGI("ota request canceled\r\n");
  return 0;
}

int ota_lock_callback(void)
{
  if (ota_write_confirmed() != 0)
  {
    LOGE("ota firmware confirm failed\r\n");
    return -1;
  }

  LOGI("ota firmware confirmed\r\n");
  return 0;
}

int ota_get_info(uint32_t *active_slot,
                 uint32_t *ota_request,
                 uint32_t *need_confirm,
                 uint32_t *rollback_count,
                 uint32_t *rollback_threshold)
{
  ota_flash_meta_t meta;
  uint8_t active_index;

  if ((active_slot == NULL) ||
      (ota_request == NULL) ||
      (need_confirm == NULL) ||
      (rollback_count == NULL) ||
      (rollback_threshold == NULL))
  {
    return -1;
  }

  if (ota_flash_read_meta(&meta) != 0)
  {
    ota_flash_make_default_meta(&meta);
  }

  active_index = ota_slot_index(meta.active_slot);
  *active_slot = meta.active_slot;
  *ota_request = meta.ota_request;
  *need_confirm = (meta.image[active_index].state == OTA_FLASH_IMAGE_PENDING) ? 1U : 0U;
  *rollback_count = meta.boot_count;
  *rollback_threshold = OTA_FLASH_CONFIRM_MAX_ATTEMPTS;

  return 0;
}
