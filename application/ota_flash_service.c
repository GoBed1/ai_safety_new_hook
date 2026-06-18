#include "ota_flash_service.h"

#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"

#define OTA_FLASH_META_MAGIC          0x4F54414DUL
#define OTA_FLASH_META_STRUCT_VERSION 2UL
#define OTA_FLASH_SECTOR_SIZE         0x00020000UL
#define OTA_FLASH_BANK1_BASE          0x08000000UL
#define OTA_FLASH_BANK2_BASE          0x08100000UL
#define OTA_FLASH_BANK_SIZE           0x00100000UL
#define OTA_FLASH_PROGRAM_UNIT        32U
#define OTA_FLASH_ERASE_VOLTAGE       FLASH_VOLTAGE_RANGE_3

#if defined(__GNUC__)
#define OTA_FLASH_ALIGNED_32 __attribute__((aligned(32)))
#else
#define OTA_FLASH_ALIGNED_32
#endif

static uint32_t ota_flash_meta_crc(const ota_flash_meta_t *meta)
{
  ota_flash_meta_t temp;

  if (meta == NULL)
  {
    return 0U;
  }

  temp = *meta;
  temp.meta_crc32 = 0U;

  return ota_flash_crc32_finish(ota_flash_crc32_update(0xFFFFFFFFUL,
                                                       (const uint8_t *)&temp,
                                                       (uint32_t)sizeof(temp)));
}

static uint8_t ota_flash_addr_in_flash(uint32_t address)
{
  return (uint8_t)((address >= OTA_FLASH_BANK1_BASE) &&
                   (address < (OTA_FLASH_BANK1_BASE + (OTA_FLASH_BANK_SIZE * 2UL))));
}

static ota_flash_status_t ota_flash_get_bank_sector(uint32_t address,
                                                    uint32_t *bank,
                                                    uint32_t *sector)
{
  uint32_t bank_base;

  if ((bank == NULL) || (sector == NULL) || (ota_flash_addr_in_flash(address) == 0U))
  {
    return OTA_FLASH_ERR_PARAM;
  }

  if (address < OTA_FLASH_BANK2_BASE)
  {
    *bank = FLASH_BANK_1;
    bank_base = OTA_FLASH_BANK1_BASE;
  }
  else
  {
    *bank = FLASH_BANK_2;
    bank_base = OTA_FLASH_BANK2_BASE;
  }

  *sector = (address - bank_base) / OTA_FLASH_SECTOR_SIZE;
  return OTA_FLASH_OK;
}

static ota_flash_status_t ota_flash_erase_range(uint32_t address, uint32_t len)
{
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t error = 0U;
  uint32_t bank = 0U;
  uint32_t first_sector = 0U;
  uint32_t last_sector = 0U;
  uint32_t end_addr;

  if ((len == 0U) || ((address % OTA_FLASH_SECTOR_SIZE) != 0U))
  {
    return OTA_FLASH_ERR_PARAM;
  }

  end_addr = address + len - 1U;
  if ((end_addr < address) ||
      (ota_flash_get_bank_sector(address, &bank, &first_sector) != OTA_FLASH_OK) ||
      (ota_flash_get_bank_sector(end_addr, &bank, &last_sector) != OTA_FLASH_OK))
  {
    return OTA_FLASH_ERR_RANGE;
  }

  erase.TypeErase = FLASH_TYPEERASE_SECTORS;
  erase.Banks = bank;
  erase.Sector = first_sector;
  erase.NbSectors = (last_sector - first_sector) + 1U;
  erase.VoltageRange = OTA_FLASH_ERASE_VOLTAGE;

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return OTA_FLASH_ERR_HAL;
  }

  if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return OTA_FLASH_ERR_HAL;
  }

  if (HAL_FLASH_Lock() != HAL_OK)
  {
    return OTA_FLASH_ERR_HAL;
  }

  return OTA_FLASH_OK;
}

static ota_flash_status_t ota_flash_program_flashword(uint32_t address, const uint8_t *data)
{
  uint8_t aligned[OTA_FLASH_PROGRAM_UNIT] OTA_FLASH_ALIGNED_32;

  if ((data == NULL) || ((address % OTA_FLASH_PROGRAM_UNIT) != 0U))
  {
    return OTA_FLASH_ERR_PARAM;
  }

  (void)memcpy(aligned, data, OTA_FLASH_PROGRAM_UNIT);

  if (HAL_FLASH_Unlock() != HAL_OK)
  {
    return OTA_FLASH_ERR_HAL;
  }

  if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, address, (uint32_t)aligned) != HAL_OK)
  {
    (void)HAL_FLASH_Lock();
    return OTA_FLASH_ERR_HAL;
  }

  if (HAL_FLASH_Lock() != HAL_OK)
  {
    return OTA_FLASH_ERR_HAL;
  }

  return OTA_FLASH_OK;
}

ota_flash_status_t ota_flash_service_init(void)
{
  ota_flash_meta_t meta;

  if (ota_flash_read_meta(&meta) == OTA_FLASH_OK)
  {
    return OTA_FLASH_OK;
  }

  ota_flash_make_default_meta(&meta);
  return ota_flash_write_meta(&meta);
}

void ota_flash_make_default_meta(ota_flash_meta_t *meta)
{
  if (meta == NULL)
  {
    return;
  }

  (void)memset(meta, 0, sizeof(*meta));
  meta->magic = OTA_FLASH_META_MAGIC;
  meta->struct_version = OTA_FLASH_META_STRUCT_VERSION;
  meta->ota_request = OTA_FLASH_OTA_REQUEST_NONE;
  meta->target_slot = OTA_FLASH_SLOT_NONE;
  meta->active_slot = OTA_FLASH_SLOT_A;
  meta->image[0].address = OTA_FLASH_SLOT_A_ADDR;
  meta->image[0].state = OTA_FLASH_IMAGE_EMPTY;
  meta->image[1].address = OTA_FLASH_SLOT_B_ADDR;
  meta->image[1].state = OTA_FLASH_IMAGE_EMPTY;
  meta->meta_crc32 = ota_flash_meta_crc(meta);
}

ota_flash_status_t ota_flash_read_meta(ota_flash_meta_t *meta)
{
  const ota_flash_meta_t *stored = (const ota_flash_meta_t *)OTA_FLASH_META_ADDR;

  if (meta == NULL)
  {
    return OTA_FLASH_ERR_PARAM;
  }

  (void)memcpy(meta, stored, sizeof(*meta));

  if ((meta->magic != OTA_FLASH_META_MAGIC) ||
      (meta->struct_version != OTA_FLASH_META_STRUCT_VERSION) ||
      (meta->meta_crc32 != ota_flash_meta_crc(meta)))
  {
    return OTA_FLASH_ERR_META;
  }

  if ((meta->active_slot != OTA_FLASH_SLOT_A) && (meta->active_slot != OTA_FLASH_SLOT_B))
  {
    return OTA_FLASH_ERR_META;
  }

  if ((meta->ota_request != OTA_FLASH_OTA_REQUEST_NONE) &&
      (meta->ota_request != OTA_FLASH_OTA_REQUEST_UPDATE))
  {
    return OTA_FLASH_ERR_META;
  }

  return OTA_FLASH_OK;
}

ota_flash_status_t ota_flash_write_meta(const ota_flash_meta_t *meta)
{
  ota_flash_meta_t writable;
  uint8_t buffer[OTA_FLASH_PROGRAM_UNIT];
  uint32_t offset = 0U;

  if (meta == NULL)
  {
    return OTA_FLASH_ERR_PARAM;
  }

  writable = *meta;
  writable.magic = OTA_FLASH_META_MAGIC;
  writable.struct_version = OTA_FLASH_META_STRUCT_VERSION;
  writable.meta_crc32 = ota_flash_meta_crc(&writable);

  if (ota_flash_erase_range(OTA_FLASH_META_ADDR, OTA_FLASH_META_SIZE) != OTA_FLASH_OK)
  {
    return OTA_FLASH_ERR_HAL;
  }

  while (offset < sizeof(writable))
  {
    uint32_t copy_len = (uint32_t)sizeof(writable) - offset;

    if (copy_len > OTA_FLASH_PROGRAM_UNIT)
    {
      copy_len = OTA_FLASH_PROGRAM_UNIT;
    }

    (void)memset(buffer, 0xFF, sizeof(buffer));
    (void)memcpy(buffer, ((const uint8_t *)&writable) + offset, copy_len);

    if (ota_flash_program_flashword(OTA_FLASH_META_ADDR + offset, buffer) != OTA_FLASH_OK)
    {
      return OTA_FLASH_ERR_HAL;
    }

    offset += OTA_FLASH_PROGRAM_UNIT;
  }

  return OTA_FLASH_OK;
}

ota_flash_slot_t ota_flash_get_active_slot(void)
{
  ota_flash_meta_t meta;

  if (ota_flash_read_meta(&meta) != OTA_FLASH_OK)
  {
    return OTA_FLASH_SLOT_A;
  }

  return (ota_flash_slot_t)meta.active_slot;
}

ota_flash_slot_t ota_flash_get_inactive_slot(void)
{
  return (ota_flash_get_active_slot() == OTA_FLASH_SLOT_A) ? OTA_FLASH_SLOT_B : OTA_FLASH_SLOT_A;
}

uint8_t ota_flash_is_valid_slot(ota_flash_slot_t slot)
{
  return (uint8_t)((slot == OTA_FLASH_SLOT_A) || (slot == OTA_FLASH_SLOT_B));
}

ota_flash_status_t ota_flash_request_ota(ota_flash_slot_t target_slot)
{
  ota_flash_meta_t meta;

  if (ota_flash_is_valid_slot(target_slot) == 0U)
  {
    return OTA_FLASH_ERR_PARAM;
  }

  if (ota_flash_read_meta(&meta) != OTA_FLASH_OK)
  {
    ota_flash_make_default_meta(&meta);
  }

  meta.ota_request = OTA_FLASH_OTA_REQUEST_UPDATE;
  meta.target_slot = target_slot;
  return ota_flash_write_meta(&meta);
}

ota_flash_status_t ota_flash_confirm_app(void)
{
  ota_flash_meta_t meta;
  ota_flash_slot_t active_slot;
  uint8_t active_index;

  if (ota_flash_read_meta(&meta) != OTA_FLASH_OK)
  {
    return OTA_FLASH_ERR_META;
  }

  active_slot = (ota_flash_slot_t)meta.active_slot;
  if (ota_flash_is_valid_slot(active_slot) == 0U)
  {
    return OTA_FLASH_ERR_META;
  }

  active_index = (active_slot == OTA_FLASH_SLOT_B) ? 1U : 0U;
  if (meta.image[active_index].state == OTA_FLASH_IMAGE_VALID)
  {
    return OTA_FLASH_OK;
  }

  if (meta.image[active_index].state != OTA_FLASH_IMAGE_PENDING)
  {
    return OTA_FLASH_ERR_STATE;
  }

  meta.image[active_index].state = OTA_FLASH_IMAGE_VALID;
  meta.boot_count = 0U;
  meta.ota_request = OTA_FLASH_OTA_REQUEST_NONE;
  meta.target_slot = OTA_FLASH_SLOT_NONE;

  return ota_flash_write_meta(&meta);
}

uint32_t ota_flash_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len)
{
  if (data == NULL)
  {
    return crc;
  }

  for (uint32_t i = 0U; i < len; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 1UL) != 0UL)
      {
        crc = (crc >> 1U) ^ 0xEDB88320UL;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

uint32_t ota_flash_crc32_finish(uint32_t crc)
{
  return crc ^ 0xFFFFFFFFUL;
}
