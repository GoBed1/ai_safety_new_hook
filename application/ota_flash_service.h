#ifndef OTA_FLASH_SERVICE_H
#define OTA_FLASH_SERVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#define OTA_FLASH_BOOT_BASE_ADDR 0x08000000UL
#define OTA_FLASH_BOOT_SIZE      0x00040000UL

#define OTA_FLASH_META_ADDR      0x08040000UL
#define OTA_FLASH_META_SIZE      0x00040000UL

#define OTA_FLASH_SLOT_A_ADDR    0x08100000UL
#define OTA_FLASH_SLOT_B_ADDR    0x08180000UL
#define OTA_FLASH_SLOT_SIZE      0x00080000UL

typedef enum
{
  OTA_FLASH_OK = 0,
  OTA_FLASH_ERR_PARAM = -1,
  OTA_FLASH_ERR_RANGE = -2,
  OTA_FLASH_ERR_HAL = -3,
  OTA_FLASH_ERR_META = -4,
  OTA_FLASH_ERR_STATE = -5,
} ota_flash_status_t;

typedef enum
{
  OTA_FLASH_SLOT_A = 0,
  OTA_FLASH_SLOT_B = 1,
  OTA_FLASH_SLOT_NONE = 0xFF,
} ota_flash_slot_t;

typedef enum
{
  OTA_FLASH_IMAGE_EMPTY = 0,
  OTA_FLASH_IMAGE_VALID = 1,
  OTA_FLASH_IMAGE_PENDING = 2,
} ota_flash_image_state_t;

typedef enum
{
  OTA_FLASH_OTA_REQUEST_NONE = 0,
  OTA_FLASH_OTA_REQUEST_UPDATE = 1,
} ota_flash_ota_request_t;

typedef struct
{
  uint32_t address;
  uint32_t size;
  uint32_t crc32;
  uint32_t version;
  uint32_t state;
} ota_flash_image_meta_t;

typedef struct
{
  uint32_t magic;
  uint32_t struct_version;
  uint32_t ota_request;
  uint32_t target_slot;
  uint32_t active_slot;
  uint32_t boot_count;
  ota_flash_image_meta_t image[2];
  uint32_t meta_crc32;
} ota_flash_meta_t;

ota_flash_status_t ota_flash_service_init(void);
ota_flash_status_t ota_flash_read_meta(ota_flash_meta_t *meta);
ota_flash_status_t ota_flash_write_meta(const ota_flash_meta_t *meta);
void ota_flash_make_default_meta(ota_flash_meta_t *meta);

ota_flash_slot_t ota_flash_get_active_slot(void);
ota_flash_slot_t ota_flash_get_inactive_slot(void);
uint8_t ota_flash_is_valid_slot(ota_flash_slot_t slot);
ota_flash_status_t ota_flash_request_ota(ota_flash_slot_t target_slot);
ota_flash_status_t ota_flash_confirm_app(void);

uint32_t ota_flash_crc32_update(uint32_t crc, const uint8_t *data, uint32_t len);
uint32_t ota_flash_crc32_finish(uint32_t crc);

#ifdef __cplusplus
}
#endif

#endif /* OTA_FLASH_SERVICE_H */
