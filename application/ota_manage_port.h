#ifndef OTA_MANAGE_PORT_H
#define OTA_MANAGE_PORT_H

#include <stdint.h>

#define OTA_FLASH_META_ADDR                 0x08040000UL
#define OTA_FLASH_META_SIZE                 0x00040000UL
#define OTA_FLASH_SLOT_A_ADDR               0x08100000UL
#define OTA_FLASH_SLOT_B_ADDR               0x08180000UL
#define OTA_FLASH_SLOT_SIZE                 0x00080000UL

#define OTA_FLASH_META_MAGIC                0x4F54414DUL
#define OTA_FLASH_META_STRUCT_VERSION       2UL
#define OTA_FLASH_PROGRAM_UNIT              32U

#define OTA_FLASH_SLOT_A                    0U
#define OTA_FLASH_SLOT_B                    1U
#define OTA_FLASH_SLOT_NONE                 0xFFU

#define OTA_FLASH_IMAGE_EMPTY               0U
#define OTA_FLASH_IMAGE_VALID               1U
#define OTA_FLASH_IMAGE_PENDING             2U

#define OTA_FLASH_OTA_REQUEST_NONE          0U
#define OTA_FLASH_OTA_REQUEST_UPDATE        1U
#define OTA_FLASH_CONFIRM_MAX_ATTEMPTS      10UL

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

int ota_boot_callback(void);
int ota_request_callback(void);
int ota_cancel_callback(void);
int ota_lock_callback(void);
int ota_get_info(uint32_t *active_slot,
                 uint32_t *ota_request,
                 uint32_t *rollback_count,
                 uint32_t *rollback_threshold);

#endif
