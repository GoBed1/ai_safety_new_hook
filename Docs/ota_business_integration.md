# craner_bootloader 双分区 OTA 业务代码接入说明

本文说明业务 APP 需要增加哪些代码，才能配合 `craner_bootloader` 完成双分区 OTA。本文只描述业务侧职责；Flash 擦写、镜像校验、分区切换和回滚策略由 bootloader 负责。

## 当前工程现状

当前业务工程已经预留了两个 AT 命令入口：

```c
craner#AT+OTABOOT
craner#AT+OTALOCK
```

位置在 `application/at_protocol_handler.c`：

```c
ota_boot_callback();
ota_lock_callback();
```

但当前仓库还缺少 `ota_manage_port.h/.c`。业务代码需要补齐这两个接口，并和 bootloader 约定一块“OTA 控制区”，用于保存升级请求、确认结果、当前镜像状态等标志位。

当前 APP 链接地址为：

```ld
FLASH (rx) : ORIGIN = 0x08100000, LENGTH = 512K
```

因此当前业务 APP 很可能运行在双分区中的一个应用分区。最终分区地址、控制区地址必须以 `craner_bootloader` 的分区表为准。

## 总体职责划分

业务 APP 负责：

- 接收 OTA 开始指令。
- 写入“请求进入 bootloader/OTA 模式”的标志位。
- 复位 MCU，让 bootloader 接管。
- 新固件启动成功后，接收 OTA 确认指令。
- 写入“当前固件确认可用”的标志位。

Bootloader 负责：

- 启动时读取 OTA 控制区。
- 判断是否进入 OTA 接收流程。
- 接收新固件并写入备用分区。
- 校验新固件完整性。
- 切换 active 分区。
- 首次启动新固件时标记为 pending。
- 如果业务 APP 未确认，下一次启动时回滚到旧分区。
- 如果业务 APP 已确认，则固化新分区为有效分区。

## OTA 控制区

业务 APP 和 bootloader 需要共享同一个控制区结构体。建议放在一个公共头文件中，例如：

```c
/* ota_shared.h */
#ifndef OTA_SHARED_H
#define OTA_SHARED_H

#include <stdint.h>

#define OTA_CTRL_MAGIC              0x43524F54UL  /* "CROT" */
#define OTA_CTRL_VERSION            1U

#define OTA_REQUEST_NONE            0U
#define OTA_REQUEST_BOOTLOADER      1U

#define OTA_IMAGE_UNCONFIRMED       0U
#define OTA_IMAGE_CONFIRMED         1U

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t request;
    uint32_t image_confirmed;
    uint32_t boot_count;
    uint32_t last_error;
    uint32_t crc32;
} ota_ctrl_t;

#endif
```

建议 `crc32` 覆盖 `crc32` 字段之前的所有字段。业务 APP 每次修改控制区后都要重新计算并写入 CRC，bootloader 读取时必须校验 magic/version/crc。

## 控制区存放位置

控制区应放在 bootloader 和两个 APP 分区都不会覆盖的位置。常见选择：

- 独立 Flash sector。
- Flash 最末尾一个小区域。
- BKPSRAM 或 RTC backup register，仅适合短期状态，不适合断电保持的升级确认。

如果需要断电后仍可靠保留 OTA 状态，建议使用独立 Flash sector。

示例：

```c
#define OTA_CTRL_FLASH_ADDR         0x081E0000UL
#define OTA_CTRL_FLASH_SECTOR       FLASH_SECTOR_x
#define OTA_CTRL_FLASH_BANK         FLASH_BANK_x
```

实际地址和 sector 必须由 bootloader 分区表统一定义，业务 APP 不要自己另起一套地址。

## 业务侧需要新增的文件

建议新增：

```text
application/ota_manage_port.h
application/ota_manage_port.c
application/ota_shared.h
```

`ota_manage_port.h` 对外提供当前 `at_protocol_handler.c` 已经调用的接口：

```c
#ifndef OTA_MANAGE_PORT_H
#define OTA_MANAGE_PORT_H

int ota_boot_callback(void);
int ota_lock_callback(void);

#endif
```

## OTA 开始指令处理

当业务 APP 收到：

```text
craner#AT+OTABOOT
```

业务侧应执行：

1. 读取 OTA 控制区。
2. 如果控制区无效，初始化默认结构。
3. 写入 `request = OTA_REQUEST_BOOTLOADER`。
4. 保持 `image_confirmed` 不变，或按 bootloader 约定清零。
5. 更新 CRC。
6. 擦除并重写控制区。
7. 适当延迟，确保 AT 应答发送完成。
8. 调用 `NVIC_SystemReset()`。

示例：

```c
int ota_boot_callback(void)
{
    ota_ctrl_t ctrl;

    if (ota_ctrl_load(&ctrl) != 0)
    {
        ota_ctrl_default(&ctrl);
    }

    ctrl.request = OTA_REQUEST_BOOTLOADER;
    ctrl.crc32 = ota_ctrl_calc_crc(&ctrl);

    if (ota_ctrl_save(&ctrl) != 0)
    {
        return -1;
    }

    HAL_Delay(100);
    NVIC_SystemReset();

    return 0;
}
```

`ota_boot_callback()` 返回 `0` 时，当前 AT 层会回复：

```text
craner#OK
```

返回非 `0` 时回复：

```text
craner#ERROR
```

注意：如果希望上位机一定收到 `craner#OK` 后再重启，不能在 `ota_boot_callback()` 内立即复位。更稳的做法是设置一个延迟复位标志，在主循环或任务中延迟 100-500 ms 后复位。

## Bootloader 收到 OTA 请求后应做什么

Bootloader 启动后读取控制区：

```c
if (ctrl.magic == OTA_CTRL_MAGIC &&
    ctrl.request == OTA_REQUEST_BOOTLOADER &&
    crc_ok)
{
    enter_ota_receive_mode();
}
```

进入 OTA 模式后，bootloader 应清除或更新 `request`，避免异常复位后反复进入同一流程。推荐流程：

1. 收到 OTA 请求。
2. 清除 `request` 或改为 `OTA_REQUEST_NONE`。
3. 接收新固件到 inactive 分区。
4. 校验固件头、长度、CRC/Hash、入口地址。
5. 将 inactive 分区标记为 pending boot。
6. 重启并跳转新固件。

## OTA 确认指令处理

当新固件运行正常后，上位机发送：

```text
craner#AT+OTALOCK
```

业务侧应执行“确认当前固件可用”：

1. 读取 OTA 控制区。
2. 校验 magic/version/crc。
3. 写入 `image_confirmed = OTA_IMAGE_CONFIRMED`。
4. 清除 `request`。
5. 可选：清除 `boot_count` 和 `last_error`。
6. 更新 CRC。
7. 擦除并重写控制区。

示例：

```c
int ota_lock_callback(void)
{
    ota_ctrl_t ctrl;

    if (ota_ctrl_load(&ctrl) != 0)
    {
        return -1;
    }

    ctrl.request = OTA_REQUEST_NONE;
    ctrl.image_confirmed = OTA_IMAGE_CONFIRMED;
    ctrl.boot_count = 0U;
    ctrl.last_error = 0U;
    ctrl.crc32 = ota_ctrl_calc_crc(&ctrl);

    return ota_ctrl_save(&ctrl);
}
```

Bootloader 下次启动时读取到 `image_confirmed == OTA_IMAGE_CONFIRMED`，应将当前分区固化为 valid，不再回滚。

## 推荐状态机

建议 bootloader/APP 共享以下状态概念：

```text
IDLE
  无 OTA 请求，启动当前 valid APP。

REQUEST_BOOTLOADER
  业务 APP 收到 OTABOOT 后写入，重启后 bootloader 进入 OTA 接收。

PENDING_CONFIRM
  bootloader 已切到新固件，但新固件尚未确认。

CONFIRMED
  业务 APP 收到 OTALOCK 后写入，新固件被确认可用。

ROLLBACK
  新固件未确认且超过允许启动次数，bootloader 回滚旧固件。
```

## 业务代码需要注意的点

- `OTABOOT` 只负责请求进入 bootloader，不负责在业务 APP 内写新固件。
- `OTALOCK` 只能在新固件启动并业务功能自检通过后发送。
- 控制区写 Flash 前要关中断或加锁，避免并发写。
- 写 Flash 前要确保没有任务正在访问同一 Flash bank 的代码或数据，STM32H7 双 bank 下也要确认 bank 分布。
- 写完控制区后建议读回校验。
- 控制区结构体要做版本号，后续字段变化时 bootloader 可兼容处理。
- 不建议只用一个裸 magic 值，至少要 magic + version + crc。
- 如果业务 APP 复位前要回 ACK，应延迟复位，避免串口 DMA 尚未发送完成。

## AT 命令建议

当前已有：

```text
craner#AT+OTABOOT
craner#AT+OTALOCK
```

建议后续增加查询命令：

```text
craner#AT+OTASTATE?
```

返回示例：

```text
craner#OTA:request=0,confirmed=1,boot_count=0,last_error=0
```

这样现场可以判断设备当前是否处于 pending、已确认或等待 bootloader 的状态。

## 最小接入清单

业务 APP 至少需要完成：

- 新增 `ota_manage_port.h/.c`。
- 新增和 bootloader 一致的 `ota_shared.h`。
- 实现 `ota_boot_callback()`：写 OTA 请求标志，延迟复位。
- 实现 `ota_lock_callback()`：确认当前固件，清除 pending/请求状态。
- 在 `at_protocol_handler.c` 保持 `OTABOOT/OTALOCK` 调用。
- 确认链接脚本中的 APP 起始地址和 bootloader 分区表一致。
- 确认 OTA 控制区不被任一 APP 分区擦写覆盖。

