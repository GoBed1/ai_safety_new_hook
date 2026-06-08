# UART Receive Ring Stats Usage

本文档说明 `uart_manage` 接收环形缓冲区统计功能的使用方法。

## 1. 功能作用

该功能用于监控 `process_ring_buffer` 是否接近满载，以及是否发生过接收数据丢弃。

统计信息包括：

- `size`: 环形缓冲区总大小
- `used`: 当前已使用字节数
- `free`: 当前剩余字节数
- `high_watermark`: 历史最高水位
- `drop_bytes`: 因缓冲区空间不足而丢弃的总字节数
- `overflow_count`: 发生写入空间不足的次数
- `near_full`: 当前是否接近满载

当前 `near_full` 阈值在 `uart_manage.c` 中定义：

```c
#define UART_RECV_RING_NEAR_FULL_PERCENT 80U
```

即缓冲区使用量达到 80% 时，`near_full` 为 `1`。

## 2. 开关宏

统计功能由宏 `UART_MANAGE_RECV_RING_STATS_ENABLE` 控制。

该宏建议在 `application/uart_manage_port.h` 中配置。

默认关闭：

```c
#ifndef UART_MANAGE_RECV_RING_STATS_ENABLE
#define UART_MANAGE_RECV_RING_STATS_ENABLE 0U
#endif
```

如需开启，修改 `application/uart_manage_port.h`：

```c
#define UART_MANAGE_RECV_RING_STATS_ENABLE 1U
```

关闭后，以下内容不会参与编译：

- `uart_recv_ring_stats_t`
- `recv_ring_high_watermark`
- `recv_ring_drop_bytes`
- `recv_ring_overflow_count`
- `uart_manage_get_recv_ring_stats()`
- `uart_manage_get_recv_ring_stats_by_name()`

## 3. 查询接口

按接口对象查询：

```c
int uart_manage_get_recv_ring_stats(uart_inferface_t *m_obj,
                                    uart_recv_ring_stats_t *stats);
```

按名称查询：

```c
int uart_manage_get_recv_ring_stats_by_name(const char *name,
                                            uart_recv_ring_stats_t *stats);
```

返回值：

- `0`: 查询成功
- `-1`: 参数错误或未找到对象

## 4. 示例代码

建议在任务上下文中周期性查询，不要在 UART 接收中断或接收回调里打印日志。

例如每 1s 监控一次 GPS：

```c
#if UART_MANAGE_RECV_RING_STATS_ENABLE
static void log_uart_ring_stats(const char *name)
{
    uart_recv_ring_stats_t stats;

    if (uart_manage_get_recv_ring_stats_by_name(name, &stats) != 0)
    {
        return;
    }

    LOGI("[UART:%s] used=%u/%u free=%u high=%u drop=%lu ovf=%lu near=%u\r\n",
         name,
         stats.used,
         stats.size,
         stats.free,
         stats.high_watermark,
         stats.drop_bytes,
         stats.overflow_count,
         stats.near_full);
}
#endif
```

在任务中调用：

```c
#if UART_MANAGE_RECV_RING_STATS_ENABLE
static TickType_t last_uart_stats_tick = 0;

if ((xTaskGetTickCount() - last_uart_stats_tick) >= pdMS_TO_TICKS(1000))
{
    last_uart_stats_tick = xTaskGetTickCount();
    log_uart_ring_stats("gps");
    log_uart_ring_stats("4g");
}
#endif
```

## 5. 判断问题的方法

如果 `near_full` 经常为 `1`，说明消费速度跟不上接收速度。

如果 `overflow_count` 持续增加，说明已经发生过缓冲区写不完整。

如果 `drop_bytes` 持续增加，说明已经有接收数据被丢弃。

常见处理方式：

- 提高对应任务的读取频率
- 增大 `process_buffer_size`
- 减少接收回调中的耗时处理
- 将解析逻辑放到任务中，不放在中断回调中
- 检查日志打印是否过多导致消费任务被拖慢

## 6. 当前项目建议

当前项目中建议重点监控：

- `"gps"`: GPS 数据可能持续输出，且当前任务周期较慢时更容易堆积
- `"4g"`: MQTT/AT 数据突发时可能出现短时间堆积

示例：

```c
log_uart_ring_stats("gps");
log_uart_ring_stats("4g");
```
