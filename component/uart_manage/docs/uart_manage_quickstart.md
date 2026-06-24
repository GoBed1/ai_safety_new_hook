# uart_manage_quickstart

本文介绍 `uart_manage` 子模块实现 Echo 的两种方式，并给出命名与使用建议。

## 1. 使用样例

### 方式A：**环形缓冲线程处理模式**（Ring-Task Mode）

- 对应示例：`echo_callback_direct_port.c.md`
- 特点：中断只负责搬运数据到 `process_ring_buffer`，业务在线程里解析/回显。
- 适合：协议解析复杂、需要状态机、希望中断逻辑尽量轻。
- 如何使用：重命名为 `uart_manage_port.c` 后，调用 `init_echo_app()`（该实现会创建线程并持续回显）。
- 预期效果：串口数据先入 ring，再由线程回发；回显稳定，便于后续扩展协议解析。

### 方式B：**直接回调直通模式**（Callback-Direct Mode）

- 对应示例：`echo_ring_task_port.c.md`
- 特点：收到数据后立即走 `recv_callback`，可直接回显。
- 适合：逻辑极简、低延迟触发、无需复杂分帧状态机。
- 如何使用：重命名为 `uart_manage_port.c` 后，调用 `init_echo_app()`（该实现仅注册并使能 DMA 接收）。
- 预期效果：数据到达后直接回调并立即回发；链路更短，回显延迟更低。

---

## 2. 通用基础（两种方式都需要）

### 2.1 管理表与缓冲区

```c
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

static uint8_t uart1_recv_buff[256U] DMA_BUFFER;
static uint8_t uart1_send_buff[256U] DMA_BUFFER;
static uint8_t uart1_send_fifo_buff[256U] DMA_BUFFER;
static uint8_t uart1_process_buff[256U * 4U] DMA_BUFFER;

const uart_inferface_t uart_manage_table[] = {
  {
    .name = "echo",
    .uart_h = &huart1,
    .dma_h = &hdma_usart1_rx,
    .recv_buffer = uart1_recv_buff,
    .recv_buffer_size = sizeof(uart1_recv_buff),
    .process_buffer = uart1_process_buff,
    .process_buffer_size = sizeof(uart1_process_buff),
    .recv_callback = NULL, /* 方式B时改为 echo_callback */
    .send_buffer = uart1_send_buff,
    .send_buffer_size = sizeof(uart1_send_buff),
    .send_fifo_buffer = uart1_send_fifo_buff,
    .send_fifo_size = sizeof(uart1_send_fifo_buff),
    .send_callback = NULL,
  }
};
```

### 2.2 HAL 回调桥接

```c
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_manage_send_completed_hook(huart);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  uart_inferface_t *m_obj = uart_manage_get_obj(huart);
  if (m_obj != NULL)
  {
    if (size > 0U)
    {
      (void)uart_manage_recv_idle_hook(m_obj, INTERRUPT_TYPE_UART, size);
    }
    (void)uart_manage_enable_dma_recv(huart);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  (void)uart_manage_enable_dma_recv(huart);
}
```

---

## 3. 方式A：环形缓冲线程处理模式（Ring-Task Mode）

### 3.1 初始化

```c
void config_echo_app(void)
{
  (void)uart_manage_init_table(uart_manage_table, uart_manage_table_size);
  (void)uart_manage_enable_dma_recv_by_name("echo");
}
```

### 3.2 周期处理

```c
void update_echo_app(void)
{
  uart_inferface_t *m_obj = uart_manage_get_obj_by_name("echo");
  if (m_obj == NULL) return;

  lwrb_sz_t available = lwrb_get_full(&m_obj->process_ring_buffer);
  if (available == 0) return;

  uint8_t to_read_buffer[128];
  lwrb_sz_t to_read = (available > sizeof(to_read_buffer)) ? sizeof(to_read_buffer) : available;
  lwrb_sz_t read_size = lwrb_read(&m_obj->process_ring_buffer, to_read_buffer, to_read);
  if (read_size > 0)
  {
    (void)uart_manage_dma_send_by_name("echo", to_read_buffer, (uint16_t)read_size);
  }
}
```

### 3.3 线程入口

```c
void echo_app(void *argument)
{
  (void)argument;
  config_echo_app();

  for (;;)
  {
    osDelay(10);
    update_echo_app();
  }
}
```

---

## 4. 方式B：直接回调直通模式（Callback-Direct Mode）

### 4.1 回调函数

```c
static uint32_t echo_callback(uint8_t *buf, uint16_t len)
{
  (void)uart_manage_dma_send_by_name("echo", buf, len);
  return 0U;
}
```

并在管理表中设置：

- `.recv_callback = echo_callback`

### 4.2 初始化（关键）

```c
void init_echo_app(void)
{
  (void)uart_manage_init_table(uart_manage_table, uart_manage_table_size);
  (void)uart_manage_enable_dma_recv_by_name("echo");
}
```

> 注意：如果只注册管理表，不调用 `uart_manage_enable_dma_recv_by_name("echo")`，回调不会被触发。

---

## 5. 如何选择

- 想要稳健、可扩展、易调试：选 **方式A（Ring-Task Mode）**。
- 想要最短路径、最小延迟回显：选 **方式B（Callback-Direct Mode）**。

## 6. 快速验证

1. 将目标示例文件重命名为 `uart_manage_port.c`。
2. 在系统初始化阶段调用 `init_echo_app()`（或 `config_echo_app()` + 任务循环）。
3. 上位机向 `USART1` 发送任意字符串，应收到同样内容回显。



