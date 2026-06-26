/*
 * 文件用途：Echo - 直接回调直通模式（Callback-Direct Mode）
 *
 * 如何使用：
 * 1) 将本文件重命名为 uart_manage_port.c 并加入工程编译。
 * 2) 在系统初始化阶段调用 init_echo_app()。
 * 3) 保证 HAL_UARTEx_RxEventCallback/HAL_UART_TxCpltCallback 未被其他文件重复定义覆盖。
 *
 * 预期效果：
 * - 收到串口数据后，uart_manage 在 RxEvent 中直接调用 echo_callback。
 * - echo_callback 内立即调用 uart_manage_dma_send_by_name("echo", ...) 回发。
 * - 上位机向 USART1 发送 "abc"，应快速收到 "abc" 回显。
 */
/* port.c */
#include "uart_manage.h"

/* DMA buffer placement */
#if defined(__GNUC__)
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define DMA_BUFFER
#endif

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

static uint8_t uart1_recv_buff[256U] DMA_BUFFER;
static uint8_t uart1_send_buff[256U] DMA_BUFFER;
static uint8_t uart1_send_fifo_buff[256U] DMA_BUFFER;
static uint8_t uart1_process_buff[256U * 4U] DMA_BUFFER;

static uint32_t echo_callback(uint8_t *buf, uint16_t len)
{
  (void)uart_manage_dma_send_by_name("echo", buf, len);
  return 0U;
}

const uart_inferface_t uart_manage_table[] = {
  {
    .name = "echo",
    .uart_h = &huart1,
    .dma_h = &hdma_usart1_rx,
    .recv_buffer = uart1_recv_buff,
    .recv_buffer_size = sizeof(uart1_recv_buff),
    .process_buffer = uart1_process_buff,
    .process_buffer_size = sizeof(uart1_process_buff),
    .recv_callback = echo_callback,
    .send_buffer = uart1_send_buff,
    .send_buffer_size = sizeof(uart1_send_buff),
    .send_fifo_buffer = uart1_send_fifo_buff,
    .send_fifo_size = sizeof(uart1_send_fifo_buff),
    .send_callback = NULL,
  },
};

const uint16_t uart_manage_table_size =
  (uint16_t)(sizeof(uart_manage_table) / sizeof(uart_manage_table[0]));

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_manage_send_completed_hook(huart);
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  (void *)huart;
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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void *)huart;
}

void init_echo_app(void)
{
  (void)uart_manage_init_table(uart_manage_table, uart_manage_table_size);
  (void)uart_manage_enable_dma_recv_by_name("echo");
}