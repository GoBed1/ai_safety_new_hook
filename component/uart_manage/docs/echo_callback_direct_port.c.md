/*
 * 文件用途：Echo - 环形缓冲线程处理模式（Ring-Task Mode）
 *
 * 如何使用：
 * 1) 将本文件重命名为 uart_manage_port.c 并加入工程编译。
 * 2) 在系统初始化阶段调用 init_echo_app()。
 * 3) 保证 USART1 与对应 DMA 已由 CubeMX/HAL 正常初始化。
 *
 * 预期效果：
 * - 串口接收数据先进入 process_ring_buffer。
 * - echo_app 线程周期调用 update_echo_app()，读取 ring 后原样回发。
 * - 上位机向 USART1 发送 "abc"，应收到 "abc"。
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

const uart_inferface_t uart_manage_table[] = {
  {
    .name = "echo",
    .uart_h = &huart1,
    .dma_h = &hdma_usart1_rx,
    .recv_buffer = uart1_recv_buff,
    .recv_buffer_size = sizeof(uart1_recv_buff),
    .process_buffer = uart1_process_buff,
    .process_buffer_size = sizeof(uart1_process_buff),
    .recv_callback = NULL,
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

void config_echo_app(void){
  uart_manage_init_table(uart_manage_table, uart_manage_table_size);
  (void)uart_manage_enable_dma_recv_by_name("echo");
}

void update_echo_app(void){
  uart_inferface_t *m_obj = uart_manage_get_obj_by_name("echo");
  if (m_obj == NULL)
  {
      return;
  }

  lwrb_sz_t available = lwrb_get_full(&m_obj->process_ring_buffer);
  if (available == 0)
  {
      return;
  }
  uint8_t to_read_buffer[128];
  lwrb_sz_t to_read = (available > sizeof(to_read_buffer)) ? sizeof(to_read_buffer) : available;
  lwrb_sz_t read_size = lwrb_read(&m_obj->process_ring_buffer, to_read_buffer, to_read);
  
  if (read_size > 0)
  {
      uart_manage_dma_send_by_name("echo", to_read_buffer, (uint16_t)read_size);
  }
}

/* app.c */
void echo_app(void *argument)
{
	(void)argument;
  config_echo_app();

	for(;;)
	{
    osDelay(10);
    update_echo_app();
	}
}

static osThreadId_t echo_app_handle;
static const osThreadAttr_t echo_app_attributes = {
	.name = "EchoApp",
	.stack_size = 1024 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

void init_echo_app(void)
{
	echo_app_handle = osThreadNew(echo_app, NULL, &echo_app_attributes);
	if (echo_app_handle == NULL) {
		Error_Handler();
	}
}

