#include "uart_manage_port.h"

#include "board_manage.h"
#include "uart_manage.h"
#include "Modbus.h"
#include "system_def.h"

/* DMA buffer placement */
#if defined(__GNUC__)
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))
#else
#define DMA_BUFFER
#endif

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
static uint8_t uart1_recv_buff[512U] DMA_BUFFER;
static uint8_t uart1_send_buff[512U] DMA_BUFFER;
static uint8_t uart1_send_fifo_buff[512U] DMA_BUFFER;
static uint8_t uart1_process_buff[512U * 2U] DMA_BUFFER;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
static uint8_t uart3_recv_buff[512U] DMA_BUFFER;
static uint8_t uart3_send_buff[512U] DMA_BUFFER;
static uint8_t uart3_send_fifo_buff[512U] DMA_BUFFER;
static uint8_t uart3_process_buff[512U * 2U] DMA_BUFFER;

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
static uint8_t uart5_recv_buff[512U] DMA_BUFFER;
static uint8_t uart5_send_buff[512U] DMA_BUFFER;
static uint8_t uart5_send_fifo_buff[512U] DMA_BUFFER;
static uint8_t uart5_process_buff[512U * 2U] DMA_BUFFER;

const uart_inferface_t uart_manage_table[] = {
    {
        .name = "shell",
        .uart_h = &huart5,
        .dma_h = &hdma_uart5_rx,
        .recv_buffer = uart5_recv_buff,
        .recv_buffer_size = sizeof(uart5_recv_buff),
        .process_buffer = uart5_process_buff,
        .process_buffer_size = sizeof(uart5_process_buff),
        .recv_callback = uart_shell_recv_callback,
        .send_buffer = uart5_send_buff,
        .send_buffer_size = sizeof(uart5_send_buff),
        .send_fifo_buffer = uart5_send_fifo_buff,
        .send_fifo_size = sizeof(uart5_send_fifo_buff),
        .send_callback = NULL,
    },
    {
        .name = "4g",
        .uart_h = &huart1,
        .dma_h = &hdma_usart1_rx,
        .recv_buffer = uart1_recv_buff,
        .recv_buffer_size = sizeof(uart1_recv_buff),
        .process_buffer = uart1_process_buff,
        .process_buffer_size = sizeof(uart1_process_buff),
        .recv_callback = uart_4g_recv_callback,
        .send_buffer = uart1_send_buff,
        .send_buffer_size = sizeof(uart1_send_buff),
        .send_fifo_buffer = uart1_send_fifo_buff,
        .send_fifo_size = sizeof(uart1_send_fifo_buff),
        .send_callback = NULL,
    },
    {
        .name = "gps",
        .uart_h = &huart3,
        .dma_h = &hdma_usart3_rx,
        .recv_buffer = uart3_recv_buff,
        .recv_buffer_size = sizeof(uart3_recv_buff),
        .process_buffer = uart3_process_buff,
        .process_buffer_size = sizeof(uart3_process_buff),
        .recv_callback = NULL,
        .send_buffer = uart3_send_buff,
        .send_buffer_size = sizeof(uart3_send_buff),
        .send_fifo_buffer = uart3_send_fifo_buff,
        .send_fifo_size = sizeof(uart3_send_fifo_buff),
        .send_callback = NULL,
    },
};

void init_uart_manage(void)
{
  const uint16_t table_size = (uint16_t)(sizeof(uart_manage_table) / sizeof(uart_manage_table[0]));
  int init_result[UART_MANAGE_MAX_OBJECTS] = {0};

  if (table_size == 0U || table_size > UART_MANAGE_MAX_OBJECTS)
  {
    LOGE("uart_manage init invalid table size: %u\r\n", table_size);
    return;
  }

  for (uint16_t i = 0U; i < table_size; ++i)
  {
    int ret = uart_manage_register_interface((uart_inferface_t *)&uart_manage_table[i]);
    if (ret != UART_MANAGE_OK)
    {
      init_result[i] = ret;
      continue;
    }

    ret = uart_manage_enable_dma_recv(uart_manage_table[i].uart_h);
    init_result[i] = ret;
  }

  for (uint16_t i = 0U; i < table_size; ++i)
  {
    if (init_result[i] == UART_MANAGE_OK)
    {
      LOGI("uart_manage init %s ok\r\n", uart_manage_table[i].name);
    }
    else
    {
      LOGE("uart_manage init %s failed: %d\r\n", uart_manage_table[i].name, init_result[i]);
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  (void)uart_manage_reset_dma_send(huart);
  (void)uart_manage_enable_dma_recv(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)uart_manage_send_completed_hook(huart);

  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int i;
  for (i = 0; i < numberHandlers; i++)
  {
    if (mHandlers[i]->port == huart)
    {
      xTaskNotifyFromISR(mHandlers[i]->myTaskModbusAHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
      break;
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int i;
  for (i = 0; i < numberHandlers; i++)
  {
    if (mHandlers[i]->port == huart)
    {
      if (mHandlers[i]->xTypeHW == USART_HW)
      {
        RingAdd(&mHandlers[i]->xBufferRX, mHandlers[i]->dataRX);
        HAL_UART_Receive_IT(mHandlers[i]->port, &mHandlers[i]->dataRX, 1);
        xTimerResetFromISR(mHandlers[i]->xTimerT35, &xHigherPriorityTaskWoken);
      }
      break;
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

  for (int i = 0; i < numberHandlers; i++)
  {
    if (mHandlers[i]->port == huart)
    {
      if (mHandlers[i]->xTypeHW == USART_HW_DMA)
      {
        while (HAL_UARTEx_ReceiveToIdle_DMA(mHandlers[i]->port, mHandlers[i]->xBufferRX.uxBuffer, MAX_BUFFER) != HAL_OK)
        {
          HAL_UART_DMAStop(mHandlers[i]->port);
        }
        __HAL_DMA_DISABLE_IT(mHandlers[i]->port->hdmarx, DMA_IT_HT);
      }

      break;
    }
  }
}
