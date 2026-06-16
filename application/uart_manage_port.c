/*
 * 文件用途：Echo - 直接回调直通模式（Callback-Direct Mode）
 *
 * 如何使用：
 * 1) 将本文件重命名为 uart_manage_port.c 并加入工程编译。
 * 2) 在系统初始化阶段调用 init_uart_manage()。
 * 3) 保证 HAL_UARTEx_RxEventCallback/HAL_UART_TxCpltCallback 未被其他文件重复定义覆盖。
 *
 * 预期效果：
 * - 收到串口数据后，uart_manage 在 RxEvent 中直接调用 echo_callback。
 * - echo_callback 内立即调用 uart_manage_dma_send_by_name("echo", ...) 回发。
 * - 上位机向 USART1 发送 "abc"，应快速收到 "abc" 回显。
 */
/* port.c */
#include <string.h>

#include "uart_manage.h"
#include "Modbus.h"
#include "app_4g.h"
#include "at_protocol_handler.h"
extern EventGroupHandle_t eg; // 初始化事件组为NULL

#ifndef UART_MANAGE_RECV_RING_STATS_ENABLE
#define UART_MANAGE_RECV_RING_STATS_ENABLE 0U
#endif

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
static uint8_t uart1_process_buff[512U * 4U] DMA_BUFFER;

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
static uint8_t uart3_recv_buff[256U] DMA_BUFFER;
static uint8_t uart3_send_buff[256U] DMA_BUFFER;
static uint8_t uart3_send_fifo_buff[256U] DMA_BUFFER;
static uint8_t uart3_process_buff[256U * 4U] DMA_BUFFER;

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
static uint8_t uart5_recv_buff[256U] DMA_BUFFER;
static uint8_t uart5_send_buff[256U] DMA_BUFFER;
static uint8_t uart5_send_fifo_buff[256U] DMA_BUFFER;
static uint8_t uart5_process_buff[256U * 4U] DMA_BUFFER;

static int32_t shell_at_reply_send(uint8_t *buf, uint16_t len)
{
  return uart_manage_dma_send_by_name("shell", buf, len);
}

static int32_t uart_4g_at_reply_send(uint8_t *buf, uint16_t len)
{
  uint8_t reply[258U];

  if ((buf == NULL) || (len == 0U) || (len > (sizeof(reply) - 2U)))
  {
    return -1;
  }

  reply[0] = '1';
  reply[1] = ',';
  (void)memcpy(&reply[2], buf, len);

  return uart_manage_dma_send_by_name("4g", reply, (uint16_t)(len + 2U));
}

// 【Shell (UART5) 收到数据 -> 转发给 4G (UART1)】
static uint32_t shell_recv_callback(uint8_t *buf, uint16_t len)
{
  printf("\r\n[DEBUG] Shell recv %d : %.*s\r\n", len, len, buf);

  //  craner 指令 ( OTA 指令.....)
  if (craner_at_handler(buf, len, shell_at_reply_send) != AT_PREFIX_NOT_MATCH) // 匹配成功
  {
    return 0U; // 是 craner 的内部 AT 指令，拦截结束
  }

  // 识别本地敲的真实 AT 指令
  if (len >= 2 && (buf[0] == 'A' || buf[0] == 'a') && (buf[1] == 'T' || buf[1] == 't'))
  {
    usr_at_handler(buf, len);
    return 0U;
  }

  // 转发给 4G 模组
  (void)uart_manage_dma_send_by_name("4g", buf, len);

  return 0U;
}
extern ParserCtx_t g_parser_ctx;                                 // 4G数据解析上下文
extern void parser_process_byte(ParserCtx_t *ctx, uint8_t byte); // 4G数据逐字节解析函数
static uint32_t uart_4g_recv_callback(uint8_t *buf, uint16_t len)
{
  if ((buf == NULL) || (len == 0U))
    return 0U;

  // 1. 如果确认是前缀 (长度>=2且带有',')
  if ((len >= 2U) && (buf[1] == ','))
  {
    if (buf[0] == '1')
    {
      if (craner_at_handler(&buf[2], len - 2U, uart_4g_at_reply_send) != AT_PREFIX_NOT_MATCH)
      {
        return 0U;
      }

      usr_at_handler(&buf[2], len - 2);
      return 0U;
    }
    else if (buf[0] == '3' || buf[0] == '4')
    {
      // 剥离前缀，把有效负荷扔进 RingBuffer
      uart_manage_write_to_recv_ring(uart_manage_get_obj_by_name("4g"), &buf[2], len - 2);
      return 0U;
    }
  }

  // 2. 防拆包兜底：如果没有特征前缀，但包含了 0xA5 (你的帧头)，
  // 说明很可能是被截断的后半截数据包，或者是紧接着的纯净指令，全部扔进 RingBuffer 让状态机处理！
  // 注意：如果有纯文本AT回复，可能会误入，但你的 CRC 状态机会自动忽略它们。
  uart_manage_write_to_recv_ring(uart_manage_get_obj_by_name("4g"), buf, len);
  // 4G 模组自身的响应 打印到本地 Shell [4G RAW] OK / ERROR
  static const uint8_t prefix[] = "[4G RAW] ";
  const uint16_t prefix_len = (uint16_t)(sizeof(prefix) - 1U);
  (void)uart_manage_dma_send_by_name("shell", (uint8_t *)prefix, prefix_len);
  (void)uart_manage_dma_send_by_name("shell", buf, len);
  // 是否打包发回给 MQTT 上位机
  // app_4G_send_ack(ACK_ID_SYSTEM_STATUS, buf, len);
  return 0U;
}

const uart_inferface_t uart_manage_table[] = {
    {
        .name = "shell",
        .uart_h = &huart5,
        .dma_h = &hdma_uart5_rx,
        .recv_buffer = uart5_recv_buff,
        .recv_buffer_size = sizeof(uart5_recv_buff),
        .process_buffer = uart5_process_buff,
        .process_buffer_size = sizeof(uart5_process_buff),
        .recv_callback = shell_recv_callback,
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
        .recv_callback = NULL, // ring_task_mode
        .send_buffer = uart3_send_buff,
        .send_buffer_size = sizeof(uart3_send_buff),
        .send_fifo_buffer = uart3_send_fifo_buff,
        .send_fifo_size = sizeof(uart3_send_fifo_buff),
        .send_callback = NULL,
    },
};

#define uart_manage_table_size \
  ((uint16_t)(sizeof(uart_manage_table) / sizeof(uart_manage_table[0])))

void init_uart_manage(void)
{
  (void)uart_manage_init_table(uart_manage_table, uart_manage_table_size);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_manage_reset_dma_send(huart);
  (void)uart_manage_enable_dma_recv(huart);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_manage_send_completed_hook(huart);

  /* Modbus RTU TX callback BEGIN */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  int i;
  for (i = 0; i < numberHandlers; i++)
  {
    if (mHandlers[i]->port == huart)
    {
      // notify the end of TX
      xTaskNotifyFromISR(mHandlers[i]->myTaskModbusAHandle, 0, eNoAction, &xHigherPriorityTaskWoken);
      break;
    }
  }
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* Modbus RTU RX callback BEGIN */
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
  if (huart == &huart1)
  {
    // printf("\r\n[DEBUG] UART1 RxEvent size: %d\r\n", size);
  }

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
        __HAL_DMA_DISABLE_IT(mHandlers[i]->port->hdmarx, DMA_IT_HT); // we don't need half-transfer interrupt
      }

      break;
    }
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  (void)huart;
}
