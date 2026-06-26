#include "uart_manage.h"
#include "stm32h743xx.h"

static uart_inferface_t uart_manage[UART_MANAGE_MAX_OBJECTS] = {0};

static inline void dma_clean_cache_by_addr(const void *addr, uint32_t len)
{
#if (UART_MANAGE_ENABLE_DMA_CACHE == 1)
  uintptr_t start = (uintptr_t)addr & ~(uintptr_t)31U;
  uintptr_t end   = ((uintptr_t)addr + len + 31U) & ~(uintptr_t)31U;
  SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)addr;
  (void)len;
#endif
}

static inline void dma_invalidate_cache_by_addr(const void *addr, uint32_t len)
{
#if (UART_MANAGE_ENABLE_DMA_CACHE == 1)
  uintptr_t start = (uintptr_t)addr & ~(uintptr_t)31U;
  uintptr_t end   = ((uintptr_t)addr + len + 31U) & ~(uintptr_t)31U;
  SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
#else
  (void)addr;
  (void)len;
#endif
}

static int uart_manage_find_slot_by_huart(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  for (uint32_t i = 0U; i < UART_MANAGE_MAX_OBJECTS; ++i)
  {
    if ((uart_manage[i].used != 0U) && (uart_manage[i].uart_h == huart))
    {
      return (int)i;
    }
  }

  return UART_MANAGE_ERR_NOT_FOUND;
}

static int uart_manage_find_free_slot(void)
{
  for (uint32_t i = 0U; i < UART_MANAGE_MAX_OBJECTS; ++i)
  {
    if (uart_manage[i].used == 0U)
    {
      return (int)i;
    }
  }

  return UART_MANAGE_ERR_NO_SLOT;
}

uart_inferface_t *uart_manage_get_obj(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return NULL;
  }

  for (uint32_t i = 0U; i < UART_MANAGE_MAX_OBJECTS; ++i)
  {
    if ((uart_manage[i].used != 0U) && (uart_manage[i].uart_h == huart))
    {
      return &uart_manage[i];
    }
  }

  return NULL;
}

uart_inferface_t *uart_manage_get_obj_by_name(const char *name)
{
  if (name == NULL)
  {
    return NULL;
  }

  for (uint32_t i = 0U; i < UART_MANAGE_MAX_OBJECTS; ++i)
  {
    if ((uart_manage[i].used != 0U) && (strcmp(uart_manage[i].name, name) == 0))
    {
      return &uart_manage[i];
    }
  }

  return NULL;
}

int uart_manage_register_interface(uart_inferface_t *m_obj)
{
  if (m_obj == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (m_obj->uart_h == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (((m_obj->send_fifo_buffer == NULL) && (m_obj->send_fifo_size > 0U)) ||
      ((m_obj->process_buffer == NULL) && (m_obj->process_buffer_size > 0U)))
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (((m_obj->send_fifo_buffer != NULL) && (m_obj->send_fifo_size == 0U)) ||
      ((m_obj->process_buffer != NULL) && (m_obj->process_buffer_size == 0U)))
  {
    return UART_MANAGE_ERR_INVALID_PARAM;
  }

  int slot = uart_manage_find_slot_by_huart(m_obj->uart_h);
  if (slot < 0)
  {
    slot = uart_manage_find_free_slot();
  }

  if (slot < 0)
  {
    return slot;
  }

  uart_manage[slot] = *m_obj;
  uart_manage[slot].used = 1U;
  uart_manage[slot].idx = (uint8_t)slot;
  if (uart_manage[slot].send_fifo_buffer != NULL && uart_manage[slot].send_fifo_size > 0U)
  {
    if (fifo_s_init(&uart_manage[slot].send_fifo, uart_manage[slot].send_fifo_buffer, uart_manage[slot].send_fifo_size) != 0)
    {
      memset(&uart_manage[slot], 0, sizeof(uart_manage[slot]));
      return UART_MANAGE_ERR_FIFO;
    }
  }
  uart_manage[slot].is_sending = 0U;

  if ((uart_manage[slot].process_buffer != NULL) && (uart_manage[slot].process_buffer_size > 0U))
  {
    if (lwrb_init(&uart_manage[slot].process_ring_buffer, uart_manage[slot].process_buffer, uart_manage[slot].process_buffer_size) == 0U)
    {
      memset(&uart_manage[slot], 0, sizeof(uart_manage[slot]));
      return UART_MANAGE_ERR_RING;
    }
  }
  
  return UART_MANAGE_OK;
}

int uart_manage_set_recv_callback(UART_HandleTypeDef *huart, interface_recv_fn_t recv_callback)
{
  if (huart == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  uart_inferface_t *obj = uart_manage_get_obj(huart);
  if (obj == NULL)
  {
    return UART_MANAGE_ERR_NOT_FOUND;
  }

  obj->recv_callback = recv_callback;
  return UART_MANAGE_OK;
}

int uart_manage_enable_dma_recv(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  uart_inferface_t *m_obj = uart_manage_get_obj(huart);

  if (m_obj == NULL)
  {
    return UART_MANAGE_ERR_NOT_FOUND;
  }

  if (m_obj->dma_h == NULL || m_obj->uart_h == NULL || m_obj->recv_buffer == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (m_obj->recv_buffer_size == 0U)
  {
    return UART_MANAGE_ERR_INVALID_PARAM;
  }

  m_obj->uart_h->Instance->ICR = USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_IDLECF;
  (void)m_obj->uart_h->Instance->RDR;

  HAL_StatusTypeDef st = HAL_UARTEx_ReceiveToIdle_DMA(m_obj->uart_h, m_obj->recv_buffer, m_obj->recv_buffer_size);
  if (st == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(m_obj->dma_h, DMA_IT_HT);
    return UART_MANAGE_OK;
  }

  (void)HAL_UART_DMAStop(m_obj->uart_h);
  m_obj->uart_h->Instance->ICR = USART_ICR_FECF | USART_ICR_ORECF | USART_ICR_NECF | USART_ICR_PECF | USART_ICR_IDLECF;
  (void)m_obj->uart_h->Instance->RDR;
  st = HAL_UARTEx_ReceiveToIdle_DMA(m_obj->uart_h, m_obj->recv_buffer, m_obj->recv_buffer_size);
  if (st == HAL_OK)
  {
    __HAL_DMA_DISABLE_IT(m_obj->dma_h, DMA_IT_HT);
    return UART_MANAGE_OK;
  }else{
    return UART_MANAGE_ERR_RECV_START;
  }
}

int uart_manage_enable_dma_recv_by_name(const char *name)
{
  if (name == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  uart_inferface_t *m_obj = uart_manage_get_obj_by_name(name);

  if (m_obj == NULL)
  {
    return UART_MANAGE_ERR_NOT_FOUND;
  }

  return uart_manage_enable_dma_recv(m_obj->uart_h);
}

static int uart_manage_dma_send_impl(uart_inferface_t *m_obj, const uint8_t *buf, uint16_t len)
{
  if (m_obj == NULL)
  {
    return UART_MANAGE_ERR_NOT_FOUND;
  }

  if (buf == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (len == 0U)
  {
    return UART_MANAGE_ERR_INVALID_PARAM;
  }

  if ((m_obj->uart_h == NULL) || (m_obj->send_buffer == NULL))
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (m_obj->send_buffer_size == 0U)
  {
    return UART_MANAGE_ERR_INVALID_PARAM;
  }

  uint16_t to_send_len = 0U;
  uint16_t to_tx_fifo_len = 0U;

  if ((m_obj->is_sending != 0U) && (m_obj->uart_h->gState != HAL_UART_STATE_BUSY_TX))
  {
    m_obj->is_sending = 0U;
  }

  if (m_obj->is_sending == 0U)
  {
    uint32_t max_len = (uint32_t)m_obj->send_buffer_size + (uint32_t)m_obj->send_fifo_size;
    if ((uint32_t)len > max_len)
    {
      len = (uint16_t)max_len;
    }

    if (len <= m_obj->send_buffer_size)
    {
      to_send_len = len;
      to_tx_fifo_len = 0;
    }
    else
    {
      to_send_len = m_obj->send_buffer_size;
      to_tx_fifo_len = len - m_obj->send_buffer_size;
    }
  }
  else
  {
    if (len > m_obj->send_fifo_size)
    {
      len = m_obj->send_fifo_size;
    }

    if (len > 0U)
    {
      to_send_len = 0;
      to_tx_fifo_len = len;
    }
  }

  if (len == 0U)
  {
    return UART_MANAGE_ERR_NO_SPACE;
  }

  if ((to_tx_fifo_len > 0U) && (m_obj->send_fifo_buffer == NULL))
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (to_send_len > 0)
  {
    memcpy(m_obj->send_buffer, buf, to_send_len);
    dma_clean_cache_by_addr(m_obj->send_buffer, to_send_len);
    m_obj->is_sending = 1U;
    if (HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->send_buffer, to_send_len) != HAL_OK)
    {
      m_obj->is_sending = 0U;
      return UART_MANAGE_ERR_SEND;
    }
  }
  if (to_tx_fifo_len > 0)
  {
    int put_len;
    put_len = fifo_s_puts(&m_obj->send_fifo, (char *)(buf) + to_send_len, to_tx_fifo_len);
    if (put_len != (int)to_tx_fifo_len)
    {
      return UART_MANAGE_ERR_FIFO;
    }
  }
  return UART_MANAGE_OK;
}

int uart_manage_dma_send(UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t len)
{
  if (huart == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  uart_inferface_t *m_obj = uart_manage_get_obj(huart);
  return uart_manage_dma_send_impl(m_obj, buf, len);
}

int uart_manage_dma_send_by_name(const char *name, const uint8_t *buf, uint16_t len)
{
  if (name == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  uart_inferface_t *m_obj = uart_manage_get_obj_by_name(name);
  return uart_manage_dma_send_impl(m_obj, buf, len);
}

int uart_manage_send_completed_hook(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  uart_inferface_t *m_obj = uart_manage_get_obj(huart);

    if(m_obj == NULL)
    {
        return UART_MANAGE_ERR_NOT_FOUND;
    }

    if ((m_obj->uart_h == NULL) || (m_obj->send_buffer == NULL))
    {
      m_obj->is_sending = 0U;
      return UART_MANAGE_ERR_POINTER_NULL;
    }

    if (m_obj->send_buffer_size == 0U)
    {
      m_obj->is_sending = 0U;
      return UART_MANAGE_ERR_INVALID_PARAM;
    }

    uint16_t fifo_data_num = 0;
    uint16_t send_num = 0;

    fifo_data_num = m_obj->send_fifo.used_num;

    if (fifo_data_num != 0)
    {
      if (fifo_data_num < m_obj->send_buffer_size)
        {
            send_num = fifo_data_num;
        }
        else
        {
        send_num = m_obj->send_buffer_size;
        }
      fifo_s_gets(&m_obj->send_fifo, (char *)m_obj->send_buffer, send_num);
      dma_clean_cache_by_addr(m_obj->send_buffer, send_num);
      if (HAL_UART_Transmit_DMA(m_obj->uart_h, m_obj->send_buffer, send_num) == HAL_OK)
      {
        m_obj->is_sending = 1U;
      }
      else
      {
        m_obj->is_sending = 0U;
        return UART_MANAGE_ERR_SEND;
      }
    }
    else
    {
      m_obj->is_sending = 0U;
    }
    return UART_MANAGE_OK;
}

int uart_manage_reset_dma_send(UART_HandleTypeDef *huart)
{
  if (huart == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  uart_inferface_t *m_obj = uart_manage_get_obj(huart);

  if (m_obj == NULL)
  {
    return UART_MANAGE_ERR_NOT_FOUND;
  }

  m_obj->is_sending = 0U;
  return UART_MANAGE_OK;
}

int uart_manage_write_to_recv_ring(uart_inferface_t *m_obj, uint8_t *buf, uint16_t len)
{
  if (m_obj == NULL || buf == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (len == 0U)
  {
    return UART_MANAGE_ERR_INVALID_PARAM;
  }

  if (m_obj->process_buffer == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (m_obj->process_buffer_size == 0U)
  {
    return UART_MANAGE_ERR_INVALID_PARAM;
  }
  
  lwrb_sz_t to_write_len = (lwrb_sz_t)len;
  lwrb_sz_t free_len = lwrb_get_free(&m_obj->process_ring_buffer);
  if (to_write_len > free_len)
  {
    to_write_len = free_len;
  }

  if (to_write_len == 0U)
  {
    return UART_MANAGE_ERR_NO_SPACE;
  }

  if ((to_write_len > 0U) && (lwrb_write(&m_obj->process_ring_buffer, buf, to_write_len) != to_write_len))
  {
    return UART_MANAGE_ERR_RING;
  }

  return (int)to_write_len;
}

int uart_manage_recv_idle_hook(uart_inferface_t *m_obj, interrput_type int_type, uint16_t size)
{
  
  if (m_obj == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (m_obj->recv_buffer == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if ((size == 0U) || (size > m_obj->recv_buffer_size))
  {
    return UART_MANAGE_ERR_INVALID_PARAM;
  }
  
  (void)int_type;

  dma_invalidate_cache_by_addr(m_obj->recv_buffer, size);
  
  if(m_obj->recv_callback != NULL)
  {
    return m_obj->recv_callback(m_obj->recv_buffer, size);
  }

  return uart_manage_write_to_recv_ring(m_obj, m_obj->recv_buffer, size);
}
