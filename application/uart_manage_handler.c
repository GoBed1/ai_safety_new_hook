#include "uart_manage_port.h"

#include "at_protocol_handler.h"
#include "uart_manage.h"

int32_t shell_inform_send(uint8_t *buf, uint16_t len)
{
  (void)uart_manage_dma_send_by_name("shell", buf, len);
  return 0;
}

int32_t mqtt_inform_send(uint8_t *buf, uint16_t len)
{
  static const uint8_t prefix[] = "1,";
  const uint16_t prefix_len = (uint16_t)(sizeof(prefix) - 1U);

  (void)uart_manage_dma_send_by_name("4g", prefix, prefix_len);
  (void)uart_manage_dma_send_by_name("4g", buf, len);
  return 0;
}
int32_t uart_shell_recv_callback(uint8_t *buf, uint16_t len)
{
  int32_t ret;

  ret = craner_at_handler(buf, len, shell_inform_send);
  if (ret != AT_PREFIX_NOT_MATCH)
  {
    return (ret < 0) ? ret : UART_MANAGE_OK;
  }

  ret = usr_at_handler(buf, len);
  if (ret != AT_PREFIX_NOT_MATCH)
  {
    return (ret < 0) ? ret : UART_MANAGE_OK;
  }

  static const uint8_t prefix[] = "[ERR]SHELL:";
  const uint16_t prefix_len = (uint16_t)(sizeof(prefix) - 1U);
  (void)uart_manage_dma_send_by_name("shell", prefix, prefix_len);
  (void)uart_manage_dma_send_by_name("shell", buf, len);
  return UART_MANAGE_OK;
}

int32_t uart_4g_recv_callback(uint8_t *buf, uint16_t len)
{
  if (buf == NULL)
  {
    return UART_MANAGE_ERR_POINTER_NULL;
  }

  if (len == 0U)
  {
    return UART_MANAGE_ERR_INVALID_PARAM;
  }

  if ((len >= 2U) && (buf[1] == ','))
  {
    int32_t ret;

    if (buf[0] == '1')
    {
      ret = craner_at_handler(&buf[2], (uint16_t)(len - 2U), mqtt_inform_send);
      if (ret != AT_PREFIX_NOT_MATCH)
      {
        return (ret < 0) ? ret : UART_MANAGE_OK;
      }

      ret = usr_at_handler(&buf[2], (uint16_t)(len - 2U));
      if (ret != AT_PREFIX_NOT_MATCH)
      {
        return (ret < 0) ? ret : UART_MANAGE_OK;
      }
      return UART_MANAGE_OK;
    }

    if (buf[0] == '2')
    {
      return UART_MANAGE_OK;
    }

    if ((buf[0] == '3') || (buf[0] == '4'))
    {
      (void)uart_manage_write_to_recv_ring(uart_manage_get_obj_by_name("4g"), &buf[2], (uint16_t)(len - 2U));
      return UART_MANAGE_OK;
    }
  }

  // (void)uart_manage_write_to_recv_ring(uart_manage_get_obj_by_name("4g"), buf, len);

  static const uint8_t prefix[] = "[ERR]4G:";
  const uint16_t prefix_len = (uint16_t)(sizeof(prefix) - 1U);
  (void)uart_manage_dma_send_by_name("shell", prefix, prefix_len);
  (void)uart_manage_dma_send_by_name("shell", buf, len);

  return UART_MANAGE_OK;
}
