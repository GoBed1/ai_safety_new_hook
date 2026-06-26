#ifndef UART_MANAGE_PORT_H
#define UART_MANAGE_PORT_H

#include <stdint.h>

#define UART_MANAGE_ENABLE_DMA_CACHE 0U

int32_t shell_inform_send(uint8_t *buf, uint16_t len);
int32_t mqtt_inform_send(uint8_t *buf, uint16_t len);
int32_t uart_shell_recv_callback(uint8_t *buf, uint16_t len);
int32_t uart_4g_recv_callback(uint8_t *buf, uint16_t len);
void init_uart_manage(void);

#endif // UART_MANAGE_PORT_H
