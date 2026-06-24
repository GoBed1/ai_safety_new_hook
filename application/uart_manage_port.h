#ifndef UART_MANAGE_PORT_H
#define UART_MANAGE_PORT_H

#define UART_MANAGE_RECV_RING_STATS_ENABLE 1U

int32_t shell_inform_send(uint8_t *buf, uint16_t len);
int32_t mqtt_inform_send(uint8_t *buf, uint16_t len);
void init_uart_manage(void);

#endif // UART_MANAGE_PORT_H
