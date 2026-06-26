// USART1 application setup
#ifndef UART_MANAGE_H
#define UART_MANAGE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "lwrb.h"
#include "fifo.h"

#include "uart_manage_port.h"

#ifndef UART_MANAGE_ENABLE_DMA_CACHE
#define UART_MANAGE_ENABLE_DMA_CACHE 0U
#endif

typedef int32_t (*interface_send_fn_t)(const uint8_t *buf, uint16_t len);
typedef int32_t (*interface_recv_fn_t)(uint8_t *buf, uint16_t len);

#define UART_MANAGE_MAX_OBJECTS 8U

typedef enum
{
    UART_MANAGE_OK = 0,
    UART_MANAGE_ERR_POINTER_NULL = -1,
    UART_MANAGE_ERR_INVALID_PARAM = -2,
    UART_MANAGE_ERR_NOT_FOUND = -3,
    UART_MANAGE_ERR_NO_SLOT = -4,
    UART_MANAGE_ERR_RECV_START = -5,
    UART_MANAGE_ERR_FIFO = -6,
    UART_MANAGE_ERR_RING = -7,
    UART_MANAGE_ERR_NO_SPACE = -8,
    UART_MANAGE_ERR_SEND = -9
} uart_manage_err_t;

typedef struct uart_interface
{
    char name[50];
    uint8_t idx;
    uint8_t used;
    UART_HandleTypeDef *uart_h;
    DMA_HandleTypeDef *dma_h;

    uint8_t *recv_buffer;
    uint16_t recv_buffer_size;
    uint8_t *process_buffer;
    uint16_t process_buffer_size;
    lwrb_t process_ring_buffer;
    interface_recv_fn_t recv_callback;

    uint8_t *send_buffer;
    uint16_t send_buffer_size;
    fifo_s_t send_fifo;
    uint8_t *send_fifo_buffer;
    uint16_t send_fifo_size;
    uint8_t is_sending;
    interface_send_fn_t send_callback;
} uart_inferface_t;

typedef enum
{
    INTERRUPT_TYPE_UART = 0,
    INTERRUPT_TYPE_DMA_HALF = 1,
    INTERRUPT_TYPE_DMA_ALL = 2
} interrput_type;

uart_inferface_t *uart_manage_get_obj(UART_HandleTypeDef *huart);
uart_inferface_t *uart_manage_get_obj_by_name(const char *name);

int uart_manage_register_interface(uart_inferface_t *m_obj);
int uart_manage_set_recv_callback(UART_HandleTypeDef *huart, interface_recv_fn_t recv_callback);

int uart_manage_enable_dma_recv(UART_HandleTypeDef *huart);
int uart_manage_enable_dma_recv_by_name(const char *name);
int uart_manage_dma_send(UART_HandleTypeDef *huart, const uint8_t *buf, uint16_t len);
int uart_manage_dma_send_by_name(const char *name, const uint8_t *buf, uint16_t len);

int uart_manage_send_completed_hook(UART_HandleTypeDef *huart);
int uart_manage_reset_dma_send(UART_HandleTypeDef *huart);
int uart_manage_write_to_recv_ring(uart_inferface_t *m_obj, uint8_t *buf, uint16_t len);
int uart_manage_recv_idle_hook(uart_inferface_t *m_obj, interrput_type int_type, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif // UART_MANAGE_H
