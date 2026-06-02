#ifndef MODBUS_RTU_SERVER_INTERFACE_H
#define MODBUS_RTU_SERVER_INTERFACE_H

#include <stdint.h>
#include "system_def.h"
#include "Modbus.h"
// 提供给外部各业务 APP 的寄存器安全访问接口
uint16_t MB_Reg_Get(uint16_t index);
void MB_Reg_Set(uint16_t index, uint16_t value);
void MB_Reg_SetBits(uint16_t index, uint16_t mask, uint16_t value);

// 仅供 Modbus Slave 底层初始化时绑定内存使用
uint16_t* MB_Reg_GetPointer(void);
uint16_t* MB_InputReg_GetPointer(void);
void init_modbus_master(modbusHandler_t *handler, UART_HandleTypeDef *huart, uint16_t *buf, uint16_t buf_len);
void init_modbus_slave(modbusHandler_t *handler, UART_HandleTypeDef *huart, uint8_t slave_id);
#endif
