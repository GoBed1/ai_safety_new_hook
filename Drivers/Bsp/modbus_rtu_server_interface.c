#include "modbus_rtu_server_interface.h"
#include "FreeRTOS.h"
#include "task.h"

static uint16_t modbus_registers[REGS_TOTAL_NUM] = {0};
static uint16_t modbus_input_registers[REGS_TOTAL_NUM] = {0};

uint16_t MB_Reg_Get(uint16_t index) {
    if (index >= REGS_TOTAL_NUM) return 0;
    uint16_t val;
    taskENTER_CRITICAL(); // FreeRTOS 临界区保护
    val = modbus_registers[index];
    taskEXIT_CRITICAL();
    return val;
}

void MB_Reg_Set(uint16_t index, uint16_t value) {
    if (index >= REGS_TOTAL_NUM) return;
    taskENTER_CRITICAL();
    modbus_registers[index] = value;
    taskEXIT_CRITICAL();
}

void MB_Reg_SetBits(uint16_t index, uint16_t mask, uint16_t value) {
    if (index >= REGS_TOTAL_NUM) return;
    taskENTER_CRITICAL();
    modbus_registers[index] = (modbus_registers[index] & ~mask) | (value & mask);
    taskEXIT_CRITICAL();
}

uint16_t* MB_Reg_GetPointer(void) { return modbus_registers; }
uint16_t* MB_InputReg_GetPointer(void) { return modbus_input_registers; }

// ========== Modbus 主机初始化接口 ==========
 void init_modbus_master(modbusHandler_t *handler, UART_HandleTypeDef *huart, uint16_t *buf, uint16_t buf_len)
{
    // 1. 清空传入的缓存数组
    memset(buf, 0, buf_len * sizeof(uint16_t));

    // 2. 配置 Modbus 主机参数
    handler->uModbusType = MB_MASTER;
    handler->port = huart;             // 传入的串口句柄
    handler->u8id = 0;
    handler->u16timeOut = 500;
    handler->EN_Port = NULL;
    handler->EN_Pin = 0;
    handler->u16regs = buf;            // 传入的缓存指针
    handler->u16regsize = buf_len;     // 传入的缓存长度
    handler->xTypeHW = USART_HW;
    
    // 3. 初始化并启动协议栈
    ModbusInit(handler);
    ModbusStart(handler);

}
