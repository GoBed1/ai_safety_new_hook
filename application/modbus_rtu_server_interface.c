#include "modbus_rtu_server_interface.h"
#include "FreeRTOS.h"
#include "task.h"

static uint16_t modbus_registers[REGS_TOTAL_NUM] = {0};
static uint16_t modbus_input_registers[REGS_TOTAL_NUM] = {0};

uint16_t MB_Reg_Get(uint16_t index) {
    if (index >= REGS_TOTAL_NUM) return 0;
    uint16_t val;
   uint32_t ms_cpu_sr; // 1. 在本地定义一个变量用来保存中断状态
    
    CRITICAL_SETCION_ENTER(ms_cpu_sr); // 2. 进入临界区（传入变量）
    val = modbus_registers[index];
    CRITICAL_SETCION_EXIT(ms_cpu_sr);
    return val;
}

void MB_Reg_Set(uint16_t index, uint16_t value) {
    if (index >= REGS_TOTAL_NUM) return;
    uint32_t ms_cpu_sr; // 1. 在本地定义一个变量用来保存中断状态
    
    CRITICAL_SETCION_ENTER(ms_cpu_sr);
    modbus_registers[index] = value;
    CRITICAL_SETCION_EXIT(ms_cpu_sr);
}

void MB_Reg_SetBits(uint16_t index, uint16_t mask, uint16_t value) {
    if (index >= REGS_TOTAL_NUM) return;
    uint32_t ms_cpu_sr; // 1. 在本地定义一个变量用来保存中断状态
    
    CRITICAL_SETCION_ENTER(ms_cpu_sr);
    modbus_registers[index] = (modbus_registers[index] & ~mask) | (value & mask);
    CRITICAL_SETCION_EXIT(ms_cpu_sr);
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

void init_modbus_slave(modbusHandler_t *handler, UART_HandleTypeDef *huart, uint8_t slave_id) 
{

    handler->uModbusType = MB_SLAVE;
    handler->u8id = slave_id; 
    handler->port = huart;
    handler->EN_Port = NULL; 
    handler->EN_Pin = 0;
    
    // 绑定受保护的寄存器内存
    handler->u16regs = MB_Reg_GetPointer();            
    handler->u16inputregs = MB_InputReg_GetPointer(); 
    
    handler->u16regsize = REGS_TOTAL_NUM;
    handler->u16timeOut = 1000; 
    handler->xTypeHW = USART_HW;
    
    // 初始化并启动
    ModbusInit(handler);
    ModbusStart(handler);
    
    LOGI("modbus slave (ID:%d) initialized on %p\n", slave_id, huart);
}