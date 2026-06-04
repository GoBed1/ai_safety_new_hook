#define MODULE_LOG_ENABLE LOG_SWITCH_CAR_TASK
#include "app_hook_task.h"
#include "modbus_rtu_server_interface.h"
#include "app_relay.h"
#include "app_bms_alarm.h"
#include "app_sys_supervisor.h"
#include "gps_app.h"
#include "app_4G.h"
// ====== 看门狗标志位======
volatile uint8_t g_task_alive_flags = 0;
extern UART_HandleTypeDef huart8;
static modbusHandler_t modbus_rtu_server;

osThreadId_t ai_safy_slave_handle;
const osThreadAttr_t ai_safy_slave_attributes = {
    .name = "AISafySlave",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

osThreadId_t ai_safy_master_handle;
const osThreadAttr_t ai_safy_master_attributes = {
    .name = "AISafyMaster",
    .stack_size = 1024 * 6,
    .priority = (osPriority_t)osPriorityNormal1,
};
// gps待机线程
osThreadId_t gps_standby_handle;
const osThreadAttr_t gps_standby_attributes = {
    .name = "GPSStandby",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal1,
};
// 心跳检测任务
osThreadId_t relay_heartbeat_handle;
const osThreadAttr_t relay_heartbeat_attributes = {
    .name = "RelayHeartbeat",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
// 系统监控线程
osThreadId_t sys_supervisor_handle;
const osThreadAttr_t sys_supervisor_attributes = {
    .name = "SysSupervisor",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal, 
};
// 工作状态判定线程
osThreadId_t work_mode_handle;
const osThreadAttr_t work_mode_attributes = {
    .name = "WorkModeTask",
    .stack_size = 1024 * 2,
    .priority = (osPriority_t)osPriorityNormal,
};
EventGroupHandle_t eg = NULL; 

void EventGroupCreate_Init(void) {
    if (eg == NULL) {
        eg = xEventGroupCreate();
    }
}

// 供外部访问本机状态的 Modbus 从机初始化
void init_ai_safy_slave(void) {
    static modbusHandler_t modbus_rtu_server;
    extern UART_HandleTypeDef huart8;
    

    modbus_rtu_server.uModbusType = MB_SLAVE;
    modbus_rtu_server.u8id = FORWARD_SLAVE_ADDR; 
    modbus_rtu_server.port = &huart8;
    modbus_rtu_server.EN_Port = NULL; 
    modbus_rtu_server.EN_Pin = 0;
    
    // 从底层接口获取受保护的数组指针交给 Modbus 协议栈
    modbus_rtu_server.u16regs = MB_Reg_GetPointer();            
    modbus_rtu_server.u16inputregs = MB_InputReg_GetPointer(); 
    
    modbus_rtu_server.u16regsize = REGS_TOTAL_NUM;
    modbus_rtu_server.u16timeOut = 1000; 
    modbus_rtu_server.xTypeHW = USART_HW;
    
    ModbusInit(&modbus_rtu_server);
    ModbusStart(&modbus_rtu_server);
}
void ai_safy_master_thread(void *argument)
{
    power_on_self_test(); // 电源上电自检
    for (;;)
    {
        g_task_alive_flags |= TASK_AI_SAFY_ALIVE; // 看门狗打卡
        
        modbus_alarm_handle();  // 处理声光模块
        modbus_bms_handle();    // 处理 BMS 模块

        osDelay(100); 
    }
}

// 继电器心跳专用线程
void relay_heartbeat_thread(void *argument)
{
    for (;;)
    {
        g_task_alive_flags |= TASK_RELAY_ALIVE; // 看门狗打卡
        process_relay_logic();                  
        osDelay(100);                           
    }
}
// GPS/待机线程
void gps_standby_thread(void *argument)
{
    //gps / 待机初始化
    gps_rtc_app_init(); 
    for (;;)
    {
        g_task_alive_flags |= TASK_GPS_ALIVE;
        process_gps_logic();  

        osDelay(1000); 
    }
}

void sys_supervisor_thread(void *argument)
{
   for (;;)
    {
        sys_supervisor_process();

        osDelay(200); 
    }
}
// 工作状态判定线程
void work_mode_thread(void *argument)
{
    for (;;)
    {
        work_mode_logic();
        osDelay(100); 
    }
}
// 吊钩系统总初始化入口
void init_app_hook_task() {
    
    EventGroupCreate_Init();
    //启动本机的 Modbus 通信服务
    init_modbus_slave(&modbus_rtu_server, &huart7, FORWARD_SLAVE_ADDR);  
    //初始化串口管理模块
    init_uart_manage();
    //声光警报和bms,主机初始化
    init_bms_alarm_module();
    //继电器初始化
    relay_app_init(); 
    // 4G模块初始化
    app_4G_init();
    // 初始化默认音量
    MB_Reg_Set(CMD_VOLUME, DEFAULT_VOLUME); 
    //初始化错误码寄存器为0x0000
    MB_Reg_Set(REG_ERROR_CODE, 0x0000); 
    // 初始化心跳使能寄存器为1（默认开启心跳）
    MB_Reg_Set(HEARTBEAT_ENABLE, 1);
    ai_safy_master_handle = osThreadNew(ai_safy_master_thread, NULL, &ai_safy_master_attributes);
    relay_heartbeat_handle = osThreadNew(relay_heartbeat_thread, NULL, &relay_heartbeat_attributes);
    gps_standby_handle = osThreadNew(gps_standby_thread, NULL, &gps_standby_attributes);
    sys_supervisor_handle = osThreadNew(sys_supervisor_thread, NULL, &sys_supervisor_attributes);
    // work_mode_handle = osThreadNew(work_mode_thread, NULL, &work_mode_attributes);
}
