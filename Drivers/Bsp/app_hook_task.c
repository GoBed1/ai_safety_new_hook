#include "app_hook_task.h"
#include "modbus_rtu_server_interface.h"
#include "app_rfid.h"
#include "app_relay.h"
#include "app_bms_alarm.h"
// ====== 看门狗标志位======
volatile uint8_t g_task_alive_flags = 0;



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
// RFID
osThreadId_t RFID_master_handle;
const osThreadAttr_t RFID_master_attributes = {
    .name = "RFIDMaster",
    .stack_size = 1024 * 4,
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

EventGroupHandle_t eg = NULL; 

void EventGroupCreate_Init(void) {
    if (eg == NULL) {
        eg = xEventGroupCreate();
    }
}

// 供外部访问本机状态的 Modbus 从机初始化
void init_ai_safy_slave(void) {
    static modbusHandler_t modbus_rtu_server;
    extern UART_HandleTypeDef huart7;
    
    MB_Reg_Set(REG_ERROR_CODE, 0x0000); 

    modbus_rtu_server.uModbusType = MB_SLAVE;
    modbus_rtu_server.u8id = FORWARD_SLAVE_ADDR; 
    modbus_rtu_server.port = &huart7;
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
//rfid线程
void RFID_master_thread(void *argument)
{
    TickType_t last_check = xTaskGetTickCount();
    for (;;)
    {
        g_task_alive_flags |= TASK_RFID_ALIVE;
        EventBits_t uxBits = xEventGroupWaitBits(
            eg,            // 事件组
            EVENT_RFID_RX, // 等待这个事件
            pdTRUE,        // 自动清除标志
            pdFALSE,       // 不需要等待所有位
            pdMS_TO_TICKS(500)   // 有看门狗的超时时间
        );
        if ((uxBits & EVENT_RFID_RX) != 0)
        {
            LOGD("Receive rfid: ");
            uint16_t rfid_valid_reg = MB_Reg_Get(REG_RFID_VALID);
            LOGD("modbus_reg[3] = %04X (", rfid_valid_reg);
            for (int i = 15; i >= 0; i--)
            {
                LOGD("%d", (rfid_valid_reg >> i) & 1);
                if (i % 4 == 0 && i != 0)
                    LOGD(" ");
            }
            
            RFID_OnFrame(&RFID_client,
                         RFID_client.Rx_RFID_buf,
                         RFID_client.Rx_RFID_len);

            // 写入Modbus寄存器
            RFID_WriteToModbusRegs(&RFID_client);
        }

        if ((TickType_t)(xTaskGetTickCount() - last_check) >= pdMS_TO_TICKS(5000))
        {
            last_check = xTaskGetTickCount();
            RFID_CheckOffline(&RFID_client);
            RFID_WriteToModbusRegs(&RFID_client);
        }

        osDelay(1000);
    }
}
void ai_safy_master_thread(void *argument)
{
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
// GPS待机线程
void gps_standby_thread(void *argument)
{

    config_gps_app();
    rtc_power_init();
    // run_10_oclock_standby_test();

    for (;;)
    {
        g_task_alive_flags |= TASK_GPS_ALIVE;
        // 每1s轮询一次GPS数据
        // 每1s轮询一次GPS数据
        update_gps_app();
        // // 检测是否进入待机状态
        // run_10_oclock_standby_test();
        // update_gps_time_loop();
        // print_internal_rtc_time();

        rtc_power_schedule_check();

        HAL_GPIO_TogglePin(GPIOD, H_B_LED_Pin);
        // HAL_GPIO_TogglePin(RELAY_1_PIN_GPIO_Port, RELAY_1_PIN_Pin);

        osDelay(1000);
    }
}
// 吊钩系统总初始化入口
void init_app_hook_task() {
    EventGroupCreate_Init();
    init_ai_safy_slave();    // 1. 启动本机的 Modbus 通信服务
    init_uart_manage();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, RFID_client.rx_buf, (uint16_t)sizeof(RFID_client.rx_buf));
    
    // 初始化默认音量
    MB_Reg_Set(CMD_VOLUME, DEFAULT_VOLUME); 
    //声光警报和bms初始化
    init_bms_alarm_module();
    //继电器初始化
    relay_app_init(); 

    
    // 5. 启动 GPS 待机线程 (维持原样，后续再重构)
    // extern osThreadAttr_t gps_standby_attributes;
    // extern void gps_standby_thread(void *argument);
    // osThreadNew(gps_standby_thread, NULL, &gps_standby_attributes);
    RFID_master_handle = osThreadNew(RFID_master_thread, NULL, &RFID_master_attributes);
    ai_safy_master_handle = osThreadNew(ai_safy_master_thread, NULL, &ai_safy_master_attributes);
    relay_heartbeat_handle = osThreadNew(relay_heartbeat_thread, NULL, &relay_heartbeat_attributes);
    gps_standby_handle = osThreadNew(gps_standby_thread, NULL, &gps_standby_attributes);

}
