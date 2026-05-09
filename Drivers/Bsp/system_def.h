#ifndef BOARD_SYSTEM_DEFINE_H
#define BOARD_SYSTEM_DEFINE_H

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include "main.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* boolean type definitions */
#ifndef TRUE
    #define TRUE                         1               /**< boolean true  */
#endif

#ifndef FALSE
    #define FALSE                        0               /**< boolean fails */
#endif

#ifndef ENABLE
    #define ENABLE                       1
#endif

#ifndef DISABLE
    #define DISABLE                      0
#endif

#ifndef RAD_TO_DEG
    #define RAD_TO_DEG 57.29f
#endif

#define INT_STATE              uint32_t
#define MASTER_INT_STATE_GET() __get_PRIMASK()
#define MASTER_INT_ENABLE()    do{__enable_irq();  }while(0)
#define MASTER_INT_DISABLE()   do{__disable_irq(); }while(0)
#define MASTER_INT_RESTORE(x)  do{__set_PRIMASK(x);}while(0)

#define CRITICAL_SETCION_ENTER()                      \
    do                                                \
    {                                                 \
        INT_STATE cpu_state = MASTER_INT_STATE_GET(); \
        MASTER_INT_DISABLE();

#define CRITICAL_SETCION_EXIT()        \
        MASTER_INT_RESTORE(cpu_state); \
    }                                  \
    while (0)

#define var_cpu_sr() register unsigned long cpu_sr

#define enter_critical()      \
  do                          \
  {                           \
    cpu_sr = __get_PRIMASK(); \
    __disable_irq();          \
  } while (0)

#define exit_critical()    \
  do                       \
  {                        \
    __set_PRIMASK(cpu_sr); \
  } while (0)

#define MUTEX_DECLARE(mutex) unsigned long mutex
#define MUTEX_INIT(mutex)    do{mutex = 0;}while(0)
#define MUTEX_LOCK(mutex)    do{__disable_irq();}while(0)
#define MUTEX_UNLOCK(mutex)  do{__enable_irq();}while(0)

#define device_assert(EX)\
  if (!(EX))           \
  {                    \
    log_assert("fail! %s %s %s", #EX, __FILE__, __LINE__);   \
    while(1);          \
  }
/* ========================================================================= */
/* 1. 基础状态与日志定义                                                     */
/* ========================================================================= */
#define LOGD(...)                                      //printf("[DEBUG] " __VA_ARGS__)
#define LOGI(...)                                      printf("[INFO]  " __VA_ARGS__)
#define LOGE(...)                                      printf("[ERROR] " __VA_ARGS__)

/* ========================================================================= */
/* 2. 系统调度、看门狗与阈值配置 (System & Thresholds)                               */
/* ========================================================================= */
#define TIMEZONE_OFFSET_BEIJING                        8       // 北京时间偏移量 (UTC+8)
#define HEARTBEAT_TIMEOUT_MS                           60000   // 继电器心跳超时时间(60s)
#define MODBUS_WAIT_TIMEOUT_MS                         1000    // Modbus等待超时时间
                           
#define LOW_BATTERY_THRESHOLD                          2000    // 低电量警告阈值 (BMS电量低于此值报警)
                          
#define BMS_SAMPLE_BUFFER_SIZE                         30      // 采样缓存数组大小
#define BMS_SAMPLE_VALID_COUNT                         20      // 实际计算平均值的采样数

#define REGS_TOTAL_NUM                                 256     // 内部Modbus寄存器总数
#define FORWARD_SLAVE_ADDR                             1       // stm32本机作为 Slave 的地址 
#define REG_ERROR_CODE                                 113     // 系统错误码
 // -看门狗打卡标志位======
#define TASK_AI_SAFY_ALIVE    (1 << 0)
#define TASK_GPS_ALIVE        (1 << 1)
#define TASK_RELAY_ALIVE      (1 << 2)
#define TASK_RFID_ALIVE       (1 << 3)  
// 需要打卡的任务总和 (二进制 0000 1111 = 0x0F)
#define TASK_ALL_ALIVE        (TASK_AI_SAFY_ALIVE | TASK_GPS_ALIVE | TASK_RELAY_ALIVE | TASK_RFID_ALIVE)

/* ========================================================================= */
/* 3. GPS 模块相关配置 和 休眠配置                                           */
/* ========================================================================= */
#define TEST_GPS_NMEA_PARSER                           0       // 开启本地假数据测试
                           
#define WT_RTK_UM982                                   1
#define WT_GPS_UM626N                                  2
#define WT_GPS_6N                                      3

#ifndef GPS_TYPE_STD
    #define GPS_TYPE_STD                               WT_GPS_6N
#endif

#define RTC_BKP_MAGIC_NUMBER                           0x5AA5  // RTC备份域校验魔数，用于判断掉电保持

// --- 休眠系统内部控制寄存器 ---
#define STATUS_POWER_OFF_TIME                          111     // 定时关机时间
#define STATUS_POWER_ON_TIME                           112     // 定时开机时间
#define RTC_TIME                                       114     // 当前RTC时间
#define STANDBY_ENABLE                                 115     // 休眠使能开关

// 时间格式：高字节=小时 / 低字节=分钟，例如 0x1500 = 21:00
#define POWER_OFF_DEFAULT                              ((10 << 8) | 22) // 默认关机 22:10 
#define POWER_ON_DEFAULT                               ((10 << 8) | 24) // 默认开机 24:10 

/* ========================================================================= */
/* 4. 声光警报模块指令与寄存器                                               */
/* ========================================================================= */
#define SLAVE_LED_ID                                   1       // 外部 LED 从机地址
#define DEFAULT_VOLUME                                 0x001E  // 默认最大音量 (30)

// --- 外部设备 (Slave) 寄存器地址与指令 ---
#define REG_LED_CTRL                                   0x00C2  // 外部 LED 控制寄存器地址
#define CMD_LED_SLOW_FLASH                             0x0051  // 指令：慢闪
#define CMD_LED_BURST_FLASH                            0x0061  // 指令：爆闪
#define CMD_LED_OFF                                    0x0060  // 指令：关闭

// --- 本机内部 Modbus 寄存器映射 ---
#define CMD_LED_SWITCH                                 0       // [接收] LED开关控制
#define STATUS_LED_SWITCH                              100     // [上报] LED当前状态

// --- 外部设备 (Slave) 寄存器地址与指令 ---
#define REG_SOUND_LIGHT_CTRL                           0x0003  // 外部 声光触发 寄存器地址
#define REG_VOLUME_CTRL                                0x0006  // 外部 音量调节 寄存器地址
#define REG_SOUND_STOP                                 0x000E  // 外部 停止播放 寄存器地址

#define CMD_SOUND_7M                                   0x0008  // 指令：7米报警声
#define CMD_SOUND_3M                                   0x0009  // 指令：3米报警声
#define CMD_SOUND_STOP                                 0x0000  // 指令：停止报警声

// --- 本机内部 Modbus 寄存器映射 ---
#define CMD_BUZZER_7M                                  1       // [接收] 7米报警控制
#define CMD_BUZZER_3M                                  2       // [接收] 3米报警控制
#define CMD_VOLUME                                     103     // [接收] 音量设置
#define STATUS_BUZZER                                  101     // [上报] 喇叭当前状态

/* ========================================================================= */
/* 5. BMS(电池管理) 模块指令与寄存器                                       */
/* ========================================================================= */
#define SLAVE_BMS_ID                                   4       // 外部 BMS 从机地址

// --- 外部设备 (Slave) 状态读取寄存器地址 ---
#define REG_BATTERY_LEVEL                              0x0000  // 获取：电池电量
#define REG_TOTAL_CURRENT                              0x0001  // 获取：总电流
#define REG_TOTAL_VOLTAGE                              0x0002  // 获取：总电压
#define REG_REMAIN_DISCHARGE                           0x0007  // 获取：剩余放电时间
#define REG_REMAIN_CHARGE                              0x0008  // 获取：剩余充电时间
#define REG_IS_CHARGING                                0x000B  // 获取：是否充电中

// --- 本机内部 Modbus 状态上报寄存器映射 ---
#define STATUS_BMS_BATTERY                             102     // [上报] BMS当前电量
#define STATUS_BMS_REMAIN_DISCHARGE_TIME               105     // [上报] BMS剩余放电时间 
#define STATUS_BMS_IS_charge                           107     // [上报] BMS是否充电中
#define STATUS_BMS_REMAIN_CHARGE_TIME                  108     // [上报] BMS剩余充电时间 
#define STATUS_BMS_TOTAL_VOLTAGE                       109     // [上报] BMS总电压
#define STATUS_BMS_TOTAL_CURRENT                       110     // [上报] BMS总电流

/* ========================================================================= */
/* 6. 其他系统逻辑与状态寄存器                                             */
/* ========================================================================= */
#define STATUS_HEART_BEAT                              104     // [上报] 主机心跳状态
#define STATUS_WORK_MODE                               106     // [上报] 吊钩工作模式


#ifdef __cplusplus
}
#endif

#endif