#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "cmsis_os.h"
#include <printf_redirect.h>
#include "app_hook_task.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define MUTEX_DECLARE(mutex) unsigned long mutex
#define MUTEX_INIT(mutex)    do{mutex = 0;}while(0)
#define MUTEX_LOCK(mutex)    do{__disable_irq();}while(0)
#define MUTEX_UNLOCK(mutex)  do{__enable_irq();}while(0)



/* Hardware Configuration */
#define LED1_PORT GPIOE
#define LED1_PIN GPIO_PIN_9
#define LED2_PORT GPIOE
#define LED2_PIN GPIO_PIN_10

/* TASK CONFIGURATION */
#define ENABLE_ENCODER_TASK
#define ENABLE_HEARTBEAT_TASK

/* DEBUG CONFIGURATION */
  // #define DEBUG_TASK_STACK
#define ENABLE_COMM_TESTING

#ifdef ENABLE_ENCODER_TASK
#define DEVICE_TYPE 0x01
#if DEVICE_TYPE == 0x00
#define ENABLE_HUB_MASTER
#define HUB_MASTER_VERSION 400
#ifndef ENCODER_SLAVE_ADDR_BASE
#define ENCODER_SLAVE_ADDR_BASE 0x01
#endif
#ifndef ENCODER_SLAVE_COUNT
#define ENCODER_SLAVE_COUNT 1
#endif
#elif DEVICE_TYPE == 0x01
#define HUB_SLAVE_VERSION 502
#define ENABLE_HUB_SLAVE
//    #define USE_FAKE_ENCODER
#elif DEVICE_TYPE == 0xFF
#define ENABLE_ERROR_CODE_RATE_TEST
#define CHECK_ECR_MASTER
//	#define CHECK_ECR_SLAVE
#else
#error "Invalid DEVICE_TYPE value. Use 0 for Slave mode or 1 for Master mode."
#endif
#endif // ENABLE_ENCODER_TASK

  uint32_t get_time_ms(void);
  void init_app(void);

#ifdef __cplusplus
}
#endif
