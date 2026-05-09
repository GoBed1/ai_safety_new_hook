#ifndef APP_HOOK_TASK_H
#define APP_HOOK_TASK_H

#include <stdint.h>
#include "system_def.h"
// #include "board.h"
#include "cmsis_os.h"
#include "app_rfid.h"
// ... (保留你原来的 RFIDClient 结构体、枚举和宏定义) ...

void init_app_hook_task(void);

#endif // APP_HOOK_TASK_H
