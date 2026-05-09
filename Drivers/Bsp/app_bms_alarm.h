#ifndef APP_BMS_ALARM_H
#define APP_BMS_ALARM_H

#include <stdint.h>
#include "system_def.h"
void init_bms_alarm_module(void);
void modbus_alarm_handle(void);
void modbus_bms_handle(void);

#endif // APP_BMS_ALARM_H
