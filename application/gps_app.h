#ifndef GPS_APP_H
#define GPS_APP_H

#include <stdint.h>
#include "system_def.h"
#include "uart_manage.h"

void gps_rtc_app_init(void);   
void process_gps_logic(void);  

#endif // GPS_APP_H