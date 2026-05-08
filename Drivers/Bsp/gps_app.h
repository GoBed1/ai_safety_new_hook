#ifndef GPS_APP_H
#define GPS_APP_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "uart_manage.h"
#include "system_def.h"


extern uart_inferface_t gps_app;
void config_gps_app(void);
void update_gps_app(void);
void rtc_power_init(void);
void rtc_power_schedule_check(void);
uint8_t rtc_is_wakeup_from_standby(void);
void gps_test_nmea_parser(void);
void update_gps_time_loop_test(void);
void print_internal_rtc_time(void);

#ifdef __cplusplus
}
#endif

#endif // GPS_APP_H
