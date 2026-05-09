#include "gps_app.h"
#include "nmea.h"
#include "modbus_rtu_server_interface.h"

extern RTC_HandleTypeDef hrtc;

// GPS是否已同步（锁星后才允许关机判断）
uint8_t s_gps_synced = 0;

static void enter_standby(void);
static void set_alarm_b(uint8_t utc_h, uint8_t utc_m);
static void gps_sync_rtc_once(void);
static void print_internal_rtc_time(void);
static uint8_t rtc_is_wakeup_from_standby(void);

// 读取PWR标志位，1=来自待机唤醒，0=正常上电
static uint8_t rtc_is_wakeup_from_standby(void)
{
    // 读PWR标志位，1=来自待机唤醒，0=正常上电
    return (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) ? 1 : 0;
}

void config_gps_app(void)
{
    (void)uart_manage_enable_dma_recv_by_name("gps");
    osDelay(1000);

#if (GPS_TYPE_STD == WT_RTK_UM982)
    osDelay(10);
    const char cfgmsg_gga[] = "GPGGA COM1 100\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gga, sizeof(cfgmsg_gga) - 1U);
    osDelay(10);
    const char cfgmsg_rmc[] = "GPRMC COM1 1\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_rmc, sizeof(cfgmsg_rmc) - 1U);
    osDelay(10);
    const char cfgmsg_save[] = "SAVECONFIG\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_save, sizeof(cfgmsg_save) - 1U);
    osDelay(10);
#elif (GPS_TYPE_STD == WT_GPS_UM626N)
    // 只启用RMC消息，关闭其他所有消息
    const char cfgmsg_gga[] = "$CFGMSG,0,0,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gga, sizeof(cfgmsg_gga) - 1U);
    osDelay(10);
    const char cfgmsg_gll[] = "$CFGMSG,0,1,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gll, sizeof(cfgmsg_gll) - 1U);
    osDelay(10);
    const char cfgmsg_gsa[] = "$CFGMSG,0,2,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gsa, sizeof(cfgmsg_gsa) - 1U);
    osDelay(10);
    const char cfgmsg_gsv[] = "$CFGMSG,0,3,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gsv, sizeof(cfgmsg_gsv) - 1U);
    osDelay(10);
    const char cfgmsg_rmc[] = "$CFGMSG,0,4,1\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_rmc, sizeof(cfgmsg_rmc) - 1U);
    osDelay(10);
    const char cfgmsg_vtg[] = "$CFGMSG,0,5,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_vtg, sizeof(cfgmsg_vtg) - 1U);
    osDelay(10);
    const char cfgmsg_zda[] = "$CFGMSG,0,6,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_zda, sizeof(cfgmsg_zda) - 1U);
    osDelay(10);
    const char cfgmsg_gst[] = "$CFGMSG,0,7,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gst, sizeof(cfgmsg_gst) - 1U);
    osDelay(10);
#elif (GPS_TYPE_STD == WT_GPS_6N)
    const char cfgmsg_freq[] = "$PCAS03,1,0,0,0,0,0,0,0*03\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_freq, sizeof(cfgmsg_freq) - 1U);
    osDelay(10);
    const char cfgmsg_save[] = "$PCAS00*01\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_save, sizeof(cfgmsg_save) - 1U);
    osDelay(10);
#endif
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET); // 高电平gps工作
    osDelay(3000);
}
// 初始化RTC电源管理，设置默认的关机和开机时间
void rtc_power_init(void)
{
    // 解锁备份域访问权限（必须要有，否则无法读取备份寄存器）
    HAL_PWR_EnableBkUpAccess();

    MB_Reg_Set(STATUS_POWER_OFF_TIME, POWER_OFF_DEFAULT);
    MB_Reg_Set(STATUS_POWER_ON_TIME, POWER_ON_DEFAULT);
    MB_Reg_Set(STANDBY_ENABLE, 1); // 默认启用定时待机功能

    if (rtc_is_wakeup_from_standby())
    {
        LOGI("[PWR] from standby\r\n");
        __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    }
    else
    {
        LOGI("[PWR] cold start\r\n");
        // 检查备份寄存器 RTC_BKP_DR1 中是否有我们写入的标记 0x5AA5
        if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == RTC_BKP_MAGIC_NUMBER)
        {
            LOGI("[PWR] RTC time is kept alive by VBAT (Coin Cell)!\r\n");
            // 纽扣电池生效，RTC 时间有效，允许直接进行关机计划检测
            s_gps_synced = 1;
            print_internal_rtc_time();
        }
        else
        {
            LOGI("[PWR] RTC time invalid or first boot, waiting for GPS lock...\r\n");
            // 时间无效，必须等待 GPS 同步
            s_gps_synced = 0;
        }
    }
}


void update_gps_app(void)
{
#if TSET_GPS_NMEA_PARSER
    gps_test_nmea_parser();
    osDelay(1000);
    return;
#endif

    uart_inferface_t *m_obj = uart_manage_get_obj_by_name("gps");
    if (m_obj == NULL)
    {
        LOGE("GPS uart interface not found\r\n");
        return;
    }

    lwrb_sz_t available = lwrb_get_full(&m_obj->process_ring_buffer);
    if (available == 0)
    {
        return;
    }
    uint8_t to_read_buffer[128];
    lwrb_sz_t to_read = (available > sizeof(to_read_buffer)) ? sizeof(to_read_buffer) : available;
    lwrb_sz_t read_size = lwrb_read(&m_obj->process_ring_buffer, to_read_buffer, to_read);

    if (read_size > 0)
    {
        LOGD("Raw GPS data (%lu bytes): %.*s\r\n", (unsigned long)read_size, (int)read_size, (const char *)to_read_buffer);
        int parse_result = nmea_parse(to_read_buffer, (uint16_t)read_size);
        if (parse_result != NMEA_OK && parse_result != NMEA_ERR_NO_FIX)
        {
            LOGE("[PE%d]%.*s\r\n", parse_result, (int)read_size, (const char *)to_read_buffer);
        }
        else
        {
            if (g_nmea_gnss.time_h > 0 || g_nmea_gnss.time_m > 0 || g_nmea_gnss.time_s > 0)
            {
                LOGD("RMC: parse_result=%d fix=%u time=%02u:%02u:%02u date=%04u-%02u-%02u\r\n",
                     parse_result,
                     g_nmea_gnss.fix_quality,
                     g_nmea_gnss.time_h,
                     g_nmea_gnss.time_m,
                     g_nmea_gnss.time_s,
                     g_nmea_gnss.date_year,
                     g_nmea_gnss.date_m,
                     g_nmea_gnss.date_d);

                gps_sync_rtc_once();
            }
        }
    }
}

//打印内部RTC时间
void print_internal_rtc_time(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // 注意：必须先调用 GetTime，再调用 GetDate！这是 STM32 硬件影子寄存器的要求。
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // 将 RTC 的 UTC 时间转换为北京时间 (UTC+8)
    uint8_t beijing_h = (sTime.Hours + 8) % 24;

    LOGI("is real write into internal RTC: 20%02u-%02u-%02u %02u:%02u:%02u | Beijing Time: %02u:%02u:%02u\r\n",
         sDate.Year, sDate.Month, sDate.Date,
         sTime.Hours, sTime.Minutes, sTime.Seconds,
         beijing_h, sTime.Minutes, sTime.Seconds);
}

// GPS同步RTC的函数，确保只同步一次
void gps_sync_rtc_once(void)
{
    static uint8_t rtc_synced = 0;
    if (rtc_synced)
    {
        // 已经同步过，跳过
        return;
    }

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = g_nmea_gnss.time_h;
    sTime.Minutes = g_nmea_gnss.time_m;
    sTime.Seconds = g_nmea_gnss.time_s;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

    // 解锁备份域，并将 0x5AA5 写入备份寄存器 1
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_BKP_MAGIC_NUMBER);

    print_internal_rtc_time();
    osDelay(100); // 确保RTC寄存器稳定

    rtc_synced = 1;
    s_gps_synced = 1; // 控制关机逻辑，必须锁星后才允许判断
}

// 循环每10s检测
void rtc_power_schedule_check(void)
{
    if (!s_gps_synced)
    {
        return; // GPS未同步，不判断
    }
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // 使用内部 RTC 记录的 UTC 时间转换为北京时间
    uint8_t beijing_h = (sTime.Hours + 8) % 24; // UTC→北京
    uint8_t beijing_m = sTime.Minutes;
    uint16_t now_hhmm = (uint16_t)((beijing_h << 8) | beijing_m);

    uint16_t off_hhmm = MB_Reg_Get(STATUS_POWER_OFF_TIME);
    uint16_t on_hhmm = MB_Reg_Get(STATUS_POWER_ON_TIME);

    LOGD("[PWR] internal RTC beijing %02d:%02d | off=%02d:%02d on=%02d:%02d\r\n",
         beijing_h, beijing_m,
         off_hhmm >> 8, off_hhmm & 0xFF,
         on_hhmm >> 8, on_hhmm & 0xFF);

    // 把当前rtc时间暴露在modbusReg中，方便外部监控
    MB_Reg_Set(RTC_TIME, now_hhmm);

    if (now_hhmm == off_hhmm && MB_Reg_Get(STANDBY_ENABLE) == 1) // 精确匹配且待机功能启用
    {
        MB_Reg_Set(CMD_LED_SWITCH, 0);
        MB_Reg_Set(CMD_BUZZER_7M, 0);
        MB_Reg_Set(CMD_BUZZER_3M, 0);
        MB_Reg_Set(STATUS_LED_SWITCH, 0);
        MB_Reg_Set(STATUS_BUZZER, 0);

        uint8_t on_h_utc = ((on_hhmm >> 8) + 24 - TIMEZONE_OFFSET_BEIJING) % 24;
        set_alarm_b(on_h_utc, (uint8_t)(on_hhmm & 0xFF));
        enter_standby();
    }
}

void set_alarm_b(uint8_t utc_h, uint8_t utc_m)
{
    HAL_PWR_EnableBkUpAccess();

    HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_B); // 先关旧闹钟

    RTC_AlarmTypeDef sAlarm = {0};
    sAlarm.AlarmTime.Hours = utc_h;
    sAlarm.AlarmTime.Minutes = utc_m;
    sAlarm.AlarmTime.Seconds = 0;

    sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
    // 时分触发，忽略秒和星期
    sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY | RTC_ALARMMASK_SECONDS;
    sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
    sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
    sAlarm.AlarmDateWeekDay = 1;
    sAlarm.Alarm = RTC_ALARM_B;

    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK)
    {
        LOGE("[RTC] Alarm B SET FAILED!\r\n");
        return;
    }

    __HAL_RTC_ALARM_EXTI_ENABLE_IT();
    __HAL_RTC_ALARM_EXTI_ENABLE_RISING_EDGE();

    LOGI("[RTC] Alarm B: UTC %02d:%02d\r\n", utc_h, utc_m);
}

// 进入待机，不返回
void enter_standby(void)
{
    // ================== 安全校验防线 ==================
    // 1. Check if the backup domain data is still there?
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x5AA5)
    {
        LOGE("[PWR-ERR] Backup domain invalid! Magic number lost.\r\n");
        s_gps_synced = 0; // 取消同步标志
        return;           // 拒绝休眠，退回去继续等 GPS 信号
    }

    if (!s_gps_synced)
    {
        LOGE("[PWR-ERR] System not synced with GPS/VBAT. Abort standby.\r\n");
        return;
    }

    LOGI("[PWR] enter standby mode...\r\n");
    osDelay(200);
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_RESET);
    osDelay(100);

    __HAL_RTC_ALARM_CLEAR_FLAG(&hrtc, RTC_FLAG_ALRBF);
    __HAL_RTC_ALARM_EXTI_CLEAR_FLAG();

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP1);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP2);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP3);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP4);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP5);
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WKUP6);

    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
    HAL_PWR_EnterSTANDBYMode();
}

void gps_rtc_app_init(void){
    config_gps_app();
    rtc_power_init();
}

// ========== 2. 核心业务总线 (自带时序) ==========
void process_gps_logic(void)
{
    static TickType_t last_1000ms = 0;

    // 1. 串口缓冲区解析 (每次循环都执行，防止缓冲区溢出)
    update_gps_app();

    // 2. 休眠日程检测与心跳灯 (每 1000ms 执行一次)
    if (xTaskGetTickCount() - last_1000ms >= pdMS_TO_TICKS(1000))
    {
        last_1000ms = xTaskGetTickCount();
        
        rtc_power_schedule_check();
        
        HAL_GPIO_TogglePin(GPIOD, H_B_LED_Pin); // 心跳灯闪烁

    }
}
