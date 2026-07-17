#define MODULE_LOG_ENABLE 1
#include "gps_app.h"
#include "nmea.h"
#include "modbus_rtu_server_interface.h"

#define GPS_RX_CHUNK_SIZE        128U
#define GPS_NMEA_SENTENCE_SIZE   160U
#define GPS_CONFIG_COMMAND_DELAY 50U

extern RTC_HandleTypeDef hrtc;

// GPS time/date has synchronized RTC; only then enable the power schedule.
uint8_t s_gps_synced = 0;

static void enter_standby(void);
static void set_alarm_b(uint8_t utc_h, uint8_t utc_m);
static void gps_sync_rtc_once(void);
static void print_internal_rtc_time(void);
static uint8_t rtc_is_wakeup_from_standby(void);
static void gps_process_rx_byte(uint8_t byte);
static void gps_process_sentence(const uint8_t *sentence, uint16_t length);
static uint8_t gps_calculate_weekday(uint16_t year, uint8_t month, uint8_t day);
volatile uint16_t is_soft_standby = 0; // 软休眠状态标志（爆闪灯断电）
// 读取PWR标志位，1=来自待机唤醒，0=正常上电
static uint8_t rtc_is_wakeup_from_standby(void)
{
    // 读PWR标志位，1=来自待机唤醒，0=正常上电
    return (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) ? 1 : 0;
}

void config_gps_app(void)
{
    HAL_GPIO_WritePin(GPS_EN_GPIO_Port, GPS_EN_Pin, GPIO_PIN_SET);
    (void)uart_manage_enable_dma_recv_by_name("gps");
    osDelay(3000);

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
    osDelay(GPS_CONFIG_COMMAND_DELAY);
    const char cfgmsg_gbs[] = "$CFGMSG,0,8,0\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_gbs, sizeof(cfgmsg_gbs) - 1U);
    osDelay(GPS_CONFIG_COMMAND_DELAY);
#elif (GPS_TYPE_STD == WT_GPS_6N)
    const char cfgmsg_freq[] = "$PCAS03,1,0,0,0,0,0,0,0*03\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_freq, sizeof(cfgmsg_freq) - 1U);
    osDelay(10);
    const char cfgmsg_save[] = "$PCAS00*01\r\n";
    uart_manage_dma_send_by_name("gps", (uint8_t *)cfgmsg_save, sizeof(cfgmsg_save) - 1U);
    osDelay(10);
#endif
#if (GPS_TYPE_STD == WT_GPS_UM626N)
    LOGI("GPS configured: UM626N, USART3 115200 8N1, RMC only\r\n");
#endif
}
// 初始化RTC电源管理，设置默认的关机和开机时间
void rtc_power_init(void)
{
    // 解锁备份域访问权限（必须要有，否则无法读取备份寄存器）
    HAL_PWR_EnableBkUpAccess();

    MB_Reg_Set(STATUS_POWER_OFF_TIME, POWER_OFF_DEFAULT);
    MB_Reg_Set(STATUS_POWER_ON_TIME, POWER_ON_DEFAULT);
    MB_Reg_Set(SOFT_STANDBY_ENABLE, 1); // 软待机默认开启

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

static void gps_process_sentence(const uint8_t *sentence, uint16_t length)
{
    int parse_result;

    if ((sentence == NULL) || (length == 0U))
    {
        return;
    }

    LOGI("[GPS RX] %.*s", (int)length, (const char *)sentence);

    /* NMEA talker IDs may be GP/GN/BD; the sentence type is bytes 3..5. */
    if ((length < 7U) ||
        (sentence[0] != '$') ||
        (sentence[3] != 'R') ||
        (sentence[4] != 'M') ||
        (sentence[5] != 'C'))
    {
        return;
    }

    /* Avoid accepting time/date flags left over from an earlier sentence. */
    g_nmea_gnss.valid_time = 0U;
    g_nmea_gnss.valid_date = 0U;
    parse_result = nmea_parse_gxrmc(sentence, length);
    if ((parse_result == NMEA_OK) || (parse_result == NMEA_ERR_NO_FIX))
    {
        LOGI("[GPS RMC] status=%c, UTC=%02u:%02u:%02u, date=%04u-%02u-%02u\r\n",
             (parse_result == NMEA_OK) ? 'A' : 'V',
             (unsigned int)g_nmea_gnss.time_h,
             (unsigned int)g_nmea_gnss.time_m,
             (unsigned int)g_nmea_gnss.time_s,
             (unsigned int)g_nmea_gnss.date_year,
             (unsigned int)g_nmea_gnss.date_m,
             (unsigned int)g_nmea_gnss.date_d);

        if ((g_nmea_gnss.valid_time != 0U) &&
            (g_nmea_gnss.valid_date != 0U))
        {
            gps_sync_rtc_once();
        }
        else
        {
            LOGE("[GPS RMC] current sentence has no valid time/date\r\n");
        }
    }
    else
    {
        LOGE("[GPS RMC] parse failed: %d\r\n", parse_result);
    }
}

static void gps_process_rx_byte(uint8_t byte)
{
    static uint8_t sentence[GPS_NMEA_SENTENCE_SIZE];
    static uint16_t sentence_length = 0U;

    if (byte == '$')
    {
        sentence_length = 0U;
    }

    if ((sentence_length == 0U) && (byte != '$'))
    {
        return;
    }

    if (sentence_length >= sizeof(sentence))
    {
        LOGE("GPS sentence overflow\r\n");
        sentence_length = 0U;
        return;
    }

    sentence[sentence_length++] = byte;
    if (byte == '\n')
    {
        gps_process_sentence(sentence, sentence_length);
        sentence_length = 0U;
    }
}

void update_gps_app(void)
{
#if TEST_GPS_NMEA_PARSER
    gps_test_nmea_parser();
    osDelay(1000);
    return;
#endif

    uart_inferface_t *m_obj = uart_manage_get_obj_by_name("gps");
    uint8_t rx_chunk[GPS_RX_CHUNK_SIZE];

    if (m_obj == NULL)
    {
        LOGE("GPS uart interface not found\r\n");
        return;
    }

    for (;;)
    {
        lwrb_sz_t available = lwrb_get_full(&m_obj->process_ring_buffer);
        lwrb_sz_t to_read;
        lwrb_sz_t read_size;

        if (available == 0U)
        {
            break;
        }

        to_read = (available > sizeof(rx_chunk)) ? sizeof(rx_chunk) : available;
        read_size = lwrb_read(&m_obj->process_ring_buffer, rx_chunk, to_read);
        if (read_size == 0U)
        {
            break;
        }

        for (lwrb_sz_t i = 0U; i < read_size; ++i)
        {
            gps_process_rx_byte(rx_chunk[i]);
        }
    }
}

// 打印内部RTC时间
void print_internal_rtc_time(void)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    // 注意：必须先调用 GetTime，再调用 GetDate！这是 STM32 硬件影子寄存器的要求。
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // 将 RTC 的 UTC 时间转换为北京时间 (UTC+8)
    uint8_t beijing_h = (sTime.Hours + 8) % 24;
    (void)beijing_h;

    LOGI("is real write into internal RTC: 20%02u-%02u-%02u %02u:%02u:%02u | Beijing Time: %02u:%02u:%02u\r\n",
         sDate.Year, sDate.Month, sDate.Date,
         sTime.Hours, sTime.Minutes, sTime.Seconds,
         beijing_h, sTime.Minutes, sTime.Seconds);
}

// GPS同步RTC的函数，确保只同步一次
static uint8_t gps_calculate_weekday(uint16_t year, uint8_t month, uint8_t day)
{
    static const uint8_t month_offset[12] = {0U, 3U, 2U, 5U, 0U, 3U,
                                             5U, 1U, 4U, 6U, 2U, 4U};
    uint32_t adjusted_year = year;
    uint32_t weekday;

    if (month < 3U)
    {
        --adjusted_year;
    }

    weekday = (adjusted_year + (adjusted_year / 4U) -
               (adjusted_year / 100U) + (adjusted_year / 400U) +
               month_offset[month - 1U] + day) % 7U;

    return (weekday == 0U) ? RTC_WEEKDAY_SUNDAY : (uint8_t)weekday;
}

static void gps_sync_rtc_once(void)
{
    static uint8_t rtc_synced = 0;
    if (rtc_synced)
    {
        // 已经同步过，跳过
        return;
    }

    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    if ((g_nmea_gnss.time_h > 23U) ||
        (g_nmea_gnss.time_m > 59U) ||
        (g_nmea_gnss.time_s > 59U) ||
        (g_nmea_gnss.date_year < 2000U) ||
        (g_nmea_gnss.date_year > 2099U) ||
        (g_nmea_gnss.date_m < 1U) ||
        (g_nmea_gnss.date_m > 12U) ||
        (g_nmea_gnss.date_d < 1U) ||
        (g_nmea_gnss.date_d > 31U))
    {
        LOGE("GPS RMC time/date out of range\r\n");
        return;
    }

    sTime.Hours = g_nmea_gnss.time_h;
    sTime.Minutes = g_nmea_gnss.time_m;
    sTime.Seconds = g_nmea_gnss.time_s;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    sDate.Year = (uint8_t)(g_nmea_gnss.date_year - 2000U);
    sDate.Month = g_nmea_gnss.date_m;
    sDate.Date = g_nmea_gnss.date_d;
    sDate.WeekDay = gps_calculate_weekday(g_nmea_gnss.date_year,
                                          g_nmea_gnss.date_m,
                                          g_nmea_gnss.date_d);

    if ((HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) ||
        (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK))
    {
        LOGE("GPS failed to synchronize RTC\r\n");
        return;
    }

    // 解锁备份域，并将 0x5AA5 写入备份寄存器 1
    HAL_PWR_EnableBkUpAccess();
    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, RTC_BKP_MAGIC_NUMBER);

    print_internal_rtc_time();
    osDelay(100); // 确保RTC寄存器稳定

    rtc_synced = 1;
    s_gps_synced = 1; // RTC has a valid GPS time/date; allow power schedule checks.
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

    uint16_t soft_enable = MB_Reg_Get(SOFT_STANDBY_ENABLE);  // 软休眠开关
    uint16_t hard_enable = MB_Reg_Get(STM32_STANDBY_ENABLE); // 硬休眠开关

    LOGD("[PWR] internal RTC beijing %02d:%02d | off=%02d:%02d on=%02d:%02d\r\n",
         beijing_h, beijing_m,
         off_hhmm >> 8, off_hhmm & 0xFF,
         on_hhmm >> 8, on_hhmm & 0xFF);

    // 把当前rtc时间暴露在modbusReg中，方便外部监控
    MB_Reg_Set(RTC_TIME, now_hhmm);

    if (now_hhmm == off_hhmm && hard_enable == 1) // 精确匹配且stm32待机功能启用
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
    // 启用软休眠功能（爆闪灯断电）
    if (soft_enable == 1)
    {
        uint8_t should_sleep = 0; // 当前时间是否休眠

        // 判断当前时间是否落在 [关机时间, 开机时间)
        if (off_hhmm < on_hhmm)
        {
            if (now_hhmm >= off_hhmm && now_hhmm < on_hhmm)
            {
                should_sleep = 1;
            }
        }
        else if (off_hhmm > on_hhmm)
        {
            if (now_hhmm >= off_hhmm || now_hhmm < on_hhmm)
            {
                should_sleep = 1;
            }
        }
        // 触发条件：到达关机时间，且当前不在待机状态
        if (should_sleep == 1 && is_soft_standby == 0)
        {
            is_soft_standby = 1;
            MB_Reg_Set(CMD_LED_SWITCH, 0);
            MB_Reg_Set(CMD_BUZZER_7M, 0);
            MB_Reg_Set(CMD_BUZZER_3M, 0);
            MB_Reg_Set(STATUS_LED_SWITCH, 0);
            MB_Reg_Set(STATUS_BUZZER, 0);

            HAL_GPIO_WritePin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin, GPIO_PIN_RESET);
            LOGI("[PWR] Enter SOFT standby. Relay 2 OFF.\r\n");

            // 清除可能存在的心跳和掉线错误，防止休眠期间板载LED还在闪错
            taskENTER_CRITICAL();
            uint16_t err = MB_Reg_Get(REG_ERROR_CODE);
            MB_Reg_Set(REG_ERROR_CODE, err & ~(ERR_HEARTBEAT_TIMEOUT | ERR_LED_OFFLINE));
            taskEXIT_CRITICAL();
        }
        else if (should_sleep == 0 && is_soft_standby == 1)
        {
            is_soft_standby = 0; // 标记系统退出软休眠状态

            HAL_GPIO_WritePin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin, GPIO_PIN_SET);
            LOGI("[PWR] Exit SOFT standby. Relay 2 ON.\r\n");
        }
    }
    // 兜底保护：如果上位机中途强行关闭了休眠功能，但系统还卡在待机里，强行唤醒
    else if (soft_enable == 0 && is_soft_standby == 1)
    {
        is_soft_standby = 0;
        HAL_GPIO_WritePin(FLASH_LIGHT_POWER_EN_GPIO_Port, FLASH_LIGHT_POWER_EN_Pin, GPIO_PIN_SET);
        LOGI("[PWR] soft standby Disabled. Force Exit SOFT standby.\r\n");
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

void gps_rtc_app_init(void)
{
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
    }
}
