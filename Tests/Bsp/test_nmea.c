#include "nmea.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* Test utilities */
#define TEST_ASSERT(cond) \
    do { \
        if (!(cond)) { \
            printf("Test failed at %s:%d\n", __FILE__, __LINE__); \
            return -1; \
        } \
    } while (0)

static void reset_gnss_data(void)
{
    memset(&g_nmea_gnss, 0, sizeof(g_nmea_gnss));
}

/* Test cases */
static int test_null_input(void)
{
    reset_gnss_data();
    TEST_ASSERT(nmea_parse(NULL, 100) == NMEA_ERR_INVALID_INPUT);
    TEST_ASSERT(nmea_parse((uint8_t*)"test", 0) == NMEA_ERR_INVALID_INPUT);
    TEST_ASSERT(nmea_parse(NULL, 0) == NMEA_ERR_INVALID_INPUT);
    return 0;
}

static int test_short_input(void)
{
    reset_gnss_data();
    uint8_t buf[10] = "$GNGGA";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)) == NMEA_ERR_INVALID_INPUT);
    return 0;
}

static int test_invalid_format(void)
{
    reset_gnss_data();
    uint8_t buf[] = "INVALID*5B\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_ERR_FORMAT);
    return 0;
}

static int test_gga_sentence(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.valid_time);
    TEST_ASSERT(g_nmea_gnss.time_h == 9);
    TEST_ASSERT(g_nmea_gnss.time_m == 27);
    TEST_ASSERT(g_nmea_gnss.time_s == 25);
    TEST_ASSERT(g_nmea_gnss.valid_latitude);
    TEST_ASSERT(g_nmea_gnss.valid_longitude);
    TEST_ASSERT(g_nmea_gnss.fix_quality == 1);
    TEST_ASSERT(g_nmea_gnss.satellite == 8);
    TEST_ASSERT(g_nmea_gnss.valid_precision);
    TEST_ASSERT(g_nmea_gnss.valid_altitude);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_GGA_BIT);
    return 0;
}

static int test_gll_sentence(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNGLL,4717.11364,N,00833.91565,E,092321.00,A,A*60\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.valid_latitude);
    TEST_ASSERT(g_nmea_gnss.valid_longitude);
    TEST_ASSERT(g_nmea_gnss.valid_time);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_GLL_BIT);
    return 0;
}

static int test_gsa_sentence(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNGSA,A,3,19,28,14,18,27,22,31,39,,,,,1.7,1.0,1.3*35\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.fix_mode == 3);
    TEST_ASSERT(g_nmea_gnss.satellite == 8); // 8 satellites used
    TEST_ASSERT(g_nmea_gnss.valid_pdop);
    TEST_ASSERT(g_nmea_gnss.valid_precision);
    TEST_ASSERT(g_nmea_gnss.valid_vdop);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_GSA_BIT);
    return 0;
}

static int test_gsv_sentence(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GPGSV,3,1,11,03,03,111,00,04,15,270,00,06,01,010,00,13,06,292,00*74\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.valid_satellite_in_view);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_GSV_BIT);
    return 0;
}

static int test_rmc_sentence(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNRMC,092725.00,A,4717.11399,N,00833.91590,E,0.004,,170323,,,A,V*2E\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.valid_time);
    TEST_ASSERT(g_nmea_gnss.valid_date);
    TEST_ASSERT(g_nmea_gnss.date_d == 17);
    TEST_ASSERT(g_nmea_gnss.date_m == 3);
    TEST_ASSERT(g_nmea_gnss.date_year == 2023);
    TEST_ASSERT(g_nmea_gnss.valid_latitude);
    TEST_ASSERT(g_nmea_gnss.valid_longitude);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_RMC_BIT);
    return 0;
}

static int test_vtg_sentence(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNVTG,,T,,M,0.004,N,0.008,K,A*13\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.valid_speed);
    TEST_ASSERT(g_nmea_gnss.valid_speed_kmh);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_VTG_BIT);
    return 0;
}

static int test_zda_sentence(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNZDA,092725.00,17,03,2023,00,00*7D\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.valid_time);
    TEST_ASSERT(g_nmea_gnss.valid_date);
    TEST_ASSERT(g_nmea_gnss.date_d == 17);
    TEST_ASSERT(g_nmea_gnss.date_m == 3);
    TEST_ASSERT(g_nmea_gnss.date_year == 2023);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_ZDA_BIT);
    return 0;
}

static int test_txt_sentence(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNTXT,01,01,02,ANTSTATUS=OK*3B\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_TXT_BIT);
    return 0;
}

static int test_mixed_sentences(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5B\r\n"
                    "$GNRMC,092725.00,A,4717.11399,N,00833.91590,E,0.004,,170323,,,A,V*2E\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_OK);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_GGA_BIT);
    TEST_ASSERT(g_nmea_gnss.sentence_mask & NMEA_SENTENCE_RMC_BIT);
    return 0;
}

static int test_checksum_failure(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,*5C\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_ERR_CHECKSUM_FAIL);
    return 0;
}

static int test_no_checksum(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNGGA,092725.00,4717.11399,N,00833.91590,E,1,08,1.01,499.6,M,48.0,M,,\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_ERR_NO_CHECKSUM);
    return 0;
}

static int test_no_fix(void)
{
    reset_gnss_data();
    uint8_t buf[] = "$GNGGA,092725.00,4717.11399,N,00833.91590,E,0,08,1.01,499.6,M,48.0,M,,*5A\r\n";
    TEST_ASSERT(nmea_parse(buf, sizeof(buf)-1) == NMEA_ERR_NO_FIX);
    TEST_ASSERT(g_nmea_gnss.fix_quality == 0);
    return 0;
}

/* Test runner */
typedef int (*test_func_t)(void);

static const test_func_t tests[] = {
    test_null_input,
    test_short_input,
    test_invalid_format,
    test_gga_sentence,
    test_gll_sentence,
    test_gsa_sentence,
    test_gsv_sentence,
    test_rmc_sentence,
    test_vtg_sentence,
    test_zda_sentence,
    test_txt_sentence,
    test_mixed_sentences,
    test_checksum_failure,
    test_no_checksum,
    test_no_fix,
    NULL
};

int main(void)
{
    printf("Running nmea_parse tests...\n");
    
    int failed = 0;
    for (const test_func_t *test = tests; *test != NULL; test++) {
        reset_gnss_data();
        if ((*test)() != 0) {
            printf("Test %td failed\n", test - tests);
            failed++;
        }
    }
    
    if (failed == 0) {
        printf("All tests passed!\n");
        return 0;
    } else {
        printf("%d tests failed\n", failed);
        return -1;
    }
}