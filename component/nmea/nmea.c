#include "nmea.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Global GNSS data structure */
nmea_gnss_t g_nmea_gnss = {0};

#define NMEA_SENTENCE_GGA_BIT (1UL << 0)
#define NMEA_SENTENCE_GLL_BIT (1UL << 1)
#define NMEA_SENTENCE_GSA_BIT (1UL << 2)
#define NMEA_SENTENCE_GSV_BIT (1UL << 3)
#define NMEA_SENTENCE_RMC_BIT (1UL << 4)
#define NMEA_SENTENCE_VTG_BIT (1UL << 5)
#define NMEA_SENTENCE_ZDA_BIT (1UL << 6)
#define NMEA_SENTENCE_TXT_BIT (1UL << 7)

static bool nmea_is_hex(char c)
{
  return ((c >= '0' && c <= '9') ||
          (c >= 'A' && c <= 'F') ||
          (c >= 'a' && c <= 'f'));
}

static uint8_t nmea_hex_to_u8(char c)
{
  if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
  if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
  if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
  return 0;
}

static int nmea_extract_sentence(const uint8_t *buf,
                                 uint16_t len,
                                 const char *type,
                                 char *sentence,
                                 uint16_t sentence_size)
{
  if (buf == NULL || sentence == NULL || sentence_size < 16)
    return NMEA_ERR_INVALID_INPUT;

  for (uint16_t i = 0; i < len; i++)
  {
    if (buf[i] != '$')
      continue;

    if ((uint16_t)(i + 6) >= len)
      continue;

    if (type != NULL)
    {
      if (buf[i + 3] != (uint8_t)type[0] ||
          buf[i + 4] != (uint8_t)type[1] ||
          buf[i + 5] != (uint8_t)type[2])
      {
        continue;
      }
    }

    uint16_t star = i;
    while (star < len && buf[star] != '*')
      star++;

    if (star >= len)
      continue;

    if ((uint16_t)(star + 2) >= len)
      continue;

    if (!nmea_is_hex((char)buf[star + 1]) || !nmea_is_hex((char)buf[star + 2]))
      continue;

    uint16_t end = (uint16_t)(star + 3);
    while (end < len && (buf[end] == '\r' || buf[end] == '\n'))
      end++;

    uint16_t copy_len = (uint16_t)(end - i);
    if (copy_len >= sentence_size)
      return NMEA_ERR_SHORT_LENGTH;

    memcpy(sentence, &buf[i], copy_len);
    sentence[copy_len] = '\0';
    return NMEA_OK;
  }

  return NMEA_ERR_FORMAT;
}

static int nmea_validate_checksum(const char *sentence)
{
  if (sentence == NULL || sentence[0] != '$')
    return NMEA_ERR_FORMAT;

  char *star = strchr(sentence, '*');
  if (star == NULL)
    return NMEA_ERR_NO_CHECKSUM;

  if (!nmea_is_hex(star[1]) || !nmea_is_hex(star[2]))
    return NMEA_ERR_CHECKSUM_FAIL;

  uint8_t calc = nmea_checksum((const uint8_t *)(sentence + 1), (uint16_t)(star - sentence - 1));
  uint8_t read = (uint8_t)((nmea_hex_to_u8(star[1]) << 4) | nmea_hex_to_u8(star[2]));
  if (calc != read)
    return NMEA_ERR_CHECKSUM_FAIL;

  return NMEA_OK;
}

static int nmea_sentence_to_fields(const char *sentence,
                                   char *payload,
                                   uint16_t payload_size,
                                   char **fields,
                                   uint8_t fields_max,
                                   uint8_t *field_count)
{
  if (sentence == NULL || payload == NULL || fields == NULL || field_count == NULL)
    return NMEA_ERR_INVALID_INPUT;

  char *star = strchr(sentence, '*');
  if (star == NULL)
    return NMEA_ERR_NO_CHECKSUM;

  if (sentence[0] != '$')
    return NMEA_ERR_FORMAT;

  uint16_t payload_len = (uint16_t)(star - (sentence + 1));
  if ((uint16_t)(payload_len + 1) > payload_size)
    return NMEA_ERR_SHORT_LENGTH;

  memcpy(payload, sentence + 1, payload_len);
  payload[payload_len] = '\0';

  uint8_t count = 0;
  fields[count++] = payload;
  for (uint16_t i = 0; i < payload_len && count < fields_max; i++)
  {
    if (payload[i] == ',')
    {
      payload[i] = '\0';
      fields[count++] = &payload[i + 1];
    }
  }

  *field_count = count;
  return NMEA_OK;
}

static bool nmea_header_match(const char *header, const char *type)
{
  if (header == NULL || type == NULL)
    return false;

  if (strlen(header) < 5)
    return false;

  return (header[2] == type[0] && header[3] == type[1] && header[4] == type[2]);
}

static void nmea_parse_time_hms(const char *str)
{
  if (str == NULL || strlen(str) < 6)
    return;

  g_nmea_gnss.time_h = (uint8_t)((str[0] - '0') * 10 + (str[1] - '0'));
  g_nmea_gnss.time_m = (uint8_t)((str[2] - '0') * 10 + (str[3] - '0'));
  g_nmea_gnss.time_s = (uint8_t)((str[4] - '0') * 10 + (str[5] - '0'));
  g_nmea_gnss.valid_time = 1;
}

static void nmea_parse_date_ddmmyy(const char *str)
{
  if (str == NULL || strlen(str) < 6)
    return;

  g_nmea_gnss.date_d = (uint8_t)((str[0] - '0') * 10 + (str[1] - '0'));
  g_nmea_gnss.date_m = (uint8_t)((str[2] - '0') * 10 + (str[3] - '0'));
  g_nmea_gnss.date_y = (uint8_t)((str[4] - '0') * 10 + (str[5] - '0'));
  g_nmea_gnss.date_year = (uint16_t)(2000U + g_nmea_gnss.date_y);
  g_nmea_gnss.valid_date = 1;
}

static void nmea_parse_lat_lon(const char *lat_str,
                               const char *lat_ns,
                               const char *lon_str,
                               const char *lon_ew)
{
  if (lat_str != NULL && lat_str[0] != '\0')
  {
    g_nmea_gnss.latitude_deg = nmea_convert((float)atof(lat_str));
    if (lat_ns != NULL && lat_ns[0] == 'S')
      g_nmea_gnss.latitude_deg = -g_nmea_gnss.latitude_deg;
    g_nmea_gnss.valid_latitude = 1;
  }

  if (lon_str != NULL && lon_str[0] != '\0')
  {
    g_nmea_gnss.longitude_deg = nmea_convert((float)atof(lon_str));
    if (lon_ew != NULL && lon_ew[0] == 'W')
      g_nmea_gnss.longitude_deg = -g_nmea_gnss.longitude_deg;
    g_nmea_gnss.valid_longitude = 1;
  }
}

static void nmea_mark_sentence(uint32_t bit)
{
  g_nmea_gnss.sentence_mask |= bit;
  g_nmea_gnss.valid_sentence_mask = 1;
}

static bool nmea_is_uint_str(const char *str)
{
  if (str == NULL || str[0] == '\0')
    return false;

  for (uint16_t i = 0; str[i] != '\0'; i++)
  {
    if (str[i] < '0' || str[i] > '9')
      return false;
  }
  return true;
}

static int nmea_parse_by_type(const uint8_t *buf, uint16_t len, const char *type, char *sentence, uint16_t sentence_size)
{
  int ret = nmea_extract_sentence(buf, len, type, sentence, sentence_size);
  if (ret != NMEA_OK)
    return ret;

  return nmea_validate_checksum(sentence);
}

/**
 * Calculate NMEA checksum
 * @param sentence: pointer to NMEA sentence (without $)
 * @param len: length of sentence
 * @return: calculated checksum byte
 */
uint8_t nmea_checksum(const uint8_t *sentence, uint16_t len)
{
  uint8_t chk = 0;
  
  if (sentence == NULL || len == 0)
    return 0;
  
  for (uint16_t i = 0; i < len; i++)
  {
    uint8_t c = sentence[i];
    if (c == '*' || c == '\r' || c == '\n')
      break;
    chk ^= c;
  }
  
  return chk;
}

/**
 * Convert raw NMEA degrees to decimal degrees
 * @param raw_degrees: raw value (e.g., 4807.038 for 48°07.038')
 * @return: decimal degrees (e.g., 48.117300)
 */
float nmea_convert(float raw_degrees)
{
  int firstdigits = ((int)raw_degrees) / 100;
  float nexttwodigits = raw_degrees - (float)(firstdigits * 100.0f);
  return (float)(firstdigits + nexttwodigits / 60.0f);
}

/**
 * Parse GNGGA sentence (GGA is the most important for positioning)
 * Format: $GNGGA,hhmmss.ss,ddmm.mmmm,a,dddmm.mmmm,a,x,xx,x.x,x.x,M,x.x,M,,*hh
 * 
 * @param buf: pointer to NMEA sentence buffer
 * @param len: length of buffer
 * @return: NMEA_OK (0) if success, negative error code on failure
 */
int nmea_parse_gngga(const uint8_t *buf, uint16_t len)
{
  char sentence[256];
  int ret = nmea_parse_by_type(buf, len, "GGA", sentence, (uint16_t)sizeof(sentence));
  if (ret != NMEA_OK)
    return ret;

  char payload[256];
  char *fields[24] = {0};
  uint8_t field_count = 0;
  ret = nmea_sentence_to_fields(sentence, payload, (uint16_t)sizeof(payload), fields, (uint8_t)24, &field_count);
  if (ret != NMEA_OK)
    return ret;

  if (field_count < 10 || !nmea_header_match(fields[0], "GGA"))
    return NMEA_ERR_FORMAT;
  
  nmea_parse_time_hms(fields[1]);
  nmea_parse_lat_lon(fields[2], fields[3], fields[4], fields[5]);

  g_nmea_gnss.fix_quality = (uint8_t)atoi(fields[6]);
  g_nmea_gnss.satellite = (uint8_t)atoi(fields[7]);
  g_nmea_gnss.valid_satellite = (g_nmea_gnss.satellite > 0) ? 1 : 0;

  if (fields[8][0] != '\0')
  {
    g_nmea_gnss.precision_m = (float)atof(fields[8]);
    g_nmea_gnss.valid_precision = 1;
  }

  if (fields[9][0] != '\0')
  {
    g_nmea_gnss.altitude_m = (float)atof(fields[9]);
    g_nmea_gnss.valid_altitude = 1;
  }

  nmea_mark_sentence(NMEA_SENTENCE_GGA_BIT);

  if (g_nmea_gnss.fix_quality == 0)
    return NMEA_ERR_NO_FIX;

  return NMEA_OK;
}

int nmea_parse_gxgll(const uint8_t *buf, uint16_t len)
{
  char sentence[256];
  int ret = nmea_parse_by_type(buf, len, "GLL", sentence, (uint16_t)sizeof(sentence));
  if (ret != NMEA_OK)
    return ret;

  char payload[256];
  char *fields[20] = {0};
  uint8_t field_count = 0;
  ret = nmea_sentence_to_fields(sentence, payload, (uint16_t)sizeof(payload), fields, (uint8_t)20, &field_count);
  if (ret != NMEA_OK)
    return ret;

  if (field_count < 7 || !nmea_header_match(fields[0], "GLL"))
    return NMEA_ERR_FORMAT;

  nmea_parse_lat_lon(fields[1], fields[2], fields[3], fields[4]);
  nmea_parse_time_hms(fields[5]);
  nmea_mark_sentence(NMEA_SENTENCE_GLL_BIT);

  if (fields[6][0] == 'A')
  {
    if (g_nmea_gnss.fix_quality == 0)
      g_nmea_gnss.fix_quality = 1;
    return NMEA_OK;
  }

  return NMEA_ERR_NO_FIX;
}

int nmea_parse_gxgsa(const uint8_t *buf, uint16_t len)
{
  char sentence[256];
  int ret = nmea_parse_by_type(buf, len, "GSA", sentence, (uint16_t)sizeof(sentence));
  if (ret != NMEA_OK)
    return ret;

  char payload[256];
  char *fields[24] = {0};
  uint8_t field_count = 0;
  ret = nmea_sentence_to_fields(sentence, payload, (uint16_t)sizeof(payload), fields, (uint8_t)24, &field_count);
  if (ret != NMEA_OK)
    return ret;

  if (field_count < 3 || !nmea_header_match(fields[0], "GSA"))
    return NMEA_ERR_FORMAT;

  g_nmea_gnss.fix_mode = (uint8_t)atoi(fields[2]);
  g_nmea_gnss.valid_fix_mode = 1;

  uint8_t sat_used = 0;
  uint8_t sat_end = 14;
  if (field_count >= 6)
  {
    uint8_t dop_pdop_idx = (uint8_t)(field_count - 3);
    if (dop_pdop_idx > 3)
      sat_end = (uint8_t)(dop_pdop_idx - 1);
  }
  if (sat_end > 14)
    sat_end = 14;

  for (uint8_t i = 3; i <= sat_end && i < field_count; i++)
  {
    if (nmea_is_uint_str(fields[i]))
      sat_used++;
  }
  if (sat_used > 0)
  {
    g_nmea_gnss.satellite = sat_used;
    g_nmea_gnss.valid_satellite = 1;
  }

  if (field_count >= 6)
  {
    uint8_t pdop_idx = (uint8_t)(field_count - 3);
    uint8_t hdop_idx = (uint8_t)(field_count - 2);
    uint8_t vdop_idx = (uint8_t)(field_count - 1);

    if (fields[pdop_idx][0] != '\0')
    {
      g_nmea_gnss.pdop_m = (float)atof(fields[pdop_idx]);
      g_nmea_gnss.valid_pdop = 1;
    }

    if (fields[hdop_idx][0] != '\0')
    {
      g_nmea_gnss.precision_m = (float)atof(fields[hdop_idx]);
      g_nmea_gnss.valid_precision = 1;
    }

    if (fields[vdop_idx][0] != '\0')
    {
      g_nmea_gnss.vdop_m = (float)atof(fields[vdop_idx]);
      g_nmea_gnss.valid_vdop = 1;
    }
  }

  nmea_mark_sentence(NMEA_SENTENCE_GSA_BIT);

  if (g_nmea_gnss.fix_mode <= 1)
    return NMEA_ERR_NO_FIX;

  if (g_nmea_gnss.fix_quality == 0)
    g_nmea_gnss.fix_quality = 1;

  return NMEA_OK;
}

int nmea_parse_gxgsv(const uint8_t *buf, uint16_t len)
{
  char sentence[256];
  int ret = nmea_parse_by_type(buf, len, "GSV", sentence, (uint16_t)sizeof(sentence));
  if (ret != NMEA_OK)
    return ret;

  char payload[256];
  char *fields[24] = {0};
  uint8_t field_count = 0;
  ret = nmea_sentence_to_fields(sentence, payload, (uint16_t)sizeof(payload), fields, (uint8_t)24, &field_count);
  if (ret != NMEA_OK)
    return ret;

  if (field_count < 4 || !nmea_header_match(fields[0], "GSV"))
    return NMEA_ERR_FORMAT;

  g_nmea_gnss.satellite_in_view = (uint8_t)atoi(fields[3]);
  g_nmea_gnss.valid_satellite_in_view = 1;
  nmea_mark_sentence(NMEA_SENTENCE_GSV_BIT);
  return NMEA_OK;
}

int nmea_parse_gxrmc(const uint8_t *buf, uint16_t len)
{
  char sentence[256];
  int ret = nmea_parse_by_type(buf, len, "RMC", sentence, (uint16_t)sizeof(sentence));
  if (ret != NMEA_OK)
    return ret;

  char payload[256];
  char *fields[24] = {0};
  uint8_t field_count = 0;
  ret = nmea_sentence_to_fields(sentence, payload, (uint16_t)sizeof(payload), fields, (uint8_t)24, &field_count);
  if (ret != NMEA_OK)
    return ret;

  if (field_count < 10 || !nmea_header_match(fields[0], "RMC"))
    return NMEA_ERR_FORMAT;

  nmea_parse_time_hms(fields[1]);
  nmea_parse_lat_lon(fields[3], fields[4], fields[5], fields[6]);

  if (fields[7][0] != '\0')
  {
    g_nmea_gnss.speed_knots = (float)atof(fields[7]);
    g_nmea_gnss.valid_speed = 1;
    g_nmea_gnss.speed_kmh = g_nmea_gnss.speed_knots * 1.852f;
    g_nmea_gnss.valid_speed_kmh = 1;
  }

  if (fields[8][0] != '\0')
  {
    g_nmea_gnss.course_deg = (float)atof(fields[8]);
    g_nmea_gnss.valid_course = 1;
  }

  nmea_parse_date_ddmmyy(fields[9]);
  nmea_mark_sentence(NMEA_SENTENCE_RMC_BIT);

  if (fields[2][0] == 'A')
  {
    if (g_nmea_gnss.fix_quality == 0)
      g_nmea_gnss.fix_quality = 1;
    return NMEA_OK;
  }

  return NMEA_ERR_NO_FIX;
}

int nmea_parse_gxvtg(const uint8_t *buf, uint16_t len)
{
  char sentence[256];
  int ret = nmea_parse_by_type(buf, len, "VTG", sentence, (uint16_t)sizeof(sentence));
  if (ret != NMEA_OK)
    return ret;

  char payload[256];
  char *fields[20] = {0};
  uint8_t field_count = 0;
  ret = nmea_sentence_to_fields(sentence, payload, (uint16_t)sizeof(payload), fields, (uint8_t)20, &field_count);
  if (ret != NMEA_OK)
    return ret;

  if (field_count < 8 || !nmea_header_match(fields[0], "VTG"))
    return NMEA_ERR_FORMAT;

  if (fields[1][0] != '\0')
  {
    g_nmea_gnss.course_deg = (float)atof(fields[1]);
    g_nmea_gnss.valid_course = 1;
  }

  if (fields[5][0] != '\0')
  {
    g_nmea_gnss.speed_knots = (float)atof(fields[5]);
    g_nmea_gnss.valid_speed = 1;
  }

  if (fields[7][0] != '\0')
  {
    g_nmea_gnss.speed_kmh = (float)atof(fields[7]);
    g_nmea_gnss.valid_speed_kmh = 1;
  }

  nmea_mark_sentence(NMEA_SENTENCE_VTG_BIT);
  return NMEA_OK;
}

int nmea_parse_gxzda(const uint8_t *buf, uint16_t len)
{
  char sentence[256];
  int ret = nmea_parse_by_type(buf, len, "ZDA", sentence, (uint16_t)sizeof(sentence));
  if (ret != NMEA_OK)
    return ret;

  char payload[256];
  char *fields[16] = {0};
  uint8_t field_count = 0;
  ret = nmea_sentence_to_fields(sentence, payload, (uint16_t)sizeof(payload), fields, (uint8_t)16, &field_count);
  if (ret != NMEA_OK)
    return ret;

  if (field_count < 5 || !nmea_header_match(fields[0], "ZDA"))
    return NMEA_ERR_FORMAT;

  nmea_parse_time_hms(fields[1]);

  if (fields[2][0] != '\0' && fields[3][0] != '\0' && fields[4][0] != '\0')
  {
    g_nmea_gnss.date_d = (uint8_t)atoi(fields[2]);
    g_nmea_gnss.date_m = (uint8_t)atoi(fields[3]);
    g_nmea_gnss.date_year = (uint16_t)atoi(fields[4]);
    g_nmea_gnss.date_y = (uint8_t)(g_nmea_gnss.date_year % 100U);
    g_nmea_gnss.valid_date = 1;
  }

  nmea_mark_sentence(NMEA_SENTENCE_ZDA_BIT);
  return NMEA_OK;
}

int nmea_parse_gxtxt(const uint8_t *buf, uint16_t len)
{
  char sentence[256];
  int ret = nmea_parse_by_type(buf, len, "TXT", sentence, (uint16_t)sizeof(sentence));
  if (ret != NMEA_OK)
    return ret;

  char payload[256];
  char *fields[16] = {0};
  uint8_t field_count = 0;
  ret = nmea_sentence_to_fields(sentence, payload, (uint16_t)sizeof(payload), fields, (uint8_t)16, &field_count);
  if (ret != NMEA_OK)
    return ret;

  if (field_count < 2 || !nmea_header_match(fields[0], "TXT"))
    return NMEA_ERR_FORMAT;

  nmea_mark_sentence(NMEA_SENTENCE_TXT_BIT);
  return NMEA_OK;
}

static void nmea_merge_result(int ret, int *first_err, uint8_t *ok_count)
{
  if (ret == NMEA_OK)
  {
    (*ok_count)++;
    return;
  }

  if (*first_err == NMEA_ERR_FORMAT)
    *first_err = ret;
}

/**
 * Generic NMEA sentence parser
 * Dispatches to specific sentence parsers based on type
 * 
 * @param buf: pointer to NMEA sentence buffer
 * @param len: length of buffer
 * @return: NMEA_OK (0) if success, negative error code on failure
 */
int nmea_parse(const uint8_t *buf, uint16_t len)
{
  if (buf == NULL || len < 20)
    return NMEA_ERR_INVALID_INPUT;

  uint8_t ok_count = 0;
  int first_err = NMEA_ERR_FORMAT;

  nmea_merge_result(nmea_parse_gngga(buf, len), &first_err, &ok_count);
  nmea_merge_result(nmea_parse_gxgll(buf, len), &first_err, &ok_count);
  nmea_merge_result(nmea_parse_gxgsa(buf, len), &first_err, &ok_count);
  nmea_merge_result(nmea_parse_gxgsv(buf, len), &first_err, &ok_count);
  nmea_merge_result(nmea_parse_gxrmc(buf, len), &first_err, &ok_count);
  nmea_merge_result(nmea_parse_gxvtg(buf, len), &first_err, &ok_count);
  nmea_merge_result(nmea_parse_gxzda(buf, len), &first_err, &ok_count);
  nmea_merge_result(nmea_parse_gxtxt(buf, len), &first_err, &ok_count);

  if (ok_count > 0)
    return NMEA_OK;

  return first_err;
}
