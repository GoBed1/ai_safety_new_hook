#include "at_protocol_handler.h"

#include <stdio.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "ota_manage_port.h"
#include "uart_manage.h"
#include "uart_manage_port.h"

typedef int32_t (*craner_cmd_handler_t)(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn);

typedef struct
{
	const char *cmd;
	craner_cmd_handler_t handler;
} craner_cmd_entry_t;

typedef struct
{
	const uint8_t *full_cmd;
	uint16_t full_cmd_len;
	const uint8_t *cmd;
	uint16_t cmd_len;
} craner_at_command_t;

static uint8_t craner_is_blank(uint8_t ch)
{
	return (uint8_t)((ch == ' ') || (ch == '\t') || (ch == '\r') || (ch == '\n'));
}

static int32_t craner_find_at_command(const uint8_t *buf, uint16_t len, craner_at_command_t *at_cmd)
{
	static const char craner_prefix[] = "craner#";
	const uint16_t craner_prefix_len = (uint16_t)(sizeof(craner_prefix) - 1U);
	uint16_t index = 0U;
	uint16_t full_cmd_len;

	if ((buf == NULL) || (at_cmd == NULL))
	{
		return AT_PREFIX_NOT_MATCH;
	}

	while ((index < len) && (craner_is_blank(buf[index]) != 0U))
	{
		index++;
	}

	if ((len - index) < craner_prefix_len)
	{
		return AT_PREFIX_NOT_MATCH;
	}

	if (memcmp(&buf[index], craner_prefix, craner_prefix_len) != 0)
	{
		return AT_PREFIX_NOT_MATCH;
	}

	full_cmd_len = (uint16_t)(len - index);
	while ((full_cmd_len > craner_prefix_len) &&
	       (craner_is_blank(buf[index + full_cmd_len - 1U]) != 0U))
	{
		full_cmd_len--;
	}

	at_cmd->full_cmd = &buf[index];
	at_cmd->full_cmd_len = full_cmd_len;
	at_cmd->cmd = &buf[index + craner_prefix_len];
	at_cmd->cmd_len = (uint16_t)(full_cmd_len - craner_prefix_len);

	return AT_OK;
}

static void craner_reply_ok(at_reply_send_fn_t reply_fn)
{
	const char ack[] = "craner#OK\r\n";
	(void)reply_fn((uint8_t *)ack, (uint16_t)(sizeof(ack) - 1U));
}

static void craner_reply_error(at_reply_send_fn_t reply_fn)
{
	const char err[] = "craner#ERROR\r\n";
	(void)reply_fn((uint8_t *)err, (uint16_t)(sizeof(err) - 1U));
}

static void craner_reply_cmd_error(at_reply_send_fn_t reply_fn, const uint8_t *cmd, uint16_t len)
{
	const char prefix[] = "craner#";
	const char suffix[] = " ERROR\r\n";

	(void)reply_fn((uint8_t *)prefix, (uint16_t)(sizeof(prefix) - 1U));
	if ((len > 2U) && (cmd[0] == 'A') && (cmd[1] == 'T'))
	{
		(void)reply_fn((uint8_t *)&cmd[2], (uint16_t)(len - 2U));
	}
	else
	{
		(void)reply_fn((uint8_t *)cmd, len);
	}
	(void)reply_fn((uint8_t *)suffix, (uint16_t)(sizeof(suffix) - 1U));
}

static int32_t craner_cmd_at(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn)
{
	(void)cmd;
	(void)len;
	craner_reply_ok(reply_fn);
	return AT_OK;
}

static int32_t craner_cmd_ota_boot(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn)
{
	if (ota_boot_callback() == 0)
	{
		craner_reply_ok(reply_fn);
		return AT_OK;
	}

	craner_reply_cmd_error(reply_fn, cmd, len);
	return AT_ACTION_EXECUTION_FAILED;
}

static int32_t craner_cmd_ota_request(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn)
{
	if (ota_request_callback() == 0)
	{
		craner_reply_ok(reply_fn);
		return AT_OK;
	}

	craner_reply_cmd_error(reply_fn, cmd, len);
	return AT_ACTION_EXECUTION_FAILED;
}

static int32_t craner_cmd_ota_cancel(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn)
{
	if (ota_cancel_callback() == 0)
	{
		craner_reply_ok(reply_fn);
		return AT_OK;
	}

	craner_reply_cmd_error(reply_fn, cmd, len);
	return AT_ACTION_EXECUTION_FAILED;
}

static int32_t craner_cmd_ota_lock(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn)
{
	if (ota_lock_callback() == 0)
	{
		craner_reply_ok(reply_fn);
		return AT_OK;
	}

	craner_reply_cmd_error(reply_fn, cmd, len);
	return AT_ACTION_EXECUTION_FAILED;
}

static int32_t craner_cmd_ota_info(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn)
{
	char info[48];
	uint32_t active_slot;
	uint32_t ota_request;
	uint32_t rollback_count;
	uint32_t rollback_threshold;
	int info_len;

	(void)cmd;
	(void)len;

	if (ota_get_info(&active_slot, &ota_request, &rollback_count, &rollback_threshold) != 0)
	{
		craner_reply_cmd_error(reply_fn, cmd, len);
		return AT_ACTION_EXECUTION_FAILED;
	}

	info_len = snprintf(info,
	                    sizeof(info),
	                    "craner#+OTAINFO:%lu,%lu,%lu,%lu\r\n",
	                    (unsigned long)active_slot,
	                    (unsigned long)ota_request,
	                    (unsigned long)rollback_count,
	                    (unsigned long)rollback_threshold);
	if ((info_len <= 0) || ((uint32_t)info_len >= sizeof(info)))
	{
		craner_reply_cmd_error(reply_fn, cmd, len);
		return AT_ACTION_EXECUTION_FAILED;
	}

	(void)reply_fn((uint8_t *)info, (uint16_t)info_len);
	return AT_OK;
}

static int32_t craner_cmd_fw_time(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn)
{
	static const char fw_time[] = "craner#FWTIME:" __DATE__ " " __TIME__ "\r\n";

	(void)cmd;
	(void)len;
	(void)reply_fn((uint8_t *)fw_time, (uint16_t)(sizeof(fw_time) - 1U));
	return AT_OK;
}

static int32_t craner_cmd_sys_reset(const uint8_t *cmd, uint16_t len, at_reply_send_fn_t reply_fn)
{
	(void)cmd;
	(void)len;
	(void)reply_fn;
	NVIC_SystemReset();
	return AT_OK;
}

static const craner_cmd_entry_t craner_cmd_table[] = {
	{"AT", craner_cmd_at},
	{"AT+OTABOOT", craner_cmd_ota_boot},
	{"AT+OTAREQUEST", craner_cmd_ota_request},
	{"AT+OTACANCEL", craner_cmd_ota_cancel},
	{"AT+OTALOCK", craner_cmd_ota_lock},
	{"AT+OTAINFO", craner_cmd_ota_info},
	{"AT+FWTIME", craner_cmd_fw_time},
	{"AT+SYSRESET", craner_cmd_sys_reset},
};

int32_t craner_at_handler(const uint8_t *buf, uint16_t len,at_reply_send_fn_t reply_fn)
{
	at_reply_send_fn_t send_fn = (reply_fn != NULL) ? reply_fn : shell_inform_send;
	craner_at_command_t at_cmd;

	if (craner_find_at_command(buf, len, &at_cmd) != AT_OK)
	{
		return AT_PREFIX_NOT_MATCH;
	}

	for (uint16_t i = 0U; i < (uint16_t)(sizeof(craner_cmd_table) / sizeof(craner_cmd_table[0])); ++i)
	{
		uint16_t table_cmd_len = (uint16_t)strlen(craner_cmd_table[i].cmd);
		if ((at_cmd.cmd_len == table_cmd_len) && (memcmp(at_cmd.cmd, craner_cmd_table[i].cmd, table_cmd_len) == 0))
		{
			return craner_cmd_table[i].handler(at_cmd.cmd, at_cmd.cmd_len, send_fn);
		}
	}

	craner_reply_error(send_fn);

	return AT_UNKNOWN_CMD;
}

int32_t usr_at_handler(const uint8_t *buf, uint16_t len)
{
	static const char at_prefix[] = "usr.cn#AT";
	const uint16_t at_prefix_len = (uint16_t)(sizeof(at_prefix) - 1U);
	uint16_t index = 0U;

	while (index < len)
	{
		if ((buf[index] != ' ') && (buf[index] != '\t') && (buf[index] != '\r') && (buf[index] != '\n'))
		{
			break;
		}
		index++;
	}

	if ((len - index) < at_prefix_len)
	{
		return AT_PREFIX_NOT_MATCH;
	}

	if (memcmp(&buf[index], at_prefix, at_prefix_len) == 0)
	{
		/* Forward the original payload to the 4G module first. */
		(void)uart_manage_dma_send_by_name("4g", (uint8_t *)buf, len);

		/* Add trailing CRLF if the command does not already end with it. */
		if ((len > 0U) && (buf[len - 1U] != '\r') && (buf[len - 1U] != '\n'))
		{
			static const uint8_t crlf[] = "\r\n";
			const uint16_t crlf_len = (uint16_t)(sizeof(crlf) - 1U);
			(void)uart_manage_dma_send_by_name("4g", (uint8_t *)crlf, crlf_len);
		}

		return AT_OK;
	}

	return AT_PREFIX_NOT_MATCH;
}
