#include "board_manage.h"
#include "ota_service_task.h"
#include "at_protocol_handler.h"
#include "stm32h7xx_hal.h"
#include "uart_manage_port.h"

int32_t craner_at_handler(const uint8_t *buf, uint16_t len,at_reply_send_fn_t reply_fn)
{
	static const char at_prefix[] = "craner#AT";
	const uint16_t at_prefix_len = (uint16_t)(sizeof(at_prefix) - 1U);
	uint16_t index = 0U;
	char tmp[256];
	uint16_t tlen;
	at_reply_send_fn_t send_fn = (reply_fn != NULL) ? reply_fn : shell_inform_send;

	/* Skip leading whitespace */
	while (index < len)
	{
		if ((buf[index] != ' ') && (buf[index] != '\t') && (buf[index] != '\r') && (buf[index] != '\n'))
		{
			break;
		}
		index++;
	}

	/* Check if remaining length is sufficient */
	if ((len - index) < at_prefix_len)
	{
		return AT_PREFIX_NOT_MATCH;
	}

	/* Compare prefix */
	if (memcmp(&buf[index], at_prefix, at_prefix_len) == 0)
	{
		/* Convert to string for easier parsing */
		tlen = (len > 255U) ? 255U : len;
		memcpy(tmp, buf, tlen);
		tmp[tlen] = '\0';

		/* Handle OTA START command */
		if (strstr(tmp, "craner#AT+OTASTART") != NULL)
		{
			int ret = ota_start_transfer_callback();
			if (ret == 0)
			{
				const char ack[] = "craner#OK\r\n";
				(void)send_fn((uint8_t *)ack, (uint16_t)(sizeof(ack) - 1U));
				return AT_OK;
			}
			{
				const char err[] = "craner#ERROR\r\n";
				(void)send_fn((uint8_t *)err, (uint16_t)(sizeof(err) - 1U));
				return AT_ACTION_EXECUTION_FAILED;
			}
		}

		/* Handle OTA RESET command: abort current session and clear OTA state */
		if (strstr(tmp, "craner#AT+OTARESET") != NULL)
		{
			(void)ota_reset_transfer_callback();
			{
				const char ok[] = "craner#OK\r\n";
				(void)send_fn((uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
			}
			return AT_OK;
		}

		/* Handle system reset command */
		if (strstr(tmp, "craner#AT+SYSRESET") != NULL)
		{
			const char ok[] = "craner#OK\r\n";
			(void)send_fn((uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
			osDelay(100U);
			NVIC_SystemReset();
			return AT_OK;
		}

        /* Must place general command handler at the end, otherwise it may preempt specific command handling */
		if (strstr(tmp, "craner#AT") != NULL)
		{
			const char ok[] = "craner#OK\r\n";
			(void)send_fn((uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
			return AT_OK;
		}

		/* Unknown command */
		{
			const char err[] = "craner#UNKNOWN\r\n";
			(void)send_fn((uint8_t *)err, (uint16_t)(sizeof(err) - 1U));
		}

		return AT_UNKNOWN_CMD;
	}

	return AT_PREFIX_NOT_MATCH;
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

