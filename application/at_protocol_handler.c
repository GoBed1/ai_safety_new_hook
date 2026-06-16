#include "at_protocol_handler.h"

#include <stdio.h>
#include <string.h>

#include "ota_flash_service.h"
#include "stm32h7xx_hal.h"
#include "uart_manage.h"

#define CRANER_AT_PREFIX        "craner#AT"
#define CRANER_OTA_REQUEST_CMD  "craner#AT+OTA=1"
#define CRANER_REPLY_OK         "craner#OK\r\n"
#define CRANER_REPLY_ERROR      "craner#ERROR\r\n"

static uint16_t at_skip_space(const uint8_t *buf, uint16_t len, uint16_t index)
{
	while (index < len)
	{
		if ((buf[index] != ' ') && (buf[index] != '\t') &&
			(buf[index] != '\r') && (buf[index] != '\n'))
		{
			break;
		}
		index++;
	}

	return index;
}

static uint8_t at_payload_equals(const uint8_t *buf, uint16_t len, const char *cmd)
{
	uint16_t cmd_len;

	if ((buf == NULL) || (cmd == NULL))
	{
		return 0U;
	}

	cmd_len = (uint16_t)strlen(cmd);
	if (len < cmd_len)
	{
		return 0U;
	}

	if (memcmp(buf, cmd, cmd_len) != 0)
	{
		return 0U;
	}

	for (uint16_t i = cmd_len; i < len; ++i)
	{
		if ((buf[i] != ' ') && (buf[i] != '\t') &&
			(buf[i] != '\r') && (buf[i] != '\n'))
		{
			return 0U;
		}
	}

	return 1U;
}

static void at_send_reply(at_reply_send_fn_t send_fn, const char *reply)
{
	if ((send_fn != NULL) && (reply != NULL))
	{
		(void)send_fn((uint8_t *)reply, (uint16_t)strlen(reply));
	}
}

static int32_t craner_request_ota_update(void)
{
	ota_flash_slot_t target_slot;
	ota_flash_status_t status;

	status = ota_flash_service_init();
	if (status != OTA_FLASH_OK)
	{
		printf("[OTA][E] flash service init failed: %d\r\n", (int)status);
		return AT_ACTION_EXECUTION_FAILED;
	}

	target_slot = ota_flash_get_inactive_slot();
	status = ota_flash_request_ota(target_slot);
	if (status != OTA_FLASH_OK)
	{
		printf("[OTA][E] request ota failed: slot=%d status=%d\r\n", (int)target_slot, (int)status);
		return AT_ACTION_EXECUTION_FAILED;
	}

	printf("[OTA][I] request ota ok: target_slot=%d\r\n", (int)target_slot);
	return AT_OK;
}

int32_t craner_at_handler(const uint8_t *buf, uint16_t len,at_reply_send_fn_t reply_fn)
{
	const uint16_t at_prefix_len = (uint16_t)(sizeof(CRANER_AT_PREFIX) - 1U);
	uint16_t index = 0U;

	if ((buf == NULL) || (len == 0U))
	{
		return AT_PREFIX_NOT_MATCH;
	}

	/* Skip leading whitespace */
	index = at_skip_space(buf, len, index);

	/* The 4G/MQTT text channel prefixes normal AT payloads with "1,". */
	if (((uint16_t)(len - index) >= 2U) && (buf[index] == '1') && (buf[index + 1U] == ','))
	{
		index += 2U;
		index = at_skip_space(buf, len, index);
	}

	/* Check if remaining length is sufficient */
	if ((len - index) < at_prefix_len)
	{
		return AT_PREFIX_NOT_MATCH;
	}

	/* Compare prefix */
	if (memcmp(&buf[index], CRANER_AT_PREFIX, at_prefix_len) == 0)
	{
		const uint8_t *payload = &buf[index];
		uint16_t payload_len = (uint16_t)(len - index);

		/* Handle OTA START command */
		if (at_payload_equals(payload, payload_len, CRANER_OTA_REQUEST_CMD) != 0U)
		{
			int32_t ret = craner_request_ota_update();

			if (ret == AT_OK)
			{
				at_send_reply(reply_fn, CRANER_REPLY_OK);
				NVIC_SystemReset();
				return AT_OK;
			}

			at_send_reply(reply_fn, CRANER_REPLY_ERROR);
			return ret;
		}

		/* Handle OTA RESET command: abort current session and clear OTA state */
		// if (strstr(tmp, "craner#AT+OTARESET") != NULL)
		// {
		// 	(void)ota_reset_transfer_callback();
		// 	{
		// 		const char ok[] = "craner#OK\r\n";
		// 		(void)send_fn((uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
		// 	}
		// 	return AT_OK;
		// }

		/* Handle system reset command */
		// if (strstr(tmp, "craner#AT+SYSRESET") != NULL)
		// {
		// 	const char ok[] = "craner#OK\r\n";
		// 	(void)send_fn((uint8_t *)ok, (uint16_t)(sizeof(ok) - 1U));
		// 	osDelay(100U);
		// 	NVIC_SystemReset();
		// 	return AT_OK;
		// }

        /* Must place general command handler at the end, otherwise it may preempt specific command handling */
		at_send_reply(reply_fn, CRANER_REPLY_OK);
		return AT_OK;
	}

	return AT_PREFIX_NOT_MATCH;
}

// 专门处理发给 有人(USR) 4G 模组的 AT 指令
int32_t usr_at_handler(const uint8_t *buf, uint16_t len)
{
    char usr_at_buf[256];
    
    if ((buf == NULL) || (len == 0U))
    {
        return 0U;
    }

    uint16_t safe_len = len;
    if (safe_len > 230) {
        safe_len = 230; 
    }

    // 组装免切 AT 指令格式： "usr.cn#" + 指令 + "\r\n"
    int at_len = snprintf(usr_at_buf, sizeof(usr_at_buf), "usr.cn#%.*s", safe_len, buf);
    
    // 3. 智能补全回车换行符 (如果上位机漏发了，单片机帮忙兜底补上)
    if (usr_at_buf[at_len - 1] != '\n') 
    {
        usr_at_buf[at_len++] = '\r';
        usr_at_buf[at_len++] = '\n';
        usr_at_buf[at_len] = '\0';
    }

    printf("[INFO] Auto-Wrap USR AT: %s", usr_at_buf);

    (void)uart_manage_dma_send_by_name("4g", (uint8_t *)usr_at_buf, at_len);

    return 1U;
}

