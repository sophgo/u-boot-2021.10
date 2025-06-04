#include <common.h>
#include <command.h>
#include <sophon_spacc.h>

static void hexdump(const char *label, const void *buf, size_t len)
{
	size_t i;
	uint8_t *data = (uint8_t *)buf;
	printf("%s (%zu bytes):", label, len);
	for (i = 0; i < len; i++) {
		printf("%x", data[i]);
		if ((i % 16 == 15) || (i == len - 1))
			printf("\n");
	}
}
extern int sophon_spacc_aes(spacc_aes_config_s *aes_config,
			    const unsigned char *dst, const unsigned char *src,
			    size_t size);
static int do_aes(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	printf("AES command\n");
	unsigned char key[32] = { 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61,
				  0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61,
				  0x61, 0x61, 0x61, 0x62, 0x63, 0x64, 0x65,
				  0x66, 0x67, 0x68, 0x61, 0x61, 0x61, 0x61,
				  0x61, 0x61, 0x61, 0x61 };
	unsigned char iv[16] = {
		0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61,
		0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61, 0x61
	};
	unsigned char result[16] = { 0 };
	unsigned char __16B_bin[] = { 0x68, 0x65, 0x6c, 0x6c, 0x6f, 0x0b,
				      0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
				      0x0b, 0x0b, 0x0b, 0x0b };

	struct spacc_aes_config aes_config = {
		.key = (uintptr_t)key,
		.iv = (uintptr_t)iv,
		.mode = SPACC_ALGO_MODE_CBC,
		.key_mode = SPACC_KEY_SIZE_128BITS,
		.action = SPACC_ACTION_ENCRYPTION,
		.otp = SPACC_KEY_SOURCE_DESCRIPTOR
	};
	memset(result, 0, sizeof(result));
	int ret = sophon_spacc_aes(&aes_config, result, __16B_bin,
				   sizeof(__16B_bin));
	hexdump("__16B_bin", __16B_bin, sizeof(__16B_bin));
	hexdump("result", result, sizeof(result));
	if (ret) {
		printf("AES operation failed\n");
		return CMD_RET_FAILURE;
	}

	printf("AES operation succeeded\n");
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(spacc, CONFIG_SYS_MAXARGS, 0, do_aes, "spacc cmd for test",
	   "command [args...]\n");