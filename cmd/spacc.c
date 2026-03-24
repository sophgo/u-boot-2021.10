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
	unsigned char result_cbc[16] = { 0 };
	unsigned char result_ecb[16] = { 0 };
	unsigned char result_ctr[16] = { 0 };
	unsigned char __16B_bin[] = { 0x68, 0x65, 0x6c, 0x6c, 0x6f, 0x0b,
				      0x0b, 0x0b, 0x0b, 0x0b, 0x0b, 0x0b,
				      0x0b, 0x0b, 0x0b, 0x0b };

	// AES CBC encryption
	struct spacc_aes_config aes_config = {
		.key = (uintptr_t)key,
		.iv = (uintptr_t)iv,
		.mode = SPACC_ALGO_MODE_CBC,
		.key_mode = SPACC_KEY_SIZE_128BITS,
		.action = SPACC_ACTION_ENCRYPTION,
		.otp = SPACC_KEY_SOURCE_DESCRIPTOR
	};
	memset(result_cbc, 0, sizeof(result_cbc));
	int ret = sophon_spacc_aes(&aes_config, result_cbc, __16B_bin,
				   sizeof(__16B_bin));
	hexdump("__16B_bin", __16B_bin, sizeof(__16B_bin));
	hexdump("result", result_cbc, sizeof(result_cbc));
	if (!ret) {
		printf("AES operation failed\n");
		return CMD_RET_FAILURE;
	}

	printf("AES CBC decryption succeeded(ret = %d)\n", ret);

	//AES ECB encryption
	aes_config.mode = SPACC_ALGO_MODE_ECB;
	memset(result_ecb, 0, sizeof(result_ecb));
	ret = sophon_spacc_aes(&aes_config, result_ecb, __16B_bin, sizeof(__16B_bin));
	hexdump("__16B_bin", __16B_bin, sizeof(__16B_bin));
	hexdump("result", result_ecb, sizeof(result_ecb));
	if (!ret) {
		printf("AES ECB encryption failed\n");
		return CMD_RET_FAILURE;
	}
	printf("AES ECB encryption succeeded(ret = %d)\n", ret);

	//AES CTR encryption
	aes_config.mode = SPACC_ALGO_MODE_CTR;
	memset(result_ctr, 0, sizeof(result_ctr));
	ret = sophon_spacc_aes(&aes_config, result_ctr, __16B_bin, sizeof(__16B_bin));
	hexdump("__16B_bin", __16B_bin, sizeof(__16B_bin));
	hexdump("result", result_ctr, sizeof(result_ctr));
	if (!ret) {
		printf("AES CTR encryption failed\n");
		return CMD_RET_FAILURE;
	}
	printf("AES CTR encryption succeeded(ret = %d)\n", ret);

	//AES CTR decryption
	aes_config.mode = SPACC_ALGO_MODE_CTR;
	aes_config.action = SPACC_ACTION_DECRYPTION;
	memset(result, 0, sizeof(result));
	ret = sophon_spacc_aes(&aes_config, result, result_ctr, sizeof(__16B_bin));
	hexdump("__16B_bin", result_ctr, sizeof(result_ctr));
	hexdump("result", result, sizeof(result));
	if (!ret) {
		printf("AES CTR decryption failed\n");
		return CMD_RET_FAILURE;
	}
	printf("AES CTR decryption succeeded\n");

	//AES ECB decryption
	aes_config.mode = SPACC_ALGO_MODE_ECB;
	aes_config.action = SPACC_ACTION_DECRYPTION;
	memset(result, 0, sizeof(result));
	ret = sophon_spacc_aes(&aes_config, result, result_ecb, sizeof(__16B_bin));
	hexdump("result", result, sizeof(result));
	if (!ret) {
		printf("AES ECB decryption failed\n");
		return CMD_RET_FAILURE;
	}
	printf("AES ECB decryption succeeded\n");

	//AES CBC decryption
	aes_config.mode = SPACC_ALGO_MODE_CBC;
	aes_config.action = SPACC_ACTION_DECRYPTION;
	memset(result, 0, sizeof(result));
	ret = sophon_spacc_aes(&aes_config, result, result_cbc, sizeof(__16B_bin));
	hexdump("result", result, sizeof(result));
	if (!ret) {
		printf("AES CBC decryption failed\n");
		return CMD_RET_FAILURE;
	}
	printf("AES CBC decryption succeeded\n");

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(spacc, CONFIG_SYS_MAXARGS, 0, do_aes, "spacc cmd for test",
	   "command [args...]\n");