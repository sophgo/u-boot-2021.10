
#include <sophon_spacc.h>
int sophon_spacc_aes(spacc_aes_config_s *config, const unsigned char *dst,
		     const unsigned char *src, size_t size)
{
	struct arm_smccc_res res = { 0 };
	uint64_t key_len;
	uint64_t arg7 = (u8)config->mode | ((u8)config->key_mode << 2) |
			((u8)config->action << 4) | ((u8)config->otp << 5);
	switch (config->key_mode) {
	case SPACC_KEY_SIZE_64BITS:
		key_len = 8;
		break;
	case SPACC_KEY_SIZE_128BITS:
		key_len = 16;
		break;
	case SPACC_KEY_SIZE_192BITS:
		key_len = 24;
		break;
	case SPACC_KEY_SIZE_256BITS:
		key_len = 32;
		break;
	default:
		return -1;
	}
	printf("key:%p, iv:%p, key_len:%llu\n", (void *)config->key,
	       (void *)config->iv, key_len);
	printf("src:%p, dst:%p, size:%zu\n", (void *)src, (void *)dst, size);
	flush_dcache_all();
	arm_smccc_smc(OPTEE_SMC_CALL_CV_SPACC_EXEC, (unsigned long)src,
		      (unsigned long)dst, size, (unsigned long)config->key,
		      (unsigned long)config->iv, key_len, arg7, &res);
	flush_dcache_all();
	return res.a0;
}