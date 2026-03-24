#ifndef _SOPHON_SPACC_H_
#define _SOPHON_SPACC_H_

#define OPTEE_SMC_CALL_CV_SPACC_EXEC 0x0300000F
#include <common.h>
#include <log.h>
#include <linux/errno.h>
#include <linux/arm-smccc.h>
#include <cpu_func.h>
typedef enum SPACC_ALGO {
	SPACC_ALGO_AES,
	SPACC_ALGO_DES,
	SPACC_ALGO_TDES,
	SPACC_ALGO_SM4,
	SPACC_ALGO_SHA1,
	SPACC_ALGO_SHA256,
	SPACC_ALGO_BASE64,
} SPACC_ALGO_E;

typedef enum SPACC_ALGO_MODE {
	SPACC_ALGO_MODE_ECB,
	SPACC_ALGO_MODE_CBC, //cv180x Not Supported
	SPACC_ALGO_MODE_CTR, //cv180x Not Supported
	SPACC_ALGO_MODE_OFB, //cv180x Not Supported
} SPACC_ALGO_MODE_E;

typedef enum SPACC_KEY_SIZE {
	SPACC_KEY_SIZE_64BITS,
	SPACC_KEY_SIZE_128BITS,
	SPACC_KEY_SIZE_192BITS,
	SPACC_KEY_SIZE_256BITS,
} SPACC_KEY_SIZE_E;

typedef enum SPACC_ACTION {
	SPACC_ACTION_ENCRYPTION,
	SPACC_ACTION_DECRYPTION,
} SPACC_ACTION_E;

typedef enum SPACC_KEY_SOURCE {
	SPACC_KEY_SOURCE_DESCRIPTOR,
	SPACC_KEY_SOURCE_OTP, // key from otp or efuse
} SPACC_KEY_SOURCE_E;

typedef struct spacc_base64 {
	u32 customer_code;
	u32 action; // 0: Decode, 1: Encode
} spacc_base64_config_s;

typedef struct spacc_base64_inner {
	u64 src;
	u64 dst;
	u64 len;
	u32 customer_code;
	u32 action; // 0: Decode, 1: Encode
} spacc_base64_inner_config_s;

typedef struct spacc_aes_config {
	// data config
	void *src; //src phy address
	size_t len;

	// spacc config
	uintptr_t key;
	uintptr_t iv;
	SPACC_ALGO_MODE_E mode;
	SPACC_KEY_SIZE_E key_mode;
	SPACC_ACTION_E action;
	SPACC_KEY_SOURCE_E otp;
} spacc_aes_config_s;

typedef struct spacc_des_config {
	uintptr_t key;
	uintptr_t iv;
	SPACC_ALGO_MODE_E mode;
	SPACC_ACTION_E action;
} spacc_des_config_s;

int sophon_spacc_aes(spacc_aes_config_s *aes_config, const unsigned char *dst,
		     const unsigned char *src, size_t size);

#endif // _SOPHON_AES_H_