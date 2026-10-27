// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <command.h>
#include <stdlib.h>
#include <stdarg.h>
#include <malloc.h>
#include <mmio.h>
#include <linux/arm-smccc.h>
#include <linux/ctype.h>
#include <cpu_func.h>

#include "cvitek/84x6_otp_defines.h"

#define OTP_DEBUG 0

#define _cc_trace(fmt, ...) __trace("", __FILE__, __func__, __LINE__, fmt, ##__VA_ARGS__)
#define _cc_error(fmt, ...) __trace("ERROR:", __FILE__, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ERROR(fmt, ...) __trace("ERROR:", __FILE__, __func__, __LINE__, fmt, ##__VA_ARGS__)

#if OTP_DEBUG

#define VERBOSE(fmt, ...) __trace("VERBOSE:", __FILE__, __func__, __LINE__, fmt, ##__VA_ARGS__)

static int __trace(const char *prefix, const char *path, const char *func, int lineno, const char *fmt, ...)
{
	va_list ap;
	int ret;

	printf("[%s%s:%s:%d] ", prefix, path, func, lineno);
	if (!fmt || fmt[0] == '\0') {
		ret = printf("\n");
	} else {
		va_start(ap, fmt);
		ret = vprintf(fmt, ap);
		va_end(ap);
	}

	return ret;
}
#else

#define VERBOSE(fmt, ...)

static int __trace(const char *prefix, const char *path, const char *func, int lineno, const char *fmt, ...)
{
	return 0;
}
#endif

static int hex2bytes(const char *hex, unsigned char *buf, int buf_size)
{
	int i, total = 0;
	char tmp[3];

	memset(buf, 0, buf_size);

	for (i = 0; i < buf_size; i++) {
		if (!hex[0] || !hex[1])
			break;

		tmp[0] = hex[0];
		tmp[1] = hex[1];
		tmp[2] = '\0';

		buf[i] = simple_strtoul(tmp, NULL, 16);
		hex += 2;
		total += 1;
	}

	return total;
}

/*
 * Base address
 */
#define SYSTEM_OTP_BASE			0x27100000
#define SYSTEM_OTP2_OFFSET		0x0

#define OPTEE_SMC_CALL_CV_OTP3_READ			0x03000006
#define OPTEE_SMC_CALL_CV_OTP3_WRITE		0x03000007
#define OPTEE_SMC_CALL_CV_OTP3_CSK_SELECT	0x03000014

// ===========================================================================
// OTP driver implementation
// ===========================================================================
static inline uint32_t otp2_read_by_line(uint32_t line)
{
	VERBOSE("otp2 read 0x%lx\n", (SYSTEM_OTP_BASE + SYSTEM_OTP2_OFFSET + (line << 2)));
	return mmio_read_32(SYSTEM_OTP_BASE + SYSTEM_OTP2_OFFSET + (line << 2));
}

static inline void otp2_program_by_line(uint32_t line, uint32_t value)
{
	VERBOSE("otp2 program 0x%x : 0x%x\n", (SYSTEM_OTP_BASE + SYSTEM_OTP2_OFFSET + (line << 2)), value);
	mmio_write_32(SYSTEM_OTP_BASE + SYSTEM_OTP2_OFFSET + (line << 2), value);
}

static int otp3_read_by_line(uint32_t line, uint32_t *value)
{
	struct arm_smccc_res res = { 0 };
	uint32_t addr = line << 2;

	flush_dcache_all();
	arm_smccc_smc(OPTEE_SMC_CALL_CV_OTP3_READ, 0, addr, 1, (unsigned long)value, 0, 0, 0, &res);
	if (res.a0 < 0) {
		ERROR("smc OPTEE_SMC_CALL_CV_OTP3_READ failed (%ld)\n", res.a0);
		return -1;
	}

	invalidate_dcache_all();
	return 0;
}

static int otp3_program_by_line(uint32_t line, uint32_t value)
{
	struct arm_smccc_res res = { 0 };
	uint32_t addr = line << 2;

	flush_dcache_all();
	arm_smccc_smc(OPTEE_SMC_CALL_CV_OTP3_WRITE, 0, addr, 1, (unsigned long)&value, 0, 0, 0, &res);
	if (res.a0 < 0) {
		ERROR("smc OPTEE_SMC_CALL_CV_OTP3_WRITE failed (%ld)\n", res.a0);
		return -1;
	}

	return 0;
}

static int otp3_read_by_name(int idx, void *buf)
{
	struct arm_smccc_res res = { 0 };
	size_t area_size = cvi_otp_areas[idx].size / 4;
	uint32_t addr = cvi_otp_areas[idx].addr;

	flush_dcache_all();
	arm_smccc_smc(OPTEE_SMC_CALL_CV_OTP3_READ, 0, addr, area_size, (unsigned long)buf, 0, 0, 0, &res);
	if (res.a0 < 0) {
		ERROR("smc OPTEE_SMC_CALL_CV_OTP3_READ failed (%ld)\n", res.a0);
		return -1;
	}

	invalidate_dcache_all();
	return 0;
}

static int otp3_program_by_name(int idx, const void *buf)
{
	uint32_t value;
	size_t area_size = cvi_otp_areas[idx].size / 4;
	uint32_t addr = cvi_otp_areas[idx].addr;
	struct arm_smccc_res res = { 0 };

	flush_dcache_all();
	for (int i = 0; i < area_size; i++) {
		memcpy(&value, buf + 4 * i, sizeof(value));
		arm_smccc_smc(OPTEE_SMC_CALL_CV_OTP3_WRITE, 0, addr + 4 * i, 1,
			      (unsigned long)&value, 0, 0, 0, &res);
		if (res.a0 < 0) {
			printf("smc OPTEE_SMC_CALL_CV_OTP3_WRITE failed (%ld)\n", res.a0);
			return -1;
		}
	}

	return 0;
}

static int find_otp_area_by_name(const char *name)
{
	for (int i = 0; i < ARRAY_SIZE(cvi_otp_areas); i++) {
		if (!cvi_otp_areas[i].name)
			continue;

		if (!strcmp(name, cvi_otp_areas[i].name))
			return i;
	}

	return -1;
}

static int do_otp3_read(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	uint32_t line, value;

	if (argc != 2)
		return CMD_RET_USAGE;

	line = simple_strtoul(argv[1], NULL, 0);
	if (line > 127) {
		printf("line %d out of range\n", line);
		return CMD_RET_USAGE;
	}

	if (otp3_read_by_line(line, &value) < 0) {
		printf("otp3 read failed\n");
		return CMD_RET_SUCCESS;
	}

	printf("Read otp3 line[0x%04x] : 0x%08x\n", line, value);

	return CMD_RET_SUCCESS;
}

static int do_otp3_program(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	uint32_t line, value;

	if (argc != 3)
		return CMD_RET_USAGE;

	line = simple_strtoul(argv[1], NULL, 0);
	if (line > 183) {
		printf("line %d out of range\n", line);
		return CMD_RET_USAGE;
	}

	value = simple_strtoul(argv[2], NULL, 16);

	if (otp3_program_by_line(line, value) < 0) {
		printf("otp3_program_by_line failed\n");
		return CMD_RET_USAGE;
	}

	return CMD_RET_SUCCESS;
}

static int do_otp2_read(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	uint32_t line, value;

	if (argc != 2)
		return CMD_RET_USAGE;

	line = simple_strtoul(argv[1], NULL, 0);
	if (line > 383) {
		printf("line %d overflow\n", line);
		return CMD_RET_USAGE;
	}

	value = otp2_read_by_line(line);
	printf("Read otp2 line[0x%04x] : 0x%08x\n", line, value);

	return CMD_RET_SUCCESS;
}

static int do_otp2_program(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	uint32_t line, value;

	if (argc != 3)
		return CMD_RET_USAGE;

	line = simple_strtoul(argv[1], NULL, 0);
	if (line > 383) {
		printf("line %d out of range\n", line);
		return CMD_RET_USAGE;
	}

	value = simple_strtoul(argv[2], NULL, 16);

	otp2_program_by_line(line, value);

	return CMD_RET_SUCCESS;
}

static int do_otp_read(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int idx;
	unsigned char buf[32] = {0};

	if (argc != 2)
		return CMD_RET_USAGE;

	idx = find_otp_area_by_name(argv[1]);
	if (idx < 0)
		return CMD_RET_USAGE;

	if (otp3_read_by_name(idx, buf) < 0)
		return CMD_RET_FAILURE;

	print_buffer(0, buf, 1, cvi_otp_areas[idx].size, 0);

	return CMD_RET_SUCCESS;
}

static int do_otp_program(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int idx;
	size_t size;
	unsigned char buf[128] = { 0 };

	if (argc != 3)
		return CMD_RET_USAGE;

	idx = find_otp_area_by_name(argv[1]);
	if (idx < 0)
		return CMD_RET_USAGE;

	size = hex2bytes(argv[2], buf, cvi_otp_areas[idx].size);
	if (size <= 0)
		return CMD_RET_USAGE;

	printf("Write OTP %s(%d) with:\n", cvi_otp_areas[idx].name, idx);
	print_buffer(0, buf, 1, size, 0);

	if (otp3_program_by_name(idx, buf) < 0) {
		printf("Failed to write %s\n", cvi_otp_areas[idx].name);
		return CMD_RET_FAILURE;
	}

	return CMD_RET_SUCCESS;
}

static int do_otp_lock(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int idx;
	char rw;
	int shift;

	if (argc != 3)
		return CMD_RET_USAGE;

	idx = find_otp_area_by_name(argv[1]);
	if (idx < 0) {
		printf("Unknown otp area: %s\n", argv[1]);
		return CMD_RET_USAGE;
	}

	if (!strcmp(argv[2], "r"))
		rw = 'r';
	else if (!strcmp(argv[2], "w"))
		rw = 'w';
	else {
		printf("Invalid lock type: %s (expect r or w)\n", argv[2]);
		return CMD_RET_USAGE;
	}

	if (rw == 'r')
		shift = (int)cvi_otp_areas[idx].rlock_shift;
	else
		shift = (int)cvi_otp_areas[idx].wlock_shift;

	if (shift < 0) {
		printf("This otp area does not support %c-lock\n", rw);
		return CMD_RET_USAGE;
	}

	if (otp3_program_by_line(cvi_otp_areas[idx].lock_offset, 1u << shift) < 0) {
		printf("otp3_program_by_line failed\n");
		return CMD_RET_USAGE;
	}

	return CMD_RET_SUCCESS;
}

static int do_otp_isLocked(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int idx;
	uint32_t value;
	int rshift, wshift;

	if (argc != 2)
		return CMD_RET_USAGE;

	idx = find_otp_area_by_name(argv[1]);
	if (idx < 0) {
		printf("Unknown otp area: %s\n", argv[1]);
		return CMD_RET_USAGE;
	}

	if (otp3_read_by_line(cvi_otp_areas[idx].lock_offset, &value) < 0) {
		printf("otp3_read_by_line failed\n");
		return CMD_RET_FAILURE;
	}

	rshift = (int)cvi_otp_areas[idx].rlock_shift;
	wshift = (int)cvi_otp_areas[idx].wlock_shift;

	printf("OTP area %s lock status (otp3 line 0x%x): ",
	       cvi_otp_areas[idx].name, cvi_otp_areas[idx].lock_offset);

	if (rshift < 0)
		printf("r=N/A ");
	else
		printf("r=%s ", (value & (1u << rshift)) ? "locked" : "unlocked");

	if (wshift < 0)
		printf("w=N/A ");
	else
		printf("w=%s ", (value & (1u << wshift)) ? "locked" : "unlocked");

	printf("\n");

	return CMD_RET_SUCCESS;
}

static int do_otp_enableSecureboot(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	uint32_t sign, encrypt, kpub_sel, ldr_sel;
	uint32_t value = 0;

	if (argc != 5)
		return CMD_RET_USAGE;

	sign = simple_strtoul(argv[1], NULL, 0);
	encrypt = simple_strtoul(argv[2], NULL, 0);
	kpub_sel = simple_strtoul(argv[3], NULL, 0);
	ldr_sel = simple_strtoul(argv[4], NULL, 0);

	if (sign > 1 || encrypt > 1 || kpub_sel > 7 || ldr_sel > 7)
		return CMD_RET_USAGE;

	value = (sign << CVI_OTP_TEE_SCS_ENABLE_SHIFT) |
	       (encrypt << CVI_OTP_BOOT_LOADER_ENCRYPTION) |
	       (kpub_sel << CVI_OTP_ROOT_PUBLIC_KEY_SELECTION_SHIFT) |
	       (ldr_sel << CVI_OTP_LDR_KEY_SELECTION_SHIFT);

	if (otp3_program_by_line(CVI_OTP_SCS_CONFIG_LINE, value) < 0) {
		printf("otp3_program_by_line failed\n");
		return CMD_RET_FAILURE;
	}
	printf("Secureboot: SCS_CONFIG = 0x%08x\n", value);

	return CMD_RET_SUCCESS;
}

static int do_otp_isSecurebootEnabled(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	uint32_t value;
	uint32_t sign, encrypt, kpub_sel, ldr_sel;

	if (argc != 1)
		return CMD_RET_USAGE;

	if (otp3_read_by_line(CVI_OTP_SCS_CONFIG_LINE, &value) < 0) {
		printf("otp3_read_by_line failed\n");
		return CMD_RET_FAILURE;
	}

	sign = (value >> CVI_OTP_TEE_SCS_ENABLE_SHIFT) & 0x3;
	encrypt = (value >> CVI_OTP_BOOT_LOADER_ENCRYPTION) & 0x3;
	kpub_sel = (value >> CVI_OTP_ROOT_PUBLIC_KEY_SELECTION_SHIFT) & 0x7;
	ldr_sel = (value >> CVI_OTP_LDR_KEY_SELECTION_SHIFT) & 0x7;

	printf("Secureboot: SCS_CONFIG = 0x%08x\n", value);

	if (!sign)
		printf("sign: disabled, encrypt: disabled\n");
	else if (encrypt)
		printf("sign: enabled, encrypt: enabled\n");
	else
		printf("sign: enabled, encrypt: disabled\n");

	printf("kpub_hash_selection: %u\n", kpub_sel);
	printf("ldr_key_selection:   %u\n", ldr_sel);

	return CMD_RET_SUCCESS;
}

static int do_otp_enableJtag(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	if (argc != 1)
		return CMD_RET_USAGE;

	if (otp3_program_by_line(CVI_OTP_JTAG_GATE_LINE, 0x1) < 0) {
		printf("otp3_program_by_line failed\n");
		return CMD_RET_FAILURE;
	}

	printf("enabled jtag\n");
	return CMD_RET_SUCCESS;
}

static int do_otp_isJtagEnabled(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	uint32_t value;

	if (argc != 1)
		return CMD_RET_USAGE;

	if (otp3_read_by_line(CVI_OTP_JTAG_GATE_LINE, &value) < 0) {
		printf("otp3_read_by_line failed\n");
		return CMD_RET_FAILURE;
	}

	printf("JTAG_GATE = 0x%08x (%s)\n",
	       value, value ? "enabled" : "disabled");
	return CMD_RET_SUCCESS;
}

static int do_otp_selectCSK(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int sel;
	struct arm_smccc_res res = { 0 };

	if (argc != 2)
		return CMD_RET_USAGE;

	sel = simple_strtoul(argv[1], NULL, 0);

	if (sel < 0 || sel >= 4) {
		printf("Invalid key selection: %d (expect 0~3)\n", sel);
		return CMD_RET_USAGE;
	}

	arm_smccc_smc(OPTEE_SMC_CALL_CV_OTP3_CSK_SELECT, sel, 0, 0, 0, 0, 0, 0, &res);
	if (res.a0 < 0) {
		printf("smc OPTEE_SMC_CALL_CV_OTP3_CSK_SELECT failed (%ld)\n", res.a0);
		return CMD_RET_FAILURE;
	}

	printf("Selected customer secure key: %d\n", sel);

	return CMD_RET_SUCCESS;
}

static char otp_help_text[] =
	"otp otp2_r <line> - read otp2 line[0~383]\n"
	"otp otp2_w <line> <Hex value> - program otp2 line[0~383]\n"
	"otp otp3_r <line> - read otp3 line[0~127]\n"
	"otp otp3_w <line> <Hex value> - program otp3 line[0~127]\n"
	"otp otp_read <name> - read otp by area name[HASH0_PUBLIC|DBG_PWD|LOADER_EK|DEVICE_EK|CUSTOMER_SECURE_KEY*]\n"
	"otp otp_write <name> <Hex value> - write otp by area name[HASH0_PUBLIC|DBG_PWD|LOADER_EK|DEVICE_EK|CUSTOMER_SECURE_KEY*]\n"
	"otp otp_lock <name> <r|w> - lock otp by area name[HASH0_PUBLIC|DBG_PWD|LOADER_EK|DEVICE_EK|CUSTOMER_SECURE_KEY*]\n"
	"otp otp_isLocked <name> - check otp lock by area name[HASH0_PUBLIC|DBG_PWD|LOADER_EK|DEVICE_EK|CUSTOMER_SECURE_KEY*]\n"
	"otp otp_enableSecureboot <sign> <encrypt> <kpub_hash_selection> <ldr_key_selection> - enable secureboot\n"
	"otp otp_isSecurebootEnabled - is secureboot enabled\n"
	"otp otp_enableJtag - enable jtag\n"
	"otp otp_isJtagEnabled - is jtag enabled\n"
	"otp otp_selectCSK <key_selection> - select customer secure key\n";

U_BOOT_CMD_WITH_SUBCMDS(otp, "OTP sub-system", otp_help_text,
			U_BOOT_SUBCMD_MKENT(otp2_r, 2, 1, do_otp2_read),
			U_BOOT_SUBCMD_MKENT(otp2_w, 3, 1, do_otp2_program),
			U_BOOT_SUBCMD_MKENT(otp3_r, 2, 1, do_otp3_read),
			U_BOOT_SUBCMD_MKENT(otp3_w, 3, 1, do_otp3_program),
			U_BOOT_SUBCMD_MKENT(otp_read, 2, 1, do_otp_read),
			U_BOOT_SUBCMD_MKENT(otp_write, 3, 1, do_otp_program),
			U_BOOT_SUBCMD_MKENT(otp_lock, 3, 1, do_otp_lock),
			U_BOOT_SUBCMD_MKENT(otp_isLocked, 2, 1, do_otp_isLocked),
			U_BOOT_SUBCMD_MKENT(otp_enableSecureboot, 5, 1, do_otp_enableSecureboot),
			U_BOOT_SUBCMD_MKENT(otp_isSecurebootEnabled, 1, 1, do_otp_isSecurebootEnabled),
			U_BOOT_SUBCMD_MKENT(otp_enableJtag, 1, 1, do_otp_enableJtag),
			U_BOOT_SUBCMD_MKENT(otp_isJtagEnabled, 1, 1, do_otp_isJtagEnabled),
			U_BOOT_SUBCMD_MKENT(otp_selectCSK, 2, 1, do_otp_selectCSK));
