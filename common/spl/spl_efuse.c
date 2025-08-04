// SPDX-License-Identifier: GPL-2.0+

#include <common.h>
#include <command.h>
#include <stdlib.h>
#include <stdarg.h>
#include <malloc.h>
#include <mmio.h>
#include <cvi_efuse.h>
#include <asm/system.h>
#include <linux/arm-smccc.h>

#define EFUSE_DEBUG 0

#define _cc_trace(fmt, ...) __trace("", __FILE__, __func__, __LINE__, fmt, ##__VA_ARGS__)
#define _cc_error(fmt, ...) __trace("ERROR:", __FILE__, __func__, __LINE__, fmt, ##__VA_ARGS__)

#define ERROR(fmt, ...) __trace("ERROR:", __FILE__, __func__, __LINE__, fmt, ##__VA_ARGS__)

#if EFUSE_DEBUG

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

// ===========================================================================
// EFUSE implementation
// ===========================================================================
#define EFUSE_SHADOW_REG (EFUSE_BASE + 0x100)
#define SEC_EFUSE_SHADOW_REG (0x020C0100)
#define EFUSE_SIZE 0x100

#define EFUSE_MODE (EFUSE_BASE + 0x0)
#define EFUSE_ADR (EFUSE_BASE + 0x4)
#define EFUSE_DIR_CMD (EFUSE_BASE + 0x8)
#define EFUSE_RD_DATA (EFUSE_BASE + 0xC)
#define EFUSE_STATUS (EFUSE_BASE + 0x10)
#define EFUSE_ONE_WAY (EFUSE_BASE + 0x14)

#define EFUSE_BIT_AREAD BIT(0)
#define EFUSE_BIT_MREAD BIT(1)
#define EFUSE_BIT_PRG BIT(2)
#define EFUSE_BIT_PWR_DN BIT(3)
#define EFUSE_BIT_CMD BIT(4)
#define EFUSE_BIT_BUSY BIT(0)
#define EFUSE_CMD_REFRESH (0x30)

#define OPTEE_SMC_CALL_CV_EFUSE_READ 0x03000006
#define OPTEE_SMC_CALL_CV_EFUSE_WRITE 0x03000007


enum EFUSE_READ_TYPE { EFUSE_AREAD, EFUSE_MREAD };

static void cvi_efuse_wait_for_ready(void)
{
	while (mmio_read_32(EFUSE_STATUS) & EFUSE_BIT_BUSY)
		;
}

static void cvi_efuse_power_on(uint32_t on)
{
	if (on)
		mmio_setbits_32(EFUSE_MODE, EFUSE_BIT_CMD);
	else
		mmio_setbits_32(EFUSE_MODE, EFUSE_BIT_PWR_DN | EFUSE_BIT_CMD);
}

static void cvi_efuse_refresh(void)
{
	mmio_write_32(EFUSE_MODE, EFUSE_CMD_REFRESH);
}

static void cvi_efuse_prog_bit(uint32_t word_addr, uint32_t bit_addr, uint32_t high_row)
{
	uint32_t phy_addr;

	// word_addr: virtual addr, take "lower 6-bits" from 7-bits (0-127)
	// bit_addr: virtual addr, 5-bits (0-31)

	// composite physical addr[11:0] = [11:7]bit_addr + [6:0]word_addr
	phy_addr = ((bit_addr & 0x1F) << 7) | ((word_addr & 0x3F) << 1) | high_row;

	cvi_efuse_wait_for_ready();

	// send efuse program cmd
	mmio_write_32(EFUSE_ADR, phy_addr);
	mmio_write_32(EFUSE_MODE, EFUSE_BIT_PRG | EFUSE_BIT_CMD);
}

static uint32_t cvi_efuse_read_from_phy(uint32_t phy_word_addr, enum EFUSE_READ_TYPE type)
{
	// power on efuse macro
	cvi_efuse_power_on(1);

	cvi_efuse_wait_for_ready();

	mmio_write_32(EFUSE_ADR, phy_word_addr);

	if (type == EFUSE_AREAD) // array read
		mmio_write_32(EFUSE_MODE, EFUSE_BIT_AREAD | EFUSE_BIT_CMD);
	else if (type == EFUSE_MREAD) // margin read
		mmio_write_32(EFUSE_MODE, EFUSE_BIT_MREAD | EFUSE_BIT_CMD);
	else {
		ERROR("EFUSE: Unsupported read type!");
		return (uint32_t)-1;
	}

	cvi_efuse_wait_for_ready();

	return mmio_read_32(EFUSE_RD_DATA);
}

static int cvi_efuse_write_word(uint32_t vir_word_addr, uint32_t val)
{
	uint32_t i, j, row_val, zero_bit;
	uint32_t new_value;
	int err_cnt = 0;

	for (j = 0; j < 2; j++) {
		VERBOSE("EFUSE: Program physical word addr #%d\n", (vir_word_addr << 1) | j);

		// array read by word address
		row_val = cvi_efuse_read_from_phy((vir_word_addr << 1) | j,
						  EFUSE_AREAD); // read low word of word_addr
		zero_bit = val & (~row_val); // only program zero bit

		// program row which bit is zero
		for (i = 0; i < 32; i++) {
			if ((zero_bit >> i) & 1)
				cvi_efuse_prog_bit(vir_word_addr, i, j);
		}

		// check by margin read
		new_value = cvi_efuse_read_from_phy((vir_word_addr << 1) | j, EFUSE_MREAD);
		VERBOSE("%s(): val=0x%x new_value=0x%x\n", __func__, val, new_value);
		if ((val & new_value) != val) {
			err_cnt += 1;
			ERROR("EFUSE: Program bits check failed (%d)!\n", err_cnt);
		}
	}

	cvi_efuse_refresh();

	return err_cnt >= 2 ? -EIO : 0;
}

static void cvi_efuse_init(void)
{
	// power on efuse macro
	cvi_efuse_power_on(1);

	// send refresh cmd to reload all eFuse values to shadow registers
	cvi_efuse_refresh();

	// efuse macro will be auto powered off after refresh cmd, so don't
	// need to turn it off manually
}

void cvi_efuse_dump(uint32_t vir_word_addr)
{
	uint32_t j, val;

	for (j = 0; j < 2; j++) {
		// check by margin read
		val = cvi_efuse_read_from_phy((vir_word_addr << 1) | j, EFUSE_MREAD);
		printf("EFUSE EFUSE_MREAD: Program bits %d check 0x%x\n", j, val);
		val = cvi_efuse_read_from_phy((vir_word_addr << 1) | j, EFUSE_AREAD);
		printf("EFUSE EFUSE_AREAD: Program bits %d check 0x%x\n", j, val);
	}
}

int64_t cvi_efuse_read_from_shadow(uint32_t addr)
{
	if (addr >= EFUSE_SIZE)
		return -EFAULT;

	if (addr % 4 != 0)
		return -EFAULT;

#ifdef __riscv
	return mmio_read_32(SEC_EFUSE_SHADOW_REG + addr);
#else
	return mmio_read_32(EFUSE_SHADOW_REG + addr);
#endif
}

int cvi_efuse_write(uint32_t addr, uint32_t value)
{
	int ret;

	VERBOSE("%s(): 0x%x = 0x%x\n", __func__, addr, value);

	if (addr >= EFUSE_SIZE)
		return -EFAULT;

	if (addr % 4 != 0)
		return -EFAULT;

	ret = cvi_efuse_write_word(addr / 4, value);
	VERBOSE("%s(): ret=%d\n", __func__, ret);
	cvi_efuse_init();
	cvi_efuse_wait_for_ready();

	return ret;
}

// ===========================================================================
// EFUSE API
// ===========================================================================
static struct _CVI_EFUSE_AREA_S {
	CVI_U32 addr;
	CVI_U32 size;
} cvi_efuse_area[] = { [CVI_EFUSE_AREA_USER] = { 0x40, 40 },
		       [CVI_EFUSE_AREA_DEVICE_ID] = { 0x8c, 8 },
		       [CVI_EFUSE_AREA_HASH0_PUBLIC] = { 0xA8, 32 },
		       [CVI_EFUSE_AREA_LOADER_EK] = { 0xD8, 16 },
		       [CVI_EFUSE_AREA_DEVICE_EK] = { 0xE8, 16 } };

static struct _CVI_EFUSE_LOCK_S {
	CVI_S32 wlock_shift;
	CVI_S32 rlock_shift;
} cvi_efuse_lock[] = { [CVI_EFUSE_LOCK_HASH0_PUBLIC] = { 0, 8 },     [CVI_EFUSE_LOCK_LOADER_EK] = { 4, 12 },
		       [CVI_EFUSE_LOCK_DEVICE_EK] = { 6, 14 },	     [CVI_EFUSE_LOCK_WRITE_HASH0_PUBLIC] = { 0, -1 },
		       [CVI_EFUSE_LOCK_WRITE_LOADER_EK] = { 4, -1 }, [CVI_EFUSE_LOCK_WRITE_DEVICE_EK] = { 6, -1 } };

static struct _CVI_EFUSE_USER_S {
	CVI_U32 addr;
	CVI_U32 size;
} cvi_efuse_user[] = {
	{ 0x40, 4 }, { 0x48, 4 }, { 0x50, 4 }, { 0x58, 4 }, { 0x60, 4 },
	{ 0x68, 4 }, { 0x70, 4 }, { 0x78, 4 }, { 0x80, 4 }, { 0x88, 4 },
};

#define CVI_EFUSE_TOTAL_SIZE 0x100

#define CVI_EFUSE_LOCK_ADDR 0xF8
#define CVI_EFUSE_SECURE_CONF_ADDR 0xA0
#define CVI_EFUSE_SCS_ENABLE_SHIFT			0
// for secure boot sign
#define CVI_EFUSE_TEE_SCS_ENABLE_SHIFT			2
#define CVI_EFUSE_ROOT_PUBLIC_KEY_SELECTION_SHIFT	20
// for secure boot encryption
#define CVI_EFUSE_BOOT_LOADER_ENCRYPTION		6
#define CVI_EFUSE_LDR_KEY_SELECTION_SHIFT		23

#define CVI_EFUSE_SW_INFO						0x2C
#define CVI_EFUSE_CUSTOMER_ADDR					0x4

CVI_S32 CVI_EFUSE_GetSize(enum CVI_EFUSE_AREA_E area, CVI_U32 *size)
{
	if (area >= ARRAY_SIZE(cvi_efuse_area) || cvi_efuse_area[area].size == 0) {
		_cc_error("area (%d) is not found\n", area);
		return CVI_ERR_EFUSE_INVALID_AREA;
	}

	if (size)
		*size = cvi_efuse_area[area].size;

	return 0;
}

#ifdef __riscv
CVI_S32 _CVI_EFUSE_Read(CVI_U32 addr, void *buf, CVI_U32 buf_size)
{
	int64_t ret = -1;
	int i;

	VERBOSE("%s(): 0x%x(%u) to %p\n", __func__, addr, buf_size, buf);

	if (!buf)
		return CVI_ERR_EFUSE_INVALID_PARA;

	if (buf_size > EFUSE_SIZE)
		buf_size = EFUSE_SIZE;

	for (i = 0; i < buf_size; i += 4) {
		ret = cvi_efuse_read_from_shadow(addr + i);
		VERBOSE("%s(): i=%x ret=%lx\n", __func__, i, ret);
		if (ret < 0)
			return ret;

		*(uint32_t *)(buf + i) = (ret >= 0) ? ret : 0;
	}

	return 0;
}
#else
static CVI_S32 _CVI_EFUSE_Read(CVI_U32 addr, void *buf, CVI_U32 buf_size)
{
	CVI_S32 ret = -1;
	struct arm_smccc_res res = { 0 };
	struct arm_smccc_quirk quirk = { 0 };

	_cc_trace("addr=0x%02x\n", addr);

	if (!buf)
		return CVI_ERR_EFUSE_INVALID_PARA;

	__asm_flush_dcache_all();
	__arm_smccc_smc(OPTEE_SMC_CALL_CV_EFUSE_READ, addr, buf_size, (unsigned long)buf, 0, 0, 0, 0, &res, &quirk);

	ret = res.a0;

	return ret;
}
#endif
CVI_U32 CVI_EFUSE_Read_Word(CVI_U32 addr)
{
	CVI_U32 addr_row;
	CVI_U32 val = 0;

	if (addr >= EFUSE_SIZE)
		return -EFAULT;

	addr_row = addr / 4;

	val |= cvi_efuse_read_from_phy((addr_row << 1) | 0, EFUSE_AREAD);

	val |= cvi_efuse_read_from_phy((addr_row << 1) | 1, EFUSE_AREAD);

	return val;
}
static CVI_S32 _CVI_EFUSE_Write(CVI_U32 addr, const void *buf, CVI_U32 buf_size)
{
	_cc_trace("addr=0x%02x\n", addr);

	int ret = -1;

	CVI_U32 value;
	int i;

	if (!buf)
		return CVI_ERR_EFUSE_INVALID_PARA;

	for (i = 0; i < buf_size; i += 4) {
		memcpy(&value, buf + i, sizeof(value));

		_cc_trace("smc call: 0x%02x=0x%08x\n", addr + i, value);
		ret = cvi_efuse_write(addr + i, value);

		if (ret < 0) {
			printf("%s: error (%d)\n", __func__, ret);
			return ret;
		}
	}

	return 0;
}

CVI_S32 CVI_EFUSE_Read(enum CVI_EFUSE_AREA_E area, CVI_U8 *buf, CVI_U32 buf_size)
{
	CVI_U32 user_size = cvi_efuse_area[CVI_EFUSE_AREA_USER].size;
	CVI_U8 user[user_size], *p;
	CVI_S32 ret;
	int i;

	if (area >= ARRAY_SIZE(cvi_efuse_area) || cvi_efuse_area[area].size == 0) {
		_cc_error("area (%d) is not found\n", area);
		return CVI_ERR_EFUSE_INVALID_AREA;
	}

	if (!buf)
		return CVI_ERR_EFUSE_INVALID_PARA;

	memset(buf, 0, buf_size);

	if (buf_size > cvi_efuse_area[area].size)
		buf_size = cvi_efuse_area[area].size;

	if (area != CVI_EFUSE_AREA_USER)
		return _CVI_EFUSE_Read(cvi_efuse_area[area].addr, buf, buf_size);

	memset(user, 0, user_size);

	p = user;
	for (i = 0; i < ARRAY_SIZE(cvi_efuse_user); i++) {
		ret = _CVI_EFUSE_Read(cvi_efuse_user[i].addr, p, cvi_efuse_user[i].size);
		if (ret < 0)
			return ret;
		p += cvi_efuse_user[i].size;
	}

	memcpy(buf, user, buf_size);

	return CVI_SUCCESS;
}

CVI_S32 CVI_EFUSE_Write(enum CVI_EFUSE_AREA_E area, const CVI_U8 *buf, CVI_U32 buf_size)
{
	CVI_U32 user_size = cvi_efuse_area[CVI_EFUSE_AREA_USER].size;
	CVI_U8 user[user_size], *p;
	CVI_S32 ret;
	int i;

	if (area >= ARRAY_SIZE(cvi_efuse_area) || cvi_efuse_area[area].size == 0) {
		_cc_error("area (%d) is not found\n", area);
		return CVI_ERR_EFUSE_INVALID_AREA;
	}
	if (!buf)
		return CVI_ERR_EFUSE_INVALID_PARA;

	if (buf_size > cvi_efuse_area[area].size)
		buf_size = cvi_efuse_area[area].size;

	if (area != CVI_EFUSE_AREA_USER)
		return _CVI_EFUSE_Write(cvi_efuse_area[area].addr, buf, buf_size);

	memset(user, 0, user_size);
	memcpy(user, buf, buf_size);

	p = user;
	for (i = 0; i < ARRAY_SIZE(cvi_efuse_user); i++) {
		ret = _CVI_EFUSE_Write(cvi_efuse_user[i].addr, p, cvi_efuse_user[i].size);
		if (ret < 0)
			return ret;
		p += cvi_efuse_user[i].size;
	}

	return CVI_SUCCESS;
}

CVI_S32 CVI_EFUSE_EnableFastBoot(void)
{
	CVI_U32 value = 0, data;
	CVI_S32 ret = 0;
	CVI_U32 chip = 0;

	if (CVI_EFUSE_IsFastBootEnabled() == CVI_SUCCESS) {
		printf("Fast Boot is already enabled.\n");
		return CVI_SUCCESS;
	}

	chip = mmio_read_32(0x0300008c);

	ret = _CVI_EFUSE_Read(CVI_EFUSE_SW_INFO, &value, sizeof(value));
	_cc_trace("ret=%d value=%u\n", ret, value);
	if (ret < 0)
		return CVI_FAILURE;

	data = (value & (0x3 << 22)) >> 22;
	if (data > 0x1)
		return CVI_FAILURE;

	data = (value & (0x3 << 24)) >> 24;
	if (data > 0x1)
		return CVI_FAILURE;

	data = (value & (0x3 << 26)) >> 26;
	if (data > 0x1)
		return CVI_FAILURE;

	ret = _CVI_EFUSE_Read(CVI_EFUSE_CUSTOMER_ADDR, &value, sizeof(value));
	_cc_trace("ret=%d value=%u\n", ret, value);
	if (ret < 0)
		return ret;

	if ((chip & 0xFFF0F) == 0x1810C && ((chip >> 4) & 0xF) <= 3) { // 181XC (X <= 3)
		value |= 0x1E1E64; // CV181X-AUX0
		if (value != 0x1E1E64) {
			_cc_trace("CUSTOMER value=%u\n", value);
			return CVI_FAILURE;
		}
	} else if (((chip & 0xFFF0F) == 0x1800C || (chip & 0xFFF0F) == 0x1800B)
					&& ((chip >> 4) & 0xF) <= 3) { // CV180X (X <= 3)
		value |= 0x1E1564; // CV180X-AUX0
		if (value != 0x1E1564) {
			_cc_trace("CUSTOMER value=%u\n", value);
			return CVI_FAILURE;
		}
	} else {
		value |= 0x1; // USB_ID
		if (value != 0x1) {
			_cc_trace("CUSTOMER value=%u\n", value);
			return CVI_FAILURE;
		}
	}

	ret = _CVI_EFUSE_Write(CVI_EFUSE_CUSTOMER_ADDR, &value, sizeof(value));
	if (ret < 0)
		return ret;

	// set sd dl button
	value = (0x1 << 22);
	value |= (0x1 << 24);
	value |= (0x1 << 26);

	return _CVI_EFUSE_Write(CVI_EFUSE_SW_INFO, &value, sizeof(value));
}

CVI_S32 CVI_EFUSE_IsFastBootEnabled(void)
{
	CVI_U32 value = 0;
	CVI_S32 ret = 0;
	CVI_U32 chip = 0;

	chip = mmio_read_32(0x0300008c);

	ret = _CVI_EFUSE_Read(CVI_EFUSE_SW_INFO, &value, sizeof(value));
	_cc_trace("ret=%d value=%u\n", ret, value);
	if (ret < 0)
		return ret;

	if (((value & (0x3 << 22)) != (0x1 << 22))
		&& ((value & (0x3 << 24)) != (0x1 << 24))
		&& ((value & (0x3 << 26)) != (0x1 << 26))) {
		_cc_trace("sw_info isn't fastboot config\n");
		return CVI_FAILURE;
	}

	ret = _CVI_EFUSE_Read(CVI_EFUSE_CUSTOMER_ADDR, &value, sizeof(value));
	_cc_trace("ret=%d value=%u\n", ret, value);
	if (ret < 0)
		return ret;

	if ((chip & 0xFFF0F) == 0x1810C && ((chip >> 4) & 0xF) <= 3) { // 181XC (X <= 3)
		if (value == 0x1E1E64)
			return CVI_SUCCESS; // CV181X-AUX0
		else
			return CVI_FAILURE;
	} else if (((chip & 0xFFF0F) == 0x1800C || (chip & 0xFFF0F) == 0x1800B)
					&& ((chip >> 4) & 0xF) <= 3) { // CV180X (X <= 3)
		if (value == 0x1E1564)
			return CVI_SUCCESS; // CV180X-AUX0
		else
			return CVI_FAILURE;
	} else {
		if (value == 0x1)
			return CVI_SUCCESS; // USB_ID
		else
			return CVI_FAILURE;
	}

	return CVI_FAILURE;
}

CVI_S32 CVI_EFUSE_EnableSecureBoot(enum CVI_EFUSE_SECUREBOOT_E sel)
{
	CVI_U32 value = 0;

	value |= 0x3 << CVI_EFUSE_TEE_SCS_ENABLE_SHIFT;
	value |= 0x4 << CVI_EFUSE_ROOT_PUBLIC_KEY_SELECTION_SHIFT;

#if IS_ENABLED(CONFIG_TARGET_CVITEK_CV181X)
	if (CVI_EFUSE_SECUREBOOT_SIGN_ENCRYPT == sel) {
		value |= 0x3 << CVI_EFUSE_BOOT_LOADER_ENCRYPTION;
		value |= 0x4 << CVI_EFUSE_LDR_KEY_SELECTION_SHIFT;
	}
#endif
	return _CVI_EFUSE_Write(CVI_EFUSE_SECURE_CONF_ADDR, &value, sizeof(value));
}

CVI_S32 CVI_EFUSE_IsSecureBootEnabled(void)
{
	CVI_U32 value = 0;
	CVI_S32 ret = 0;

	ret = _CVI_EFUSE_Read(CVI_EFUSE_SECURE_CONF_ADDR, &value, sizeof(value));
	_cc_trace("ret=%d value=%u\n", ret, value);
	if (ret < 0)
		return ret;

	value &= 0x3 << CVI_EFUSE_SCS_ENABLE_SHIFT;
	return !!value;
}

CVI_S32 CVI_EFUSE_Lock(enum CVI_EFUSE_LOCK_E lock)
{
	CVI_U32 value = 0;
	CVI_U32 ret = 0;

	if (lock >= ARRAY_SIZE(cvi_efuse_lock)) {
		_cc_error("lock (%d) is not found\n", lock);
		return CVI_ERR_EFUSE_INVALID_AREA;
	}

	value = 0x3 << cvi_efuse_lock[lock].wlock_shift;
	ret = _CVI_EFUSE_Write(CVI_EFUSE_LOCK_ADDR, &value, sizeof(value));
	if (ret < 0)
		return ret;

	if (cvi_efuse_lock[lock].rlock_shift >= 0) {
		value = 0x3 << cvi_efuse_lock[lock].rlock_shift;
		ret = _CVI_EFUSE_Write(CVI_EFUSE_LOCK_ADDR, &value, sizeof(value));
	}

	return ret;
}

CVI_S32 CVI_EFUSE_IsLocked(enum CVI_EFUSE_LOCK_E lock)
{
	CVI_S32 ret = 0;
	CVI_U32 value = 0;

	if (lock >= ARRAY_SIZE(cvi_efuse_lock)) {
		_cc_error("lock (%d) is not found\n", lock);
		return CVI_ERR_EFUSE_INVALID_AREA;
	}

	ret = CVI_EFUSE_Read_Word(CVI_EFUSE_LOCK_ADDR);
	_cc_trace("ret=%d value=%u\n", ret, value);
	if (ret < 0)
		return ret;

	value &= 0x3 << cvi_efuse_lock[lock].wlock_shift;
	return !!value;
}

CVI_S32 CVI_EFUSE_LockWrite(enum CVI_EFUSE_LOCK_E lock)
{
	CVI_U32 value = 0;
	CVI_S32 ret = 0;

	if (lock >= ARRAY_SIZE(cvi_efuse_lock)) {
		_cc_error("lock (%d) is not found\n", lock);
		return CVI_ERR_EFUSE_INVALID_AREA;
	}

	value = 0x3 << cvi_efuse_lock[lock].wlock_shift;
	ret = _CVI_EFUSE_Write(CVI_EFUSE_LOCK_ADDR, &value, sizeof(value));
	return ret;
}

CVI_S32 CVI_EFUSE_IsWriteLocked(enum CVI_EFUSE_LOCK_E lock)
{
	CVI_S32 ret = 0;
	CVI_U32 value = 0;

	if (lock >= ARRAY_SIZE(cvi_efuse_lock)) {
		_cc_error("lock (%d) is not found\n", lock);
		return CVI_ERR_EFUSE_INVALID_AREA;
	}
	ret = _CVI_EFUSE_Read(CVI_EFUSE_LOCK_ADDR, &value, sizeof(value));
	_cc_trace("ret=%d value=%u\n", ret, value);
	if (ret < 0)
		return ret;

	value &= 0x3 << cvi_efuse_lock[lock].wlock_shift;
	return !!value;
}

