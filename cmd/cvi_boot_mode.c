// SPDX-License-Identifier: GPL-2.0+
// #define DEBUG /* open debug log */
#include <common.h>
#include <command.h>
#include <stdlib.h>
#include <stdarg.h>
#include <malloc.h>
#include <asm/io.h>
#include <mmc.h>
#include <cvi_update.h>
#include <u-boot/crc.h>
#include <env.h>
#include <search.h>
#include "cvi_boot_mode.h"
#include "cvipart.h"
#include "cvsnfc.h"
#include <nand.h>
#include <ubi_uboot.h>

extern struct mtd_info *nand_get_mtd(void);

struct boot_dev_info_t g_boot_dev;
static enum boot_mode_t g_boot_mode;

#define REGISTER_BOOT_MODE (uintptr_t *)(BOOT_MODE_REGISTER)

enum boot_mode_t get_cvi_boot_mode(void)
{
	return g_boot_mode;
}

static uint32_t ab_crc32(uint32_t crc, const char *p, uint32_t len)
{
	/* too many crc32 functions, so we crc32_no_comp in u-boot-2021.10/lib/crc32.c */
	return crc32_no_comp(crc ^ 0xffffffffL, p, len) ^ 0xffffffffL;
}

static enum boot_mode_t set_init_env(char *p)
{
	if (p == NULL) {
		debug("[%s:%d] ptr is null\n", __func__, __LINE__);
		return BOOT_MODE_INVALID;
	}

	if (strncmp(p, "_a", 2) == 0) {
		debug("use slot A\n");
		return BOOT_MODE_A;
	} else if (strncmp(p, "_b", 2) == 0) {
		debug("use slot B\n");
		return BOOT_MODE_B;
	}

	debug("come in recovery\n");
	return BOOT_MODE_R;
}

#if defined(CONFIG_ENV_IS_IN_NAND)
char *misc_volname = "misc_vol0";

static int nand_erase_part(void)
{
	int ret;
	struct mtd_info *mtd;
	nand_erase_options_t opts = {0};

#if defined(CONFIG_SPL_BUILD)
	mtd = nand_get_mtd();
	if (!mtd) {
		printf("Invalid NAND device\n");
		return -ENODEV;
	}
#else
	int dev = nand_curr_device;

	mtd = get_nand_dev_by_index(dev);
	if (!mtd) {
		printf("Invalid NAND device: %d\n", dev);
		return -ENODEV;
	}
#endif

	opts.offset = MISC_PART_OFFSET;
	opts.length = MISC_PART_SIZE;
	opts.jffs2  = 0;
	opts.quiet  = 0;
	opts.spread = 0;
	ret = nand_erase_opts(mtd, &opts);
	if (ret) {
		printf("nand_erase failed\n");
		return -1;
	}

	return ubi_part("MISC", NULL);
}

static int ubi_part_misc(void)
{
	static int has_ubi_part;
	int ret = 0;

	if (has_ubi_part)
		return 0;

	ret = ubi_part("MISC", NULL) ? nand_erase_part() : 0;
	if (ret) {
		printf("ubi_part(MISC) failed\n");
		return -1;
	}

	if (!ubi_find_volume(misc_volname)) {
		ret = ubi_create_vol(misc_volname, sizeof(struct AvbABData), 1, -1, 0);
		if (ret) {
			printf("create vol(MISC) failed\n");
			return -1;
		}
	}

	has_ubi_part = 1;
	return 0;
}
#endif

static struct AvbABData *load_abdata(void)
{
	unsigned long message_blocks = 0;
	unsigned long read_bl_len = 0;
	int check_ret = -1;
	void *data;

#if defined(CONFIG_ENV_IS_IN_MMC)
	if (!g_boot_dev.mmc_host) {
		printf("[%s:%d]error : mmc ptr null\n", __func__, __LINE__);
		return NULL;
	}
	read_bl_len = g_boot_dev.mmc_host->read_bl_len;
#elif defined(CONFIG_ENV_IS_IN_NAND)
	check_ret = ubi_part_misc();
	if (check_ret) {
		printf("ubi_part(MISC) failed\n");
		return NULL;
	}
	if (!g_boot_dev.nand_host) {
		printf("[%s:%d]error : nand ptr null\n", __func__, __LINE__);
		return NULL;
	}
	read_bl_len = g_boot_dev.nand_host->nand_chip_info->pagesize;
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	message_blocks = DIV_ROUND_UP(sizeof(struct AvbABData), read_bl_len);
	if (message_blocks == 0) {
		printf("MISC partition too small.\n");
		return NULL;
	}

	data = malloc(message_blocks * read_bl_len);
	if (data == NULL) {
		printf("malloc data failed\n");
		return NULL;
	}

#if defined(CONFIG_ENV_IS_IN_MMC)
	check_ret = blk_dread(mmc_get_blk_desc(g_boot_dev.mmc_host), MISC_PART_OFFSET, message_blocks, data);
	if (check_ret != message_blocks) {
		printf("Could not read from misc partition\n");
		free(data);
		data = NULL;
	}
#elif defined(CONFIG_ENV_IS_IN_NAND)
	check_ret = ubi_volume_read(misc_volname, data, message_blocks * read_bl_len);
	if (check_ret) {
		printf("Could not read from misc partition\n");
		free(data);
		data = NULL;
	}
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	return (struct AvbABData *)data;
}

void avb_ab_data_dump(void)
{
	struct AvbABData *ab_data;
	char magic[5] = {0};

	ab_data = load_abdata();
	if (!ab_data) {
		printf("load abdata failed\n");
		return;
	}

	if (strncmp(&ab_data->magic[1], "AB0", 3)) {
		printf("abdata magic error\n");
		return;
	}

	memcpy(magic, &ab_data->magic[1], 3);
	printf("magic=%s\n", magic);
	printf("version_major=%u\n", ab_data->version_major);
	printf("version_minor=%u\n", ab_data->version_minor);
	printf("slots[0].priority=%u\n", ab_data->slots[0].priority);
	printf("slots[0].tries_remaining=%u\n", ab_data->slots[0].tries_remaining);
	printf("slots[0].successful_boot=%u\n", ab_data->slots[0].successful_boot);
	printf("slots[1].priority=%u\n", ab_data->slots[1].priority);
	printf("slots[1].tries_remaining=%u\n", ab_data->slots[1].tries_remaining);
	printf("slots[1].successful_boot=%u\n", ab_data->slots[1].successful_boot);
	printf("crc=%x\n", ab_data->crc32);

	free(ab_data);
}


/*
 * Initializes the A/B metadata in the misc partition.
 */
static AvbABFlowResult avb_ab_data_init(void)
{
	void *data = NULL;
	struct AvbABData *ab_data;
	unsigned long check_ret;
	AvbABFlowResult ret = AVB_AB_FLOW_RESULT_OK;
	unsigned long message_blocks;
	unsigned long read_bl_len = 0;

#if defined(CONFIG_ENV_IS_IN_MMC)
	if (!g_boot_dev.mmc_host) {
		printf("[%s:%d]error : mmc ptr null\n", __func__, __LINE__);
		return -1;
	}
	read_bl_len = g_boot_dev.mmc_host->read_bl_len;
#elif defined(CONFIG_ENV_IS_IN_NAND)
	check_ret = ubi_part_misc();
	if (check_ret) {
		printf("ubi_part(MISC) failed\n");
		return -1;
	}
	if (!g_boot_dev.nand_host) {
		printf("[%s:%d]error : nand ptr null\n", __func__, __LINE__);
		return -1;
	}
	read_bl_len = g_boot_dev.nand_host->nand_chip_info->pagesize;
	debug("read_bl_len = %ld,pagesize = %d\n", read_bl_len, g_boot_dev.nand_host->nand_chip_info->pagesize);
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	message_blocks = DIV_ROUND_UP(sizeof(struct AvbABData), read_bl_len);
	debug("message_blocks = %ld, sizeof(AvbABData) = %lu\n", message_blocks, sizeof(struct AvbABData));
	if (message_blocks == 0) {
		printf("misc partition too small.\n");
		ret = AVB_AB_FLOW_RESULT_ERROR_IO;
		goto out;
	}

	data = malloc(message_blocks * read_bl_len);
	if (data == NULL) {
		printf("malloc data failed\n");
		goto out;
	}

	memset(data, '\0', message_blocks * read_bl_len);
	ab_data = (struct AvbABData *)data;

	memcpy(ab_data->magic, AVB_AB_MAGIC, AVB_AB_MAGIC_LEN);
	ab_data->version_major = AVB_AB_MAJOR_VERSION;
	ab_data->version_minor = AVB_AB_MINOR_VERSION;
	ab_data->slots[0].priority = AVB_AB_MAX_PRIORITY;
	ab_data->slots[0].tries_remaining = AVB_AB_MAX_TRIES_REMAINING;
	ab_data->slots[0].successful_boot = 0;
	ab_data->slots[1].priority = AVB_AB_MAX_PRIORITY - 1;
	ab_data->slots[1].tries_remaining = AVB_AB_MAX_TRIES_REMAINING;
	ab_data->slots[1].successful_boot = 0;
	ab_data->crc32 = ab_crc32(0, (const uint8_t *)ab_data, sizeof(struct AvbABData) - sizeof(uint32_t));
#if defined(CONFIG_ENV_IS_IN_MMC)
	check_ret = blk_dwrite(mmc_get_blk_desc(g_boot_dev.mmc_host), MISC_PART_OFFSET, message_blocks, data);
	if (check_ret != message_blocks) {
		printf("Can not save metadata\n");
		ret = AVB_AB_FLOW_RESULT_ERROR_IO;
	}
#elif defined(CONFIG_ENV_IS_IN_NAND)
	debug("w page at: 0x%x, message_blocks:%ld\n", MISC_PART_OFFSET, message_blocks);
	check_ret = ubi_volume_write(misc_volname, data, message_blocks * read_bl_len);
	if (check_ret) {
		printf("Can not save metadata\n");
		ret = AVB_AB_FLOW_RESULT_ERROR_IO;
	}
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	free(data);
	data = NULL;

out:
	return ret;
}

static bool slot_is_bootable(struct AvbABSlotData *slot)
{
	return (slot->priority > 0) && (slot->successful_boot || (slot->tries_remaining > 0));
}

/*
 * checks if the A/B metadata in the misc partition is valid.
 */
static int8_t avb_ab_slot_vertify(void)
{
	int8_t ret = 0;
	struct AvbABData *ab_data;

	uint32_t crc_result = 0;

	ab_data = load_abdata();
	if (!ab_data) {
		printf("load abdata failed\n");
		return AVB_AB_FLOW_RESULT_ERROR_IO;
	}

	if (strncmp(&ab_data->magic[1], "AB0", 3)) {
		uintptr_t *p = REGISTER_BOOT_MODE;
		*p = BOOT_NORMAL;
		ret = AVB_AB_FLOW_RESULT_ERROR_IO;
		goto out;
	}

	crc_result = ab_crc32(0, (const uint8_t *)ab_data, sizeof(struct AvbABData) - sizeof(uint32_t));

	debug("cal crc32 = %x,real c32 = %x\n", crc_result, ab_data->crc32);
	if (crc_result != ab_data->crc32)
		ret = AVB_AB_FLOW_RESULT_ERROR_IO;

out:
	free(ab_data);
	return ret;
}

/**
 * Updates the A/B metadata in the misc partition to reflect that the
 * given slot (a or b) has been booted.
 *
 * @param slot_suffix	"_a" or "_b" to indicate which slot to update
 * @param mmc		pointer to a struct mmc describing the mmc device
 *			holding the misc partition
 *
 * @return AVB_AB_FLOW_RESULT_OK if the function was successful,
 *         AVB_AB_FLOW_RESULT_ERROR_IO if there was an IO error
 */
static AvbABFlowResult avb_ab_slot_update(char *slot_suffix)
{
	struct AvbABData serialized;
	struct AvbABData *ab_data = NULL;

	int slot_index_to_boot = 0;
	AvbABFlowResult ret = AVB_AB_FLOW_RESULT_OK;
	unsigned long message_blocks;
	unsigned long check_ret;
	unsigned long read_bl_len = 0;

#if defined(CONFIG_ENV_IS_IN_MMC)
	if (!g_boot_dev.mmc_host) {
		printf("[%s:%d]error : mmc ptr null\n", __func__, __LINE__);
		return -1;
	}
	read_bl_len = g_boot_dev.mmc_host->read_bl_len;
#elif defined(CONFIG_ENV_IS_IN_NAND)
	check_ret = ubi_part_misc();
	if (check_ret) {
		printf("ubi_part(misc) failed\n");
		return -1;
	}
	if (!g_boot_dev.nand_host) {
		printf("[%s:%d]error : nand ptr null\n", __func__, __LINE__);
		return -1;
	}
	read_bl_len = g_boot_dev.nand_host->nand_chip_info->pagesize;
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	message_blocks = DIV_ROUND_UP(sizeof(struct AvbABData), read_bl_len);
	if (message_blocks == 0) {
		printf("misc partition too small.\n");
		return AVB_AB_FLOW_RESULT_ERROR_IO;
	}

	ab_data = load_abdata();
	if (!ab_data) {
		printf("load abdata failed\n");
		return AVB_AB_FLOW_RESULT_ERROR_IO;
	}

	memset((void *)&serialized, '\0', sizeof(struct AvbABData));
	if (!strncmp(slot_suffix, "_b", 2))
		slot_index_to_boot = 1;

	/* ... and decrement tries remaining, if applicable. */
	if (!ab_data->slots[slot_index_to_boot].successful_boot &&
		ab_data->slots[slot_index_to_boot].tries_remaining > 0) {
		ab_data->slots[slot_index_to_boot].tries_remaining -= 1;
	}

	// update crc
	ab_data->crc32 = ab_crc32(0, (const uint8_t *)ab_data, sizeof(struct AvbABData) - sizeof(uint32_t));
	printf("update crc32 = %x\n", ab_data->crc32);
#if defined(CONFIG_ENV_IS_IN_MMC)
	check_ret = blk_dwrite(mmc_get_blk_desc(g_boot_dev.mmc_host), MISC_PART_OFFSET, message_blocks, ab_data);
	if (check_ret != message_blocks) {
		printf("Can not save metadata\n");
		ret = AVB_AB_FLOW_RESULT_ERROR_IO;
	}
#elif defined(CONFIG_ENV_IS_IN_NAND)
	check_ret = ubi_volume_write(misc_volname, ab_data, message_blocks * read_bl_len);
	if (check_ret) {
		printf("Can not save metadata\n");
		ret = AVB_AB_FLOW_RESULT_ERROR_IO;
	}
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	free(ab_data);
	ab_data = NULL;
	return ret;
}

/**
 * Reads the A/B metadata from the misc partition and determines which slot
 * should be booted. The result is stored in the |select_slot| string.
 *
 * @param select_slot	pointer to a string of length 3 where the result
 *				will be stored, "_a" for slot 0, "_b" for slot 1
 * @param mmc		pointer to a struct mmc describing the mmc device
 *				holding the misc partition
 *
 * @return AVB_AB_FLOW_RESULT_OK if the function was successful,
 *         AVB_AB_FLOW_RESULT_ERROR_IO if there was an IO error,
 *         AVB_AB_FLOW_RESULT_ERROR_NO_BOOTABLE_SLOTS if neither slot is
 *         bootable
 */
static int avb_get_current_slot(char *select_slot)
{
	AvbABFlowResult ret = AVB_AB_FLOW_RESULT_OK;
	struct AvbABData *ab_data;
	size_t slot_index_to_boot;

	ab_data = load_abdata();
	if (!ab_data) {
		printf("load abdata failed\n");
		return AVB_AB_FLOW_RESULT_ERROR_IO;
	}

	if (slot_is_bootable(&ab_data->slots[0]) && slot_is_bootable(&ab_data->slots[1])) {
		if (ab_data->slots[1].priority > ab_data->slots[0].priority)
			slot_index_to_boot = 1;
		else
			slot_index_to_boot = 0;

	} else if (slot_is_bootable(&ab_data->slots[0]))
		slot_index_to_boot = 0;
	else if (slot_is_bootable(&ab_data->slots[1]))
		slot_index_to_boot = 1;
	else {
		printf("No bootable slots found.\n");
		free(ab_data);
		ab_data = NULL;
		ret = AVB_AB_FLOW_RESULT_ERROR_NO_BOOTABLE_SLOTS;
		goto out;
	}

	if (slot_index_to_boot == 0)
		strcpy(select_slot, "_a");
	else if (slot_index_to_boot == 1)
		strcpy(select_slot, "_b");

	free(ab_data);
	ab_data = NULL;

out:
	return ret;
}

/**
 * Read the BootloaderMessage from MISC partition.
 * If the command is "boot-recovery", return 0, else return -1.
 * @param mmc	MMC device
 * @return 0 if the command is "boot-recovery", else -1
 */
int bootloader_message_load(void)
{
	unsigned long read_bl_len = 0;
	unsigned long message_blocks;
	unsigned long check_ret;
	void *data = NULL;
	struct BootloaderMessage boot_loader_message;

#if defined(CONFIG_ENV_IS_IN_MMC)
	if (!g_boot_dev.mmc_host) {
		printf("[%s:%d]error : mmc ptr null\n", __func__, __LINE__);
		return -1;
	}
	read_bl_len = g_boot_dev.mmc_host->read_bl_len;
#elif defined(CONFIG_ENV_IS_IN_NAND)
	check_ret = ubi_part_misc();
	if (check_ret) {
		printf("ubi_part(misc) failed\n");
		return -1;
	}
	if (!g_boot_dev.nand_host) {
		printf("[%s:%d]error : nand ptr null\n", __func__, __LINE__);
		return -1;
	}
	read_bl_len = g_boot_dev.nand_host->nand_chip_info->pagesize;
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	message_blocks = DIV_ROUND_UP(sizeof(struct BootloaderMessage), read_bl_len);

	if (message_blocks == 0) {
		printf("misc partition too small.\n");
		return -1;
	}

	data = malloc(message_blocks * read_bl_len);
	if (data == NULL) {
		printf("malloc data failed\n");
		return -1;
	}
#if defined(CONFIG_ENV_IS_IN_MMC)
	check_ret = blk_dread(mmc_get_blk_desc(g_boot_dev.mmc_host),
		MISC_PART_OFFSET + BOOTLOADER_MESSAGE_OFFSET_IN_MISC/read_bl_len, message_blocks, data);
	if (check_ret != message_blocks) {
		printf("Could not read from misc partition\n");
		free(data);
		data = NULL;
		return -1;
	}
#elif defined(CONFIG_ENV_IS_IN_NAND)
	check_ret = ubi_volume_read(misc_volname, data, message_blocks * read_bl_len);
	if (check_ret) {
		printf("Could not read from misc partition\n");
		free(data);
		data = NULL;
		return -1;
	}
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	memcpy((void *)&boot_loader_message, data, sizeof(struct BootloaderMessage));

	if (!strcmp("boot-recovery", boot_loader_message.command)) {
		free(data);
		return 0;
	}

	free(data);
	return -1;
}

/**
 * Get the boot mode from the register and the misc partition.
 *
 * @param mmc	The device to read from
 * @return	The boot mode, one of ANDROID_BOOT_MODE_*
 */
int cvi_get_boot_mode(void)
{
	uintptr_t *p = REGISTER_BOOT_MODE;

	debug("boot mode registet: addr = %lx, value = %lx\n", (uintptr_t)p, *p);

	if (bootloader_message_load() == 0)
		return ANDROID_BOOT_MODE_RECOVERY;
	else if (*p == BOOT_BL_DOWNLOAD)
		return ANDROID_BOOT_MODE_LOADER;
	else if (*p == BOOT_RECOVERY)
		return ANDROID_BOOT_MODE_RECOVERY;
	else if (*p == BOOT_CHARGING)
		return ANDROID_BOOT_MODE_CHARGING;
	else if (*p == BOOT_NORMAL)
		return ANDROID_BOOT_MODE_NORMAL;

	return ANDROID_BOOT_MODE_UNDEFINE;
}

int reset_bootmode_to_normal(void)
{
	uintptr_t *p = REGISTER_BOOT_MODE;
	*p = BOOT_NORMAL;
	return 0;
}

enum boot_mode_t cvi_boot_flow(void)
{
	enum android_boot_mode mode;
	char slot_suffix[3] = {0};
	enum boot_mode_t ret;

	debug("-%s:%d-\n", __func__, __LINE__);

#if defined(CONFIG_ENV_IS_IN_MMC)
	g_boot_dev.mmc_host = get_mmcdevice();
#elif defined(CONFIG_ENV_IS_IN_NAND)
	g_boot_dev.nand_host = cvsnfc_get_host();
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	// 1. check ab_data magic, if not exist, init it
	if (avb_ab_slot_vertify() != 0)
		avb_ab_data_init();

	// 2. get boot mode
	mode = cvi_get_boot_mode();

	switch (mode) {
	case ANDROID_BOOT_MODE_NORMAL:
	case ANDROID_BOOT_MODE_LOADER:
	case ANDROID_BOOT_MODE_CHARGING:
	case ANDROID_BOOT_MODE_UNDEFINE:
	{
		// 3. get current slot, (slot is a or b)
		avb_get_current_slot(slot_suffix);
		debug("slot_suffix = %s\n", slot_suffix);
		// 4. update remaintrying count
		avb_ab_slot_update(slot_suffix);

		ret = set_init_env(slot_suffix);
		break;
	}
	case ANDROID_BOOT_MODE_RECOVERY:
	{
		ret = set_init_env("_r");
		break;
	}
	default:
		break;
	}

	debug("do_cvi_bootmode mode %d\n", mode);
	reset_bootmode_to_normal();
	return ret;
}

/***************** TEST TOOLS *****************/
#if defined(CONFIG_ENV_IS_IN_MMC)
static struct mmc *__init_mmc_device(int dev, bool force_init,
				     enum bus_mode speed_mode)
{
	struct mmc *mmc;

	mmc = find_mmc_device(dev);
	if (!mmc) {
		printf("no mmc device at slot %x\n", dev);
		return NULL;
	}

	if (!mmc_getcd(mmc))
		force_init = true;

	if (force_init)
		mmc->has_init = 0;

	if (IS_ENABLED(CONFIG_MMC_SPEED_MODE_SET))
		mmc->user_speed_mode = speed_mode;

	if (mmc_init(mmc))
		return NULL;

#ifdef CONFIG_BLOCK_CACHE
	struct blk_desc *bd = mmc_get_blk_desc(mmc);

	blkcache_invalidate(bd->if_type, bd->devnum);
#endif

	return mmc;
}

static struct mmc *init_mmc_device(int dev, bool force_init)
{
	return __init_mmc_device(dev, force_init, MMC_MODES_END);
}

struct mmc *get_mmcdevice(void)
{
	struct mmc *mmc;

	if (get_mmc_num() <= 0) {
		printf("No MMC device available\n");
		return NULL;
	}

	// default using mmc device 0
	mmc = init_mmc_device(0, false);
	if (!mmc) {
		printf("init mmc device failed\n");
		return NULL;
	}

	return mmc;
} /* CONFIG_ENV_IS_IN_MMC */
#elif defined(CONFIG_ENV_IS_IN_NAND)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#endif

static int do_get_slot(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	char slot_suffix[3] = {0};
	int ret;
#if defined(CONFIG_ENV_IS_IN_MMC)
	g_boot_dev.mmc_host = get_mmcdevice();
#elif defined(CONFIG_ENV_IS_IN_NAND)
	g_boot_dev.nand_host = cvsnfc_get_host();
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	ret = avb_get_current_slot(slot_suffix);
	if (ret != AVB_AB_FLOW_RESULT_OK) {
		printf("get current slot failed\n");
		return CMD_RET_FAILURE;
	}

	printf("slot_suffix = %s\n", slot_suffix);
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(get_slot, 1, 0, do_get_slot,
	"get_slot - get current active slot\n",
	"run get_slot without parameter will output current boot slot\n");


static int do_dump_abdata(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
#if defined(CONFIG_ENV_IS_IN_MMC)
	g_boot_dev.mmc_host = get_mmcdevice();
#elif defined(CONFIG_ENV_IS_IN_NAND)
	g_boot_dev.nand_host = cvsnfc_get_host();
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif
	avb_ab_data_dump();

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(dump_abdata, 1, 0, do_dump_abdata,
	"dump_abdata - dump abdata\n",
	"run dump_abdata without parameter will dump abdata\n");

void get_addr_part_offset_size(void **addr, ulong *part_offset, ulong *part_size)
{
#if defined(CONFIG_SPL_BUILD)
	*addr = (void *)CVIMMAP_UIMAG_ADDR;
#else
	*addr = (void *)simple_strtoul(env_get("uImage_addr"), NULL, 16);
#endif

	g_boot_mode = cvi_boot_flow();

	switch (g_boot_mode) {
	case BOOT_MODE_A:
#if defined(CONFIG_SPL_BUILD)
		*part_offset = SPL_BOOT_PART_OFFSET;
#else
		*part_offset = simple_strtoul(env_get("BOOT_PART_OFFSET"), NULL, 16);
		*part_size = simple_strtoul(env_get("BOOT_PART_SIZE"), NULL, 16);
		env_set("root", ROOTARGS);
#endif
		printf("boot mode A\n");
		break;
#ifdef CONFIG_ROOTFS_B
	case BOOT_MODE_B:
#if defined(CONFIG_SPL_BUILD)
		*part_offset = SPL_BOOT_B_PART_OFFSET;
#else
		*part_offset = simple_strtoul(env_get("BOOT_B_PART_OFFSET"), NULL, 16);
		*part_size = simple_strtoul(env_get("BOOT_B_PART_SIZE"), NULL, 16);
		env_set("root", ROOTARGSB);
#endif
		printf("boot mode B\n");
		break;
#endif
#ifdef CONFIG_ROOTFS_RECOVERY
	case BOOT_MODE_R:
		fallthrough;
	default:
#if defined(CONFIG_SPL_BUILD)
		*part_offset = RECOVERY_PART_OFFSET;
#else
		*part_offset = simple_strtoul(env_get("RECOVERY_PART_OFFSET"), NULL, 16);
		*part_size = simple_strtoul(env_get("RECOVERY_PART_SIZE"), NULL, 16);
		env_set("root", ROOTARGSR);
#endif
		printf("boot mode recovery\n");
		break;
#else
	default:
#if defined(CONFIG_SPL_BUILD)
		*part_offset = SPL_BOOT_PART_OFFSET;
#else
		*part_offset = simple_strtoul(env_get("BOOT_PART_OFFSET"), NULL, 16);
		*part_size = simple_strtoul(env_get("BOOT_PART_SIZE"), NULL, 16);
		env_set("root", ROOTARGS);
#endif
		printf("default boot mode A\n");
		break;
#endif
	}

}

#if !defined(CONFIG_SPL_BUILD)
static int do_loadboot(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
	ulong part_offset = BOOT_MODE_INVALID;
	ulong part_size = 0;
	void *uimage_addr;
	int check_ret = 0;

	get_addr_part_offset_size(&uimage_addr, &part_offset, &part_size);

#if defined(CONFIG_ENV_IS_IN_MMC)
	check_ret = blk_dread(mmc_get_blk_desc(g_boot_dev.mmc_host), part_offset, part_size, uimage_addr);
	if (check_ret != part_size) {
		printf("Could not read from boot partition\n");
		return CMD_RET_FAILURE;
	}
#elif defined(CONFIG_ENV_IS_IN_NAND)
	int dev = nand_curr_device;
	struct mtd_info *mtd;

	mtd = get_nand_dev_by_index(dev);
	check_ret = nand_read_skip_bad(mtd, part_offset, &part_size, NULL, part_size, (u_char *)uimage_addr);
	if (check_ret) {
		printf("Could not read from boot partition\n");
		return CMD_RET_FAILURE;
	}
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(loadboot, 1, 0, do_loadboot,
	"loadboot - according to abdata load uimage from a or b partition\n",
	"loadboot - according to abdata load uimage from a or b partition\n");
#endif

static int do_reinit_abdata(struct cmd_tbl *cmdtp, int flag, int argc, char * const argv[])
{
#if defined(CONFIG_ENV_IS_IN_MMC)
	g_boot_dev.mmc_host = get_mmcdevice();
#elif defined(CONFIG_ENV_IS_IN_NAND)
	g_boot_dev.nand_host = cvsnfc_get_host();
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#else
#endif
	avb_ab_data_init();

	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(reinit_abdata, 1, 0, do_reinit_abdata,
	"reinit_abdata - reinit abdata\n",
	"reinit_abdata - reinit abdata\n");
