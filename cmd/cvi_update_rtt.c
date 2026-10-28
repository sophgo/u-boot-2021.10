/***************************************
 * function: mars rtt update
 * *****************************************/

#include <common.h>
#include <command.h>
#include <asm/io.h>
#include <ubifs_uboot.h>
#include <serial.h>
#include <asm/global_data.h>
#include <linux/delay.h>
#include <usb/dwc2_udc.h>
#ifdef CONFIG_NAND_SUPPORT
#include <nand.h>
#endif
#include "cvi_update.h"

#include <mmc.h>
#include <blk.h>
#include <fs.h>
#include <stdlib.h>

#define HEADER_SIZE 64
#ifdef CONFIG_CMD_SAVEENV
#define SET_DL_COMPLETE()			\
	do {							\
		env_set("dl_flag", "prog");	\
		run_command("saveenv", 0);	\
	} while (0)
#else
#define SET_DL_COMPLETE() writel(0x50524F47, (unsigned int *)UPGRADE_SRAM_ADDR)
#endif /* CONFIG_CMD_SAVEENV */

#define MAX_LABELS 32

enum parse_state {
	STATE_SEARCH_TAG,
	STATE_IN_PARTITION,
	STATE_READ_SIZE,
	STATE_READ_LABEL
};

static char img_files[MAX_LABELS][128] = {0};
/* Partition size in bytes (size_in_kb * 1024), index-aligned with img_files[].
 * Used to compute each partition's NAND erase offset/size from the XML. */
static uint32_t img_sizes[MAX_LABELS];
static int img_file_cnt;

/* Defined in cvi_update.c; shared boot-source flag read by _prgImage(). */
extern uint32_t update_magic;
/* Image name list defined in imgs.h / cvi_update.c; used by _uart_update(). */
extern char imgs[][255];
extern const int imgs_count;

#ifdef CONFIG_NAND_SUPPORT
#define FIP_MAGIC_NUMBER "CVBL01\n\0"

/* Layout used by cvi_sd_update_spinand_v3.c: 20 reserved blocks,
 * backup starts at block 9, with a 16-byte header per FIP block. */
#define RTT_NAND_FIP_BLOCK_COUNT 20
#define RTT_NAND_FIP_PRIMARY_BLOCKS 9
#define RTT_NAND_FIP_BLOCK_HEADER_SIZE 16

/* RTT NAND packages send boot0 first. The host issues PROGRAM after
 * transferring the chunk header and payload; boot0 must be a single chunk. */
static bool usb_first_file;

static void cvi_update_rtt_usb_reset(void)
{
	usb_first_file = true;
}

int cvi_update_rtt_usb_program(void)
{
	uint32_t *chunk = (void *)(uintptr_t)UPDATE_ADDR;
	uint32_t size, offset, part_size;
	u_char *payload = (void *)(uintptr_t)(UPDATE_ADDR + HEADER_SIZE);
	struct mtd_info *mtd;
	size_t written;
	char cmd[128];
	int ret;

	size = chunk[1];
	offset = chunk[2];
	part_size = chunk[3];
	if (!size || size > MAX_LOADSIZE ||
	    nand_curr_device < 0 ||
	    nand_curr_device >= CONFIG_SYS_MAX_NAND_DEVICE) {
		printf("USB NAND: invalid chunk size or NAND device\n");
		return -1;
	}
	mtd = nand_info[nand_curr_device];
	if (!mtd || mtd->erasesize <= RTT_NAND_FIP_BLOCK_HEADER_SIZE ||
	    !part_size || size > part_size ||
	    offset % mtd->erasesize || part_size % mtd->erasesize ||
	    (u64)offset + part_size > mtd->size ||
	    (!offset && part_size < (u64)RTT_NAND_FIP_BLOCK_COUNT * mtd->erasesize) ||
	    (offset && offset < (u64)RTT_NAND_FIP_BLOCK_COUNT * mtd->erasesize)) {
		printf("USB NAND: invalid partition offset/size\n");
		return -1;
	}

	if (usb_first_file) {
		/* ROM needs FIPH/FIPB block headers and the NAND geometry/CRC.
		 * A raw nand write of boot0 cannot create a bootable NAND FIP. */
		if (offset || size < 0x1000 ||
		    size > (u64)RTT_NAND_FIP_PRIMARY_BLOCKS *
			   (mtd->erasesize - RTT_NAND_FIP_BLOCK_HEADER_SIZE) ||
		    memcmp(payload, FIP_MAGIC_NUMBER, 8)) {
			printf("USB NAND: first file must be boot0 FIP at offset 0, refusing to erase\n");
			return -1;
		}
		ret = env_set_hex("filesize", size);
		if (ret)
			return ret;
		snprintf(cmd, sizeof(cmd), "cvi_sd_update %p spinand fip", payload);
		ret = run_command(cmd, 0);
		if (ret)
			return ret;
		printf("Program USB boot0 done (%u bytes)\n", size);
	} else {
		if (!offset) {
			printf("USB NAND: boot0 must be the first file, refusing to erase\n");
			return -1;
		}
		/* Use the current image's partition coordinates, never the
		 * mtdparts names baked into this download U-Boot. Bound bad-block
		 * skipping to this partition and verify the written payload. */
		snprintf(cmd, sizeof(cmd), "nand erase 0x%x 0x%x", offset, part_size);
		ret = run_command(cmd, 0);
		if (ret)
			return ret;
		written = size;
		ret = nand_write_skip_bad(mtd, offset, &written, NULL, part_size,
					  payload, WITH_WR_VERIFY);
		if (ret || written != size)
			return ret ? ret : -1;
		printf("Program USB partition 0x%x done (%u bytes)\n", offset, size);
	}
	usb_first_file = false;
	return 0;
}
#endif

#if (!defined CONFIG_TARGET_CVITEK_CV181X_FPGA) && (!defined CONFIG_TARGET_CVITEK_ATHENA2_FPGA)
static uint32_t bcd2hex4(uint32_t bcd)
{
	return ((bcd) & 0x0f) + (((bcd) >> 4) & 0xf0) + (((bcd) >> 8) & 0xf00) + (((bcd) >> 12) & 0xf000);
}

static int _checkHeader(char *file, char strStorage[10])
{
	char *magic = (void *)HEADER_ADDR;
	uint32_t version = *(uint32_t *)((uintptr_t)HEADER_ADDR + 4);
	uint32_t chunk_sz = *(uint32_t *)((uintptr_t)HEADER_ADDR + 8);
	uint32_t total_chunk = *(uint32_t *)((uintptr_t)HEADER_ADDR + 12);
	uint32_t file_sz = *(uint32_t *)((uintptr_t)HEADER_ADDR + 16);
	int ret = strncmp(magic, HEADER_MAGIC, 4);

	if (ret) {
		printf("File:%s Magic number is wrong, skip it\n", file);
		return ret;
	}
	printf("Header Version:%d\n", version);
	char cmd[255] = { '\0' };
	uint32_t pos = HEADER_SIZE;
	/* NAND erase is done per-partition in _storage_update_rtt() using the
	 * partition offset/size from the XML, not here by partition name: the
	 * mtdparts baked into u-boot may not match the partition table being
	 * flashed, so erase.part <name> is unreliable. */
	for (int i = 0; i < total_chunk; i++) {
		uint32_t load_size = file_sz > (MAX_LOADSIZE + chunk_sz) ?
				     MAX_LOADSIZE + chunk_sz :
				     file_sz;
		snprintf(cmd, 255, "fatload %s %p %s 0x%x 0x%x;", strStorage,
			 (void *)UPDATE_ADDR, file, load_size, pos);
		pr_debug("%s\n", cmd);
		ret = run_command(cmd, 0);
		if (ret)
			return ret;

		ret = _prgImage((void *)UPDATE_ADDR, chunk_sz, file);
		if (ret == 0) {
			printf("program file:%s failed\n", file);
			break;
		}
		pos += load_size;
		file_sz -= load_size;
	}
	return 0;
}

static void parse_xml(char *xml)
{
	enum parse_state state = STATE_SEARCH_TAG;
	char *ptr = xml;
	int quote_count = 0;
	int label_idx = 0;
	uint32_t size_kb = 0;

	while (*ptr && img_file_cnt < MAX_LABELS) {
		switch (state) {
		case STATE_SEARCH_TAG:
			if (strncmp(ptr, "<partition", 10) == 0) {
				state = STATE_IN_PARTITION;
				/* Reset per-element state so a fileless
				 * <partition> (e.g. RESERVED) cannot leak its
				 * size/label into the next element. */
				size_kb = 0;
				label_idx = 0;
				ptr += 10;
			}
			break;

		case STATE_IN_PARTITION:
			/* Each case ends with ptr++, so advance by (keylen - 1)
			 * to land on the value's opening quote next iteration
			 * (matches the original file= handling). */
			if (strncmp(ptr, "size_in_kb=", 11) == 0) {
				state = STATE_READ_SIZE;
				ptr += 10;
				quote_count = 0;
			} else if (strncmp(ptr, "file=", 5) == 0) {
				state = STATE_READ_LABEL;
				ptr += 4;
				quote_count = 0;
			} else if (*ptr == '>') {
				/* End of element ('>' or '/>'). Commit the
				 * partition regardless of whether it had a
				 * file= attribute, so fileless partitions still
				 * occupy a slot and contribute their size to the
				 * running offset. */
				img_files[img_file_cnt][label_idx] = '\0';
				img_sizes[img_file_cnt] = size_kb * 1024;
				img_file_cnt++;
				state = STATE_SEARCH_TAG;
			}
			break;

		case STATE_READ_SIZE:
			if (*ptr == '"' && ++quote_count == 2) {
				/* back to IN_PARTITION to still catch file= */
				state = STATE_IN_PARTITION;
			} else if (*ptr >= '0' && *ptr <= '9' && quote_count == 1) {
				size_kb = size_kb * 10 + (*ptr - '0');
			}
			break;

		case STATE_READ_LABEL:
			if (*ptr == '"' && ++quote_count == 2) {
				/* Store the file name but do NOT commit here;
				 * commit happens on '>' so size= that appears
				 * after file= is still captured. */
				img_files[img_file_cnt][label_idx] = '\0';
				state = STATE_IN_PARTITION;
			} else if (*ptr != '"' && quote_count == 1) {
				img_files[img_file_cnt][label_idx++] = *ptr;
			}
			break;
		}
		ptr++;
	}
}

int read_parse_xml(int dev_num, int part_num)
{
	struct blk_desc *mmc_dev;

	blk_select_hwpart_devnum(IF_TYPE_MMC, dev_num, part_num);

	#define BUF_SIZE 2048
	static char *xml_buf;
	loff_t file_size;

#if defined(CONFIG_NAND_SUPPORT)
	const char *xml_name = "partition_spinand.xml";
#elif defined(CONFIG_EMMC_SUPPORT)
	const char *xml_name = "partition_emmc.xml";
#else
	const char *xml_name = "partition_spinor.xml";
#endif

	xml_buf = (char *)malloc(BUF_SIZE);
	if (!xml_buf) {
		printf("failed to malloc xml_buf\n");
		return -ENOMEM;
	}

	mmc_dev = blk_get_devnum_by_type(IF_TYPE_MMC, dev_num);

	if (fs_set_blk_dev_with_part(mmc_dev, part_num)) {
		printf("failed to mount sd card\n");
		return -1;
	}

	if (fs_read(xml_name, (ulong)xml_buf, 0, BUF_SIZE, &file_size)) {
		printf("failed to read %s\n", xml_name);
		return -1;
	}

	if (strstr(xml_buf, "<physical_partition") == NULL) {
		printf("invaled file format\n");
		return -EINVAL;
	}

	parse_xml(xml_buf);

	/* Dump the parsed partition layout so the SD-burn flow can be checked
	 * against the table on the card (offset = running sum of prior sizes). */
	printf("\n");
	printf("+-----+--------------------------------+------------+------------+----------+\n");
	printf("| idx | file                           |     offset |       size |     size |\n");
	printf("+-----+--------------------------------+------------+------------+----------+\n");
	uint32_t dump_offset = 0;

	for (int i = 0; i < img_file_cnt; i++) {
		/* Human-readable size: sizes are always KB-aligned, so show MB
		 * when it divides evenly into MB, otherwise KB. */
		char hsize[16];
		uint32_t kb = img_sizes[i] / 1024;

		if (kb && kb % 1024 == 0)
			snprintf(hsize, sizeof(hsize), "%u MB", kb / 1024);
		else
			snprintf(hsize, sizeof(hsize), "%u KB", kb);

		printf("| %3d | %-30s | 0x%08x | 0x%08x | %8s |\n", i,
		       img_files[i][0] ? img_files[i] : "(none)",
		       dump_offset, img_sizes[i], hsize);
		dump_offset += img_sizes[i];
	}
	printf("+-----+--------------------------------+------------+------------+----------+\n");
	printf("\n");

	return 0;
}

static int _storage_update_rtt(void)
{

	int ret = 0;
	char cmd[255] = { '\0' };
	char strStorage[10] = { '\0' };
	uint8_t sd_index = 0;

	printf("Start SD downloading...\n");
	// Consider SD card with MBR as default
#if defined(CONFIG_NAND_SUPPORT) || defined(CONFIG_SPI_FLASH)
	strlcpy(strStorage, "mmc 0:1", 9);
	sd_index = 0;
#elif defined(CONFIG_EMMC_SUPPORT)
	sd_index = 1;
	strlcpy(strStorage, "mmc 1:1", 9);
#endif
	snprintf(cmd, 255, "mmc dev %u:1 SD_HS", sd_index);
	run_command(cmd, 0);

#if defined(CONFIG_SPI_FLASH)
	run_command("sf probe", 0);
#endif

	/* Parse the partition table on the SD card at runtime; the RTT fip.bin
	 * is shared across projects, so the file list cannot be baked into
	 * imgs.h and must come from partition_*.xml. */
	read_parse_xml(sd_index, 1);

	/* Walk the partitions and dispatch by file name (same idea as the
	 * ALIOS cvi_update_rtos burn_nand): the boot slot (boot0 / fip.bin)
	 * is a downloader image burnt via cvi_sd_update, every other partition
	 * is a CIMG chunk image burnt through _checkHeader/_prgImage. */
	uint32_t part_offset = 0;

	for (int i = 0; i < img_file_cnt; i++) {
		int is_boot = !strcmp(img_files[i], "boot0") ||
			      !strcmp(img_files[i], "fip.bin") ||
			      !strcmp(img_files[i], "fip_spl.bin");
		/* Offset of this partition in flash = sum of preceding sizes
		 * (XML partitions are laid out back-to-back). Advance the
		 * running offset for every partition, boot slot included, so
		 * it stays aligned with the XML order even across skips. */
		uint32_t __maybe_unused cur_offset = part_offset;

		part_offset += img_sizes[i];

		/* Fileless partition (e.g. RESERVED): nothing to burn, it only
		 * reserves space, so just keep the offset advanced and move on. */
		if (!img_files[i][0])
			continue;

		/* Boot slot: load the whole image (it is burnt as-is via the fip
		 * path). Other partitions: load only the CIMG header first, the
		 * chunk body is streamed inside _checkHeader/_prgImage. */
		if (is_boot)
			snprintf(cmd, 255, "fatload %s %p %s;", strStorage,
				 (void *)HEADER_ADDR, img_files[i]);
		else
			snprintf(cmd, 255, "fatload %s %p %s 0x%x 0;", strStorage,
				 (void *)HEADER_ADDR, img_files[i], HEADER_SIZE);
		pr_debug("%s\n", cmd);
		ret = run_command(cmd, 0);
		if (ret) {
			printf("load %s failed, skip it!\n", img_files[i]);
			continue;
		}

		if (is_boot) {
			/* The boot image (boot0/fip.bin) may be wrapped in a CIMG
			 * container (64B file header + one chunk header), same as
			 * the other partitions. cvi_sd_update/sf update expect a
			 * raw fip, so strip the headers before handing over the
			 * data. A boot fip always fits in a single chunk, so the
			 * raw fip starts right after the file header + chunk header,
			 * and its real length is the chunk's size field. If the
			 * magic does not match, the image is already raw: program it
			 * from HEADER_ADDR with the loaded ${filesize} as before. */
			void *fip = (void *)HEADER_ADDR;
			char fip_len[16] = "${filesize}";

			if (strncmp((char *)HEADER_ADDR, HEADER_MAGIC, 4) == 0) {
				uint32_t chunk_sz =
					*(uint32_t *)((uintptr_t)HEADER_ADDR + 8);
				/* size lives at chunk_header + 4 (see _prgImage()) */
				uint32_t fip_sz = *(uint32_t *)((uintptr_t)HEADER_ADDR
							+ HEADER_SIZE + 4);

				fip = (void *)((uintptr_t)HEADER_ADDR +
					       HEADER_SIZE + chunk_sz);
				snprintf(fip_len, sizeof(fip_len), "0x%x", fip_sz);
				printf("%s: CIMG header found, skip %d bytes, fip 0x%x\n",
				       img_files[i], HEADER_SIZE + chunk_sz, fip_sz);
			}
#if defined(CONFIG_NAND_SUPPORT)
			(void)fip_len;
			snprintf(cmd, 255, "cvi_sd_update %p spinand fip", fip);
			ret = run_command(cmd, 0);
#elif defined(CONFIG_SPI_FLASH)
			snprintf(cmd, 255,
				 "sf update %p ${fip_PART_OFFSET} %s;",
				 fip, fip_len);
			ret = run_command(cmd, 0);
#elif defined(CONFIG_EMMC_SUPPORT)
			(void)fip_len;
			// Switch to boot partition
			run_command("mmc dev 0 1", 0);
			snprintf(cmd, 255, "mmc write %p 0 0x800;", fip);
			run_command(cmd, 0);
			snprintf(cmd, 255, "mmc write %p 0x800 0x800;", fip);
			ret = run_command(cmd, 0);
			// Switch to user partition
			run_command("mmc dev 0 0", 0);
#endif
			printf("Program %s done\n", img_files[i]);
			continue;
		}

#if defined(CONFIG_NAND_SUPPORT)
		/* Erase the whole partition by its XML offset/size before
		 * writing (mtdparts baked into u-boot may not match the table
		 * being flashed, so erase.part <name> is unreliable). */
		if (img_sizes[i]) {
			snprintf(cmd, 255, "nand erase 0x%x 0x%x;", cur_offset,
				 img_sizes[i]);
			pr_debug("%s\n", cmd);
			run_command(cmd, 0);
		}
#endif

		if (_checkHeader(img_files[i], strStorage))
			continue;
	}

	SET_DL_COMPLETE();
	return ret;
}

static int _usb_update(uint32_t usb_pid)
{
	int ret = 0;
	char utask_cmd[255] = { '\0' };

	printf("Start USB downloading...\n");

	// Clean download flags
	writel(0x0, (unsigned int *)BOOT_SOURCE_FLAG_ADDR); //mw.l 0xe00fc00 0x0;
	// Always download Fip first
	snprintf(utask_cmd, 255, "cvi_utask vid 0x3346 pid 0x%x", usb_pid);
#ifdef CONFIG_NAND_SUPPORT
	/* boot0 is programmed by the USB PROGRAM callback after receiving it. */
	cvi_update_rtt_usb_reset();
#elif defined(CONFIG_SPI_FLASH)
	ret = run_command("sf probe", 0);
#else
	{
		char cmd[255] = { '\0' };

		// Switch to boot partition
		run_command("mmc dev 0 1", 0);
		snprintf(cmd, 255, "mmc write %p 0 0x800;", (void *)UPDATE_ADDR);
		pr_debug("%s\n", cmd);
		run_command(cmd, 0);
		snprintf(cmd, 255, "mmc write %p 0x800 0x800;", (void *)UPDATE_ADDR);
		pr_debug("%s\n", cmd);
		run_command(cmd, 0);
		printf("Program fip.bin done\n");
		// Switch to user partition
		run_command("mmc dev 0 0", 0);
	}
#endif
	// Since device will reset by host tool, set flag first
	SET_DL_COMPLETE();
	while (1) {
		ret = run_command(utask_cmd, 0);
		if (ret) {
			pr_debug("cvi_utask failed(%d)\n", ret);
			return ret;
		}
		//_prgImage((void *)UPDATE_ADDR, readl(HEADER_ADDR + 8));
	};
	return 0;
}
#endif

DECLARE_GLOBAL_DATA_PTR;
static void set_baudrate(unsigned int baudrate)
{
	mdelay(50);
	gd->baudrate = baudrate;
	serial_setbrg();
	mdelay(50);
}

static int uart_download(void *buf, const char *filename)
{
	int ret = 0;
	char cmd[255] = { '\0' };

	snprintf(cmd, 255, "loadb %p %d ", (void *)HEADER_ADDR, UART_DL_BAUDRATE);
	ret = run_command(cmd, 0);
	if (ret)
		return ret;

	char *magic = (void *)HEADER_ADDR;

	if (!strncmp(magic, "O", 1)) {
		printf("File %s not exist, skip it!\n", filename);
		return ret;
	}

	uint32_t version = *(uint32_t *)((uintptr_t)HEADER_ADDR + 4);
	uint32_t chunk_header_sz = *(uint32_t *)((uintptr_t)HEADER_ADDR + 8);
	uint32_t total_chunk = *(uint32_t *)((uintptr_t)HEADER_ADDR + 12);
	uint32_t file_sz = *(uint32_t *)((uintptr_t)HEADER_ADDR + 16);
#ifdef CONFIG_NAND_SUPPORT
	char *extra = (void *)((uintptr_t)HEADER_ADDR + 20);
	static char prevExtra[EXTRA_FLAG_SIZE + 1] = { '\0' };
#endif

	ret = strncmp(magic, HEADER_MAGIC, 4);
	if (ret) {
		printf("File %s's magic number is wrong, skip it!\n", filename);
		return ret;
	}

	printf("Header Version:%d\n", version);
	uint32_t pos = HEADER_SIZE;
#ifdef CONFIG_NAND_SUPPORT
	// Erase partition first
	if (strncmp(extra, prevExtra, EXTRA_FLAG_SIZE)) {
		strncpy(prevExtra, extra, EXTRA_FLAG_SIZE);
		snprintf(cmd, 255, "nand erase.part -y %s", prevExtra);
		pr_debug("%s\n", cmd);
		run_command(cmd, 0);
	}
#endif

	for (int i = 0; i < total_chunk; i++) {
		uint32_t load_size = file_sz > (MAX_LOADSIZE + chunk_header_sz) ?
				     MAX_LOADSIZE + chunk_header_sz :
				     file_sz;
		snprintf(cmd, 255, "loadb %p %d ", (void *)UPDATE_ADDR, UART_DL_BAUDRATE);
		pr_debug("%s\n", cmd);
		ret = run_command(cmd, 0);
		if (ret)
			return ret;

		ret = _prgImage((void *)UPDATE_ADDR, chunk_header_sz, NULL);
		if (ret == 0) {
			printf("program file:%s failed\n", filename);
			break;
		}
		pos += load_size;
		file_sz -= load_size;
	}
	return 0;
}

static int _uart_update(void)
{
	int ret = 0;
	char cmd[255] = { '\0' };

	printf("Start UART downloading... Change boadrate to %d\n", UART_DL_BAUDRATE);
	set_baudrate(UART_DL_BAUDRATE);

	snprintf(cmd, 255, "loadb %p %d ", (void *)HEADER_ADDR, UART_DL_BAUDRATE);
	ret = run_command(cmd, 0);
	if (ret) {
		printf("Download fip.bin failed!\n");
		return ret;
	}

#ifdef CONFIG_NAND_SUPPORT
	snprintf(cmd, 255, "cvi_sd_update %p spinand fip", (void *)UPDATE_ADDR);
	pr_debug("%s\n", cmd);
	ret = run_command(cmd, 0);
#elif defined(CONFIG_SPI_FLASH)
	ret = run_command("sf probe", 0);
	snprintf(cmd, 255, "sf update %p ${fip_PART_OFFSET} ${fip_PART_SIZE};", (void *)UPDATE_ADDR);
	pr_debug("%s\n", cmd);
	ret = run_command(cmd, 0);
#else
	// Switch to boot partition
	ret = run_command("mmc dev 0 1", 0);
	snprintf(cmd, 255, "mmc write %p 0 0x800;", (void *)UPDATE_ADDR);
	pr_debug("%s\n", cmd);
	ret = run_command(cmd, 0);
	snprintf(cmd, 255, "mmc write %p 0x800 0x800;", (void *)UPDATE_ADDR);
	pr_debug("%s\n", cmd);
	ret = run_command(cmd, 0);
	// Switch to user partition
	ret = run_command("mmc dev 0 0", 0);
#endif
	if (ret) {
		printf("Program fip.bin failed!\n");
		return ret;
	}

	SET_DL_COMPLETE();
	printf("Program fip.bin done\n");

	for (int i = 1; i < imgs_count; i++) {
		ret = uart_download((void *)HEADER_ADDR, imgs[i]);
		if (ret) {
			printf("Load %s failed, skip it!\n", imgs[i]);
			continue;
		}
	}
	// set_baudrate(CONFIG_BAUDRATE);

	return ret;
}

int do_cvi_update_rtt(struct cmd_tbl *cmdtp, int flag, int argc,
			     char *const argv[])
{

#if (!defined CONFIG_TARGET_CVITEK_CV181X_FPGA) && (!defined CONFIG_TARGET_CVITEK_ATHENA2_FPGA)
	int ret = 1;
	uint32_t usb_pid = 0;

	if (argc == 1) {
		update_magic = readl((unsigned int *)BOOT_SOURCE_FLAG_ADDR);
		if (update_magic == UART_UPDATE_MAGIC) {
			run_command("env default -a", 0);
			ret = _uart_update();
		} else if (update_magic == SD_UPDATE_MAGIC) {
			run_command("env default -a", 0);
			ret = _storage_update_rtt();
		} else if (update_magic == USB_UPDATE_MAGIC) {
			#ifdef CONFIG_CHECK_USB_PLUG
			if (cvi_get_chg_plug() != CHG_PLUG_HUB) {
				printf("usb download but not plug into pc\n");
				return -1;
			}
			#endif
			run_command("env default -a", 0);
			usb_pid = in_be32(UBOOT_PID_SRAM_ADDR);
			usb_pid = bcd2hex4(usb_pid);
			ret = _usb_update(usb_pid);
		}
	} else {
		printf("Usage:\n%s\n", cmdtp->usage);
	}

	return ret;
#else
	return 0;
#endif
}
