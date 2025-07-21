// SPDX-License-Identifier: GPL-2.0-or-later

#include <common.h>
#include <malloc.h>
#include "cvi_vendor.h"
#include "blk.h"
#include "nand.h"
#include "command.h"
#include "ubi_uboot.h"

/* tag for vendor check */
#define VENDOR_TAG 0x524B5644
/* The Vendor partition contains the number of Vendor blocks */
#define NAND_VENDOR_PART_NUM 1
#define VENDOR_PART_NUM 4
/* align to 64 bytes */
#define VENDOR_BTYE_ALIGN 0x3F
#define VENDOR_BLOCK_SIZE 512

/* --- Emmc define --- */
/* Starting address of the Vendor in memory. */
#define EMMC_VENDOR_PART_OFFSET 0
// #define EMMC_VENDOR_PART_OFFSET (1024 * 7)
/*
 * The number of memory blocks used by each
 * Vendor structure(128 * 512B = 64KB)
 */
#define EMMC_VENDOR_PART_BLKS 128
/* The maximum number of items in each Vendor block */
#define EMMC_VENDOR_ITEM_NUM 126

/* --- Spi Nand/SLC/MLC large capacity case define --- */
/* The Vendor partition contains the number of Vendor blocks */
#define NAND_VENDOR_PART_OFFSET 0
/*
 * The number of memory blocks used by each
 * Vendor structure(8 * 512B = 4KB)
 */
#define NAND_VENDOR_PART_BLKS 128
/* The maximum number of items in each Vendor block */
#define NAND_VENDOR_ITEM_NUM 126

/* --- Spi/Spi Nand/SLC/MLC small capacity case define --- */
/* The Vendor partition contains the number of Vendor blocks */
#define FLASH_VENDOR_PART_OFFSET 8
/*
 * The number of memory blocks used by each
 * Vendor structure(8 * 512B = 4KB)
 */
#define FLASH_VENDOR_PART_BLKS 8
/* The maximum number of items in each Vendor block */
#define FLASH_VENDOR_ITEM_NUM 62

/* Vendor uinit test define */
int vendor_storage_test(void);

struct vendor_hdr {
	u32 tag;
	u32 version;
	u16 next_index;
	u16 item_num;
	u16 free_offset; /* Free space offset */
	u16 free_size;   /* Free space size */
};

/*
 * Different types of Flash vendor info are different.
 * EMMC:EMMC_VENDOR_PART_BLKS * BLOCK_SIZE(512) = 64KB;
 * Spi Nor/Spi Nand/SLC/MLC: FLASH_VENDOR_PART_BLKS *
 * BLOCK_SIZE(512) = 4KB.
 * hash: For future expansion.
 * version2: Together with hdr->version, it is used to
 * ensure the current Vendor block content integrity.
 *   (version2 == hdr->version):Data valid;
 *   (version2 != hdr->version):Data invalid.
 */
struct vendor_info {
	struct vendor_hdr *hdr;
	struct vendor_item *item;
	u8 *data;
	u32 *hash;
	u32 *version2;
};

/*
 * Calculate the offset of each field for emmc.
 * Emmc vendor info size: 64KB
 */
#define EMMC_VENDOR_INFO_SIZE (EMMC_VENDOR_PART_BLKS * VENDOR_BLOCK_SIZE)
#define EMMC_VENDOR_DATA_OFFSET (sizeof(struct vendor_hdr) + EMMC_VENDOR_ITEM_NUM * sizeof(struct vendor_item))
#define EMMC_VENDOR_HASH_OFFSET (EMMC_VENDOR_INFO_SIZE - 8)
#define EMMC_VENDOR_VERSION2_OFFSET (EMMC_VENDOR_INFO_SIZE - 4)

/*
 * Calculate the offset of each field for spi nand/slc/mlc large capacity case.
 * Flash vendor info size: 4KB
 */
#define NAND_VENDOR_INFO_SIZE	(NAND_VENDOR_PART_BLKS * VENDOR_BLOCK_SIZE)
#define NAND_VENDOR_DATA_OFFSET	(sizeof(struct vendor_hdr) + NAND_VENDOR_ITEM_NUM * sizeof(struct vendor_item))
#define NAND_VENDOR_HASH_OFFSET (NAND_VENDOR_INFO_SIZE - 8)
#define NAND_VENDOR_VERSION2_OFFSET (NAND_VENDOR_INFO_SIZE - 4)

/* vendor info */
static struct vendor_info vendor_info;
/* The storage type of the device */
static int bootdev_type;

/* vendor private read write ops*/
static	int (*_flash_read)(struct blk_desc *dev_desc,
			   u32 sec,
			   u32 n_sec,
			   void *buffer);
static	int (*_flash_write)(struct blk_desc *dev_desc,
			    u32 sec,
			    u32 n_sec,
			    void *buffer);

int flash_vendor_dev_ops_register(int (*read)(struct blk_desc *dev_desc,
					      u32 sec,
					      u32 n_sec,
					      void *p_data),
				  int (*write)(struct blk_desc *dev_desc,
					       u32 sec,
					       u32 n_sec,
					       void *p_data))
{
	if (!_flash_read) {
		_flash_read = read;
		_flash_write = write;
		return 0;
	}

	return -EPERM;
}

#if defined(CONFIG_ENV_IS_IN_NAND)
char *vendor_volname = "vendor_vol0";

static int nand_erase_part(void)
{
	int ret;
	int dev = nand_curr_device;
	struct mtd_info *mtd;
	nand_erase_options_t opts = {0};

	mtd = get_nand_dev_by_index(dev);
	if (!mtd) {
		printf("Invalid NAND device: %d\n", dev);
		return -ENODEV;
	}
	opts.offset = VENDOR_PART_OFFSET;
	opts.length = VENDOR_PART_SIZE;
	opts.jffs2  = 0;
	opts.quiet  = 0;
	opts.spread = 0;
	ret = nand_erase_opts(mtd, &opts);
	if (ret) {
		debug("nand_erase failed\n");
		return -1;
	}

	return ubi_part("VENDOR", NULL);
}

static int ubi_part_vendor(u32 n_sec)
{
	static int has_ubi_part;
	int ret = 0;

	if (has_ubi_part)
		return 0;

	ret = ubi_part("VENDOR", NULL) ? nand_erase_part() : 0;
	if (ret) {
		printf("ubi_part(VENDOR) failed\n");
		return -1;
	}

	if (!ubi_find_volume(vendor_volname)) {
		ret = ubi_create_vol(vendor_volname, n_sec * VENDOR_BLOCK_SIZE, 1, -1, 0);
		if (ret) {
			printf("create vol(%s) failed\n", vendor_volname);
			return -1;
		}
	}

	has_ubi_part = 1;
	return 0;
}

static int nand_vendor_read_by_ubi(struct blk_desc *dev_desc,
			    u32 sec,
			    u32 n_sec,
			    void *buffer)
{
	int ret = ubi_part_vendor(n_sec);

	if (ret) {
		printf("ubi_part(vendor) failed\n");
		return -1;
	}
	ret = ubi_volume_read(vendor_volname, buffer, n_sec * VENDOR_BLOCK_SIZE);
	if (ret) {
		printf("Could not read from %s partition\n", vendor_volname);
		return -1;
	}
	return n_sec;
}

static int nand_vendor_write_by_ubi(struct blk_desc *dev_desc,
			    u32 sec,
			    u32 n_sec,
			    void *buffer)
{
	int ret = ubi_part_vendor(n_sec);

	if (ret) {
		printf("ubi_part(vendor) failed\n");
		return -1;
	}
	ret = ubi_volume_write(vendor_volname, buffer, n_sec * VENDOR_BLOCK_SIZE);
	if (ret) {
		printf("Could not write to %s partition\n", vendor_volname);
		return -1;
	}
	return n_sec;
}
#endif

struct blk_desc *cvitek_get_bootdev(void)
{
	struct blk_desc *dev_desc;
#if defined(CONFIG_ENV_IS_IN_MMC)
	dev_desc = blk_get_devnum_by_type(IF_TYPE_MMC, 0);
#elif defined(CONFIG_ENV_IS_IN_NAND)
	// dev_desc = blk_get_devnum_by_type(IF_TYPE_SPINAND, 0);
	static struct blk_desc nand_dev_desc = {0};

	nand_dev_desc.if_type = IF_TYPE_SPINAND;
	dev_desc = &nand_dev_desc;
#endif

	if (!dev_desc) {
		printf("%s: Can't find dev_desc!\n", __func__);
		return NULL;
	}
	return dev_desc;
}

static int vendor_ops(u8 *buffer, u32 addr, u32 n_sec, int write)
{
	struct blk_desc *dev_desc;
	unsigned int lba = 0;
	int ret = 0;

	dev_desc = cvitek_get_bootdev();
	if (!dev_desc) {
		printf("%s: dev_desc is NULL!\n", __func__);
		return -ENODEV;
	}

	/* Get the offset address according to the device type */
	switch (dev_desc->if_type) {
	case IF_TYPE_MMC:
		/*
		 * The location of VendorStorage in Flash is shown in the
		 * following figure. The starting address of the VendorStorage
		 * partition offset is 3.5MB(EMMC_VENDOR_PART_OFFSET*BLOCK_SIZE(512)),
		 * and the partition size is 256KB.
		 * ----------------------------------------------------
		 * |   3.5MB    |  VendorStorage  |                   |
		 * ----------------------------------------------------
		 */
		lba = VENDOR_PART_OFFSET;
		debug("[Vendor INFO]:VendorStorage offset address=0x%x\n", lba);
		break;
	case IF_TYPE_SPINAND:
		/*
		 * The location of VendorStorage in Flash is shown in the
		 * following figure. The starting address of the VendorStorage
		 * partition offset is 0KB in FTL vendor block,
		 * and the partition size is 128KB.
		 * ----------------------------------------------------
		 * |  VendorStorage  |                     |
		 * ----------------------------------------------------
		 */
		lba = NAND_VENDOR_PART_OFFSET;
		debug("[Vendor INFO]:VendorStorage offset address=0x%x\n", lba);
		break;
	default:
		printf("[Vendor ERROR]:Boot device type is invalid!\n");
		return -ENODEV;
	}

	if (write) {
		if (_flash_write)
			ret = _flash_write(dev_desc, lba + addr, n_sec, buffer);
		else
			ret = blk_dwrite(dev_desc, lba + addr, n_sec, buffer);
	} else {
		if (_flash_read)
			ret = _flash_read(dev_desc, lba + addr, n_sec, buffer);
		else
			ret = blk_dread(dev_desc, lba + addr, n_sec, buffer);
	}

	debug("[Vendor INFO]:op=%s, ret=%d\n", write ? "write" : "read", ret);

	return ret;
}

int vendor_storage_init(void)
{
	int ret = 0;
	int ret_size;
	u8 *buffer;
	u32 size, i;
	u32 max_ver = 0;
	u32 max_index = 0;
	u16 data_offset, hash_offset, part_num;
	u16 version2_offset, part_size;
	struct blk_desc *dev_desc;

	//	dev_desc = rockchip_get_bootdev();

	dev_desc = cvitek_get_bootdev();

	if (!dev_desc) {
		printf("[Vendor ERROR]:Invalid boot device type(%d)\n",
		       bootdev_type);
		return -ENODEV;
	}

	switch (dev_desc->if_type) {
	case IF_TYPE_MMC:
		size = EMMC_VENDOR_INFO_SIZE;
		part_size = EMMC_VENDOR_PART_BLKS;
		data_offset = EMMC_VENDOR_DATA_OFFSET;
		hash_offset = EMMC_VENDOR_HASH_OFFSET;
		version2_offset = EMMC_VENDOR_VERSION2_OFFSET;
		part_num = VENDOR_PART_NUM;
		break;

#if defined(CONFIG_ENV_IS_IN_NAND)
	case IF_TYPE_SPINAND:
		size = NAND_VENDOR_INFO_SIZE;
		part_size = NAND_VENDOR_PART_BLKS;
		data_offset = NAND_VENDOR_DATA_OFFSET;
		hash_offset = NAND_VENDOR_HASH_OFFSET;
		version2_offset = NAND_VENDOR_VERSION2_OFFSET;
		part_num = NAND_VENDOR_PART_NUM;
		flash_vendor_dev_ops_register(nand_vendor_read_by_ubi, nand_vendor_write_by_ubi);
		break;
#endif

	default:
		debug("[Vendor ERROR]:Boot device type is invalid!\n");
		ret = -ENODEV;
		break;
	}
	/* Invalid bootdev type */
	if (ret)
		return ret;

	/* Initialize */
	bootdev_type = dev_desc->if_type;

	/* Always use, no need to release */
	buffer = (u8 *)malloc(size);
	if (!buffer) {
		printf("[Vendor ERROR]:Malloc failed!\n");
		return -ENOMEM;
	}
	/* Pointer initialization */
	vendor_info.hdr = (struct vendor_hdr *)buffer;
	vendor_info.item = (struct vendor_item *)(buffer + sizeof(struct vendor_hdr));
	vendor_info.data = buffer + data_offset;
	vendor_info.hash = (u32 *)(buffer + hash_offset);
	vendor_info.version2 = (u32 *)(buffer + version2_offset);

	/* Find valid and up-to-date one from (vendor0 - vendor3) */
	for (i = 0; i < part_num; i++) {
		ret_size = vendor_ops((u8 *)vendor_info.hdr, part_size * i, part_size, 0);
		if (ret_size != part_size) {
			printf("part[%d] read failed, ret_size=%d, part_size=%d\n", i, ret_size, part_size);
			ret = -EIO;
			goto out;
		}

		if (vendor_info.hdr->tag == VENDOR_TAG && *vendor_info.version2 == vendor_info.hdr->version) {
			if (max_ver < vendor_info.hdr->version) {
				max_index = i;
				max_ver = vendor_info.hdr->version;
			}
		}
	}

	if (max_ver) {
		debug("[Vendor INFO]:max_ver=%d, vendor_id=%d.\n", max_ver, max_index);
		/*
		 * Keep vendor_info the same as the largest
		 * version of vendor
		 */
		if (max_index != (part_num - 1)) {
			ret_size = vendor_ops((u8 *)vendor_info.hdr, part_size * max_index, part_size, 0);
			if (ret_size != part_size) {
				printf("part[%d] read failed, ret_size=%d, part_size=%d\n",
					max_index, ret_size, part_size);
				ret = -EIO;
				goto out;
			}
		}
	} else {
		debug("[Vendor INFO]:Reset vendor info...\n");
		memset((u8 *)vendor_info.hdr, 0, size);
		vendor_info.hdr->version = 1;
		vendor_info.hdr->tag = VENDOR_TAG;
		/* data field length */
		vendor_info.hdr->free_size = (u32)((size_t)vendor_info.hash - (u32)(size_t)vendor_info.data);
		*vendor_info.version2 = vendor_info.hdr->version;
	}
	debug("[Vendor INFO]:ret=%d.\n", ret);

out:
	if (ret)
		bootdev_type = 0;

	return ret;
}

int vendor_storage_read(u16 id, void *pbuf, u16 size)
{
	int ret = 0;
	u32 i;
	u16 offset;
	struct vendor_item *item;

	/* init vendor storage */
	if (!bootdev_type) {
		ret = vendor_storage_init();
		if (ret < 0)
			return ret;
	}

	item = vendor_info.item;
	for (i = 0; i < vendor_info.hdr->item_num; i++) {
		if ((item + i)->id == id) {
			//debug("[Vendor INFO]:Find the matching item, id=%d\n", id);
			/* Correct the size value */
			if (size > (item + i)->size)
				size = (item + i)->size;
			offset = (item + i)->offset;
			memcpy(pbuf, (vendor_info.data + offset), size);
			return size;
		}
	}

	return -EINVAL;
}

int vendor_storage_write(u16 id, void *pbuf, u16 size)
{
	int cnt, ret = 0;
	u32 i, next_index, align_size;
	struct vendor_item *item;
	u16 part_size, max_item_num, offset, part_num;

	/* init vendor storage */
	if (!bootdev_type) {
		ret = vendor_storage_init();
		if (ret < 0)
			return ret;
	}

	switch (bootdev_type) {
	case IF_TYPE_MMC:
		part_size = EMMC_VENDOR_PART_BLKS;
		max_item_num = EMMC_VENDOR_ITEM_NUM;
		part_num = VENDOR_PART_NUM;
		break;
	case IF_TYPE_SPINAND:
		part_size = NAND_VENDOR_PART_BLKS;
		max_item_num = NAND_VENDOR_ITEM_NUM;
		part_num = NAND_VENDOR_PART_NUM;
		break;
	default:
		ret = -ENODEV;
		break;
	}

	/* Invalid bootdev? */
	if (ret < 0)
		return ret;

	next_index = vendor_info.hdr->next_index;
	/* algin to 64 bytes*/
	align_size = (size + VENDOR_BTYE_ALIGN) & (~VENDOR_BTYE_ALIGN);
	if (size > align_size)
		return -EINVAL;

	item = vendor_info.item;
	/* If item already exist, update the item data */
	for (i = 0; i < vendor_info.hdr->item_num; i++) {
		if ((item + i)->id == id) {
			debug("[Vendor INFO]:Find the matching item, id=%d\n", id);
			offset = (item + i)->offset;
			memcpy((vendor_info.data + offset), pbuf, size);
			(item + i)->size = size;
			vendor_info.hdr->version++;
			*vendor_info.version2 = vendor_info.hdr->version;
			vendor_info.hdr->next_index++;
			if (vendor_info.hdr->next_index >= part_num)
				vendor_info.hdr->next_index = 0;
			cnt = vendor_ops((u8 *)vendor_info.hdr, part_size * next_index, part_size, 1);
			return (cnt == part_size) ? size : -EIO;
		}
	}
	/*
	 * If item does not exist, and free size is enough,
	 * creat a new one
	 */
	if (vendor_info.hdr->item_num < max_item_num && vendor_info.hdr->free_size >= align_size) {
		debug("[Vendor INFO]:Create new Item, id=%d\n", id);
		item = vendor_info.item + vendor_info.hdr->item_num;
		item->id = id;
		item->offset = vendor_info.hdr->free_offset;
		item->size = size;

		vendor_info.hdr->free_offset += align_size;
		vendor_info.hdr->free_size -= align_size;
		memcpy((vendor_info.data + item->offset), pbuf, size);
		vendor_info.hdr->item_num++;
		vendor_info.hdr->version++;
		vendor_info.hdr->next_index++;
		*vendor_info.version2 = vendor_info.hdr->version;
		if (vendor_info.hdr->next_index >= part_num)
			vendor_info.hdr->next_index = 0;

		cnt = vendor_ops((u8 *)vendor_info.hdr, part_size * next_index, part_size, 1);
		return (cnt == part_size) ? size : -EIO;
	}
	debug("[Vendor ERROR]:Vendor has no space left!\n");

	return -ENOMEM;
}

//check vendor id , correct 1, error 0
static int check_vendor_id(int id)
{
	if (id < VENDOR_SN_ID || id > VENDOR_UPDATE_FLAG) {
		printf("id error [%d] .\n", id);
		printf("1 SN\n");
		printf("2 WIFI MAC\n");
		printf("3 LAN MAC\n");
		printf("4 BLUETOOTH MAC\n");
		return 0;
	} else {
		return 1;
	}
}

//write empty content instead of delete function
int cvi_vendor_del(int id)
{
	if (check_vendor_id(id) != 1)
		return -1;

	int ret = 0;

	ret = vendor_storage_init();
	if (ret) {
		printf("%s: vendor storage init failed, ret = %d\n", __func__, ret);
		return ret;
	}

	ret = vendor_storage_write(id, "", VENDOR_BLOCK_SIZE);
	if (ret < 0) {
		printf("[Vendor Test]:vendor delete failed(id=%d)\n", id);
		return ret;
	}

	return 0;
}

static int do_cvi_vendor_read(struct cmd_tbl *cmdt, int flag, int argc, char *const argv[])
{
	if (argc != 2) {
		printf("%s\n", cmdt->usage);
		return 0;
	}
	int ret = 0;
	char serialno_str[513] = {0};
	u16 id = 0;

	id = simple_strtoul(argv[1], NULL, 10);

	if (check_vendor_id(id) != 1)
		return 0;

	ret = vendor_storage_read(id, serialno_str, 512);
	printf("id:%d,  %s\n", id, serialno_str);

	return 0;
}

static int do_cvi_vendor_write(struct cmd_tbl *cmdt, int flag, int argc, char *const argv[])
{
	if (argc != 3) {
		printf("%s\n", cmdt->usage);
		return 0;
	}

	u8 buffer[64] = {0};
	int ret = 0;
	u16 id = 0;

	id = simple_strtoul(argv[1], NULL, 10);

	if (check_vendor_id(id) != 1)
		return 0;

	printf("[%s] will be writen\n", argv[2]);
	memcpy(buffer, argv[2], 64);

	ret = vendor_storage_write(id, buffer, 64);
	if (ret < 0) {
		printf("[Vendor Test]:vendor write failed(id=%d)!\n", id);
		return ret;
	}

	return 0;
}

U_BOOT_CMD(vendor_read, 2, 0, do_cvi_vendor_read, "vendor_read", "");
U_BOOT_CMD(vendor_write, 3, 0, do_cvi_vendor_write, "vendor_write", "");
