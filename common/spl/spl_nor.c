// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012 Stefan Roese <sr@denx.de>
 */

#include <common.h>
#include <image.h>
#include <log.h>
#include <spl.h>
#include "mmio.h"
#include "spi_flash.h"
#include "spi.h"
#include <dm/device-internal.h>

#define REG_BASE                        0x10000000
#define REG_SPI_CTRL                    0x00
#define REG_SPI_CE_CTRL                 0x04
#define REG_SPI_DLY_CTRL                0x08
#define REG_SPI_DMMR                    0x0C
#define REG_SPI_TRAN_CSR                0x10
#define REG_SPI_TRAN_NUM                0x14
#define REG_SPI_FIFO_PORT               0x18
#define REG_SPI_FIFO_PT                 0x20
#define REG_SPI_INT_STS                 0x28
#define REG_SPI_INT_EN                  0x2C
#define REG_SPI_OPT                     0x30

#define BIT_SPI_CTRL_SCK_DIV_MASK       0x7FF
#define BIT_SPI_DLY_CTRL_CET            (3 << 8)
#define BIT_SPI_DLY_CTRL_NEG_SAMPLE     BIT(14)

#ifndef CONFIG_SYS_OS_BASE
#define CONFIG_SYS_OS_BASE		0x300000
#endif

static struct spi_flash *flash;
static int spi_flash_init(void)
{
	unsigned int bus = CONFIG_SF_DEFAULT_BUS;
	unsigned int cs = CONFIG_SF_DEFAULT_CS;
	unsigned int speed = CONFIG_SF_DEFAULT_SPEED;
	unsigned int mode = CONFIG_SF_DEFAULT_MODE;
#if CONFIG_IS_ENABLED(DM_SPI_FLASH)
	struct udevice *new, *bus_dev;
	int ret;
#else
	struct spi_flash *new;
#endif
#if CONFIG_IS_ENABLED(DM_SPI_FLASH)
	/* Remove the old device, otherwise probe will just be a nop */
	ret = spi_find_bus_and_cs(bus, cs, &bus_dev, &new);
	if (!ret)
		device_remove(new, DM_REMOVE_NORMAL);

	flash = NULL;
	ret = spi_flash_probe_bus_cs(bus, cs, speed, mode, &new);

	if (ret) {
		printf("Failed to initialize SPI flash at %u:%u (error %d)\n",
		       bus, cs, ret);
		return 1;
	}

	flash = dev_get_uclass_priv(new);
#else
	if (flash)
		spi_flash_free(flash);

	new = spi_flash_probe(bus, cs, speed, mode);
	flash = new;
	if (!new) {
		printf("Failed to initialize SPI flash at %u:%u\n", bus, cs);
		return 1;
	}
#endif

	return 0;
}

static ulong spl_nor_load_read(struct spl_load_info *load, ulong sector,
			       ulong count, void *buf)
{
	debug("spl_nor_load_read sector %lx, sectors=%lu, dst=%p\n", sector, count, buf);

	if (!flash) {
		printf("No flash init, failed!!");
		return 0;
	}

	return spi_flash_read(flash, sector, count, buf) ? -1 : count;
}

unsigned long __weak spl_nor_get_uboot_base(void)
{
	return 0;//CONFIG_SYS_UBOOT_BASE;	// not use
}

uint32_t swap32(uint32_t value)
{
	return  ((value >> 24) & 0x000000FF) |
			((value >> 8)  & 0x0000FF00) |
			((value << 8)  & 0x00FF0000) |
			((value << 24) & 0xFF000000);
}

static int spl_nor_load_image(struct spl_image_info *spl_image,
			      struct spl_boot_device *bootdev)
{
	__maybe_unused const struct image_header *header;
	__maybe_unused struct image_header *tmp;
	__maybe_unused struct spl_load_info load;

	/*
	 * Loading of the payload to SDRAM is done with skipping of
	 * the mkimage header in this SPL NOR driver
	 */
	spl_image->flags |= SPL_COPY_PAYLOAD_ONLY;

#ifdef CONFIG_SPL_OS_BOOT
	if (!spl_start_uboot()) {
		/*
		 * Load Linux from its location in NOR flash to its defined
		 * location in SDRAM
		 */

		if (spi_flash_init())
			return -1;

		tmp = (struct image_header *)malloc(sizeof(struct image_header));
		if (!tmp) {
			printf("tmp is null\n");
			return -1;
		}
#ifdef CONFIG_SPL_LOAD_FIT
		if (!spl_nor_load_read(NULL, SPL_BOOT_PART_OFFSET, sizeof(struct image_header), tmp)) {
			printf("header read fail\n");
			free(tmp);
			return -1;
		}

		header = (const struct image_header *)tmp;

		debug("magic:%x\n", swap32(header->ih_magic));
		if (swap32(header->ih_magic) == FDT_MAGIC) {
			int ret;

			debug("Found FIT\n");
			load.bl_len = 1;
			load.read = spl_nor_load_read;

			ret = spl_load_simple_fit(spl_image, &load,
						  (SPL_BOOT_PART_OFFSET),
						  (void *)header);

#if defined CONFIG_SYS_SPL_ARGS_ADDR && defined CONFIG_CMD_SPL_NOR_OFS
			memcpy((void *)CONFIG_SYS_SPL_ARGS_ADDR,
			       (void *)CONFIG_CMD_SPL_NOR_OFS,
			       CONFIG_CMD_SPL_WRITE_SIZE);
#endif
			free(tmp);
			return ret;
		}
		printf("Not Found FIT\n");
#endif
		if (image_get_os(header) == IH_OS_LINUX) {
			/* happy - was a Linux */
			int ret;

			ret = spl_parse_image_header(spl_image, header);
			if (ret) {
				free(tmp);
				return ret;
			}
			memcpy((void *)spl_image->load_addr,
			       (void *)((SPL_BOOT_PART_OFFSET) +
					sizeof(struct image_header)),
			       spl_image->size);
#ifdef CONFIG_SYS_FDT_BASE
			spl_image->arg = (void *)CONFIG_SYS_FDT_BASE;
#endif
			free(tmp);
			return 0;
		} else {
			puts("The Expected Linux image was not found.\n"
			     "Please check your NOR configuration.\n"
			     "Trying to start u-boot now...\n");
		}
	}
#endif

	/*
	 * Load real U-Boot from its location in NOR flash to its
	 * defined location in SDRAM
	 */
#ifdef CONFIG_SPL_LOAD_FIT
	header = (const struct image_header *)spl_nor_get_uboot_base();
	if (image_get_magic(header) == FDT_MAGIC) {
		int ret;
		debug("Found FIT format U-Boot\n");
		load.bl_len = 1;
		load.read = spl_nor_load_read;
		ret = spl_load_simple_fit(spl_image, &load,
					   spl_nor_get_uboot_base(),
					   (void *)header);
		if (tmp != NULL)
			free(tmp);
		return ret;
	}
#endif

	if (IS_ENABLED(CONFIG_SPL_LOAD_IMX_CONTAINER)) {
		int ret;
		load.bl_len = 1;
		load.read = spl_nor_load_read;
		ret = spl_load_imx_container(spl_image, &load,
					      spl_nor_get_uboot_base());
		if (tmp != NULL)
			free(tmp);
		return ret;
	}

	/* Legacy image handling */
	if (IS_ENABLED(CONFIG_SPL_LEGACY_IMAGE_SUPPORT)) {
		int ret;
		load.bl_len = 1;
		load.read = spl_nor_load_read;
		ret = spl_load_legacy_img(spl_image, &load,
					   spl_nor_get_uboot_base());
		if (tmp != NULL)
			free(tmp);
		return ret;
	}
	if (tmp != NULL)
		free(tmp);
	return 0;
}
SPL_LOAD_IMAGE_METHOD("NOR", 0, BOOT_DEVICE_NOR, spl_nor_load_image);
