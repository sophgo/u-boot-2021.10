// SPDX-License-Identifier: GPL-2.0+
/*
 * (C) Copyright 2013
 * David Feng <fenghua@phytium.com.cn>
 * Sharma Bhupesh <bhupesh.sharma@freescale.com>
 *
 */
#include <common.h>
#include <dm.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <console.h>
#include <linux/compiler.h>
#if defined(__aarch64__)
#include <asm/armv8/mmu.h>
#endif
#include <usb/dwc2_udc.h>
#include <usb.h>
#include "cv84x6_reg.h"
#include "mmio.h"
#include "pinmux/cv84x6_pinmux.h"
#include <linux/delay.h>
#include <bootstage.h>
#include <version.h>
#include <command.h>
#include <mmc.h>
#include <net.h>
#include <serial.h>
#include <part.h>
#include <fat.h>

#ifdef CONFIG_VIDEO_SOPH
#include <video_soph.h>
#endif

#include <linux/stringify.h>
#include <fdt_support.h>

#if defined(__riscv)
#include <asm/csr.h>
#endif

/*
 * PCIe controllers are brought up by FSBL. Tell whether a controller was
 * configured as RC by reading its SII device_type field; only then does the
 * kernel driver have a usable controller. If not, disable the node passed
 * to the kernel.
 *
 * ctrl0(X8_0)  0x21302050
 * ctrl1(X2_0)  0x21306050
 * ctrl2(X4_0)  0x2130a050
 * ctrl3(X2_1)  0x2130e050
 *
 * A controller taken by C2C has its sub-cfg register programmed by FSBL;
 * the kernel must not touch such a controller either. RC + not claimed by
 * C2C -> keep okay, otherwise disable.
 *
 * ctrl0(X8_0)  0x21790400
 * ctrl1(X2_0)  0x21793030
 * ctrl2(X4_0)  0x2179302c
 * ctrl3(X2_1)  0x21793034
 * bit[19:0] != 0 means the controller is used for C2C.
 */
#ifdef CONFIG_OF_BOARD_SETUP
#define PCIE_CFG_BASE_84X6	0x21300000
#define PCIE_CFG_SIZE_84X6	0x4000
#define PCIE_CFG_SII_OFFSET_84X6	0x2000
#define PCIE_SII_DEV_TYPE_REG	0x50
#define PCIE_SII_DEV_TYPE_SHIFT	9
#define PCIE_SII_DEV_TYPE_MASK	(0xf << PCIE_SII_DEV_TYPE_SHIFT)
#define PCIE_SII_DEV_TYPE_RC	(0x4 << PCIE_SII_DEV_TYPE_SHIFT)

#define PCIE_CTRL_MAX_84X6	4

/* per-controller c2c sub-cfg reg, bit[19:0] set when c2c uses the ctrl */
static const uintptr_t pcie_c2c_cfg[PCIE_CTRL_MAX_84X6] = {
	0x21790400,	/* X8_0 */
	0x21793030,	/* X2_0 */
	0x2179302c,	/* X4_0 */
	0x21793034,	/* X2_1 */
};

#define PCIE_C2C_CFG_MASK	0x000fffff

static const char *const pcie_aliases[PCIE_CTRL_MAX_84X6] = {
	"pcie0", "pcie1", "pcie2", "pcie3",
};

static int cv84x6_pcie_fixup(void *blob)
{
	int i;

	for (i = 0; i < PCIE_CTRL_MAX_84X6; i++) {
		uintptr_t ctrl_base = PCIE_CFG_BASE_84X6 +
				      i * PCIE_CFG_SIZE_84X6;
		int ret;

		/* controller not brought out of reset by FSBL → disable */
		if ((mmio_read_32(ctrl_base + 0x60) & 0x3) != 0x3) {
			printf("pcie%d: not initialized, disable node\n", i);
			ret = fdt_status_disabled_by_alias(blob, pcie_aliases[i]);
		} else {
			uintptr_t sii_dev_type = ctrl_base +
						 PCIE_CFG_SII_OFFSET_84X6 +
						 PCIE_SII_DEV_TYPE_REG;
			uint32_t dev_type = mmio_read_32(sii_dev_type);
			uint32_t c2c_cfg = mmio_read_32(pcie_c2c_cfg[i]);
			bool is_rc = (dev_type & PCIE_SII_DEV_TYPE_MASK) ==
				     PCIE_SII_DEV_TYPE_RC;
			bool c2c_in_use = (c2c_cfg & PCIE_C2C_CFG_MASK) != 0;

			if (is_rc && !c2c_in_use) {
				printf("pcie%d: RC (0x%x), c2c unused (0x%x), keep okay\n",
				       i, dev_type, c2c_cfg);
				ret = fdt_status_okay_by_alias(blob, pcie_aliases[i]);
			} else {
				printf("pcie%d: not usable (dev_type 0x%x c2c 0x%x), disable node\n",
				       i, dev_type, c2c_cfg);
				ret = fdt_status_disabled_by_alias(blob, pcie_aliases[i]);
			}
		}

		if (ret < 0)
			printf("pcie%d: failed to set status: %s\n", i,
			       fdt_strerror(ret));
	}

	return 0;
}
#endif /* CONFIG_OF_BOARD_SETUP */

#define DTSNAME_MAX_LEN 32
void get_dts_type_from_oem(unsigned char *dtsname);
void get_dts_type_from_sram(unsigned char *dtsname);
u64 get_ddr_size_from_sram(void);
DECLARE_GLOBAL_DATA_PTR;
#define SD1_SDIO_PAD

#ifdef CONFIG_SPL
const char version_string[] = U_BOOT_VERSION_STRING;
#endif

#if defined(__aarch64__)
static struct mm_region cv84x6_mem_map[] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0x1000000000UL,
		.attrs = PTE_BLOCK_MEMTYPE(MT_DEVICE_NGNRNE) |
			 PTE_BLOCK_NON_SHARE |
			 PTE_BLOCK_PXN | PTE_BLOCK_UXN
	}, {
		.virt = PHYS_SDRAM_1,
		.phys = PHYS_SDRAM_1,
		.size = PHYS_SDRAM_1_SIZE,
		.attrs = PTE_BLOCK_MEMTYPE(MT_NORMAL) |
			 PTE_BLOCK_INNER_SHARE
	}, {
		/* List terminator */
		0,
	}
};

struct mm_region *mem_map = cv84x6_mem_map;
#endif

void pinmux_config(int io_type)
{
	switch (io_type) {
	case PINMUX_SDIO0:
		PINMUX_CONFIG(SD0_CD_X, SD0_CD_X, G3);
		PINMUX_CONFIG(SD0_PWR_EN, SD0_PWR_EN, G3);
		PINMUX_CONFIG(IIC3_SCL, GPIO66, G3);
		break;
	case PINMUX_SPI_NAND:
		/*
		 * SPI NAND shares the I2S0 pin group (G4). ROM only inits the
		 * boot device, so on eMMC-boot boards these pins are left in
		 * their default function and the NFC gets no response (readid
		 * TRX_DONE timeout). Mirror the ROM cv84x2 mapping here.
		 * NOTE: verify against the board schematic before use.
		 */
		PINMUX_CONFIG(I2S0_MCLK, SPINAND_SCK, G4);
		PINMUX_CONFIG(I2S0_WSI, SPINAND_SDI, G4);
		PINMUX_CONFIG(I2S0_SCLK, SPINAND_CS_X, G4);
		PINMUX_CONFIG(I2S0_SDI0, SPINAND_SDO, G4);
		PINMUX_CONFIG(I2S0_SDO, SPINAND_HOLD_X, G4);
		PINMUX_CONFIG(I2S0_SDI1, SPINAND_WP_X, G4);
		/* pad pull: CS/MOSI/MISO/WP/HOLD pull-up, SCK pull-down */
		PIN_PULL_CONFIG(I2S0_SCLK, G4, PIN_PULL_UP);
		PIN_PULL_CONFIG(I2S0_MCLK, G4, PIN_PULL_DOWN);
		PIN_PULL_CONFIG(I2S0_WSI, G4, PIN_PULL_UP);
		PIN_PULL_CONFIG(I2S0_SDI0, G4, PIN_PULL_UP);
		PIN_PULL_CONFIG(I2S0_SDI1, G4, PIN_PULL_UP);
		PIN_PULL_CONFIG(I2S0_SDO, G4, PIN_PULL_UP);
		break;
	default:
	case PINMUX_UART3_2:
		PINMUX_CONFIG(PWR_UART_TX, PWR_UART_TX, G7);
		PINMUX_CONFIG(PWR_UART_RX, PWR_UART_RX, G7);
		break;
	}
}

#include "../cvi_board_init.c"

void cpu_pwr_ctrl(void)
{
#if defined(CONFIG_RISCV)
	mmio_write_32(0x01901008, 0x30001);// cortexa53_pwr_iso_en
#elif defined(CONFIG_ARM)
	//mmio_write_32(0x01901004, 0x30001);// c906_top_pwr_iso_en   (pld mark)
#endif
}

void bm_storage_boot_loader_version_uboot(void)
{
	int size = 0;

	size = strlen(version_string);
	for (int i = 0; i < size; i++)
		mmio_write_8(UBOOT_VERSION_BASE + i, version_string[i]);

	mmio_write_8(UBOOT_VERSION_BASE + size, '\0');
}

#ifdef CONFIG_VIDEO_SOPH
void board_show_logo(void)
{
	struct udevice *dev;

	uclass_get_device_by_driver(UCLASS_VIDEO,
			DM_DRIVER_GET(soph_display), &dev);
	soph_show_logo();
}
#endif

void edge_hdcp_load_key(void)
{
	/*Load hdmi hdcp key*/
	#if defined(CONFIG_NVME_BOOT)
	run_command("pci e", 0);
	run_command("nvme scan", 0);
	run_command("load nvme 0:1 0x1002f80000 hdcp_key.bin", 0);
	#elif defined(CONFIG_SATA_BOOT)
	run_command("scsi scan", 0);
	run_command("load scsi 0:1 0x1002f80000 hdcp_key.bin", 0);
	#else
	run_command("load mmc 0:1 0x1002f80000 hdcp_key.bin", 0);
	#endif
}

void device_hdcp_load_key(void)
{
	run_command("load mmc 0:6 0x1002f80000 hdcp_key.bin", 0);
}

/*sm9v1 pinmux init*/
//TODO: update new pinlist
void sm9v1_board_init(void)
{
#if 0
	PINMUX_CONFIG(CAM_MCLK0, CAM_MCLK0, G9);
	PINMUX_CONFIG(CAM_MCLK1, CAM_MCLK1, G9);
	PINMUX_CONFIG(CAM_MCLK2, CAM_MCLK2, G9);
	PINMUX_CONFIG(CAM_MCLK3, CAM_MCLK3, G9);
	PINMUX_CONFIG(CAM_MCLK4, CAM_MCLK4, G9);
	PINMUX_CONFIG(CAM_MCLK5, CAM_MCLK5, G9);
	PINMUX_CONFIG(IIC0_SCL, IIC0_SCL, G12);
	PINMUX_CONFIG(IIC0_SDA, IIC0_SDA, G12);
	PINMUX_CONFIG(IIC1_SCL, IIC1_SCL, G12);
	PINMUX_CONFIG(IIC1_SDA, IIC1_SDA, G12);
	PINMUX_CONFIG(IIC2_SCL, IIC2_SCL, G12);
	PINMUX_CONFIG(IIC2_SDA, IIC2_SDA, G12);
	//FOR WS BD
	PINMUX_CONFIG(IIC4_SCL, SPI3_SDI, G12);
	PINMUX_CONFIG(IIC4_SDA, SPI3_CS_X, G12);
	PINMUX_CONFIG(IIC5_SCL, SPI3_SCK, G12);
	PINMUX_CONFIG(IIC5_SDA, SPI3_SDO, G12);

	PINMUX_CONFIG(UART2_RX, IIC6_SCL, G12);
	PINMUX_CONFIG(UART2_TX, IIC6_SDA, G12);
	PINMUX_CONFIG(CAM_XLR0, GPIO69, G9);
	PINMUX_CONFIG(CAM_XLR1, GPIO70, G9);
	PINMUX_CONFIG(GPIO4, GPIO115, G12);
	PINMUX_CONFIG(GPIO5, GPIO116, G12);
	PINMUX_CONFIG(UART4_RTS, GPIO93, G12);
	PINMUX_CONFIG(UART4_CTS, GPIO94, G12);
	PINMUX_CONFIG(UART4_RX, UART4_RX, G12);
	PINMUX_CONFIG(UART4_TX, UART4_TX, G12);
	PINMUX_CONFIG(UART1_CTS, GPIO86, G11);
	mmio_write_32(0x27012004, mmio_read_32(0x27012004) | 0x400000);
	mmio_write_32(0x27012000, mmio_read_32(0x27012000) | 0x400000);
	PINMUX_CONFIG(UART1_RTS, GPIO85, G11);
	mmio_write_32(0x27012004, mmio_read_32(0x27012004) | 0x200000);
	mmio_write_32(0x27012000, mmio_read_32(0x27012000) | 0x200000);
	//Power on SM9 VCC_3V3_SYS to make sure phy led active
	mmio_write_32(0x5021004, mmio_read_32(0x5021004) | 0x800);
	mmio_write_32(0x5021000, mmio_read_32(0x5021000) | 0x800);
	//mipi dsi
	//PINMUX_CONFIG(PAD_MIPI0_TX0P, PAD_MIPI0_TX0P, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX0N, PAD_MIPI0_TX0N, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX1P, PAD_MIPI0_TX1P, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX1N, PAD_MIPI0_TX1N, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX2P, PAD_MIPI0_TX2P, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX2N, PAD_MIPI0_TX2N, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX3P, PAD_MIPI0_TX3P, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX3N, PAD_MIPI0_TX3N, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX4P, PAD_MIPI0_TX4P, PHY);
	//PINMUX_CONFIG(PAD_MIPI0_TX4N, PAD_MIPI0_TX4N, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX0P, PAD_MIPI1_TX0P, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX0N, PAD_MIPI1_TX0N, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX1P, PAD_MIPI1_TX1P, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX1N, PAD_MIPI1_TX1N, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX2P, PAD_MIPI1_TX2P, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX2N, PAD_MIPI1_TX2N, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX3P, PAD_MIPI1_TX3P, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX3N, PAD_MIPI1_TX3N, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX4P, PAD_MIPI1_TX4P, PHY);
	//PINMUX_CONFIG(PAD_MIPI1_TX4N, PAD_MIPI1_TX4N, PHY);
	PINMUX_CONFIG(PWR_UART_TX, PWR_UART_TX, G7);
	PINMUX_CONFIG(PWR_UART_RX, PWR_UART_RX, G7);
	//PINMUX_CONFIG(UART1_TX, UART1_TX, G11);
	//PINMUX_CONFIG(UART1_RX, UART1_RX, G11);
	PINMUX_CONFIG(GPIO1, GPIO112, G12);
	PINMUX_CONFIG(GPIO0, GPIO111, G12);
	PINMUX_CONFIG(GPIO3, GPIO114, G12);
	//LED GPIO
	PINMUX_CONFIG(PWR_GPIO1, PWR_GPIO1, G7);
	//VCC12V-DCIN VCC_SYS_5V VCC_SYS_3V3
	PINMUX_CONFIG(PWR_SEQ1, PWR_GPIO11, G7);//VCC12V-DCIN
	PINMUX_CONFIG(GPIO1, GPIO112, G12);//VCC_SYS_5V
	PINMUX_CONFIG(IIC2_SDA, GPIO99, G12);//VCC_SYS_3V3
	//USB HOST
	PINMUX_CONFIG(CAM_MCLK5, GPIO51, G9);//VCC HUB
	PINMUX_CONFIG(PWR_GPIO5, PWR_GPIO5, G7);//VCC USB CON
	//I2C5
	PINMUX_CONFIG(SD1_D0, IIC5_SDA, G11);
	PINMUX_CONFIG(SD1_D1, IIC5_SCL, G11);
	//I2C6
	PINMUX_CONFIG(SD1_D2, IIC6_SDA, G11);
	PINMUX_CONFIG(SD1_D3, IIC6_SCL, G11);
	//PCEI-NVME POWER EN
	PINMUX_CONFIG(CAM_MCLK4, GPIO50, G9);
	//UART1,UART2,UART4,UART5,UART6
	PINMUX_CONFIG(PAD_MIPI1_TX0P, UART1_TX, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX0N, UART1_RX, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX1P, UART1_RTS, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX1N, UART1_CTS, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX2P, UART2_TX, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX2N, UART2_RX, PHY);
	//绿色旧板使用这组UART4
	PINMUX_CONFIG(FAN0, GPIO79, G9);
	PINMUX_CONFIG(FAN1, GPIO80, G9);
	//PINMUX_CONFIG(FAN0, UART4_TX, G9);
	//PINMUX_CONFIG(FAN1, UART4_RX, G9);
	PINMUX_CONFIG(PWM0, UART5_TX, G9);
	PINMUX_CONFIG(PWM1, UART5_RX, G9);
	PINMUX_CONFIG(PWM2, UART6_TX, G9);
	PINMUX_CONFIG(PWM3, UART6_RX, G9);
	PINMUX_CONFIG(SD1_CD_X, GPIO61, G12);
	//PINMUX_CONFIG(SD0_PWR_EN, GPIO60, G9);
	mmio_write_32(0x6700b0c0, 0x3fffffff);
	mmio_write_32(0x6700b160, 0x0);
	//MCU I2C GPIO
	PINMUX_CONFIG(PAD_VIVO0_D13, GPIO131, G5);
	PINMUX_CONFIG(PAD_VIVO0_D14, GPIO132, G5);
	//VCC FOR 4G
	PINMUX_CONFIG(PWR_GPIO4, PWR_GPIO4, G7);
	//For GPIO
	PINMUX_CONFIG(SD1_PWR_EN, GPIO68, G12);
	PINMUX_CONFIG(PAD_MIPI1_TX3P, GPIO124, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX3N, GPIO125, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX4P, IIC3_SDA, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX4N, IIC3_SCL, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX0P, GPIO174, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX0N, GPIO175, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX1P, GPIO176, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX1N, GPIO177, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX2P, GPIO178, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX2N, GPIO179, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX3P, GPIO180, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX3N, GPIO181, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX4P, GPIO182, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX4N, GPIO183, PHY);
	//SHUTDOWN IO
	PINMUX_CONFIG(PWR_GPIO0, PWR_GPIO0, G7);
	//CAM I2C
	PINMUX_CONFIG(PAD_VIVO0_D15, IIC7_SDA, G5);
	PINMUX_CONFIG(PAD_VIVO0_D16, IIC7_SCL, G5);
	//FAN PWM
	PINMUX_CONFIG(GPIO3, PWM11, G12);
	//RECOVERY KEY
	PINMUX_CONFIG(PWR_SEQ2, PWR_GPIO12, G7);
	//GPIO
	PINMUX_CONFIG(I2S0_SCLK, GPIO0, G6);
	PINMUX_CONFIG(I2S0_WSI, GPIO1, G6);
	PINMUX_CONFIG(I2S0_SDI0, GPIO2, G6);
	PINMUX_CONFIG(I2S0_SDI1, GPIO3, G6);
	PINMUX_CONFIG(I2S0_SDO, GPIO4, G6);
	PINMUX_CONFIG(I2S0_MCLK, GPIO5, G6);
	PINMUX_CONFIG(PWR_GPIO2, PWR_GPIO2, G7);
	PINMUX_CONFIG(PWR_GPIO3, PWR_GPIO3, G7);
	//PCIE RST
	PINMUX_CONFIG(PAD_VIVO0_D11, GPIO129, G5);
	PINMUX_CONFIG(PAD_VIVO0_D12, GPIO130, G5);
	//BD UART3
	PINMUX_CONFIG(IIC2_SDA, UART3_TX, G12);
	PINMUX_CONFIG(IIC2_SCL, UART3_RX, G12);
	//GPIO
	PINMUX_CONFIG(UART1_TX, GPIO83, G11);
	PINMUX_CONFIG(UART1_RX, GPIO84, G11);
#endif
}

/*sm9v2 pinmux init*/
void sm9v2_board_init(void)
{
#if 0
	PINMUX_CONFIG(CAM_MCLK0, CAM_MCLK0, G9);
	PINMUX_CONFIG(CAM_MCLK1, CAM_MCLK1, G9);
	PINMUX_CONFIG(CAM_MCLK2, CAM_MCLK2, G9);
	PINMUX_CONFIG(CAM_MCLK3, CAM_MCLK3, G9);
	PINMUX_CONFIG(CAM_MCLK4, CAM_MCLK4, G9);
	PINMUX_CONFIG(CAM_MCLK5, CAM_MCLK5, G9);
	PINMUX_CONFIG(IIC0_SCL, IIC0_SCL, G12);
	PINMUX_CONFIG(IIC0_SDA, IIC0_SDA, G12);
	PINMUX_CONFIG(IIC1_SCL, IIC1_SCL, G12);
	PINMUX_CONFIG(IIC1_SDA, IIC1_SDA, G12);
	PINMUX_CONFIG(IIC2_SCL, IIC2_SCL, G12);
	PINMUX_CONFIG(IIC2_SDA, IIC2_SDA, G12);
	PINMUX_CONFIG(IIC4_SCL, IIC4_SCL, G12);
	PINMUX_CONFIG(IIC4_SDA, IIC4_SDA, G12);
	PINMUX_CONFIG(UART2_RX, UART2_RX, G12);
	PINMUX_CONFIG(UART2_TX, UART2_TX, G12);
	PINMUX_CONFIG(UART2_RTS, UART2_RTS, G12);
	PINMUX_CONFIG(UART2_CTS, UART2_CTS, G12);
	PINMUX_CONFIG(CAM_XLR0, GPIO69, G9);
	PINMUX_CONFIG(CAM_XLR1, GPIO70, G9);
	PINMUX_CONFIG(GPIO4, GPIO115, G12);
	PINMUX_CONFIG(GPIO5, GPIO116, G12);
	PINMUX_CONFIG(UART4_RTS, GPIO93, G12);
	PINMUX_CONFIG(UART4_CTS, GPIO94, G12);
	PINMUX_CONFIG(UART4_RX, UART4_RX, G12);
	PINMUX_CONFIG(UART4_TX, UART4_TX, G12);

	PINMUX_CONFIG(UART1_CTS, UART5_RX, G11);
	mmio_write_32(0x27012004, mmio_read_32(0x27012004) | 0x400000);
	mmio_write_32(0x27012000, mmio_read_32(0x27012000) | 0x400000);
	PINMUX_CONFIG(UART1_RTS, UART5_TX, G11);
	mmio_write_32(0x27012004, mmio_read_32(0x27012004) | 0x200000);
	mmio_write_32(0x27012000, mmio_read_32(0x27012000) | 0x200000);

	//mipi dsi
	PINMUX_CONFIG(PAD_MIPI0_TX0P, PAD_MIPI0_TX0P, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX0N, PAD_MIPI0_TX0N, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX1P, PAD_MIPI0_TX1P, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX1N, PAD_MIPI0_TX1N, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX2P, PAD_MIPI0_TX2P, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX2N, PAD_MIPI0_TX2N, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX3P, PAD_MIPI0_TX3P, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX3N, PAD_MIPI0_TX3N, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX4P, PAD_MIPI0_TX4P, PHY);
	PINMUX_CONFIG(PAD_MIPI0_TX4N, PAD_MIPI0_TX4N, PHY);

	PINMUX_CONFIG(PAD_MIPI1_TX0P, PAD_MIPI1_TX0P, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX0N, PAD_MIPI1_TX0N, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX1P, PAD_MIPI1_TX1P, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX1N, PAD_MIPI1_TX1N, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX2P, PAD_MIPI1_TX2P, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX2N, PAD_MIPI1_TX2N, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX3P, PAD_MIPI1_TX3P, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX3N, PAD_MIPI1_TX3N, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX4P, PAD_MIPI1_TX4P, PHY);
	PINMUX_CONFIG(PAD_MIPI1_TX4N, PAD_MIPI1_TX4N, PHY);

	PINMUX_CONFIG(PWR_UART_TX, PWR_GPIO16, G7);
	PINMUX_CONFIG(PWR_UART_RX, PWR_GPIO17, G7);
	PINMUX_CONFIG(UART1_TX, UART1_TX, G11);
	PINMUX_CONFIG(UART1_RX, UART1_RX, G11);
	PINMUX_CONFIG(GPIO1, GPIO112, G12);
	PINMUX_CONFIG(GPIO0, GPIO111, G12);
	PINMUX_CONFIG(GPIO3, GPIO114, G12);
	PINMUX_CONFIG(GPIO2, GPIO113, G12);
	PINMUX_CONFIG(IIC5_SCL, GPIO104, G12);
	PINMUX_CONFIG(IIC5_SDA, GPIO103, G12);

	PINMUX_CONFIG(PAD_VIVO0_D15, UART7_TX, G5);
	PINMUX_CONFIG(PAD_VIVO0_D16, UART7_RX, G5);

	//spi
	PINMUX_CONFIG(PAD_VIVO0_D14, SPI0_SCK, G5);
	PINMUX_CONFIG(PAD_VIVO0_D13, SPI0_SDO, G5);
	PINMUX_CONFIG(PAD_VIVO0_D12, SPI0_SDI, G5);
	PINMUX_CONFIG(PAD_VIVO0_D11, SPI0_CS_X, G5);

	PINMUX_CONFIG(I2S0_SDI1, GPIO3, G6);
	PINMUX_CONFIG(CAM_MCLK2, GPIO48, G9);

	//GPIO
	PINMUX_CONFIG(CAM_MCLK5, GPIO51, G9);
	PINMUX_CONFIG(SD0_PWR_EN, GPIO60, G9);
	PINMUX_CONFIG(SD1_PWR_EN, GPIO68, G12);
	PINMUX_CONFIG(SD1_CD_X, GPIO61, G12);
	PINMUX_CONFIG(CLK_25M_OUT, GPIO119, G7);
	PINMUX_CONFIG(PCIE0_L0_CLKREQ_IN_X, GPIO42, G5);
	PINMUX_CONFIG(PWR_GPIO0, PWR_GPIO0, G7);
#endif
}

/*set pinmux by prd*/
void set_product_pinmux(void)
{
	char dtstype[DTSNAME_MAX_LEN] = { 0 };
	get_dts_type_from_sram(dtstype);
	if (strstr(dtstype, "sm9v1"))
		sm9v1_board_init();
	else if (strstr(dtstype, "se9b1"))
		sm9v1_board_init();
	else if (strstr(dtstype, "se9b2"))
		sm9v1_board_init();
	else if (strstr(dtstype, "se9b3"))
		sm9v1_board_init();
	else if (strstr(dtstype, "se9b4"))
		sm9v1_board_init();
	else if (strstr(dtstype, "sm9v2"))
		sm9v2_board_init();
}

int board_init(void)
{
	printf("start board_init\n");

	/* Enable MAC layer RX/TX delay: bit16/18 for eth0, bit17/19 for eth1 */
	mmio_write_32(TOP_BASE + 0x8, mmio_read_32(TOP_BASE + 0x8) | 0xF0000);

	pinmux_config(PINMUX_SDIO0);
	pinmux_config(PINMUX_UART3_2);
#ifdef CONFIG_NAND_SUPPORT
	/* Mux the SPI NAND pins before the NFC issues its first command */
	pinmux_config(PINMUX_SPI_NAND);
#endif
	// usb vbus
	PINMUX_CONFIG(USB3_VBUS_EN0, GPIO69, G3);
	PINMUX_CONFIG(USB3_VBUS_EN1, GPIO70, G3);

	cvi_board_init();
	bm_storage_boot_loader_version_uboot();
	return 0;
}

#define DTSNAME_MAX_LEN 32
#define DEFAULT_DTSNAME "config-" __stringify(CVICHIP) "_" __stringify(CVIBOARD)

/* Buffer in DDR for reading eMMC boot1 data (below scriptaddr, avoids overwrite) */
#define BOOT1_BUF_ADDR  0x1000600000
#define BOOT1_MAC_OFFSET  0x40
#define ETHER_NUM 2
#define MAC_SIZE 6

static int boot1_loaded;

static __maybe_unused void load_boot1(void)
{
	if (boot1_loaded)
		return;

	run_command("mmc dev 0 2\0", 0);
	run_command("mmc read " __stringify(BOOT1_BUF_ADDR) " 0 0x60", 0);
	boot1_loaded = 1;
}

static __maybe_unused void load_boot1_force(void)
{
	boot1_loaded = 0;
	load_boot1();
}

void get_dts_type_from_oem(unsigned char *dtsname)
{
#ifdef CONFIG_EMMC_SUPPORT
	if (!dtsname) {
		printf("get dts type from oem failed!\n");
		return;
	}
	load_boot1();
	memcpy(dtsname, (void *)(BOOT1_BUF_ADDR + 0xa0), DTSNAME_MAX_LEN);
	dtsname[DTSNAME_MAX_LEN - 1] = '\0';
#endif
	// use default dts
	if (strlen(dtsname) == 0)
		memcpy(dtsname, DEFAULT_DTSNAME, sizeof(DEFAULT_DTSNAME));
	printf("OEM INFO: DTS_TYPE[%s]\n", dtsname);

}

/*get dts from sram*/
void get_dts_type_from_sram(unsigned char *dtsname)
{
#ifdef CONFIG_EMMC_SUPPORT
	if (!dtsname) {
		return;
	}
	memcpy(dtsname, (void *)DTSTYPE_OEM_INFO, DTSNAME_MAX_LEN);
	dtsname[DTSNAME_MAX_LEN - 1] = '\0';
#endif
	/*use default dts*/
	if (strlen(dtsname) == 0)
		memcpy(dtsname, DEFAULT_DTSNAME, sizeof(DEFAULT_DTSNAME));
}

u64 get_ddr_size_from_sram(void)
{
	//u64 ddr_size;
	return PHYS_SDRAM_1_SIZE;
/*
	ddr_size = mmio_read_32(DDR_SIZE_OEM_INFO);
	switch (ddr_size) {
	case 2:
	case 4:
	case 8:
	case 12:
	case 16:
		break;

	default:
		return PHYS_SDRAM_1_SIZE;
	}
	return ddr_size * 1024 * 1024 * 1024;
*/
}

/*set default console by oem*/
struct serial_device *default_serial_console(void)
{
	/*select console uart*/
#ifdef CONFIG_SYS_NS16550_SERIAL
#ifdef CONFIG_SYS_NS16550_COM3
	if (mmio_read_8(CONSOLE_OEM_INFO) == CONSOLE_USE_UART2){
		return &eserial3_device;//uart2
	}
#endif
	return &eserial1_device;//uart0
#endif
}

/*set consoledev by oem*/
void setup_sophgo_console(void)
{
	if (mmio_read_8(CONSOLE_OEM_INFO) == CONSOLE_USE_UART2)//uart2
		env_set("consoledev", "ttyS2");
	printf("set console 0x%x\n",mmio_read_8(CONSOLE_OEM_INFO));
}

int setup_sophgo_dts(void)
{
	char configName[DTSNAME_MAX_LEN * 2] = {0};
	char dtsType[DTSNAME_MAX_LEN] = {0};
	char *ptr = NULL;

#ifdef CONFIG_SD_BOOT
	//for SD/spinor boot, read dts_type file on /boot
	if (fat_exists("dts_type")) {
		file_fat_read("dts_type", dtsType, DTSNAME_MAX_LEN);
		dtsType[DTSNAME_MAX_LEN - 1] = '\0';
		printf("read dts_type: %s.\n", dtsType);
	} else {
		dtsType[0] = '\0';
	}

	// use default dts
	if (strlen(dtsType) == 0)
		memcpy(dtsType, DEFAULT_DTSNAME, sizeof(DEFAULT_DTSNAME));
	printf("OEM INFO: DTS_TYPE[%s]\n", dtsType);
#else
	get_dts_type_from_oem(dtsType);
#endif
	ptr = strstr(dtsType, "config-");
	//If the string does not start with "config-"
	if (!ptr && ptr != dtsType) {
		sprintf(configName, "config-%s", dtsType);
		env_set("DTS_TYPE", configName);
		return 0;
	}

	env_set("DTS_TYPE", dtsType);
	return 0;
}

#if defined(CONFIG_ROOTFS_UBUNTU) || defined(CONFIG_ROOTFS_DEBIAN)
#ifdef CONFIG_BOARD_LATE_INIT
static void get_ether_addr_from_emmc(unsigned char *mac, int i)
{
	#if defined(CONFIG_SD_BOOT)
	//todo: read from sdcard
	#elif defined(CONFIG_SPI_FLASH)
	//todo: read from spinor flash
	#else	//default eMMC
	load_boot1();
	memcpy(mac, (void *)(BOOT1_BUF_ADDR + BOOT1_MAC_OFFSET + i * 0x10), 6);
	#endif
	debug("OEM MAC eth%d: %pM\n", i, mac);
}
static int setup_mac(void)
{
	int i;
	uint8_t mac[ETHER_NUM][MAC_SIZE];
	char eth[16];
	int flag = 0;
	char buf[ARP_HLEN_ASCII + 1];
	char *env_eth = NULL;

	for (i = 0; i < ETHER_NUM; i++) {
		get_ether_addr_from_emmc(mac[i], i);
		if (i == 0)
			snprintf(eth, sizeof(eth), "ethaddr");
		else
			snprintf(eth, sizeof(eth), "eth%uaddr", i);

		if (is_zero_ethaddr(mac[i])) {
			debug("%s not specified, skip\n", eth);
			continue;
		}

		if (!is_valid_ethaddr(mac[i])) {
			printf("Invalid mac %pM\n", mac[i]);
			continue;
		}

		sprintf(buf, "%pM", mac[i]);
		env_eth = env_get(eth);
		if (!env_eth || strncasecmp(env_eth, buf, sizeof(buf))) {
			env_set(eth, buf);
			flag = 1;
		}
	}

	if (flag) {
		printf("MAC updated from OEM, saving env\n");
		run_command("saveenv", 0);
	}

	return 0;
}

int check_ubootenv_file_exists(void)
{
	const char *filename = "u-boot.env";

	return fat_exists(filename);
}

int board_late_init(void)
{
	console_record_reset_enable();
	setup_mac();
	setup_sophgo_dts();
	setup_sophgo_console();
	//edge_hdcp_load_key();
#ifdef CONFIG_VIDEO_SOPH
	board_show_logo();
#endif
	//check u-boot.env exist
	if (!check_ubootenv_file_exists()) {
		printf("save default env to /boot\n");
		run_command("saveenv", 0);	//save default env
	}

	return 0;
}
#endif

#ifdef CONFIG_OF_BOARD_SETUP
static void set_ethaddr_from_env_or_boot1(void *blob, const char *path, int idx)
{
	unsigned char mac[6];
	int nodeoff;

	nodeoff = fdt_path_offset(blob, path);
	if (nodeoff < 0)
		return;

	if (!eth_env_get_enetaddr_by_index("eth", idx, mac)) {
		load_boot1_force();
		memcpy(mac, (void *)(BOOT1_BUF_ADDR + BOOT1_MAC_OFFSET +
				     idx * 0x10), 6);
	}

	if (is_valid_ethaddr(mac) && !is_zero_ethaddr(mac)) {
		fdt_setprop(blob, nodeoff, "mac-address", mac, 6);
		fdt_setprop(blob, nodeoff, "local-mac-address", mac, 6);
		debug("FDT %s: mac-address %pM\n", path, mac);
	}
}

int ft_board_setup(void *blob, struct bd_info *bd)
{
	set_ethaddr_from_env_or_boot1(blob, "/ethernet@290d0000", 0);
	set_ethaddr_from_env_or_boot1(blob, "/ethernet@290e0000", 1);

	cv84x6_pcie_fixup(blob);

	return 0;
}
#endif

#else
int board_late_init(void)
{
/*** add new code when cv84x6 is ready ***/
/*
	setup_sophgo_dts();
	setup_sophgo_console();
	device_hdcp_load_key();
#ifdef CONFIG_VIDEO_SOPH
	board_show_logo();
#endif
#endif
*/
	return 0;
}

#ifdef CONFIG_OF_BOARD_SETUP
int ft_board_setup(void *blob, struct bd_info *bd)
{
	cv84x6_pcie_fixup(blob);

	return 0;
}
#endif
#endif

#if defined(__aarch64__)
int dram_init(void)
{
	gd->ram_size = get_ddr_size_from_sram();
	mem_map[1].size = gd->ram_size;
	return 0;
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = get_ddr_size_from_sram();

	return 0;
}
#endif

#ifdef CV_SYS_OFF
static void cv_system_off(void)
{
	mmio_write_32(REG_RTC_BASE + RTC_EN_SHDN_REQ, 0x01);
	while (mmio_read_32(REG_RTC_BASE + RTC_EN_SHDN_REQ) != 0x01)
		;
	mmio_write_32(REG_RTC_CTRL_BASE + RTC_CTRL0_UNLOCKKEY, 0xAB18);
	mmio_setbits_32(REG_RTC_CTRL_BASE + RTC_CTRL0, 0xFFFF0800 | (0x1 << 0));

	while (1)
		;
}
#endif

void cv_system_reset(void)
{
	mmio_write_32(REG_RTC_BASE + RTC_EN_PWR_CYC_REQ, 0x01);
	while (mmio_read_32(REG_RTC_BASE + RTC_EN_PWR_CYC_REQ) != 0x01)
		;
	mmio_write_32(REG_RTC_CTRL_BASE + RTC_CTRL0_UNLOCKKEY, 0xAB18);
	mmio_setbits_32(REG_RTC_CTRL_BASE + RTC_CTRL0, 0xFFFF0800 | (0x1 << 3));

	while (1)
		;
}

/*
 * Board specific reset that is system reset.
 */
void reset_cpu(void)
{
	cv_system_reset();
}

#ifdef CONFIG_USB_GADGET_DWC2_OTG
struct dwc2_plat_otg_data cv182x_otg_data = {
	.regs_otg = USB_BASE,
	.usb_gusbcfg	= 0x40081400,
	.rx_fifo_sz	 = 512,
	.np_tx_fifo_sz  = 512,
	.tx_fifo_sz	 = 512,
};

int board_usb_init(int index, enum usb_init_type init)
{
	uint32_t value;

	value = mmio_read_32(TOP_BASE + REG_TOP_SOFT_RST) & (~BIT_TOP_SOFT_RST_USB);
	mmio_write_32(TOP_BASE + REG_TOP_SOFT_RST, value);
	udelay(50);
	value = mmio_read_32(TOP_BASE + REG_TOP_SOFT_RST) | BIT_TOP_SOFT_RST_USB;
	mmio_write_32(TOP_BASE + REG_TOP_SOFT_RST, value);

	/* Set USB phy configuration */
	value = mmio_read_32(REG_TOP_USB_PHY_CTRL);
	mmio_write_32(REG_TOP_USB_PHY_CTRL, value | BIT_TOP_USB_PHY_CTRL_EXTVBUS
					| USB_PHY_ID_OVERRIDE_ENABLE
					| USB_PHY_ID_VALUE);

	/* Enable ECO RXF */
	mmio_write_32(REG_TOP_USB_ECO, mmio_read_32(REG_TOP_USB_ECO) | BIT_TOP_USB_ECO_RX_FLUSH);

	printf("cvi_usb_hw_init done\n");

	return dwc2_udc_probe(&cv182x_otg_data);
}
#endif

void board_save_time_record(uintptr_t saveaddr)
{
	uint64_t boot_us = 0;
#if defined(__aarch64__)
	boot_us = timer_get_boot_us();
#elif defined(__riscv)
	// Read from CSR_TIME directly. RISC-V timers is initialized later.
	boot_us = csr_read(CSR_TIME) / (SYS_COUNTER_FREQ_IN_SECOND / 1000000);
#else
#error "Unknown ARCH"
#endif

	mmio_write_16(saveaddr, DIV_ROUND_UP(boot_us, 1000));
}

#if defined(CONFIG_MULTI_DTB_FIT)
int board_fit_config_name_match(const char *name)
{
	char fit_name[DTSNAME_MAX_LEN] = {0};
	char dtstype[DTSNAME_MAX_LEN] = {0};

	get_dts_type_from_sram(dtstype);
	memcpy(fit_name, name + 7, strlen(name) - 7 -2);//only match product name eg:_sm9v1_
	if (strstr((char *)dtstype, fit_name)) {
		return 0;
	}
	return -1;
}
#endif

struct image_header *spl_get_load_buffer(ssize_t offset, size_t size)
{
	return (struct image_header *)CVIMMAP_UIMAG_ADDR;
}
