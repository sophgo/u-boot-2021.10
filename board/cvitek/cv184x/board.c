/*
 * SPDX-License-Identifier:	GPL-2.0+
 *
 * (C) Copyright 2013
 * David Feng <fenghua@phytium.com.cn>
 * Sharma Bhupesh <bhupesh.sharma@freescale.com>
 */
#include <common.h>
#include <command.h>
#include <dm.h>
#include <malloc.h>
#include <errno.h>
#include <asm/io.h>
#include <linux/compiler.h>
#if defined(__aarch64__)
#include <asm/armv8/mmu.h>
#endif
#include <usb/dwc2_udc.h>
#include <usb.h>
#include "cv184x_reg.h"
#include "mmio.h"
#include "cv184x_reg_fmux_gpio.h"
#include "cv184x_pinlist_swconfig.h"
#include <linux/delay.h>
#include <bootstage.h>
#include <cvitek/cvi_efuse.h>
// #include <configs/cv184x-asic.h>

#if defined(__riscv)
#include <asm/csr.h>
#endif

DECLARE_GLOBAL_DATA_PTR;
#define SD1_SDIO_PAD

#if defined(__aarch64__)
static struct mm_region cv184x_mem_map[] = {
	{
		.virt = 0x0UL,
		.phys = 0x0UL,
		.size = 0x80000000UL,
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

struct mm_region *mem_map = cv184x_mem_map;
#endif

// #define PINMUX_CONFIG(PIN_NAME, FUNC_NAME) printf ("%s\n", PIN_NAME ##_ ##FUNC_NAME);
#define PINMUX_CONFIG(PIN_NAME, FUNC_NAME) \
		mmio_clrsetbits_32(PINMUX_BASE + FMUX_GPIO_FUNCSEL_##PIN_NAME, \
			FMUX_GPIO_FUNCSEL_##PIN_NAME##_MASK << FMUX_GPIO_FUNCSEL_##PIN_NAME##_OFFSET, \
			PIN_NAME##__##FUNC_NAME)

void pinmux_config(int io_type)
{
		switch (io_type) {
		case PINMUX_UART0:
			PINMUX_CONFIG(UART0_RX, UART0_RX);
			PINMUX_CONFIG(UART0_TX, UART0_TX);
		break;
		case PINMUX_SDIO0:
			PINMUX_CONFIG(SD0_CD, SDIO0_CD);
			PINMUX_CONFIG(SD0_PWR_EN, SDIO0_PWR_EN);
			PINMUX_CONFIG(SD0_CMD, SDIO0_CMD);
			PINMUX_CONFIG(SD0_CLK, SDIO0_CLK);
			PINMUX_CONFIG(SD0_D0, SDIO0_D_0);
			PINMUX_CONFIG(SD0_D1, SDIO0_D_1);
			PINMUX_CONFIG(SD0_D2, SDIO0_D_2);
			PINMUX_CONFIG(SD0_D3, SDIO0_D_3);
			break;
		case PINMUX_SDIO1:
#if defined(SD1_SDIO_PAD)
			/*
			 * Name            Address            SD1  MIPI
			 * reg_sd1_phy_sel REG_0x300_0294[10] 0x0  0x1
			 */
			mmio_write_32(TOP_BASE + 0x294,
				      (mmio_read_32(TOP_BASE + 0x294) & 0xFFFFFBFF));
			PINMUX_CONFIG(SD1_CMD, PWR_SD1_CMD_VO36);
			PINMUX_CONFIG(SD1_CLK, PWR_SD1_CLK_VO37);
			PINMUX_CONFIG(SD1_D0, PWR_SD1_D0_VO35);
			PINMUX_CONFIG(SD1_D1, PWR_SD1_D1_VO34);
			PINMUX_CONFIG(SD1_D2, PWR_SD1_D2_VO33);
			PINMUX_CONFIG(SD1_D3, PWR_SD1_D3_VO32);
#elif defined(SD1_MIPI_PAD)
			/*
			 * Name            Address            SD1  MIPI
			 * reg_sd1_phy_sel REG_0x300_0294[10] 0x0  0x1
			 */
			mmio_write_32(TOP_BASE + 0x294,
				      (mmio_read_32(TOP_BASE + 0x294) & 0xFFFFFBFF) | BIT(10));
			PINMUX_CONFIG(PAD_MIPI_TXM4, SD1_CLK);
			PINMUX_CONFIG(PAD_MIPI_TXP4, SD1_CMD);
			PINMUX_CONFIG(PAD_MIPI_TXM3, SD1_D0);
			PINMUX_CONFIG(PAD_MIPI_TXP3, SD1_D1);
			PINMUX_CONFIG(PAD_MIPI_TXM2, SD1_D2);
			PINMUX_CONFIG(PAD_MIPI_TXP2, SD1_D3);
#endif
			break;
		case PINMUX_EMMC:
			PINMUX_CONFIG(EMMC_CLK, EMMC_CLK);
			PINMUX_CONFIG(EMMC_RSTN, EMMC_RSTN);
			PINMUX_CONFIG(EMMC_CMD, EMMC_CMD);
			PINMUX_CONFIG(EMMC_DAT1, EMMC_DAT_1);
			PINMUX_CONFIG(EMMC_DAT0, EMMC_DAT_0);
			PINMUX_CONFIG(EMMC_DAT2, EMMC_DAT_2);
			PINMUX_CONFIG(EMMC_DAT3, EMMC_DAT_3);
			break;
		case PINMUX_SPI_NAND:
			PINMUX_CONFIG(EMMC_DAT2, SPINAND_HOLD);
			PINMUX_CONFIG(EMMC_CLK, SPINAND_CLK);
			PINMUX_CONFIG(EMMC_DAT0, SPINAND_MOSI);
			PINMUX_CONFIG(EMMC_DAT3, SPINAND_WP);
			PINMUX_CONFIG(EMMC_CMD, SPINAND_MISO);
			PINMUX_CONFIG(EMMC_DAT1, SPINAND_CS);
		break;
		case PINMUX_DSI:
			PINMUX_CONFIG(PAD_MIPI_TXM0, XGPIOC_12);
			PINMUX_CONFIG(PAD_MIPI_TXP0, XGPIOC_13);
			PINMUX_CONFIG(PAD_MIPI_TXM1, XGPIOC_14);
			PINMUX_CONFIG(PAD_MIPI_TXP1, XGPIOC_15);
			PINMUX_CONFIG(PAD_MIPI_TXM2, XGPIOC_16);
			PINMUX_CONFIG(PAD_MIPI_TXP2, XGPIOC_17);
			PINMUX_CONFIG(PAD_MIPI_TXM3, XGPIOC_20);
			PINMUX_CONFIG(PAD_MIPI_TXP3, XGPIOC_21);
			PINMUX_CONFIG(PAD_MIPI_TXM4, XGPIOC_18);
			PINMUX_CONFIG(PAD_MIPI_TXP4, XGPIOC_19);
		break;
		case PINMUX_LVDS:
			PINMUX_CONFIG(PAD_MIPI_TXM0, XGPIOC_12);
			PINMUX_CONFIG(PAD_MIPI_TXP0, XGPIOC_13);
			PINMUX_CONFIG(PAD_MIPI_TXM1, XGPIOC_14);
			PINMUX_CONFIG(PAD_MIPI_TXP1, XGPIOC_15);
			PINMUX_CONFIG(PAD_MIPI_TXM2, XGPIOC_16);
			PINMUX_CONFIG(PAD_MIPI_TXP2, XGPIOC_17);
			PINMUX_CONFIG(PAD_MIPI_TXM3, XGPIOC_20);
			PINMUX_CONFIG(PAD_MIPI_TXP3, XGPIOC_21);
			PINMUX_CONFIG(PAD_MIPI_TXM4, XGPIOC_18);
			PINMUX_CONFIG(PAD_MIPI_TXP4, XGPIOC_19);
		break;
		default:
			break;
	}
}

#include "../cvi_board_init.c"

#if defined(CONFIG_PHY_CVITEK) /* config cvitek cv184x eth internal phy on ASIC board */
static void cv184x_ephy_id_init(void)
{
	// set rg_ephy_apb_rw_sel 0x0804@[0]=1/APB by using APB interface
	mmio_write_32(0x03009804, 0x0001);

	// Release 0x0800[0]=0/shutdown
	mmio_write_32(0x03009800, 0x0900);

	// Release 0x0800[2]=1/dig_rst_n, Let mii_reg can be accessabile
	mmio_write_32(0x03009800, 0x0904);

	// ANA INIT (PD/EN), switch to MII-page5
	mmio_write_32(0x0300907c, 0x0500);
	// Release ANA_PD p5.0x10@[13:8] = 6'b001100
	mmio_write_32(0x03009040, 0x0c00);
	// Release ANA_EN p5.0x10@[7:0] = 8'b01111110
	mmio_write_32(0x03009040, 0x0c7e);

	// Wait PLL_Lock, Lock_Status p5.0x12@[15] = 1
	//mdelay(1);

	// Release 0x0800[1] = 1/ana_rst_n
	mmio_write_32(0x03009800, 0x0906);

	// ANA INIT
	// @Switch to MII-page5
	mmio_write_32(0x0300907c, 0x0500);

	// PHY_ID
	mmio_write_32(0x03009008, 0x0043);
	mmio_write_32(0x0300900c, 0x5649);

	// switch to MDIO control by ETH_MAC
	mmio_write_32(0x03009804, 0x0000);
}

static void cv184x_ephy_led_pinmux(void)
{
	// LED PAD MUX
	mmio_write_32(0x030010e0, 0x05);
	mmio_write_32(0x030010e4, 0x05);
	//(SD1_CLK selphy)
	mmio_write_32(0x050270b0, 0x11111111);
	//(SD1_CMD selphy)
	mmio_write_32(0x050270b4, 0x11111111);
}
#endif

void cpu_pwr_ctrl(void)
{
#if defined(CONFIG_RISCV)
	mmio_write_32(0x01901008, 0x30001);// cortexa53_pwr_iso_en
#elif defined(CONFIG_ARM)
	mmio_write_32(0x01901004, 0x30001);// c906_top_pwr_iso_en
#endif
}

//rom patch for emmc trap failed issue.
void set_boottrap_to_emmc(void)
{
#if defined(CONFIG_EMMC_SUPPORT) && defined(CONFIG_SET_BOOTTRAP_EMMC)
	/* addr */
	if (run_command("efusew_word 0x84  0x441885c", 0))
		return;
	/* value */
	if (run_command("efusew_word 0x88  0x890de035", 0))
		return;
	/* efuse password */
	if (run_command("mw.l 0x03050024 0xe9546b8d", 0))
		return;
	/* secureRegionctrl enable rom patch*/
	if (run_command("efusew_word 0xF8  0x300000", 0))
		return;
#endif
}

int board_init(void)
{
/*
 * The default value of uart clk is 25M
 * If the UART CLK changes, you need to change the CLK source in DTS and cv184x-asic.h
 * eg:
 * cv184x-asic.h: #define CONFIG_SYS_NS16550_CLK		1188000000
 *
 * cv184x_base.dtsi: uart0 ~ uart4
 * uart0: serial@04140000 {
 *	compatible = "snps,dw-apb-uart";
 *	reg = <0x0 0x04140000 0x0 0x1000>;
 *	clock-frequency = <1188000000>;
 *	reg-shift = <2>;
 *	reg-io-width = <4>;
 *	status = "okay";
 *};
 */
#if CONFIG_SYS_NS16550_CLK == 396000000
	mmio_write_32(DIV_CLK_CAM0_200, BIT_DIV_RESET_CONT | BIT_SELT_DIV_REG | BIT_CLK_SRC |
		BIT_CLK_DIV_FACT_16 | BIT_CLK_DIV_FACT_17);
#elif CONFIG_SYS_NS16550_CLK == 594000000
	mmio_write_32(DIV_CLK_CAM0_200, BIT_DIV_RESET_CONT | BIT_SELT_DIV_REG | BIT_CLK_SRC |
		BIT_CLK_DIV_FACT_17);
#elif CONFIG_SYS_NS16550_CLK == 1188000000
	mmio_write_32(DIV_CLK_CAM0_200, BIT_DIV_RESET_CONT | BIT_SELT_DIV_REG | BIT_CLK_SRC |
		BIT_CLK_DIV_FACT_16);
#endif

#ifndef CONFIG_DUAL_OS
#ifndef CONFIG_TARGET_CVITEK_CV184X_FPGA
	extern volatile uint32_t BOOT0_START_TIME;
	uint16_t start_time = DIV_ROUND_UP(BOOT0_START_TIME, SYS_COUNTER_FREQ_IN_SECOND / 1000);

	// Save uboot start time. time is from boot0.h
	mmio_write_16(TIME_RECORDS_FIELD_UBOOT_START, start_time);
#endif
#else
	extern uint32_t BOOT0_START_TIME;
	uint16_t start_time = DIV_ROUND_UP(BOOT0_START_TIME, SYS_COUNTER_FREQ_IN_SECOND / 1000);

	// Save uboot start time. time is from boot0.h
	mmio_write_16(TIME_RECORDS_FIELD_UBOOT_START, start_time);
#endif

	cpu_pwr_ctrl();

#if defined(CONFIG_PHY_CVITEK) /* config cvitek cv184x eth internal phy on ASIC board */
	cv184x_ephy_id_init();
	cv184x_ephy_led_pinmux();
#endif

#if defined(CONFIG_NAND_SUPPORT)
	pinmux_config(PINMUX_SPI_NAND);
#elif defined(CONFIG_SPI_FLASH)
	pinmux_config(PINMUX_SPI_NOR);
	//set spi_nor_xip_en 0 to disable spi nor xip mode
	mmio_write_32(HSPERI_COMMON_REG, 0);
#elif defined(CONFIG_EMMC_SUPPORT)
	pinmux_config(PINMUX_EMMC);
#endif
#ifdef CONFIG_DISPLAY_CVITEK_MIPI
	pinmux_config(PINMUX_DSI);
#elif defined(CONFIG_DISPLAY_CVITEK_LVDS)
	pinmux_config(PINMUX_LVDS);
#endif
	pinmux_config(PINMUX_SDIO1);
	PINMUX_CONFIG(CAM_MCLK0, CAM_MCLK0);
	cvi_board_init();

#if defined(CONFIG_SPL_BUILD) && defined(CONFIG_SPL_EFUSE_ENABLE_FASTBOOT)
	CVI_EFUSE_EnableFastBoot();
#endif

	return 0;
}

#if defined(__aarch64__)
int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_1_SIZE;
	return 0;
}

int dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;

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
	mmio_write_32(REG_RTC_BASE + RTC_EN_WARM_RST_REQ, 0x01);
	while (mmio_read_32(REG_RTC_BASE + RTC_EN_WARM_RST_REQ) != 0x01)
		;
	mmio_write_32(REG_RTC_CTRL_BASE + RTC_CTRL0_UNLOCKKEY, 0xAB18);
	mmio_setbits_32(REG_RTC_CTRL_BASE + RTC_CTRL0, 0xFFFF0800 | (0x1 << 4));

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

/*
 * Gate USB clocks from clkgen (see linux .../clk/cvitek/clk-cv184x.c).
 * U-Boot has no cvitek,cv184x-clk driver, so dwc2_clk_enable_bulk() is a
 * no-op; without this GSNPSID reads garbage and probe returns -ENODEV
 * ("Port not available.").
 */
#if defined(CONFIG_USB_GADGET_DWC2_OTG) || defined(CONFIG_USB_DWC2)
#define CV184X_REG_CLK_EN_0		0x0E8
#define CV184X_REG_CLK_EN_1		0x0EC
#define CV184X_REG_CLK_EN_4		0x0F8

static void cv184x_usb_clocks_enable(void)
{
	uint32_t v;

	v = mmio_read_32(CLOCK_GEN_BASE + CV184X_REG_CLK_EN_0);
	/* fab_100m + hsperi: parents for hsperi-domain clocks (see clk-cv184x.c) */
	v |= BIT(0) | BIT(1) | BIT(31); /* + USB20_BUS_EARLY */
	mmio_write_32(CLOCK_GEN_BASE + CV184X_REG_CLK_EN_0, v);

	v = mmio_read_32(CLOCK_GEN_BASE + CV184X_REG_CLK_EN_1);
	v |= BIT(0) | BIT(1) | BIT(2); /* suspend, ref, coreclkin */
	mmio_write_32(CLOCK_GEN_BASE + CV184X_REG_CLK_EN_1, v);

	v = mmio_read_32(CLOCK_GEN_BASE + CV184X_REG_CLK_EN_4);
	v |= BIT(6) | BIT(7); /* clk_axi4_usb, clk_apb_usb */
	mmio_write_32(CLOCK_GEN_BASE + CV184X_REG_CLK_EN_4, v);
}

/*
 * Low-level USB prep matching Linux drivers/usb/dwc2/platform.c
 * (dwc2_set_hw_id): host clears ID bits then sets 0x40; peripheral sets 0xC0.
 */
static void cv184x_usb_controller_prepare(int peripheral)
{
	uint32_t val;

	cv184x_usb_clocks_enable();
	udelay(200);

	val = mmio_read_32(TOP_BASE + REG_TOP_SOFT_RST) & ~BIT_TOP_SOFT_RST_USB;
	mmio_write_32(TOP_BASE + REG_TOP_SOFT_RST, val);
	udelay(50);
	val = mmio_read_32(TOP_BASE + REG_TOP_SOFT_RST) | BIT_TOP_SOFT_RST_USB;
	mmio_write_32(TOP_BASE + REG_TOP_SOFT_RST, val);

	/* TOP USB reset may affect gate state; reprogram clocks. */
	cv184x_usb_clocks_enable();
	udelay(200);

	val = mmio_read_32(REG_TOP_USB_PHY_CTRL);
	val &= ~0xC0;
	val |= BIT_TOP_USB_PHY_CTRL_EXTVBUS;
	val |= peripheral ? 0xC0 : 0x40;
	mmio_write_32(REG_TOP_USB_PHY_CTRL, val);

	/*
	 * Linux dwc2 cvitek UTMI path uses phy REG014; clearing matches
	 * utmi_reset() so the transceiver can leave reset before reading
	 * controller ID (GSNPSID).
	 */
	mmio_write_32(USB2_0_PHY_BASE + 0x14, 0);
	udelay(50);

	mmio_write_32(REG_TOP_USB_ECO,
		      mmio_read_32(REG_TOP_USB_ECO) | BIT_TOP_USB_ECO_RX_FLUSH);
}
#endif

#ifdef CONFIG_USB_DWC2
int dwc2_board_usb_init(struct udevice *dev)
{
#ifdef CONFIG_USB_DWC2_VERBOSE_DEBUG
	printf("[USB dwc2] board hook %s: CV184X PHY+clk+reset (host)\n",
	       dev ? dev->name : "(no dev)");
#endif
	cv184x_usb_controller_prepare(0);
	return 0;
}
#endif

#ifdef CONFIG_USB_GADGET_DWC2_OTG
struct dwc2_plat_otg_data cv182x_otg_data = {
	.regs_otg = USB_BASE,
	.usb_gusbcfg    = 0x40081400,
	.rx_fifo_sz     = 512,
	.np_tx_fifo_sz  = 512,
	.tx_fifo_sz     = 512,
};

int board_usb_init(int index, enum usb_init_type init)
{
	cv184x_usb_controller_prepare(1);

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

struct image_header *spl_get_load_buffer(ssize_t offset, size_t size)
{
	return (struct image_header *)CVIMMAP_UIMAG_ADDR;
}
