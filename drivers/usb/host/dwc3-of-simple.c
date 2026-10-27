// SPDX-License-Identifier: GPL-2.0+
/*
 * dwc3-of-simple.c - OF glue layer for simple integrations
 *
 * Copyright (c) 2015 Texas Instruments Incorporated - http://www.ti.com
 *
 * Author: Felipe Balbi <balbi@ti.com>
 *
 * Copyright (C) 2018 BayLibre, SAS
 * Author: Neil Armstrong <narmstron@baylibre.com>
 */

#include <common.h>
#include <dm.h>
#include <reset.h>
#include <clk.h>
#include <asm/io.h>
#include <dm/of_access.h>
#include <linux/delay.h>
#include <asm/gpio.h>

struct dwc3_of_simple {
	struct clk_bulk		clks;
	struct reset_ctl_bulk	resets;
};

static int dwc3_of_simple_reset_init(struct udevice *dev,
				     struct dwc3_of_simple *simple)
{
	int ret;

	ret = reset_get_bulk(dev, &simple->resets);
	if (ret == -ENOTSUPP)
		return 0;
	else if (ret)
		return ret;

	ret = reset_deassert_bulk(&simple->resets);
	if (ret) {
		reset_release_bulk(&simple->resets);
		return ret;
	}

	return 0;
}

static int dwc3_of_simple_clk_init(struct udevice *dev,
				   struct dwc3_of_simple *simple)
{
	int ret;

	ret = clk_get_bulk(dev, &simple->clks);
	if (ret == -ENOSYS)
		return 0;
	if (ret)
		return ret;

#if CONFIG_IS_ENABLED(CLK)
	ret = clk_enable_bulk(&simple->clks);
	if (ret) {
		clk_release_bulk(&simple->clks);
		return ret;
	}
#endif

	return 0;
}

/*
 * Both the CV186X (88a2) and CV84X6 (84x6) DWC3 controllers are described in
 * device tree with the same "sophgo,cv186x-dwc3" compatible, but their USB
 * subsystem / SuperSpeed-PHY register layouts and bring-up sequences are
 * completely different. The two SoCs are mutually-exclusive build targets
 * (CONFIG_TARGET_CVITEK_CV84X6 vs CONFIG_TARGET_CVITEK_CV186X),
 * so the correct PHY-init variant is selected at compile time below.
 */
#if IS_ENABLED(CONFIG_TARGET_CVITEK_CV84X6)

#define USB_SUBSYS_TOP_OFFSET			0x200000
#define USB_SUBSYS_PHY0_OFFSET			0x140000
#define USB_SUBSYS_PHY1_OFFSET			0x1c0000

/*
 * The two DWC3 controllers share a single USB subsystem, but each owns its own
 * SuperSpeed PHY and its own TOP "general control" register:
 *   USB0: top @ subsys+0x200000, PHY0 @ subsys+0x140000
 *   USB1: top @ subsys+0x200100, PHY1 @ subsys+0x1c0000
 * The controllers' TOP registers are 0x100 apart while the subsystem base is
 * common, so USB1's base cannot be derived as (top - 0x200000).
 */
#define USB_TOP_CTRL_STRIDE			0x100

#define USB_PHY_SRAM_CTRL			0x08
#define USB_PHY_MISC_INPUT			0x10
#define USB_TOP_GENERAL_PWR_CTRL_USB0		0x00

#define USB_PHY_BL_BYPASS_EN			GENMASK(1, 0)
#define USB_PHY_SRAM_INIT_BYP			BIT(0)
#define USB_PHY_SRAM_LOAD_DONE			BIT(1)
#define USB_PHY_SRAM_INIT_DONE			BIT(2)

#define USB_PHY_RX0_TERM_MODE			BIT(5)

static void cv84x6_usb_config_phy_bl_bypass(void __iomem *reg_phy, bool bypass)
{
	u32 value;

	value = readl(reg_phy + USB_PHY_SRAM_CTRL);
	value &= ~USB_PHY_BL_BYPASS_EN;
	if (bypass)
		value |= USB_PHY_SRAM_INIT_BYP;
	else
		value |= USB_PHY_SRAM_LOAD_DONE;
	writel(value, reg_phy + USB_PHY_SRAM_CTRL);
}

static int cv84x6_usb_wait_sram_init_done(void __iomem *reg_phy)
{
	u32 value = 0;
	u32 check = USB_PHY_SRAM_INIT_DONE;
	u32 timeout = 1000;

	while (timeout--) {
		value = readl(reg_phy + USB_PHY_SRAM_CTRL);
		if (value & check)
			return 0;
		udelay(10);
	}

	pr_err("%s timeout: %x %x\n", __func__, value, check);

	return -ETIMEDOUT;
}

static int cv84x6_usb_phy_init(fdt_addr_t reg_usb_top)
{
	bool is_usb1 = reg_usb_top & USB_TOP_CTRL_STRIDE;
	/* Subsystem base is common to both controllers; strip USB1's 0x100 stride. */
	fdt_addr_t usb_subsys_base = reg_usb_top - USB_SUBSYS_TOP_OFFSET -
				     (is_usb1 ? USB_TOP_CTRL_STRIDE : 0);
	void __iomem *reg_top = (void __iomem *)reg_usb_top;
	void __iomem *reg_phy = (void __iomem *)(usb_subsys_base +
			(is_usb1 ? USB_SUBSYS_PHY1_OFFSET : USB_SUBSYS_PHY0_OFFSET));
	u32 value;
	int ret;

	value = readl(reg_phy + USB_PHY_MISC_INPUT);
	value |= USB_PHY_RX0_TERM_MODE;
	writel(value, reg_phy + USB_PHY_MISC_INPUT);

	/* assert phy reset (usb3: bit6, usb2: bit5) */
	value = readl(reg_top + USB_TOP_GENERAL_PWR_CTRL_USB0);
	value |= BIT(6) | BIT(5);
	writel(value, reg_top + USB_TOP_GENERAL_PWR_CTRL_USB0);
	udelay(5);

	/* lane reset */
	value = readl(reg_top + USB_TOP_GENERAL_PWR_CTRL_USB0);
	value &= ~GENMASK(1, 0);
	writel(value, reg_top + USB_TOP_GENERAL_PWR_CTRL_USB0);
	udelay(15);

	/* release phy reset */
	value = readl(reg_top + USB_TOP_GENERAL_PWR_CTRL_USB0);
	value &= ~(BIT(6) | BIT(5));
	writel(value, reg_top + USB_TOP_GENERAL_PWR_CTRL_USB0);
	udelay(10);

	ret = cv84x6_usb_wait_sram_init_done(reg_phy);
	if (ret)
		return ret;

	cv84x6_usb_config_phy_bl_bypass(reg_phy, false);

	/* release lane reset */
	setbits_le32(reg_top + USB_TOP_GENERAL_PWR_CTRL_USB0, GENMASK(1, 0));
	udelay(20 * 1000);

	return 0;
}

static int sophgo_dwc3_phy_init(struct udevice *dev)
{
	fdt_addr_t reg_usb_top;

	reg_usb_top = dev_read_addr_index(dev, 0);
	if (reg_usb_top == FDT_ADDR_T_NONE)
		return -EINVAL;

	printf("CV84X6 USB: simple probe %s\n", dev->name);

	return cv84x6_usb_phy_init(reg_usb_top);
}

#else /* CV186X (88a2) */

#define OTP_USB_XTAL_PHY_MASK	0x3

#define REG_USB_SYS_REG_00		0x0
#define REG_USB_EN				BIT(0)

#define REG_USB_SYS_REG_0C		0x0c
#define REG_PHY_REF_CLKDIV2                BIT(0)
#define REG_PHY_FSEL_POS                   1
#define REG_PHY_FSEL_MSK                   (0x3fL << REG_PHY_FSEL_POS)
#define REG_PHY_MPLL_MULTIPLIER_POS        7
#define REG_PHY_MPLL_MULTIPLIER_MSK        (0x7fL << REG_PHY_MPLL_MULTIPLIER_POS)
#define REG_PHY_SSC_REF_CLK_SEL_POS        14
#define REG_PHY_SSC_REF_CLK_SEL_MSK        (0x1ffL << REG_PHY_SSC_REF_CLK_SEL_POS)
#define REG_PHY_REF_SSP_EN					BIT(24)

#define REG_USB_SYS_REG_10			0x10
#define REG_PHY_RX0LOSLEPSEM			BIT(23)

#define REG_USB_SYS_REG_14					0x14
#define REG_PHY_PHY_RESET					BIT(5)

#define USB_PHY_TUNE_CTRL_REG0				0x0

#define USB_PHY_TUNE_CTRL_REG1				0x4
#define REG_USB_PHY_PCS_RX_LOS_MASK_VAL_POS 13
#define REG_USB_PHY_PCS_RX_LOS_MASK_VAL_MSK (0x3ff << REG_USB_PHY_PCS_RX_LOS_MASK_VAL_POS)

#define USB_PHY_TUNE_CTRL_REG2				0x8

static int sophgo_dwc3_phy_init(struct udevice *dev)
{
	fdt_addr_t reg_usbsys, reg_phy_tune_ctrl_reg;
	u32 value;
	u32 los_mask;
	u32 otp_usb_xtal_phy;
	struct gpio_desc vbus;

	reg_usbsys = dev_read_addr_index(dev, 0);
	reg_phy_tune_ctrl_reg = dev_read_addr_index(dev, 1);

	value = readl(reg_usbsys + REG_USB_SYS_REG_14) | (REG_PHY_PHY_RESET);
	writel(value, reg_usbsys + REG_USB_SYS_REG_14);

	mdelay(20);

	value = readl(reg_usbsys + REG_USB_SYS_REG_14) & (~REG_PHY_PHY_RESET);
	writel(value, reg_usbsys + REG_USB_SYS_REG_14);

	value = readl(reg_usbsys + REG_USB_SYS_REG_0C) | (REG_PHY_REF_SSP_EN);
	writel(value, reg_usbsys + REG_USB_SYS_REG_0C);

	value = readl(reg_usbsys + REG_USB_SYS_REG_00) | (REG_USB_EN);
	writel(value, reg_usbsys + REG_USB_SYS_REG_00);

	value = readl(reg_usbsys + REG_USB_SYS_REG_10) | REG_PHY_RX0LOSLEPSEM;
	writel(value, reg_usbsys + REG_USB_SYS_REG_10);

	otp_usb_xtal_phy = readl(reg_usbsys + REG_USB_SYS_REG_0C) & OTP_USB_XTAL_PHY_MASK;

	value &= readl(reg_usbsys + REG_USB_SYS_REG_0C)
			& ~(REG_PHY_REF_CLKDIV2) & ~(REG_PHY_FSEL_MSK)
			& ~(REG_PHY_MPLL_MULTIPLIER_MSK) & ~(REG_PHY_SSC_REF_CLK_SEL_MSK);
	switch (otp_usb_xtal_phy) {
	case 0x0:    // xtal = 24 MHz
		value |= (0x2A << REG_PHY_FSEL_POS);
		los_mask = 240;
		break;
	case 0x1:    // xtal = 19.2 MHz
		value |= (0x38 << REG_PHY_FSEL_POS);
		los_mask = 192;
		break;
	case 0x2:    // xtal = 20 MHz
		value |= (0x31 << REG_PHY_FSEL_POS);
		los_mask = 200;
		break;
	case 0x3:    // xtal = 40 MHz
		value |= REG_PHY_REF_CLKDIV2 | (0x31 << REG_PHY_FSEL_POS);
		los_mask = 200;
		break;
	}
	writel(value, reg_usbsys + REG_USB_SYS_REG_0C);

	value = readl(reg_phy_tune_ctrl_reg + USB_PHY_TUNE_CTRL_REG1) & ~(REG_USB_PHY_PCS_RX_LOS_MASK_VAL_MSK);
	value |= (los_mask << REG_USB_PHY_PCS_RX_LOS_MASK_VAL_POS);
	writel(value, reg_phy_tune_ctrl_reg + USB_PHY_TUNE_CTRL_REG1);

	if (gpio_request_by_name(dev, "vbus-gpio", 0, &vbus, GPIOD_IS_OUT))
		return -EINVAL;

	dm_gpio_set_value(&vbus, 1);

	return 0;
}

#endif /* CONFIG_TARGET_CVITEK_CV84X6 */

static int dwc3_of_simple_probe(struct udevice *dev)
{
	struct dwc3_of_simple *simple = dev_get_plat(dev);
	int ret;

	ret = dwc3_of_simple_clk_init(dev, simple);
	if (ret)
		return ret;

	ret = dwc3_of_simple_reset_init(dev, simple);
	if (ret)
		return ret;

	if (device_is_compatible(dev, "sophgo,cv186x-dwc3")) {
		ret = sophgo_dwc3_phy_init(dev);
		if (ret)
			return ret;
	}

	return 0;
}

static int dwc3_of_simple_remove(struct udevice *dev)
{
	struct dwc3_of_simple *simple = dev_get_plat(dev);

	reset_release_bulk(&simple->resets);

	clk_release_bulk(&simple->clks);

	return dm_scan_fdt_dev(dev);
}

static const struct udevice_id dwc3_of_simple_ids[] = {
	{ .compatible = "amlogic,meson-gxl-dwc3" },
	{ .compatible = "rockchip,rk3399-dwc3" },
	{ .compatible = "ti,dwc3" },
	{ .compatible = "sophgo,cv186x-dwc3" },
	{ }
};

U_BOOT_DRIVER(dwc3_of_simple) = {
	.name = "dwc3-of-simple",
	.id = UCLASS_SIMPLE_BUS,
	.of_match = dwc3_of_simple_ids,
	.probe = dwc3_of_simple_probe,
	.remove = dwc3_of_simple_remove,
	.plat_auto	= sizeof(struct dwc3_of_simple),
	.flags = DM_FLAG_ALLOC_PRIV_DMA,
};
