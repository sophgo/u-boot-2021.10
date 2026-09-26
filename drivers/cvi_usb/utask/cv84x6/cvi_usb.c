// SPDX-License-Identifier: GPL-2.0+
/**********************************************************************
 * main.c
 *
 * USB Core Driver
 * main component function
 ***********************************************************************/
#include "include/debug.h"
#include "include/cvi_usb.h"
#include "include/platform_def.h"
#include <common.h>
#include <linux/delay.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <mmio.h>
#include <cpu_func.h>
#include <asm/io.h>

extern int acm_app(void);

/*
 * USB subsystem / AKS SuperSpeed PHY register map and init sequence.
 * Synced from drivers/usb/host/dwc3-of-simple.c (cv84x2_usb_phy_init) so the
 * cvi_utask device-mode path brings the PHY up exactly like the DWC3 host
 * probe does. The two DWC3 controllers share one subsystem but each owns a
 * SuperSpeed PHY and a TOP "general control" register:
 *   USB0: top @ subsys+0x200000, PHY0 @ subsys+0x140000
 *   USB1: top @ subsys+0x200100, PHY1 @ subsys+0x1c0000
 * The utask only drives controller 0 (USB_DRD0_BASE), so cvi_usb_hw_init()
 * passes the USB0 top base.
 */
#define USB_SUBSYS_TOP_OFFSET			0x200000
#define USB_SUBSYS_PHY0_OFFSET			0x140000
#define USB_SUBSYS_PHY1_OFFSET			0x1c0000
/* ICP data windows: each PHY's internal-register window sits PHY_offset+0x20000 */
#define USB_SUBSYS_ICP0_OFFSET			0x160000
#define USB_SUBSYS_ICP1_OFFSET			0x1e0000

/* USB1's TOP register is 0x100 above USB0's; the subsystem base is common. */
#define USB_TOP_CTRL_STRIDE			0x100

#define USB_PHY0_SRAM_CTRL			0x08
#define USB_PHY0_MISC_INPUT			0x10
#define USB_TOP_GENERAL_PWR_CTRL_USB0		0x00

#define USB_PHY0_BL_BYPASS_EN		GENMASK(1, 0)
#define USB_PHY0_SRAM_INIT_BYP		BIT(0)
#define USB_PHY0_SRAM_LOAD_DONE		BIT(1)
#define USB_PHY0_SRAM_INIT_DONE		BIT(2)

#define USB_PHY0_RX0_TERM_MODE		BIT(5)

static void cv84x2_usb_config_phy_bl_bypass(void __iomem *reg_phy, bool bypass)
{
	u32 value;

	value = readl(reg_phy + USB_PHY0_SRAM_CTRL);
	value &= ~USB_PHY0_BL_BYPASS_EN;
	if (bypass)
		value |= USB_PHY0_SRAM_INIT_BYP;
	else
		value |= USB_PHY0_SRAM_LOAD_DONE;
	writel(value, reg_phy + USB_PHY0_SRAM_CTRL);
}

static int cv84x2_usb_wait_sram_init_done(void __iomem *reg_phy)
{
	u32 value = 0;
	u32 check = USB_PHY0_SRAM_INIT_DONE;
	u32 timeout = 1000;

	while (timeout--) {
		value = readl(reg_phy + USB_PHY0_SRAM_CTRL);
		if (value & check)
			return 0;
		udelay(10);
	}

	printf("%s timeout: %x %x\n", __func__, value, check);

	return -ETIMEDOUT;
}

static int cv84x2_usb_phy_init(unsigned long reg_usb_top)
{
	bool is_usb1 = reg_usb_top & USB_TOP_CTRL_STRIDE;
	/* Subsystem base is common to both controllers; strip USB1's 0x100 stride. */
	unsigned long usb_subsys_base = reg_usb_top - USB_SUBSYS_TOP_OFFSET -
					(is_usb1 ? USB_TOP_CTRL_STRIDE : 0);
	void __iomem *reg_top = (void __iomem *)reg_usb_top;
	void __iomem *reg_phy = (void __iomem *)(usb_subsys_base +
			(is_usb1 ? USB_SUBSYS_PHY1_OFFSET : USB_SUBSYS_PHY0_OFFSET));
	u32 value;
	int ret;

	printf("CV84X6 USB: init PHY%d top=0x%lx phy=0x%lx\n",
	       is_usb1 ? 1 : 0, (ulong)reg_usb_top, (ulong)reg_phy);

	value = readl(reg_phy + USB_PHY0_MISC_INPUT);
	value |= USB_PHY0_RX0_TERM_MODE;
	writel(value, reg_phy + USB_PHY0_MISC_INPUT);

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

	ret = cv84x2_usb_wait_sram_init_done(reg_phy);
	if (ret)
		return ret;

	cv84x2_usb_config_phy_bl_bypass(reg_phy, false);

	/* release lane reset */
	setbits_le32(reg_top + USB_TOP_GENERAL_PWR_CTRL_USB0, GENMASK(1, 0));
	udelay(20 * 1000);

	printf("CV84X6 USB: PHY%d init done\n", is_usb1 ? 1 : 0);

	return 0;
}

void cvi_usb_hw_init(void)
{
	/* utask drives controller 0: USB0 TOP general-control register. */
	cv84x2_usb_phy_init(USB_SUBSYS_BASE + USB_SUBSYS_TOP_OFFSET);
}


/* program starts here */
int cvi_usb_polling(void)
{
	cvi_usb_hw_init();

	acm_app();

	return 0;
}
