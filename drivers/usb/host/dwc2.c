// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2012 Oleksandr Tymoshenko <gonzo@freebsd.org>
 * Copyright (C) 2014 Marek Vasut <marex@denx.de>
 */

#include <common.h>
#include <clk.h>
#include <cpu_func.h>
#include <dm.h>
#include <errno.h>
#include <generic-phy.h>
#include <log.h>
#include <malloc.h>
#include <memalign.h>
#include <phys2bus.h>
#include <usb.h>
#include <usbroothubdes.h>
#include <wait_bit.h>
#include <asm/cache.h>
#include <asm/io.h>
#include <dm/device_compat.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <power/regulator.h>
#include <reset.h>

#include "dwc2.h"

/*
 * Keep verbose DWC2 diagnostics opt-in for production images.
 * Board defconfig/Kconfig can override these defaults.
 */
#ifndef CONFIG_USB_DWC2_PIO_IN_DIAG
#define CONFIG_USB_DWC2_PIO_IN_DIAG 0
#endif
#ifndef CONFIG_USB_DWC2_VERBOSE_DEBUG
#define CONFIG_USB_DWC2_VERBOSE_DEBUG 0
#endif
#ifndef CONFIG_USB_DWC2_PIO_TIMEOUT_MS
#define CONFIG_USB_DWC2_PIO_TIMEOUT_MS 2000
#endif
#if CONFIG_USB_DWC2_VERBOSE_DEBUG
#define DWC2_INFO(fmt, ...) printf("[USB dwc2] " fmt, ##__VA_ARGS__)
#else
#define DWC2_INFO(fmt, ...) do { } while (0)
#endif
#define DWC2_ERR(fmt, ...) printf("[USB dwc2] " fmt, ##__VA_ARGS__)
#define DWC2_PIO_IN_POP_LOG_MAX		24

/* Use only HC channel 0. */
#define DWC2_HC_CHANNEL			0

#define DWC2_STATUS_BUF_SIZE		64
#define DWC2_DATA_BUF_SIZE		(CONFIG_USB_DWC2_BUFFER_SIZE * 1024)

#define MAX_DEVICE			16
#define MAX_ENDPOINT			16

struct dwc2_priv {
#if CONFIG_IS_ENABLED(DM_USB)
	uint8_t aligned_buffer[DWC2_DATA_BUF_SIZE] __aligned(ARCH_DMA_MINALIGN);
	uint8_t status_buffer[DWC2_STATUS_BUF_SIZE] __aligned(ARCH_DMA_MINALIGN);
#ifdef CONFIG_DM_REGULATOR
	struct udevice *vbus_supply;
#endif
	struct phy phy;
	struct clk_bulk clks;
#else
	uint8_t *aligned_buffer;
	uint8_t *status_buffer;
#endif
	u8 in_data_toggle[MAX_DEVICE][MAX_ENDPOINT];
	u8 out_data_toggle[MAX_DEVICE][MAX_ENDPOINT];
	struct dwc2_core_regs *regs;
	int root_hub_devnum;
	bool ext_vbus;
	/*
	 * The hnp/srp capability must be disabled if the platform
	 * does't support hnp/srp. Otherwise the force mode can't work.
	 */
	bool hnp_srp_disable;
	bool oc_disable;
	bool dma_enabled;

	struct reset_ctl_bulk	resets;
};

#if !CONFIG_IS_ENABLED(DM_USB)
/* We need cacheline-aligned buffers for DMA transfers and dcache support */
DEFINE_ALIGN_BUFFER(uint8_t, aligned_buffer_addr, DWC2_DATA_BUF_SIZE,
		ARCH_DMA_MINALIGN);
DEFINE_ALIGN_BUFFER(uint8_t, status_buffer_addr, DWC2_STATUS_BUF_SIZE,
		ARCH_DMA_MINALIGN);

static struct dwc2_priv local;
#endif

static inline ulong dwc2_pio_timeout_ms(void)
{
	return CONFIG_USB_DWC2_PIO_TIMEOUT_MS;
}

/*
 * DWC2 IP interface
 */

/*
 * Initializes the FSLSPClkSel field of the HCFG register
 * depending on the PHY type.
 */
static void init_fslspclksel(struct dwc2_core_regs *regs)
{
	uint32_t phyclk;

#if (CONFIG_DWC2_PHY_TYPE == DWC2_PHY_TYPE_FS)
	phyclk = DWC2_HCFG_FSLSPCLKSEL_48_MHZ;	/* Full speed PHY */
#else
	/* High speed PHY running at full speed or high speed */
	phyclk = DWC2_HCFG_FSLSPCLKSEL_30_60_MHZ;
#endif

#ifdef CONFIG_DWC2_ULPI_FS_LS
	uint32_t hwcfg2 = readl(&regs->ghwcfg2);
	uint32_t hval = (ghwcfg2 & DWC2_HWCFG2_HS_PHY_TYPE_MASK) >>
			DWC2_HWCFG2_HS_PHY_TYPE_OFFSET;
	uint32_t fval = (ghwcfg2 & DWC2_HWCFG2_FS_PHY_TYPE_MASK) >>
			DWC2_HWCFG2_FS_PHY_TYPE_OFFSET;

	if (hval == 2 && fval == 1)
		phyclk = DWC2_HCFG_FSLSPCLKSEL_48_MHZ;	/* Full speed PHY */
#endif

	clrsetbits_le32(&regs->host_regs.hcfg,
			DWC2_HCFG_FSLSPCLKSEL_MASK,
			phyclk << DWC2_HCFG_FSLSPCLKSEL_OFFSET);
}

/*
 * Flush a Tx FIFO.
 *
 * @param regs Programming view of DWC_otg controller.
 * @param num Tx FIFO to flush.
 */
static void dwc_otg_flush_tx_fifo(struct udevice *dev,
				  struct dwc2_core_regs *regs, const int num)
{
	int ret;

	writel(DWC2_GRSTCTL_TXFFLSH | (num << DWC2_GRSTCTL_TXFNUM_OFFSET),
	       &regs->grstctl);
	ret = wait_for_bit_le32(&regs->grstctl, DWC2_GRSTCTL_TXFFLSH,
				false, 1000, false);
	if (ret)
		dev_info(dev, "%s: Timeout!\n", __func__);

	/* Wait for 3 PHY Clocks */
	udelay(1);
}

/*
 * Flush Rx FIFO.
 *
 * @param regs Programming view of DWC_otg controller.
 */
static void dwc_otg_flush_rx_fifo(struct udevice *dev,
				  struct dwc2_core_regs *regs)
{
	int ret;

	writel(DWC2_GRSTCTL_RXFFLSH, &regs->grstctl);
	ret = wait_for_bit_le32(&regs->grstctl, DWC2_GRSTCTL_RXFFLSH,
				false, 1000, false);
	if (ret)
		dev_info(dev, "%s: Timeout!\n", __func__);

	/* Wait for 3 PHY Clocks */
	udelay(1);
}

/*
 * Do core a soft reset of the core.  Be careful with this because it
 * resets all the internal state machines of the core.
 */
static void dwc_otg_core_reset(struct udevice *dev,
			       struct dwc2_core_regs *regs)
{
	int ret;

	/* Wait for AHB master IDLE state. */
	ret = wait_for_bit_le32(&regs->grstctl, DWC2_GRSTCTL_AHBIDLE,
				true, 1000, false);
	if (ret)
		dev_info(dev, "%s: Timeout!\n", __func__);

	/* Core Soft Reset */
	writel(DWC2_GRSTCTL_CSFTRST, &regs->grstctl);
	ret = wait_for_bit_le32(&regs->grstctl, DWC2_GRSTCTL_CSFTRST,
				false, 1000, false);
	if (ret)
		dev_info(dev, "%s: Timeout!\n", __func__);

	/*
	 * Wait for core to come out of reset.
	 * NOTE: This long sleep is _very_ important, otherwise the core will
	 *       not stay in host mode after a connector ID change!
	 */
	mdelay(100);
}

#if CONFIG_IS_ENABLED(DM_USB) && defined(CONFIG_DM_REGULATOR)
static int dwc_vbus_supply_init(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);
	int ret;

	ret = device_get_supply_regulator(dev, "vbus-supply",
					  &priv->vbus_supply);
	if (ret) {
		debug("%s: No vbus supply\n", dev->name);
		return 0;
	}

	ret = regulator_set_enable(priv->vbus_supply, true);
	if (ret) {
		dev_err(dev, "Error enabling vbus supply\n");
		return ret;
	}

	return 0;
}

static int dwc_vbus_supply_exit(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);
	int ret;

	if (priv->vbus_supply) {
		ret = regulator_set_enable(priv->vbus_supply, false);
		if (ret) {
			dev_err(dev, "Error disabling vbus supply\n");
			return ret;
		}
	}

	return 0;
}
#else
static int dwc_vbus_supply_init(struct udevice *dev)
{
	return 0;
}

#if CONFIG_IS_ENABLED(DM_USB)
static int dwc_vbus_supply_exit(struct udevice *dev)
{
	return 0;
}
#endif
#endif

/*
 * This function initializes the DWC_otg controller registers for
 * host mode.
 *
 * This function flushes the Tx and Rx FIFOs and it flushes any entries in the
 * request queues. Host channels are reset to ensure that they are ready for
 * performing transfers.
 *
 * @param dev USB Device (NULL if driver model is not being used)
 * @param regs Programming view of DWC_otg controller
 *
 */
static void dwc_otg_core_host_init(struct udevice *dev,
				   struct dwc2_core_regs *regs)
{
	uint32_t nptxfifosize = 0;
	uint32_t ptxfifosize = 0;
	uint32_t hprt0 = 0;
	int i, num_channels;

	/* Restart the Phy Clock */
	writel(0, &regs->pcgcctl);

	/* Initialize Host Configuration Register */
	init_fslspclksel(regs);
#ifdef CONFIG_DWC2_DFLT_SPEED_FULL
	setbits_le32(&regs->host_regs.hcfg, DWC2_HCFG_FSLSSUPP);
#endif

	/* Configure data FIFO sizes */
#ifdef CONFIG_DWC2_ENABLE_DYNAMIC_FIFO
	if (readl(&regs->ghwcfg2) & DWC2_HWCFG2_DYNAMIC_FIFO) {
		/* Rx FIFO */
		writel(CONFIG_DWC2_HOST_RX_FIFO_SIZE, &regs->grxfsiz);

		/* Non-periodic Tx FIFO */
		nptxfifosize |= CONFIG_DWC2_HOST_NPERIO_TX_FIFO_SIZE <<
				DWC2_FIFOSIZE_DEPTH_OFFSET;
		nptxfifosize |= CONFIG_DWC2_HOST_RX_FIFO_SIZE <<
				DWC2_FIFOSIZE_STARTADDR_OFFSET;
		writel(nptxfifosize, &regs->gnptxfsiz);

		/* Periodic Tx FIFO */
		ptxfifosize |= CONFIG_DWC2_HOST_PERIO_TX_FIFO_SIZE <<
				DWC2_FIFOSIZE_DEPTH_OFFSET;
		ptxfifosize |= (CONFIG_DWC2_HOST_RX_FIFO_SIZE +
				CONFIG_DWC2_HOST_NPERIO_TX_FIFO_SIZE) <<
				DWC2_FIFOSIZE_STARTADDR_OFFSET;
		writel(ptxfifosize, &regs->hptxfsiz);
	}
#endif

	/* Clear Host Set HNP Enable in the OTG Control Register */
	clrbits_le32(&regs->gotgctl, DWC2_GOTGCTL_HSTSETHNPEN);

	/* Make sure the FIFOs are flushed. */
	dwc_otg_flush_tx_fifo(dev, regs, 0x10);	/* All Tx FIFOs */
	dwc_otg_flush_rx_fifo(dev, regs);

	/*
	 * Clean up channels after core reset.
	 *
	 * The original code used CHEN+CHDIS to halt each channel, but on
	 * some DWC2 instances (e.g. CV184x GSNPSID 4.20a) CHEN/CHDIS are
	 * W1S (write-1-to-set) and can only be cleared by hardware.  If the
	 * halt never completes (HCINT stuck at 0 — observed when GAHBCFG is
	 * hardwired to 0), CHDIS stays asserted and permanently blocks all
	 * subsequent channel transfers.
	 *
	 * Since we just did a core soft-reset, channels are already idle.
	 * Simply clear the interrupt flags so they are in a known state.
	 */
	num_channels = readl(&regs->ghwcfg2);
	num_channels &= DWC2_HWCFG2_NUM_HOST_CHAN_MASK;
	num_channels >>= DWC2_HWCFG2_NUM_HOST_CHAN_OFFSET;
	num_channels += 1;

	for (i = 0; i < num_channels; i++) {
		writel(0, &regs->hc_regs[i].hcintmsk);
		writel(0x3fff, &regs->hc_regs[i].hcint);
	}

	/* Turn on the vbus power. */
	if (readl(&regs->gintsts) & DWC2_GINTSTS_CURMODE_HOST) {
		hprt0 = readl(&regs->hprt0);
		hprt0 &= ~(DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET);
		hprt0 &= ~(DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG);
		if (!(hprt0 & DWC2_HPRT0_PRTPWR)) {
			hprt0 |= DWC2_HPRT0_PRTPWR;
			writel(hprt0, &regs->hprt0);
		}
	}

	if (dev)
		dwc_vbus_supply_init(dev);
}

/*
 * This function initializes the DWC_otg controller registers and
 * prepares the core for device mode or host mode operation.
 *
 * @param regs Programming view of the DWC_otg controller
 */
static void dwc_otg_core_init(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);
	struct dwc2_core_regs *regs = priv->regs;
	uint32_t ahbcfg = 0;
	uint32_t usbcfg = 0;
	uint8_t brst_sz = CONFIG_DWC2_DMA_BURST_SIZE;

	/* Common Initialization */
	usbcfg = readl(&regs->gusbcfg);

	/* Program the ULPI External VBUS bit if needed */
	if (priv->ext_vbus) {
		usbcfg |= DWC2_GUSBCFG_ULPI_EXT_VBUS_DRV;
		if (!priv->oc_disable) {
			usbcfg |= DWC2_GUSBCFG_ULPI_INT_VBUS_INDICATOR |
				  DWC2_GUSBCFG_INDICATOR_PASSTHROUGH;
		}
	} else {
		usbcfg &= ~DWC2_GUSBCFG_ULPI_EXT_VBUS_DRV;
	}

	/* Set external TS Dline pulsing */
#ifdef CONFIG_DWC2_TS_DLINE
	usbcfg |= DWC2_GUSBCFG_TERM_SEL_DL_PULSE;
#else
	usbcfg &= ~DWC2_GUSBCFG_TERM_SEL_DL_PULSE;
#endif
	writel(usbcfg, &regs->gusbcfg);

	/* Reset the Controller */
	dwc_otg_core_reset(dev, regs);

	/*
	 * This programming sequence needs to happen in FS mode before
	 * any other programming occurs
	 */
#if defined(CONFIG_DWC2_DFLT_SPEED_FULL) && \
	(CONFIG_DWC2_PHY_TYPE == DWC2_PHY_TYPE_FS)
	/* If FS mode with FS PHY */
	setbits_le32(&regs->gusbcfg, DWC2_GUSBCFG_PHYSEL);

	/* Reset after a PHY select */
	dwc_otg_core_reset(dev, regs);

	/*
	 * Program DCFG.DevSpd or HCFG.FSLSPclkSel to 48Mhz in FS.
	 * Also do this on HNP Dev/Host mode switches (done in dev_init
	 * and host_init).
	 */
	if (readl(&regs->gintsts) & DWC2_GINTSTS_CURMODE_HOST)
		init_fslspclksel(regs);

#ifdef CONFIG_DWC2_I2C_ENABLE
	/* Program GUSBCFG.OtgUtmifsSel to I2C */
	setbits_le32(&regs->gusbcfg, DWC2_GUSBCFG_OTGUTMIFSSEL);

	/* Program GI2CCTL.I2CEn */
	clrsetbits_le32(&regs->gi2cctl, DWC2_GI2CCTL_I2CEN |
			DWC2_GI2CCTL_I2CDEVADDR_MASK,
			1 << DWC2_GI2CCTL_I2CDEVADDR_OFFSET);
	setbits_le32(&regs->gi2cctl, DWC2_GI2CCTL_I2CEN);
#endif

#else
	/* High speed PHY. */

	/*
	 * HS PHY parameters. These parameters are preserved during
	 * soft reset so only program the first time. Do a soft reset
	 * immediately after setting phyif.
	 */
	usbcfg &= ~(DWC2_GUSBCFG_ULPI_UTMI_SEL | DWC2_GUSBCFG_PHYIF);
	usbcfg |= CONFIG_DWC2_PHY_TYPE << DWC2_GUSBCFG_ULPI_UTMI_SEL_OFFSET;

	if (usbcfg & DWC2_GUSBCFG_ULPI_UTMI_SEL) {	/* ULPI interface */
#ifdef CONFIG_DWC2_PHY_ULPI_DDR
		usbcfg |= DWC2_GUSBCFG_DDRSEL;
#else
		usbcfg &= ~DWC2_GUSBCFG_DDRSEL;
#endif
	} else {	/* UTMI+ interface */
#if (CONFIG_DWC2_UTMI_WIDTH == 16)
		usbcfg |= DWC2_GUSBCFG_PHYIF;
#endif
	}

	writel(usbcfg, &regs->gusbcfg);

	/* Reset after setting the PHY parameters */
	dwc_otg_core_reset(dev, regs);
#endif

	usbcfg = readl(&regs->gusbcfg);
	usbcfg &= ~(DWC2_GUSBCFG_ULPI_FSLS | DWC2_GUSBCFG_ULPI_CLK_SUS_M);
#ifdef CONFIG_DWC2_ULPI_FS_LS
	uint32_t hwcfg2 = readl(&regs->ghwcfg2);
	uint32_t hval = (ghwcfg2 & DWC2_HWCFG2_HS_PHY_TYPE_MASK) >>
			DWC2_HWCFG2_HS_PHY_TYPE_OFFSET;
	uint32_t fval = (ghwcfg2 & DWC2_HWCFG2_FS_PHY_TYPE_MASK) >>
			DWC2_HWCFG2_FS_PHY_TYPE_OFFSET;
	if (hval == 2 && fval == 1) {
		usbcfg |= DWC2_GUSBCFG_ULPI_FSLS;
		usbcfg |= DWC2_GUSBCFG_ULPI_CLK_SUS_M;
	}
#endif
	if (priv->hnp_srp_disable)
		usbcfg |= DWC2_GUSBCFG_FORCEHOSTMODE;

	writel(usbcfg, &regs->gusbcfg);

	/* Program the GAHBCFG Register. */
	ahbcfg |= DWC2_GAHBCFG_GLBLINTRMSK;
	switch (readl(&regs->ghwcfg2) & DWC2_HWCFG2_ARCHITECTURE_MASK) {
	case DWC2_HWCFG2_ARCHITECTURE_SLAVE_ONLY:
#ifdef CONFIG_DWC2_DMA_ENABLE
		/*
		 * Many SoCs (incl. CV18xx/CV184x DWC2 4xx) advertises
		 * "slave-only" in GHWCFG2, but the U-Boot DWC2 host path is
		 * entirely HCDMA-based: without DMAENABLE, programmed
		 * transactions never complete (CHHLTD never asserts, HCINT
		 * stays 0, control SETUP times out with -ETIMEDOUT).
		 * Linux enables g_dma/g_dma_desc for the same silicon.
		 */
		if (dev)
			DWC2_INFO("GHWCFG2=SLAVE_ONLY: enabling AHB DMA for HCDMA host path\n");
		ahbcfg |= DWC2_GAHBCFG_HBURSTLEN_INCR4;
		ahbcfg |= DWC2_GAHBCFG_DMAENABLE;
#endif
		break;
	case DWC2_HWCFG2_ARCHITECTURE_EXT_DMA:
		while (brst_sz > 1) {
			ahbcfg |= ahbcfg + (1 << DWC2_GAHBCFG_HBURSTLEN_OFFSET);
			ahbcfg &= DWC2_GAHBCFG_HBURSTLEN_MASK;
			brst_sz >>= 1;
		}

#ifdef CONFIG_DWC2_DMA_ENABLE
		ahbcfg |= DWC2_GAHBCFG_DMAENABLE;
#endif
		break;

	case DWC2_HWCFG2_ARCHITECTURE_INT_DMA:
		ahbcfg |= DWC2_GAHBCFG_HBURSTLEN_INCR4;
#ifdef CONFIG_DWC2_DMA_ENABLE
		ahbcfg |= DWC2_GAHBCFG_DMAENABLE;
#endif
		break;
	}

	writel(ahbcfg, &regs->gahbcfg);
	mb();

	{
		uint32_t ahbcfg_rb = readl(&regs->gahbcfg);
		/*
		 * DMA path programs HCDMA and expects the core in AHB DMA mode.
		 * Only enable it when DMAENABLE reads back set.
		 *
		 * Do NOT infer DMA from the written value alone: on CV184x,
		 * GAHBCFG may read as 0 at this stage while the write does not
		 * latch (DMA inactive). Forcing dma_enabled then breaks all
		 * transfers (CHHLTD timeout). Linux may show a non-zero GAHBCFG
		 * after full clk + dwc2 init; U-Boot stays on PIO until readback
		 * shows DMA, or until board code makes the register stick.
		 */
		priv->dma_enabled = !!(ahbcfg_rb & DWC2_GAHBCFG_DMAENABLE);
		if (dev)
			DWC2_INFO("GAHBCFG: wrote=0x%08x readback=0x%08x DMA_EN=%d HBURSTLEN=%d -> %s mode\n",
				  ahbcfg, ahbcfg_rb,
				  priv->dma_enabled,
				  (ahbcfg_rb & DWC2_GAHBCFG_HBURSTLEN_MASK) >>
					DWC2_GAHBCFG_HBURSTLEN_OFFSET,
				  priv->dma_enabled ? "DMA" : "PIO/slave");
	}

	/* Program the capabilities in GUSBCFG Register */
	usbcfg = 0;

	if (!priv->hnp_srp_disable)
		usbcfg |= DWC2_GUSBCFG_HNPCAP | DWC2_GUSBCFG_SRPCAP;
#ifdef CONFIG_DWC2_IC_USB_CAP
	usbcfg |= DWC2_GUSBCFG_IC_USB_CAP;
#endif

	setbits_le32(&regs->gusbcfg, usbcfg);
}

/*
 * Prepares a host channel for transferring packets to/from a specific
 * endpoint. The HCCHARn register is set up with the characteristics specified
 * in _hc. Host channel interrupts that may need to be serviced while this
 * transfer is in progress are enabled.
 *
 * @param regs Programming view of DWC_otg controller
 * @param hc Information needed to initialize the host channel
 */
static void dwc_otg_hc_init(struct dwc2_core_regs *regs, uint8_t hc_num,
		struct usb_device *dev, uint8_t dev_addr, uint8_t ep_num,
		uint8_t ep_is_in, uint8_t ep_type, uint16_t max_packet)
{
	struct dwc2_hc_regs *hc_regs = &regs->hc_regs[hc_num];
	uint32_t hcchar;

	/* Clear any stale interrupt flags before re-configuring */
	writel(0x3fff, &hc_regs->hcint);

	hcchar = (dev_addr << DWC2_HCCHAR_DEVADDR_OFFSET) |
		 (ep_num << DWC2_HCCHAR_EPNUM_OFFSET) |
		 (ep_is_in << DWC2_HCCHAR_EPDIR_OFFSET) |
		 (ep_type << DWC2_HCCHAR_EPTYPE_OFFSET) |
		 (max_packet << DWC2_HCCHAR_MPS_OFFSET);

	if (dev->speed == USB_SPEED_LOW)
		hcchar |= DWC2_HCCHAR_LSPDDEV;

	writel(hcchar, &hc_regs->hcchar);

	/* Program the HCSPLIT register, default to no SPLIT */
	writel(0, &hc_regs->hcsplt);

	/* Unmask host-channel events we poll through HCINT/CHHLTD. */
	writel(DWC2_HCINT_CHHLTD | DWC2_HCINT_XFERCOMP | DWC2_HCINT_STALL |
	       DWC2_HCINT_NAK | DWC2_HCINT_ACK | DWC2_HCINT_NYET |
	       DWC2_HCINT_XACTERR | DWC2_HCINT_AHBERR |
	       DWC2_HCINT_DATATGLERR | DWC2_HCINT_FRMOVRUN,
	       &hc_regs->hcintmsk);

	/* Route channel IRQ status to HAINT/GINTSTS in host mode. */
	writel(BIT(hc_num), &regs->host_regs.haintmsk);
	setbits_le32(&regs->gintmsk, DWC2_GINTSTS_HCINTR |
		     DWC2_GINTSTS_RXSTSQLVL | DWC2_GINTSTS_PORTINTR);
	if (hc_num == DWC2_HC_CHANNEL) {
		static int hc_diag_once;
		if (!hc_diag_once) {
#if CONFIG_USB_DWC2_VERBOSE_DEBUG
			uint32_t hcchar_rb = readl(&hc_regs->hcchar);
			DWC2_INFO("hc_init: HCCHAR=%08x(CHEN=%d CHDIS=%d) "
				  "GINTMSK=%08x HAINTMSK=%08x HCINTMSK=%08x\n",
				  hcchar_rb,
				  !!(hcchar_rb & DWC2_HCCHAR_CHEN),
				  !!(hcchar_rb & DWC2_HCCHAR_CHDIS),
				  readl(&regs->gintmsk),
				  readl(&regs->host_regs.haintmsk),
				  readl(&hc_regs->hcintmsk));
#endif
			hc_diag_once = 1;
		}
	}
}

static void dwc_otg_hc_init_split(struct dwc2_hc_regs *hc_regs,
				  uint8_t hub_devnum, uint8_t hub_port)
{
	uint32_t hcsplt = 0;

	hcsplt = DWC2_HCSPLT_SPLTENA;
	hcsplt |= hub_devnum << DWC2_HCSPLT_HUBADDR_OFFSET;
	hcsplt |= hub_port << DWC2_HCSPLT_PRTADDR_OFFSET;

	/* Program the HCSPLIT register for SPLITs */
	writel(hcsplt, &hc_regs->hcsplt);
}

/*
 * DWC2 to USB API interface
 */
/* Direction: In ; Request: Status */
static int dwc_otg_submit_rh_msg_in_status(struct dwc2_core_regs *regs,
					   struct usb_device *dev, void *buffer,
					   int txlen, struct devrequest *cmd)
{
	uint32_t hprt0 = 0;
	uint32_t port_status = 0;
	uint32_t port_change = 0;
	int len = 0;
	int stat = 0;

	switch (cmd->requesttype & ~USB_DIR_IN) {
	case 0:
		*(uint16_t *)buffer = cpu_to_le16(1);
		len = 2;
		break;
	case USB_RECIP_INTERFACE:
	case USB_RECIP_ENDPOINT:
		*(uint16_t *)buffer = cpu_to_le16(0);
		len = 2;
		break;
	case USB_TYPE_CLASS:
		*(uint32_t *)buffer = cpu_to_le32(0);
		len = 4;
		break;
	case USB_RECIP_OTHER | USB_TYPE_CLASS:
		hprt0 = readl(&regs->hprt0);
		if (hprt0 & DWC2_HPRT0_PRTCONNSTS)
			port_status |= USB_PORT_STAT_CONNECTION;
		if (hprt0 & DWC2_HPRT0_PRTENA)
			port_status |= USB_PORT_STAT_ENABLE;
		if (hprt0 & DWC2_HPRT0_PRTSUSP)
			port_status |= USB_PORT_STAT_SUSPEND;
		if (hprt0 & DWC2_HPRT0_PRTOVRCURRACT)
			port_status |= USB_PORT_STAT_OVERCURRENT;
		if (hprt0 & DWC2_HPRT0_PRTRST)
			port_status |= USB_PORT_STAT_RESET;
		if (hprt0 & DWC2_HPRT0_PRTPWR)
			port_status |= USB_PORT_STAT_POWER;

		if ((hprt0 & DWC2_HPRT0_PRTSPD_MASK) == DWC2_HPRT0_PRTSPD_LOW)
			port_status |= USB_PORT_STAT_LOW_SPEED;
		else if ((hprt0 & DWC2_HPRT0_PRTSPD_MASK) ==
			 DWC2_HPRT0_PRTSPD_HIGH)
			port_status |= USB_PORT_STAT_HIGH_SPEED;

		if (hprt0 & DWC2_HPRT0_PRTENCHNG)
			port_change |= USB_PORT_STAT_C_ENABLE;
		if (hprt0 & DWC2_HPRT0_PRTCONNDET)
			port_change |= USB_PORT_STAT_C_CONNECTION;
		if (hprt0 & DWC2_HPRT0_PRTOVRCURRCHNG)
			port_change |= USB_PORT_STAT_C_OVERCURRENT;

		*(uint32_t *)buffer = cpu_to_le32(port_status |
					(port_change << 16));
		len = 4;
		break;
	default:
		puts("unsupported root hub command\n");
		stat = USB_ST_STALLED;
	}

	dev->act_len = min(len, txlen);
	dev->status = stat;

	return stat;
}

/* Direction: In ; Request: Descriptor */
static int dwc_otg_submit_rh_msg_in_descriptor(struct usb_device *dev,
					       void *buffer, int txlen,
					       struct devrequest *cmd)
{
	unsigned char data[32];
	uint32_t dsc;
	int len = 0;
	int stat = 0;
	uint16_t wValue = cpu_to_le16(cmd->value);
	uint16_t wLength = cpu_to_le16(cmd->length);

	switch (cmd->requesttype & ~USB_DIR_IN) {
	case 0:
		switch (wValue & 0xff00) {
		case 0x0100:	/* device descriptor */
			len = min3(txlen, (int)sizeof(root_hub_dev_des), (int)wLength);
			memcpy(buffer, root_hub_dev_des, len);
			break;
		case 0x0200:	/* configuration descriptor */
			len = min3(txlen, (int)sizeof(root_hub_config_des), (int)wLength);
			memcpy(buffer, root_hub_config_des, len);
			break;
		case 0x0300:	/* string descriptors */
			switch (wValue & 0xff) {
			case 0x00:
				len = min3(txlen, (int)sizeof(root_hub_str_index0),
					   (int)wLength);
				memcpy(buffer, root_hub_str_index0, len);
				break;
			case 0x01:
				len = min3(txlen, (int)sizeof(root_hub_str_index1),
					   (int)wLength);
				memcpy(buffer, root_hub_str_index1, len);
				break;
			}
			break;
		default:
			stat = USB_ST_STALLED;
		}
		break;

	case USB_TYPE_CLASS:
		/* Root port config, set 1 port and nothing else. */
		dsc = 0x00000001;

		data[0] = 9;		/* min length; */
		data[1] = 0x29;
		data[2] = dsc & RH_A_NDP;
		data[3] = 0;
		if (dsc & RH_A_PSM)
			data[3] |= 0x1;
		if (dsc & RH_A_NOCP)
			data[3] |= 0x10;
		else if (dsc & RH_A_OCPM)
			data[3] |= 0x8;

		/* corresponds to data[4-7] */
		data[5] = (dsc & RH_A_POTPGT) >> 24;
		data[7] = dsc & RH_B_DR;
		if (data[2] < 7) {
			data[8] = 0xff;
		} else {
			data[0] += 2;
			data[8] = (dsc & RH_B_DR) >> 8;
			data[9] = 0xff;
			data[10] = data[9];
		}

		len = min3(txlen, (int)data[0], (int)wLength);
		memcpy(buffer, data, len);
		break;
	default:
		puts("unsupported root hub command\n");
		stat = USB_ST_STALLED;
	}

	dev->act_len = min(len, txlen);
	dev->status = stat;

	return stat;
}

/* Direction: In ; Request: Configuration */
static int dwc_otg_submit_rh_msg_in_configuration(struct usb_device *dev,
						  void *buffer, int txlen,
						  struct devrequest *cmd)
{
	int len = 0;
	int stat = 0;

	switch (cmd->requesttype & ~USB_DIR_IN) {
	case 0:
		*(uint8_t *)buffer = 0x01;
		len = 1;
		break;
	default:
		puts("unsupported root hub command\n");
		stat = USB_ST_STALLED;
	}

	dev->act_len = min(len, txlen);
	dev->status = stat;

	return stat;
}

/* Direction: In */
static int dwc_otg_submit_rh_msg_in(struct dwc2_priv *priv,
				    struct usb_device *dev, void *buffer,
				    int txlen, struct devrequest *cmd)
{
	switch (cmd->request) {
	case USB_REQ_GET_STATUS:
		return dwc_otg_submit_rh_msg_in_status(priv->regs, dev, buffer,
						       txlen, cmd);
	case USB_REQ_GET_DESCRIPTOR:
		return dwc_otg_submit_rh_msg_in_descriptor(dev, buffer,
							   txlen, cmd);
	case USB_REQ_GET_CONFIGURATION:
		return dwc_otg_submit_rh_msg_in_configuration(dev, buffer,
							      txlen, cmd);
	default:
		puts("unsupported root hub command\n");
		return USB_ST_STALLED;
	}
}

/* Direction: Out */
static int dwc_otg_submit_rh_msg_out(struct dwc2_priv *priv,
				     struct usb_device *dev,
				     void *buffer, int txlen,
				     struct devrequest *cmd)
{
	struct dwc2_core_regs *regs = priv->regs;
	int len = 0;
	int stat = 0;
	uint16_t bmrtype_breq = cmd->requesttype | (cmd->request << 8);
	uint16_t wValue = cpu_to_le16(cmd->value);

	switch (bmrtype_breq & ~USB_DIR_IN) {
	case (USB_REQ_CLEAR_FEATURE << 8) | USB_RECIP_ENDPOINT:
	case (USB_REQ_CLEAR_FEATURE << 8) | USB_TYPE_CLASS:
		break;

	case (USB_REQ_CLEAR_FEATURE << 8) | USB_RECIP_OTHER | USB_TYPE_CLASS:
		switch (wValue) {
		case USB_PORT_FEAT_C_CONNECTION:
			setbits_le32(&regs->hprt0, DWC2_HPRT0_PRTCONNDET);
			break;
		}
		break;

	case (USB_REQ_SET_FEATURE << 8) | USB_RECIP_OTHER | USB_TYPE_CLASS:
		switch (wValue) {
		case USB_PORT_FEAT_SUSPEND:
			break;

		case USB_PORT_FEAT_RESET:
			clrsetbits_le32(&regs->hprt0, DWC2_HPRT0_PRTENA |
					DWC2_HPRT0_PRTCONNDET |
					DWC2_HPRT0_PRTENCHNG |
					DWC2_HPRT0_PRTOVRCURRCHNG,
					DWC2_HPRT0_PRTRST);
			mdelay(50);
			clrbits_le32(&regs->hprt0, DWC2_HPRT0_PRTRST);
			break;

		case USB_PORT_FEAT_POWER:
			clrsetbits_le32(&regs->hprt0, DWC2_HPRT0_PRTENA |
					DWC2_HPRT0_PRTCONNDET |
					DWC2_HPRT0_PRTENCHNG |
					DWC2_HPRT0_PRTOVRCURRCHNG,
					DWC2_HPRT0_PRTRST);
			break;

		case USB_PORT_FEAT_ENABLE:
			break;
		}
		break;
	case (USB_REQ_SET_ADDRESS << 8):
		priv->root_hub_devnum = wValue;
		break;
	case (USB_REQ_SET_CONFIGURATION << 8):
		break;
	default:
		puts("unsupported root hub command\n");
		stat = USB_ST_STALLED;
	}

	len = min(len, txlen);

	dev->act_len = len;
	dev->status = stat;

	return stat;
}

static int dwc_otg_submit_rh_msg(struct dwc2_priv *priv, struct usb_device *dev,
				 unsigned long pipe, void *buffer, int txlen,
				 struct devrequest *cmd)
{
	int stat = 0;

	if (usb_pipeint(pipe)) {
		puts("Root-Hub submit IRQ: NOT implemented\n");
		return 0;
	}

	if (cmd->requesttype & USB_DIR_IN)
		stat = dwc_otg_submit_rh_msg_in(priv, dev, buffer, txlen, cmd);
	else
		stat = dwc_otg_submit_rh_msg_out(priv, dev, buffer, txlen, cmd);

	mdelay(1);

	return stat;
}

int wait_for_chhltd(struct dwc2_hc_regs *hc_regs, uint32_t *sub, u8 *toggle)
{
	int ret;
	uint32_t hcint, hctsiz;

	ret = wait_for_bit_le32(&hc_regs->hcint, DWC2_HCINT_CHHLTD, true,
				dwc2_pio_timeout_ms(), false);
	if (ret) {
		hcint = readl(&hc_regs->hcint);
		printf("[USB dwc2] HC CHHLTD timeout (err=%d) HCINT=%08x HCTSIZ=%08x\n",
		       ret, hcint, readl(&hc_regs->hctsiz));
		printf("[USB dwc2]  HCCHAR=%08x(CHEN=%d CHDIS=%d) HCDMA=%08x HCINTMSK=%08x\n",
		       readl(&hc_regs->hcchar),
		       !!(readl(&hc_regs->hcchar) & DWC2_HCCHAR_CHEN),
		       !!(readl(&hc_regs->hcchar) & DWC2_HCCHAR_CHDIS),
		       readl(&hc_regs->hcdma),
		       readl(&hc_regs->hcintmsk));
		return ret;
	}

	hcint = readl(&hc_regs->hcint);
	hctsiz = readl(&hc_regs->hctsiz);
	*sub = (hctsiz & DWC2_HCTSIZ_XFERSIZE_MASK) >>
		DWC2_HCTSIZ_XFERSIZE_OFFSET;
	*toggle = (hctsiz & DWC2_HCTSIZ_PID_MASK) >> DWC2_HCTSIZ_PID_OFFSET;

	debug("%s: HCINT=%08x sub=%u toggle=%d\n", __func__, hcint, *sub,
	      *toggle);

	if (hcint & DWC2_HCINT_XFERCOMP)
		return 0;

	if (hcint & (DWC2_HCINT_NAK | DWC2_HCINT_FRMOVRUN))
		return -EAGAIN;

	printf("[USB dwc2] HC fault HCINT=%08x (stall=%d xacterr=%d ahberr=%d tog=%d)\n",
	       hcint, !!(hcint & DWC2_HCINT_STALL), !!(hcint & DWC2_HCINT_XACTERR),
	       !!(hcint & DWC2_HCINT_AHBERR), !!(hcint & DWC2_HCINT_DATATGLERR));
	debug("%s: Error (HCINT=%08x)\n", __func__, hcint);
	return -EINVAL;
}

static int dwc2_eptype[] = {
	DWC2_HCCHAR_EPTYPE_ISOC,
	DWC2_HCCHAR_EPTYPE_INTR,
	DWC2_HCCHAR_EPTYPE_CONTROL,
	DWC2_HCCHAR_EPTYPE_BULK,
};

/*
 * Flush the non-periodic TX FIFO.  Required on CV184x before halting a
 * channel in slave mode — the halt won't complete if the FIFO is
 * occupied by stale packet data from a failed OUT transfer.
 */
static void dwc2_flush_tx_fifo(struct dwc2_core_regs *regs)
{
	/* TXFNUM=0 selects non-periodic TX FIFO */
	writel(DWC2_GRSTCTL_TXFFLSH | (0 << DWC2_GRSTCTL_TXFNUM_OFFSET),
	       &regs->grstctl);

	/* Wait for flush to complete (TXFFLSH self-clears) */
	{
		ulong start = get_timer(0);

		while (readl(&regs->grstctl) & DWC2_GRSTCTL_TXFFLSH) {
			if (get_timer(start) > 10)
				break;
			udelay(1);
		}
	}
}

/*
 * Flush Rx FIFO (no dev — for PIO error paths).
 * IN timeouts leave status/data in GRXFIFO; TX flush does not help.
 */
static void dwc2_flush_rx_fifo(struct dwc2_core_regs *regs)
{
	writel(DWC2_GRSTCTL_RXFFLSH, &regs->grstctl);
	{
		ulong start = get_timer(0);

		while (readl(&regs->grstctl) & DWC2_GRSTCTL_RXFFLSH) {
			if (get_timer(start) > 10)
				break;
			udelay(1);
		}
	}
	udelay(1);
}

/*
 * PIO channel cleanup: clear interrupt flags only.
 *
 * CV184x quirk: the standard DWC2 halt sequence (CHDIS+CHEN → CHHLTD)
 * does NOT work in slave mode on this SoC — the halt never completes
 * and permanently jams the channel (CHEN=1 CHDIS=1 stuck).
 *
 * Instead we rely on the fact that after XFERCOMP/CHHLTD the hardware
 * clears CHEN naturally.  After NAK/XACTERR the channel may remain
 * enabled; the next dwc_otg_hc_init writes a fresh HCCHAR and
 * dwc2_hc_enable toggles CHEN via clrsetbits_le32 which works because
 * the transition 0→1 re-arms the channel.
 *
 * If the channel is truly stuck (HCINT stays 0x00000000 during a
 * transfer), the PIO timeout path flushes FIFOs as a last resort.
 */
static void dwc2_hc_cleanup(struct dwc2_hc_regs *hc_regs)
{
	writel(0x3fff, &hc_regs->hcint);
}

/*
 * Enable the host channel: programs MULTICNT, clears CHDIS, sets CHEN.
 */
static void dwc2_hc_enable(struct dwc2_hc_regs *hc_regs, int odd_frame)
{
	clrsetbits_le32(&hc_regs->hcchar, DWC2_HCCHAR_MULTICNT_MASK |
			DWC2_HCCHAR_CHEN | DWC2_HCCHAR_CHDIS |
			DWC2_HCCHAR_ODDFRM,
			(1 << DWC2_HCCHAR_MULTICNT_OFFSET) |
			(odd_frame << DWC2_HCCHAR_ODDFRM_OFFSET) |
			DWC2_HCCHAR_CHEN);
}

/*
 * PIO/slave-mode write: push data into the TX FIFO word-by-word.
 * Must be called after the channel has been enabled (CHEN=1).
 */
static void dwc2_pio_write_fifo(struct dwc2_core_regs *regs, int channel,
				const void *buf, int len)
{
	volatile uint32_t *dfifo = (volatile uint32_t *)
		((uintptr_t)regs + DWC2_DFIFO_OFFSET(channel));
	const uint32_t *src = (const uint32_t *)buf;
	int words = (len + 3) / 4;
	int i;

	for (i = 0; i < words; i++)
		writel(src[i], dfifo);
}

/*
 * PIO/slave-mode read: pull data from RX FIFO word-by-word.
 */
static void dwc2_pio_read_fifo(struct dwc2_core_regs *regs, int channel,
			       void *buf, int len)
{
	volatile uint32_t *dfifo = (volatile uint32_t *)
		((uintptr_t)regs + DWC2_DFIFO_OFFSET(channel));
	uint32_t *dst = (uint32_t *)buf;
	int words = (len + 3) / 4;
	int i;

	for (i = 0; i < words; i++)
		dst[i] = readl(dfifo);
}

/*
 * Wait for non-periodic TX FIFO to have enough space for @len bytes.
 * Returns 0 on success, -ETIMEDOUT on failure.
 */
static int dwc2_pio_wait_tx_fifo(struct dwc2_core_regs *regs, int len)
{
	int words_needed = (len + 3) / 4;
	ulong start = get_timer(0);

	while (get_timer(start) < dwc2_pio_timeout_ms()) {
		uint32_t txsts = readl(&regs->gnptxsts);
		int avail = (txsts & DWC2_GNPTXSTS_NPTXFSPCAVAIL_MASK) >>
			     DWC2_GNPTXSTS_NPTXFSPCAVAIL_OFFSET;
		int qavail = (txsts & DWC2_GNPTXSTS_NPTXQSPCAVAIL_MASK) >>
			      DWC2_GNPTXSTS_NPTXQSPCAVAIL_OFFSET;
		if (avail >= words_needed && qavail > 0)
			return 0;
		udelay(1);
	}
	printf("[USB dwc2] PIO TX FIFO space timeout (need %d words)\n",
	       words_needed);
	return -ETIMEDOUT;
}

/*
 * Wait for PIO/slave-mode transfer completion.
 *
 * In slave mode, completed transfers set XFERCOMP (without CHHLTD).
 * NAK/STALL/errors also appear alone (no CHHLTD wrapper).
 * Returns 0 on success, -EAGAIN on NAK, -EINVAL on error, -ETIMEDOUT.
 */
#define DWC2_HCINT_PIO_DONE  (DWC2_HCINT_CHHLTD | DWC2_HCINT_XFERCOMP | \
			      DWC2_HCINT_NAK | DWC2_HCINT_STALL | \
			      DWC2_HCINT_XACTERR | DWC2_HCINT_AHBERR | \
			      DWC2_HCINT_DATATGLERR | DWC2_HCINT_FRMOVRUN)

static int wait_for_pio_complete(struct dwc2_core_regs *regs,
				 struct dwc2_hc_regs *hc_regs,
				 uint32_t *sub, u8 *toggle)
{
	ulong start = get_timer(0);

	while (get_timer(start) < dwc2_pio_timeout_ms()) {
		uint32_t hcint = readl(&hc_regs->hcint);

		if (hcint & DWC2_HCINT_PIO_DONE) {
			uint32_t hctsiz = readl(&hc_regs->hctsiz);

			*sub = (hctsiz & DWC2_HCTSIZ_XFERSIZE_MASK) >>
				DWC2_HCTSIZ_XFERSIZE_OFFSET;
			*toggle = (hctsiz & DWC2_HCTSIZ_PID_MASK) >>
				   DWC2_HCTSIZ_PID_OFFSET;

			/* Success: XFERCOMP, or CHHLTD without error */
			if (hcint & DWC2_HCINT_XFERCOMP) {
				writel(0x3fff, &hc_regs->hcint);
				return 0;
			}

			if (hcint & (DWC2_HCINT_NAK | DWC2_HCINT_FRMOVRUN |
				     DWC2_HCINT_XACTERR)) {
				dwc2_hc_cleanup(hc_regs);
				return -EAGAIN;
			}

			if (hcint & (DWC2_HCINT_STALL | DWC2_HCINT_AHBERR |
				     DWC2_HCINT_DATATGLERR)) {
				dwc2_hc_cleanup(hc_regs);
				printf("[USB dwc2] PIO fault HCINT=%08x\n",
				       hcint);
				return -EINVAL;
			}

			/*
			 * CHHLTD alone (or CHHLTD+ACK) without error bits:
			 * PIO slave-mode success for zero-length or
			 * completed transfers.
			 */
			writel(0x3fff, &hc_regs->hcint);
			return 0;
		}
		udelay(1);
	}

	/* Channel may be stuck — flush FIFOs as last-resort recovery */
	dwc2_flush_tx_fifo(regs);
	dwc2_hc_cleanup(hc_regs);
	printf("[USB dwc2] PIO timeout HCINT=%08x HCTSIZ=%08x HCCHAR=%08x\n",
	       readl(&hc_regs->hcint), readl(&hc_regs->hctsiz),
	       readl(&hc_regs->hcchar));
	return -ETIMEDOUT;
}

/*
 * PIO/slave-mode OUT transfer (follows DWC2 programming guide):
 *  1. Program HCTSIZ
 *  2. Clear HCINT
 *  3. Enable channel (HCCHAR CHEN) — controller requests FIFO data
 *  4. Write packet data to DFIFO
 *  5. Wait for CHHLTD
 */
static int transfer_chunk_pio_out(struct dwc2_core_regs *regs,
				  struct dwc2_hc_regs *hc_regs,
				  u8 *pid, void *buffer, int num_packets,
				  int xfer_len, int *actual_len, int odd_frame)
{
	uint32_t sub;
	int ret;

	writel((xfer_len << DWC2_HCTSIZ_XFERSIZE_OFFSET) |
	       (num_packets << DWC2_HCTSIZ_PKTCNT_OFFSET) |
	       (*pid << DWC2_HCTSIZ_PID_OFFSET),
	       &hc_regs->hctsiz);

	writel(0x3fff, &hc_regs->hcint);

	/* Step 1: enable channel — per DWC2 slave-mode spec, CHEN before FIFO write */
	dwc2_hc_enable(hc_regs, odd_frame);

	/* Step 2: write OUT data to TX FIFO (matches Linux dwc2_hc_write_packet) */
	if (xfer_len > 0) {
		ret = dwc2_pio_wait_tx_fifo(regs, xfer_len);
		if (ret)
			return ret;
		dwc2_pio_write_fifo(regs, DWC2_HC_CHANNEL, buffer, xfer_len);
	}

	ret = wait_for_pio_complete(regs, hc_regs, &sub, pid);
	if (ret < 0)
		return ret;

	*actual_len = xfer_len;
	return 0;
}

#if CONFIG_USB_DWC2_PIO_IN_DIAG
static unsigned int dwc2_pio_in_xfer_no;

static bool dwc2_pio_in_diag_on(int xfer_len, int num_packets)
{
	return xfer_len > 512 || num_packets > 1;
}

static void dwc2_pio_in_print_regs(const char *tag, unsigned int seq,
				   struct dwc2_core_regs *regs,
				   struct dwc2_hc_regs *hc_regs,
				   int received, int xfer_len,
				   int rx_pops_this_inner, int rx_guard_val)
{
	uint32_t gint = readl(&regs->gintsts);
	uint32_t hctsiz = readl(&hc_regs->hctsiz);
	uint32_t xfersz = (hctsiz & DWC2_HCTSIZ_XFERSIZE_MASK) >>
			  DWC2_HCTSIZ_XFERSIZE_OFFSET;
	uint32_t pktcnt = (hctsiz & DWC2_HCTSIZ_PKTCNT_MASK) >>
			  DWC2_HCTSIZ_PKTCNT_OFFSET;

	printf("[USB dwc2][PIO-IN] %s xfer#%u recv=%d/%d inner_pops=%d rx_guard=%d\n",
	       tag, seq, received, xfer_len, rx_pops_this_inner, rx_guard_val);
	printf("[USB dwc2][PIO-IN]   GINTSTS=0x%08x RXSTSQLVL=%d HCINTR=%d\n",
	       gint, !!(gint & DWC2_GINTSTS_RXSTSQLVL),
	       !!(gint & DWC2_GINTSTS_HCINTR));
	printf("[USB dwc2][PIO-IN]   HAINT=0x%08x HAINTMSK=0x%08x\n",
	       readl(&regs->host_regs.haint),
	       readl(&regs->host_regs.haintmsk));
	printf("[USB dwc2][PIO-IN]   HCINT=0x%08x HCTSIZ=0x%08x (xfersz_rem=%u pktcnt=%u) HCCHAR=0x%08x\n",
	       readl(&hc_regs->hcint), hctsiz, xfersz, pktcnt,
	       readl(&hc_regs->hcchar));
	printf("[USB dwc2][PIO-IN]   HCSPLT=0x%08x HCINTMSK=0x%08x\n",
	       readl(&hc_regs->hcsplt), readl(&hc_regs->hcintmsk));
}
#endif

/*
 * PIO/slave-mode IN transfer:
 *  1. Program HCTSIZ
 *  2. Clear HCINT, enable channel
 *  3. Poll GINTSTS.RXSTSQLVL for RX data, read GRXSTSP and DFIFO
 *  4. Loop until CHHLTD
 */
static int transfer_chunk_pio_in(struct dwc2_core_regs *regs,
				 struct dwc2_hc_regs *hc_regs,
				 u8 *pid, void *buffer, int num_packets,
				 int xfer_len, int *actual_len, int odd_frame)
{
	int received = 0;
	ulong start;
#if CONFIG_USB_DWC2_PIO_IN_DIAG
	bool diag = false;
	unsigned int seq;
#endif

#if CONFIG_USB_DWC2_PIO_IN_DIAG
	dwc2_pio_in_xfer_no++;
	seq = dwc2_pio_in_xfer_no;
	diag = dwc2_pio_in_diag_on(xfer_len, num_packets);
	if (diag) {
		printf("[USB dwc2][PIO-IN] start xfer#%u len=%d npkt=%d pid=%u odd=%d\n",
		       seq, xfer_len, num_packets, *pid, odd_frame);
	}
#endif

	writel((xfer_len << DWC2_HCTSIZ_XFERSIZE_OFFSET) |
	       (num_packets << DWC2_HCTSIZ_PKTCNT_OFFSET) |
	       (*pid << DWC2_HCTSIZ_PID_OFFSET),
	       &hc_regs->hctsiz);

	writel(0x3fff, &hc_regs->hcint);
	dwc2_hc_enable(hc_regs, odd_frame);

	start = get_timer(0);
	while (get_timer(start) < dwc2_pio_timeout_ms()) {
		/*
		 * Slave mode: drain the Rx status queue for every RXSTSQLVL
		 * pulse. One outer loop iteration is not enough — multiple
		 * packets (e.g. 16 KiB / 512) stack in GRXFIFO; each needs
		 * GRXSTSP pop + DFIFO read for IN_DATA. Status-only entries
		 * (IN_COMPLETE, CH_HALTED) must also be popped.
		 */
		{
		int rx_guard = 0;
#if CONFIG_USB_DWC2_PIO_IN_DIAG
		int rx_pops_inner = 0;
#endif

		while ((readl(&regs->gintsts) & DWC2_GINTSTS_RXSTSQLVL) &&
		       rx_guard++ < 512) {
			uint32_t grxstsp = readl(&regs->grxstsp);
			int ch = (grxstsp & DWC2_GRXSTS_EPNUM_MASK) >>
				 DWC2_GRXSTS_EPNUM_OFFSET;
			int bcnt = (grxstsp & DWC2_GRXSTS_BCNT_MASK) >>
				    DWC2_GRXSTS_BCNT_OFFSET;
			int pktsts = (grxstsp & DWC2_GRXSTS_PKTSTS_MASK) >>
				      DWC2_GRXSTS_PKTSTS_OFFSET;

#if CONFIG_USB_DWC2_PIO_IN_DIAG
			rx_pops_inner++;
			if (diag && rx_pops_inner <= DWC2_PIO_IN_POP_LOG_MAX) {
				printf("[USB dwc2][PIO-IN] xfer#%u pop#%d raw=0x%08x pktsts=%d bcnt=%d ch=%d cum_rx=%d\n",
				       seq, rx_pops_inner, grxstsp, pktsts,
				       bcnt, ch, received);
			}
#endif

			switch (pktsts) {
			case DWC2_GRXSTS_PKTSTS_IN_DATA:
				if (bcnt > 0) {
					/*
					 * Single host channel (HC 0) in this
					 * driver; if HW reports another ch,
					 * still pull bytes from that DFIFO.
					 */
					if (ch == DWC2_HC_CHANNEL) {
						if (received + bcnt > xfer_len)
							bcnt = xfer_len -
							       received;
						dwc2_pio_read_fifo(regs, ch,
								   (char *)buffer +
								   received,
								   bcnt);
						received += bcnt;
					} else {
						uint8_t drop[512];

						while (bcnt > 0) {
							int n = min(bcnt, 512);

							dwc2_pio_read_fifo(regs,
									   ch,
									   drop,
									   n);
							bcnt -= n;
						}
					}
				}
				break;
			case DWC2_GRXSTS_PKTSTS_IN_COMPLETE:
			case DWC2_GRXSTS_PKTSTS_CH_HALTED:
				/* Status entry; data length 0 */
				break;
			case DWC2_GRXSTS_PKTSTS_DT_ERROR:
				dwc2_hc_cleanup(hc_regs);
				*actual_len = received;
				return -EINVAL;
			default:
				break;
			}
		}
#if CONFIG_USB_DWC2_PIO_IN_DIAG
		if (diag) {
			uint32_t g = readl(&regs->gintsts);

			if (rx_guard >= 512 &&
			    (g & DWC2_GINTSTS_RXSTSQLVL))
				printf("[USB dwc2][PIO-IN] xfer#%u RXSTSQLVL still 1 after rx_guard=%d pops (fifo stuck?)\n",
				       seq, rx_guard - 1);
		}
#endif
		}

		{
		uint32_t hcint = readl(&hc_regs->hcint);

		if (hcint & DWC2_HCINT_PIO_DONE) {
			uint32_t hctsiz = readl(&hc_regs->hctsiz);

			*pid = (hctsiz & DWC2_HCTSIZ_PID_MASK) >>
				DWC2_HCTSIZ_PID_OFFSET;

			if (hcint & DWC2_HCINT_XFERCOMP) {
				writel(0x3fff, &hc_regs->hcint);
				*actual_len = received;
				return 0;
			}

			if (hcint & (DWC2_HCINT_NAK | DWC2_HCINT_FRMOVRUN |
				     DWC2_HCINT_XACTERR)) {
				dwc2_hc_cleanup(hc_regs);
				*actual_len = received;
				return -EAGAIN;
			}

			if (hcint & (DWC2_HCINT_STALL | DWC2_HCINT_AHBERR |
				     DWC2_HCINT_DATATGLERR)) {
				dwc2_hc_cleanup(hc_regs);
				printf("[USB dwc2] PIO IN fault HCINT=%08x\n",
				       hcint);
				*actual_len = received;
				return -EINVAL;
			}

			/* CHHLTD alone/+ACK: success in PIO slave mode */
			writel(0x3fff, &hc_regs->hcint);
			*actual_len = received;
			return 0;
		}
		}

		udelay(1);
	}

	dwc2_flush_rx_fifo(regs);
	dwc2_flush_tx_fifo(regs);
	dwc2_hc_cleanup(hc_regs);
	printf("[USB dwc2] PIO IN timeout HCINT=%08x HCTSIZ=%08x received=%d\n",
	       readl(&hc_regs->hcint), readl(&hc_regs->hctsiz), received);
#if CONFIG_USB_DWC2_PIO_IN_DIAG
	dwc2_pio_in_print_regs("TIMEOUT", seq, regs, hc_regs, received,
			       xfer_len, 0, 0);
#endif
	*actual_len = received;
	return -ETIMEDOUT;
}

/*
 * DMA-mode transfer (original path).
 */
static int transfer_chunk_dma(struct dwc2_hc_regs *hc_regs,
			      void *aligned_buffer,
			      u8 *pid, int in, void *buffer, int num_packets,
			      int xfer_len, int *actual_len, int odd_frame)
{
	int ret = 0;
	uint32_t sub;

	writel((xfer_len << DWC2_HCTSIZ_XFERSIZE_OFFSET) |
	       (num_packets << DWC2_HCTSIZ_PKTCNT_OFFSET) |
	       (*pid << DWC2_HCTSIZ_PID_OFFSET),
	       &hc_regs->hctsiz);

	if (xfer_len) {
		if (in) {
			invalidate_dcache_range(
					(uintptr_t)aligned_buffer,
					(uintptr_t)aligned_buffer +
					roundup(xfer_len, ARCH_DMA_MINALIGN));
		} else {
			memcpy(aligned_buffer, buffer, xfer_len);
			flush_dcache_range(
					(uintptr_t)aligned_buffer,
					(uintptr_t)aligned_buffer +
					roundup(xfer_len, ARCH_DMA_MINALIGN));
		}
	}

	writel(phys_to_bus((unsigned long)aligned_buffer), &hc_regs->hcdma);

	writel(0x3fff, &hc_regs->hcint);
	dwc2_hc_enable(hc_regs, odd_frame);

	ret = wait_for_chhltd(hc_regs, &sub, pid);
	if (ret < 0)
		return ret;

	if (in) {
		xfer_len -= sub;

		invalidate_dcache_range((unsigned long)aligned_buffer,
					(unsigned long)aligned_buffer +
					roundup(xfer_len, ARCH_DMA_MINALIGN));

		memcpy(buffer, aligned_buffer, xfer_len);
	}
	*actual_len = xfer_len;

	return ret;
}

static int transfer_chunk(struct dwc2_priv *priv,
			  struct dwc2_hc_regs *hc_regs, void *aligned_buffer,
			  u8 *pid, int in, void *buffer, int num_packets,
			  int xfer_len, int *actual_len, int odd_frame)
{
	debug("%s: chunk: pid %d xfer_len %u pkts %u dma=%d\n", __func__,
	      *pid, xfer_len, num_packets, priv->dma_enabled);

	if (priv->dma_enabled) {
		return transfer_chunk_dma(hc_regs, aligned_buffer, pid, in,
					  buffer, num_packets, xfer_len,
					  actual_len, odd_frame);
	}

	/* PIO/slave mode — use aligned_buffer as staging for word-aligned FIFO access */
	if (in) {
		int ret = transfer_chunk_pio_in(priv->regs, hc_regs, pid,
						aligned_buffer, num_packets,
						xfer_len, actual_len, odd_frame);
		if (ret == 0 || *actual_len > 0)
			memcpy(buffer, aligned_buffer, *actual_len);
		return ret;
	} else {
		if (xfer_len > 0)
			memcpy(aligned_buffer, buffer, xfer_len);
		return transfer_chunk_pio_out(priv->regs, hc_regs, pid,
					      aligned_buffer, num_packets,
					      xfer_len, actual_len, odd_frame);
	}
}

static void dwc2_prepare_transfer_params(struct dwc2_priv *priv, int len, int done,
					 int max, uint32_t max_xfer_len,
					 uint32_t *xfer_len, uint32_t *num_packets)
{
	*xfer_len = len - done;
	if (*xfer_len > max_xfer_len)
		*xfer_len = max_xfer_len;
	else if (*xfer_len > max)
		*num_packets = (*xfer_len + max - 1) / max;
	else
		*num_packets = 1;

	/*
	 * PIO/slave: split to one maxpacket chunk to avoid multi-packet
	 * channel stalls on CV184x.
	 */
	if (!priv->dma_enabled && max > 0 && *xfer_len > (uint32_t)max) {
		*xfer_len = max;
		*num_packets = 1;
	}
}

int chunk_msg(struct dwc2_priv *priv, struct usb_device *dev,
	      unsigned long pipe, u8 *pid, int in, void *buffer, int len)
{
	struct dwc2_core_regs *regs = priv->regs;
	struct dwc2_hc_regs *hc_regs = &regs->hc_regs[DWC2_HC_CHANNEL];
	struct dwc2_host_regs *host_regs = &regs->host_regs;
	int devnum = usb_pipedevice(pipe);
	int ep = usb_pipeendpoint(pipe);
	int max = usb_maxpacket(dev, pipe);
	int eptype = dwc2_eptype[usb_pipetype(pipe)];
	int done = 0;
	int ret = 0;
	int do_split = 0;
	int complete_split = 0;
	uint32_t xfer_len;
	uint32_t num_packets;
	int stop_transfer = 0;
	uint32_t max_xfer_len;
	int ssplit_frame_num = 0;

	debug("%s: msg: pipe %lx pid %d in %d len %d\n", __func__, pipe, *pid,
	      in, len);

	max_xfer_len = CONFIG_DWC2_MAX_PACKET_COUNT * max;
	if (max_xfer_len > CONFIG_DWC2_MAX_TRANSFER_SIZE)
		max_xfer_len = CONFIG_DWC2_MAX_TRANSFER_SIZE;
	if (max_xfer_len > DWC2_DATA_BUF_SIZE)
		max_xfer_len = DWC2_DATA_BUF_SIZE;

	/* Make sure that max_xfer_len is a multiple of max packet size. */
	num_packets = max_xfer_len / max;
	max_xfer_len = num_packets * max;

	/* Initialize channel */
	dwc_otg_hc_init(regs, DWC2_HC_CHANNEL, dev, devnum, ep, in,
			eptype, max);

	/* Check if the target is a FS/LS device behind a HS hub */
	if (dev->speed != USB_SPEED_HIGH) {
		uint8_t hub_addr;
		uint8_t hub_port;
		uint32_t hprt0 = readl(&regs->hprt0);
		if ((hprt0 & DWC2_HPRT0_PRTSPD_MASK) ==
		     DWC2_HPRT0_PRTSPD_HIGH) {
			usb_find_usb2_hub_address_port(dev, &hub_addr,
						       &hub_port);
			dwc_otg_hc_init_split(hc_regs, hub_addr, hub_port);

			do_split = 1;
			num_packets = 1;
			max_xfer_len = max;
		}
	}

	do {
		int actual_len = 0;
		uint32_t hcint;
		int odd_frame = 0;
		dwc2_prepare_transfer_params(priv, len, done, max, max_xfer_len,
					     &xfer_len, &num_packets);

#if CONFIG_USB_DWC2_PIO_IN_DIAG
		if (!priv->dma_enabled && in && xfer_len >= 512) {
			printf("[USB dwc2][chunk] PIO IN: len=%u pkts=%u total=%d done=%d dma=%d\n",
			       xfer_len, num_packets, len, done,
			       priv->dma_enabled);
		}
#endif

		if (complete_split)
			setbits_le32(&hc_regs->hcsplt, DWC2_HCSPLT_COMPSPLT);
		else if (do_split)
			clrbits_le32(&hc_regs->hcsplt, DWC2_HCSPLT_COMPSPLT);

		if (eptype == DWC2_HCCHAR_EPTYPE_INTR) {
			int uframe_num = readl(&host_regs->hfnum);
			if (!(uframe_num & 0x1))
				odd_frame = 1;
		}

#if CONFIG_USB_DWC2_VERBOSE_DEBUG
		{
		static int xfer_diag_cnt;
		if (xfer_diag_cnt < 10) {
				uint32_t hp = readl(&regs->hprt0);
				printf("[USB dwc2] xfer#%d(%s): HPRT0=0x%08x(conn=%d ena=%d "
				       "spd=%d pwr=%d) pid=%d len=%d pkts=%d %s\n",
				       xfer_diag_cnt,
				       priv->dma_enabled ? "DMA" : "PIO",
				       hp,
				       !!(hp & DWC2_HPRT0_PRTCONNSTS),
				       !!(hp & DWC2_HPRT0_PRTENA),
				       (hp & DWC2_HPRT0_PRTSPD_MASK) >> DWC2_HPRT0_PRTSPD_OFFSET,
				       !!(hp & DWC2_HPRT0_PRTPWR),
				       *pid, xfer_len, num_packets,
				       in ? "IN" : "OUT");
				xfer_diag_cnt++;
			}
		}
#endif

		ret = transfer_chunk(priv, hc_regs, priv->aligned_buffer, pid,
				     in, (char *)buffer + done, num_packets,
				     xfer_len, &actual_len, odd_frame);

		hcint = readl(&hc_regs->hcint);
		if (complete_split) {
			stop_transfer = 0;
			if (hcint & DWC2_HCINT_NYET) {
				ret = 0;
				int frame_num = DWC2_HFNUM_MAX_FRNUM &
						readl(&host_regs->hfnum);
				if (((frame_num - ssplit_frame_num) &
				    DWC2_HFNUM_MAX_FRNUM) > 4)
					ret = -EAGAIN;
			} else
				complete_split = 0;
		} else if (do_split) {
			if (hcint & DWC2_HCINT_ACK) {
				ssplit_frame_num = DWC2_HFNUM_MAX_FRNUM &
						   readl(&host_regs->hfnum);
				ret = 0;
				complete_split = 1;
			}
		}

		if (ret)
			break;

		if (actual_len < xfer_len)
			stop_transfer = 1;

		done += actual_len;

	/* Transactions are done when when either all data is transferred or
	 * there is a short transfer. In case of a SPLIT make sure the CSPLIT
	 * is executed.
	 */
	} while (((done < len) && !stop_transfer) || complete_split);

	dwc2_hc_cleanup(hc_regs);
	writel(0, &hc_regs->hcintmsk);

	dev->act_len = done;

	if (ret == -EAGAIN)
		dev->status = USB_ST_NAK_REC;
	else if (ret)
		dev->status = USB_ST_CRC_ERR;
	else
		dev->status = 0;

	return ret;
}

/* U-Boot USB transmission interface */
int _submit_bulk_msg(struct dwc2_priv *priv, struct usb_device *dev,
		     unsigned long pipe, void *buffer, int len)
{
	int devnum = usb_pipedevice(pipe);
	int ep = usb_pipeendpoint(pipe);
	u8 *pid;
	int ret;
	ulong t0;

	if ((devnum >= MAX_DEVICE) || (devnum == priv->root_hub_devnum)) {
		dev->status = 0;
		return -EINVAL;
	}

	if (usb_pipein(pipe))
		pid = &priv->in_data_toggle[devnum][ep];
	else
		pid = &priv->out_data_toggle[devnum][ep];

	/*
	 * NAK retry must not restart chunk_msg from buffer+0 with the same
	 * remaining length once bytes have been ACKed: *pid (DATA0/1) lives in
	 * priv and advances per successful packet, but chunk_msg always begins
	 * with done=0. Retrying the full len desynchronizes the data toggle and
	 * causes endless NAK (common after long PIO IN then first Bulk OUT).
	 * Mirror the control DATA stage: on partial -EAGAIN, advance ptr/remain.
	 */
	t0 = get_timer(0);
	{
		u8 *ptr = buffer;
		int remain = len;
		int total_done = 0;
		int dir_in = usb_pipein(pipe);

		do {
			int chunk_done;

			dev->act_len = 0;
			ret = chunk_msg(priv, dev, pipe, pid,
					dir_in, ptr, remain);
			chunk_done = dev->act_len;
			if (chunk_done < 0 || chunk_done > remain) {
				printf("[USB dwc2] bulk dev%d ep%d %s: invalid act_len=%d remain=%d\n",
				       devnum, ep, dir_in ? "IN" : "OUT",
				       chunk_done, remain);
				ret = -EINVAL;
				dev->act_len = total_done;
				break;
			}

			if (chunk_done > 0) {
				total_done += chunk_done;
				ptr += chunk_done;
				remain -= chunk_done;
			}

			if (ret == 0) {
				dev->act_len = total_done;
				break;
			}
			if (ret != -EAGAIN) {
				dev->act_len = total_done;
				break;
			}
			if (get_timer(t0) >= 5000) {
				dev->act_len = total_done;
				break;
			}
		} while (1);
	}

	if (ret == -EAGAIN)
		printf("[USB dwc2] bulk dev%d ep%d %s: NAK timeout after %lums\n",
		       devnum, ep, usb_pipein(pipe) ? "IN" : "OUT",
		       get_timer(t0));

	return ret;
}

static void dwc2_print_ctrl_setup(struct usb_device *udev, unsigned long pipe,
				  struct devrequest *setup)
{
#if CONFIG_USB_DWC2_VERBOSE_DEBUG
	printf("[USB dwc2] control dev=%d ep0 %s rq=0x%02x rt=0x%02x "
	       "v=%04x i=%04x l=%04x\n",
	       udev ? udev->devnum : -1,
	       usb_pipein(pipe) ? "in" : "out",
	       setup->request, setup->requesttype,
	       le16_to_cpu(setup->value), le16_to_cpu(setup->index),
	       le16_to_cpu(setup->length));
#else
	(void)udev;
	(void)pipe;
	(void)setup;
#endif
}

static int _submit_control_msg(struct dwc2_priv *priv, struct usb_device *dev,
			       unsigned long pipe, void *buffer, int len,
			       struct devrequest *setup)
{
	int devnum = usb_pipedevice(pipe);
	int ret, act_len;
	u8 pid;
	/* For CONTROL endpoint pid should start with DATA1 */
	int status_direction;

	if (devnum == priv->root_hub_devnum) {
		dev->status = 0;
		dev->speed = USB_SPEED_HIGH;
		return dwc_otg_submit_rh_msg(priv, dev, pipe, buffer, len,
					     setup);
	}

	/* SETUP stage */
	pid = DWC2_HC_PID_SETUP;
	{
		ulong t0 = get_timer(0);
		do {
			ret = chunk_msg(priv, dev, pipe, &pid, 0, setup, 8);
		} while (ret == -EAGAIN && get_timer(t0) < 5000);
	}
	if (ret) {
		dwc2_print_ctrl_setup(dev, pipe, setup);
		printf("[USB dwc2] control failed at SETUP stage ret=%d\n", ret);
		return ret;
	}

	/* DATA stage */
	act_len = 0;
	if (buffer) {
		pid = DWC2_HC_PID_DATA1;
		{
			ulong t0 = get_timer(0);
			do {
				ret = chunk_msg(priv, dev, pipe, &pid,
						usb_pipein(pipe),
						buffer, len);
				act_len += dev->act_len;
				buffer += dev->act_len;
				len -= dev->act_len;
			} while (ret == -EAGAIN && get_timer(t0) < 5000);
		}
		if (ret) {
			dwc2_print_ctrl_setup(dev, pipe, setup);
			printf("[USB dwc2] control failed at DATA stage ret=%d\n",
			       ret);
			return ret;
		}
		status_direction = usb_pipeout(pipe);
	} else {
		/* No-data CONTROL always ends with an IN transaction */
		status_direction = 1;
	}

	/* STATUS stage */
	pid = DWC2_HC_PID_DATA1;
	{
		ulong t0 = get_timer(0);
		do {
			ret = chunk_msg(priv, dev, pipe, &pid,
					status_direction,
					priv->status_buffer, 0);
		} while (ret == -EAGAIN && get_timer(t0) < 5000);
	}
	if (ret) {
		dwc2_print_ctrl_setup(dev, pipe, setup);
		printf("[USB dwc2] control failed at STATUS stage ret=%d "
		       "(status %s)\n",
		       ret, status_direction ? "IN" : "OUT");
		return ret;
	}

	dev->act_len = act_len;

	/*
	 * After a successful CLEAR_FEATURE(ENDPOINT_HALT), the device
	 * resets its data toggle to DATA0.  Sync DWC2's private toggle
	 * so the next bulk transfer uses the correct PID.
	 */
	if (setup->request == USB_REQ_CLEAR_FEATURE &&
	    (setup->requesttype & USB_RECIP_MASK) == USB_RECIP_ENDPOINT &&
	    le16_to_cpu(setup->value) == 0) {
		int ep_addr = le16_to_cpu(setup->index);
		int ep_num = ep_addr & 0x0f;

		if (ep_addr & USB_DIR_IN)
			priv->in_data_toggle[devnum][ep_num] = DWC2_HC_PID_DATA0;
		else
			priv->out_data_toggle[devnum][ep_num] = DWC2_HC_PID_DATA0;
		debug("[USB dwc2] CLEAR_HALT dev%d ep%d %s: toggle -> DATA0\n",
		      devnum, ep_num,
		      (ep_addr & USB_DIR_IN) ? "IN" : "OUT");
	}

	return 0;
}

int _submit_int_msg(struct dwc2_priv *priv, struct usb_device *dev,
		    unsigned long pipe, void *buffer, int len, int interval,
		    bool nonblock)
{
	unsigned long timeout;
	int ret;

	/* FIXME: what is interval? */

	timeout = get_timer(0) + USB_TIMEOUT_MS(pipe);
	for (;;) {
		if (get_timer(0) > timeout) {
#if CONFIG_IS_ENABLED(DM_USB)
			dev_err(dev->dev,
				"Timeout poll on interrupt endpoint\n");
#else
			log_err("Timeout poll on interrupt endpoint\n");
#endif
			return -ETIMEDOUT;
		}
		ret = _submit_bulk_msg(priv, dev, pipe, buffer, len);
		if ((ret != -EAGAIN) || nonblock)
			return ret;
	}
}

static int dwc2_reset(struct udevice *dev)
{
	int ret;
	struct dwc2_priv *priv = dev_get_priv(dev);

	ret = reset_get_bulk(dev, &priv->resets);
	if (ret) {
		dev_warn(dev, "Can't get reset: %d\n", ret);
		/* Return 0 if error due to !CONFIG_DM_RESET and reset
		 * DT property is not present.
		 */
		if (ret == -ENOENT || ret == -ENOTSUPP)
			return 0;
		else
			return ret;
	}

	/* force reset to clear all IP register */
	reset_assert_bulk(&priv->resets);
	ret = reset_deassert_bulk(&priv->resets);
	if (ret) {
		reset_release_bulk(&priv->resets);
		dev_err(dev, "Failed to reset: %d\n", ret);
		return ret;
	}

	return 0;
}

static int dwc2_init_common(struct udevice *dev, struct dwc2_priv *priv)
{
	struct dwc2_core_regs *regs = priv->regs;
	uint32_t snpsid;
	uint32_t devid;
	int i, j;
	int ret;

	ret = dwc2_reset(dev);
	if (ret) {
		DWC2_ERR("dwc2_reset(%s) failed: err=%d\n",
			 dev->name, ret);
		return ret;
	}

	snpsid = readl(&regs->gsnpsid);
	devid = snpsid & DWC2_SNPSID_DEVID_MASK;
	DWC2_INFO("%s regs=%p GSNPSID=0x%08x devid=0x%x (expect 2xx=0x%x or 3xx=0x%x or 4xx=0x%x)\n",
		  dev->name, regs, snpsid, devid,
		  DWC2_SNPSID_DEVID_VER_2xx, DWC2_SNPSID_DEVID_VER_3xx,
		  DWC2_SNPSID_DEVID_VER_4xx);
	dev_info(dev, "Core Release: %x.%03x\n",
		 snpsid >> 12 & 0xf, snpsid & 0xfff);

	if (devid != DWC2_SNPSID_DEVID_VER_2xx &&
	    devid != DWC2_SNPSID_DEVID_VER_3xx &&
	    devid != DWC2_SNPSID_DEVID_VER_4xx) {
		DWC2_ERR("SNPSID mismatch -> -ENODEV (clocks/reset/phys addr?)\n");
		dev_info(dev, "SNPSID invalid (not DWC2 OTG device): %08x\n",
			 snpsid);
		return -ENODEV;
	}

#ifdef CONFIG_DWC2_PHY_ULPI_EXT_VBUS
	priv->ext_vbus = 1;
#else
	priv->ext_vbus = 0;
#endif

	dwc_otg_core_init(dev);

	if (usb_get_dr_mode(dev_ofnode(dev)) == USB_DR_MODE_PERIPHERAL) {
		DWC2_INFO("%s: dr_mode=peripheral, skip host core init\n",
			  dev->name);
		dev_dbg(dev, "USB device %s dr_mode set to %d. Skipping host_init.\n",
			dev->name, usb_get_dr_mode(dev_ofnode(dev)));
	} else {
		dwc_otg_core_host_init(dev, regs);
	}

	clrsetbits_le32(&regs->hprt0, DWC2_HPRT0_PRTENA |
			DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG |
			DWC2_HPRT0_PRTOVRCURRCHNG,
			DWC2_HPRT0_PRTRST);
	mdelay(50);
	clrbits_le32(&regs->hprt0, DWC2_HPRT0_PRTENA | DWC2_HPRT0_PRTCONNDET |
		     DWC2_HPRT0_PRTENCHNG | DWC2_HPRT0_PRTOVRCURRCHNG |
		     DWC2_HPRT0_PRTRST);

	for (i = 0; i < MAX_DEVICE; i++) {
		for (j = 0; j < MAX_ENDPOINT; j++) {
			priv->in_data_toggle[i][j] = DWC2_HC_PID_DATA0;
			priv->out_data_toggle[i][j] = DWC2_HC_PID_DATA0;
		}
	}

	/*
	 * Add a 1 second delay here. This gives the host controller
	 * a bit time before the comminucation with the USB devices
	 * is started (the bus is scanned) and  fixes the USB detection
	 * problems with some problematic USB keys.
	 */
	if (readl(&regs->gintsts) & DWC2_GINTSTS_CURMODE_HOST)
		mdelay(1000);

#if CONFIG_USB_DWC2_VERBOSE_DEBUG
	{
		uint32_t hprt0_val = readl(&regs->hprt0);
		uint32_t gahbcfg_val = readl(&regs->gahbcfg);
		uint32_t gintsts_val = readl(&regs->gintsts);
		uint32_t hfnum1 = readl(&regs->host_regs.hfnum);
		udelay(125);
		uint32_t hfnum2 = readl(&regs->host_regs.hfnum);
		DWC2_INFO("post-init HPRT0=0x%08x (conn=%d ena=%d pwr=%d spd=%d)\n",
			  hprt0_val,
			  !!(hprt0_val & DWC2_HPRT0_PRTCONNSTS),
			  !!(hprt0_val & DWC2_HPRT0_PRTENA),
			  !!(hprt0_val & DWC2_HPRT0_PRTPWR),
			  (hprt0_val & DWC2_HPRT0_PRTSPD_MASK) >>
				DWC2_HPRT0_PRTSPD_OFFSET);
		DWC2_INFO(" HFNUM=%08x->%08x(%s) HCFG=%08x "
			  "PCGCCTL=%08x GAHBCFG=%08x GINTSTS=%08x\n",
			  hfnum1, hfnum2,
			  (hfnum1 != hfnum2) ? "running" : "STUCK",
			  readl(&regs->host_regs.hcfg),
			  readl(&regs->pcgcctl),
			  gahbcfg_val, gintsts_val);
		DWC2_INFO(" GRXFSIZ=%08x GNPTXFSIZ=%08x GNPTXSTS=%08x\n",
			  readl(&regs->grxfsiz),
			  readl(&regs->gnptxfsiz),
			  readl(&regs->gnptxsts));
	}
#endif

	printf("USB DWC2\n");

	return 0;
}

static void dwc2_uninit_common(struct dwc2_core_regs *regs)
{
	/* Put everything in reset. */
	clrsetbits_le32(&regs->hprt0, DWC2_HPRT0_PRTENA |
			DWC2_HPRT0_PRTCONNDET | DWC2_HPRT0_PRTENCHNG |
			DWC2_HPRT0_PRTOVRCURRCHNG,
			DWC2_HPRT0_PRTRST);
}

#if !CONFIG_IS_ENABLED(DM_USB)
int submit_control_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
		       int len, struct devrequest *setup)
{
	return _submit_control_msg(&local, dev, pipe, buffer, len, setup);
}

int submit_bulk_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
		    int len)
{
	return _submit_bulk_msg(&local, dev, pipe, buffer, len);
}

int submit_int_msg(struct usb_device *dev, unsigned long pipe, void *buffer,
		   int len, int interval, bool nonblock)
{
	return _submit_int_msg(&local, dev, pipe, buffer, len, interval,
			       nonblock);
}

/* U-Boot USB control interface */
int usb_lowlevel_init(int index, enum usb_init_type init, void **controller)
{
	struct dwc2_priv *priv = &local;

	memset(priv, '\0', sizeof(*priv));
	priv->root_hub_devnum = 0;
	priv->regs = (struct dwc2_core_regs *)CONFIG_USB_DWC2_REG_ADDR;
	priv->aligned_buffer = aligned_buffer_addr;
	priv->status_buffer = status_buffer_addr;

	/* board-dependant init */
	if (board_usb_init(index, USB_INIT_HOST))
		return -1;

	return dwc2_init_common(NULL, priv);
}

int usb_lowlevel_stop(int index)
{
	dwc2_uninit_common(local.regs);

	return 0;
}
#endif

#if CONFIG_IS_ENABLED(DM_USB)
static int dwc2_submit_control_msg(struct udevice *dev, struct usb_device *udev,
				   unsigned long pipe, void *buffer, int length,
				   struct devrequest *setup)
{
	struct dwc2_priv *priv = dev_get_priv(dev);

	debug("%s: dev='%s', udev=%p, udev->dev='%s', portnr=%d\n", __func__,
	      dev->name, udev, udev->dev->name, udev->portnr);

	return _submit_control_msg(priv, udev, pipe, buffer, length, setup);
}

static int dwc2_submit_bulk_msg(struct udevice *dev, struct usb_device *udev,
				unsigned long pipe, void *buffer, int length)
{
	struct dwc2_priv *priv = dev_get_priv(dev);

	debug("%s: dev='%s', udev=%p\n", __func__, dev->name, udev);

	return _submit_bulk_msg(priv, udev, pipe, buffer, length);
}

static int dwc2_submit_int_msg(struct udevice *dev, struct usb_device *udev,
			       unsigned long pipe, void *buffer, int length,
			       int interval, bool nonblock)
{
	struct dwc2_priv *priv = dev_get_priv(dev);

	debug("%s: dev='%s', udev=%p\n", __func__, dev->name, udev);

	return _submit_int_msg(priv, udev, pipe, buffer, length, interval,
			       nonblock);
}

static int dwc2_usb_of_to_plat(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);

	priv->regs = dev_read_addr_ptr(dev);
	if (!priv->regs) {
		DWC2_ERR("%s: dev_read_addr_ptr(reg) NULL -> -EINVAL\n",
			 dev->name);
		return -EINVAL;
	}
	DWC2_INFO("%s: MMIO base %p (from DT reg)\n", dev->name,
		  priv->regs);

	priv->oc_disable = dev_read_bool(dev, "disable-over-current");
	priv->hnp_srp_disable = dev_read_bool(dev, "hnp-srp-disable");

	return 0;
}

static int dwc2_setup_phy(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);
	int ret;

	ret = generic_phy_get_by_index(dev, 0, &priv->phy);
	if (ret) {
		if (ret == -ENOENT)
			return 0; /* no PHY, nothing to do */
		DWC2_ERR("%s: generic_phy_get err=%d\n", dev->name, ret);
		dev_err(dev, "Failed to get USB PHY: %d.\n", ret);
		return ret;
	}

	ret = generic_phy_init(&priv->phy);
	if (ret) {
		DWC2_ERR("%s: generic_phy_init err=%d\n",
			 dev->name, ret);
		dev_dbg(dev, "Failed to init USB PHY: %d.\n", ret);
		return ret;
	}

	ret = generic_phy_power_on(&priv->phy);
	if (ret) {
		DWC2_ERR("%s: generic_phy_power_on err=%d\n",
			 dev->name, ret);
		dev_dbg(dev, "Failed to power on USB PHY: %d.\n", ret);
		generic_phy_exit(&priv->phy);
		return ret;
	}

	return 0;
}

static int dwc2_shutdown_phy(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);
	int ret;

	/* PHY is not valid when generic_phy_get_by_index() = -ENOENT */
	if (!generic_phy_valid(&priv->phy))
		return 0; /* no PHY, nothing to do */

	ret = generic_phy_power_off(&priv->phy);
	if (ret) {
		dev_dbg(dev, "Failed to power off USB PHY: %d.\n", ret);
		return ret;
	}

	ret = generic_phy_exit(&priv->phy);
	if (ret) {
		dev_dbg(dev, "Failed to power off USB PHY: %d.\n", ret);
		return ret;
	}

	return 0;
}

static int dwc2_clk_init(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);
	int ret;

	ret = clk_get_bulk(dev, &priv->clks);
	if (ret == -ENOSYS || ret == -ENOENT) {
		DWC2_INFO("%s: clk_get_bulk not used (err=%d %s); clocks rely on board init\n",
			  dev->name, ret, ret == -ENOSYS ? "ENOSYS" : "ENOENT");
		return 0;
	}
	if (ret) {
		DWC2_ERR("%s: clk_get_bulk failed err=%d\n",
			 dev->name, ret);
		return ret;
	}

	ret = clk_enable_bulk(&priv->clks);
	if (ret) {
		DWC2_ERR("%s: clk_enable_bulk failed err=%d\n",
			 dev->name, ret);
		clk_release_bulk(&priv->clks);
		return ret;
	}

	DWC2_INFO("%s: clocks enabled via clk bulk\n", dev->name);
	return 0;
}

/*
 * SoC-specific hooks (optional): reset/USB PHY / pinmux before the DWC2
 * register programming in dwc2_init_common(). CV184X implements this in
 * board/cvitek/cv184x/board.c to match Linux dwc2 platform code.
 */
__weak int dwc2_board_usb_init(struct udevice *dev)
{
	return 0;
}

static int dwc2_usb_probe(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);
	struct usb_bus_priv *bus_priv = dev_get_uclass_priv(dev);
	int ret;

	bus_priv->desc_before_addr = true;

	DWC2_INFO("probe %s: -> dwc2_clk_init\n", dev->name);
	ret = dwc2_clk_init(dev);
	if (ret) {
		DWC2_ERR("probe %s: exit err=%d at clk_init\n",
			 dev->name, ret);
		return ret;
	}

	DWC2_INFO("probe %s: -> dwc2_board_usb_init (SoC hooks)\n",
		  dev->name);
	ret = dwc2_board_usb_init(dev);
	if (ret) {
		DWC2_ERR("probe %s: exit err=%d at board_usb_init\n",
			 dev->name, ret);
		return ret;
	}

	DWC2_INFO("probe %s: -> dwc2_setup_phy\n", dev->name);
	ret = dwc2_setup_phy(dev);
	if (ret) {
		DWC2_ERR("probe %s: exit err=%d at setup_phy\n",
			 dev->name, ret);
		return ret;
	}

	DWC2_INFO("probe %s: -> dwc2_init_common\n", dev->name);
	ret = dwc2_init_common(dev, priv);
	if (ret)
		DWC2_ERR("probe %s: exit err=%d at init_common\n",
			 dev->name, ret);
	return ret;
}

static int dwc2_usb_remove(struct udevice *dev)
{
	struct dwc2_priv *priv = dev_get_priv(dev);
	int ret;

	ret = dwc_vbus_supply_exit(dev);
	if (ret)
		return ret;

	ret = dwc2_shutdown_phy(dev);
	if (ret) {
		dev_dbg(dev, "Failed to shutdown USB PHY: %d.\n", ret);
		return ret;
	}

	dwc2_uninit_common(priv->regs);

	reset_release_bulk(&priv->resets);
	clk_disable_bulk(&priv->clks);
	clk_release_bulk(&priv->clks);

	return 0;
}

struct dm_usb_ops dwc2_usb_ops = {
	.control = dwc2_submit_control_msg,
	.bulk = dwc2_submit_bulk_msg,
	.interrupt = dwc2_submit_int_msg,
};

static const struct udevice_id dwc2_usb_ids[] = {
	{ .compatible = "brcm,bcm2835-usb" },
	{ .compatible = "brcm,bcm2708-usb" },
	{ .compatible = "snps,dwc2" },
	{ .compatible = "cvitek,cv182x-usb" },
	{ }
};

U_BOOT_DRIVER(usb_dwc2) = {
	.name	= "dwc2_usb",
	.id	= UCLASS_USB,
	.of_match = dwc2_usb_ids,
	.of_to_plat = dwc2_usb_of_to_plat,
	.probe	= dwc2_usb_probe,
	.remove = dwc2_usb_remove,
	.ops	= &dwc2_usb_ops,
	.priv_auto	= sizeof(struct dwc2_priv),
	.flags	= DM_FLAG_ALLOC_PRIV_DMA,
};
#endif
