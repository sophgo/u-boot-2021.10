// SPDX-License-Identifier: GPL-2.0
/**
 * core.c - DesignWare USB3 DRD Controller Core file
 *
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com
 *
 * Authors: Felipe Balbi <balbi@ti.com>,
 *	    Sebastian Andrzej Siewior <bigeasy@linutronix.de>
 *
 * Taken from Linux Kernel v3.19-rc1 (drivers/usb/dwc3/core.c) and ported
 * to uboot.
 *
 * commit cd72f890d2 : usb: dwc3: core: enable phy suspend quirk on non-FPGA
 */

#include <common.h>
#include <cpu_func.h>
#include <malloc.h>
#include <dwc3-uboot.h>
// #include <linux/bug.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/ioport.h>
#include <dm.h>
#include <generic-phy.h>
#include <linux/usb/ch9.h>
#include "include/gadget.h"
#include "include/debug.h"
#include "include/cvi_usb.h"
#include "core.h"
#include "gadget.h"
#include "io.h"

#include "linux-compat.h"

uint8_t __attribute__((aligned(64))) _DWC_BASE[DWC_SIZE];
uint8_t __attribute__((aligned(64))) _EVENT_BUF_BASE[EVENT_BUF_SIZE];
uint8_t __attribute__((aligned(64))) _EVT_BUF_BASE[EVT_BUF_SIZE];

#define USB_STATIC_MEM 0

static LIST_HEAD(dwc3_list);

static void dwc3_set_mode(struct dwc3 *dwc, u32 mode)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~(DWC3_GCTL_PRTCAPDIR(DWC3_GCTL_PRTCAP_OTG));
	reg |= DWC3_GCTL_PRTCAPDIR(mode);
	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

/**
 * dwc3_core_soft_reset - Issues core soft reset and PHY reset
 * @dwc: pointer to our context structure
 */
static int dwc3_core_soft_reset(struct dwc3 *dwc)
{
	u32		reg;

	int		retries = 1000;

	reg = dwc3_readl(dwc->regs, DWC3_DCTL);
	reg |= DWC3_DCTL_CSFTRST;
	reg &= ~DWC3_DCTL_RUN_STOP;
	reg &= ~DWC3_DCTL_ULSTCHNGREQ_MASK;
	dwc3_writel(dwc->regs, DWC3_DCTL, reg);

	/*
	 * For DWC_usb31 controller 1.90a and later, the DCTL.CSFRST bit
	 * is cleared only after all the clocks are synchronized. This can
	 * take a little more than 50ms. Set the polling rate at 20ms
	 * for 10 times instead.
	 */
	if (DWC3_VER_IS_WITHIN(DWC31, 190A, ANY) || DWC3_IP_IS(DWC32))
		retries = 10;

	do {
		reg = dwc3_readl(dwc->regs, DWC3_DCTL);
		if (!(reg & DWC3_DCTL_CSFTRST))
			goto done;

		if (DWC3_VER_IS_WITHIN(DWC31, 190A, ANY) || DWC3_IP_IS(DWC32))
			mdelay(20);
		else
			udelay(1);
	} while (--retries);

	ERROR("DWC3 controller soft reset failed.\n");
	return -ETIMEDOUT;

done:
	/*
	 * For DWC_usb31 controller 1.80a and prior, once DCTL.CSFRST bit
	 * is cleared, we must wait at least 50ms before accessing the PHY
	 * domain (synchronization delay).
	 */
	if (DWC3_VER_IS_WITHIN(DWC31, ANY, 180A))
		mdelay(50);

	return 0;
}

/**
 * dwc3_free_one_event_buffer - Frees one event buffer
 * @dwc: Pointer to our controller context structure
 * @evt: Pointer to event buffer to be freed
 */
static void dwc3_free_one_event_buffer(struct dwc3 *dwc,
		struct dwc3_event_buffer *evt)
{
#if USB_STATIC_MEM
	memset(evt->buf, 0, EVT_BUF_SIZE);
#else
	dma_free_coherent(evt->buf);
#endif
}

#if USB_STATIC_MEM
static struct dwc3_event_buffer g_evt;
#endif

/**
 * dwc3_alloc_one_event_buffer - Allocates one event buffer structure
 * @dwc: Pointer to our controller context structure
 * @length: size of the event buffer
 *
 * Returns a pointer to the allocated event buffer structure on success
 * otherwise ERR_PTR(errno).
 */
static struct dwc3_event_buffer *dwc3_alloc_one_event_buffer(struct dwc3 *dwc,
		unsigned length)
{
	struct dwc3_event_buffer	*evt;

#if USB_STATIC_MEM
	evt = &g_evt;
#else
	evt = kzalloc(sizeof(*evt), 0);
	if (!evt)
		return ERR_PTR(-ENOMEM);
#endif

	evt->dwc	= dwc;
	evt->length	= length;

#if USB_STATIC_MEM
	evt->dma = (dma_addr_t)EVT_BUF_BASE;
	evt->buf = (uint8_t *)EVT_BUF_BASE;
#else
	evt->buf	= dma_alloc_coherent(length,
					     (unsigned long *)&evt->dma);
	if (!evt->buf)
		return ERR_PTR(-ENOMEM);
#endif

	dwc3_flush_cache((uintptr_t)evt->buf, evt->length);

	return evt;
}

/**
 * dwc3_free_event_buffers - frees all allocated event buffers
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_free_event_buffers(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int i;

	for (i = 0; i < dwc->num_event_buffers; i++) {
		evt = dwc->ev_buffs[i];
		if (evt)
			dwc3_free_one_event_buffer(dwc, evt);
	}
}

/**
 * dwc3_alloc_event_buffers - Allocates @num event buffers of size @length
 * @dwc: pointer to our controller context structure
 * @length: size of event buffer
 *
 * Returns 0 on success otherwise negative errno. In the error case, dwc
 * may contain some buffers allocated but not all which were requested.
 */
static int dwc3_alloc_event_buffers(struct dwc3 *dwc, unsigned length)
{
	int			num;
	int			i;
	num = DWC31_NUM_INT(dwc->hwparams.hwparams1);
	dwc->num_event_buffers = num;

#if USB_STATIC_MEM
	dwc->ev_buffs = (struct dwc3_event_buffer **)EVENT_BUF_BASE;
#else
	dwc->ev_buffs = memalign(CONFIG_SYS_CACHELINE_SIZE,
				 sizeof(*dwc->ev_buffs) * num);
	if (!dwc->ev_buffs)
		return -ENOMEM;
#endif

	for (i = 0; i < num; i++) {
		struct dwc3_event_buffer	*evt;

		evt = dwc3_alloc_one_event_buffer(dwc, length);
#if !USB_STATIC_MEM
		if (IS_ERR(evt)) {
			ERROR("can't allocate event buffer\n");
			return PTR_ERR(evt);
		}
#endif
		dwc->ev_buffs[i] = evt;
	}

	return 0;
}

/**
 * dwc3_event_buffers_setup - setup our allocated event buffers
 * @dwc: pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_event_buffers_setup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n),
				lower_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n),
				upper_32_bits(evt->dma));
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n),
				DWC3_GEVNTSIZ_SIZE(evt->length));
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}

	return 0;
}

static void dwc3_event_buffers_cleanup(struct dwc3 *dwc)
{
	struct dwc3_event_buffer	*evt;
	int				n;

	for (n = 0; n < dwc->num_event_buffers; n++) {
		evt = dwc->ev_buffs[n];

		evt->lpos = 0;

		dwc3_writel(dwc->regs, DWC3_GEVNTADRLO(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTADRHI(n), 0);
		dwc3_writel(dwc->regs, DWC3_GEVNTSIZ(n), DWC3_GEVNTSIZ_INTMASK
				| DWC3_GEVNTSIZ_SIZE(0));
		dwc3_writel(dwc->regs, DWC3_GEVNTCOUNT(n), 0);
	}
}

static int dwc3_alloc_scratch_buffers(struct dwc3 *dwc)
{
	if (!dwc->has_hibernation)
		return 0;

	if (!dwc->nr_scratch)
		return 0;

#if USB_STATIC_MEM
	dwc->scratchbuf = NULL;
#else
	dwc->scratchbuf = kmalloc_array(dwc->nr_scratch,
			DWC3_SCRATCHBUF_SIZE, GFP_KERNEL);
#endif
	if (!dwc->scratchbuf)
		return -ENOMEM;

	return 0;
}

static int dwc3_setup_scratch_buffers(struct dwc3 *dwc)
{
	dma_addr_t scratch_addr;
	u32 param;
	int ret;

	if (!dwc->has_hibernation)
		return 0;

	if (!dwc->nr_scratch)
		return 0;

	scratch_addr = dma_map_single(dwc->scratchbuf,
				      dwc->nr_scratch * DWC3_SCRATCHBUF_SIZE,
				      DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dwc->dev, scratch_addr)) {
		ERROR("failed to map scratch buffer\n");
		ret = -EFAULT;
		goto err0;
	}

	dwc->scratch_addr = scratch_addr;

	param = lower_32_bits(scratch_addr);

	ret = dwc3_send_gadget_generic_command(dwc,
			DWC3_DGCMD_SET_SCRATCHPAD_ADDR_LO, param);
	if (ret < 0)
		goto err1;

	param = upper_32_bits(scratch_addr);

	ret = dwc3_send_gadget_generic_command(dwc,
			DWC3_DGCMD_SET_SCRATCHPAD_ADDR_HI, param);
	if (ret < 0)
		goto err1;

	return 0;

err1:
	dma_unmap_single(scratch_addr, dwc->nr_scratch * DWC3_SCRATCHBUF_SIZE,
			 DMA_BIDIRECTIONAL);

err0:
	return ret;
}

static void dwc3_free_scratch_buffers(struct dwc3 *dwc)
{
	if (!dwc->has_hibernation)
		return;

	if (!dwc->nr_scratch)
		return;

	dma_unmap_single(dwc->scratch_addr, dwc->nr_scratch *
			 DWC3_SCRATCHBUF_SIZE, DMA_BIDIRECTIONAL);
#if !USB_STATIC_MEM
	kfree(dwc->scratchbuf);
#endif
}

static void dwc3_core_num_eps(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	dwc->num_in_eps = DWC3_NUM_IN_EPS(parms);
	dwc->num_out_eps = DWC3_NUM_EPS(parms) - dwc->num_in_eps;

}

static void dwc3_cache_hwparams(struct dwc3 *dwc)
{
	struct dwc3_hwparams	*parms = &dwc->hwparams;

	parms->hwparams0 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS0);
	parms->hwparams1 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS1);
	parms->hwparams2 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS2);
	parms->hwparams3 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS3);
	parms->hwparams4 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS4);
	parms->hwparams5 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS5);
	parms->hwparams6 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS6);
	parms->hwparams7 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS7);
	parms->hwparams8 = dwc3_readl(dwc->regs, DWC3_GHWPARAMS8);
}

/**
 * dwc3_phy_setup - Configure USB PHY Interface of DWC3 Core
 * @dwc: Pointer to our controller context structure
 */
static void dwc3_phy_setup(struct dwc3 *dwc)
{
	unsigned int hw_mode;
	u32 reg;

	hw_mode = DWC3_GHWPARAMS0_MODE(dwc->hwparams.hwparams0);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));

	/*
	 * Make sure UX_EXIT_PX is cleared as that causes issues with some
	 * PHYs. Also, this bit is not supposed to be used in normal operation.
	 */
	reg &= ~DWC3_GUSB3PIPECTL_UX_EXIT_PX;

	/*
	 * Above 1.94a, it is recommended to set DWC3_GUSB3PIPECTL_SUSPHY
	 * to '0' during coreConsultant configuration. So default value
	 * will be '0' when the core is reset. Application needs to set it
	 * to '1' after the core initialization is completed.
	 */
	if (!DWC3_VER_IS_WITHIN(DWC3, ANY, 194A))
		reg |= DWC3_GUSB3PIPECTL_SUSPHY;

	/*
	 * For DRD controllers, GUSB3PIPECTL.SUSPENDENABLE must be cleared after
	 * power-on reset, and it can be set after core initialization, which is
	 * after device soft-reset during initialization.
	 */
	if (hw_mode == DWC3_GHWPARAMS0_MODE_DRD)
		reg &= ~DWC3_GUSB3PIPECTL_SUSPHY;

	if (dwc->u2ss_inp3_quirk)
		reg |= DWC3_GUSB3PIPECTL_U2SSINP3OK;

	if (dwc->req_p1p2p3_quirk)
		reg |= DWC3_GUSB3PIPECTL_REQP1P2P3;

	if (dwc->del_p1p2p3_quirk)
		reg |= DWC3_GUSB3PIPECTL_DEP1P2P3_EN;

	if (dwc->del_phy_power_chg_quirk)
		reg |= DWC3_GUSB3PIPECTL_DEPOCHANGE;

	if (dwc->lfps_filter_quirk)
		reg |= DWC3_GUSB3PIPECTL_LFPSFILT;

	if (dwc->rx_detect_poll_quirk)
		reg |= DWC3_GUSB3PIPECTL_RX_DETOPOLL;

	if (dwc->tx_de_emphasis_quirk)
		reg |= DWC3_GUSB3PIPECTL_TX_DEEPH(dwc->tx_de_emphasis);

	if (dwc->dis_u3_susphy_quirk)
		reg &= ~DWC3_GUSB3PIPECTL_SUSPHY;

	if (dwc->dis_del_phy_power_chg_quirk)
		reg &= ~DWC3_GUSB3PIPECTL_DEPOCHANGE;

	dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);

	reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));

	switch (dwc->hsphy_mode) {
	case USBPHY_INTERFACE_MODE_UTMI:
		reg &= ~(DWC3_GUSB2PHYCFG_PHYIF_MASK |
			DWC3_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_8_BIT) |
			DWC3_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_8_BIT);
		break;
	case USBPHY_INTERFACE_MODE_UTMIW:
		reg &= ~(DWC3_GUSB2PHYCFG_PHYIF_MASK |
			DWC3_GUSB2PHYCFG_USBTRDTIM_MASK);
		reg |= DWC3_GUSB2PHYCFG_PHYIF(UTMI_PHYIF_16_BIT) |
			DWC3_GUSB2PHYCFG_USBTRDTIM(USBTRDTIM_UTMI_16_BIT);
		break;
	default:
		break;
	}

	/*
	 * Above 1.94a, it is recommended to set DWC3_GUSB2PHYCFG_SUSPHY to
	 * '0' during coreConsultant configuration. So default value will
	 * be '0' when the core is reset. Application needs to set it to
	 * '1' after the core initialization is completed.
	 */
	if (!DWC3_VER_IS_WITHIN(DWC3, ANY, 194A))
		reg |= DWC3_GUSB2PHYCFG_SUSPHY;

	/*
	 * For DRD controllers, GUSB2PHYCFG.SUSPHY must be cleared after
	 * power-on reset, and it can be set after core initialization, which is
	 * after device soft-reset during initialization.
	 */
	if (hw_mode == DWC3_GHWPARAMS0_MODE_DRD)
		reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;

	if (dwc->dis_u2_susphy_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_SUSPHY;

	if (dwc->dis_enblslpm_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_ENBLSLPM;
	else
		reg |= DWC3_GUSB2PHYCFG_ENBLSLPM;

	if (dwc->dis_u2_freeclk_exists_quirk)
		reg &= ~DWC3_GUSB2PHYCFG_U2_FREECLK_EXISTS;

	dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);

#if !defined(CONFIG_TARGET_PALLADIUM) && !defined(CONFIG_TARGET_EMULATOR)
	mdelay(10);
#endif
}

static void dwc3_core_setup_global_control(struct dwc3 *dwc)
{
	u32 hwparams4 = dwc->hwparams.hwparams4;
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	reg &= ~DWC3_GCTL_SCALEDOWN_MASK;

	switch (DWC3_GHWPARAMS1_EN_PWROPT(dwc->hwparams.hwparams1)) {
	case DWC3_GHWPARAMS1_EN_PWROPT_CLK:
		/**
		 * WORKAROUND: DWC3 revisions between 2.10a and 2.50a have an
		 * issue which would cause xHCI compliance tests to fail.
		 *
		 * Because of that we cannot enable clock gating on such
		 * configurations.
		 *
		 * Refers to:
		 *
		 * STAR#9000588375: Clock Gating, SOF Issues when ref_clk-Based
		 * SOF/ITP Mode Used
		 */
		if ((dwc->dr_mode == USB_DR_MODE_HOST ||
				dwc->dr_mode == USB_DR_MODE_OTG) &&
				DWC3_VER_IS_WITHIN(DWC3, 210A, 250A))
			reg |= DWC3_GCTL_DSBLCLKGTNG | DWC3_GCTL_SOFITPSYNC;
		else
			reg &= ~DWC3_GCTL_DSBLCLKGTNG;
		break;
	case DWC3_GHWPARAMS1_EN_PWROPT_HIB:
		/* enable hibernation here */
		dwc->nr_scratch = DWC3_GHWPARAMS4_HIBER_SCRATCHBUFS(hwparams4);

		/*
		 * REVISIT Enabling this bit so that host-mode hibernation
		 * will work. Device-mode hibernation is not yet implemented.
		 */
		reg |= DWC3_GCTL_GBLHIBERNATIONEN;
		break;
	default:
		/* nothing */
		break;
	}

	/* check if current dwc3 is on simulation board */
	if (dwc->hwparams.hwparams6 & DWC3_GHWPARAMS6_EN_FPGA) {
		ERROR("Running with FPGA optimizations\n");
		dwc->is_fpga = true;
	}
	if (dwc->disable_scramble_quirk && !dwc->is_fpga)
		ERROR("disable_scramble cannot be used on non-FPGA builds\n");

	if (dwc->disable_scramble_quirk && dwc->is_fpga)
		reg |= DWC3_GCTL_DISSCRAMBLE;
	else
		reg &= ~DWC3_GCTL_DISSCRAMBLE;

	if (dwc->u2exit_lfps_quirk)
		reg |= DWC3_GCTL_U2EXIT_LFPS;

	/*
	 * WORKAROUND: DWC3 revisions <1.90a have a bug
	 * where the device can fail to connect at SuperSpeed
	 * and falls back to high-speed mode which causes
	 * the device to enter a Connect/Disconnect loop
	 */
	if (DWC3_VER_IS_PRIOR(DWC3, 190A))
		reg |= DWC3_GCTL_U2RSTECN;

	dwc3_writel(dwc->regs, DWC3_GCTL, reg);
}

static void dwc3_set_power_down_clk_scale(struct dwc3 *dwc)
{
	u32 scale;
	u32 reg;
	if (!dwc->susp_clk)
		return;
	/*
	 * The power down scale field specifies how many suspend_clk
	 * periods fit into a 16KHz clock period. When performing
	 * the division, round up the remainder.
	 *
	 * The power down scale value is calculated using the fastest
	 * frequency of the suspend_clk. If it isn't fixed (but within
	 * the accuracy requirement), the driver may not know the max
	 * rate of the suspend_clk, so only update the power down scale
	 * if the default is less than the calculated value from
	 * clk_get_rate() or if the default is questionably high
	 * (3x or more) to be within the requirement.
	 */
	scale = DIV_ROUND_UP(dwc->susp_clk, 16000);
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
	if ((reg & DWC3_GCTL_PWRDNSCALE_MASK) < DWC3_GCTL_PWRDNSCALE(scale) ||
		(reg & DWC3_GCTL_PWRDNSCALE_MASK) > DWC3_GCTL_PWRDNSCALE(scale*3)) {
		reg &= ~(DWC3_GCTL_PWRDNSCALE_MASK);
		reg |= DWC3_GCTL_PWRDNSCALE(scale);
		dwc3_writel(dwc->regs, DWC3_GCTL, reg);
	}
	reg = dwc3_readl(dwc->regs, DWC3_GCTL);
}

/**
 * dwc3_core_init - Low-level initialization of DWC3 Core
 * @dwc: Pointer to our controller context structure
 *
 * Returns 0 on success otherwise negative errno.
 */
static int dwc3_core_init(struct dwc3 *dwc)
{
	unsigned int		hw_mode;
	u32			reg;
	int			ret;

	hw_mode = DWC3_GHWPARAMS0_MODE(dwc->hwparams.hwparams0);

	/* Handle USB2.0-only core configuration */
	if (DWC3_GHWPARAMS3_SSPHY_IFC(dwc->hwparams.hwparams3) ==
			DWC3_GHWPARAMS3_SSPHY_IFC_DIS) {
		if (dwc->maximum_speed == USB_SPEED_SUPER)
			dwc->maximum_speed = USB_SPEED_HIGH;
	}

	dwc3_phy_setup(dwc);

	ret = dwc3_core_soft_reset(dwc);
	if (ret)
		goto err0;

	if (hw_mode == DWC3_GHWPARAMS0_MODE_DRD &&
		!DWC3_VER_IS_WITHIN(DWC3, ANY, 194A)) {
		if (!dwc->dis_u3_susphy_quirk) {
			reg = dwc3_readl(dwc->regs, DWC3_GUSB3PIPECTL(0));
			reg |= DWC3_GUSB3PIPECTL_SUSPHY;
			dwc3_writel(dwc->regs, DWC3_GUSB3PIPECTL(0), reg);
		}

		if (!dwc->dis_u2_susphy_quirk) {
			reg = dwc3_readl(dwc->regs, DWC3_GUSB2PHYCFG(0));
			reg |= DWC3_GUSB2PHYCFG_SUSPHY;
			dwc3_writel(dwc->regs, DWC3_GUSB2PHYCFG(0), reg);
		}
	}

	dwc3_core_setup_global_control(dwc);
	dwc3_core_num_eps(dwc);

	ret = dwc3_setup_scratch_buffers(dwc);
	if (ret)
		goto err1;

	/* Set power down scale of suspend_clk */
	dwc3_set_power_down_clk_scale(dwc);

	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		ERROR("failed to setup event buffers\n");
		goto err1;
	}

	if (!DWC3_VER_IS_PRIOR(DWC3, 250A)) {
		reg = dwc3_readl(dwc->regs, DWC3_GUCTL1);

		/*
		 * Enable hardware control of sending remote wakeup
		 * in HS when the device is in the L1 state.
		 */
		if (!DWC3_VER_IS_PRIOR(DWC3, 290A))
			reg |= DWC3_GUCTL1_DEV_L1_EXIT_BY_HW;

		/*
		 * Decouple USB 2.0 L1 & L2 events which will allow for
		 * gadget driver to only receive U3/L2 suspend & wakeup
		 * events and prevent the more frequent L1 LPM transitions
		 * from interrupting the driver.
		 */
		if (!DWC3_VER_IS_PRIOR(DWC3, 300A))
			reg |= DWC3_GUCTL1_DEV_DECOUPLE_L1L2_EVT;

		if (DWC3_VER_IS_WITHIN(DWC3, 290A, ANY) &&
			(dwc->maximum_speed == USB_SPEED_HIGH ||
			 dwc->maximum_speed == USB_SPEED_FULL))
			reg |= DWC3_GUCTL1_DEV_FORCE_20_CLK_FOR_30_CLK;

		dwc3_writel(dwc->regs, DWC3_GUCTL1, reg);
	}

	return 0;

err1:
	dwc3_free_scratch_buffers(dwc);

err0:
	return ret;
}

static void dwc3_core_exit(struct dwc3 *dwc)
{
	dwc3_free_scratch_buffers(dwc);
}

static int dwc3_core_init_mode(struct dwc3 *dwc)
{
	int ret;

	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_DEVICE);
		ret = dwc3_gadget_init(dwc);
		if (ret) {
			ERROR("failed to initialize gadget\n");
			return ret;
		}
		break;
	case USB_DR_MODE_HOST:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_HOST);
		ret = dwc3_host_init(dwc);
		if (ret) {
			ERROR("failed to initialize host\n");
			return ret;
		}
		break;
	case USB_DR_MODE_OTG:
		dwc3_set_mode(dwc, DWC3_GCTL_PRTCAP_OTG);
		ret = dwc3_host_init(dwc);
		if (ret) {
			ERROR("failed to initialize host\n");
			return ret;
		}

		ret = dwc3_gadget_init(dwc);
		if (ret) {
			ERROR("failed to initialize gadget\n");
			return ret;
		}
		break;
	default:
		ERROR("Unsupported mode of operation %d\n", dwc->dr_mode);
		return -EINVAL;
	}

	return 0;
}

static void dwc3_core_exit_mode(struct dwc3 *dwc)
{
	switch (dwc->dr_mode) {
	case USB_DR_MODE_PERIPHERAL:
		dwc3_gadget_exit(dwc);
		break;
	case USB_DR_MODE_HOST:
		dwc3_host_exit(dwc);
		break;
	case USB_DR_MODE_OTG:
		dwc3_host_exit(dwc);
		dwc3_gadget_exit(dwc);
		break;
	default:
		/* do nothing */
		break;
	}

}

static bool dwc3_core_is_valid(struct dwc3 *dwc)
{
	u32 reg;

	reg = dwc3_readl(dwc->regs, DWC3_GSNPSID);
	dwc->ip = DWC3_GSNPS_ID(reg);

	/* This should read as U3 followed by revision number */
	if (DWC3_IP_IS(DWC3)) {
		dwc->revision = reg;
	} else if (DWC3_IP_IS(DWC31) || DWC3_IP_IS(DWC32)) {
		dwc->revision = dwc3_readl(dwc->regs, DWC3_VER_NUMBER);
	} else {
		return false;
	}
	return true;
}

/**
 * dwc3_uboot_init - dwc3 core uboot initialization code
 * @dwc3_dev: struct dwc3_device containing initialization data
 *
 * Entry point for dwc3 driver (equivalent to dwc3_probe in linux
 * kernel driver). Pointer to dwc3_device should be passed containing
 * base address and other initialization data. Returns '0' on success and
 * a negative value on failure.
 *
 * Generally called from board_usb_init() implemented in board file.
 */
int soph_dwc3_uboot_init(struct dwc3_device *dwc3_dev)
{
	struct dwc3		*dwc;
	u8			lpm_nyet_threshold;
	u8			tx_de_emphasis;
	u8			hird_threshold;

	int			ret;

	void			*mem;

#if USB_STATIC_MEM
	mem = (uint8_t *)DWC_BASE;
#else
	mem = kzalloc(sizeof(*dwc) + DWC3_ALIGN_MASK, 0);
	if (!mem)
		return -ENOMEM;
#endif

	dwc = PTR_ALIGN(mem, DWC3_ALIGN_MASK + 1);
	dwc->mem = mem;

	dwc->regs = (void *)(uintptr_t)(dwc3_dev->base +
					DWC3_GLOBALS_REGS_START);

	/* default to highest possible threshold */
	lpm_nyet_threshold = 0xf;

	/* default to -3.5dB de-emphasis */
	tx_de_emphasis = 1;

	/*
	 * default to assert utmi_sleep_n and use maximum allowed HIRD
	 * threshold value of 0b1100
	 */
	hird_threshold = 12;

	dwc->maximum_speed = dwc3_dev->maximum_speed;
	dwc->has_lpm_erratum = dwc3_dev->has_lpm_erratum;
	if (dwc3_dev->lpm_nyet_threshold)
		lpm_nyet_threshold = dwc3_dev->lpm_nyet_threshold;
	dwc->is_utmi_l1_suspend = dwc3_dev->is_utmi_l1_suspend;
	if (dwc3_dev->hird_threshold)
		hird_threshold = dwc3_dev->hird_threshold;

	dwc->needs_fifo_resize = dwc3_dev->tx_fifo_resize;
	dwc->dr_mode = dwc3_dev->dr_mode;

	dwc->disable_scramble_quirk = dwc3_dev->disable_scramble_quirk;
	dwc->u2exit_lfps_quirk = dwc3_dev->u2exit_lfps_quirk;
	dwc->u2ss_inp3_quirk = dwc3_dev->u2ss_inp3_quirk;
	dwc->req_p1p2p3_quirk = dwc3_dev->req_p1p2p3_quirk;
	dwc->del_p1p2p3_quirk = dwc3_dev->del_p1p2p3_quirk;
	dwc->del_phy_power_chg_quirk = dwc3_dev->del_phy_power_chg_quirk;
	dwc->lfps_filter_quirk = dwc3_dev->lfps_filter_quirk;
	dwc->rx_detect_poll_quirk = dwc3_dev->rx_detect_poll_quirk;
	dwc->dis_u3_susphy_quirk = dwc3_dev->dis_u3_susphy_quirk;
	dwc->dis_u2_susphy_quirk = dwc3_dev->dis_u2_susphy_quirk;
	dwc->dis_del_phy_power_chg_quirk = dwc3_dev->dis_del_phy_power_chg_quirk;
	dwc->dis_tx_ipgap_linecheck_quirk = dwc3_dev->dis_tx_ipgap_linecheck_quirk;
	dwc->dis_enblslpm_quirk = dwc3_dev->dis_enblslpm_quirk;
	dwc->dis_u2_freeclk_exists_quirk = dwc3_dev->dis_u2_freeclk_exists_quirk;

	dwc->tx_de_emphasis_quirk = dwc3_dev->tx_de_emphasis_quirk;
	if (dwc3_dev->tx_de_emphasis)
		tx_de_emphasis = dwc3_dev->tx_de_emphasis;

	/* default to superspeed if no maximum_speed passed */
	if (dwc->maximum_speed == USB_SPEED_UNKNOWN)
		dwc->maximum_speed = USB_SPEED_SUPER;

	dwc->lpm_nyet_threshold = lpm_nyet_threshold;
	dwc->tx_de_emphasis = tx_de_emphasis;

	dwc->hird_threshold = hird_threshold
		| (dwc->is_utmi_l1_suspend << 4);

	dwc->hsphy_mode = dwc3_dev->hsphy_mode;

	dwc->index = dwc3_dev->index;
	dwc->susp_clk = dwc3_dev->susp_clk;
	if (!dwc3_core_is_valid(dwc)) {
		ERROR("This is not a DesignWare USB3 DRD Core\n");
		ret = -ENODEV;
		return ret;
	}

	dwc3_cache_hwparams(dwc);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		ERROR("failed to allocate event buffers\n");
		return -ENOMEM;
	}

	dwc->dr_mode = USB_DR_MODE_PERIPHERAL;

	ret = dwc3_alloc_scratch_buffers(dwc);
	if (ret)
		goto err0;

	ret = dwc3_core_init(dwc);
	if (ret) {
		ERROR("failed to initialize core\n");
		goto err0;
	}

	ret = dwc3_core_init_mode(dwc);
	if (ret)
		goto err2;

	list_add_tail(&dwc->list, &dwc3_list);

	return 0;

err2:
	dwc3_event_buffers_cleanup(dwc);

	dwc3_core_exit(dwc);

err0:
	dwc3_free_event_buffers(dwc);

	return ret;
}

/**
 * dwc3_uboot_exit - dwc3 core uboot cleanup code
 * @index: index of this controller
 *
 * Performs cleanup of memory allocated in dwc3_uboot_init and other misc
 * cleanups (equivalent to dwc3_remove in linux). index of _this_ controller
 * should be passed and should match with the index passed in
 * dwc3_device during init.
 *
 * Generally called from board file.
 */
void soph_dwc3_uboot_exit(int index)
{
	struct dwc3 *dwc;

	list_for_each_entry(dwc, &dwc3_list, list) {
		if (dwc->index != index)
			continue;

		dwc3_core_exit_mode(dwc);
		dwc3_event_buffers_cleanup(dwc);
		dwc3_free_event_buffers(dwc);
		dwc3_core_exit(dwc);
		list_del(&dwc->list);
#if USB_STATIC_MEM
		memset(dwc->mem, 0, DWC_SIZE);
		memset(&g_evt, 0, sizeof(g_evt));
#else
		kfree(dwc->mem);
#endif
		break;
	}
}

/**
 * dwc3_uboot_handle_interrupt - handle dwc3 core interrupt
 * @index: index of this controller
 *
 * Invokes dwc3 gadget interrupts.
 *
 * Generally called from board file.
 */
void soph_dwc3_uboot_handle_interrupt(int index)
{
	struct dwc3 *dwc = NULL;

	list_for_each_entry(dwc, &dwc3_list, list) {
		if (dwc->index != index)
			continue;
		dwc3_gadget_uboot_handle_interrupt(dwc);
		break;
	}
}

int soph_dwc3_init(struct dwc3 *dwc)
{
	int ret;
	u32 reg;

	dwc3_cache_hwparams(dwc);

	ret = dwc3_alloc_event_buffers(dwc, DWC3_EVENT_BUFFERS_SIZE);
	if (ret) {
		ERROR("failed to allocate event buffers\n");
		return -ENOMEM;
	}

	ret = dwc3_core_init(dwc);
	if (ret) {
		ERROR("failed to initialize core\n");
		goto core_fail;
	}

	ret = dwc3_event_buffers_setup(dwc);
	if (ret) {
		ERROR("failed to setup event buffers\n");
		goto event_fail;
	}

	if (!DWC3_VER_IS_PRIOR(DWC3, 250A)) {
		reg = dwc3_readl(dwc->regs, DWC3_GUCTL1);

		/*
		 * Enable hardware control of sending remote wakeup
		 * in HS when the device is in the L1 state.
		 */
		if (!DWC3_VER_IS_PRIOR(DWC3, 290A))
			reg |= DWC3_GUCTL1_DEV_L1_EXIT_BY_HW;

		if (dwc->dis_tx_ipgap_linecheck_quirk)
			reg |= DWC3_GUCTL1_TX_IPGAP_LINECHECK_DIS;

		dwc3_writel(dwc->regs, DWC3_GUCTL1, reg);
	}

	if (dwc->dr_mode == USB_DR_MODE_HOST ||
		dwc->dr_mode == USB_DR_MODE_OTG) {
		reg = dwc3_readl(dwc->regs, DWC3_GUCTL);

		reg |= DWC3_GUCTL_HSTINAUTORETRY;

		dwc3_writel(dwc->regs, DWC3_GUCTL, reg);
	}

	ret = dwc3_core_init_mode(dwc);
	if (ret)
		goto mode_fail;

	return 0;

mode_fail:
	dwc3_event_buffers_cleanup(dwc);

event_fail:
	dwc3_core_exit(dwc);

core_fail:
	dwc3_free_event_buffers(dwc);

	return ret;
}

void soph_dwc3_remove(struct dwc3 *dwc)
{
	dwc3_core_exit_mode(dwc);
	dwc3_event_buffers_cleanup(dwc);
	dwc3_free_event_buffers(dwc);
	dwc3_core_exit(dwc);

#if USB_STATIC_MEM
	memset(dwc->mem, 0, DWC_SIZE);
#else
	kfree(dwc->mem);
#endif
}
