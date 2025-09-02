// SPDX-License-Identifier: GPL-2.0
/*
 * Designware master SPI core controller driver
 *
 * Copyright (C) 2014 Stefan Roese <sr@denx.de>
 * Copyright (C) 2020 Sean Anderson <seanga2@gmail.com>
 *
 * Very loosely based on the Linux driver:
 * drivers/spi/spi-dw.c, which is:
 * Copyright (c) 2009, Intel Corporation.
 */

#define LOG_CATEGORY UCLASS_SPI
#include <common.h>
#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <errno.h>
#include <fdtdec.h>
#include <log.h>
#include <malloc.h>
#include <reset.h>
#include <spi.h>
#include <spi-mem.h>
#include <asm/io.h>
#include <asm-generic/gpio.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/compat.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>
#include <mmio.h>
#include <spi-dma.h>
#include <spi_tuning.h>
/* Register offsets */
#define DW_SPI_CTRLR0			0x00
#define DW_SPI_CTRLR1			0x04
#define DW_SPI_SSIENR			0x08
#define DW_SPI_MWCR			0x0c
#define DW_SPI_SER			0x10
#define DW_SPI_BAUDR			0x14
#define DW_SPI_TXFTLR			0x18
#define DW_SPI_RXFTLR			0x1c
#define DW_SPI_TXFLR			0x20
#define DW_SPI_RXFLR			0x24
#define DW_SPI_SR			0x28
#define DW_SPI_IMR			0x2c
#define DW_SPI_ISR			0x30
#define DW_SPI_RISR			0x34
#define DW_SPI_TXOICR			0x38
#define DW_SPI_RXOICR			0x3c
#define DW_SPI_RXUICR			0x40
#define DW_SPI_MSTICR			0x44
#define DW_SPI_ICR			0x48
#define DW_SPI_DMACR			0x4c
#define DW_SPI_DMATDLR			0x50
#define DW_SPI_DMARDLR			0x54
#define DW_SPI_IDR			0x58
#define DW_SPI_VERSION			0x5c
#define DW_SPI_DR			0x60
#define DW_SPI_RX_SAMPLE_DLY		0xf0
#define DW_SPI_CTRLR0_EXT		0xf4
#define DW_TXD_DRIVE_EDGE		0xf8

#ifndef DW_VERSION_4_04
#define DW_SPI_CS_OVERRIDE		0xf4
#endif

/* Bit fields in CTRLR0 */
/*
 * Only present when SSI_MAX_XFER_SIZE=16. This is the default, and the only
 * option before version 3.23a.
 */
#define CTRLR0_DFS_MASK			GENMASK(3, 0)

#define CTRLR0_FRF_MASK			GENMASK(5, 4)
#define CTRLR0_FRF_SPI			0x0
#define CTRLR0_FRF_SSP			0x1
#define CTRLR0_FRF_MICROWIRE		0x2
#define CTRLR0_FRF_RESV			0x3

#define CTRLR0_MODE_MASK		GENMASK(7, 6)
#define CTRLR0_MODE_SCPH		0x1
#define CTRLR0_MODE_SCPOL		0x2

#define CTRLR0_TMOD_MASK		GENMASK(9, 8)
#define	CTRLR0_TMOD_TR			0x0		/* xmit & recv */
#define CTRLR0_TMOD_TO			0x1		/* xmit only */
#define CTRLR0_TMOD_RO			0x2		/* recv only */
#define CTRLR0_TMOD_EPROMREAD		0x3		/* eeprom read mode */

#define CTRLR0_SLVOE_OFFSET		10
#define CTRLR0_SRL_OFFSET		11
#define CTRLR0_CFS_MASK			GENMASK(15, 12)

/* Only present when SSI_MAX_XFER_SIZE=32 */
#define CTRLR0_DFS_32_MASK		GENMASK(20, 16)

/* The next field is only present on versions after 4.00a */
#define CTRLR0_SPI_FRF_MASK		GENMASK(22, 21)
#define CTRLR0_SPI_FRF_BYTE		0x0
#define	CTRLR0_SPI_FRF_DUAL		0x1
#define	CTRLR0_SPI_FRF_QUAD		0x2

/* Bit fields in CTRLR0 based on DWC_ssi_databook.pdf v1.01a */
#define DWC_SSI_CTRLR0_DFS_MASK		GENMASK(4, 0)
#define DWC_SSI_CTRLR0_FRF_MASK		GENMASK(7, 6)
#define DWC_SSI_CTRLR0_MODE_MASK	GENMASK(9, 8)
#define DWC_SSI_CTRLR0_TMOD_MASK	GENMASK(11, 10)
#define DWC_SSI_CTRLR0_SRL_OFFSET	13
#define DWC_SSI_CTRLR0_SPI_FRF_MASK	GENMASK(23, 22)

/* Bit fields in SR, 7 bits */
#define SR_MASK				GENMASK(6, 0)	/* cover 7 bits */
#define SR_BUSY				BIT(0)
#define SR_TF_NOT_FULL			BIT(1)
#define SR_TF_EMPT			BIT(2)
#define SR_RF_NOT_EMPT			BIT(3)
#define SR_RF_FULL			BIT(4)
#define SR_TX_ERR			BIT(5)
#define SR_DCOL				BIT(6)

#define RX_TIMEOUT			1000		/* timeout in ms */

/* Bit fields in ISR, IMR, RISR, 7 bits */
#define SPI_INT_TXEI                    BIT(0)
#define SPI_INT_TXOI                    BIT(1)
#define SPI_INT_RXUI                    BIT(2)
#define SPI_INT_RXOI                    BIT(3)
#define SPI_INT_RXFI                    BIT(4)
#define SPI_INT_MSTI                    BIT(5)

#define SPI_TMOD_OFFSET                 8
#define SPI_TMOD_MASK                   (0x3 << SPI_TMOD_OFFSET)
#define SPI_TMOD_TR                     0x0             /* xmit & recv */
#define SPI_TMOD_TO                     0x1             /* xmit only */
#define SPI_TMOD_RO                     0x2             /* recv only */
#define SPI_TMOD_EPROMREAD              0x3             /* eeprom read mode */

struct dw_spi_plat {
	s32 frequency;		/* Default clock frequency, -1 for none */
	u8 rx_sample;
	u8 txd_drive_edge;
	void __iomem *regs;
};

struct dw_spi_priv {
	struct clk clk;
	struct reset_ctl_bulk resets;
	struct gpio_desc cs_gpio;	/* External chip-select gpio */

	u32 (*update_cr0)(struct dw_spi_priv *priv);

	void __iomem *regs;
	unsigned long bus_clk_rate;
	unsigned int freq;		/* Default frequency */
	unsigned int mode;

	const void *tx;
	const void *tx_end;
	void *rx;
	void *rx_end;
	u32 fifo_len;			/* depth of the FIFO buffer */
	u32 max_xfer;			/* Maximum transfer size (in bits) */

	int bits_per_word;
	int len;
	u8 cs;				/* chip select pin */
	u8 tmode;			/* TR/TO/RO/EEPROM */
	u8 type;			/* SPI/SSP/MicroWire */
	u8 n_bytes;
	u8 rx_sample;
	u8 txd_drive_edge;
};

static inline u32 dw_read(struct dw_spi_priv *priv, u32 offset)
{
	return __raw_readl(priv->regs + offset);
}

static inline void dw_write(struct dw_spi_priv *priv, u32 offset, u32 val)
{
	__raw_writel(val, priv->regs + offset);
}

static u32 dw_spi_dw16_update_cr0(struct dw_spi_priv *priv)
{
	return FIELD_PREP(CTRLR0_DFS_MASK, priv->bits_per_word - 1)
	     | FIELD_PREP(CTRLR0_FRF_MASK, priv->type)
	     | FIELD_PREP(CTRLR0_MODE_MASK, priv->mode)
	     | FIELD_PREP(CTRLR0_TMOD_MASK, priv->tmode);
}

static u32 dw_spi_dw32_update_cr0(struct dw_spi_priv *priv)
{
	return FIELD_PREP(CTRLR0_DFS_32_MASK, priv->bits_per_word - 1)
	     | FIELD_PREP(CTRLR0_FRF_MASK, priv->type)
	     | FIELD_PREP(CTRLR0_MODE_MASK, priv->mode)
	     | FIELD_PREP(CTRLR0_TMOD_MASK, priv->tmode);
}

static u32 dw_spi_dwc_update_cr0(struct dw_spi_priv *priv)
{
	return FIELD_PREP(DWC_SSI_CTRLR0_DFS_MASK, priv->bits_per_word - 1)
	     | FIELD_PREP(DWC_SSI_CTRLR0_FRF_MASK, priv->type)
	     | FIELD_PREP(DWC_SSI_CTRLR0_MODE_MASK, priv->mode)
	     | FIELD_PREP(DWC_SSI_CTRLR0_TMOD_MASK, priv->tmode);
}

static int dw_spi_apb_init(struct udevice *bus, struct dw_spi_priv *priv)
{
	/* If we read zeros from DFS, then we need to use DFS_32 instead */
	dw_write(priv, DW_SPI_SSIENR, 0);
	dw_write(priv, DW_SPI_CTRLR0, 0xffffffff);
	if (FIELD_GET(CTRLR0_DFS_MASK, dw_read(priv, DW_SPI_CTRLR0))) {
		priv->max_xfer = 16;
		priv->update_cr0 = dw_spi_dw16_update_cr0;
	} else {
		priv->max_xfer = 32;
		priv->update_cr0 = dw_spi_dw32_update_cr0;
	}

	return 0;
}

static int dw_spi_dwc_init(struct udevice *bus, struct dw_spi_priv *priv)
{
	priv->max_xfer = 32;
	priv->update_cr0 = dw_spi_dwc_update_cr0;
	return 0;
}

static int request_gpio_cs(struct udevice *bus)
{
#if CONFIG_IS_ENABLED(DM_GPIO) && !defined(CONFIG_SPL_BUILD)
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret;

	/* External chip select gpio line is optional */
	ret = gpio_request_by_name(bus, "cs-gpios", 0, &priv->cs_gpio,
				   GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	if (ret == -ENOENT)
		return 0;

	if (ret < 0) {
		dev_err(bus, "Couldn't request gpio! (error %d)\n", ret);
		return ret;
	}

	if (dm_gpio_is_valid(&priv->cs_gpio)) {
		dm_gpio_set_dir_flags(&priv->cs_gpio,
				      GPIOD_IS_OUT | GPIOD_IS_OUT_ACTIVE);
	}

	dev_dbg(bus, "Using external gpio for CS management\n");
#endif
	return 0;
}

static int dw_spi_of_to_plat(struct udevice *bus)
{
	struct dw_spi_plat *plat = dev_get_plat(bus);

	plat->regs = dev_read_addr_ptr(bus);
	if (!plat->regs)
		return -EINVAL;

	/* Use 500KHz as a suitable default */
	plat->frequency = dev_read_u32_default(bus, "spi-max-frequency",
					       500000);
	plat->rx_sample = dev_read_u32_default(bus, "rx-sample-delay-ns", 1);
	plat->txd_drive_edge = dev_read_u32_default(bus, "txd-drive-edge", 0);

	if (dev_read_bool(bus, "spi-slave"))
		return -EINVAL;

	dev_info(bus, "max-frequency=%d\n", plat->frequency);

	return request_gpio_cs(bus);
}

/* Restart the controller, disable all interrupts, clean rx fifo */
static void spi_hw_init(struct udevice *bus, struct dw_spi_priv *priv)
{
	dw_write(priv, DW_SPI_SSIENR, 0);
	dw_write(priv, DW_SPI_IMR, 0xff);
	dw_write(priv, DW_SPI_SSIENR, 1);

	/*
	 * Try to detect the FIFO depth if not set by interface driver,
	 * the depth could be from 2 to 256 from HW spec
	 */
	if (!priv->fifo_len) {
		u32 fifo;

		for (fifo = 1; fifo < 256; fifo++) {
			dw_write(priv, DW_SPI_TXFTLR, fifo);
			if (fifo != dw_read(priv, DW_SPI_TXFTLR))
				break;
		}

		priv->fifo_len = (fifo == 1) ? 0 : fifo;
		dw_write(priv, DW_SPI_TXFTLR, 0);
	}
	dw_write(priv, DW_SPI_SSIENR, 0);
	dw_write(priv, DW_SPI_IMR, 0xff);
	dev_dbg(bus, "fifo_len=%d\n", priv->fifo_len);
}

/*
 * We define dw_spi_get_clk function as 'weak' as some targets
 * (like SOCFPGA_GEN5 and SOCFPGA_ARRIA10) don't use standard clock API
 * and implement dw_spi_get_clk their own way in their clock manager.
 */
__weak int dw_spi_get_clk(struct udevice *bus, ulong *rate)
{
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret;

	ret = clk_get_by_index(bus, 0, &priv->clk);
	if (ret)
		return ret;

	ret = clk_enable(&priv->clk);
	if (ret && ret != -ENOSYS && ret != -ENOTSUPP)
		return ret;

	*rate = clk_get_rate(&priv->clk);
	if (!*rate)
		goto err_rate;

	dev_dbg(bus, "Got clock via device tree: %lu Hz\n", *rate);

	return 0;

err_rate:
	clk_disable(&priv->clk);
	clk_free(&priv->clk);

	return -EINVAL;
}

static int dw_spi_reset(struct udevice *bus)
{
	int ret;
	struct dw_spi_priv *priv = dev_get_priv(bus);

	ret = reset_get_bulk(bus, &priv->resets);
	if (ret) {
		/*
		 * Return 0 if error due to !CONFIG_DM_RESET and reset
		 * DT property is not present.
		 */
		if (ret == -ENOENT || ret == -ENOTSUPP)
			return 0;

		dev_warn(bus, "Couldn't find/assert reset device (error %d)\n",
			 ret);
		return ret;
	}

	ret = reset_deassert_bulk(&priv->resets);
	if (ret) {
		reset_release_bulk(&priv->resets);
		dev_err(bus, "Failed to de-assert reset for SPI (error %d)\n",
			ret);
		return ret;
	}

	return 0;
}

typedef int (*dw_spi_init_t)(struct udevice *bus, struct dw_spi_priv *priv);

static int dw_spi_probe(struct udevice *bus)
{
	dw_spi_init_t init = (dw_spi_init_t)dev_get_driver_data(bus);
	struct dw_spi_plat *plat = dev_get_plat(bus);
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret;
	u32 version;

	priv->regs = plat->regs;
	priv->freq = plat->frequency;
	priv->rx_sample = plat->rx_sample;
	priv->txd_drive_edge = plat->txd_drive_edge;
	//ret = dw_spi_get_clk(bus, &priv->bus_clk_rate);
	//if (ret) {
	//	dev_err(bus, "[%s]{%d} get the spi clk failed, ret:%d!\n", __FUNCTION__, __LINE__, ret);
	//	return ret;
	//}
	priv->bus_clk_rate = 300000000;

	ret = dw_spi_reset(bus);
	if (ret)
		return ret;

	if (!init)
		return -EINVAL;
	ret = init(bus, priv);
	if (ret)
		return ret;

	version = dw_read(priv, DW_SPI_VERSION);
	dev_dbg(bus, "ssi_version_id=%c.%c%c%c ssi_max_xfer_size=%u\n",
		version >> 24, version >> 16, version >> 8, version,
		priv->max_xfer);

	/* Currently only bits_per_word == 8 supported */
	priv->bits_per_word = 8;
	priv->n_bytes = 1;
	priv->tmode = 0; /* Tx & Rx */

	/* Basic HW init */
	spi_hw_init(bus, priv);

	return 0;
}

/* Return the max entries we can fill into tx fifo */
static inline u32 tx_max(struct dw_spi_priv *priv)
{
	u32 tx_left, tx_room, rxtx_gap;

	tx_left = (priv->tx_end - priv->tx) / (priv->bits_per_word >> 3);
	tx_room = priv->fifo_len - dw_read(priv, DW_SPI_TXFLR);

	/*
	 * Another concern is about the tx/rx mismatch, we
	 * thought about using (priv->fifo_len - rxflr - txflr) as
	 * one maximum value for tx, but it doesn't cover the
	 * data which is out of tx/rx fifo and inside the
	 * shift registers. So a control from sw point of
	 * view is taken.
	 */
	rxtx_gap = ((priv->rx_end - priv->rx) - (priv->tx_end - priv->tx)) /
		(priv->bits_per_word >> 3);

	return min3(tx_left, tx_room, (u32)(priv->fifo_len - rxtx_gap));
}

/* Return the max entries we should read out of rx fifo */
static inline u32 rx_max(struct dw_spi_priv *priv)
{
	u32 rx_left = (priv->rx_end - priv->rx) / (priv->bits_per_word >> 3);

	return min_t(u32, rx_left, dw_read(priv, DW_SPI_RXFLR));
}

static void dw_writer(struct dw_spi_priv *priv)
{
	u32 max = tx_max(priv);
	u32 txw = 0xFFFFFFFF;

	while (max--) {
		/* Set the tx word if the transfer's original "tx" is not null */
		if (priv->tx_end - priv->len) {
			if (priv->bits_per_word == 8)
				txw = *(u8 *)(priv->tx);
			else
				txw = *(u16 *)(priv->tx);
		}
		dw_write(priv, DW_SPI_DR, txw);
		log_content("tx=0x%02x\n", txw);
		priv->tx += priv->bits_per_word >> 3;
	}
}

static void dw_reader(struct dw_spi_priv *priv)
{
	u32 max = rx_max(priv);
	u16 rxw;

	while (max--) {
		rxw = dw_read(priv, DW_SPI_DR);
		log_content("rx=0x%02x\n", rxw);

		/* Care about rx if the transfer's original "rx" is not null */
		if (priv->rx_end - priv->len) {
			if (priv->bits_per_word == 8)
				*(u8 *)(priv->rx) = rxw;
			else
				*(u16 *)(priv->rx) = rxw;
		}
		priv->rx += priv->bits_per_word >> 3;
	}
}

static int poll_transfer(struct dw_spi_priv *priv)
{
	do {
		dw_writer(priv);
		dw_reader(priv);
	} while (priv->rx_end > priv->rx);

	return 0;
}

/*
 * We define external_cs_manage function as 'weak' as some targets
 * (like MSCC Ocelot) don't control the external CS pin using a GPIO
 * controller. These SoCs use specific registers to control by
 * software the SPI pins (and especially the CS).
 */
__weak void external_cs_manage(struct udevice *dev, bool on)
{
#if CONFIG_IS_ENABLED(DM_GPIO) && !defined(CONFIG_SPL_BUILD)
	struct dw_spi_priv *priv = dev_get_priv(dev->parent);

	if (!dm_gpio_is_valid(&priv->cs_gpio))
		return;

	dm_gpio_set_value(&priv->cs_gpio, on ? 1 : 0);
#endif
}

static int dw_spi_xfer(struct udevice *dev, unsigned int bitlen,
		       const void *dout, void *din, unsigned long flags)
{
	struct udevice *bus = dev->parent;
	struct dw_spi_priv *priv = dev_get_priv(bus);
	const u8 *tx = dout;
	u8 *rx = din;
	int ret = 0;
	u32 cr0 = 0;
	u32 val;
	u32 cs;

	/* spi core configured to do 8 bit transfers */
	if (bitlen % 8) {
		dev_err(dev, "Non byte aligned SPI transfer.\n");
		return -1;
	}

	/* Start the transaction if necessary. */
	if (flags & SPI_XFER_BEGIN)
		external_cs_manage(dev, false);

	if (rx && tx)
		priv->tmode = CTRLR0_TMOD_TR;
	else if (rx)
		priv->tmode = CTRLR0_TMOD_RO;
	else
		/*
		 * In transmit only mode (CTRL0_TMOD_TO) input FIFO never gets
		 * any data which breaks our logic in poll_transfer() above.
		 */
		priv->tmode = CTRLR0_TMOD_TR;

	cr0 = priv->update_cr0(priv);

	priv->len = bitlen >> 3;

	priv->tx = (void *)tx;
	priv->tx_end = priv->tx + priv->len;
	priv->rx = rx;
	priv->rx_end = priv->rx + priv->len;

	/* Disable controller before writing control registers */
	dw_write(priv, DW_SPI_SSIENR, 0);

	dev_dbg(dev, "cr0=%08x rx=%p tx=%p len=%d [bytes]\n", cr0, rx, tx,
		priv->len);
	/* Reprogram cr0 only if changed */
	if (dw_read(priv, DW_SPI_CTRLR0) != cr0)
		dw_write(priv, DW_SPI_CTRLR0, cr0);

	/*
	 * Configure the desired SS (slave select 0...3) in the controller
	 * The DW SPI controller will activate and deactivate this CS
	 * automatically. So no cs_activate() etc is needed in this driver.
	 */
	cs = spi_chip_select(dev);
	dw_write(priv, DW_SPI_SER, 1 << cs);

	/* Enable controller after writing control registers */
	dw_write(priv, DW_SPI_SSIENR, 1);

	/* Start transfer in a polling loop */
	ret = poll_transfer(priv);

	/*
	 * Wait for current transmit operation to complete.
	 * Otherwise if some data still exists in Tx FIFO it can be
	 * silently flushed, i.e. dropped on disabling of the controller,
	 * which happens when writing 0 to DW_SPI_SSIENR which happens
	 * in the beginning of new transfer.
	 */
	if (readl_poll_timeout(priv->regs + DW_SPI_SR, val,
			       (val & SR_TF_EMPT) && !(val & SR_BUSY),
			       RX_TIMEOUT * 1000)) {
		ret = -ETIMEDOUT;
	}

	/* Stop the transaction if necessary */
	if (flags & SPI_XFER_END)
		external_cs_manage(dev, true);

	return ret;
}

static inline void spi_reset_chip(struct dw_spi_priv *priv)
{
	dw_write(priv, DW_SPI_SSIENR, 0);
	dw_read(priv, DW_SPI_ICR);
	dw_write(priv, DW_SPI_SER, 0);
	dw_write(priv, DW_SPI_SSIENR, 1);
}

int dw_spi_check_status(struct dw_spi_priv *priv, int raw)
{
	u32 irq_status;
	int ret = 0;

	if (raw)
		irq_status = dw_read(priv, DW_SPI_RISR);
	else
		irq_status = dw_read(priv, DW_SPI_ISR);

	if (irq_status & SPI_INT_RXOI) {
		printf("RX FIFO overflow detected\n");
		ret = -1;
	}

	if (irq_status & SPI_INT_RXUI) {
		printf("RX FIFO underflow detected\n");
		ret = -1;
	}

	if (irq_status & SPI_INT_TXOI) {
		printf("TX FIFO overflow detected\n");
		ret = -1;
	}

	/* Generically handle the erroneous situation */
	if (ret)
		spi_reset_chip(priv);

	return ret;
}

#define REG_IO_WIDTH		4

u8 select_data_width(u8 width)
{
	u8 w = 0;

	switch (width) {
	case 1:
		w = 0;
		break;

	case 2:
		w = 1;
		break;

	case 4:
		w = 2;
		break;
	}
	return w;
}

void spi_update_config(struct spi_slave *slave, const struct spi_mem_op *op)
{
	struct udevice *bus = slave->dev->parent;
	struct dw_spi_priv *priv = dev_get_priv(bus);
	u32 ctrl0 = 0;
	u32 spi_ctrl0 = 0;

	struct spi_cfg {
		u8 tmode;
		u8 dfs;
		u32 ndf;
	};

	struct spi_param {
		u8 cur_rx_delay;
		u8 cur_txd_drive_edge;
	};
	struct spi_cfg cfg;
	static struct spi_param param = {0};

	cfg.dfs = 8;
	if (op->data.buswidth > 1 && op->data.nbytes > 4) {
		cfg.dfs = 32;
		ctrl0 |= 1 << 25;
	} else {
		cfg.dfs = 8;
	}
	priv->n_bytes = cfg.dfs / 8;

	if (op->data.dir == SPI_MEM_DATA_IN) {
		cfg.tmode = SPI_TMOD_EPROMREAD;
		cfg.ndf = (op->data.nbytes * 8 + cfg.dfs - 1) / cfg.dfs;
	} else {
		cfg.tmode = SPI_TMOD_TO;
	}

	if (cfg.tmode == SPI_TMOD_EPROMREAD || cfg.tmode == SPI_TMOD_RO)
		dw_write(priv, DW_SPI_CTRLR1, cfg.ndf - 1);
	/*
	 * disable Chip Select Toggle Enable.
	 * spi0 mode
	 * Data Frame Size 8(bit 16:20)
	 * TMOD
	 * Motorola SPI Frame Format
	 */
	ctrl0 |= (((cfg.dfs - 1) & 0x1F)  << 16) |
		cfg.tmode << 8;

	if (op->data.nbytes) {
		ctrl0 &= ~(0x3 << 21);
		ctrl0 |= ((select_data_width(op->data.buswidth) & 0x3) << 21);
	}

	/* only have cmd */
	if (!op->addr.nbytes && !op->data.nbytes) {
		ctrl0 &= ~(0x3 << 21);
		ctrl0 |= ((select_data_width(op->cmd.buswidth) & 0x3) << 21);
	}
	dw_write(priv, DW_SPI_CTRLR0, ctrl0);

	/* Instruction Length: always 8bit inst */
	if (op->cmd.nbytes)
		spi_ctrl0 |= (2 << 8);

	/*  set Address Length */
	if (op->addr.nbytes)
		spi_ctrl0 |= ((op->addr.nbytes * 2) << 2);

	if (op->dummy.nbytes)
		spi_ctrl0 |= (op->dummy.nbytes * 8 / op->dummy.buswidth) << 11;

	if (op->cmd.buswidth && op->addr.buswidth) {
		/*
		 * Address and instruction transfer in Standard SPI Mode format
		 */
		if (op->cmd.buswidth == 1 && op->cmd.buswidth == op->addr.buswidth)
			spi_ctrl0 &= ~0x3;

		/*
		 * Address and instruction transfer format
		 * for 1-x-x, x != 1
		 */
		if (op->cmd.buswidth == 1 && op->cmd.buswidth != op->addr.buswidth)
			spi_ctrl0 |= 0x1;

		/*
		 *  QSPI mode or std mode
		 */
		if (op->cmd.buswidth != 1)
			spi_ctrl0 |= 0x2;
	}

	/* for only cmd */
	if (op->cmd.buswidth && !op->addr.buswidth)
		spi_ctrl0 |= 0x2;

	if (op->data.dtr == 1)
		spi_ctrl0 |= 1 << 16;

	dw_write(priv, DW_SPI_CTRLR0_EXT, spi_ctrl0);

	if (priv->rx_sample != param.cur_rx_delay) {
		dw_write(priv, DW_SPI_RX_SAMPLE_DLY, priv->rx_sample);
		param.cur_rx_delay = priv->rx_sample;
	}

	if (priv->txd_drive_edge != param.cur_txd_drive_edge) {
		dw_write(priv, DW_TXD_DRIVE_EDGE, priv->txd_drive_edge);
		param.cur_txd_drive_edge = priv->txd_drive_edge;
	}
}

#define SPI_GET_BYTE(_val, _idx) \
	((_val) >> (BITS_PER_BYTE * (_idx)) & 0xff)

void write_addr_data(struct spi_slave *slave, const struct spi_mem_op *op)
{
	u8 addr[8] = {0};
	int j = 0;
	u32 addr_len = op->addr.nbytes;
	struct udevice *bus = slave->dev->parent;
	struct dw_spi_priv *priv = dev_get_priv(bus);

	if (!op->addr.nbytes)
		return;

	/* higth first */
	for (j = 0; j < op->addr.nbytes; j++)
		addr[j] = SPI_GET_BYTE(op->addr.val, op->addr.nbytes - j - 1);

	switch (op->data.buswidth) {
	/* erase operation */
	case 0:
	case 1:
		for (j = 0; j < addr_len; j++)
			dw_write(priv, DW_SPI_DR, addr[j]);
		break;

	case 2:
		dw_write(priv, DW_SPI_DR, (u32)op->addr.val);
		break;

	case 4:
		dw_write(priv, DW_SPI_DR, (u32)op->addr.val);
		break;

	default:
		printf("error data width\n");
	}
}

static inline u32 dw_swap(u32 x, u8 size)
{
	switch (size) {
	case 4: // 32-bit swap
		return (((x) & 0x000000FF) << 24) |
				(((x) & 0x0000FF00) << 8) |
				(((x) & 0x00FF0000) >> 8) |
				(((x) & 0xFF000000) >> 24);
		break;
	case 2: // 16-bit swap
		return (((x) & 0x00FF) << 8) |
				(((x) & 0xFF00) >> 8);
		break;
	default:
		printf("Swap error: incorrect size\n");
		return x;
	}
}

int dw_spi_write_then_read(struct spi_slave *slave, const struct spi_mem_op *op)
{
	struct udevice *bus = slave->dev->parent;
	struct dw_spi_priv *priv = dev_get_priv(bus);
	u32 entries, room, len;
	u32 val = 0, tmp = 0xffffffff;
	void *in_buf;
	const void *out_buf;
	u8 data_width = priv->n_bytes;

	len = 0;
	in_buf = NULL;
	out_buf = NULL;
	u32 retry_num = 0;
	u32 cnt = 0;

	if (op->data.dir == SPI_MEM_DATA_OUT) {
		out_buf = op->data.buf.out;
		len = op->data.nbytes;
	}

	/* write cmd */
	if (op->cmd.nbytes)
		dw_write(priv, DW_SPI_DR, op->cmd.opcode);

	/* write addr */
	write_addr_data(slave, op);

	room = min((priv->fifo_len - dw_read(priv, DW_SPI_TXFLR)),  (len + data_width - 1) >> __builtin_ctz(data_width));
	while (room) {
		if (len < data_width) {
			tmp = 0xffffffff;
			memcpy(&tmp, out_buf, len);
			tmp = dw_swap(tmp, data_width);
			dw_write(priv, DW_SPI_DR, tmp);
			len -= len;
			out_buf += len;
		} else {
			if (data_width == 1)
				dw_write(priv, DW_SPI_DR, *(u8 *)out_buf);
			else if (data_width == 2) {
				dw_write(priv, DW_SPI_DR, *(u16 *)out_buf);
			} else {
				dw_write(priv, DW_SPI_DR, *(u32 *)out_buf);
			}
			len -= data_width;
			out_buf += data_width;
		}
		room -= 1;
	}

	dw_write(priv, DW_SPI_SER, 1);
	external_cs_manage(slave->dev, false);

	while (len > 0) {
		entries = dw_read(priv, DW_SPI_TXFLR);
		if (!entries) {
			retry_num++;
			if (dw_spi_check_status(priv, 1) || retry_num > 1000) {
				printf("CS de-assertion on Tx!\n");
				return -1;
			}
		}

		room = min(priv->fifo_len - entries, (len + data_width - 1) >> __builtin_ctz(data_width));
		for (; room; --room) {
			if (len < data_width) {
				tmp = 0xffffffff;
				memcpy(&tmp, out_buf, len);
				tmp = dw_swap(tmp, data_width);
				dw_write(priv, DW_SPI_DR, tmp);
				len -= len;
				out_buf += len;
			} else {
				if (data_width == 1)
					dw_write(priv, DW_SPI_DR, *(u8 *)out_buf);
				else if (data_width == 2) {
					dw_write(priv, DW_SPI_DR, *(u16 *)out_buf);
				} else {
					dw_write(priv, DW_SPI_DR, *(u32 *)out_buf);
				}
				len -= data_width;
				out_buf += data_width;
			}
		}
	}

	if (op->data.dir == SPI_MEM_DATA_IN) {
		len = op->data.nbytes;
		in_buf = op->data.buf.in;
	}
	while (len) {
		entries = 0;
		entries = dw_read(priv, DW_SPI_RXFLR);
		if (!entries) {
			if (cnt++ >= 50) {
				val = dw_read(priv, DW_SPI_RISR);
				cnt = 0;
				if (val & SPI_INT_RXOI) {
					printf("FIFO overflow on Rx\n");
					return -1;
				}
			}
			continue;
		}

		for (; entries; --entries) {
			if (data_width == 4) {
				if (len >= REG_IO_WIDTH) {
					*(u32 *)in_buf = dw_read(priv, DW_SPI_DR);
					in_buf += data_width;
					len -=  data_width;
				} else {
					val = dw_read(priv, DW_SPI_DR);
					memcpy(in_buf, &val, len);
					len -=  len;
				}
			} else {
				*(u8 *)in_buf++ = dw_read(priv, DW_SPI_DR);
				len -= data_width;
			}
		}
	}
	return 0;
}

// Spinor use CH2 and CH3
static void dma_channel_init(void)
{
	mmio_clrsetbits_32(TOP_DMA_CH_REMAP0, 0x3f << DMA_REMAP_CH2_OFFSET, DMA_RX_REQ_SPI_NOR << DMA_REMAP_CH2_OFFSET);
	mmio_clrsetbits_32(TOP_DMA_CH_REMAP0, 0x3f << DMA_REMAP_CH3_OFFSET, DMA_TX_REQ_SPI_NOR << DMA_REMAP_CH3_OFFSET);
	mmio_clrsetbits_32(TOP_DMA_CH_REMAP0, 0x1 << 31, 1 << DMA_REMAP_UPDATE_OFFSET);
}

bool can_dma(struct dw_spi_priv *priv, const struct spi_mem_op *op)
{
	if (op->data.nbytes > priv->fifo_len * priv->n_bytes)
		return true;
	return false;
}

static void dw_spi_nor_dma_setup(struct dw_spi_priv *priv, const struct spi_mem_op *op)
{
	u32 reg = 0;
	u32 trigger_len = 0;
	u8 *out_buf = NULL;
	void *in_buf = NULL;

	dma_channel_init();

	if (op->data.dir == SPI_MEM_DATA_OUT) {
		out_buf = (u8 *)op->data.buf.out;
		/* CPU fill 64 byte data to fifo */
		out_buf += PRE_FILL_SIZE;
		dma_mem2dev_setting((unsigned int *)out_buf, op->data.nbytes - PRE_FILL_SIZE, (unsigned int *)(priv->regs + DW_SPI_DR), DMA_SPI0);
	} else if (op->data.dir == SPI_MEM_DATA_IN) {
		in_buf = op->data.buf.in;
		dma_dev2mem_setting(in_buf, op->data.nbytes, (unsigned int *)(priv->regs + DW_SPI_DR), DMA_SPI0);
	}

	if (op->data.dir == SPI_MEM_DATA_OUT) {
		reg |= 1 << 1;
		trigger_len = op->cmd.nbytes + op->addr.nbytes;
		dw_write(priv, DW_SPI_DMACR, reg);
		dw_write(priv, DW_SPI_DMATDLR, 0xF);
	} else if (op->data.dir == SPI_MEM_DATA_IN) {
		reg |= 1 << 0;
		trigger_len = priv->fifo_len / 2;

		dw_write(priv, DW_SPI_DMACR, reg);
		dw_write(priv, DW_SPI_DMARDLR, trigger_len - 1);
	}
}

int dw_spinor_dma_transfer(struct spi_slave *slave, const struct spi_mem_op *op)
{
	struct udevice *bus = slave->dev->parent;
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret = 0;
	u32 room = 0, tx_len;
	u8 *buf = NULL;
	u32 len;

	if (op->cmd.nbytes)
		dw_write(priv, DW_SPI_DR, op->cmd.opcode);

	write_addr_data(slave, op);

	if (op->data.dir == SPI_MEM_DATA_OUT) {
		room = min((u32)(priv->fifo_len - dw_read(priv, DW_SPI_TXFLR)), (u32)PRE_FILL_SIZE / priv->n_bytes);//FIXME:There may be an issue with one line.
		buf = (u8 *)op->data.buf.out;
		while (room) {
			if (priv->n_bytes == 1)
				dw_write(priv, DW_SPI_DR, *buf);
			else
				dw_write(priv, DW_SPI_DR, *(u32 *)buf);

			buf += priv->n_bytes;
			room -= 1;
		}
		tx_len = op->data.nbytes - (buf - (u8 *)op->data.buf.out);
	}

	if (op->data.dir == SPI_MEM_DATA_IN)
		len = op->data.nbytes;
	else
		len = tx_len;

	dw_write(priv, DW_SPI_SER, 1);
	external_cs_manage(slave->dev, false);

	// wait dma transfer done
	if (op->data.dir == SPI_MEM_DATA_OUT)
		ret = dma_start_transfer(DMA_SPI0);

	if (op->data.dir == SPI_MEM_DATA_IN)
		ret = dma_start_receive(DMA_SPI0);

	return ret;
}

int dw_spi_wait_mem_op_done(struct dw_spi_priv *priv)
{
#define NSEC_PER_USEC   1000L
#define NSEC_PER_SEC    1000000000L
#define SPI_WAIT_RETRIES	5000

	int retry = SPI_WAIT_RETRIES;
	unsigned long ns, delay;
	u32 nents;

	nents = dw_read(priv, DW_SPI_TXFLR);
	ns = NSEC_PER_SEC / priv->freq * nents;
	ns *= priv->n_bytes * 8;
	/* unit: us */
	if (ns <= NSEC_PER_USEC)
		delay = 1;
	else
		delay = (ns + NSEC_PER_USEC - 1) / NSEC_PER_USEC;

	while ((dw_read(priv, DW_SPI_SR) & SR_BUSY) && retry--)
		udelay(delay);

	if (retry < 0) {
		pr_err("dw_spi_wait_mem_op_done: timeout!\n");
		return -1;
	}
	return 0;
}

void handle_data_for_write(const u8 *out, const struct spi_mem_op *op)
{
	u8 *buf = (u8 *)out;
	u32 len = op->data.nbytes;
	int i;

	if (op->data.dir == SPI_MEM_DATA_IN)
		return;

	if (op->data.nbytes > 4 && op->data.buswidth > 1) {
		for (i = 0; i < len / 4; i++) {
			swap(buf[0], buf[3]);
			swap(buf[1], buf[2]);
			buf += 4;
		}
	}

	//switch (len % 4) {
	//case 3:
	//	swap(buf[0], buf[2]);
	//case 2:
	//	swap(buf[0], buf[1]);
	//	break;
	//case 1:
	//default:
	//	break;
	//}
}

static int dw_spi_init_mem_buf(const struct spi_mem_op *op)
{
	if (op->data.dir == SPI_MEM_DATA_OUT)
		handle_data_for_write(op->data.buf.out, op);

	return 0;
}

/*
 * This function is necessary for reading SPI flash with the native CS
 * c.f. https://lkml.org/lkml/2015/12/23/132
 */
static int dw_spi_exec_op(struct spi_slave *slave, const struct spi_mem_op *op)
{
	int ret = 0;
	struct udevice *bus = slave->dev->parent;
	struct dw_spi_priv *priv = dev_get_priv(bus);
	bool support_dma = false;

	/*
	 * Collect the outbound data into a single buffer to speed the
	 * transmission up at least on the initial stage.
	 */
	ret = dw_spi_init_mem_buf(op);
	if (ret)
		return ret;

	dw_write(priv, DW_SPI_SSIENR, 0);
	spi_update_config(slave, op);

	support_dma = can_dma(priv, op);
	if (support_dma)
		dw_spi_nor_dma_setup(priv, op);

	dw_write(priv, DW_SPI_SSIENR, 1);

	if (support_dma) {
		ret = dw_spinor_dma_transfer(slave, op);
		dw_write(priv, DW_SPI_DMACR, 0);
	} else {
		ret = dw_spi_write_then_read(slave, op);
	}

	if (unlikely(ret))
		ret = dw_spi_check_status(priv, true);
	else
		ret = dw_spi_wait_mem_op_done(priv);

	dw_write(priv, DW_SPI_SER, 0);
	external_cs_manage(slave->dev, true);

	// dev_dbg(bus, "%u bytes xfered\n", op->data.nbytes);
	return ret;
}

/* The size of ctrl1 limits data transfers to 64K */
static int dw_spi_adjust_op_size(struct spi_slave *slave, struct spi_mem_op *op)
{
	op->data.nbytes = min(op->data.nbytes, (unsigned int)SZ_64K);

	return 0;
}

static const struct spi_controller_mem_ops dw_spi_mem_ops = {
	.exec_op = dw_spi_exec_op,
	.adjust_op_size = dw_spi_adjust_op_size,
};

static int dw_spi_set_speed(struct udevice *bus, uint speed)
{
	struct dw_spi_plat *plat = dev_get_plat(bus);
	struct dw_spi_priv *priv = dev_get_priv(bus);
	u16 clk_div;

	if (speed > plat->frequency)
		speed = plat->frequency;

	/* Disable controller before writing control registers */
	dw_write(priv, DW_SPI_SSIENR, 0);

	/* clk_div doesn't support odd number */
	clk_div = priv->bus_clk_rate / speed;
	clk_div = (clk_div + 1) & 0xfffe;
	dw_write(priv, DW_SPI_BAUDR, clk_div);

	/* Enable controller after writing control registers */
	dw_write(priv, DW_SPI_SSIENR, 1);

	priv->freq = speed;
	dev_dbg(bus, "speed=%d clk_div=%d\n", priv->freq, clk_div);

	return 0;
}

static int dw_spi_set_mode(struct udevice *bus, uint mode)
{
	struct dw_spi_priv *priv = dev_get_priv(bus);

	/*
	 * Can't set mode yet. Since this depends on if rx, tx, or
	 * rx & tx is requested. So we have to defer this to the
	 * real transfer function.
	 */
	priv->mode = mode;
	dev_dbg(bus, "mode=%d\n", priv->mode);

	return 0;
}

static int dw_spi_remove(struct udevice *bus)
{
	struct dw_spi_priv *priv = dev_get_priv(bus);
	int ret;

	ret = reset_release_bulk(&priv->resets);
	if (ret)
		return ret;

#if CONFIG_IS_ENABLED(CLK)
	ret = clk_disable(&priv->clk);
	if (ret)
		return ret;

	ret = clk_free(&priv->clk);
	if (ret)
		return ret;
#endif
	return 0;
}

void dw_spi_set_param(struct udevice *bus, unsigned int *param)
{
	struct dw_spi_priv *priv = dev_get_priv(bus);

	priv->rx_sample = param[0];
	priv->txd_drive_edge = param[1];
}

#ifdef CONFIG_ENABLE_SPINOR_TUNING
void dw_spi_get_tuning_param(struct udevice *bus, struct tuning_ops *tuning_param)
{
	struct dw_spi_priv *priv = dev_get_priv(bus);

	tuning_param->param_num = 2;

	//RX_SAMPLE_DLY:
	tuning_param->param_ranges[0].min = 0;
	tuning_param->param_ranges[0].max = 255;

	//TXD_DRIVE_EDGE:
	tuning_param->param_ranges[0].min = 0;
	tuning_param->param_ranges[0].max = (dw_read(priv, DW_SPI_BAUDR) / 2);

	tuning_param->old_param[0] = dw_read(priv, DW_SPI_RX_SAMPLE_DLY);
	tuning_param->old_param[1] = dw_read(priv, DW_TXD_DRIVE_EDGE);
}

int dw_spi_tuning_fail_policy(struct udevice *bus, int retry, struct tuning_ops *tuning_param)
{
	unsigned int param[2];

	if (retry < 1) {
		dw_spi_set_speed(bus, 50000000);
		pr_err("Tuning fail policy: speed = 50M\n");
	} else {
		//Use old param
		param[0] = tuning_param->old_param[0];
		param[1] = tuning_param->old_param[1];
		dw_spi_set_param(bus, param);
		pr_err("Tuning fail policy: use old param\n");
		return -1;
	}
	return 0;
}
#endif

static const struct dm_spi_ops dw_spi_ops = {
	.xfer		= dw_spi_xfer,
	.mem_ops	= &dw_spi_mem_ops,
	.set_speed	= dw_spi_set_speed,
	.set_mode	= dw_spi_set_mode,
	.param_set = dw_spi_set_param,
#ifdef CONFIG_ENABLE_SPINOR_TUNING
	.tuning_param_get = dw_spi_get_tuning_param,
	.tuning_fail_policy = dw_spi_tuning_fail_policy,
#endif
	/*
	 * cs_info is not needed, since we require all chip selects to be
	 * in the device tree explicitly
	 */
};

static const struct udevice_id dw_spi_ids[] = {
	/* Generic compatible strings */

	{ .compatible = "snps,dw-apb-ssi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dw-apb-ssi-3.20a", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dw-apb-ssi-3.22a", .data = (ulong)dw_spi_apb_init },
	/* First version with SSI_MAX_XFER_SIZE */
	{ .compatible = "snps,dw-apb-ssi-3.23a", .data = (ulong)dw_spi_apb_init },
	/* First version with Dual/Quad SPI; unused by this driver */
	{ .compatible = "snps,dw-apb-ssi-4.00a", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dw-apb-ssi-4.01", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dw-apb-ssi-4.04a", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,dwc-ssi-1.01a", .data = (ulong)dw_spi_dwc_init },

	/* Compatible strings for specific SoCs */

	/*
	 * Both the Cyclone V and Arria V share a device tree and have the same
	 * version of this device. This compatible string is used for those
	 * devices, and is not used for sofpgas in general.
	 */
	{ .compatible = "altr,socfpga-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "altr,socfpga-arria10-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "canaan,kendryte-k210-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "canaan,kendryte-k210-ssi", .data = (ulong)dw_spi_dwc_init },
	{ .compatible = "intel,stratix10-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "intel,agilex-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "mscc,ocelot-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "mscc,jaguar2-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,axs10x-spi", .data = (ulong)dw_spi_apb_init },
	{ .compatible = "snps,hsdk-spi", .data = (ulong)dw_spi_apb_init },
	{ }
};

U_BOOT_DRIVER(dw_spi) = {
	.name = "dw_spi",
	.id = UCLASS_SPI,
	.of_match = dw_spi_ids,
	.ops = &dw_spi_ops,
	.of_to_plat = dw_spi_of_to_plat,
	.plat_auto	= sizeof(struct dw_spi_plat),
	.priv_auto	= sizeof(struct dw_spi_priv),
	.probe = dw_spi_probe,
	.remove = dw_spi_remove,
};
