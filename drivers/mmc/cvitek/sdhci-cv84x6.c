// SPDX-License-Identifier: GPL-2.0+
/*
 * cv84x6 eMMC/SD/SDIO SDHCI Platform Driver for U-Boot
 *
 * Copyright (C) Sophgo 2025-2026
 */

#include <common.h>
#include <dm.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/sizes.h>
#include <linux/libfdt.h>
#include <reset.h>
#include <mmc.h>
#include <sdhci.h>
#include <mmio.h>
#include <command.h>

static inline void mmio_setbits_16(uintptr_t addr, uint16_t set)
{
	mmio_write_16(addr, mmio_read_16(addr) | set);
}

static inline void mmio_clrbits_16(uintptr_t addr, uint16_t clear)
{
	mmio_write_16(addr, mmio_read_16(addr) & ~clear);
}

#define MMC_TYPE_MMC 0
#define MMC_TYPE_SD 1
#define MMC_TYPE_SD1 2
#define MMC_TYPE_SDIO 3

/* Host Control 2 bit4: sample clock select for tuning */
#define SDHCI_CTRL_SAMPLE_CLK_SEL BIT(4)

#ifdef DEBUG
#define pr_debug(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif

#define DWC_MSHC_CTRL_R (0x8)
#define VENDOR_EMMC_CTRL_R (0x2c)
#define EMMC_CARD_IS_EMMC BIT(0)
#define EMMC_RST_N BIT(2)
#define DWC_MSHC_NEGEDGE_DATAOUT_EN BIT(2) /* MSHC_CTRL_R bit2: HS400 negedge data out */
#define ENH_STROBE_ENABLE BIT(8) /* EMMC_CTRL_R bit8: HS400 enhanced strobe */
#define SDHCI_CLOCK_PLL_EN BIT(3)
#define DEFAULT_DIV_EMMC_INIT_CLOCK 0x2
#define EMMC_INIT_FREQ_HZ (200 * 1000)

/* ===== DWC_mshc PHY DLL Registers at PHY+0x300 (for HS400) ===== */
#define SDHCI_PHY_DLL_CTRL	(SDHCI_PHY_R_OFFSET + 0x24)
#define SDHCI_PHY_DLL_CNFG1	(SDHCI_PHY_R_OFFSET + 0x25)
#define SDHCI_PHY_DLL_CNFG2	(SDHCI_PHY_R_OFFSET + 0x26)
#define SDHCI_PHY_DLLDL_CNFG	(SDHCI_PHY_R_OFFSET + 0x28)
#define SDHCI_PHY_DLL_STATUS	(SDHCI_PHY_R_OFFSET + 0x2E)
#define DLL_CTRL_DLL_EN		BIT(0)
#define DLL_STATUS_LOCK_STS	BIT(0)
#define DLL_STATUS_ERR_STS	BIT(1)

#define MAX_TUNING_CMD_RETRY_COUNT 50
#define TUNE_MAX_PHCODE 128

/* ===== Data Structures ===== */
struct cvi_sdhci_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct cvi_sdhci_host {
	struct sdhci_host host;
	int no_1_8_v;
	int is_64_addressing;
	int reset_tx_rx_phy;
	u32 mmc_fmax_freq;
	u32 mmc_fmin_freq;
	u8 final_tap;
	u32 vendor_base; /* P_VENDOR_SPECIFIC_AREA + ioaddr */
	struct reset_ctl reset_ctl;
};

struct cvi_sdhci_driver_data {
	const struct sdhci_ops *ops;
	int index;
};

/* ===== Helper: Get vendor register base ===== */
static u32 cvi_get_vendor_base(struct sdhci_host *host)
{
	u16 vendor_area = sdhci_readw(host, 0xE8); /* P_VENDOR_SPECIFIC_AREA */

	return (u32)(uintptr_t)host->ioaddr + (vendor_area & 0xFFF);
}

#define SDHCI_PHY_R_OFFSET 0x300
#define SDHCI_P_PHY_CNFG (SDHCI_PHY_R_OFFSET + 0x00)
#define SDHCI_P_CMDPAD_CNFG (SDHCI_PHY_R_OFFSET + 0x04)
#define SDHCI_P_DATPAD_CNFG (SDHCI_PHY_R_OFFSET + 0x06)
#define SDHCI_P_CLKPAD_CNFG (SDHCI_PHY_R_OFFSET + 0x08)
#define SDHCI_P_STBPAD_CNFG (SDHCI_PHY_R_OFFSET + 0x0A)
#define SDHCI_P_RSTNPAD_CNFG (SDHCI_PHY_R_OFFSET + 0x0C)
#define SDHCI_P_COMMDL_CNFG (SDHCI_PHY_R_OFFSET + 0x1C)
#define SDHCI_P_SDCLKDL_CNFG (SDHCI_PHY_R_OFFSET + 0x1D)
#define SDHCI_P_SDCLKDL_DC (SDHCI_PHY_R_OFFSET + 0x1E)
#define SDHCI_P_SMPLDL_CNFG (SDHCI_PHY_R_OFFSET + 0x20)
#define SDHCI_P_ATDL_CNFG (SDHCI_PHY_R_OFFSET + 0x21)

#define PHY_CNFG_PHY_RSTN 0
#define PHY_CNFG_PHY_PWRGOOD 1
#define PHY_CNFG_PAD_SN 20
#define PHY_CNFG_PAD_SP 16

#define PAD_CNFG_RXSEL 0
#define PAD_CNFG_WEAKPULL_EN 3
#define PAD_CNFG_TXSLEW_CTRL_P 5
#define PAD_CNFG_TXSLEW_CTRL_N 9

#define SDCLKDL_CNFG_EXTDLY_EN 0
#define SMPLDL_CNFG_BYPASS_EN 1
#define ATDL_CNFG_INPSEL_CNFG 2

static int cvi_sdhci_wait_clk_stable(struct sdhci_host *host)
{
	int timeout = 200;

	while (!(sdhci_readw(host, SDHCI_CLOCK_CONTROL) &
		 SDHCI_CLOCK_INT_STABLE)) {
		if (timeout-- <= 0) {
			printf("mmc%d: clk stable timeout clk=0x%x\n",
			       host->index,
			       sdhci_readw(host, SDHCI_CLOCK_CONTROL));
			return -ETIMEDOUT;
		}
		udelay(100);
	}
	return 0;
}

static int cvi_sdhci_enable_pll(struct sdhci_host *host)
{
	u16 clk;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	if (!(clk & SDHCI_CLOCK_INT_EN))
		return 0;
	if (clk & SDHCI_CLOCK_PLL_EN)
		return 0;

	sdhci_writew(host, clk | SDHCI_CLOCK_PLL_EN, SDHCI_CLOCK_CONTROL);
	udelay(150);
	return cvi_sdhci_wait_clk_stable(host);
}

static int cvi_sdhci_enable_int_clk(struct sdhci_host *host)
{
	u16 clk;

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= ~0x9;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

	clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
	clk &= 0x3f;
	clk |= (DEFAULT_DIV_EMMC_INIT_CLOCK << SDHCI_DIVIDER_SHIFT) |
	       SDHCI_CLOCK_INT_EN;
	sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
	udelay(150);

	if (cvi_sdhci_wait_clk_stable(host))
		return -ETIMEDOUT;

	return cvi_sdhci_enable_pll(host);
}

static int cvi_phy_wait_pwrgood(struct sdhci_host *host, const char *name)
{
	int loop = 100;

	while (!(sdhci_readl(host, SDHCI_P_PHY_CNFG) &
		 BIT(PHY_CNFG_PHY_PWRGOOD))) {
		if (loop-- <= 0) {
			printf("%s PHY power good timeout pstate=0x%x phy=0x%x\n",
			       name, sdhci_readl(host, SDHCI_PRESENT_STATE),
			       sdhci_readl(host, SDHCI_P_PHY_CNFG));
			return -ETIMEDOUT;
		}
		mdelay(10);
	}
	return 0;
}

static int emmc_phy_pad_setting(struct sdhci_host *host)
{
	u32 val;

	if (cvi_phy_wait_pwrgood(host, "eMMC"))
		return -ETIMEDOUT;

	val = sdhci_readl(host, SDHCI_P_PHY_CNFG);
	val &= ~BIT(PHY_CNFG_PHY_RSTN);
	sdhci_writel(host, val, SDHCI_P_PHY_CNFG);

	sdhci_writel(host,
		     BIT(PHY_CNFG_PHY_PWRGOOD) | (0xA << PHY_CNFG_PAD_SP) |
			     (0xA << PHY_CNFG_PAD_SN),
		     SDHCI_P_PHY_CNFG);

	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x1 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_CMDPAD_CNFG);
	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x1 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_DATPAD_CNFG);
	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x0 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_CLKPAD_CNFG);
	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x2 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_STBPAD_CNFG);
	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x1 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_RSTNPAD_CNFG);

	sdhci_writeb(host, 0x0, SDHCI_P_COMMDL_CNFG);
	sdhci_writeb(host, 0x10, SDHCI_P_SDCLKDL_CNFG);
	sdhci_writeb(host, 0x3f, SDHCI_P_SDCLKDL_DC);
	sdhci_writeb(host, 0x11, SDHCI_P_SDCLKDL_CNFG);
	sdhci_writeb(host, 0x1, SDHCI_P_SDCLKDL_CNFG);
	sdhci_writeb(host, 0xC, SDHCI_P_SMPLDL_CNFG);
	sdhci_writeb(host, 0xC, SDHCI_P_ATDL_CNFG);

	val = sdhci_readl(host, SDHCI_P_PHY_CNFG);
	val |= BIT(PHY_CNFG_PHY_RSTN);
	sdhci_writel(host, val, SDHCI_P_PHY_CNFG);
	return 0;
}

static void cvi_sdhci_enable_irq(struct sdhci_host *host)
{
	sdhci_writel(host, SDHCI_INT_DATA_MASK | SDHCI_INT_CMD_MASK,
		     SDHCI_INT_ENABLE);
	sdhci_writel(host, 0x0, SDHCI_SIGNAL_ENABLE);
	sdhci_writel(host, SDHCI_INT_ALL_MASK, SDHCI_INT_STATUS);
}

/* ===== DTS Parsing ===== */
static int cvi_ofdata_to_platdata(struct udevice *dev)
{
	struct cvi_sdhci_host *cvi_host = dev_get_priv(dev);
	struct sdhci_host *host = &cvi_host->host;
	int node = dev_of_offset(dev);

	host->name = strdup(dev->name);
	host->ioaddr = (void *)devfdt_get_addr(dev);

	if (host->ioaddr == (void *)FDT_ADDR_T_NONE)
		return -EINVAL;

	host->bus_width = fdtdec_get_int(gd->fdt_blob, node, "bus-width", 8);
	host->max_clk = fdtdec_get_uint(gd->fdt_blob, node, "src-frequency", 0);

	cvi_host->mmc_fmin_freq =
		fdtdec_get_uint(gd->fdt_blob, node, "min-frequency", 200000);
	cvi_host->mmc_fmax_freq =
		fdtdec_get_uint(gd->fdt_blob, node, "max-frequency", 0);
	cvi_host->is_64_addressing =
		fdtdec_get_bool(gd->fdt_blob, node, "64_addressing");
	cvi_host->reset_tx_rx_phy =
		fdtdec_get_bool(gd->fdt_blob, node, "reset_tx_rx_phy");
	cvi_host->no_1_8_v = fdtdec_get_bool(gd->fdt_blob, node, "no-1-8-v");

	if (cvi_host->no_1_8_v)
		host->quirks |= SDHCI_QUIRK_NO_1_8_V;

	return 0;
}

static int cvi_sdhci_bind(struct udevice *dev)
{
	struct cvi_sdhci_plat *plat = dev_get_plat(dev);

	return sdhci_bind(dev, &plat->mmc, &plat->cfg);
}

/* ===== SD0/SD1 PHY Pad Configuration ===== */
static void sdio_phy_pad_setting(struct sdhci_host *host)
{
	sdhci_writel(host,
		     BIT(PHY_CNFG_PHY_PWRGOOD) | (0xC << PHY_CNFG_PAD_SP) |
			     (0xC << PHY_CNFG_PAD_SN),
		     SDHCI_P_PHY_CNFG);

	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x1 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_CMDPAD_CNFG);
	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x1 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_DATPAD_CNFG);
	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x0 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_CLKPAD_CNFG);
	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x2 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_STBPAD_CNFG);
	sdhci_writew(host,
		     (0x6 << PAD_CNFG_RXSEL) | (0x1 << PAD_CNFG_WEAKPULL_EN) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_P) |
			     (0x2 << PAD_CNFG_TXSLEW_CTRL_N),
		     SDHCI_P_RSTNPAD_CNFG);

	sdhci_writeb(host, 0x0, SDHCI_P_COMMDL_CNFG);
	sdhci_writeb(host, 0x10, SDHCI_P_SDCLKDL_CNFG);
	sdhci_writeb(host, 0x0, SDHCI_P_SDCLKDL_DC);
	sdhci_writeb(host, 0x11, SDHCI_P_SDCLKDL_CNFG);
	sdhci_writeb(host, 0x1, SDHCI_P_SDCLKDL_CNFG);
	sdhci_writeb(host, 0xC, SDHCI_P_SMPLDL_CNFG);
	sdhci_writeb(host, 0xC, SDHCI_P_ATDL_CNFG);
}

/* ===== DWC_mshc Tuning Registers ===== */
#define DWC_AT_CTRL_R (0x40) /* Auto-Tuning Control, 32-bit */
#define DWC_AT_STAT_R (0x44) /* Auto-Tuning Status, 32-bit */

/* DWC_AT_CTRL_R bit fields */
#define AT_SW_TUNE_EN BIT(4)
#define AT_TUNE_CLK_STOP_EN BIT(16)

/* DWC_AT_STAT_R bit fields */
#define AT_CENTER_PH_CODE_MASK 0xFF /* [7:0] Center phase code */

/* ===== Tuning Support (per DWC_mshc User Guide Section 8.6.4) ===== */
#ifdef CONFIG_MMC_SUPPORTS_TUNING
/*
 * Software Tuning Flow (User Guide Figure 8-14):
 *	1. Reset tuning engine: HOST_CTRL2_R.SAMPLE_CLK_SEL=0
 *	2. Setup registers for CMD21/CMD19: BLOCK_SIZE_R, BLOCKCOUNT_R, XFER_MODE_R
 *	3. Enable software tuning: AT_CTRL_R.SW_TUNE_EN=1
 *	4. Set AT_STAT_R.CENTER_PH_CODE=0
 *	5. Send CMD21/CMD19
 *	6. Wait for BUF_RD_READY or ERROR_INT_STAT_R[6:0]
 *	7. If error: clear ERROR_INTERRUPT, clear errors, SW_RST_DAT+SW_RST_CMD,
 *	   wait for reset, increment CENTER_PH_CODE, goto 5
 *	8. If no error: clear BUF_RD_READY, record phase as passing
 *	9. Turn off SD_CLK_EN, increment CENTER_PH_CODE+1, turn on SD_CLK_EN
 *	10. All phases exhausted? If no: goto 5
 *	11. Find center of widest passing window, program CENTER_PH_CODE
 *	12. Disable SW_TUNE_EN, set SAMPLE_CLK_SEL=1
 */
void cvi_tuning_setup_transfer(struct sdhci_host *host)
{
	/* Step 3: Configure BLOCK_SIZE, BLOCK_COUNT, and TRANSFER_MODE.
	 * Tuning block size depends on bus width:
	 *   8-bit: 128 bytes (tuning_blk_pattern_8bit)
	 *   4-bit: 64 bytes  (tuning_blk_pattern_4bit)
	 */
	u16 blk_sz = (host->mmc->bus_width == 8) ? 128 : 64;

	sdhci_writew(host, SDHCI_MAKE_BLKSZ(7, blk_sz), SDHCI_BLOCK_SIZE);
	sdhci_writew(host, 1, SDHCI_BLOCK_COUNT);
	sdhci_writew(host, SDHCI_TRNS_READ, SDHCI_TRANSFER_MODE);
}

void cvi_tuning_phase_recovery(struct sdhci_host *host)
{
	int timeout = 1000;

	/* Clear error interrupts */
	sdhci_writew(host, 0xFFFF, SDHCI_INT_STATUS);
	sdhci_writew(host, 0xFFFF, SDHCI_ERR_INT_STATUS);

	/* Step 7: SW_RST_DAT + SW_RST_CMD to clear buffered tuning block */
	sdhci_writeb(host,
		     sdhci_readb(host, SDHCI_SOFTWARE_RESET) | BIT(2) | BIT(1),
		     SDHCI_SOFTWARE_RESET);

	/* Wait for reset to complete */
	while (sdhci_readb(host, SDHCI_SOFTWARE_RESET) & 0x6) {
		if (--timeout <= 0)
			break;
		udelay(1);
	}
	/* Clear reset bits */
	sdhci_writeb(host,
		     sdhci_readb(host, SDHCI_SOFTWARE_RESET) &
			     ~(BIT(2) | BIT(1)),
		     SDHCI_SOFTWARE_RESET);
}

int cvi_tuning_try_phase(struct sdhci_host *host, u8 opcode)
{
	/*
	 * DWC_mshc Software Tuning Sequence (Figure 8-14):
	 * Steps 7-11: Send CMD21, then wait for EITHER
	 *   BUF_RD_READY (data matched) OR
	 *   ERROR_INT_STAT_R[6:0] (data failed).
	 * No separate CMD response polling — in SW_TUNE_EN mode,
	 * the controller handles CMD+data internally as one sequence.
	 */
	u16 mode;
	int retry;
	u16 int_status, err_status;

	/* Clear pending interrupts */
	sdhci_writew(host, 0xFFFF, SDHCI_INT_STATUS);
	sdhci_writew(host, 0xFFFF, SDHCI_ERR_INT_STATUS);

	/* Step 7: Send CMD21/CMD19 — R1 response + data present */
	sdhci_writel(host, 0, SDHCI_ARGUMENT);
	mode = SDHCI_CMD_RESP_SHORT | SDHCI_CMD_CRC | SDHCI_CMD_INDEX |
	       SDHCI_CMD_DATA;
	sdhci_writew(host, SDHCI_MAKE_CMD(opcode, mode), SDHCI_COMMAND);

	/* Steps 8-9: Wait for BUF_RD_READY or ERROR_INT_STAT_R[6:0] */
	retry = 5000;
	while (1) {
		int_status = sdhci_readw(host, SDHCI_INT_STATUS);
		err_status = sdhci_readw(host, SDHCI_ERR_INT_STATUS);

		/* Check error bits [6:0] in ERR_INT_STATUS */
		if (err_status & 0x7F) {
			if (int_status & SDHCI_INT_ERROR)
				sdhci_writew(host, 0xFFFF, SDHCI_ERR_INT_STATUS);
			cvi_tuning_phase_recovery(host);
			return -EIO;
		}

		/* BUF_RD_READY = data received and matched */
		if (int_status & SDHCI_INT_DATA_AVAIL) {
			sdhci_writew(host, SDHCI_INT_DATA_AVAIL,
				     SDHCI_INT_STATUS);
			break;
		}

		if (--retry <= 0) {
			printf("  tuning: timeout, int=0x%x err=0x%x pstate=0x%x\n",
			       int_status, err_status,
			       sdhci_readl(host, SDHCI_PRESENT_STATE));
			cvi_tuning_phase_recovery(host);
			return -ETIMEDOUT;
		}
		udelay(10);
	}

	/* Step 10-11: Clear and reset for next phase */
	cvi_tuning_phase_recovery(host);

	return 0;
}

void cvi_tuning_set_phase(struct sdhci_host *host, u32 vendor_base, u8 phase)
{
	u32 at_stat;
	int retry;

	/* Databook Figure 8-14 Step 12:
	 * With TUNE_CLK_STOP_EN=1, the controller auto-stops the clock.
	 * Write CENTER_PH_CODE and read back to confirm it took effect.
	 */
	at_stat = mmio_read_32(vendor_base + DWC_AT_STAT_R);
	at_stat &= ~AT_CENTER_PH_CODE_MASK;
	at_stat |= (phase & AT_CENTER_PH_CODE_MASK);
	mmio_write_32(vendor_base + DWC_AT_STAT_R, at_stat);

	/* Wait for CENTER_PH_CODE write to take effect (databook: 180 hclk or read-back) */
	retry = 100;
	do {
		at_stat = mmio_read_32(vendor_base + DWC_AT_STAT_R);
		if ((at_stat & AT_CENTER_PH_CODE_MASK) ==
		    (phase & AT_CENTER_PH_CODE_MASK))
			break;
		udelay(1);
	} while (--retry > 0);
}

int cvi_general_execute_tuning(struct mmc *mmc, u8 opcode)
{
	struct sdhci_host *host = mmc->priv;
	struct cvi_sdhci_host *cvi_host =
		container_of(host, struct cvi_sdhci_host, host);
	u32 vendor_base = cvi_host->vendor_base;
	u16 ctrl2;
	u32 at_ctrl;
	u8 phase;
	u8 pass_map[TUNE_MAX_PHCODE / 8];
	u8 best_start = 0, best_len = 0;
	u8 cur_start = 0, cur_len = 0;
	bool in_window = false;
	int dl2_retry = 0;

	memset(pass_map, 0, sizeof(pass_map));

	ctrl2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	ctrl2 &= ~SDHCI_CTRL_SAMPLE_CLK_SEL;
	sdhci_writew(host, ctrl2, SDHCI_HOST_CONTROL2);

	/* Step 3: Pre-configure BLOCK_SIZE/COUNT/XFER_MODE for CMD21 */
	cvi_tuning_setup_transfer(host);

	at_ctrl = mmio_read_32(vendor_base + DWC_AT_CTRL_R);
	at_ctrl |= AT_SW_TUNE_EN | AT_TUNE_CLK_STOP_EN;
	mmio_write_32(vendor_base + DWC_AT_CTRL_R, at_ctrl);

	mmio_write_32(vendor_base + DWC_AT_STAT_R, 0);
	/* Allow tuning circuit to initialize */
	udelay(100);

	for (phase = 0; phase < TUNE_MAX_PHCODE; phase++) {
		cvi_tuning_set_phase(host, vendor_base, phase);

		if (!cvi_tuning_try_phase(host, opcode))
			pass_map[phase / 8] |= (1 << (phase % 8));
	}

	for (phase = 0; phase < TUNE_MAX_PHCODE; phase++) {
		bool pass = pass_map[phase / 8] & (1 << (phase % 8));

		if (pass && !in_window) {
			in_window = true;
			cur_start = phase;
			cur_len = 1;
		} else if (pass && in_window) {
			cur_len++;
		} else if (!pass && in_window) {
			in_window = false;
			if (cur_len > best_len) {
				best_len = cur_len;
				best_start = cur_start;
			}
		}
	}
	if (in_window && cur_len > best_len) {
		best_len = cur_len;
		best_start = cur_start;
	}

	if (best_len == 0) {
		int p, cnt = 0;
		for (p = 0; p < TUNE_MAX_PHCODE; p++)
			if (pass_map[p / 8] & (1 << (p % 8)))
				cnt++;
		printf("  tuning: %d/%d phases passed, no window found\n",
		       cnt, TUNE_MAX_PHCODE);
		/* Disable SW tuning mode before returning! */
		at_ctrl &= ~(AT_SW_TUNE_EN | AT_TUNE_CLK_STOP_EN);
		mmio_write_32(vendor_base + DWC_AT_CTRL_R, at_ctrl);
		cvi_tuning_phase_recovery(host);
		/* Full reset: SW_RST_ALL + power cycle to clear stuck inhibit */
		sdhci_writeb(host, SDHCI_RESET_ALL, SDHCI_SOFTWARE_RESET);
		while (sdhci_readb(host, SDHCI_SOFTWARE_RESET))
			udelay(100);
		/* Power cycle: toggle bus power to force PHY reset */
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
		udelay(100);
		sdhci_writeb(host, 0xe, SDHCI_POWER_CONTROL);
		udelay(100);
		/* Re-enable internal clock */
		cvi_sdhci_enable_int_clk(host);
		/* Re-apply PHY pad config destroyed by SW_RST_ALL */
		if (host->index == MMC_TYPE_MMC)
			emmc_phy_pad_setting(host);
		else
			sdio_phy_pad_setting(host);
		/* Verify inhibit bits are cleared */
		if (sdhci_readl(host, SDHCI_PRESENT_STATE) &
		    (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT))
			printf("  tuning: WARN inhibit still stuck after reset!\n");
		return -EIO;
	}

	phase = best_start + best_len / 2;

	printf("mmc%d tuning: window [%d-%d] len=%d, center phase=%d\n",
	       host->index, best_start, best_start + best_len - 1,
	       best_len, phase);

	/* Wait for DL2_BUSY to clear before writing the final CENTER_PH_CODE */
	while (mmio_read_32(vendor_base + DWC_AT_STAT_R) & BIT(31)) {
		if (++dl2_retry > 1000)
			break;
		udelay(1);
	}

	cvi_tuning_set_phase(host, vendor_base, phase);

	at_ctrl &= ~(AT_SW_TUNE_EN | AT_TUNE_CLK_STOP_EN);
	mmio_write_32(vendor_base + DWC_AT_CTRL_R, at_ctrl);

	ctrl2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);
	ctrl2 |= SDHCI_CTRL_SAMPLE_CLK_SEL;
	sdhci_writew(host, ctrl2, SDHCI_HOST_CONTROL2);

	return 0;
}
#endif /* CONFIG_MMC_SUPPORTS_TUNING */

static int sdhci_cvi_set_ios_post(struct sdhci_host *host)
{
	struct mmc *mmc = (struct mmc *)host->mmc;
	u32 ctrl;
	u8 hc;

	/* If CMD/DATA inhibit bits are stuck, force a full reset */
	if (sdhci_readl(host, SDHCI_PRESENT_STATE) &
	    (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT)) {
		u16 clk;

		printf("mmc%d: inhibit stuck in set_ios, resetting... pstate=0x%x\n",
		       host->index,
		       sdhci_readl(host, SDHCI_PRESENT_STATE));
		sdhci_writeb(host, SDHCI_RESET_ALL, SDHCI_SOFTWARE_RESET);
		while (sdhci_readb(host, SDHCI_SOFTWARE_RESET))
			udelay(100);
		sdhci_writeb(host, 0, SDHCI_POWER_CONTROL);
		udelay(100);
		sdhci_writeb(host, 0xe, SDHCI_POWER_CONTROL);
		udelay(100);
		/* Also reset the eMMC card via RST_n pin */
		if (host->index == MMC_TYPE_MMC) {
			struct cvi_sdhci_host *ch =
				container_of(host, struct cvi_sdhci_host, host);
			mmio_clrbits_16(ch->vendor_base + VENDOR_EMMC_CTRL_R,
					EMMC_RST_N);
			mdelay(1);
			mmio_setbits_16(ch->vendor_base + VENDOR_EMMC_CTRL_R,
					EMMC_RST_N);
			mdelay(1);
		}
		cvi_sdhci_enable_int_clk(host);
		/* Re-apply PHY pad config destroyed by SW_RST_ALL */
		if (host->index == MMC_TYPE_MMC)
			emmc_phy_pad_setting(host);
		else
			sdio_phy_pad_setting(host);
		/* Program the clock: set divider to lowest freq for recovery */
		clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
		clk &= ~(SDHCI_DIV_MASK << SDHCI_DIVIDER_SHIFT);
		clk &= ~(SDHCI_DIV_HI_MASK << SDHCI_DIVIDER_HI_SHIFT);
		clk |= SDHCI_CLOCK_INT_EN | SDHCI_CLOCK_CARD_EN;
		sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);
		udelay(150);
		cvi_sdhci_wait_clk_stable(host);
		if (sdhci_readl(host, SDHCI_PRESENT_STATE) &
		    (SDHCI_CMD_INHIBIT | SDHCI_DATA_INHIBIT))
			printf("mmc%d: inhibit still stuck after reset!\n", host->index);
	}

	if (host->index == MMC_TYPE_MMC ||
	    mmc->signal_voltage == MMC_SIGNAL_VOLTAGE_180) {
		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		ctrl |= SDHCI_CTRL_VDD_180;
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
	}

	if (host->index == MMC_TYPE_MMC &&
	    mmc->selected_mode != MMC_HS_200 &&
	    mmc->selected_mode != MMC_HS_400 &&
	    mmc->selected_mode != MMC_HS_400_ES) {
		ctrl = sdhci_readw(host, SDHCI_HOST_CONTROL2);
		ctrl &= ~SDHCI_CTRL_UHS_MASK;
		if (mmc->selected_mode == MMC_HS ||
		    mmc->selected_mode == MMC_HS_52)
			ctrl |= SDHCI_CTRL_UHS_SDR25;
		else
			ctrl |= SDHCI_CTRL_UHS_SDR12;
		sdhci_writew(host, ctrl, SDHCI_HOST_CONTROL2);
	} else {
		sdhci_set_uhs_timing(host);
	}

	if (host->index == MMC_TYPE_MMC) {
		hc = sdhci_readb(host, SDHCI_HOST_CONTROL);
		hc &= ~(SDHCI_CTRL_4BITBUS | SDHCI_CTRL_8BITBUS);
		if (mmc->bus_width == 8)
			hc |= SDHCI_CTRL_8BITBUS;
		else if (mmc->bus_width == 4)
			hc |= SDHCI_CTRL_4BITBUS;
		sdhci_writeb(host, hc, SDHCI_HOST_CONTROL);
	}

	if (cvi_sdhci_enable_pll(host))
		printf("mmc%d: PLL enable failed clk=0x%x\n", host->index,
		       sdhci_readw(host, SDHCI_CLOCK_CONTROL));

	if (host->index == MMC_TYPE_MMC) {
		struct cvi_sdhci_host *cvi_host =
			container_of(host, struct cvi_sdhci_host, host);
		u32 vendor_base = cvi_host->vendor_base;

		mmio_setbits_16(vendor_base + VENDOR_EMMC_CTRL_R,
				EMMC_CARD_IS_EMMC | EMMC_RST_N);
		sdhci_writeb(host, 0xe, SDHCI_TIMEOUT_CONTROL);

		/* ===== HS400 DLL Configuration (PHY+0x300 registers) ===== */
		if (mmc->selected_mode == MMC_HS_400 ||
		    mmc->selected_mode == MMC_HS_400_ES) {
			u8 dll_status;
			int dll_retry;
			u16 clk;

			/* Enable negedge data out for HS400 */
			mmio_setbits_16(vendor_base + DWC_MSHC_CTRL_R,
					DWC_MSHC_NEGEDGE_DATAOUT_EN);

			/*
			 * DLL Config (ref: kernel sdhci-of-dwcmshc):
			 * 1. Stop card clock
			 * 2. Configure DLL registers
			 * 3. Restart card clock
			 * 4. Enable DLL, wait for lock
			 */
			clk = sdhci_readw(host, SDHCI_CLOCK_CONTROL);
			clk &= ~SDHCI_CLOCK_CARD_EN;
			sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

			sdhci_writeb(host, 0x20, SDHCI_PHY_DLL_CNFG1);
			sdhci_writeb(host, 0x00, SDHCI_PHY_DLL_CNFG2);
			sdhci_writeb(host, 0x60, SDHCI_PHY_DLLDL_CNFG);

			clk |= SDHCI_CLOCK_CARD_EN;
			sdhci_writew(host, clk, SDHCI_CLOCK_CONTROL);

			/* Enable DLL, wait for lock */
			sdhci_writeb(host, DLL_CTRL_DLL_EN,
				     SDHCI_PHY_DLL_CTRL);

			dll_retry = 100;
			do {
				dll_status = sdhci_readb(host,
						 SDHCI_PHY_DLL_STATUS);
				if (dll_status & DLL_STATUS_LOCK_STS)
					break;
				udelay(100);
			} while (--dll_retry > 0);

			printf("mmc%d: DLL status=0x%x %s\n",
			       host->index, dll_status,
			       (dll_status & DLL_STATUS_LOCK_STS) ?
			       "locked" : "timeout");
		} else {
			/* Disable HS400 features when downgrading */
			mmio_clrbits_16(vendor_base + DWC_MSHC_CTRL_R,
					DWC_MSHC_NEGEDGE_DATAOUT_EN);
			sdhci_writeb(host, 0, SDHCI_PHY_DLL_CTRL);
		}
	}

	return 0;
}

int cvi_get_cd(struct sdhci_host *host)
{
	u32 reg;

	reg = sdhci_readl(host, SDHCI_PRESENT_STATE);
	pr_debug("%s reg = 0x08%x\n", __func__, reg);
	if (reg & SDHCI_CARD_PRESENT)
		return 1;
	else
		return 0;
}

static void cvi_general_reset(struct sdhci_host *host, u8 mask)
{
	if (mask & SDHCI_RESET_ALL) {
		if (host->index == MMC_TYPE_MMC) {
			/* DWC_mshc: clear CLK_CTRL bit5 before enabling clock */
			sdhci_writew(host,
				     sdhci_readw(host, SDHCI_CLOCK_CONTROL) &
					     ~BIT(5),
				     SDHCI_CLOCK_CONTROL);
		}
		cvi_sdhci_enable_int_clk(host);
		cvi_sdhci_enable_irq(host);
		if (host->index == MMC_TYPE_MMC) {
			struct cvi_sdhci_host *cvi_host =
				container_of(host, struct cvi_sdhci_host, host);

			mmio_setbits_16(cvi_host->vendor_base +
						VENDOR_EMMC_CTRL_R,
					EMMC_CARD_IS_EMMC | EMMC_RST_N);
		}
	}
}

/* ===== SD Voltage Switch ===== */
#ifdef CONFIG_MMC_UHS_SUPPORT
static void cvi_sd_voltage_switch(struct mmc *mmc)
{
	struct sdhci_host *host = mmc->priv;

	if (host->index != MMC_TYPE_SD && host->index != MMC_TYPE_SD1)
		return;

#if CONFIG_IS_ENABLED(DM_MMC) && CONFIG_IS_ENABLED(DM_GPIO)
	if (dm_gpio_is_valid(&host->pwr_gpio))
		dm_gpio_set_value(&host->pwr_gpio, 1);
#endif
	udelay(1000);
}
#endif

/* ===== Probe ===== */
static int cvi_sdhci_probe(struct udevice *dev)
{
	struct cvi_sdhci_driver_data *drv_data =
		(struct cvi_sdhci_driver_data *)dev_get_driver_data(dev);
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct cvi_sdhci_plat *plat = dev_get_plat(dev);
	struct cvi_sdhci_host *cvi_host = dev_get_priv(dev);
	struct sdhci_host *host = &cvi_host->host;
	int ret;

	host->ops = drv_data->ops;
	host->index = drv_data->index;
	printf("%s: mmc%d probe\n", __func__, host->index);

	ret = reset_get_by_name(dev, "sdhci", &cvi_host->reset_ctl);
	if (ret) {
		pr_debug("warning: reset_get_by_name failed\n");
	} else {
		ret = reset_assert(&cvi_host->reset_ctl);
		if (ret) {
			printf("%s failed assert reset\n", __func__);
			return ret;
		}
		ret = reset_deassert(&cvi_host->reset_ctl);
		if (ret) {
			printf("%s failed deassert reset\n", __func__);
			return ret;
		}
	}

	upriv->mmc = &plat->mmc;
	host->mmc = &plat->mmc;
	host->mmc->priv = host;
	host->mmc->dev = dev;

#if CONFIG_IS_ENABLED(DM_GPIO) && defined(CONFIG_MMC_UHS_SUPPORT)
	/* SD card power GPIO from device tree pwr-gpios */
	gpio_request_by_name(dev, "pwr-gpios", 0,
			     &host->pwr_gpio, GPIOD_IS_OUT);
	if (dm_gpio_is_valid(&host->pwr_gpio))
		dm_gpio_set_value(&host->pwr_gpio, 0);
#endif

	cvi_host->vendor_base = cvi_get_vendor_base(host);

	ret = sdhci_setup_cfg(&plat->cfg, host, cvi_host->mmc_fmax_freq,
			      cvi_host->mmc_fmin_freq);
	if (ret)
		return ret;

	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret)
		return ret;

	ret = sdhci_probe(dev);
	if (ret) {
		printf("mmc%d sdhci_probe failed: %d\n", host->index, ret);
		return ret;
	}

	if (cvi_host->is_64_addressing) {
		sdhci_writew(host,
			     sdhci_readw(host, SDHCI_HOST_CONTROL2) |
			     SDHCI_HOST_VER4_ENABLE |
			     SDHCI_HOST_ADDRESSING,
			     SDHCI_HOST_CONTROL2);
	}

	if (host->index == MMC_TYPE_MMC) {
		ret = emmc_phy_pad_setting(host);
		if (ret) {
			printf("mmc%d emmc_phy_pad_setting failed: %d\n",
			       host->index, ret);
			return ret;
		}
		cvi_host->vendor_base = cvi_get_vendor_base(host);
		mmio_setbits_16(cvi_host->vendor_base + VENDOR_EMMC_CTRL_R,
				EMMC_CARD_IS_EMMC);

		/* DWC_mshc: enable auto CMD23 (HOST_CONTROL2 bit11) */
		sdhci_writew(host,
			     sdhci_readw(host, SDHCI_HOST_CONTROL2) | BIT(11),
			     SDHCI_HOST_CONTROL2);
	}

	if (host->index == MMC_TYPE_SD || host->index == MMC_TYPE_SD1)
		sdio_phy_pad_setting(host);

	return 0;
}

/* ===== HS400 Callbacks (called via sdhci_ops platform callbacks) ===== */
#if CONFIG_IS_ENABLED(MMC_HS400_ES_SUPPORT)
static int cvi_set_enhanced_strobe(struct sdhci_host *host)
{
	struct cvi_sdhci_host *cvi_host =
		container_of(host, struct cvi_sdhci_host, host);

	/* Enable Enhanced Strobe per DWC_mshc User Guide Figure 8-42 */
	mmio_setbits_16(cvi_host->vendor_base + VENDOR_EMMC_CTRL_R,
			ENH_STROBE_ENABLE);
	return 0;
}
#endif

#if CONFIG_IS_ENABLED(MMC_HS400_SUPPORT)
static int cvi_hs400_prepare_ddr(struct sdhci_host *host)
{
	struct cvi_sdhci_host *cvi_host =
		container_of(host, struct cvi_sdhci_host, host);

	/* Clear enhanced strobe before DDR switch, per User Guide Figure 8-42 */
	mmio_clrbits_16(cvi_host->vendor_base + VENDOR_EMMC_CTRL_R,
			ENH_STROBE_ENABLE);
	return 0;
}
#endif

/* ===== Ops Structures ===== */
const struct sdhci_ops cvi_sdhci_emmc_ops = {
	.reset = cvi_general_reset,
	.set_ios_post = sdhci_cvi_set_ios_post,
#ifdef CONFIG_MMC_SUPPORTS_TUNING
	.platform_execute_tuning = cvi_general_execute_tuning,
#endif
#if CONFIG_IS_ENABLED(MMC_HS400_ES_SUPPORT)
	.set_enhanced_strobe = cvi_set_enhanced_strobe,
#endif
#if CONFIG_IS_ENABLED(MMC_HS400_SUPPORT)
	.hs400_prepare_ddr = cvi_hs400_prepare_ddr,
#endif
};

const struct sdhci_ops cvi_sdhci_sd_ops = {
	.get_cd = cvi_get_cd,
#ifdef CONFIG_MMC_SUPPORTS_TUNING
	.platform_execute_tuning = cvi_general_execute_tuning,
#endif
#ifdef CONFIG_MMC_UHS_SUPPORT
	.voltage_switch = cvi_sd_voltage_switch,
#endif
	.reset = cvi_general_reset,
	.set_ios_post = sdhci_cvi_set_ios_post,
};

/* ===== Driver Data ===== */
static const struct cvi_sdhci_driver_data sdhci_cvi_emmc_drvdata = {
	.ops = &cvi_sdhci_emmc_ops,
	.index = MMC_TYPE_MMC,
};

static const struct cvi_sdhci_driver_data sdhci_cvi_sd_drvdata = {
	.ops = &cvi_sdhci_sd_ops,
	.index = MMC_TYPE_SD,
};

static const struct cvi_sdhci_driver_data sdhci_cvi_sd1_drvdata = {
	.ops = &cvi_sdhci_sd_ops,
	.index = MMC_TYPE_SD1,
};

static const struct udevice_id cvi_sdhci_ids[] = {
	{.compatible = "cvitek,cv84x6-emmc",
	 .data = (ulong)&sdhci_cvi_emmc_drvdata},
	{.compatible = "cvitek,cv84x6-sd",
	 .data = (ulong)&sdhci_cvi_sd_drvdata},
	{.compatible = "cvitek,cv84x6-sd1",
	 .data = (ulong)&sdhci_cvi_sd1_drvdata},
	{/* sentinel */}};

U_BOOT_DRIVER(cvi_sdhci_drv) = {
	.name = "cvi_sdhci_cv84x6",
	.id = UCLASS_MMC,
	.of_match = cvi_sdhci_ids,
	.of_to_plat = cvi_ofdata_to_platdata,
	.bind = cvi_sdhci_bind,
	.probe = cvi_sdhci_probe,
	.priv_auto = sizeof(struct cvi_sdhci_host),
	.plat_auto = sizeof(struct cvi_sdhci_plat),
	.ops = &sdhci_ops,
};

/* ===== PHY Tune Command ===== */
static int do_phy_tune(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	struct mmc *mmc;
	struct sdhci_host *host;
	u32 vendor_base, phy_cnfg;
	u16 cmdpad, datpad, clkpad, stbpad, rstnpad;
	u8 smpldl, atdl, commdl, sdclkdl;

	/* Find eMMC device */
	mmc = find_mmc_device(0);
	if (!mmc) {
		printf("No MMC device 0\n");
		return CMD_RET_FAILURE;
	}
	host = mmc->priv;
	vendor_base = ((struct cvi_sdhci_host *)host)->vendor_base;
	phy_cnfg = sdhci_readl(host, SDHCI_P_PHY_CNFG);
	cmdpad = sdhci_readw(host, SDHCI_P_CMDPAD_CNFG);
	datpad = sdhci_readw(host, SDHCI_P_DATPAD_CNFG);
	clkpad = sdhci_readw(host, SDHCI_P_CLKPAD_CNFG);
	stbpad = sdhci_readw(host, SDHCI_P_STBPAD_CNFG);
	rstnpad = sdhci_readw(host, SDHCI_P_RSTNPAD_CNFG);
	commdl = sdhci_readb(host, SDHCI_P_COMMDL_CNFG);
	sdclkdl = sdhci_readb(host, SDHCI_P_SDCLKDL_CNFG);
	smpldl = sdhci_readb(host, SDHCI_P_SMPLDL_CNFG);
	atdl = sdhci_readb(host, SDHCI_P_ATDL_CNFG);

	if (argc == 1) {
		/* Dump current settings */
		printf("PHY_CNFG    = 0x%08x (PAD_SP=%d PAD_SN=%d RSTN=%d PWRGOOD=%d)\n",
		       phy_cnfg, (phy_cnfg >> 16) & 0xF, (phy_cnfg >> 20) & 0xF,
		       phy_cnfg & 1, (phy_cnfg >> 1) & 1);
		printf("CMDPAD_CNFG = 0x%04x (RXSEL=%d WEAKPULL=%d TXSLEW_P=%d TXSLEW_N=%d)\n",
		       cmdpad, cmdpad & 7, (cmdpad >> 3) & 3,
		       (cmdpad >> 5) & 0xF, (cmdpad >> 9) & 0xF);
		printf("DATPAD_CNFG = 0x%04x (RXSEL=%d WEAKPULL=%d TXSLEW_P=%d TXSLEW_N=%d)\n",
		       datpad, datpad & 7, (datpad >> 3) & 3,
		       (datpad >> 5) & 0xF, (datpad >> 9) & 0xF);
		printf("CLKPAD_CNFG = 0x%04x (RXSEL=%d WEAKPULL=%d TXSLEW_P=%d TXSLEW_N=%d)\n",
		       clkpad, clkpad & 7, (clkpad >> 3) & 3,
		       (clkpad >> 5) & 0xF, (clkpad >> 9) & 0xF);
		printf("STBPAD_CNFG = 0x%04x (RXSEL=%d WEAKPULL=%d TXSLEW_P=%d TXSLEW_N=%d)\n",
		       stbpad, stbpad & 7, (stbpad >> 3) & 3,
		       (stbpad >> 5) & 0xF, (stbpad >> 9) & 0xF);
		printf("RSTNPAD_CNFG = 0x%04x\n", rstnpad);
		printf("COMMDL=%d SDCLKDL=%d SMPLDL=%d ATDL=%d\n",
		       commdl, sdclkdl, smpldl, atdl);
	} else {
		/* Parse arguments: sn=8 sp=9 rxsl=1 smpl=0xc atdl=0xc */
		int i;
		for (i = 1; i < argc; i++) {
			if (!strncmp(argv[i], "sn=", 3))
				phy_cnfg = (phy_cnfg & ~(0xF << 20)) |
					   ((simple_strtoul(argv[i]+3, NULL, 0) & 0xF) << 20);
			else if (!strncmp(argv[i], "sp=", 3))
				phy_cnfg = (phy_cnfg & ~(0xF << 16)) |
					   ((simple_strtoul(argv[i]+3, NULL, 0) & 0xF) << 16);
			else if (!strncmp(argv[i], "rxsl=", 5))
				cmdpad = (cmdpad & ~7) | (simple_strtoul(argv[i]+5, NULL, 0) & 7);
			else if (!strncmp(argv[i], "smpl=", 5))
				smpldl = simple_strtoul(argv[i]+5, NULL, 0) & 0xFF;
			else if (!strncmp(argv[i], "atdl=", 5))
				atdl = simple_strtoul(argv[i]+5, NULL, 0) & 0xFF;
			else
				printf("Unknown arg: %s\n", argv[i]);
		}
		/* Write new values */
		sdhci_writel(host, phy_cnfg, SDHCI_P_PHY_CNFG);
		sdhci_writew(host, cmdpad, SDHCI_P_CMDPAD_CNFG);
		sdhci_writew(host, datpad, SDHCI_P_DATPAD_CNFG);
		sdhci_writew(host, clkpad, SDHCI_P_CLKPAD_CNFG);
		sdhci_writew(host, stbpad, SDHCI_P_STBPAD_CNFG);
		sdhci_writew(host, rstnpad, SDHCI_P_RSTNPAD_CNFG);
		sdhci_writeb(host, smpldl, SDHCI_P_SMPLDL_CNFG);
		sdhci_writeb(host, atdl, SDHCI_P_ATDL_CNFG);
		printf("PHY settings updated. Use 'mmc rescan 0' to re-init\n");
	}
	return CMD_RET_SUCCESS;
}

U_BOOT_CMD(phy_tune, 8, 0, do_phy_tune,
	   "Read/write eMMC PHY settings",
	   "\n"
	   "  phy_tune                      - dump current PHY settings\n"
	   "  phy_tune sn=8 sp=9            - set PAD_SN/PAD_SP\n"
	   "  phy_tune rxsl=1               - set RXSEL for CMD/DAT pads\n"
	   "  phy_tune smpl=0xc atdl=0xc    - set delay line values\n"
	   "  Then use 'mmc rescan 0' to re-init eMMC with new settings\n");
