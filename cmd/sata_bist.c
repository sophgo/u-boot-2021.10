// SPDX-License-Identifier: GPL-2.0+
/*
 * SATA BIST (Built-In Self Test) command for cv84x6 DWC AHCI.
 *
 * Tests the 4 SATA ports of the two DWC_ahsata controllers (2 ports each):
 *   - TXO   (Transmit Only):      transmit a pattern on the wire, no device
 *                                 needed, for scope measurement.
 *   - NEALB (Near-End Analog LB): PHY internal TX->RX loopback, counts errors.
 *   - FERLB (Far-End Retimed LB): Link layer loopback, counts errors.
 *
 * Register map (offsets relative to each HBA base 0x21310000 / 0x21350000):
 *   BISTAFR 0xa0  BISTCR 0xa4  BISTFCTR 0xa8  BISTSR 0xac  BISTDECR 0xb0
 *   TESTR   0xf4  (PSEL[18:16] port select, BSEL[25:24] = 0b00)
 *
 * The BIST register block is shared between ports: TESTR.PSEL must be
 * written before any BIST register access.
 *
 * BISTCR.PATTERN[3:0]:
 *   0x0 SSOP  0x1 HTDP  0x2 LTDP  0x3 LFSCP  0x4 COMP
 *   0x5 LBP   0x6 MFTP  0x7 HFTP  0x8 LFTP
 */

#include <common.h>
#include <command.h>
#include <dm.h>
#include <ahci.h>
#include <asm/io.h>
#include <linux/delay.h>

#define DWC_SATA_BISTAFR	0xa0
#define DWC_SATA_BISTCR		0xa4
#define DWC_SATA_BISTFCTR	0xa8
#define DWC_SATA_BISTSR		0xac
#define DWC_SATA_BISTDECR	0xb0
#define DWC_SATA_TESTR		0xf4

/* cv84x6 instantiates two DWC_ahsata cores, two ports per core */
#define DWC_SATA_PORTS_PER_CTRL	2

#define BISTCR_FERLB		(1 << 20)
#define BISTCR_TXO		(1 << 18)
#define BISTCR_CNTCLR		(1 << 17)
#define BISTCR_NEALB		(1 << 16)
#define BISTCR_ERREN		(1 << 6)
#define BISTCR_FLIP		(1 << 5)
#define BISTCR_PV		(1 << 4)

/* BISTCR fields this command drives; all other bits are preserved on write */
#define BISTCR_CTRL_MASK \
	(BISTCR_FERLB | BISTCR_TXO | BISTCR_CNTCLR | BISTCR_NEALB | \
	 BISTCR_ERREN | BISTCR_FLIP | BISTCR_PV | 0xf)

#define BISTSR_FRAMERR_MASK	0x0000ffff
#define BISTSR_BRSTERR_MASK	0x00ff0000
#define BISTSR_BRSTERR_SHIFT	16

#define BIST_DEFAULT_TIME_MS	1000

static const char * const pattern_names[] = {
	"SSOP", "HTDP", "LTDP", "LFSCP", "COMP", "LBP", "MFTP", "HFTP", "LFTP"
};

static void *sata_bist_get_base(int port)
{
	struct udevice *dev;
	struct ahci_uc_priv *uc_priv;
	int ctrl = port / DWC_SATA_PORTS_PER_CTRL;
	int rc;

	rc = uclass_get_device(UCLASS_AHCI, ctrl, &dev);
	if (rc) {
		printf("sata_bist: AHCI controller %d not probed, run 'sata init' first (err=%d)\n",
		       ctrl, rc);
		return NULL;
	}

	/* ahci_probe_scsi() stores the HBA base in the AHCI device's uc_priv */
	uc_priv = dev_get_uclass_priv(dev);
	if (!uc_priv || !uc_priv->mmio_base) {
		printf("sata_bist: no mmio base\n");
		return NULL;
	}

	return (void *)uc_priv->mmio_base;
}

static void sata_bist_select_port(void *base, int port)
{
	/* flat port -> per-controller port; PSEL[18:16], BSEL[25:24] = 0b00 */
	writel((port % DWC_SATA_PORTS_PER_CTRL) << 16, base + DWC_SATA_TESTR);
}

static void sata_bist_write_ctrl(void *base, u32 val)
{
	u32 cr = readl(base + DWC_SATA_BISTCR);

	/*
	 * Preserve llc[10:8]/sdfe[12]/errlossen[13]/qphyinit[14], set at reset.
	 * A bare write clears llc, and SCRAM=0 enables scrambling in BIST mode,
	 * which scrambles the test pattern (scope shows noise, not the pattern).
	 */
	cr &= ~BISTCR_CTRL_MASK;
	cr |= val & BISTCR_CTRL_MASK;
	writel(cr, base + DWC_SATA_BISTCR);
}

static void sata_bist_dump_regs(void *base, int port)
{
	int ctrl_port = port % DWC_SATA_PORTS_PER_CTRL;
	u32 testr, bistafr, bistcr, bistfctr, bistsr, bistdecr, ssts;

	sata_bist_select_port(base, port);

	testr    = readl(base + DWC_SATA_TESTR);
	bistafr  = readl(base + DWC_SATA_BISTAFR);
	bistcr   = readl(base + DWC_SATA_BISTCR);
	bistfctr = readl(base + DWC_SATA_BISTFCTR);
	bistsr   = readl(base + DWC_SATA_BISTSR);
	bistdecr = readl(base + DWC_SATA_BISTDECR);
	/* PxSSTS = HBA port reg block (0x100 + port*0x80) + 0x28 */
	ssts     = readl(base + 0x100 + ctrl_port * 0x80 + 0x28);

	printf("--- sata_bist regs: port %d (ctrl %d) ---\n",
	       port, port / DWC_SATA_PORTS_PER_CTRL);
	printf("  TESTR    0xf4 = 0x%08x  [test_if=%u psel=%u bsel=%u]\n",
	       testr, testr & 1, (testr >> 16) & 0x7, (testr >> 24) & 0x3);
	printf("  BISTAFR  0xa0 = 0x%08x\n", bistafr);
	printf("  BISTCR   0xa4 = 0x%08x  [pattern=%u(%s) pv=%u flip=%u erren=%u llc=0x%x sdfe=%u errlossen=%u qphyinit=%u llb=%u]\n",
	       bistcr, bistcr & 0xf,
	       ((bistcr & 0xf) <= 8) ? pattern_names[bistcr & 0xf] : "reserved",
	       (bistcr >> 4) & 1, (bistcr >> 5) & 1, (bistcr >> 6) & 1,
	       (bistcr >> 8) & 0x7, (bistcr >> 12) & 1, (bistcr >> 13) & 1,
	       (bistcr >> 14) & 1, (bistcr >> 15) & 1);
	printf("  BISTFCTR 0xa8 = 0x%08x  (FIS count %u)\n", bistfctr, bistfctr);
	printf("  BISTSR   0xac = 0x%08x  [framerr=%u brsterr=%u]\n",
	       bistsr, bistsr & 0xffff, (bistsr >> 16) & 0xff);
	printf("  BISTDECR 0xb0 = 0x%08x  (dword err %u)\n", bistdecr, bistdecr);
	printf("  P%dSSTS   0x%03x = 0x%08x  [DET=0x%x SPD=%u IPM=0x%x]\n",
	       ctrl_port, 0x100 + ctrl_port * 0x80 + 0x28, ssts,
	       ssts & 0xf, (ssts >> 4) & 0xf, (ssts >> 8) & 0xf);
}

static void sata_bist_comreset(void *base, int port)
{
	int ctrl_port = port % DWC_SATA_PORTS_PER_CTRL;
	void *sctl = base + 0x100 + ctrl_port * 0x80 + 0x2c;
	u32 v = readl(sctl);

	/*
	 * A BIST mode (TXO/NEALB/FERLB) latches until a Port reset. DET=1
	 * asserts phy_reset (COMRESET); after >=1ms, DET=0 sends the COMRESET
	 * OOB and re-initializes the PHY, exiting the previous BIST mode so a
	 * new pattern can be loaded.
	 */
	v &= ~0xf;
	writel(v | 0x1, sctl);
	mdelay(1);
	writel(v, sctl);
	/* allow the OOB state machine to settle back to no-device (NOCOMM) */
	mdelay(10);
}

static int sata_bist_txo(void *base, int port, int pattern, int pv, int flip)
{
	u32 val = BISTCR_TXO | (pattern & 0xf);

	if (pv)
		val |= BISTCR_PV;
	if (flip)
		val |= BISTCR_FLIP;

	sata_bist_comreset(base, port);
	sata_bist_select_port(base, port);
	sata_bist_write_ctrl(base, val);

	printf("port %d: TXO started, pattern=%s%s%s (BISTCR=0x%08x)\n",
	       port, pattern_names[pattern], pv ? " long" : "",
	       flip ? " flip" : "", readl(base + DWC_SATA_BISTCR));
	sata_bist_dump_regs(base, port);
	return 0;
}

static int sata_bist_loopback(void *base, int port, int pattern, int pv,
			      int flip, int nealb, u32 time_ms)
{
	u32 trig = nealb ? BISTCR_NEALB : BISTCR_FERLB;
	u32 val = trig | (pattern & 0xf);
	u32 fctr, sr, decr;
	int fail = 0;

	if (pv)
		val |= BISTCR_PV;
	if (flip)
		val |= BISTCR_FLIP;

	sata_bist_comreset(base, port);
	sata_bist_select_port(base, port);

	/* clear counters first */
	sata_bist_write_ctrl(base, BISTCR_CNTCLR);

	/* start loopback with pattern; ERREN reports PHY errors outside FIS */
	sata_bist_write_ctrl(base, val | BISTCR_ERREN);

	mdelay(time_ms);

	/* re-select port, read results */
	sata_bist_select_port(base, port);
	fctr = readl(base + DWC_SATA_BISTFCTR);
	sr = readl(base + DWC_SATA_BISTSR);
	decr = readl(base + DWC_SATA_BISTDECR);

	printf("port %d: %s pattern=%s%s%s time=%ums -> "
	       "FIS=%u frame_err=%u burst_err=%u dword_err=%u  %s\n",
	       port, nealb ? "NEALB" : "FERLB",
	       pattern_names[pattern], pv ? " long" : "", flip ? " flip" : "",
	       time_ms, fctr,
	       sr & BISTSR_FRAMERR_MASK,
	       (sr & BISTSR_BRSTERR_MASK) >> BISTSR_BRSTERR_SHIFT,
	       decr,
	       (sr || decr || !fctr) ? "FAIL" : "PASS");

	sata_bist_dump_regs(base, port);

	if (sr || decr || !fctr)
		fail = 1;

	return fail;
}

static int do_sata_bist(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	void *base;
	int port_lo, port_hi, pattern = 4; /* COMP */
	int pv = 0, flip = 0, nealb = 1;
	u32 time_ms = BIST_DEFAULT_TIME_MS;
	int mode, p, fail = 0;

	if (argc < 2)
		return CMD_RET_USAGE;

	if (!strcmp(argv[1], "nealb")) {
		mode = 0;
		nealb = 1;
	} else if (!strcmp(argv[1], "ferlb")) {
		mode = 0;
		nealb = 0;
	} else if (!strcmp(argv[1], "txo")) {
		mode = 1;
	} else if (!strcmp(argv[1], "dump")) {
		mode = 2;
	} else {
		return CMD_RET_USAGE;
	}

	/* port argument: "all" (default) or 0-3 */
	port_lo = 0;
	port_hi = 3;
	if (argc >= 3 && strcmp(argv[2], "all")) {
		port_lo = (int)dectoul(argv[2], NULL);
		if (port_lo < 0 || port_lo > 3) {
			printf("sata_bist: invalid port %d (0-3 or all)\n", port_lo);
			return CMD_RET_FAILURE;
		}
		port_hi = port_lo;
	}

	if (argc >= 4)
		pattern = (int)dectoul(argv[3], NULL);
	if (pattern < 0 || pattern > 8) {
		printf("sata_bist: invalid pattern %d (0-8)\n", pattern);
		return CMD_RET_FAILURE;
	}

	if (argc >= 5)
		time_ms = dectoul(argv[4], NULL);
	if (argc >= 6)
		pv = (int)dectoul(argv[5], NULL);
	if (argc >= 7)
		flip = (int)dectoul(argv[6], NULL);

	for (p = port_lo; p <= port_hi; p++) {
		base = sata_bist_get_base(p);
		if (!base) {
			fail = 1;
			continue;
		}

		if (mode == 1)
			sata_bist_txo(base, p, pattern, pv, flip);
		else if (mode == 2)
			sata_bist_dump_regs(base, p);
		else
			fail |= sata_bist_loopback(base, p, pattern, pv, flip,
						   nealb, time_ms);
	}

	if (mode == 1)
		printf("sata_bist: TXO active. Stop with port reset/COMRESET "
		       "(rerun 'sata init' or reset board).\n");

	return fail ? CMD_RET_FAILURE : CMD_RET_SUCCESS;
}

U_BOOT_CMD(
	sata_bist, 7, 1, do_sata_bist,
	"SATA BIST loopback / transmit test (DWC AHCI, 4 ports)",
	"nealb [port|all] [pattern] [time_ms] [pv] [flip]\n"
	"sata_bist ferlb [port|all] [pattern] [time_ms] [pv] [flip]\n"
	"sata_bist txo   [port|all] [pattern] [time_ms] [pv] [flip]\n"
	"sata_bist dump  [port|all]\n"
	"  port    0-3 or 'all' (default all)\n"
	"  pattern 0=SSOP 1=HTDP 2=LTDP 3=LFSCP 4=COMP(default)\n"
	"          5=LBP 6=MFTP 7=HFTP 8=LFTP\n"
	"  time_ms test duration for loopback (default 1000)\n"
	"  pv      pattern version 0=short 1=long (SSOP/HTDP/LTDP/LFSCP/COMP)\n"
	"  flip    flip disparity\n"
	"  nealb - near-end analog loopback, no device needed\n"
	"  ferlb - far-end retimed loopback, link loopback\n"
	"  txo   - transmit pattern only, for scope measurement\n"
	"  dump  - dump BIST/TESTR/PxSSTS registers without running a test\n"
	"Run 'sata init' first."
);
