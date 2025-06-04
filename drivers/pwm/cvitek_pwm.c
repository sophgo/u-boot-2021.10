// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2022 cvitek All rights reserved.
 * Author: jinyu zhao <jinyu.zhaok@cvitek.com>
 *
 * cvitek PWM driver for U-Boot
 */
#include <common.h>
#include <dm.h>
#include <errno.h>
#include <pwm.h>
#include <asm/io.h>
#include <log.h>

#define CVITEK_PWM_CLK_MHZ 100 //250

enum {
	CHANNEL0 = 0,
	CHANNEL1,
	CHANNEL2,
	CHANNEL3,
	CHANNEL4,
	CHANNEL5,
	MAX_CHANNEL,
};

struct cvitek_pwm_regs {
	unsigned int hlperiod0;/* 0x00 */
	unsigned int period0;/* 0x04 */
	unsigned int hlperiod1;/* 0x08 */
	unsigned int period1;/* 0x0c */
	unsigned int hlperiod2;/* 0x10 */
	unsigned int period2;/* 0x14 */
	unsigned int hlperiod3;/* 0x18 */
	unsigned int period3;/* 0x1c */
	unsigned int hlperiod4;/* 0x20 */
	unsigned int period4;/* 0x24 */
	unsigned int hlperiod5;/* 0x28 */
	unsigned int period5;/* 0x2c */
	unsigned int freq0num;/* 0x30 */
	unsigned int freq0data;/* 0x34 */
	unsigned int freq1num;/* 0x38 */
	unsigned int freq1data;/* 0x3c */
	unsigned int freq2num;/* 0x40 */
	unsigned int freq2data;/* 0x44 */
	unsigned int freq3num;/* 0x48 */
	unsigned int freq3data;/* 0x4c */
	unsigned int freq4num;/* 0x50 */
	unsigned int freq4data;/* 0x54 */
	unsigned int freq5num;/* 0x58 */
	unsigned int freq5data;/* 0x5c */
	unsigned int polarity;/* 0x60 */
	unsigned int pwmstart;/* 0x64 */
	unsigned int pwmdone;/* 0x68 */
	unsigned int pwmupdate;/* 0x6c */
	unsigned int pcount[6];/* 0x70 - 0x84 */
	unsigned int pulsecount[6];/* 0x88 - 0x9c */
	unsigned int pulsecnt[6];/* 0xa0 - 0xb4 */
	unsigned int shiftcount[6];/* 0xb8 - 0xcc */
	unsigned int shiftstart;/* 0xd0 */
	unsigned int freqen;/* 0xd4 */
	unsigned int freq0_hcount;/* 0xd8 */
	unsigned int freq0_lcount;/* 0xdc */
	unsigned int freq1_hcount;/* 0xe0 */
	unsigned int freq1_lcount;/* 0xe4 */
	unsigned int freq2_hcount;/* 0xe8 */
	unsigned int freq2_lcount;/* 0xec */
	unsigned int freq3_hcount;/* 0xf0 */
	unsigned int freq3_lcount;/* 0xf4 */
	unsigned int freq4_hcount;/* 0xf8 */
	unsigned int freq4_lcount;/* 0xfc */
	unsigned int freq5_hcount;/* 0x100 */
	unsigned int freq5_lcount;/* 0x104 */
	unsigned int freq0_done_num[6];/* 0x108 - 0x11c */
	unsigned int pwm_oe;/* 0x120 */
	unsigned int pwm_rev0;/* 0x124 */
	unsigned int pwm_ver;/* 0x128 */
	unsigned int mask_period[6];/* 0x12c - 0x140 */
	unsigned int mask_cnt[6];/* 0x144 - 0x158 */
	unsigned int freq_db_cnt[6];/* 0x15c - 0x170 */
	unsigned int pwm0_start_point;/* 0x174 */
	unsigned int pwm0_end_point;/* 0x178 */
	unsigned int pwm1_start_point;/* 0x17c */
	unsigned int pwm1_end_point;/* 0x180 */
	unsigned int pwm2_start_point;/* 0x184 */
	unsigned int pwm2_end_point;/* 0x188 */
	unsigned int pwm3_start_point;/* 0x18c */
	unsigned int pwm3_end_point;/* 0x190 */
	unsigned int pwm4_start_point;/* 0x194 */
	unsigned int pwm4_end_point;/* 0x198 */
	unsigned int pwm5_start_point;/* 0x19c */
	unsigned int pwm5_end_point;/* 0x1a0 */
	unsigned int pwm2adc_cnt_h0;/* 0x1a4 */
	unsigned int pwm2adc_cnt_l0;/* 0x1a8 */
	unsigned int pwm2adc_cnt_h1;/* 0x1ac */
	unsigned int pwm2adc_cnt_l1;/* 0x1b0 */
	unsigned int pwm2adc_cnt_h2;/* 0x1b4 */
	unsigned int pwm2adc_cnt_l2;/* 0x1b8 */
	unsigned int pwm2adc_cnt_h3;/* 0x1bc */
	unsigned int pwm2adc_cnt_l3;/* 0x1c0 */
	unsigned int pwm2adc_cnt_h4;/* 0x1c4 */
	unsigned int pwm2adc_cnt_l4;/* 0x1c8 */
	unsigned int pwm2adc_cnt_h5;/* 0x1cc */
	unsigned int pwm2adc_cnt_l5;/* 0x1d0 */
	unsigned int pwm_start_toggle_mode;/* 0x1d4 */
	unsigned int pwm_end_toggle_mode;/* 0x1d8 */
	unsigned int pwm_output_disable;/* 0x1dc */
	unsigned int pwm_update_cnt[6];/* 0x1e0 - 0x1f4 */
};

struct pwm_chip_priv {
	const char *chip_name;
	struct udevice *dev;		/* Device, NULL for invalid pwm */
	void __iomem	*base;
	int pwm_base;			/* this device pwm base number */
	int pwm_count;
	unsigned long flags;
};

static int cvitek_pwm_set_config(struct udevice *dev, uint channel, uint period_ns, uint duty_ns)
{
	struct pwm_chip_priv *chip = dev_get_priv(dev);
	struct cvitek_pwm_regs *regs = (struct cvitek_pwm_regs *)chip->base;
	unsigned int period_val, hlperiod_val;

	if (channel < 0 || channel >= chip->pwm_count)
		return -EINVAL;
	if (duty_ns <= 0)
		duty_ns = 1;
	if (duty_ns >= period_ns)
		duty_ns = period_ns - 1;

	period_val = CVITEK_PWM_CLK_MHZ * period_ns / 1000;
	hlperiod_val = CVITEK_PWM_CLK_MHZ * (period_ns - duty_ns) / 1000;

	switch (channel) {
	case CHANNEL0:
		/* set duty cycles */
		writel(hlperiod_val, &regs->hlperiod0);
		/* set start_point cycles */
		writel(0, &regs->pwm0_start_point);
		/* set end_point cycles */
		writel(hlperiod_val, &regs->pwm0_end_point);
		/* set period cycles */
		writel(period_val, &regs->period0);
		break;
	case CHANNEL1:
		/* set duty cycles */
		writel(hlperiod_val, &regs->hlperiod1);
		/* set start_point cycles */
		writel(0, &regs->pwm1_start_point);
		/* set end_point cycles */
		writel(hlperiod_val, &regs->pwm1_end_point);
		/* set period cycles */
		writel(period_val, &regs->period1);
		break;
	case CHANNEL2:
		/* set duty cycles */
		writel(hlperiod_val, &regs->hlperiod2);
		/* set start_point cycles */
		writel(0, &regs->pwm2_start_point);
		/* set end_point cycles */
		writel(hlperiod_val, &regs->pwm2_end_point);
		/* set period cycles */
		writel(period_val, &regs->period2);
		break;
	case CHANNEL3:
		/* set duty cycles */
		writel(hlperiod_val, &regs->hlperiod3);
		/* set start_point cycles */
		writel(0, &regs->pwm3_start_point);
		/* set end_point cycles */
		writel(hlperiod_val, &regs->pwm3_end_point);
		/* set period cycles */
		writel(period_val, &regs->period3);
		break;
	case CHANNEL4:
		/* set duty cycles */
		writel(hlperiod_val, &regs->hlperiod4);
		/* set start_point cycles */
		writel(0, &regs->pwm4_start_point);
		/* set end_point cycles */
		writel(hlperiod_val, &regs->pwm4_end_point);
		/* set period cycles */
		writel(period_val, &regs->period4);
		break;
	case CHANNEL5:
		/* set duty cycles */
		writel(hlperiod_val, &regs->hlperiod5);
		/* set start_point cycles */
		writel(0, &regs->pwm5_start_point);
		/* set end_point cycles */
		writel(hlperiod_val, &regs->pwm5_end_point);
		/* set period cycles */
		writel(period_val, &regs->period5);
		break;
	default:
		break;
	}

	return 0;
}

static int cvitek_pwm_set_enable(struct udevice *dev, uint channel, bool enable)
{
	struct pwm_chip_priv *chip = dev_get_priv(dev);
	struct cvitek_pwm_regs *regs = (struct cvitek_pwm_regs *)chip->base;
	unsigned int value;

	if (channel < 0 || channel >= chip->pwm_count)
		return -EINVAL;

	value = readl(&regs->pwm_oe);
	writel(value | (0x1 << channel), &regs->pwm_oe);

	value = readl(&regs->pwmstart);
	writel(value & ~(0x1 << channel), &regs->pwmstart);
	/*update pwm*/
	writel(0x1, &regs->pwmupdate);
	/* enable pwmstart */
	if (enable)
		writel(value | (0x1 << channel), &regs->pwmstart);

	return 0;
}

static int cvitek_pwm_set_invert(struct udevice *dev, uint channel, bool polarity)
{
	struct pwm_chip_priv *chip = dev_get_priv(dev);
	struct cvitek_pwm_regs *regs = (struct cvitek_pwm_regs *)chip->base;
	unsigned int value;

	if (channel < 0 || channel >= chip->pwm_count)
		return -EINVAL;

	value = readl(&regs->polarity);
	/* polarity: default high level output */
	value = polarity ? value | (0x1 << channel) :
				value & ~(0x1 << channel);
	writel(value, &regs->polarity);

	return 0;
}

static const struct pwm_ops cvitek_pwm_ops = {
	.set_config	= cvitek_pwm_set_config,
	.set_enable	= cvitek_pwm_set_enable,
	.set_invert	= cvitek_pwm_set_invert,
};

static int cvitek_pwm_probe(struct udevice *dev)
{
	struct pwm_chip_priv *chip = dev_get_priv(dev);
	const char *name;
	fdt_addr_t base;

	base = dev_read_addr(dev);
	if (base == FDT_ADDR_T_NONE) {
		pr_err("Can't get the PWM register base address\n");
		return -ENXIO;
	}

	chip->base = (void *)base;
	chip->pwm_count = MAX_CHANNEL;

	/* Ensure that we have a base for each bank */
	name = dev_read_name(dev);
	if (!name)
		return -ENOENT;

	chip->pwm_base = dev->seq_ * chip->pwm_count;
	if (chip->pwm_base < 0)
		return -ENOENT;

	return 0;
}

static const struct udevice_id cvitek_pwm_ids[] = {
	{ .compatible = "cvitek,cvi-pwm" },
	{ }
};

U_BOOT_DRIVER(pwm_cvitek) = {
	.name		= "pwm_cvitek",
	.id		= UCLASS_PWM,
	.of_match	= cvitek_pwm_ids,
	.ops		= &cvitek_pwm_ops,
	.probe		= cvitek_pwm_probe,
	.priv_auto	= sizeof(struct pwm_chip_priv),
};
