/*
 * drivers/mmc/host/sdhci-msm.c - Qualcomm MSM SDHCI Platform
 * driver source file
 *
 * Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/mmc/mmc.h>
#include <linux/slab.h>

#include "sdhci-pltfm.h"

#define CORE_HC_MODE		0x78
#define HC_MODE_EN		0x1

#define CORE_POWER		0x0
#define CORE_SW_RST		BIT(7)

#define CORE_PWRCTL_STATUS	0xdc
#define CORE_PWRCTL_MASK	0xe0
#define CORE_PWRCTL_CLEAR	0xe4
#define CORE_PWRCTL_CTL		0xe8

#define CORE_PWRCTL_BUS_OFF	BIT(0)
#define CORE_PWRCTL_BUS_ON	BIT(1)
#define CORE_PWRCTL_IO_LOW	BIT(2)
#define CORE_PWRCTL_IO_HIGH	BIT(3)

#define CORE_PWRCTL_BUS_SUCCESS	BIT(0)
#define CORE_PWRCTL_BUS_FAIL	BIT(1)
#define CORE_PWRCTL_IO_SUCCESS	BIT(2)
#define CORE_PWRCTL_IO_FAIL	BIT(3)

#define INT_MASK		0xf
#define MAX_PHASES		16

#define CORE_DLL_LOCK		BIT(7)
#define CORE_DLL_EN		BIT(16)
#define CORE_CDR_EN		BIT(17)
#define CORE_CK_OUT_EN		BIT(18)
#define CORE_CDR_EXT_EN		BIT(19)
#define CORE_DLL_PDN		BIT(29)
#define CORE_DLL_RST		BIT(30)
#define CORE_DLL_CONFIG		0x100
#define CORE_DLL_TEST_CTL	0x104
#define CORE_DLL_STATUS		0x108

#define CORE_VENDOR_SPEC	0x10c
#define CORE_CLK_PWRSAVE	BIT(1)
#define CORE_IO_PAD_PWR_SWITCH	BIT(16)

static const u32 tuning_block_64[] = {
	0x00ff0fff, 0xccc3ccff, 0xffcc3cc3, 0xeffefffe,
	0xddffdfff, 0xfbfffbff, 0xff7fffbf, 0xefbdf777,
	0xf0fff0ff, 0x3cccfc0f, 0xcfcc33cc, 0xeeffefff,
	0xfdfffdff, 0xffbfffdf, 0xfff7ffbb, 0xde7b7ff7
};

static const u32 tuning_block_128[] = {
	0xff00ffff, 0x0000ffff, 0xccccffff, 0xcccc33cc,
	0xcc3333cc, 0xffffcccc, 0xffffeeff, 0xffeeeeff,
	0xffddffff, 0xddddffff, 0xbbffffff, 0xbbffffff,
	0xffffffbb, 0xffffff77, 0x77ff7777, 0xffeeddbb,
	0x00ffffff, 0x00ffffff, 0xccffff00, 0xcc33cccc,
	0x3333cccc, 0xffcccccc, 0xffeeffff, 0xeeeeffff,
	0xddffffff, 0xddffffff, 0xffffffdd, 0xffffffbb,
	0xffffbbbb, 0xffff77ff, 0xff7777ff, 0xeeddbb77
};

/* This structure keeps information per regulator */
struct sdhci_msm_reg_data {
	struct regulator *reg;
	const char *name;
	/* Voltage level values */
	u32 low_vol_level;
	u32 high_vol_level;
};

struct sdhci_msm_pltfm_data {
	u32 caps;				/* Supported UHS-I Modes */
	u32 caps2;				/* More capabilities */
	struct sdhci_msm_reg_data vdd;		/* VDD/VCC regulator info */
	struct sdhci_msm_reg_data vdd_io;	/* VDD IO regulator info */
};

struct sdhci_msm_host {
	struct platform_device *pdev;
	void __iomem *core_mem;	/* MSM SDCC mapped address */
	int pwr_irq;		/* power irq */
	struct clk *clk;	/* main SD/MMC bus clock */
	struct clk *pclk;	/* SDHC peripheral bus clock */
	struct clk *bus_clk;	/* SDHC bus voter clock */
	struct sdhci_msm_pltfm_data pdata;
	struct mmc_host *mmc;
	struct sdhci_pltfm_data sdhci_msm_pdata;
};

/* MSM platform specific tuning */
static inline int msm_dll_poll_ck_out_en(struct sdhci_host *host, u8 poll)
{
	u32 wait_cnt = 50;
	u8 ck_out_en = 0;
	struct mmc_host *mmc = host->mmc;

	/* poll for CK_OUT_EN bit.  max. poll time = 50us */
	ck_out_en = !!(readl_relaxed(host->ioaddr + CORE_DLL_CONFIG) &
			CORE_CK_OUT_EN);

	while (ck_out_en != poll) {
		if (--wait_cnt == 0) {
			dev_err(mmc_dev(mmc), "%s: CK_OUT_EN bit is not %d\n",
			       mmc_hostname(mmc), poll);
			return -ETIMEDOUT;
		}
		udelay(1);

		ck_out_en = !!(readl_relaxed(host->ioaddr + CORE_DLL_CONFIG) &
				CORE_CK_OUT_EN);
	}

	return 0;
}

static int msm_config_cm_dll_phase(struct sdhci_host *host, u8 phase)
{
	int rc = 0;
	u8 grey_coded_phase_table[] = {
		0x0, 0x1, 0x3, 0x2, 0x6, 0x7, 0x5, 0x4,
		0xc, 0xd, 0xf, 0xe, 0xa, 0xb, 0x9, 0x8
	};
	unsigned long flags;
	u32 config;
	struct mmc_host *mmc = host->mmc;

	spin_lock_irqsave(&host->lock, flags);

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config &= ~(CORE_CDR_EN | CORE_CK_OUT_EN);
	config |= (CORE_CDR_EXT_EN | CORE_DLL_EN);
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);

	/* Wait until CK_OUT_EN bit of DLL_CONFIG register becomes '0' */
	rc = msm_dll_poll_ck_out_en(host, 0);
	if (rc)
		goto err_out;

	/*
	 * Write the selected DLL clock output phase (0 ... 15)
	 * to CDR_SELEXT bit field of DLL_CONFIG register.
	 */
	writel_relaxed(((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			 & ~(0xF << 20))
			| (grey_coded_phase_table[phase] << 20)),
		       host->ioaddr + CORE_DLL_CONFIG);

	/* Set CK_OUT_EN bit of DLL_CONFIG register to 1. */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_CK_OUT_EN), host->ioaddr + CORE_DLL_CONFIG);

	/* Wait until CK_OUT_EN bit of DLL_CONFIG register becomes '1' */
	rc = msm_dll_poll_ck_out_en(host, 1);
	if (rc)
		goto err_out;

	config = readl_relaxed(host->ioaddr + CORE_DLL_CONFIG);
	config |= CORE_CDR_EN;
	config &= ~CORE_CDR_EXT_EN;
	writel_relaxed(config, host->ioaddr + CORE_DLL_CONFIG);
	goto out;

err_out:
	dev_err(mmc_dev(mmc), "%s: Failed to set DLL phase: %d\n",
	       mmc_hostname(mmc), phase);
out:
	spin_unlock_irqrestore(&host->lock, flags);
	return rc;
}

/*
 * Find out the greatest range of consecuitive selected
 * DLL clock output phases that can be used as sampling
 * setting for SD3.0 UHS-I card read operation (in SDR104
 * timing mode) or for eMMC4.5 card read operation (in HS200
 * timing mode).
 * Select the 3/4 of the range and configure the DLL with the
 * selected DLL clock output phase.
 */

static int msm_find_most_appropriate_phase(struct sdhci_host *host,
					   u8 *phase_table, u8 total_phases)
{
	int ret;
	u8 ranges[MAX_PHASES][MAX_PHASES] = { {0}, {0} };
	u8 phases_per_row[MAX_PHASES] = { 0 };
	int row_index = 0, col_index = 0, selected_row_index = 0, curr_max = 0;
	int i, cnt, phase_0_raw_index = 0, phase_15_raw_index = 0;
	bool phase_0_found = false, phase_15_found = false;
	struct mmc_host *mmc = host->mmc;

	if (!total_phases || (total_phases > MAX_PHASES)) {
		dev_err(mmc_dev(mmc), "%s: invalid argument: total_phases=%d\n",
		       mmc_hostname(mmc), total_phases);
		return -EINVAL;
	}

	for (cnt = 0; cnt < total_phases; cnt++) {
		ranges[row_index][col_index] = phase_table[cnt];
		phases_per_row[row_index] += 1;
		col_index++;

		if ((cnt + 1) == total_phases) {
			continue;
		/* check if next phase in phase_table is consecutive or not */
		} else if ((phase_table[cnt] + 1) != phase_table[cnt + 1]) {
			row_index++;
			col_index = 0;
		}
	}

	if (row_index >= MAX_PHASES)
		return -EINVAL;

	/* Check if phase-0 is present in first valid window? */
	if (!ranges[0][0]) {
		phase_0_found = true;
		phase_0_raw_index = 0;
		/* Check if cycle exist between 2 valid windows */
		for (cnt = 1; cnt <= row_index; cnt++) {
			if (phases_per_row[cnt]) {
				for (i = 0; i < phases_per_row[cnt]; i++) {
					if (ranges[cnt][i] == 15) {
						phase_15_found = true;
						phase_15_raw_index = cnt;
						break;
					}
				}
			}
		}
	}

	/* If 2 valid windows form cycle then merge them as single window */
	if (phase_0_found && phase_15_found) {
		/* number of phases in raw where phase 0 is present */
		u8 phases_0 = phases_per_row[phase_0_raw_index];
		/* number of phases in raw where phase 15 is present */
		u8 phases_15 = phases_per_row[phase_15_raw_index];

		if (phases_0 + phases_15 >= MAX_PHASES)
			/*
			 * If there are more than 1 phase windows then total
			 * number of phases in both the windows should not be
			 * more than or equal to MAX_PHASES.
			 */
			return -EINVAL;

		/* Merge 2 cyclic windows */
		i = phases_15;
		for (cnt = 0; cnt < phases_0; cnt++) {
			ranges[phase_15_raw_index][i] =
			    ranges[phase_0_raw_index][cnt];
			if (++i >= MAX_PHASES)
				break;
		}

		phases_per_row[phase_0_raw_index] = 0;
		phases_per_row[phase_15_raw_index] = phases_15 + phases_0;
	}

	for (cnt = 0; cnt <= row_index; cnt++) {
		if (phases_per_row[cnt] > curr_max) {
			curr_max = phases_per_row[cnt];
			selected_row_index = cnt;
		}
	}

	i = ((curr_max * 3) / 4);
	if (i)
		i--;

	ret = (int)ranges[selected_row_index][i];

	if (ret >= MAX_PHASES) {
		ret = -EINVAL;
		dev_err(mmc_dev(mmc), "%s: invalid phase selected=%d\n",
		       mmc_hostname(mmc), ret);
	}

	return ret;
}

static inline void msm_cm_dll_set_freq(struct sdhci_host *host)
{
	u32 mclk_freq = 0;

	/* Program the MCLK value to MCLK_FREQ bit field */
	if (host->clock <= 112000000)
		mclk_freq = 0;
	else if (host->clock <= 125000000)
		mclk_freq = 1;
	else if (host->clock <= 137000000)
		mclk_freq = 2;
	else if (host->clock <= 150000000)
		mclk_freq = 3;
	else if (host->clock <= 162000000)
		mclk_freq = 4;
	else if (host->clock <= 175000000)
		mclk_freq = 5;
	else if (host->clock <= 187000000)
		mclk_freq = 6;
	else if (host->clock <= 200000000)
		mclk_freq = 7;

	writel_relaxed(((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			 & ~(7 << 24)) | (mclk_freq << 24)),
		       host->ioaddr + CORE_DLL_CONFIG);
}

/* Initialize the DLL (Programmable Delay Line ) */
static int msm_init_cm_dll(struct sdhci_host *host)
{
	struct mmc_host *mmc = host->mmc;
	int wait_cnt = 50;
	unsigned long flags;

	spin_lock_irqsave(&host->lock, flags);

	/*
	 * Make sure that clock is always enabled when DLL
	 * tuning is in progress. Keeping PWRSAVE ON may
	 * turn off the clock.
	 */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_VENDOR_SPEC)
			& ~CORE_CLK_PWRSAVE), host->ioaddr + CORE_VENDOR_SPEC);

	/* Write 1 to DLL_RST bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_DLL_RST), host->ioaddr + CORE_DLL_CONFIG);

	/* Write 1 to DLL_PDN bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_DLL_PDN), host->ioaddr + CORE_DLL_CONFIG);
	msm_cm_dll_set_freq(host);

	/* Write 0 to DLL_RST bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			& ~CORE_DLL_RST), host->ioaddr + CORE_DLL_CONFIG);

	/* Write 0 to DLL_PDN bit of DLL_CONFIG register */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			& ~CORE_DLL_PDN), host->ioaddr + CORE_DLL_CONFIG);

	/* Set DLL_EN bit to 1. */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_DLL_EN), host->ioaddr + CORE_DLL_CONFIG);

	/* Set CK_OUT_EN bit to 1. */
	writel_relaxed((readl_relaxed(host->ioaddr + CORE_DLL_CONFIG)
			| CORE_CK_OUT_EN), host->ioaddr + CORE_DLL_CONFIG);

	/* Wait until DLL_LOCK bit of DLL_STATUS register becomes '1' */
	while (!(readl_relaxed(host->ioaddr + CORE_DLL_STATUS) &
		 CORE_DLL_LOCK)) {
		/* max. wait for 50us sec for LOCK bit to be set */
		if (--wait_cnt == 0) {
			dev_err(mmc_dev(mmc), "%s: DLL failed to LOCK\n",
			       mmc_hostname(mmc));
			spin_unlock_irqrestore(&host->lock, flags);
			return -ETIMEDOUT;
		}
		/* wait for 1us before polling again */
		udelay(1);
	}

	spin_unlock_irqrestore(&host->lock, flags);
	return 0;
}

int sdhci_msm_execute_tuning(struct sdhci_host *host, u32 opcode)
{
	int tuning_seq_cnt = 3;
	u8 phase, *data_buf, tuned_phases[16], tuned_phase_cnt = 0;
	const u32 *tuning_block_pattern = tuning_block_64;
	int size = sizeof(tuning_block_64);	/* Pattern size in bytes */
	int rc;
	struct mmc_host *mmc = host->mmc;
	struct mmc_ios ios = host->mmc->ios;

	/*
	 * Tuning is required for SDR104, HS200 and HS400 cards and
	 * if clock frequency is greater than 100MHz in these modes.
	 */
	if (host->clock <= 100 * 1000 * 1000 ||
	    !((ios.timing == MMC_TIMING_MMC_HS200) ||
	      (ios.timing == MMC_TIMING_UHS_SDR104)))
		return 0;

	if ((opcode == MMC_SEND_TUNING_BLOCK_HS200) &&
	    (mmc->ios.bus_width == MMC_BUS_WIDTH_8)) {
		tuning_block_pattern = tuning_block_128;
		size = sizeof(tuning_block_128);
	}

	data_buf = kmalloc(size, GFP_KERNEL);
	if (!data_buf)
		return -ENOMEM;

retry:
	/* first of all reset the tuning block */
	rc = msm_init_cm_dll(host);
	if (rc)
		goto out;

	phase = 0;
	do {
		struct mmc_command cmd = { 0 };
		struct mmc_data data = { 0 };
		struct mmc_request mrq = {
			.cmd = &cmd,
			.data = &data
		};
		struct scatterlist sg;

		/* set the phase in delay line hw block */
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			goto out;

		cmd.opcode = opcode;
		cmd.flags = MMC_RSP_R1 | MMC_CMD_ADTC;

		data.blksz = size;
		data.blocks = 1;
		data.flags = MMC_DATA_READ;
		data.timeout_ns = 1000 * 1000 * 1000;	/* 1 sec */

		data.sg = &sg;
		data.sg_len = 1;
		sg_init_one(&sg, data_buf, sizeof(data_buf));
		memset(data_buf, 0, sizeof(data_buf));
		mmc_wait_for_req(mmc, &mrq);

		if (!cmd.error && !data.error &&
		    !memcmp(data_buf, tuning_block_pattern, sizeof(data_buf))) {
			/* tuning is successful at this tuning point */
			tuned_phases[tuned_phase_cnt++] = phase;
			dev_dbg(mmc_dev(mmc), "%s: found good phase = %d\n",
				 mmc_hostname(mmc), phase);
		}
	} while (++phase < 16);

	if (tuned_phase_cnt) {
		rc = msm_find_most_appropriate_phase(host, tuned_phases,
						     tuned_phase_cnt);
		if (rc < 0)
			goto out;
		else
			phase = (u8) rc;

		/*
		 * Finally set the selected phase in delay
		 * line hw block.
		 */
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc)
			goto out;
		dev_dbg(mmc_dev(mmc), "%s: setting the tuning phase to %d\n",
			 mmc_hostname(mmc), phase);
	} else {
		if (--tuning_seq_cnt)
			goto retry;
		/* tuning failed */
		dev_dbg(mmc_dev(mmc), "%s: no tuning point found\n",
		       mmc_hostname(mmc));
		rc = -EIO;
	}

out:
	kfree(data_buf);
	return rc;
}

#define MAX_PROP_SIZE 32
static int sdhci_msm_dt_parse_vreg_info(struct device *dev,
			struct sdhci_msm_reg_data *vreg, const char *vreg_name)
{
	char prop_name[MAX_PROP_SIZE];
	struct device_node *np = dev->of_node;

	vreg->name = vreg_name;

	snprintf(prop_name, MAX_PROP_SIZE, "qcom,%s-voltage-min", vreg_name);
	of_property_read_u32(np, prop_name, &vreg->low_vol_level);
	snprintf(prop_name, MAX_PROP_SIZE, "qcom,%s-voltage-max", vreg_name);
	of_property_read_u32(np, prop_name, &vreg->high_vol_level);

	/* sanity check */
	if (vreg->low_vol_level > vreg->high_vol_level) {
		dev_err(dev, "%s invalid constraints specified\n", vreg->name);
		return -EINVAL;
	}

	return 0;
}

/* Parse devicetree data */
static int sdhci_msm_populate_pdata(struct device *dev,
				    struct sdhci_msm_pltfm_data *pdata)
{
	if (sdhci_msm_dt_parse_vreg_info(dev, &pdata->vdd, "vdd")) {
		dev_err(dev, "failed parsing vdd data\n");
		return -EINVAL;
	}

	if (sdhci_msm_dt_parse_vreg_info(dev, &pdata->vdd_io, "vdd-io")) {
		dev_err(dev, "failed parsing vdd-io data\n");
		return -EINVAL;
	}

	return 0;
}

static int sdhci_msm_vreg_enable(struct device *dev,
				 struct sdhci_msm_reg_data *vreg)
{
	int ret = 0;

	if (!regulator_is_enabled(vreg->reg)) {
		/* Set voltage level */
		ret = regulator_set_voltage(vreg->reg, vreg->high_vol_level,
					    vreg->high_vol_level);
		if (ret)
			return ret;
	}

	ret = regulator_enable(vreg->reg);
	if (ret) {
		dev_err(dev, "regulator_enable(%s) fail (%d)\n",
			vreg->name, ret);
	}

	return ret;
}

static int sdhci_msm_vreg_disable(struct device *dev,
				  struct sdhci_msm_reg_data *vreg)
{
	int ret = 0;

	if (!regulator_is_enabled(vreg->reg))
		return ret;

	/* Set min. voltage to 0 */
	ret = regulator_set_voltage(vreg->reg, 0, vreg->high_vol_level);
	if (ret)
		return ret;

	ret = regulator_disable(vreg->reg);
	if (ret) {
		dev_err(dev, "regulator_disable(%s) fail (%d)\n",
			vreg->name, ret);
	}

	return ret;
}

static int sdhci_msm_setup_vreg(struct sdhci_msm_host *msm_host, bool enable)
{
	int ret, i;
	struct sdhci_msm_reg_data *vreg_table[2];

	vreg_table[0] = &msm_host->pdata.vdd;
	vreg_table[1] = &msm_host->pdata.vdd_io;

	for (i = 0; i < ARRAY_SIZE(vreg_table); i++) {
		if (enable)
			ret = sdhci_msm_vreg_enable(&msm_host->pdev->dev,
						    vreg_table[i]);
		else
			ret = sdhci_msm_vreg_disable(&msm_host->pdev->dev,
						     vreg_table[i]);
		if (ret)
			return ret;
	}

	return 0;
}

/* This init function should be called only once for each SDHC slot */
static int sdhci_msm_vreg_init(struct device *dev,
			       struct sdhci_msm_pltfm_data *pdata)
{
	struct sdhci_msm_reg_data *vdd_reg = &pdata->vdd;
	struct sdhci_msm_reg_data *vdd_io_reg = &pdata->vdd_io;

	vdd_reg->reg = devm_regulator_get(dev, vdd_reg->name);
	if (IS_ERR(vdd_reg->reg))
		return PTR_ERR(vdd_reg->reg);

	vdd_io_reg->reg = devm_regulator_get(dev, vdd_io_reg->name);
	if (IS_ERR(vdd_io_reg->reg))
		return PTR_ERR(vdd_io_reg->reg);

	return 0;
}

static irqreturn_t sdhci_msm_pwr_irq(int irq, void *data)
{
	struct sdhci_host *host = (struct sdhci_host *)data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	u8 irq_status;
	u8 irq_ack = 0;
	int ret = 0;

	irq_status = readb_relaxed(msm_host->core_mem + CORE_PWRCTL_STATUS);
	dev_dbg(mmc_dev(msm_host->mmc), "%s: Received IRQ(%d), status=0x%x\n",
		mmc_hostname(msm_host->mmc), irq, irq_status);

	/* Clear the interrupt */
	writeb_relaxed(irq_status, (msm_host->core_mem + CORE_PWRCTL_CLEAR));
	/*
	 * SDHC has core_mem and hc_mem device memory and these memory
	 * addresses do not fall within 1KB region. Hence, any update to
	 * core_mem address space would require an mb() to ensure this gets
	 * completed before its next update to registers within hc_mem.
	 */
	mb();

	/* Handle BUS ON/OFF */
	if (irq_status & CORE_PWRCTL_BUS_ON) {
		ret = sdhci_msm_setup_vreg(msm_host, true);
		if (!ret)
			ret = regulator_set_voltage(msm_host->pdata.vdd_io.reg,
						    msm_host->pdata.
						    vdd_io.high_vol_level,
						    msm_host->pdata.
						    vdd_io.high_vol_level);
		if (ret)
			irq_ack |= CORE_PWRCTL_BUS_FAIL;
		else
			irq_ack |= CORE_PWRCTL_BUS_SUCCESS;
	}

	if (irq_status & CORE_PWRCTL_BUS_OFF) {
		ret = sdhci_msm_setup_vreg(msm_host, false);
		if (!ret)
			ret = regulator_set_voltage(msm_host->pdata.vdd_io.reg,
						    msm_host->pdata.
						    vdd_io.low_vol_level,
						    msm_host->pdata.
						    vdd_io.low_vol_level);
		if (ret)
			irq_ack |= CORE_PWRCTL_BUS_FAIL;
		else
			irq_ack |= CORE_PWRCTL_BUS_SUCCESS;
	}

	/* Handle IO LOW/HIGH */
	if (irq_status & CORE_PWRCTL_IO_LOW) {
		ret = regulator_set_voltage(msm_host->pdata.vdd_io.reg,
					    msm_host->pdata.
					    vdd_io.low_vol_level,
					    msm_host->pdata.
					    vdd_io.low_vol_level);
		if (ret)
			irq_ack |= CORE_PWRCTL_IO_FAIL;
		else
			irq_ack |= CORE_PWRCTL_IO_SUCCESS;
	}

	if (irq_status & CORE_PWRCTL_IO_HIGH) {
		ret = regulator_set_voltage(msm_host->pdata.vdd_io.reg,
					    msm_host->pdata.
					    vdd_io.high_vol_level,
					    msm_host->pdata.
					    vdd_io.high_vol_level);
		if (ret)
			irq_ack |= CORE_PWRCTL_IO_FAIL;
		else
			irq_ack |= CORE_PWRCTL_IO_SUCCESS;
	}

	/* ACK status to the core */
	writeb_relaxed(irq_ack, (msm_host->core_mem + CORE_PWRCTL_CTL));
	/*
	 * SDHC has core_mem and hc_mem device memory and these memory
	 * addresses do not fall within 1KB region. Hence, any update to
	 * core_mem address space would require an mb() to ensure this gets
	 * completed before its next update to registers within hc_mem.
	 */
	mb();

	dev_dbg(mmc_dev(msm_host->mmc), "%s: Handled IRQ(%d), ret=%d, ack=0x%x\n",
		 mmc_hostname(msm_host->mmc), irq, ret, irq_ack);
	return IRQ_HANDLED;
}

static const struct of_device_id sdhci_msm_dt_match[] = {
	{ .compatible = "qcom,sdhci-msm" },
	{},
};

MODULE_DEVICE_TABLE(of, sdhci_msm_dt_match);

static struct sdhci_ops sdhci_msm_ops = {
	.platform_execute_tuning = sdhci_msm_execute_tuning,
};

static int sdhci_msm_probe(struct platform_device *pdev)
{
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_msm_host *msm_host;
	struct resource *core_memres = NULL;
	int ret, dead;
	u16 host_version;

	if (!pdev->dev.of_node) {
		dev_err(&pdev->dev, "No device tree data\n");
		return -ENOENT;
	}

	msm_host = devm_kzalloc(&pdev->dev, sizeof(*msm_host), GFP_KERNEL);
	if (!msm_host)
		return -ENOMEM;

	msm_host->sdhci_msm_pdata.ops = &sdhci_msm_ops;
	host = sdhci_pltfm_init(pdev, &msm_host->sdhci_msm_pdata, 0);
	if (IS_ERR(host)) {
		dev_err(&pdev->dev, "sdhci_pltfm_init error\n");
		return PTR_ERR(host);
	}

	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = msm_host;
	msm_host->mmc = host->mmc;
	msm_host->pdev = pdev;

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		dev_err(&pdev->dev, "failed parsing mmc device tree\n");
		goto pltfm_free;
	}

	sdhci_get_of_property(pdev);

	ret = sdhci_msm_populate_pdata(&pdev->dev, &msm_host->pdata);
	if (ret) {
		dev_err(&pdev->dev, "DT parsing error\n");
		goto pltfm_free;
	}

	/* Setup SDCC bus voter clock. */
	msm_host->bus_clk = devm_clk_get(&pdev->dev, "bus");
	if (!IS_ERR(msm_host->bus_clk)) {
		/* Vote for max. clk rate for max. performance */
		ret = clk_set_rate(msm_host->bus_clk, INT_MAX);
		if (ret)
			goto pltfm_free;
		ret = clk_prepare_enable(msm_host->bus_clk);
		if (ret)
			goto pltfm_free;
	}

	/* Setup main peripheral bus clock */
	msm_host->pclk = devm_clk_get(&pdev->dev, "iface");
	if (!IS_ERR(msm_host->pclk)) {
		ret = clk_prepare_enable(msm_host->pclk);
		if (ret) {
			dev_err(&pdev->dev,
				"Main peripheral clock setup fail (%d)\n", ret);
			goto bus_clk_disable;
		}
	}

	/* Setup SDC MMC clock */
	msm_host->clk = devm_clk_get(&pdev->dev, "core");
	if (IS_ERR(msm_host->clk)) {
		ret = PTR_ERR(msm_host->clk);
		dev_err(&pdev->dev, "SDC MMC clock setup fail (%d)\n", ret);
		goto pclk_disable;
	}

	ret = clk_prepare_enable(msm_host->clk);
	if (ret)
		goto pclk_disable;

	/* Setup regulators */
	ret = sdhci_msm_vreg_init(&pdev->dev, &msm_host->pdata);
	if (ret) {
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Regulator setup fail (%d)\n", ret);
		goto clk_disable;
	}

	core_memres = platform_get_resource_byname(pdev,
						   IORESOURCE_MEM, "core_mem");
	msm_host->core_mem = devm_ioremap_resource(&pdev->dev, core_memres);

	if (IS_ERR(msm_host->core_mem)) {
		dev_err(&pdev->dev, "Failed to remap registers\n");
		ret = PTR_ERR(msm_host->core_mem);
		goto vreg_disable;
	}

	/* Reset the core and Enable SDHC mode */
	writel_relaxed(readl_relaxed(msm_host->core_mem + CORE_POWER) |
		       CORE_SW_RST, msm_host->core_mem + CORE_POWER);

	/* SW reset can take upto 10HCLK + 15MCLK cycles. (min 40us) */
	usleep_range(1000, 5000);
	if (readl(msm_host->core_mem + CORE_POWER) & CORE_SW_RST) {
		dev_err(&pdev->dev, "Stuck in reset\n");
		ret = -ETIMEDOUT;
		goto vreg_disable;
	}

	/* Set HC_MODE_EN bit in HC_MODE register */
	writel_relaxed(HC_MODE_EN, (msm_host->core_mem + CORE_HC_MODE));

	/*
	 * Following are the deviations from SDHC spec v3.0 -
	 * 1. Card detection is handled using separate GPIO.
	 * 2. Bus power control is handled by interacting with PMIC.
	 */
	host->quirks |= SDHCI_QUIRK_BROKEN_CARD_DETECTION;
	host->quirks |= SDHCI_QUIRK_SINGLE_POWER_WRITE;

	host_version = readw_relaxed((host->ioaddr + SDHCI_HOST_VERSION));
	dev_dbg(&pdev->dev, "Host Version: 0x%x Vendor Version 0x%x\n",
		host_version, ((host_version & SDHCI_VENDOR_VER_MASK) >>
			       SDHCI_VENDOR_VER_SHIFT));

	/* Setup PWRCTL irq */
	msm_host->pwr_irq = platform_get_irq_byname(pdev, "pwr_irq");
	if (msm_host->pwr_irq < 0) {
		dev_err(&pdev->dev, "Failed to get pwr_irq by name (%d)\n",
			msm_host->pwr_irq);
		goto vreg_disable;
	}
	ret = devm_request_threaded_irq(&pdev->dev, msm_host->pwr_irq, NULL,
					sdhci_msm_pwr_irq, IRQF_ONESHOT,
					dev_name(&pdev->dev), host);
	if (ret) {
		dev_err(&pdev->dev, "Request threaded irq(%d) fail (%d)\n",
			msm_host->pwr_irq, ret);
		goto vreg_disable;
	}

	/* Enable pwr irq interrupts */
	writel_relaxed(INT_MASK, (msm_host->core_mem + CORE_PWRCTL_MASK));

	msm_host->mmc->caps |= msm_host->pdata.caps;
	msm_host->mmc->caps2 |= msm_host->pdata.caps2;

	ret = sdhci_add_host(host);
	if (ret) {
		dev_err(&pdev->dev, "Add host fail (%d)\n", ret);
		goto vreg_disable;
	}

	ret = clk_set_rate(msm_host->clk, host->max_clk);
	if (ret) {
		dev_err(&pdev->dev, "MClk rate set fail (%d)\n", ret);
		goto remove_host;
	}

	return 0;

remove_host:
	dead = (readl_relaxed(host->ioaddr + SDHCI_INT_STATUS) == 0xffffffff);
	sdhci_remove_host(host, dead);
vreg_disable:
	if (!IS_ERR(msm_host->pdata.vdd.reg))
		sdhci_msm_vreg_disable(&pdev->dev, &msm_host->pdata.vdd);
	if (!IS_ERR(msm_host->pdata.vdd_io.reg))
		sdhci_msm_vreg_disable(&pdev->dev, &msm_host->pdata.vdd_io);
clk_disable:
	if (!IS_ERR(msm_host->clk))
		clk_disable_unprepare(msm_host->clk);
pclk_disable:
	if (!IS_ERR(msm_host->pclk))
		clk_disable_unprepare(msm_host->pclk);
bus_clk_disable:
	if (!IS_ERR(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
pltfm_free:
	sdhci_pltfm_free(pdev);
	return ret;
}

static int sdhci_msm_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_msm_host *msm_host = pltfm_host->priv;
	int dead = (readl_relaxed(host->ioaddr + SDHCI_INT_STATUS) ==
		    0xffffffff);

	sdhci_remove_host(host, dead);
	sdhci_pltfm_free(pdev);
	sdhci_msm_vreg_disable(&pdev->dev, &msm_host->pdata.vdd);
	sdhci_msm_vreg_disable(&pdev->dev, &msm_host->pdata.vdd_io);
	clk_disable_unprepare(msm_host->clk);
	clk_disable_unprepare(msm_host->pclk);
	if (!IS_ERR(msm_host->bus_clk))
		clk_disable_unprepare(msm_host->bus_clk);
	return 0;
}

static struct platform_driver sdhci_msm_driver = {
	.probe = sdhci_msm_probe,
	.remove = sdhci_msm_remove,
	.driver = {
		   .name = "sdhci_msm",
		   .owner = THIS_MODULE,
		   .of_match_table = sdhci_msm_dt_match,
	},
};

module_platform_driver(sdhci_msm_driver);

MODULE_DESCRIPTION("Qualcomm Secure Digital Host Controller Interface driver");
MODULE_LICENSE("GPL v2");
