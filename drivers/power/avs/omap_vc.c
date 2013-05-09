/*
 * OMAP Voltage Controller (VC) interface
 *
 * Idea based on arch/arm/mach-omap2/vc.c
 * Copyright (C) 2011 Texas Instruments Incorporated.
 *
 * Copyright (C) 2013 Texas Instruments Incorporated
 * Taras Kondratiuk
 * Grygorii Strashko
 * Nishanth Menon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regulator/omap-pmic-regulator.h>
#include <linux/regmap.h>
#include <linux/err.h>
#include <linux/clk.h>
#include "omap_vc.h"

#define DRIVER_NAME	"omap-vc"

/**
 * struct vc_channel_regs - Register description for VC channel
 * @cfg_reg_offset:  Channel configuration register offset
 * @sa_reg_offset: Slave Address configuration register offset
 * @voltage_reg_offset: Voltage address register offset
 * @command_addr_reg_offset: Command address register offset
 * @command_val_reg_offset: Command value register offset
 *
 * NOTE: All register offsets are from VC base, as certain flavor
 * of SoCs use shared registers and other have unique register offsets
 */
struct vc_channel_regs {
	u16 cfg_reg_offset;
	u16 sa_reg_offset;
	u16 voltage_reg_offset;
	u16 command_addr_reg_offset;
	u16 command_val_reg_offset;

	/* private: */
	/* cfg_reg_offset details */
	u32 racen_mask;
	u32 rac_enable_mask;
	u32 rav_enable_mask;
	u32 cmd_sel_mask;
	u32 sa_enable_mask;

	/* sa_reg_offset details */
	u32 sa_addr_mask;

	/* voltage_reg_offset details */
	u32 voltage_addr_mask;

	/* command_addr_reg_offset details */
	u32 command_addr_mask;

	/* command_val_reg_offset details */
	u32 command_val_on_mask;
	u32 command_val_onlp_mask;
	u32 command_val_ret_mask;
	u32 command_val_off_mask;
};

/**
 * struct omap_vc_channel - internal representation of VC channel
 * @dev:		device pointer for VC channel device
 * @list:		channel list
 * @usage_count:	Usage count - only 1 user at a time.(not always module)
 * @is_master_channel:	Is this channel the master channel?
 * @use_master_channel_sa: if this channel uses master channel's slave address
 * @use_master_channel_voltage_reg: if this channel uses master channel's
 *			voltage register address
 * @use_master_channel_cmd_reg: if this channel uses master channel's command
 *			register address
 * @use_master_channel_cmd_val: if this channel uses master channel's command
 *			value address
 * @ch_regs:		Channel register description
 * @info:		exported information containing PMIC hooked to this
 *			channel, low power state voltage information etc.
 */
struct omap_vc_channel {
	struct device *dev;
	struct list_head list;
	int usage_count;
	bool is_master_channel;
	bool use_master_channel_sa;
	bool use_master_channel_voltage_reg;
	bool use_master_channel_cmd_reg;
	bool use_master_channel_cmd_val;
	const struct vc_channel_regs *ch_regs;
	struct omap_vc_channel_info info;
};

/**
 * struct omap_omap_vc_i2c_config - Voltage controller channel's I2C config
 * @clk_rate:		Frequency of sys_clock for the voltage controller's I2C
 * @highspeed:		used in I2C highspeed mode?
 * @mcode:		Master Code used in High speed mode
 * @i2c_clk_pad_load:	What is the pad load at i2c
 * @i2c_clk_pcb_length:	What is the pcb length to the PMIC (all inclusive)
 * @i2c_clk_scll:	Clock low timing config for slow speed
 * @i2c_clk_sclh:	Clock high timing config for slow speed
 * @i2c_clk_hsscll:	Clock low timing config for high speed
 * @i2c_clk_hssclh:	Clock high timing config for high speed
 *
 * Used one time at boot.
 */
struct omap_omap_vc_i2c_config {
	unsigned long clk_rate;
	bool highspeed;
	u8 mcode;
	u32 i2c_clk_pad_load;
	u32 i2c_clk_pcb_length;
	u8 i2c_clk_scll;
	u8 i2c_clk_sclh;
	u8 i2c_clk_hsscll;
	u8 i2c_clk_hssclh;
};

/**
 * struct omap_vc_common_reg - Voltage Controller Common registers
 * @i2c_clk_cfg_reg:	I2C clock configuration register offset
 * @i2c_clk_mode_reg:	I2C clock operation mode register offset
 * @bypass_cmd_reg:	VC_BYPASS (very low level control) register offset
 *
 * These may be -1 to indicate register not being present.
 */
struct omap_vc_common_reg {
	s16 i2c_clk_cfg_reg;
	u16 i2c_clk_mode_reg;
	u16 bypass_cmd_reg;
};

/**
 * struct omap_vc - represents the voltage controller(vc) device
 * @regmap:	regmap to the entire voltage controller region(includes channel)
 * @regs:	Voltage Controller common registers
 * @master_channel_configured:	Is Master channel has been configured.
 */
struct omap_vc {
	struct regmap *regmap;
	const struct omap_vc_common_reg *regs;
	bool master_channel_configured;
};

/* VC Channel register configurations */
static const struct vc_channel_regs omap3_ch_1_regs = {
	.cfg_reg_offset = 0x14,
	.racen_mask = 0x80000,
	.rac_enable_mask = 0x40000,
	.rav_enable_mask = 0x20000,
	.cmd_sel_mask = 0x100000,
	.sa_enable_mask = 0x10000,

	.sa_reg_offset = 0x0,
	.sa_addr_mask = 0x7f0000,

	.voltage_reg_offset = 0x4,
	.voltage_addr_mask = 0xFF0000,

	.command_addr_reg_offset = 0x8,
	.command_addr_mask = 0xFF0000,

	.command_val_reg_offset = 0x10,
	.command_val_on_mask = 0xFF000000,
	.command_val_onlp_mask = 0xFF0000,
	.command_val_ret_mask = 0xFF00,
	.command_val_off_mask = 0xFF,
};

static const struct vc_channel_regs omap3_ch_0_regs = {
	.cfg_reg_offset = 0x14,
	.racen_mask = 0x8,
	.rac_enable_mask = 0x4,
	.rav_enable_mask = 0x2,
	.cmd_sel_mask = 0x10,
	.sa_enable_mask = 0x1,

	.sa_reg_offset = 0x0,
	.sa_addr_mask = 0x7f,

	.voltage_reg_offset = 0x4,
	.voltage_addr_mask = 0xFF,

	.command_addr_reg_offset = 0x8,
	.command_addr_mask = 0xFF,

	.command_val_reg_offset = 0xC,
	.command_val_on_mask = 0xFF000000,
	.command_val_onlp_mask = 0xFF0000,
	.command_val_ret_mask = 0xFF00,
	.command_val_off_mask = 0xFF,
};

static const struct vc_channel_regs omap4_ch_mpu_regs = {
	.cfg_reg_offset = 0x1c,
	.racen_mask = 0x100000,
	.rac_enable_mask = 0x80000,
	.rav_enable_mask = 0x40000,
	.cmd_sel_mask = 0x20000,
	.sa_enable_mask = 0x10000,

	.sa_reg_offset = 0x0,
	.sa_addr_mask = 0x7f0000,

	.voltage_reg_offset = 0x4,
	.voltage_addr_mask = 0xFF0000,

	.command_addr_reg_offset = 0x8,
	.command_addr_mask = 0xFF0000,

	.command_val_reg_offset = 0x10,
	.command_val_on_mask = 0xFF000000,
	.command_val_onlp_mask = 0xFF0000,
	.command_val_ret_mask = 0xFF00,
	.command_val_off_mask = 0xFF,
};

static const struct vc_channel_regs omap4_ch_iva_regs = {
	.cfg_reg_offset = 0x1c,
	.racen_mask = 0x800,
	.rac_enable_mask = 0x400,
	.rav_enable_mask = 0x200,
	.cmd_sel_mask = 0x1000,
	.sa_enable_mask = 0x100,

	.sa_reg_offset = 0x0,
	.sa_addr_mask = 0x7f00,

	.voltage_reg_offset = 0x4,
	.voltage_addr_mask = 0xFF00,

	.command_addr_reg_offset = 0x8,
	.command_addr_mask = 0xFF00,

	.command_val_reg_offset = 0x14,
	.command_val_on_mask = 0xFF000000,
	.command_val_onlp_mask = 0xFF0000,
	.command_val_ret_mask = 0xFF00,
	.command_val_off_mask = 0xFF,
};

static const struct vc_channel_regs omap4_ch_core_regs = {
	.cfg_reg_offset = 0x1c,
	.racen_mask = 0x8,
	.rac_enable_mask = 0x4,
	.rav_enable_mask = 0x2,
	.cmd_sel_mask = 0x10,
	.sa_enable_mask = 0x1,

	.sa_reg_offset = 0x0,
	.sa_addr_mask = 0x7f,

	.voltage_reg_offset = 0x4,
	.voltage_addr_mask = 0xFF,

	.command_addr_reg_offset = 0x8,
	.command_addr_mask = 0xFF,

	.command_val_reg_offset = 0xC,
	.command_val_on_mask = 0xFF000000,
	.command_val_onlp_mask = 0xFF0000,
	.command_val_ret_mask = 0xFF00,
	.command_val_off_mask = 0xFF,
};

static const struct vc_channel_regs omap5_ch_core_regs = {
	.cfg_reg_offset = 0x0,
	.racen_mask = 0x8000000,
	.rac_enable_mask = 0x4000000,
	.rav_enable_mask = 0x2000000,
	.cmd_sel_mask = 0x10000000,
	.sa_enable_mask = 0x1000000,

	.sa_reg_offset = 0x0,
	.sa_addr_mask = 0x7f,

	.voltage_reg_offset = 0x0,
	.voltage_addr_mask = 0xFF00,

	.command_addr_reg_offset = 0x0,
	.command_addr_mask = 0xFF0000,

	.command_val_reg_offset = 0xC,
	.command_val_on_mask = 0xFF000000,
	.command_val_onlp_mask = 0xFF0000,
	.command_val_ret_mask = 0xFF00,
	.command_val_off_mask = 0x00,	/* Reserved */
};

static const struct vc_channel_regs omap5_ch_mm_regs = {
	.cfg_reg_offset = 0x4,
	.racen_mask = 0x8000000,
	.rac_enable_mask = 0x4000000,
	.rav_enable_mask = 0x2000000,
	.cmd_sel_mask = 0x10000000,
	.sa_enable_mask = 0x1000000,

	.sa_reg_offset = 0x4,
	.sa_addr_mask = 0x7f,

	.voltage_reg_offset = 0x4,
	.voltage_addr_mask = 0xFF00,

	.command_addr_reg_offset = 0x4,
	.command_addr_mask = 0xFF0000,

	.command_val_reg_offset = 0x10,
	.command_val_on_mask = 0xFF000000,
	.command_val_onlp_mask = 0xFF0000,
	.command_val_ret_mask = 0xFF00,
	.command_val_off_mask = 0x00,	/* Reserved */
};

static const struct vc_channel_regs omap5_ch_mpu_regs = {
	.cfg_reg_offset = 0x8,
	.racen_mask = 0x8000000,
	.rac_enable_mask = 0x4000000,
	.rav_enable_mask = 0x2000000,
	.cmd_sel_mask = 0x10000000,
	.sa_enable_mask = 0x1000000,

	.sa_reg_offset = 0x8,
	.sa_addr_mask = 0x7f,

	.voltage_reg_offset = 0x8,
	.voltage_addr_mask = 0xFF00,

	.command_addr_reg_offset = 0x8,
	.command_addr_mask = 0xFF0000,

	.command_val_reg_offset = 0x14,
	.command_val_on_mask = 0xFF000000,
	.command_val_onlp_mask = 0xFF0000,
	.command_val_ret_mask = 0xFF00,
	.command_val_off_mask = 0x00,	/* Reserved */
};

/* VC Generic register configurations */
static const struct omap_vc_common_reg omap3_vc_regs = {
	.i2c_clk_cfg_reg = -1,
	/* NOTE: register is called I2C_CFG in TRM, but compatible with MODE */
	.i2c_clk_mode_reg = 0x18,
	.bypass_cmd_reg = 0x1C,
};

static const struct omap_vc_common_reg omap4_vc_regs = {
	.i2c_clk_cfg_reg = 0x24,
	.i2c_clk_mode_reg = 0x20,
	.bypass_cmd_reg = 0x18,
};

static const struct omap_vc_common_reg omap5_vc_regs = {
	.i2c_clk_cfg_reg = 0x30,
	.i2c_clk_mode_reg = 0x2c,
	.bypass_cmd_reg = 0x18,
};

/* VCBYPASS register bit definitions - No variance at all */
#define VC_BYPASS_VALID_MASK	(0x01 << 24)
#define VC_BYPASS_DATA_MASK	(0xff << 16)
#define VC_BYPASS_REGADDR_MASK	(0xff << 8)
#define VC_BYPASS_SA_MASK	(0x7f << 0)
/* I2C_CLK_MODE register bit definitions - No variance at all */
#define VC_MODE_REPEAT_START_MASK	BIT(4)
#define VC_MODE_HSENABLE_MASK		BIT(3)
#define VC_MODE_MCODE_MASK		(0x3 << 0)
/* I2C_CLK_CONFIG register bit definitions - No variance at all */
#define VC_CLK_CFG_HSSCLL_MASK		(0xff << 24)
#define VC_CLK_CFG_HSSCLH_MASK		(0xff << 16)
#define VC_CLK_CFG_SCLL_MASK		(0xff << 8)
#define VC_CLK_CFG_SCLH_MASK		(0xff << 0)

/* Stores the list containing all voltage controller channels */
static LIST_HEAD(omap_vc_channel_list);
static DEFINE_MUTEX(omap_vc_channel_list_mutex);

/**
 * omap_vc_channel_set_on_voltage() - Update the ON transition voltage
 * @info:	channel info (must be valid pointer from
 *		devm_omap_vc_channel_get)
 * @uv:		ON voltage in micro volts.
 *
 * Updates the voltage that OMAP comes back to when resuming from a low
 * power state. It is recommended to invoke this as part of voltage transition
 * as the low power transitions are more performance sensitive that voltage
 * transition path (part of DVFS).
 *
 * This needs to be invoked either as part of DVFS sequence or prior to
 * attempting to enter low power state.
 *
 * NOTE: no extra error handling is performed to reduce overhead as much as
 * sanely possible. No explicit locks are needed as regmap takes care of
 * the same.
 *
 * Return: 0 if all operations are successful, else returns appropriate
 * error value.
 */
int omap_vc_channel_set_on_voltage(struct omap_vc_channel_info *info, u32 uv)
{
	struct omap_vc *vc;
	struct omap_vc_channel *vc_channel;
	struct device *dev;
	const struct vc_channel_regs *ch_regs;
	struct omap_pmic *pmic;
	struct omap_pmic_ops *ops;
	struct regmap *regmap;
	int ret = 0;
	u32 val = 0;
	u8 vsel;

	if (!info) {
		pr_err("Bad parameters\n");
		ret = -EINVAL;
		goto out;
	}

	vc_channel = info->ch;
	dev = vc_channel->dev;
	ch_regs = vc_channel->ch_regs;
	pmic = info->pmic;
	ops = pmic->ops;

	vc = dev_get_drvdata(dev->parent);
	if (!vc) {
		dev_err(dev, "Unable to find parent VC data\n");
		ret = -EINVAL;
		goto out;
	}

	ret = ops->uv_to_vsel(pmic, uv, &vsel);
	if (ret) {
		dev_err(dev, "%s: Conversion onV %d to vsel fail(%d)\n",
			__func__, uv, ret);
		goto out;
	}
	regmap = vc->regmap;

	if (ch_regs->command_val_onlp_mask) {
		val = vsel << __ffs(ch_regs->command_val_onlp_mask);
		ret = regmap_update_bits(regmap,
					 ch_regs->command_val_reg_offset,
					 ch_regs->command_val_onlp_mask,
					 val);
	}
	if (ret)
		goto fail_reg;

	if (ch_regs->command_val_on_mask) {
		val = vsel << __ffs(ch_regs->command_val_on_mask);
		ret = regmap_update_bits(regmap,
					 ch_regs->command_val_reg_offset,
					 ch_regs->command_val_on_mask, val);
	}

fail_reg:
	if (ret)
		dev_err(dev, "%s: Register operation failed with %d\n",
			__func__, ret);
out:
	return ret;
}
EXPORT_SYMBOL_GPL(omap_vc_channel_set_on_voltage);

/**
 * omap_vc_channel_setup_lp() - Low power voltage configuration
 * @vc_channel:	VC channel to configure for
 * @vc:		VC to which this channel belongs to.
 *
 * The Low power states such as OFF, RETENTION are pre-determined SoC specific
 * voltage values. These can be configured at boot time and the voltages are
 * achieved as needed. This also configures the boot voltage as ON voltage if
 * it is available.
 */
static int omap_vc_channel_setup_lp(struct omap_vc_channel *vc_channel,
				    struct omap_vc *vc)
{
	struct device *dev = vc_channel->dev;
	const struct vc_channel_regs *ch_regs = vc_channel->ch_regs;
	struct regmap *regmap = vc->regmap;
	struct omap_pmic *pmic = vc_channel->info.pmic;
	struct omap_pmic_ops *ops = pmic->ops;
	struct omap_vc_channel_info *info = &vc_channel->info;
	int ret = 0;
	u32 val;
	u8 vsel;

	if (ch_regs->command_val_off_mask) {
		ret = ops->uv_to_vsel(pmic, info->off_uV, &vsel);
		if (ret) {
			dev_err(dev, "%s: Conversion OFF %d to vsel fail(%d)\n",
				__func__, info->off_uV, ret);
			goto out;
		}
		val = vsel << __ffs(ch_regs->command_val_off_mask);
		ret = regmap_update_bits(regmap,
					 ch_regs->command_val_reg_offset,
					 ch_regs->command_val_off_mask, val);
		if (ret)
			goto fail_reg;
	}

	if (ch_regs->command_val_ret_mask) {
		ret = ops->uv_to_vsel(pmic, info->retention_uV, &vsel);
		if (ret) {
			dev_err(dev, "%s: Conversion RET %d to vsel fail(%d)\n",
				__func__, info->retention_uV, ret);
			goto out;
		}
		val = vsel << __ffs(ch_regs->command_val_ret_mask);
		ret = regmap_update_bits(regmap,
					 ch_regs->command_val_reg_offset,
					 ch_regs->command_val_ret_mask, val);
		if (ret)
			goto fail_reg;
	}

	if (pmic->boot_voltage_uV && (ch_regs->command_val_on_mask |
				      ch_regs->command_val_onlp_mask)) {
		ret = ops->uv_to_vsel(pmic, pmic->boot_voltage_uV, &vsel);
		if (ret) {
			dev_err(dev, "%s: Conversion ON %d to vsel fail(%d)\n",
				__func__,  pmic->boot_voltage_uV, ret);
			goto out;
		}
	}
	if (pmic->boot_voltage_uV && ch_regs->command_val_on_mask) {
		val = vsel << __ffs(ch_regs->command_val_on_mask);
		ret = regmap_update_bits(regmap,
					 ch_regs->command_val_reg_offset,
					 ch_regs->command_val_on_mask, val);
		if (ret)
			goto fail_reg;
	}

	if (pmic->boot_voltage_uV && ch_regs->command_val_onlp_mask) {
		val = vsel << __ffs(ch_regs->command_val_onlp_mask);
		ret = regmap_update_bits(regmap,
					 ch_regs->command_val_reg_offset,
					 ch_regs->command_val_onlp_mask,
					 val);
	}

fail_reg:
	if (ret)
		dev_err(dev, "%s: Register operation failed with %d\n",
			__func__, ret);

out:
	return ret;
}

/**
 * omap_vc_send_msg() - Send a VC bypass command
 * @vc:		Voltage controller
 * @vc_channel:	Voltage controller channel
 * @reg_addr:	Register address to write to
 * @data:	data to write.
 *
 * This bypasses all channel scheduling mechanisms inside Voltage controller
 * and must be used sparingly in controlled environments. Using this to scale
 * voltage is NOT a recommended procedure.
 *
 * The only safe usage is when Voltage Processors, SmartReflex, VFSM(PRCM)
 * is in idle or known state - example at boot, this may be used for
 * configuration.
 */
static int omap_vc_send_msg(struct omap_vc *vc,
			    struct omap_vc_channel *vc_channel, u8 reg_addr,
			    u8 data)
{
	struct device *dev = vc_channel->dev;
	struct regmap *regmap = vc->regmap;
	struct omap_pmic *pmic = vc_channel->info.pmic;
	const struct omap_pmic_info *pinfo = pmic->info;
	u32 loop_cnt = 0, retries_cnt = 0;
	u32 vc_bypass_value;
	int ret = 0;

	ret = regmap_update_bits(regmap, vc->regs->bypass_cmd_reg,
				 VC_BYPASS_DATA_MASK,
				 data << __ffs(VC_BYPASS_DATA_MASK));
	if (ret)
		goto reg_fail;
	ret = regmap_update_bits(regmap, vc->regs->bypass_cmd_reg,
				 VC_BYPASS_REGADDR_MASK,
				 reg_addr << __ffs(VC_BYPASS_REGADDR_MASK));
	if (ret)
		goto reg_fail;
	ret = regmap_update_bits(regmap, vc->regs->bypass_cmd_reg,
				 VC_BYPASS_SA_MASK,
				 pinfo->slave_addr << __ffs(VC_BYPASS_SA_MASK));
	if (ret)
		goto reg_fail;
	/* Activate the transfer */
	ret = regmap_update_bits(regmap, vc->regs->bypass_cmd_reg,
				 VC_BYPASS_VALID_MASK, VC_BYPASS_VALID_MASK);
	if (ret)
		goto reg_fail;

	/* See if transfer complete */
	ret = regmap_read(regmap, vc->regs->bypass_cmd_reg, &vc_bypass_value);
	if (ret)
		goto reg_fail;

	dev_dbg(dev,
		"bypass_val = 0x%08x, sa=0x%02x, reg=0x%02x, data=0x%02x\n",
		vc_bypass_value, pinfo->slave_addr, reg_addr, data);

	/*
	 * Loop which polls continously 50 times before sleeping and retry
	 * around for atleast pinfo->i2c_timeout_us * 5. Rationale as follows:
	 * 1) continuous poll loops for 50 times, if it cant get it, then (2)
	 * 2) sleeps between 5-15 uSec. then try (1)
	 * if it cant get in timeout_us *5 ish (in step 1) then give up
	 *
	 * The continous loop(1) is used because the first VALID setting
	 * is expected to appear quiet fast. However, the delay occurs only
	 * when VC internally does round robin scheduling between PRCM's VFSM,
	 * VC channels (1-n) followed by vcbypass. Only if there is pending
	 * transaction will the bypass valid bit get delayed, 99.99% of the
	 * cases at this point in time(configuration and setup time), we do
	 * not expect this conflict to take place.
	 */
	while (vc_bypass_value & VC_BYPASS_VALID_MASK) {
		loop_cnt++;

		/* Get at least 5 times pinfo->i2c_timeout_us for completion */
		if (retries_cnt > pinfo->i2c_timeout_us) {
			dev_err(dev, "%s: Retry count exceeded\n", __func__);
			ret = -ETIMEDOUT;
			goto out;
		}

		if (loop_cnt > 50) {
			retries_cnt++;
			loop_cnt = 0;
			usleep_range(5, 15);
		}
		ret =
		    regmap_read(regmap, vc->regs->bypass_cmd_reg,
				&vc_bypass_value);
		if (ret)
			goto reg_fail;
	}

reg_fail:
	if (ret)
		dev_err(dev, "%s: register operation failed(%d)\n",
			__func__, ret);

out:
	return ret;
}

/**
 * omap_vc_channel_setup_pmic() - Setup PMIC configuration commands if any
 * @vc_channel:	vc channel
 * @vc:		vc
 *
 * Uses vc_bypass to send commands to the PMIC for mandatory configurations
 * for the device to function.
 */
static int omap_vc_channel_setup_pmic(struct omap_vc_channel *vc_channel,
				      struct omap_vc *vc)
{
	struct device *dev = vc_channel->dev;
	struct omap_pmic *pmic = vc_channel->info.pmic;
	const struct omap_pmic_info *pinfo = pmic->info;
	const struct omap_pmic_setup_commands *cmd;
	int i, ret = 0;

	if (!pinfo->setup_num_commands)
		return 0;
	if (!pinfo->setup_command_list) {
		dev_err(dev, "Bad setup command list\n");
		return -EINVAL;
	}
	cmd = pinfo->setup_command_list;

	for (i = 0; i < pinfo->setup_num_commands; i++, cmd++) {
		ret = omap_vc_send_msg(vc, vc_channel, cmd->reg, cmd->cmd_val);
		if (ret) {
			dev_err(dev, "Failed cmd [r=0x%02x v=0x%02x] [%d]\n",
				cmd->reg, cmd->cmd_val, ret);
			return ret;
		}
	}

	return ret;
}

/* Quick helper to avoid having to define /bits/ 8 <0x20> for 8 bit params */
static inline int vc_property_read_u8(const struct device_node *np,
				      const char *propname, u8 *out_value)
{
	u32 val;
	int r;

	r = of_property_read_u32(np, propname, &val);
	if (r)
		return r;

	if (val > 0xFF)
		return -ERANGE;

	*out_value = (u8) val;
	return 0;
}

/**
 * omap_vc_channel_setup_sa() - setup Slave address
 * @vc_channel:	vc channel
 * @vc:		vc
 *
 * setups the slave address configuration as needed.
 */
static int omap_vc_channel_setup_sa(struct omap_vc_channel *vc_channel,
				    struct omap_vc *vc)
{
	struct device *dev = vc_channel->dev;
	const struct vc_channel_regs *ch_regs = vc_channel->ch_regs;
	struct regmap *regmap = vc->regmap;
	struct omap_pmic *pmic = vc_channel->info.pmic;
	const struct omap_pmic_info *pinfo = pmic->info;
	int ret;
	u32 enable_mask = 0;

	if (!vc_channel->use_master_channel_sa) {
		u32 val;

		val = pinfo->slave_addr << __ffs(ch_regs->sa_addr_mask);
		ret =
		    regmap_update_bits(regmap, ch_regs->sa_reg_offset,
				       ch_regs->sa_addr_mask, val);
		if (ret)
			goto out;

		/* SA bit is never set for master */
		if (!vc_channel->is_master_channel)
			enable_mask = ch_regs->sa_enable_mask;
	}

	ret = regmap_update_bits(regmap, ch_regs->cfg_reg_offset,
				 ch_regs->sa_enable_mask, enable_mask);

out:
	if (ret)
		dev_err(dev, "%s: update reg failed(%d)\n", __func__, ret);

	return ret;
}

/**
 * omap_vc_channel_setup_voltage() - configure channel's voltage register addr
 * @vc_channel:	vc channel
 * @vc:		voltage controller
 */
static int omap_vc_channel_setup_voltage(struct omap_vc_channel *vc_channel,
					 struct omap_vc *vc)
{
	struct device *dev = vc_channel->dev;
	struct regmap *regmap = vc->regmap;
	const struct vc_channel_regs *ch_regs = vc_channel->ch_regs;
	struct omap_pmic *pmic = vc_channel->info.pmic;
	const struct omap_pmic_info *pinfo = pmic->info;
	int ret;
	u32 enable_mask = 0;

	if (!vc_channel->use_master_channel_voltage_reg) {
		u32 val;

		val = pinfo->voltage_reg_addr <<
			__ffs(ch_regs->voltage_addr_mask);
		ret = regmap_update_bits(regmap, ch_regs->voltage_reg_offset,
				       ch_regs->voltage_addr_mask, val);
		if (ret)
			goto out;

		/* RAV bit is never set for master */
		if (!vc_channel->is_master_channel)
			enable_mask = ch_regs->rav_enable_mask;
	}
	ret = regmap_update_bits(regmap, ch_regs->cfg_reg_offset,
				 ch_regs->rav_enable_mask, enable_mask);
out:
	if (ret)
		dev_err(dev, "%s: update reg failed(%d)\n", __func__, ret);

	return ret;
}

/**
 * omap_vc_channel_setup_command() - configure channel's command register addr
 * @vc_channel:	vc channel
 * @vc:		voltage controller
 */
static int omap_vc_channel_setup_command(struct omap_vc_channel *vc_channel,
					 struct omap_vc *vc)
{
	struct device *dev = vc_channel->dev;
	struct regmap *regmap = vc->regmap;
	const struct vc_channel_regs *ch_regs = vc_channel->ch_regs;
	struct omap_pmic *pmic = vc_channel->info.pmic;
	const struct omap_pmic_info *pinfo = pmic->info;
	int ret;
	u32 cmd_sel_mask = ch_regs->cmd_sel_mask;
	u32 rac_enable_mask = 0;
	u32 racen_mask = 0;

	if (!vc_channel->use_master_channel_cmd_reg) {
		u32 val;

		val = pinfo->cmd_reg_addr << __ffs(ch_regs->command_addr_mask);
		ret =
		    regmap_update_bits(regmap, ch_regs->command_addr_reg_offset,
				       ch_regs->command_addr_mask, val);
		if (ret)
			goto out;

		/* RAC bit is never set for master */
		if (!vc_channel->is_master_channel)
			rac_enable_mask = ch_regs->rac_enable_mask;
	}

	ret = regmap_update_bits(regmap, ch_regs->cfg_reg_offset,
				 ch_regs->rac_enable_mask, rac_enable_mask);
	if (ret)
		goto out;

	/* Retaining legacy logic - if needed for special case, switch to DT? */
	if (pinfo->cmd_reg_addr == pinfo->voltage_reg_addr)
		racen_mask = ch_regs->racen_mask;

	ret = regmap_update_bits(regmap, ch_regs->cfg_reg_offset,
				 ch_regs->racen_mask, racen_mask);
	if (ret)
		goto out;

	/* CMD bit is never set for master */
	if (vc_channel->is_master_channel ||
	    vc_channel->use_master_channel_cmd_val)
		cmd_sel_mask = 0;

	ret = regmap_update_bits(regmap, ch_regs->cfg_reg_offset,
				 ch_regs->cmd_sel_mask, cmd_sel_mask);
out:
	if (ret)
		dev_err(dev, "%s: update reg failed(%d)\n", __func__, ret);

	return ret;
}

/* Quick helper to find the OF node for a device matching to channel */
static struct device_node *of_get_omap_vc_channel(struct device *dev)
{
	struct device_node *vc_channel_node = NULL;
	char *prop_name = "ti,vc-channel";

	dev_dbg(dev, "Looking up %s from device tree\n", prop_name);
	vc_channel_node = of_parse_phandle(dev->of_node, prop_name, 0);

	if (!vc_channel_node) {
		dev_err(dev, "Looking up %s property in node %s failed",
			prop_name, dev->of_node->full_name);
		return ERR_PTR(-ENODEV);
	}
	return vc_channel_node;
}

/* Helper to cleanup when the managed device is released */
static void devm_omap_vc_channel_release(struct device *dev, void *res)
{
	struct omap_vc_channel *vc_channel = *((struct omap_vc_channel **)res);

	mutex_lock(&omap_vc_channel_list_mutex);

	vc_channel->usage_count--;
	if (!vc_channel->usage_count)
		vc_channel->info.pmic = NULL;
	module_put(vc_channel->dev->driver->owner);

	mutex_unlock(&omap_vc_channel_list_mutex);

	return;
}

/**
 * devm_omap_vc_channel_get() - managed request to get a VC channel
 * @dev:	Generic device to handle the request for
 * @pmic:	PMIC resource this will be assigned to
 *
 * Ensures that usage count is maintained. Uses managed device,
 * so everything is undone on driver detach.
 *
 * Return: -EPROBE_DEFER if the node is present, however device is
 * not yet probed.
 * -EINVAL if bad pointers or node description is not found.
 * -ENODEV if the property cannot be found
 * -ENOMEM if allocation could not be done.
 *  device pointer to vp dev if all successful. error handling should be
 *  performed with IS_ERR
 */
struct omap_vc_channel_info *devm_omap_vc_channel_get(struct device *dev,
						      struct omap_pmic *pmic)
{
	struct omap_vc_channel *vc_channel, **ptr;
	struct device_node *node;
	struct device *vc_dev;
	struct omap_vc_channel_info *info;
	struct omap_vc *vc;
	int ret = 0;

	if (!dev || !dev->of_node || !pmic) {
		pr_err("Invalid parameters\n");
		return ERR_PTR(-EINVAL);
	}

	node = of_get_omap_vc_channel(dev);
	if (IS_ERR(node))
		return (void *)node;

	mutex_lock(&omap_vc_channel_list_mutex);
	list_for_each_entry(vc_channel, &omap_vc_channel_list, list)
	    if (vc_channel->dev->of_node == node)
		goto found;

	/* Node definition is present, but not probed yet.. request defer */
	info = ERR_PTR(-EPROBE_DEFER);
	goto out_unlock;

found:
	vc_dev = vc_channel->dev;
	if (!try_module_get(vc_dev->driver->owner)) {
		dev_err(dev, "%s: Cant get device owner\n", __func__);
		info = ERR_PTR(-EINVAL);
		goto out_unlock;
	}

	/* Allow ONLY 1 user at a time */
	if (vc_channel->usage_count) {
		dev_err(dev, "%s: device %s is busy..\n", __func__,
			dev_name(vc_dev));
		ret = -EBUSY;
		goto out;
	}
	ptr = devres_alloc(devm_omap_vc_channel_release, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr) {
		ret = -ENOMEM;
		goto out;
	}
	info = &vc_channel->info;

	vc = dev_get_drvdata(vc_dev->parent);
	if (!vc) {
		dev_err(dev, "Unable to find parent VC data\n");
		ret = -EINVAL;
		goto out;
	}
	info->pmic = pmic;
	ret = omap_vc_channel_setup_sa(vc_channel, vc);
	if (ret)
		goto out;

	ret = omap_vc_channel_setup_voltage(vc_channel, vc);
	if (ret)
		goto out;

	ret = omap_vc_channel_setup_command(vc_channel, vc);
	if (ret)
		goto out;

	ret = omap_vc_channel_setup_pmic(vc_channel, vc);
	if (ret)
		goto out;

	ret = omap_vc_channel_setup_lp(vc_channel, vc);
	if (ret)
		goto out;

	if (vc_channel->is_master_channel)
		vc->master_channel_configured = true;

	*ptr = vc_channel;
	vc_channel->usage_count++;
	devres_add(dev, ptr);

out:
	if (ret) {
		module_put(vc_dev->driver->owner);
		info = ERR_PTR(ret);
		dev_err(dev, "Failed setup vc with (%d)\n", ret);
	}

out_unlock:
	mutex_unlock(&omap_vc_channel_list_mutex);

	return info;
}
EXPORT_SYMBOL_GPL(devm_omap_vc_channel_get);

static const struct of_device_id omap_vc_channel_of_match_tbl[] = {
	{.compatible = "ti,omap3-vc-channel-0", .data = &omap3_ch_0_regs},
	{.compatible = "ti,omap3-vc-channel-1", .data = &omap3_ch_1_regs},
	{.compatible = "ti,omap4-vc-channel-mpu", .data = &omap4_ch_mpu_regs},
	{.compatible = "ti,omap4-vc-channel-iva", .data = &omap4_ch_iva_regs},
	{.compatible = "ti,omap4-vc-channel-core", .data = &omap4_ch_core_regs},
	{.compatible = "ti,omap5-vc-channel-mpu", .data = &omap5_ch_mpu_regs},
	{.compatible = "ti,omap5-vc-channel-mm", .data = &omap5_ch_mm_regs},
	{.compatible = "ti,omap5-vc-channel-core", .data = &omap5_ch_core_regs},
	{},
};
MODULE_DEVICE_TABLE(of, omap_vc_channel_of_match_tbl);

static int omap_vc_channel_probe(struct platform_device *pdev)
{
	struct device *vc_dev, *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct omap_vc *vc;
	struct omap_vc_channel *vc_channel;
	struct omap_vc_channel_info *info;
	const struct of_device_id *match;

	if (!node) {
		dev_err(dev, "%s: no OF information?\n", __func__);
		return -EINVAL;
	}

	match = of_match_device(omap_vc_channel_of_match_tbl, dev);
	if (!match) {
		/* We do not expect this to happen */
		dev_err(dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}
	if (!match->data) {
		dev_err(dev, "%s: Bad data in match\n", __func__);
		return -EINVAL;
	}

	if (!dev->parent) {
		dev_err(dev, "%s: Unable to find parent VC device\n", __func__);
		return -EINVAL;
	}
	vc_dev = dev->parent;

	vc = dev_get_drvdata(vc_dev);
	if (!vc) {
		dev_err(dev, "%s: Unable to find parent VC data\n", __func__);
		return -EINVAL;
	}

	vc_channel = devm_kzalloc(dev, sizeof(*vc_channel), GFP_KERNEL);
	if (!vc_channel) {
		dev_err(dev, "%s: Unable to allocate vc channel\n", __func__);
		return -ENOMEM;
	}
	vc_channel->dev = dev;
	vc_channel->ch_regs = match->data;
	info = &vc_channel->info;
	info->ch = vc_channel;

	/* Pick up optional parameters */
	of_property_read_u32(node, "ti,retention-micro-volts",
			     &info->retention_uV);
	of_property_read_u32(node, "ti,off-micro-volts", &info->off_uV);

	vc_channel->is_master_channel =
	    of_property_read_bool(node, "ti,master-channel");
	if (vc_channel->is_master_channel && vc->master_channel_configured) {
		dev_err(dev, "There can only be a single master channel!\n");
		return -EINVAL;
	}
	vc_channel->use_master_channel_sa =
	    of_property_read_bool(node, "ti,use-master-slave-addr");
	vc_channel->use_master_channel_voltage_reg =
	    of_property_read_bool(node, "ti,use-master-voltage-reg-addr");
	vc_channel->use_master_channel_cmd_reg =
	    of_property_read_bool(node, "ti,use-master-command-reg-addr");
	vc_channel->use_master_channel_cmd_val =
	    of_property_read_bool(node, "ti,use-master-command-reg-val");

	if (vc_channel->is_master_channel &&
	    (vc_channel->use_master_channel_sa ||
	     vc_channel->use_master_channel_voltage_reg ||
	     vc_channel->use_master_channel_cmd_reg ||
	     vc_channel->use_master_channel_cmd_val)) {
		dev_err(dev, "Master channel cannot route to slave channel!\n");
		return -EINVAL;
	}
	if (!vc->master_channel_configured &&
	    (vc_channel->use_master_channel_sa ||
	     vc_channel->use_master_channel_voltage_reg ||
	     vc_channel->use_master_channel_cmd_reg ||
	     vc_channel->use_master_channel_cmd_val)) {
		dev_dbg(dev, "Deferring - Master channel not yet ready!\n");
		return -EPROBE_DEFER;
	}

	platform_set_drvdata(pdev, vc_channel);

	/* Add it to VC channel list */
	mutex_lock(&omap_vc_channel_list_mutex);
	list_add(&vc_channel->list, &omap_vc_channel_list);
	mutex_unlock(&omap_vc_channel_list_mutex);

	return 0;
}

/**
 * omap_vc_channel_remove() - Cleanup operations for channel
 * @pdev:	platform device
 *
 * Return: -EBUSY if all users have not transitioned out, else return 0
 */
static int omap_vc_channel_remove(struct platform_device *pdev)
{
	struct omap_vc_channel *vc_channel = platform_get_drvdata(pdev);
	int ret = 0;

	mutex_lock(&omap_vc_channel_list_mutex);
	list_del(&vc_channel->list);
	mutex_unlock(&omap_vc_channel_list_mutex);

	return ret;
}

static struct platform_driver omap_vc_channel_driver = {
	.driver = {
		   .name = DRIVER_NAME "-channel",
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(omap_vc_channel_of_match_tbl),
		   },
	.probe = omap_vc_channel_probe,
	.remove = omap_vc_channel_remove,
};

/* Regular 32 bit registers for Voltage controller */
static struct regmap_config omap_vc_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

/**
 * omap_vc_i2c_config() - configure the I2C clock and Mode registers
 * @vc:		voltage controller
 * @i2c_config:	i2c configuration to configure
 */
static int omap_vc_i2c_config(struct omap_vc *vc,
			 struct omap_omap_vc_i2c_config *i2c_config)
{
	u32 mask;

	mask = VC_MODE_HSENABLE_MASK;
	regmap_update_bits(vc->regmap, vc->regs->i2c_clk_mode_reg, mask,
			   i2c_config->highspeed << __ffs(mask));
	mask = VC_MODE_MCODE_MASK;
	regmap_update_bits(vc->regmap, vc->regs->i2c_clk_mode_reg, mask,
			   i2c_config->mcode << __ffs(mask));
	/* Disable repeated start */
	mask = VC_MODE_REPEAT_START_MASK;
	regmap_update_bits(vc->regmap, vc->regs->i2c_clk_mode_reg, mask, 0);

	/* if there is clock config register on the SoC, skip.. */
	if (vc->regs->i2c_clk_cfg_reg < 0)
		return 0;

	if (!i2c_config->i2c_clk_hsscll &&
	    !i2c_config->i2c_clk_hssclh &&
	    !i2c_config->i2c_clk_scll && !i2c_config->i2c_clk_sclh)
		return -EINVAL;

	/* Configure up clk timings */
	mask = VC_CLK_CFG_HSSCLL_MASK;
	regmap_update_bits(vc->regmap, vc->regs->i2c_clk_cfg_reg, mask,
			   i2c_config->i2c_clk_hsscll << __ffs(mask));
	mask = VC_CLK_CFG_HSSCLH_MASK;
	regmap_update_bits(vc->regmap, vc->regs->i2c_clk_cfg_reg, mask,
			   i2c_config->i2c_clk_hssclh << __ffs(mask));
	mask = VC_CLK_CFG_SCLL_MASK;
	regmap_update_bits(vc->regmap, vc->regs->i2c_clk_cfg_reg, mask,
			   i2c_config->i2c_clk_scll << __ffs(mask));
	mask = VC_CLK_CFG_SCLH_MASK;
	regmap_update_bits(vc->regmap, vc->regs->i2c_clk_cfg_reg, mask,
			   i2c_config->i2c_clk_sclh << __ffs(mask));
	return 0;
}

/**
 * struct i2c_load_data - table mapping load to clock configs
 * @load:	load of the I2C bus
 * @hsscll_38_4:	38.4MHz Sysclk HSSCLL configuration
 * @hsscll_26:		26MHz Sysclk HSSCLL configuration
 * @hsscll_19_2:	19.2MHz Sysclk HSSCLL configuration
 * @hsscll_16_8:	16.8MHz Sysclk HSSCLL configuration
 * @hsscll_12:		12MHz Sysclk HSSCLL configuration
 *
 * Instead of doing an hard multi-parameter computation, if load is provided by
 * OF data then we pick the values from the table, There is always an option
 * of defining the clocks from OF data itself.
 */
struct i2c_load_data {
	u8 load;
	u8 hsscll_38_4;
	u8 hsscll_26;
	u8 hsscll_19_2;
	u8 hsscll_16_8;
	u8 hsscll_12;
};

/**
 * omap_vc_i2c_timing_init() - sets up board I2C timing parameters
 * @dev:	vc channel device
 * @cfg:	I2C configuration
 *
 * Use PMIC + board supplied settings for calculating the total I2C channel
 * capacitance and set the timing parameters based on this. Pre-calculated
 * values are provided in data tables, as it is not too straightforward to
 * calculate these runtime.
 */
static int omap_vc_i2c_timing_init(struct device *dev,
				   struct omap_omap_vc_i2c_config *cfg)
{
	u32 capacitance;
	u16 hsscll;
	const struct i2c_load_data i2c_timing_data[] = {
		{
		 .load = 50,
		 .hsscll_38_4 = 13,
		 .hsscll_26 = 11,
		 .hsscll_19_2 = 9,
		 .hsscll_16_8 = 9,
		 .hsscll_12 = 8,
		 },
		{
		 .load = 25,
		 .hsscll_38_4 = 13,
		 .hsscll_26 = 11,
		 .hsscll_19_2 = 9,
		 .hsscll_16_8 = 9,
		 .hsscll_12 = 8,
		 },
		{
		 .load = 12,
		 .hsscll_38_4 = 11,
		 .hsscll_26 = 10,
		 .hsscll_19_2 = 9,
		 .hsscll_16_8 = 9,
		 .hsscll_12 = 8,
		 },
		{
		 .load = 0,
		 .hsscll_38_4 = 12,
		 .hsscll_26 = 10,
		 .hsscll_19_2 = 9,
		 .hsscll_16_8 = 8,
		 .hsscll_12 = 8,
		 },
	};
	const struct i2c_load_data *i2c_data;

	if (!cfg->i2c_clk_pad_load && !cfg->i2c_clk_pcb_length)
		return 0;

	/* PCB trace capacitance, 0.125pF / mm => mm / 8 */
	capacitance = DIV_ROUND_UP(cfg->i2c_clk_pcb_length, 8);

	/* OMAP pad capacitance */
	capacitance += 4;

	/* PMIC pad capacitance */
	capacitance += cfg->i2c_clk_pad_load;

	/* Search for capacitance match in the table */
	i2c_data = i2c_timing_data;

	while (i2c_data->load > capacitance)
		i2c_data++;

	/* Select proper values based on sysclk frequency */
	switch (cfg->clk_rate) {
	case 38400000:
		hsscll = i2c_data->hsscll_38_4;
		break;
	case 26000000:
		hsscll = i2c_data->hsscll_26;
		break;
	case 19200000:
		hsscll = i2c_data->hsscll_19_2;
		break;
	case 16800000:
		hsscll = i2c_data->hsscll_16_8;
		break;
	case 12000000:
		hsscll = i2c_data->hsscll_12;
		break;
	default:
		dev_err(dev, "%s: Unsupported sysclk rate: %ld!\n", __func__,
			cfg->clk_rate);
		return -ERANGE;
	}

	/* HSSCLH can always be zero */
	cfg->i2c_clk_hssclh = 0x0;
	cfg->i2c_clk_hsscll = hsscll;
	/* FS timing - standard */
	cfg->i2c_clk_scll = 0x28;
	cfg->i2c_clk_sclh = 0x2C;

	return 0;
}

static const struct of_device_id omap_vc_of_match_tbl[] = {
	{.compatible = "ti,omap3-vc", .data = &omap3_vc_regs,},
	{.compatible = "ti,omap4-vc", .data = &omap4_vc_regs,},
	{.compatible = "ti,omap5-vc", .data = &omap5_vc_regs,},
	{},
};
MODULE_DEVICE_TABLE(of, omap_vc_of_match_tbl);

static int omap_vc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct omap_vc *vc;
	struct omap_omap_vc_i2c_config i2c_config = { 0 };
	int ret;
	struct resource *res;
	char *pname;
	struct clk *clk;
	void __iomem *base;
	struct regmap *regmap;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;

	if (!node) {
		dev_err(dev, "no OF information?\n");
		return -EINVAL;
	}

	match = of_match_device(omap_vc_of_match_tbl, dev);
	if (!match) {
		/* We do not expect this to happen */
		dev_err(dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}
	if (!match->data) {
		dev_err(dev, "%s: Bad data in match\n", __func__);
		return -EINVAL;
	}

	/* Read mandatory parameters */
	clk = clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "%s: Unable to get clk(%d)\n", __func__, ret);
		return ret;
	}
	i2c_config.clk_rate = clk_get_rate(clk);
	/* We do not need the clock any more */
	clk_put(clk);

	pname = "base-address";
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, pname);
	base = devm_request_and_ioremap(dev, res);
	if (!base) {
		dev_err(dev, "Unable to map '%s'\n", pname);
		return -EADDRNOTAVAIL;
	}

	regmap = devm_regmap_init_mmio(dev, base, &omap_vc_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "regmap init failed(%d)\n", ret);
		return ret;
	}

	/* Read optional highspeed mode param */
	pname = "ti,i2c-high-speed";
	i2c_config.highspeed = of_property_read_bool(node, pname);

	/* Mcode if the platform likes to set it explicitly */
	if (i2c_config.highspeed)
		vc_property_read_u8(node, "ti,i2c-high-speed-mcode",
				    &i2c_config.mcode);

	if (i2c_config.highspeed) {
		pname = "ti,i2c-pad-load";
		of_property_read_u32(node, pname, &i2c_config.i2c_clk_pad_load);
		pname = "ti,i2c-pcb-length";
		of_property_read_u32(node, pname,
				     &i2c_config.i2c_clk_pcb_length);

		if (i2c_config.i2c_clk_pad_load) {
			if (!i2c_config.i2c_clk_pcb_length)
				i2c_config.i2c_clk_pcb_length = 63;

			ret = omap_vc_i2c_timing_init(dev, &i2c_config);
			if (ret)
				goto out;

			goto skip_i2c_clk_config;
		}
	}

	/* So, we do not have pad load defined, expect proper timing config */
	pname = "ti,i2c-clk-scl-low";
	ret = vc_property_read_u8(node, pname, &i2c_config.i2c_clk_scll);
	if (ret)
		goto invalid_of_property;

	pname = "ti,i2c-clk-scl-high";
	ret = vc_property_read_u8(node, pname, &i2c_config.i2c_clk_sclh);
	if (ret)
		goto invalid_of_property;

	pname = "ti,i2c-clk-hsscl-low";
	ret = vc_property_read_u8(node, pname, &i2c_config.i2c_clk_hsscll);
	if (ret && i2c_config.highspeed)
		goto invalid_of_property;

	pname = "ti,i2c-clk-hsscl-high";
	ret = vc_property_read_u8(node, pname, &i2c_config.i2c_clk_hssclh);
	if (ret && i2c_config.highspeed)
		goto invalid_of_property;

skip_i2c_clk_config:
	vc = devm_kzalloc(dev, sizeof(*vc), GFP_KERNEL);
	if (!vc) {
		dev_err(dev, "unable to allocate vc\n");
		ret = -ENOMEM;
		goto out;
	}
	vc->regmap = regmap;
	vc->regs = match->data;

	ret = omap_vc_i2c_config(vc, &i2c_config);
	if (ret) {
		dev_err(dev, "Bad I2C configuration: %d\n", ret);
		goto out;
	}

	platform_set_drvdata(pdev, vc);
	ret = of_platform_populate(dev->of_node,
				   omap_vc_channel_of_match_tbl, NULL, dev);

	if (ret)
		dev_err(dev, "Failed to create DT children: %d\n", ret);

out:
	return ret;

invalid_of_property:
	dev_err(dev, "Missing/Invalid '%s' property - error(%d)\n", pname, ret);
	return ret;
}

static struct platform_driver omap_vc_driver = {
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(omap_vc_of_match_tbl),
		   },
	.probe = omap_vc_probe,
};

static int __init omap_vc_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&omap_vc_driver);
	if (ret) {
		pr_err("platform driver register failed for VC(%d)\n", ret);
		return ret;
	}

	ret = platform_driver_register(&omap_vc_channel_driver);
	if (ret) {
		pr_err("platform driver register failed for VC Channel(%d)\n",
		       ret);
		return ret;
	}

	return 0;
}
module_init(omap_vc_init);

static void __exit omap_vc_exit(void)
{
	platform_driver_unregister(&omap_vc_channel_driver);
	platform_driver_unregister(&omap_vc_driver);
}
module_exit(omap_vc_exit);

MODULE_DESCRIPTION("OMAP Voltage Controller Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc.");
