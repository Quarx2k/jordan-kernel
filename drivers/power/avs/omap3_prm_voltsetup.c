/*
 * Copyright (C) 2014 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/power/omap_prm.h>
#include <linux/regulator/omap-pmic-regulator.h>
#include "omap_vc.h"
#include "omap_prm_voltsetup.h"

#define DRIVER_NAME	"omap3-glbl-prm"

#define PRM_RSTCTRL_OFFS			0x00
#define PRM_RSTTIME_OFFS			0x04
#define PRM_RSTST_OFFS				0x08
#define PRM_VOLTCTRL_OFFS			0x10
#define PRM_SRAM_PCHARGE_OFFS			0x14
#define PRM_CLKSRC_CTRL_OFFS			0x20
#define PRM_OBS					0x30
#define PRM_VOLTSETUP1_OFFS			0x40
#define PRM_VOLTOFFSET_OFFS			0x44
#define PRM_CLKSETUP_OFFS			0x48
#define PRM_POLCTRL_OFFS			0x4C
#define PRM_VOLTSETUP2_OFFS			0x50

#define VOLTCTRL_AUTO_SLEEP_MASK		0x01
#define VOLTCTRL_AUTO_RET_MASK			0x02
#define VOLTCTRL_AUTO_OFF_MASK			0x04
#define VOLTCTRL_AUTO_MASK			0x07
#define VOLTCTRL_SEL_OFF_MASK			0x08
#define CLKSRC_CTRL_AUTOEXTCLKMODE_MASK		0x18
#define CLKSETUP_MASK				0xffff
#define OFFMODESETUPTIME_MASK			0xffff
#define VOLTOFFSET_MASK				0xffff
#define VOLTSETUP1_TIME1_MASK			0x0000ffff
#define VOLTSETUP1_TIME2_MASK			0xffff0000
#define VOLTSETUP1_TIME1_2_MASK			0xffffffff


struct omap_prm_voltsetup_data {
	struct device *dev;
	struct regmap *rmap;
	const struct omap_vc_common_reg *regs;
	bool sys_off_mode;
	bool auto_retention;
	bool auto_off;
	u32 retsetup1_2; /* sys_clk ticks for time1 and time2*/
	u32 offsetup1_2; /* sys_clk ticks for time1 and time2*/
	u32 offmodesetup_cnt; /* 32K ticks */
	u32 offoffset_cnt; /* 32K ticks */
	u32 sys_clk_rate; /* Hz */
};

/* Regular 32 bit registers */
static struct regmap_config regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};
static struct omap_prm_voltsetup_data *prm_voltsetup_data;

static const struct of_device_id omap3_prm_voltsetup_match_tbl[] = {
	{.compatible = "ti,omap3-glbl-prm"},
	{},
};
MODULE_DEVICE_TABLE(of, omap3_prm_voltsetup_match_tbl);

int omap_prm_configure(bool off)
{
	u32 val, msk;
	int ret = 0;
	struct omap_prm_voltsetup_data *pd = prm_voltsetup_data;

	if (!pd || !pd->dev) {
		pr_err("PRM voltsetup device is not initialized\n");
		return -ENODEV;
	}

	/* set auto bit exclusive */
	regmap_update_bits(pd->rmap, PRM_VOLTCTRL_OFFS, VOLTCTRL_AUTO_MASK, 0);
	msk = off ? VOLTCTRL_AUTO_OFF_MASK : VOLTCTRL_AUTO_RET_MASK;
	val = off ? (pd->auto_off ? 1 : 0) : (pd->auto_retention ? 1 : 0);
	regmap_update_bits(pd->rmap, PRM_VOLTCTRL_OFFS, msk, val << __ffs(msk));

	if (off && pd->auto_off) {
		if (pd->sys_off_mode) {
			val = pd->offmodesetup_cnt;
			ret = regmap_update_bits(pd->rmap, PRM_VOLTSETUP2_OFFS,
					OFFMODESETUPTIME_MASK,
					val << __ffs(OFFMODESETUPTIME_MASK));
			if (ret)
				goto fail_reg;
			val = pd->offoffset_cnt;
			ret = regmap_update_bits(pd->rmap, PRM_VOLTOFFSET_OFFS,
				VOLTOFFSET_MASK, val << __ffs(VOLTOFFSET_MASK));
		} else
			ret = regmap_update_bits(pd->rmap, PRM_VOLTSETUP1_OFFS,
				VOLTSETUP1_TIME1_2_MASK, pd->offsetup1_2);
		if (ret)
			goto fail_reg;
	} else if (!off && pd->auto_retention) {
		ret = regmap_update_bits(pd->rmap, PRM_VOLTSETUP1_OFFS,
			VOLTSETUP1_TIME1_2_MASK, pd->retsetup1_2);
		if (ret)
			goto fail_reg;
		ret = regmap_update_bits(pd->rmap, PRM_VOLTSETUP2_OFFS,
				OFFMODESETUPTIME_MASK,
				0 << __ffs(OFFMODESETUPTIME_MASK));
		if (ret)
			goto fail_reg;
		ret = regmap_update_bits(pd->rmap, PRM_VOLTOFFSET_OFFS,
			VOLTOFFSET_MASK, 0 << __ffs(VOLTOFFSET_MASK));
		if (ret)
			goto fail_reg;
	}
	return 0;
fail_reg:
	dev_err(pd->dev, "%s: Register operation failed (%d)\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(omap_prm_configure);

int omap_prm_voltsetup(struct omap_vc_channel_info *inf, struct omap_pmic *pmic,
			u32 uv)
{
	u32 uv_diff, val, msk;
	struct omap_prm_voltsetup_data *pd = prm_voltsetup_data;

	if (!pd || !pd->dev) {
		pr_err("PRM voltsetup device is not initialized\n");
		return -ENODEV;
	}
	if (!inf || !pmic || uv == 0 || pmic->info->slew_rate_uV == 0 ||
	    inf->retention_uV > uv) {
		dev_err(pd->dev, "Invalid parameters\n");
		return -EINVAL;
	}
	if (inf->ch_num != 0 && inf->ch_num != 1) {
		dev_err(pd->dev, "Unable to identify vc channel\n");
		return -EINVAL;
	}
	msk = inf->ch_num == 0 ? VOLTSETUP1_TIME1_MASK : VOLTSETUP1_TIME2_MASK;
	uv_diff = uv - inf->retention_uV;
	val = DIV_ROUND_UP(uv_diff, pmic->info->slew_rate_uV);
	val = DIV_ROUND_UP_ULL((u64)val * pd->sys_clk_rate, 8000000);
	pd->retsetup1_2 &= ~msk;
	pd->retsetup1_2 |= val << __ffs(msk);
	if (!pd->sys_off_mode) {
		uv_diff = uv - inf->off_uV;
		val = DIV_ROUND_UP(uv_diff, pmic->info->slew_rate_uV);
		val = DIV_ROUND_UP_ULL((u64)val * pd->sys_clk_rate, 8000000);
		pd->offsetup1_2 &= ~msk;
		pd->retsetup1_2 |= val << __ffs(msk);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(omap_prm_voltsetup);
/**
 * omap_pm_enable_off_mode - notify OMAP PM that off-mode is enabled
 *
 * Intended for use only by OMAP PM core code to notify this layer
 * that off mode has been enabled.
 */
void omap_pm_enable_off_mode(void)
{
	prm_voltsetup_data->auto_off = true;
}
EXPORT_SYMBOL_GPL(omap_pm_enable_off_mode);

/**
 * omap_pm_disable_off_mode - notify OMAP PM that off-mode is disabled
 *
 * Intended for use only by OMAP PM core code to notify this layer
 * that off mode has been disabled.
 */
void omap_pm_disable_off_mode(void)
{
	prm_voltsetup_data->auto_off = false;
}
EXPORT_SYMBOL_GPL(omap_pm_disable_off_mode);

/**
 * omap_pm_get_off_mode - get OMAP PM off-mode state
 *
 * Intended for use only by OMAP PM core code to query off mode state
 */
bool omap_pm_get_off_mode(void)
{
	return prm_voltsetup_data->auto_off;
}
EXPORT_SYMBOL_GPL(omap_pm_get_off_mode);

static int omap3_prm_voltsetup_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *nd = dev->of_node;
	int ret;
	struct resource *res;
	char *pname;
	const char *clk_name = NULL;
	void __iomem *base;
	struct regmap *rmap;
	struct omap_prm_voltsetup_data *data;
	u32 clksetup_time; /* uSec */
	u32 val = 0;
	struct clk *sys_clk;

	if (!nd) {
		dev_err(dev, "no OF information?\n");
		return -EINVAL;
	}

	pname = "base-address";
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, pname);
	if (!res) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
	base = devm_request_and_ioremap(dev, res);
	if (!base) {
		dev_err(dev, "Unable to map registers\n");
		return -EADDRNOTAVAIL;
	}
	rmap = devm_regmap_init_mmio(dev, base, &regmap_cfg);
	if (IS_ERR(rmap)) {
		ret = PTR_ERR(rmap);
		dev_err(dev, "regmap init failed(%d)\n", ret);
		return ret;
	}
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s: Unable to allocate data\n", __func__);
		return -ENOMEM;
	}
	data->rmap = rmap;

	pname = "auto_off";
	data->auto_off = of_property_read_bool(nd, pname);
	pname = "auto_retention";
	data->auto_retention = of_property_read_bool(nd, pname);

	if (!data->auto_off && !data->auto_retention) {
		dev_warn(dev, "WARNING: No auto_retention nor auto_off\n");
		return -EINVAL;
	}
	/* clean all auto bits */
	regmap_update_bits(rmap, PRM_VOLTCTRL_OFFS, VOLTCTRL_AUTO_MASK, 0);
	/* set to retention if enabled by default, off mode is set on demand */
	val = data->auto_retention ? 1 : 0;
	regmap_update_bits(rmap, PRM_VOLTCTRL_OFFS, VOLTCTRL_AUTO_RET_MASK,
		val << __ffs(VOLTCTRL_AUTO_RET_MASK));

	pname = "sys_clk";
	ret = of_property_read_string(nd, pname, &clk_name);
	if (ret || IS_ERR_OR_NULL(clk_name))
		goto property_err;

	sys_clk = clk_get(NULL, clk_name);
	if (IS_ERR(sys_clk)) {
		dev_err(dev, "Clock %s does not exists\n", clk_name);
		return -EINVAL;
	}
	data->sys_clk_rate = clk_get_rate(sys_clk);
	clk_put(sys_clk);

	pname = "clksetup_time";
	ret = of_property_read_u32(nd, pname, &clksetup_time);
	if (ret || !clksetup_time)
		goto property_err;
	val = DIV_ROUND_UP(clksetup_time * 32768, 1000000);
	ret = regmap_update_bits(rmap, PRM_CLKSETUP_OFFS, CLKSETUP_MASK,
		val << __ffs(CLKSETUP_MASK));
	if (ret)
		goto fail_reg;

	pname = "sys_off_mode";
	data->sys_off_mode = of_property_read_bool(nd, pname);
	val = data->sys_off_mode ? 1 : 0;
	regmap_update_bits(rmap, PRM_VOLTCTRL_OFFS, VOLTCTRL_SEL_OFF_MASK,
		val << __ffs(VOLTCTRL_SEL_OFF_MASK));

	if (data->sys_off_mode) {
		pname = "offmodesetup_time";
		ret = of_property_read_u32(nd, pname, &val);
		if (ret || !val)
			goto property_err;
		data->offmodesetup_cnt = DIV_ROUND_UP(val * 32768, 1000000);
		/* ~1 32K tick by default */
		val = clksetup_time > val ? clksetup_time - val : 30;
		data->offoffset_cnt = DIV_ROUND_UP(val * 32768, 1000000);
	}

	pname = "autoextclkmode";
	of_property_read_u32(nd, pname, &val);
	ret = regmap_update_bits(rmap, PRM_CLKSRC_CTRL_OFFS,
		CLKSRC_CTRL_AUTOEXTCLKMODE_MASK,
		val << __ffs(CLKSRC_CTRL_AUTOEXTCLKMODE_MASK));
	if (ret)
		goto fail_reg;

	data->dev = dev;
	prm_voltsetup_data = data;
	platform_set_drvdata(pdev, prm_voltsetup_data);

	return ret;

property_err:
	if (!ret) {
		dev_err(dev, " Invalid value 0x0 in '%s' property.\n", pname);
		ret = -EINVAL;
	} else {
		dev_err(dev, " Missing/Invalid '%s' property\n", pname);
	}
	return ret;
fail_reg:
	dev_err(dev, "%s: Register operation failed with %d\n", __func__, ret);
	return ret;
}

static struct platform_driver omap3_prm_voltsetup_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(omap3_prm_voltsetup_match_tbl),
		},
	.probe = omap3_prm_voltsetup_probe,
};

static int __init omap3_prm_voltsetup_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&omap3_prm_voltsetup_driver);
	if (ret) {
		pr_err("platform driver register failed for Global PRM(%d)\n",
			ret);
		return ret;
	}

	return 0;
}

static void __exit omap3_prm_voltsetup_exit(void)
{
	platform_driver_unregister(&omap3_prm_voltsetup_driver);
	prm_voltsetup_data = NULL;
}

module_init(omap3_prm_voltsetup_init);
module_exit(omap3_prm_voltsetup_exit);


MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("OMAP Global PRM driver");
MODULE_LICENSE("GPL v2");

