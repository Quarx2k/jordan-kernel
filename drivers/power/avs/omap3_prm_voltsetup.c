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
#include <linux/regulator/omap-pmic-regulator.h>
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
#define VOLTCTRL_SEL_OFF_MASK			0x08
#define CLKSRC_CTRL_AUTOEXTCLKMODE_MASK		0x18
#define CLKSETUP_MASK				0xffff
#define VOLTSETUP2_MASK				0xffff
#define VOLTOFFSET_MASK				0xffff
#define VOLTSETUP1_TIME1_MASK			0x0000ffff
#define VOLTSETUP1_TIME2_MASK			0xffff0000


struct omap_prm_voltsetup_data {
	struct regmap *regmap;
	const struct omap_vc_common_reg *regs;
	bool sys_off_mode;
	u32 autoextclkmode;
	u32 clksetup_time; /* uSec */
	u32 sys_clk_rate; /* Hz */
};

/* Regular 32 bit registers */
static struct regmap_config regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};
static struct omap_prm_voltsetup_data *prm_voltsetup_data;

static const struct of_device_id omap3_prm_voltsetup_match_tbl[] __initdata = {
	{.compatible = "ti,omap3-glbl-prm"},
	{},
};
MODULE_DEVICE_TABLE(of, omap3_prm_voltsetup_match_tbl);

int omap_prm_voltsetup(struct device *dev, struct omap_pmic *pmic, u32 uv)
{
	struct regmap *regmap;
	const char *str;
	int ret = 0, val;
	u32 clsetup, v1, v2, v2_old, msk;

	if (!dev || !dev->of_node || !pmic || uv == 0) {
		pr_err("Invalid parameters\n");
		return -EINVAL;
	}
	if (IS_ERR_OR_NULL(prm_voltsetup_data)) {
		pr_err("Device is not initialized\n");
		return -ENODEV;
	}
	regmap = prm_voltsetup_data->regmap;

	if (prm_voltsetup_data->sys_off_mode) {
		ret = regmap_read(regmap, PRM_CLKSETUP_OFFS, &clsetup);
		if (ret)
			goto fail_reg;
		clsetup &= CLKSETUP_MASK;
		clsetup >>= __ffs(CLKSETUP_MASK);

		ret = regmap_read(regmap, PRM_VOLTSETUP2_OFFS, &v2_old);
		if (ret)
			goto fail_reg;
		v2_old &= VOLTSETUP2_MASK;
		v2_old >>= __ffs(VOLTSETUP2_MASK);

		/* voltage / slew_rate, 2uS added as buffer */
		val = DIV_ROUND_UP(uv, pmic->info->slew_rate_uV) + 2;
		/* convert to 32k clk cycles */
		v2 = DIV_ROUND_UP(val * 32768, 1000000);
		/*
		 * Update v2 if higher than current value (needed because
		 * we have multiple channels with different ramp times), also
		 * update voltoffset always to value recommended by TRM
		 */
		if (v2 > v2_old) {
			pr_info("voltsetup2 is set to %d uS\n", val);
			ret = regmap_update_bits(regmap, PRM_VOLTSETUP2_OFFS,
					VOLTSETUP2_MASK,
					v2 << __ffs(VOLTSETUP2_MASK));
			if (ret)
				goto fail_reg;
			val = clsetup - v2;
		} else
			val = clsetup - v2_old;

		if (val < 0) {
			/*
			* in case of clock setup time is less then voltsetup
			* we should correct it to allow start clock usage after
			* voltage rumped up
			*/
			val = 1;
			clsetup = v2 > v2_old ? v2 : v2_old;
			clsetup += val;
			ret = regmap_update_bits(regmap, PRM_CLKSETUP_OFFS,
				CLKSETUP_MASK, clsetup << __ffs(CLKSETUP_MASK));
			if (ret)
				goto fail_reg;
			pr_info("Clock setup time is ajusted to %d uS\n",
				DIV_ROUND_CLOSEST(clsetup * 1000000, 32768));
		}

		ret = regmap_update_bits(regmap, PRM_VOLTOFFSET_OFFS,
				VOLTOFFSET_MASK, val << __ffs(VOLTOFFSET_MASK));
		if (ret)
			goto fail_reg;
	} else {
		/* voltage / slew_rate */
		v1 = DIV_ROUND_UP(uv, pmic->info->slew_rate_uV);
		v1 = DIV_ROUND_UP_ULL(
			(u64)v1 * prm_voltsetup_data->sys_clk_rate, 8000000);

		ret = of_property_read_string(dev->of_node, "compatible", &str);
		if (ret) {
			pr_err("No compatible property in channel (%d)\n", ret);
			return ret;
		}
		val = strlen(str);
		if (str[val - 1] == '0')
			msk = VOLTSETUP1_TIME1_MASK;
		else if (str[val - 1] == '1')
			msk = VOLTSETUP1_TIME2_MASK;
		else {
			pr_err("Unable to identify vc channel\n");
			return -EINVAL;
		}
		ret = regmap_update_bits(regmap, PRM_VOLTSETUP1_OFFS,
				msk, v1 << __ffs(msk));
		if (ret)
			goto fail_reg;
	}
	return ret;

fail_reg:
	dev_err(dev, "%s: Register operation failed with %d\n", __func__, ret);
	return ret;
}
EXPORT_SYMBOL_GPL(omap_prm_voltsetup);


static int omap3_prm_voltsetup_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *nd = dev->of_node;
	int ret;
	struct resource *res;
	char *pname;
	const char *clk_name = NULL;
	void __iomem *base;
	struct regmap *regmap;
	struct omap_prm_voltsetup_data *data;
	u32 clkmode, val = 0, msk = 0;
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
	regmap = devm_regmap_init_mmio(dev, base, &regmap_cfg);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "regmap init failed(%d)\n", ret);
		return ret;
	}
	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data) {
		dev_err(dev, "%s: Unable to allocate data\n", __func__);
		return -ENOMEM;
	}
	data->regmap = regmap;

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

	pname = "autoextclkmode";
	of_property_read_u32(nd, pname, &clkmode);

	pname = "clksetup_time";
	ret = of_property_read_u32(nd, pname, &data->clksetup_time);
	if (ret || !data->clksetup_time)
		goto property_err;

	pname = "sys_off_mode";
	if (of_property_read_bool(nd, pname))
		data->sys_off_mode = true;

	if (!data->sys_off_mode) {
		pname = "auto_off";
		val |= of_property_read_bool(nd, pname) ?
			VOLTCTRL_AUTO_OFF_MASK : 0;
	} else
		val |= VOLTCTRL_SEL_OFF_MASK;

	pname = "auto_sleep";
	val |= of_property_read_bool(nd, pname) ? VOLTCTRL_AUTO_SLEEP_MASK : 0;
	pname = "auto_retention";
	val |= of_property_read_bool(nd, pname) ? VOLTCTRL_AUTO_RET_MASK : 0;
	val |= data->sys_off_mode ? VOLTCTRL_SEL_OFF_MASK : 0;

	msk = VOLTCTRL_SEL_OFF_MASK | VOLTCTRL_AUTO_OFF_MASK |
		VOLTCTRL_AUTO_RET_MASK | VOLTCTRL_AUTO_SLEEP_MASK;
	ret = regmap_update_bits(regmap, PRM_VOLTCTRL_OFFS, msk,
				val << __ffs(msk));

	if (ret)
		goto fail_reg;

	ret = regmap_update_bits(regmap, PRM_VOLTCTRL_OFFS,
		VOLTCTRL_SEL_OFF_MASK,
		(data->sys_off_mode ? 1 : 0) << __ffs(VOLTCTRL_SEL_OFF_MASK));
	if (ret)
		goto fail_reg;


	ret = regmap_update_bits(regmap, PRM_CLKSRC_CTRL_OFFS,
		CLKSRC_CTRL_AUTOEXTCLKMODE_MASK,
		clkmode << __ffs(CLKSRC_CTRL_AUTOEXTCLKMODE_MASK));
	if (ret)
		goto fail_reg;

	val = DIV_ROUND_UP(data->clksetup_time * 32768, 1000000);
	ret = regmap_update_bits(regmap, PRM_CLKSETUP_OFFS, CLKSETUP_MASK,
		val << __ffs(CLKSETUP_MASK));
	if (ret)
		goto fail_reg;

	if (data->sys_off_mode) {
		ret = regmap_write(regmap, PRM_VOLTSETUP1_OFFS, 0);
		if (ret)
			goto fail_reg;
	} else {
		ret = regmap_write(regmap, PRM_VOLTSETUP2_OFFS, 0);
		if (ret)
			goto fail_reg;
	}
	prm_voltsetup_data = data;

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

