/*
 * OMAP Voltage Processor (VP) interface
 *
 * Idea based on arch/arm/mach-omap2/vp.c
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
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/omap-pmic-regulator.h>
#include <linux/regmap.h>
#include "omap_vc.h"

#define DRIVER_NAME	"omap-vp"

/**
 * omap_vp_test_timeout - busy-loop, testing a condition
 * @cond: condition to test until it evaluates to true
 * @timeout: maximum number of microseconds in the timeout
 * @index: loop index (integer)
 *
 * Loop waiting for @cond to become true or until at least @timeout
 * microseconds have passed.  To use, define some integer @index in the
 * calling code.  After running, if @index == @timeout, then the loop has
 * timed out.
 */
#define omap_vp_test_timeout(cond, timeout, index)		\
({								\
	for (index = 0; index < timeout; index++) {		\
		if (cond)					\
			break;					\
		udelay(1);					\
	}							\
})

/**
 * struct omap_vp_reg_data - Voltage processor register offsets
 * @config:	CONFIG register
 * @status:	STATUS register
 * @vlimitto:	VLIMITTO register
 * @voltage:	VOLTAGE register
 * @step_max:	STEP_MAX register
 * @step_min:	STEP_MAX register
 */
struct omap_vp_reg_data {
	u8 config;
	u8 status;
	u8 vlimitto;
	u8 voltage;
	u8 step_max;
	u8 step_min;
};

static const struct omap_vp_reg_data omap_vp_reg_type1 = {
	.config = 0x00,
	.status = 0x14,
	.vlimitto = 0x0C,
	.voltage = 0x10,
	.step_max = 0x08,
	.step_min = 0x04,
};

static const struct omap_vp_reg_data omap_vp_reg_type2 = {
	.config = 0x00,
	.status = 0x04,
	.vlimitto = 0x08,
	.voltage = 0x0C,
	.step_max = 0x10,
	.step_min = 0x14,
};

/* Config register masks - All revisions */
#define CONFIG_ERROR_OFFSET_MASK	(0xff << 24)
#define CONFIG_ERROR_GAIN_MASK		(0xff << 16)
#define CONFIG_INIT_VOLTAGE_MASK	(0xff << 8)
#define CONFIG_TIMEOUT_ENABLE_MASK	(0x01 << 3)
#define CONFIG_INITVDD_MASK		(0x01 << 2)
#define CONFIG_FORCEUPDATE_MASK		(0x01 << 1)
#define CONFIG_VP_ENABLE_MASK		(0x01 << 0)

/* Status register masks - All revisions */
#define STATUS_VP_IN_IDLE_MASK		(0x01 << 0)

/* Vlmitto register masks - All revisions */
#define VLIMITTO_VDDMAX_MASK		(0xff << 24)
#define VLIMITTO_VDDMIN_MASK		(0xff << 16)
#define VLIMITTO_TIMEOUT_MASK		(0xffff << 0)

/* Voltage register masks - All revisions */
#define VOLTAGE_MASK			(0xff << 0)

/* Step max/min register masks - All revisions */
#define STEP_SMPSTIMEOUT_MASK		(0xffff << 8)
#define STEP_VSTEP_MASK			(0xff << 0)

/* 32 bit voltage processor registers */
static struct regmap_config omap_vp_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
};

/**
 * struct omap_vp - Structure representing Voltage Processor info
 * @dev:		device pointer for Voltage Processor
 * @list:		list head for VP list.
 * @usage_count:	Usage count - only 1 user at a time.(not always module)
 * @clk_rate:		Sysclk rate for VP computation.
 * @vc:			Voltage controller channel corresponding to VP
 * @pmic:		PMIC used for this path
 * @regmap:		regmap for VP instance
 * @regs:		register map
 * @int_base:		interrupt register base address
 * @txdone_mask:	TRANXDONE interrupt mask for this VP instance in intreg
 * @min_uV:		minimum voltage allowed by VP in micro-volts
 * @max_uV:		maximum voltage allowed by VP in micro-volts
 * @min_step_uV:	minimum continous voltage step in micro-volts for VP
 * @max_step_uV:	maximum continous voltage step in micro-volts for VP
 */
struct omap_vp {
	struct device *dev;
	struct list_head list;
	int usage_count;

	unsigned long clk_rate;
	struct omap_vc_channel_info *vc;
	struct omap_pmic *pmic;

	struct regmap *regmap;
	const struct omap_vp_reg_data *regs;

	void __iomem *int_base;
	u32 txdone_mask;

	u32 min_uV;
	u32 max_uV;
	u32 min_step_uV;
	u32 max_step_uV;
};

static const struct of_device_id omap_vp_of_match[] = {
	{.compatible = "ti,omap3-vp", .data = &omap_vp_reg_type1},
	{.compatible = "ti,omap4-vp", .data = &omap_vp_reg_type2},
	{.compatible = "ti,omap5-vp", .data = &omap_vp_reg_type2},
	{},
};
MODULE_DEVICE_TABLE(of, omap_vp_of_match);

static LIST_HEAD(omap_vp_list);
static DEFINE_MUTEX(omap_vp_list_mutex);

/**
 * omap_vp_check_txdone() - inline helper to see if TRANXDONE is set
 * @vp:	pointer to voltage processor
 */
static inline bool omap_vp_check_txdone(const struct omap_vp *vp)
{
	return !!(readl(vp->int_base) & vp->txdone_mask);
}

/**
 * omap_vp_clear_txdone() - inline helper to clear TRANXDONE
 * @vp:	pointer to voltage processor
 *
 * write of 1 bit clears that interrupt bit only.
 */
static inline void omap_vp_clear_txdone(const struct omap_vp *vp)
{
	writel(vp->txdone_mask, vp->int_base);
};

/**
 * omap_vp_read_idle() - inline helper to read idle register
 * @regmap:	regmap for voltage processor
 * @regs:	registers for voltage processor
 */
static inline u32 omap_vp_read_idle(struct regmap *regmap,
				    const struct omap_vp_reg_data *regs)
{
	u32 val = 0;
	regmap_read(regmap, regs->status, &val);
	return val;
}

/**
 * omap_vp_wait_for_idle() - Wait for Voltage processor to idle
 * @vp:	pointer to voltage processor
 *
 * helper to ensure that VP is idle (no pending AVS / previous VP operations)
 */
static inline int omap_vp_wait_for_idle(struct omap_vp *vp)
{
	struct device *dev = vp->dev;
	struct regmap *regmap = vp->regmap;
	const struct omap_vp_reg_data *regs = vp->regs;
	struct omap_pmic *pmic = vp->pmic;
	const struct omap_pmic_info *pinfo = pmic->info;
	int timeout;

	omap_vp_test_timeout((omap_vp_read_idle(regmap, regs) &
			      STATUS_VP_IN_IDLE_MASK), pinfo->i2c_timeout_us,
			     timeout);

	if (timeout >= pinfo->i2c_timeout_us) {
		dev_warn_ratelimited(dev, "%s: idle timedout(%d)\n",
				     __func__, pinfo->i2c_timeout_us);
		return -ETIMEDOUT;
	}

	return 0;
}

/**
 * omap_vp_set_init_voltage() - Setup voltage for transmission.
 * @vp:	pointer to voltage processor
 * @volt: voltage to setup the voltage processor with
 */
static int omap_vp_set_init_voltage(struct omap_vp *vp, u32 volt)
{
	struct regmap *regmap = vp->regmap;
	const struct omap_vp_reg_data *regs = vp->regs;
	struct omap_pmic *pmic = vp->pmic;
	struct omap_pmic_ops *ops = pmic->ops;
	char vsel;
	int ret;

	ret = ops->uv_to_vsel(pmic, volt, &vsel);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, regs->config,
				 (CONFIG_INIT_VOLTAGE_MASK |
				  CONFIG_FORCEUPDATE_MASK |
				  CONFIG_INITVDD_MASK),
				 vsel << __ffs(CONFIG_INIT_VOLTAGE_MASK));
	if (ret)
		return ret;

	/* Trigger initVDD value copy to voltage processor */
	ret = regmap_update_bits(regmap, regs->config,
				 CONFIG_INITVDD_MASK, CONFIG_INITVDD_MASK);
	if (ret)
		return ret;

	/* Clear initVDD copy trigger bit */
	ret = regmap_update_bits(regmap, regs->config,
				 CONFIG_INITVDD_MASK, 0x0);

	return ret;
}

/**
 * omap_vp_get_current_voltage() - get the current voltage processor voltage
 * @vp:	pointer to voltage processor
 * @uv:	returns with voltage in micro-volts if read was successful.
 */
static int omap_vp_get_current_voltage(struct omap_vp *vp, u32 *uv)
{
	struct device *dev = vp->dev;
	struct regmap *regmap = vp->regmap;
	const struct omap_vp_reg_data *regs = vp->regs;
	struct omap_pmic *pmic = vp->pmic;
	struct omap_pmic_ops *ops = pmic->ops;
	u32 val;
	u8 vsel;
	int ret;

	ret = regmap_read(regmap, regs->config, &val);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "%s: unable to read config reg (%d)\n",
				     __func__, ret);
		return ret;
	}

	val &= CONFIG_INIT_VOLTAGE_MASK;
	vsel = val >> __ffs(CONFIG_INIT_VOLTAGE_MASK);
	ret = ops->vsel_to_uv(pmic, vsel, &val);

	if (!ret)
		*uv = val;
	return ret;
}

/**
 * omap_vp_forceupdate_scale() - Update voltage on PMIC using VP "Forceupdate"
 * @vp:			pointer to voltage processor
 * @target_volt:	voltage to set the PMIC to
 *
 * This will wait for the slew duration to ensure that the voltage is sync-ed
 * on the PMIC.
 */
static int omap_vp_forceupdate_scale(struct omap_vp *vp, u32 target_volt)
{
	struct device *dev = vp->dev;
	struct regmap *regmap = vp->regmap;
	const struct omap_vp_reg_data *regs = vp->regs;
	struct omap_pmic *pmic = vp->pmic;
	const struct omap_pmic_info *pinfo = pmic->info;
	int ret, timeout = 0, max_timeout;
	u32 old_volt = 0;
	u32 smps_transition_uv, smps_delay;

	ret = omap_vp_wait_for_idle(vp);
	if (ret)
		return ret;

	ret = omap_vp_get_current_voltage(vp, &old_volt);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "%s: Unable to convert old voltage(%d)\n",
				     __func__, ret);
		/* We will use worst case start voltage - 0V for delay */
	}

	/*
	 * Clear all pending TransactionDone interrupt/status. Typical latency
	 * is <3us - use an conservative value from pmic info.
	 */
	max_timeout = 2 * pinfo->i2c_timeout_us;
	while (timeout++ < max_timeout) {
		omap_vp_clear_txdone(vp);
		if (!omap_vp_check_txdone(vp))
			break;
		udelay(1);
	}
	if (timeout >= max_timeout) {
		dev_warn_ratelimited(dev,
				     "%s: TRANXDONE not clear(t=%d v=%d)\n",
				     __func__, max_timeout, target_volt);
		return -ETIMEDOUT;
	}

	ret = omap_vp_set_init_voltage(vp, target_volt);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "%s: Fail set init voltage at v=%d(%d)\n",
				     __func__, target_volt, ret);
		return ret;
	}

	/* Force update of voltage */
	ret = regmap_update_bits(regmap, regs->config,
				 CONFIG_FORCEUPDATE_MASK,
				 CONFIG_FORCEUPDATE_MASK);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "%s: Forceupdate not set v=%d (%d)\n",
				     __func__, target_volt, ret);
		return ret;
	}

	/*
	 * Wait for TransactionDone. Typical latency is <200us.
	 * Depends on SMPSWAITTIMEMIN/MAX and voltage change
	 */
	timeout = 0;
	omap_vp_test_timeout(omap_vp_check_txdone(vp), max_timeout, timeout);
	if (timeout >= max_timeout) {
		dev_warn_ratelimited(dev,
				     "%s: TRANXDONE not set(t=%d v=%d)\n",
				     __func__, max_timeout, target_volt);
		return -ETIMEDOUT;
	}

	/*
	 * Due to the inability of OMAP Voltage controller OR voltage processor
	 * to precisely know when the voltage has achieved the requested value,
	 * we need a delay loop to ensure that the voltage has transitioned to
	 * the required level.
	 */
	smps_transition_uv = abs(target_volt - old_volt);

	/* delta_voltage / slew_rate, 2uS added as buffer */
	smps_delay = DIV_ROUND_UP(smps_transition_uv, pinfo->slew_rate_uV) + 2;

	/* We dont want to sleep for too long either */
	usleep_range(smps_delay, smps_delay + 2);

	/*
	 * Disable TransactionDone interrupt , clear all status, clear
	 * control registers
	 */
	timeout = 0;
	while (timeout++ < max_timeout) {
		omap_vp_clear_txdone(vp);
		if (!omap_vp_check_txdone(vp))
			break;
		udelay(1);
	}
	if (timeout >= max_timeout) {
		dev_warn_ratelimited(dev,
				     "%s: TRANXDONE not recleared(t=%d v=%d)\n",
				     __func__, max_timeout, target_volt);
		return -ETIMEDOUT;
	}

	/* Clear force bit */
	ret = regmap_update_bits(regmap, regs->config,
				 CONFIG_FORCEUPDATE_MASK, 0x0);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "%s: Forceupdate not cleared v=%d (%d)\n",
				     __func__, target_volt, ret);
		return ret;
	}

	/* Do the required updates */
	ret = omap_vc_channel_set_on_voltage(vp->vc, target_volt);
	if (ret) {
		dev_warn_ratelimited(dev,
				     "%s: Fail update VC onV at v=%d (%d)\n",
				     __func__, target_volt, ret);
		return ret;
	}

	/* Now, Wait for VP to idle down */
	ret = omap_vp_wait_for_idle(vp);

	return ret;
}

/**
 * omap_vp_setup() - Setup voltage processor
 * @vp:	pointer to voltage processor
 */
static int omap_vp_setup(struct omap_vp *vp)
{
	struct omap_pmic *pmic = vp->pmic;
	const struct omap_pmic_info *pinfo = pmic->info;
	struct omap_pmic_ops *ops = pmic->ops;
	struct regmap *regmap = vp->regmap;
	const struct omap_vp_reg_data *regs = vp->regs;
	u32 val, clk_rate, timeout, waittime;
	u8 vstepmin, vstepmax;
	u8 vddmin, vddmax;
	int ret;

	/* Div 1000 to avoid overflow */
	clk_rate = vp->clk_rate / 1000;

	ret = ops->uv_to_vsel(pmic, vp->min_uV, &vddmin);
	if (ret)
		return ret;
	ret = ops->uv_to_vsel(pmic, vp->max_uV, &vddmax);
	if (ret)
		return ret;

	timeout = DIV_ROUND_UP_ULL(clk_rate * pinfo->i2c_timeout_us, 1000);
	waittime = DIV_ROUND_UP_ULL(pinfo->step_size_uV * clk_rate,
				    1000 * pinfo->slew_rate_uV);

	vstepmin = DIV_ROUND_UP(vp->min_step_uV, pinfo->step_size_uV);
	vstepmax = DIV_ROUND_UP(vp->max_step_uV, pinfo->step_size_uV);

	/* VSTEPMIN */
	val =
	    (waittime << __ffs(STEP_SMPSTIMEOUT_MASK)) & STEP_SMPSTIMEOUT_MASK;
	val |= (vstepmin << __ffs(STEP_VSTEP_MASK)) & STEP_VSTEP_MASK;
	ret = regmap_write(regmap, regs->step_min, val);
	if (ret)
		return ret;

	/* VSTEPMIN */
	val =
	    (waittime << __ffs(STEP_SMPSTIMEOUT_MASK)) & STEP_SMPSTIMEOUT_MASK;
	val |= (vstepmax << __ffs(STEP_VSTEP_MASK)) & STEP_VSTEP_MASK;
	ret = regmap_write(regmap, regs->step_max, val);
	if (ret)
		return ret;

	/* VLIMITTO */
	val = (vddmax << __ffs(VLIMITTO_VDDMAX_MASK)) & VLIMITTO_VDDMAX_MASK;
	val |= (vddmin << __ffs(VLIMITTO_VDDMIN_MASK)) & VLIMITTO_VDDMIN_MASK;
	val |=
	    (timeout << __ffs(VLIMITTO_TIMEOUT_MASK)) & VLIMITTO_TIMEOUT_MASK;
	ret = regmap_write(regmap, regs->vlimitto, val);
	if (ret)
		return ret;

	/* CONFIG */
	ret =
	    regmap_update_bits(regmap, regs->config, CONFIG_TIMEOUT_ENABLE_MASK,
			       CONFIG_TIMEOUT_ENABLE_MASK);
	if (ret)
		return ret;

	return ret;
}

/**
 * devm_omap_vp_release() -  helper to keep track of free usage.
 * @dev:	device
 * @res:	resource
 */
static void devm_omap_vp_release(struct device *dev, void *res)
{
	struct omap_vp *vp = *((struct omap_vp **)res);

	mutex_lock(&omap_vp_list_mutex);

	if (!vp->usage_count) {
		vp->pmic = NULL;
		vp->vc = NULL;
	}
	module_put(vp->dev->driver->owner);

	mutex_unlock(&omap_vp_list_mutex);

	return;
}

/**
 * of_get_omap_vp() - get the pmic node
 * @dev:	device to pull information from
 */
static struct device_node *of_get_omap_vp(struct device *dev)
{
	struct device_node *pmic_node = NULL;
	char *prop_name = "ti,vp";

	dev_dbg(dev, "%s: Looking up %s from device tree\n", __func__,
		prop_name);

	pmic_node = of_parse_phandle(dev->of_node, prop_name, 0);

	if (!pmic_node) {
		dev_err(dev, "%s: Looking up %s property in node %s failed",
			__func__, prop_name, dev->of_node->full_name);
		return ERR_PTR(-ENODEV);
	}
	return pmic_node;
}

/**
 * devm_omap_vp_get() - managed request to get a VP device
 * @dev:	Generic device to handle the request for
 * @pmic:	PMIC resource this will be assigned to
 *
 * Ensures that vp usage count is maintained. Uses managed device,
 * so everything is undone on driver detach.
 *
 * Return: -EPROBE_DEFER if the node is present, however device is
 * not yet probed.
 * -EINVAL if bad pointers or node description is not found.
 * -ENODEV if the property cannot be found
 * -ENOMEM if allocation could not be done.
 *  device pointer to vp dev if all successful.
 *  Error handling should be performed with IS_ERR
 */
static struct device *devm_omap_vp_get(struct device *dev,
				       struct omap_pmic *pmic)
{
	const struct omap_pmic_info *pinfo;
	struct omap_vp *vp, **ptr;
	struct device_node *node;
	struct device *vp_dev;
	struct omap_vc_channel_info *vc;
	u32 min_uV, max_uV;
	int ret = 0;

	if (!dev || !dev->of_node) {
		pr_err("%s: invalid parameters\n", __func__);
		return ERR_PTR(-EINVAL);
	}

	node = of_get_omap_vp(dev);
	if (IS_ERR(node))
		return (void *)node;

	mutex_lock(&omap_vp_list_mutex);
	list_for_each_entry(vp, &omap_vp_list, list)
	    if (vp->dev->of_node == node)
		goto found;

	/* Node definition is present, but not probed yet.. request defer */
	vp_dev = ERR_PTR(-EPROBE_DEFER);
	goto out_unlock;

found:
	vp_dev = vp->dev;
	if (!try_module_get(vp_dev->driver->owner)) {
		dev_err(dev, "%s: Cant get device owner\n", __func__);
		vp_dev = ERR_PTR(-EINVAL);
		goto out_unlock;
	}

	/* Allow ONLY 1 user at a time */
	if (vp->usage_count) {
		dev_err(dev, "%s: device %s is busy..\n", __func__,
			dev_name(vp_dev));
		ret = -EBUSY;
		goto out;
	}
	vc = devm_omap_vc_channel_get(vp_dev, pmic);
	if (IS_ERR(vc)) {
		ret = PTR_ERR(vc);
		dev_err(dev, "%s: vc channel not ready(%d) in %s?\n",
			__func__, ret, dev_name(vp_dev));
		goto out;
	}
	vp->vc = vc;
	vp->pmic = pmic;
	pinfo = pmic->info;

	/* Adjust our voltages */
	/* Cant go below PMIC min voltage */
	min_uV = max(vp->min_uV, pinfo->min_uV);
	/* Cant go below SoC retention voltage for operational case */
	min_uV = max(min_uV, vc->retention_uV);
	vp->min_uV = min_uV;

	/* Cant go above PMIC max voltage */
	max_uV = min(vp->max_uV, pinfo->max_uV);
	vp->max_uV = max_uV;

	ret = omap_vp_setup(vp);
	if (ret) {
		dev_err(dev, "%s: Failed to setup vp (%d) dev %s\n",
			__func__, ret, dev_name(vp_dev));
		goto out;
	}

	if (pmic->boot_voltage_uV) {
		ret = omap_vp_forceupdate_scale(vp, pmic->boot_voltage_uV);
		if (ret) {
			dev_err(dev, "%s: Failed to set boot voltage %d(%d)\n",
				__func__, pmic->boot_voltage_uV, ret);
			goto out;
		}
	}

	ptr = devres_alloc(devm_omap_vp_release, sizeof(*ptr), GFP_KERNEL);
	if (!ptr) {
		ret = -ENOMEM;
		goto out;
	}
	*ptr = vp;
	vp->usage_count++;
	devres_add(dev, ptr);

out:
	if (ret) {
		module_put(vp_dev->driver->owner);
		vp_dev = ERR_PTR(ret);
	}
out_unlock:
	mutex_unlock(&omap_vp_list_mutex);

	return vp_dev;
}

/**
 * omap_vp_voltage_set() - controller operation to set voltage
 * @dev:	VP device to set voltage
 * @uv:		voltage in micro-volts to set
 */
static int omap_vp_voltage_set(struct device *dev, u32 uv)
{
	struct omap_vp *vp = dev_get_drvdata(dev);

	if (!vp)
		return -EINVAL;
	if (!vp->pmic || !vp->vc)
		return -EINVAL;

	return omap_vp_forceupdate_scale(vp, uv);
}

/**
 * omap_vp_voltage_get() - controller operation to get voltage
 * @dev:	VP device to get voltage from
 * @uv:		returns voltage in micro-volts if successful
 */
static int omap_vp_voltage_get(struct device *dev, u32 *uv)
{
	struct omap_vp *vp = dev_get_drvdata(dev);

	if (!vp || !uv)
		return -EINVAL;
	if (!vp->pmic || !vp->vc)
		return -EINVAL;

	return omap_vp_get_current_voltage(vp, uv);
}

/**
 * omap_vp_voltage_get_range() - controller function to return VP voltage range
 * @dev:	VP device to query
 * @min_uv:	if successful, returns min voltage supported by VP
 * @max_uv:	if successful, returns max voltage supported by VP
 */
static int omap_vp_voltage_get_range(struct device *dev, u32 *min_uv,
				     u32 *max_uv)
{
	struct omap_vp *vp = dev_get_drvdata(dev);

	if (!vp || !min_uv || !max_uv)
		return -EINVAL;
	if (!vp->pmic || !vp->vc)
		return -EINVAL;

	*min_uv = vp->min_uV;
	*max_uv = vp->max_uV;
	return 0;
}

static struct omap_pmic_controller_ops voltage_processor_ops = {
	.devm_pmic_register = devm_omap_vp_get,
	.voltage_set = omap_vp_voltage_set,
	.voltage_get = omap_vp_voltage_get,
	.voltage_get_range = omap_vp_voltage_get_range,
};
static bool voltage_processor_ops_registered;

static int omap_vp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	const struct of_device_id *match;
	struct omap_vp *vp;
	struct resource *res;
	struct regmap *regmap;
	char *pname;
	struct clk *clk;
	int ret = 0;
	void __iomem *base, *int_base;

	if (!node) {
		dev_err(dev, "%s: missing device tree nodes?\n", __func__);
		return -EINVAL;
	}

	match = of_match_device(omap_vp_of_match, dev);
	if (!match) {
		/* We do not expect this to happen */
		dev_err(dev, "%s: Unable to match device\n", __func__);
		return -ENODEV;
	}
	if (!match->data) {
		dev_err(dev, "%s: Bad data in match\n", __func__);
		return -EINVAL;
	}

	pname = "base-address";
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, pname);
	base = devm_request_and_ioremap(dev, res);
	if (!base) {
		dev_err(dev, "Unable to map '%s'\n", pname);
		return -EADDRNOTAVAIL;
	}
	pname = "int-address";
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, pname);
	if (!res) {
		dev_err(dev, "Missing '%s' IO resource\n", pname);
		return -ENODEV;
	}

	/*
	 * We may have shared interrupt register offsets which are
	 * write-1-to-clear between domains ensuring exclusivity.
	 */
	int_base = devm_ioremap_nocache(dev, res->start, resource_size(res));
	if (!int_base) {
		dev_err(dev, "Unable to map '%s'\n", pname);
		return -ENOMEM;
	}

	regmap = devm_regmap_init_mmio(dev, base, &omap_vp_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(dev, "regmap init failed(%d)\n", ret);
		return ret;
	}

	vp = devm_kzalloc(dev, sizeof(*vp), GFP_KERNEL);
	if (!vp) {
		dev_err(dev, "%s: Unable to allocate VP\n", __func__);
		return -ENOMEM;
	}
	vp->dev = dev;
	vp->regs = match->data;
	vp->regmap = regmap;
	vp->int_base = int_base;

	clk = clk_get(dev, NULL);
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		dev_err(dev, "%s: Unable to get clk(%d)\n", __func__, ret);
		return ret;
	}
	vp->clk_rate = clk_get_rate(clk);
	/* We dont need the clk any more */
	clk_put(clk);

	pname = "ti,min-micro-volts";
	ret = of_property_read_u32(node, pname, &vp->min_uV);
	if (ret)
		goto invalid_of_property;

	pname = "ti,max-micro-volts";
	ret = of_property_read_u32(node, pname, &vp->max_uV);
	if (ret || !vp->max_uV)
		goto invalid_of_property;

	pname = "ti,min-step-micro-volts";
	ret = of_property_read_u32(node, pname, &vp->min_step_uV);
	if (ret || !vp->min_step_uV)
		goto invalid_of_property;

	pname = "ti,max-step-micro-volts";
	ret = of_property_read_u32(node, pname, &vp->max_step_uV);
	if (ret || !vp->max_step_uV)
		goto invalid_of_property;

	pname = "ti,tranxdone-status-mask";
	ret = of_property_read_u32(node, pname, &vp->txdone_mask);
	if (ret || !vp->txdone_mask)
		goto invalid_of_property;

	platform_set_drvdata(pdev, vp);

	mutex_lock(&omap_vp_list_mutex);
	if (!voltage_processor_ops_registered) {
		ret = omap_pmic_register_controller_ops(&voltage_processor_ops);
		if (ret)
			dev_err(dev, "Failed register pmic cops (%d)\n", ret);
		else
			voltage_processor_ops_registered = true;
	}
	if (!ret)
		list_add(&vp->list, &omap_vp_list);

	mutex_unlock(&omap_vp_list_mutex);
	return ret;

invalid_of_property:
	if (!ret) {
		dev_err(dev, "%s: Invalid value 0x0 in '%s' property.\n",
			__func__, pname);
		ret = -EINVAL;
	} else {
		dev_err(dev, "%s: Missing/Invalid '%s' property - error(%d)\n",
			__func__, pname, ret);
	}
	return ret;
}

static int omap_vp_remove(struct platform_device *pdev)
{
	struct omap_vp *vp = platform_get_drvdata(pdev);

	mutex_lock(&omap_vp_list_mutex);
	list_del(&vp->list);
	mutex_unlock(&omap_vp_list_mutex);

	return 0;
}

static struct platform_driver omap_vp_driver = {
	.probe = omap_vp_probe,
	.remove = omap_vp_remove,
	.driver = {
		   .name = DRIVER_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = of_match_ptr(omap_vp_of_match),
		   },
};
module_platform_driver(omap_vp_driver);

MODULE_DESCRIPTION("OMAP Voltage Processor Regulator Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
MODULE_AUTHOR("Texas Instruments Inc.");
