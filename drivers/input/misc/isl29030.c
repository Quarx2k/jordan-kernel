/*
 * Copyright (C) 2009-2010 Motorola, Inc.
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

#include <linux/isl29030.h>

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/types.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

struct isl29030_data {
	struct input_dev *dev;
	struct i2c_client *client;
	struct regulator *regulator;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct isl29030_platform_data *pdata;
	struct miscdevice miscdevice;
	struct notifier_block pm_notifier;
	struct mutex mutex;
	unsigned int suspended;
	unsigned int hw_initialized;
	unsigned int prox_enabled;
	unsigned int als_enabled;
	unsigned int prox_near;
	unsigned int last_prox_near;
	unsigned int lux_level;
};

static struct isl29030_data *isl29030_misc_data;

static struct isl29030_reg {
	const char *name;
	u8 reg;
} isl29030_regs[] = {
	{ "CHIP_ID",		ISL29030_CHIPID },
	{ "CONFIGURE",		ISL29030_CONFIGURE },
	{ "INTERRUPT",		ISL29030_INTERRUPT },
	{ "PROX_LT",		ISL29030_PROX_LT },
	{ "PROX_HT",		ISL29030_PROX_HT },
	{ "ALS_IR_TH1",		ISL29030_ALSIR_TH1 },
	{ "ALS_IR_TH2",		ISL29030_ALSIR_TH2 },
	{ "ALS_IR_TH3",		ISL29030_ALSIR_TH3 },
	{ "PROX_DATA",		ISL29030_PROX_DATA },
	{ "ALS_IR_DT1",		ISL29030_ALSIR_DT1 },
	{ "ALS_IR_DT2",		ISL29030_ALSIR_DT2 },
	{ "ENABLE",		ISL29030_TEST1 },
	{ "DISABLE",		ISL29030_TEST2 },
};

#define ISL29030_DBG_INPUT		0x00000001
#define ISL29030_DBG_POWER_ON_OFF	0x00000002
#define ISL29030_DBG_ENABLE_DISABLE	0x00000004
#define ISL29030_DBG_IOCTL		0x00000008
#define ISL29030_DBG_SUSPEND_RESUME	0x00000010
static u32 isl29030_debug = 0x00000000;

module_param_named(als_debug, isl29030_debug, uint, 0664);

static int isl29030_read_reg(struct isl29030_data *isl,
			     u8 reg,
			     u8 *value)
{
	int error = 0;
	int i = 0;
	u8 dest_buffer;

	do {
		dest_buffer = reg;
		error = i2c_master_send(isl->client, &dest_buffer, 1);
		if (error == 1) {
			error = i2c_master_recv(isl->client,
				&dest_buffer, LD_ISL29030_ALLOWED_R_BYTES);
		}
		if (error != LD_ISL29030_ALLOWED_R_BYTES) {
			pr_err("%s: read[%i] failed: %d\n", __func__, i, error);
			msleep_interruptible(LD_ISL29030_I2C_RETRY_DELAY);
		}
	} while ((error != LD_ISL29030_ALLOWED_R_BYTES) &&
			((++i) < LD_ISL29030_MAX_RW_RETRIES));

	if (error == LD_ISL29030_ALLOWED_R_BYTES) {
		error = 0;
		if (value)
			*value = dest_buffer;
	}

	return error;
}

static int isl29030_write_reg(struct isl29030_data *isl,
			      u8 reg,
			      u8 value)
{
	u8 buf[LD_ISL29030_ALLOWED_W_BYTES] = { reg, value };
	int bytes;
	int i = 0;

	do {
		bytes = i2c_master_send(isl->client, buf,
			LD_ISL29030_ALLOWED_W_BYTES);

		if (bytes != LD_ISL29030_ALLOWED_W_BYTES) {
			pr_err("%s: write %d failed: %d\n", __func__, i, bytes);
			msleep_interruptible(LD_ISL29030_I2C_RETRY_DELAY);
		}
	} while ((bytes != (LD_ISL29030_ALLOWED_W_BYTES))
		&& ((++i) < LD_ISL29030_MAX_RW_RETRIES));

	if (bytes != LD_ISL29030_ALLOWED_W_BYTES) {
		pr_err("%s: i2c_master_send error\n", __func__);
		return -EIO;
	}

	return 0;
}

static int isl29030_set_bit(struct isl29030_data *isl,
				   u8 reg,
				   u8 bit_mask,
				   u8 value)
{
	int error;
	u8 reg_val;

	error = isl29030_read_reg(isl, reg, &reg_val);
	if (error != 0) {
		pr_err("%s:Unable to read register 0x%x: %d\n",
			__func__, reg, error);
		return -EFAULT;
	}

	if (value)
		reg_val |= (u8)bit_mask;
	else
		reg_val &= (u8)~bit_mask;

	error = isl29030_write_reg(isl, reg, reg_val);
	if (error != 0) {
		pr_err("%s:Unable to write register 0x%x: %d\n",
			__func__, reg, error);
		return -EFAULT;
	}

	return 0;
}

static int isl29030_set_als_enable(struct isl29030_data *isl,
				   unsigned int value)
{
	if (value && isl->prox_near)
		return 0;

	return isl29030_set_bit(isl, ISL29030_CONFIGURE,
		ISL29030_CNF_ALS_EN_MASK, value);
}

static int isl29030_set_als_range(struct isl29030_data *isl,
				  unsigned int value)
{
	return isl29030_set_bit(isl, ISL29030_CONFIGURE,
		ISL29030_CNF_ALS_RANGE_MASK, value);
}

static int isl29030_clear_als_flag(struct isl29030_data *isl)
{
	return isl29030_set_bit(isl, ISL29030_INTERRUPT,
		ISL29030_ALS_FLAG_MASK, 0);
}

static int isl29030_clear_prox_flag(struct isl29030_data *isl)
{
	return isl29030_set_bit(isl, ISL29030_INTERRUPT,
		ISL29030_PROX_FLAG_MASK, 0);
}

static int isl29030_clear_prox_and_als_flags(struct isl29030_data *isl)
{
	return isl29030_set_bit(isl, ISL29030_INTERRUPT,
		ISL29030_ALS_FLAG_MASK | ISL29030_PROX_FLAG_MASK, 0);
}

static int isl29030_set_prox_enable(struct isl29030_data *isl,
				    unsigned int value)
{
	int error;
	if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
		pr_info("%s: bit = %d prox_near = %d\n",
			__func__, value, isl->prox_near);

	error = isl29030_set_bit(isl, ISL29030_CONFIGURE,
		ISL29030_CNF_PROX_EN_MASK,
		value);

	if (!error && !value && isl->prox_near) {
		isl->prox_near = 0;

		if (isl->als_enabled && !isl->suspended) {
			if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
				pr_info("%s: Prox disabled - "
					"re-enabling ALS\n", __func__);
			error = isl29030_set_als_enable(isl, 1);
		}
	}
	return error;
}

static int isl29030_init_registers(struct isl29030_data *isl)
{
	/* as per intersil recommendations */
	if (isl29030_write_reg(isl, ISL29030_CONFIGURE, 0) ||
		isl29030_write_reg(isl, ISL29030_TEST2, 0x29) ||
		isl29030_write_reg(isl, ISL29030_TEST1, 0) ||
		isl29030_write_reg(isl, ISL29030_TEST2, 0)) {

		pr_err("%s:Register initialization failed\n", __func__);
		return -EIO;
	}
	msleep(2);

	if (isl29030_write_reg(isl, ISL29030_CONFIGURE,
			isl->pdata->configure) ||
		isl29030_write_reg(isl, ISL29030_INTERRUPT,
			isl->pdata->interrupt_cntrl) ||
		isl29030_write_reg(isl, ISL29030_PROX_LT,
			isl->pdata->prox_lower_threshold) ||
		isl29030_write_reg(isl, ISL29030_PROX_HT,
			isl->pdata->prox_higher_threshold) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH1, 0xFF) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH2, 0xFF) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH3, 0xFF)) {
		pr_err("%s:Register initialization failed\n", __func__);
		return -EIO;
	}
	isl->lux_level = ISL29030_HIGH_LUX_RANGE;
	return 0;
}

static int isl29030_read_adj_als(struct isl29030_data *isl,
				 unsigned int *raw_als_count)
{
	int lens_adj_lux = -1;
	int error;
	unsigned int als_read_data;
	u8 als_lower;
	u8 als_upper;

	error = isl29030_read_reg(isl, ISL29030_ALSIR_DT1, &als_lower);
	if (error != 0) {
		pr_err("%s:Unable to read ISL29030_ALSIR_DT1 register: %d\n",
			__func__, error);
		return error;
	}

	error = isl29030_read_reg(isl, ISL29030_ALSIR_DT2, &als_upper);
	if (error != 0) {
		pr_err("%s:Unable to read ISL29030_ALSIR_DT2 register: %d\n",
			__func__, error);
		return error;
	}

	als_read_data = (als_upper << 8) | als_lower;
	if (raw_als_count)
		*raw_als_count = als_read_data;

	if (isl29030_debug & ISL29030_DBG_INPUT)
		pr_info("%s: Data read from ALS 0x%X\n",
		__func__, als_read_data);

	lens_adj_lux = (isl->lux_level * als_read_data) /
		(isl->pdata->lens_percent_t * 41);

	return lens_adj_lux;
}

static int isl29030_set_als_thresholds(struct isl29030_data *isl,
				       unsigned int raw_als_count)
{
	unsigned int zone_size, als_low, als_high;
	unsigned int switch_range = 0;

	/* turn off ALS since we're going to be reconfiguring it */
	if (isl29030_set_als_enable(isl, 0))
		return -EFAULT;

	/* if we're in the highest low-lux range, switch to high lux
		or if in lowest high-lux range, switch to low lux*/
	if ((isl->lux_level == ISL29030_LOW_LUX_RANGE) &&
		(raw_als_count >= ISL29030_LOW_TO_HIGH_COUNTS)) {
		if (isl29030_debug & ISL29030_DBG_INPUT)
			pr_info("%s: Switching to high lux range\n", __func__);
		isl->lux_level	= ISL29030_HIGH_LUX_RANGE;
		switch_range	= 1;
		raw_als_count	= raw_als_count * ISL29030_LOW_LUX_RANGE
			/ ISL29030_HIGH_LUX_RANGE;

	} else if ((isl->lux_level == ISL29030_HIGH_LUX_RANGE) &&
		(raw_als_count <= ISL29030_HIGH_TO_LOW_COUNTS)) {
		if (isl29030_debug & ISL29030_DBG_INPUT)
			pr_info("%s: Switching to low lux range\n", __func__);
		isl->lux_level	= ISL29030_LOW_LUX_RANGE;
		switch_range	= 1;
		raw_als_count	= raw_als_count * ISL29030_HIGH_LUX_RANGE
			/ ISL29030_LOW_LUX_RANGE;
	}

	zone_size = 1;
	als_low = ((raw_als_count > zone_size) ? raw_als_count - zone_size : 0);
	if (raw_als_count <= 0xFFE) {
		als_high = raw_als_count + zone_size;
		als_high = ((als_high > 0xFFE) ? 0xFFE : als_high);
	} else {
		als_high = 0xFFF;
	}

	/* reconfigure if needed */
	if (switch_range) {
		isl29030_set_als_range(isl,
			(isl->lux_level == ISL29030_LOW_LUX_RANGE) ? 0 : 1);
	}

	if (isl29030_write_reg(isl, ISL29030_ALSIR_TH1, als_low & 0x0FF) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH2,
			((als_low & 0xF00) >> 8) | ((als_high & 0x00F) << 4)) ||
		isl29030_write_reg(isl, ISL29030_ALSIR_TH3,
			(als_high & 0xFF0) >> 4)) {
		pr_err("%s: Couldn't set als thresholds\n", __func__);

		return -EFAULT;
	}

	if (isl29030_set_als_enable(isl, 1))
		return -EFAULT;

	return 0;
}

static int isl29030_report_prox(struct isl29030_data *isl, int force_report)
{
	int error = 0;
	u8 interrupt_reg;

	error = isl29030_read_reg(isl, ISL29030_INTERRUPT, &interrupt_reg);
	if (error != 0) {
		pr_err("%s:Unable to read interrupt register: %d\n",
			__func__, error);
		return 1;
	}

	if (interrupt_reg & ISL29030_PROX_FLAG_MASK) {
		if (!isl->prox_near) {
			isl->prox_near = 1;

			if (isl29030_debug & ISL29030_DBG_INPUT)
				pr_info("%s:Prox near - disabling ALS if "
					"active due to part limitation\n",
					__func__);
			isl29030_set_als_enable(isl, 0);
		}
	} else {
		if (isl->prox_near) {
			isl->prox_near = 0;

			if (isl->als_enabled && !isl->suspended) {
				if (isl29030_debug & ISL29030_DBG_INPUT)
					pr_info("%s: Prox far - "
						"re-enabling ALS\n", __func__);
				isl29030_set_als_enable(isl, 1);
			}
		}
	}

	/* Don't report the prox state if it hasn't changed. */
	if (force_report || (isl->prox_near != isl->last_prox_near)) {
		isl->last_prox_near = isl->prox_near;
		input_report_abs(isl->dev, ABS_DISTANCE,
			(isl->prox_near ? PROXIMITY_NEAR : PROXIMITY_FAR));
		input_sync(isl->dev);
	}
	return !isl->prox_near;
}

static void isl29030_report_als(struct isl29030_data *isl)
{
	unsigned int raw_als_data = 0;
	int lux_val;

	lux_val = isl29030_read_adj_als(isl, &raw_als_data);
	isl29030_set_als_thresholds(isl, raw_als_data);

	if (lux_val >= 0) {
		input_event(isl->dev, EV_LED, LED_MISC,
			((lux_val > 1) ? lux_val : 2));
		input_sync(isl->dev);
	}
}

static unsigned int isl29030_get_avg_noise_floor(struct isl29030_data *isl)
{
	int err = -EINVAL;
	unsigned int i, sum = 0, avg = 0;
	u8 prox_data;
	unsigned int num_samples = isl->pdata->num_samples_for_noise_floor;

	/* turn off PROX_EN */
	isl29030_set_prox_enable(isl, 0);
	msleep(2);

	for (i = 0; i < num_samples; i++) {
		/* turn on PROX_EN */
		err = isl29030_set_prox_enable(isl, 1);
		if (err) {
			pr_err("%s: Error enabling proximity sensor: %d\n",
				__func__, err);
			break;
		}
		msleep(2); /* sleep for a bit before reading PROX_DATA */
		err = isl29030_read_reg(isl, ISL29030_PROX_DATA, &prox_data);
		if (err) {
			pr_err("%s: Error reading prox data: %d\n",
				__func__, err);
			break;
		}
		/* turn back off */
		err = isl29030_set_prox_enable(isl, 0);
		if (err) {
			pr_err("%s: Error disabling proximity sensor: %d\n",
				__func__, err);
			break;
		}
		msleep(2);
		sum += prox_data;
	}

	if (!err)
		avg = sum / num_samples;

	if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
		pr_info("%s: Noise floor is 0x%x ", __func__, avg);

	return avg;
}

static int isl29030_set_prox_thresholds(struct isl29030_data *isl,
					int avg_noise_floor)
{
	unsigned int prox_ht, prox_lt, offset;

	if ((avg_noise_floor >
		(isl->pdata->crosstalk_vs_covered_threshold)) ||
		(avg_noise_floor == 0)) {
		offset = isl->pdata->default_prox_noise_floor;
	} else {
		offset = avg_noise_floor;
	}
	prox_lt = offset + isl->pdata->prox_lower_threshold;
	prox_ht = offset + isl->pdata->prox_higher_threshold;

	/* check for overflow beyond 1 byte */
	if ((prox_lt > 0xFF) || (prox_ht > 0xFF)) {
		pr_err("%s: noise adjusted proximity thresholds are 0x%x "
			"and 0x%x, overflowing 8 bits, using defaults\n",
			__func__, prox_lt, prox_ht);
		prox_lt = isl->pdata->prox_lower_threshold;
		prox_ht = isl->pdata->prox_higher_threshold;
	}

	if (isl29030_write_reg(isl, ISL29030_PROX_LT, (u8)prox_lt) ||
		isl29030_write_reg(isl, ISL29030_PROX_HT, (u8)prox_ht)) {
		pr_err("%s: Error writing to prox threshold registers\n",
			__func__);
		return -EIO;
	}
	return 0;
}

static void isl29030_device_power_off(struct isl29030_data *isl)
{
	int error;

	if (isl29030_debug & ISL29030_DBG_POWER_ON_OFF)
		pr_info("%s: initialized=%d\n", __func__, isl->hw_initialized);

	if (isl->hw_initialized && isl->regulator) {
		disable_irq_nosync(isl->client->irq);
		error = regulator_disable(isl->regulator);
		if (error) {
			pr_err("%s: regulator_disable failed: %d\n",
				__func__, error);
			enable_irq(isl->client->irq);
			return;
		}
		isl->hw_initialized = 0;
	}
}

static int isl29030_device_power_on(struct isl29030_data *isl)
{
	int error;

	if (isl29030_debug & ISL29030_DBG_POWER_ON_OFF)
		pr_info("%s: initialized=%d\n", __func__, isl->hw_initialized);

	if (!isl->hw_initialized) {
		if (isl->regulator) {
			error = regulator_enable(isl->regulator);
			if (error) {
				pr_err("%s: regulator_enable failed: %d\n",
					__func__, error);
				return error;
			}
		}

		error = isl29030_init_registers(isl);
		if (error < 0) {
			pr_err("%s: init_registers failed: %d\n",
				__func__, error);
			if (isl->regulator)
				regulator_disable(isl->regulator);
			return error;
		}

		enable_irq(isl->client->irq);

		isl->hw_initialized = 1;
	}

	return 0;
}

static int isl29030_enable_prox(struct isl29030_data *isl)
{
	int error;
	int avg_noise_floor;

	if (isl->suspended) {
		if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
			pr_info("%s: Powering on\n", __func__);
		error = isl29030_device_power_on(isl);
		if (error)
			return error;
	}

	if (!isl->prox_enabled) {
		avg_noise_floor = isl29030_get_avg_noise_floor(isl);
		error = isl29030_set_prox_thresholds(isl, avg_noise_floor);
		if (error) {
			pr_err("%s: Error settin prox thresholds: %d\n",
				__func__, error);
			return error;
		}

		error = isl29030_set_prox_enable(isl, 1);
		if (error) {
			pr_err("%s: Error enabling prox: %d\n",
				__func__, error);
			return error;
		}

		isl29030_report_prox(isl, 1);
		
	}

	isl->prox_enabled = 1;

	return 0;
}

static int isl29030_disable_prox(struct isl29030_data *isl)
{
	int error;

	if (isl->prox_enabled) {
		error = isl29030_set_prox_enable(isl, 0);
		if (error) {
			pr_err("%s: Unable to turn off prox: %d\n",
				__func__, error);
			return error;
		}
		isl29030_clear_prox_flag(isl);
	}

	isl->prox_enabled = 0;

	if (isl->suspended && isl->hw_initialized) {
		if (isl29030_debug & ISL29030_DBG_ENABLE_DISABLE)
			pr_info("%s: Powering off\n", __func__);
		isl29030_device_power_off(isl);
	}

	return 0;
}

static int isl29030_enable_als(struct isl29030_data *isl)
{
	int error;

	if (!isl->als_enabled && !isl->suspended) {
		error = isl29030_set_als_enable(isl, 1);
		if (error) {
			pr_err("%s: Unable to turn on ALS: %d\n",
				__func__, error);
			return error;
		}

		msleep(100); /* Allow the light sensor to read the zone */

		isl29030_report_als(isl);
	}

	isl->als_enabled = 1;

	return 0;
}

static int isl29030_disable_als(struct isl29030_data *isl)
{
	int error;

	if (isl->als_enabled) {
		error = isl29030_set_als_enable(isl, 0);
		if (error) {
			pr_err("%s: Unable to turn off ALS: %d\n",
				__func__, error);
			return error;
		}
	}

	isl->als_enabled = 0;

	return 0;
}

static int isl29030_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = isl29030_misc_data;

	return 0;
}

static long isl29030_misc_ioctl_locked(struct isl29030_data *isl,
				       unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 enable;

	if (isl29030_debug & ISL29030_DBG_IOCTL)
		pr_info("%s\n", __func__);

	switch (cmd) {
	case ISL29030_IOCTL_SET_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			isl29030_enable_prox(isl);
		else
			isl29030_disable_prox(isl);

		break;

	case ISL29030_IOCTL_GET_ENABLE:
		enable = isl->prox_enabled;
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	case ISL29030_IOCTL_SET_LIGHT_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			isl29030_enable_als(isl);
		else
			isl29030_disable_als(isl);

		break;

	case ISL29030_IOCTL_GET_LIGHT_ENABLE:
		enable = isl->als_enabled;
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static long isl29030_misc_ioctl(struct file *file, unsigned int cmd,
				unsigned long arg)
{
	struct isl29030_data *isl = file->private_data;
	int error;

	if (isl29030_debug & ISL29030_DBG_IOCTL)
		pr_info("%s: cmd = 0x%08X\n", __func__, cmd);

	mutex_lock(&isl->mutex);
	error = isl29030_misc_ioctl_locked(isl, cmd, arg);
	mutex_unlock(&isl->mutex);

	return error;
}

static const struct file_operations isl29030_misc_fops = {
	.owner = THIS_MODULE,
	.open = isl29030_misc_open,
	.unlocked_ioctl = isl29030_misc_ioctl,
};

static ssize_t ld_isl29030_registers_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct isl29030_data *isl = i2c_get_clientdata(client);
	unsigned int i, n, reg_count;
	u8 value;

	reg_count = sizeof(isl29030_regs) / sizeof(isl29030_regs[0]);
	mutex_lock(&isl->mutex);
	for (i = 0, n = 0; i < reg_count; i++) {
		isl29030_read_reg(isl, isl29030_regs[i].reg, &value);
		n += scnprintf(buf + n, PAGE_SIZE - n,
			"%-20s = 0x%02X\n",
			isl29030_regs[i].name,
			value);
	}
	mutex_unlock(&isl->mutex);

	return n;
}

static ssize_t ld_isl29030_registers_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	struct isl29030_data *isl = i2c_get_clientdata(client);
	unsigned int i, reg_count, value;
	int error;
	char name[30];

	if (count >= 30) {
		pr_err("%s:input too long\n", __func__);
		return -EMSGSIZE;
	}

	if (sscanf(buf, "%30s %x", name, &value) != 2) {
		pr_err("%s:unable to parse input\n", __func__);
		return -EINVAL;
	}

	reg_count = sizeof(isl29030_regs) / sizeof(isl29030_regs[0]);
	for (i = 0; i < reg_count; i++) {
		if (!strcmp(name, isl29030_regs[i].name)) {
			switch (isl29030_regs[i].reg) {
			case ISL29030_PROX_LT:
				isl->pdata->
					prox_lower_threshold = value;
			break;

			case ISL29030_PROX_HT:
				isl->pdata->
					prox_higher_threshold = value;
			break;

			case ISL29030_TEST1:
				mutex_lock(&isl->mutex);
				isl29030_enable_prox(isl);
				mutex_unlock(&isl->mutex);
				return count;
			break;

			case ISL29030_TEST2:
				mutex_lock(&isl->mutex);
				isl29030_disable_prox(isl);
				mutex_unlock(&isl->mutex);
				return count;
			break;
			}
			mutex_lock(&isl->mutex);
			error = isl29030_write_reg(isl,
				isl29030_regs[i].reg,
				value);
			mutex_unlock(&isl->mutex);
			if (error) {
				pr_err("%s:Failed to write register %s\n",
					__func__, name);
				return error;
			}
			return count;
		}
	}

	pr_err("%s:no such register %s\n", __func__, name);
	return -EINVAL;
}

static DEVICE_ATTR(registers, 0644, ld_isl29030_registers_show,
		   ld_isl29030_registers_store);

static irqreturn_t ld_isl29030_irq_handler(int irq, void *dev)
{
	struct isl29030_data *isl = dev;

	disable_irq_nosync(isl->client->irq);
	queue_work(isl->workqueue, &isl->work);
	enable_irq(isl->client->irq);

	return IRQ_HANDLED;
}

static void isl29030_work_func_locked(struct isl29030_data *isl)
{
	int error;
	int clear_prox = 1;
	u8 interrupt_reg;

	error = isl29030_read_reg(isl, ISL29030_INTERRUPT, &interrupt_reg);
	if (error != 0) {
		pr_err("%s: Unable to read interrupt register: %d\n",
			__func__, error);
		return;
	}

	if (isl->als_enabled && (interrupt_reg & ISL29030_ALS_FLAG_MASK))
		isl29030_report_als(isl);

	if (isl->prox_enabled)
		clear_prox = isl29030_report_prox(isl, 0);

	if (clear_prox)
		isl29030_clear_prox_and_als_flags(isl);
	else
		isl29030_clear_als_flag(isl);
}

static void ld_isl29030_work_func(struct work_struct *work)
{
	struct isl29030_data *isl =
		container_of(work, struct isl29030_data, work);

	mutex_lock(&isl->mutex);
	if (isl->hw_initialized)
		isl29030_work_func_locked(isl);
	mutex_unlock(&isl->mutex);
}

static int isl29030_suspend(struct isl29030_data *isl)
{
	int error;

	if (isl->als_enabled) {
		error = isl29030_set_als_enable(isl, 0);
		isl29030_clear_als_flag(isl);
		if (error) {
			pr_err("%s: Unable to turn off ALS: %d\n",
				__func__, error);
			return error;
		}
		if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
			pr_info("%s: turned off ALS\n", __func__);
	}

	if (!isl->prox_enabled)
		isl29030_device_power_off(isl);

	isl->suspended = 1;

	return 0;
}

static int isl29030_resume(struct isl29030_data *isl)
{
	int error;

	isl29030_device_power_on(isl);

	if (isl->als_enabled) {
		error = isl29030_set_als_enable(isl, 1);
		if (error) {
			pr_err("%s: Unable to turn on ALS: %d\n",
				__func__, error);
			return error;
		}

		msleep(100); /* Allow the light sensor to read the zone */

		isl29030_report_als(isl);
	}

	isl->suspended = 0;

	return 0;
}

static int isl29030_pm_event(struct notifier_block *this, unsigned long event,
			     void *ptr)
{
	struct isl29030_data *isl = container_of(this,
		struct isl29030_data, pm_notifier);

	if (isl29030_debug & ISL29030_DBG_SUSPEND_RESUME)
		pr_info("%s: event = %lu\n", __func__, event);

	mutex_lock(&isl->mutex);

	switch (event) {
	case PM_SUSPEND_PREPARE:
		isl29030_suspend(isl);
		break;
	case PM_POST_SUSPEND:
		isl29030_resume(isl);
		break;
	}

	mutex_unlock(&isl->mutex);

	return NOTIFY_DONE;
}

static int ld_isl29030_probe(struct i2c_client *client,
			     const struct i2c_device_id *id)
{
	struct isl29030_platform_data *pdata = client->dev.platform_data;
	struct isl29030_data *isl;
	int error = 0;

	if (pdata == NULL) {
		pr_err("%s: platform data required\n", __func__);
		return -ENODEV;
	}

	client->irq = pdata->irq;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s:I2C_FUNC_I2C not supported\n", __func__);
		return -ENODEV;
	}

	isl = kzalloc(sizeof(struct isl29030_data), GFP_KERNEL);
	if (isl == NULL) {
		error = -ENOMEM;
		goto error_alloc_data_failed;
	}

	pdata->configure &= ~ISL29030_CNF_ALS_EN_MASK;
	pdata->configure &= ~ISL29030_CNF_PROX_EN_MASK;
	pdata->configure |= ISL29030_CNF_ALS_RANGE_MASK;

	isl->client = client;
	isl->pdata = pdata;

	if (isl->pdata->regulator_name[0] != '\0') {
		isl->regulator = regulator_get(NULL,
			isl->pdata->regulator_name);
		if (IS_ERR(isl->regulator)) {
			pr_err("%s: cannot acquire regulator [%s]\n",
				__func__, isl->pdata->regulator_name);
			error = PTR_ERR(isl->regulator);
			goto error_regulator_get_failed;
		}
	}

	isl->dev = input_allocate_device();
	if (!isl->dev) {
		error = -ENOMEM;
		pr_err("%s: input device allocate failed: %d\n", __func__,
			error);
		goto error_input_allocate_failed;
	}

	isl->dev->name = "light-prox";
	input_set_capability(isl->dev, EV_LED, LED_MISC);
	input_set_capability(isl->dev, EV_ABS, ABS_DISTANCE);

	isl29030_misc_data = isl;
	isl->miscdevice.minor = MISC_DYNAMIC_MINOR;
	isl->miscdevice.name = FOPS_ISL29030_NAME;
	isl->miscdevice.fops = &isl29030_misc_fops;
	error = misc_register(&isl->miscdevice);
	if (error < 0) {
		pr_err("%s: misc_register failed\n", __func__);
		goto error_misc_register_failed;
	}

	isl->pm_notifier.notifier_call = isl29030_pm_event;
	error = register_pm_notifier(&isl->pm_notifier);
	if (error < 0) {
		pr_err("%s: register_pm_notifier failed\n", __func__);
		goto error_register_pm_notifier_failed;
	}

	isl->hw_initialized = 0;
	isl->suspended = 0;

	isl->prox_enabled = 0;
	isl->als_enabled = 0;
	isl->lux_level = ISL29030_HIGH_LUX_RANGE;
	isl->prox_near = 0;
	isl->last_prox_near = 0;

	isl->workqueue = create_singlethread_workqueue("als_wq");
	if (!isl->workqueue) {
		pr_err("%s: Cannot create work queue\n", __func__);
		error = -ENOMEM;
		goto error_create_wq_failed;
	}

	INIT_WORK(&isl->work, ld_isl29030_work_func);

	error = request_irq(client->irq,
		ld_isl29030_irq_handler,
		(IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING),
		LD_ISL29030_NAME, isl);
	if (error != 0) {
		pr_err("%s: irq request failed: %d\n", __func__, error);
		error = -ENODEV;
		goto error_req_irq_failed;
	}
	enable_irq_wake(client->irq);
	disable_irq(client->irq);

	i2c_set_clientdata(client, isl);

	mutex_init(&isl->mutex);

	error = input_register_device(isl->dev);
	if (error) {
		pr_err("%s: input device register failed:%d\n", __func__,
			error);
		goto error_input_register_failed;
	}

	error = device_create_file(&client->dev, &dev_attr_registers);
	if (error < 0) {
		pr_err("%s:File device creation failed: %d\n", __func__, error);
		error = -ENODEV;
		goto error_create_registers_file_failed;
	}

	error = isl29030_device_power_on(isl);
	if (error < 0) {
		pr_err("%s:power_on failed: %d\n", __func__, error);
		goto error_power_on_failed;
	}
	printk("ISL29030 probe ok!\n");
	return 0;

error_power_on_failed:
	device_remove_file(&client->dev, &dev_attr_registers);
error_create_registers_file_failed:
	input_unregister_device(isl->dev);
	isl->dev = NULL;
error_input_register_failed:
	mutex_destroy(&isl->mutex);
	i2c_set_clientdata(client, NULL);
	free_irq(isl->client->irq, isl);
error_req_irq_failed:
	destroy_workqueue(isl->workqueue);
error_create_wq_failed:
	unregister_pm_notifier(&isl->pm_notifier);
error_register_pm_notifier_failed:
	misc_deregister(&isl->miscdevice);
error_misc_register_failed:
	input_free_device(isl->dev);
error_input_allocate_failed:
	regulator_put(isl->regulator);
error_regulator_get_failed:
	kfree(isl);
error_alloc_data_failed:
	return error;
}

static int ld_isl29030_remove(struct i2c_client *client)
{
	struct isl29030_data *isl = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_registers);

	isl29030_device_power_off(isl);

	input_unregister_device(isl->dev);

	mutex_destroy(&isl->mutex);
	i2c_set_clientdata(client, NULL);
	free_irq(isl->client->irq, isl);

	destroy_workqueue(isl->workqueue);

	unregister_pm_notifier(&isl->pm_notifier);

	misc_deregister(&isl->miscdevice);

	regulator_put(isl->regulator);

	kfree(isl);

	return 0;
}

static const struct i2c_device_id isl29030_id[] = {
	{LD_ISL29030_NAME, 0},
	{}
};

static struct i2c_driver ld_isl29030_i2c_driver = {
	.probe		= ld_isl29030_probe,
	.remove		= ld_isl29030_remove,
	.id_table	= isl29030_id,
	.driver = {
		.name = LD_ISL29030_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ld_isl29030_init(void)
{
	return i2c_add_driver(&ld_isl29030_i2c_driver);
}

static void __exit ld_isl29030_exit(void)
{
	i2c_del_driver(&ld_isl29030_i2c_driver);
}

module_init(ld_isl29030_init);
module_exit(ld_isl29030_exit);

MODULE_DESCRIPTION("ALS and Proximity driver for ISL29030");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
