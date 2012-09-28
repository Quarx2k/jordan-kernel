/*
 * Copyright (C) 2009 Kionix, Inc.
 * Copyright (C) 2010 Motorola, Inc.
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

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>

#include <linux/workqueue.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/kxtf9.h>

#define NAME			"kxtf9"
#define G_MAX			8000
#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define INT_SRC_REG1		0x15
#define INT_SRC_REG2		0x16
#define TILT_POS_CUR		0x10
#define INT_REL			0x1A
/* CONTROL REGISTERS */
#define DATA_CTRL		0x21
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define CTRL_REG3		0x1D
#define TILT_TIMER		0x28
#define WUF_TIMER		0x29
#define WUF_THRESH		0x5A
#define TDT_TIMER		0x2B
#define SELF_TEST_REG		0x3A
/* CONTROL REGISTER 1 POWER BIT */
#define PC1_ON			0x80
/* INTERRUPT SOURCE 2 BITS */
#define TPS			0x01
#define TDTS0			0x04
#define TDTS1			0x08
/* INPUT_ABS CONSTANTS */
#define FUZZ			32
#define FLAT			32
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RES_TILT_TIMER		3
#define RES_CTRL_REG3		4
#define RES_WUF_TIMER		5
#define RES_WUF_THRESH		6
#define RES_TDT_TIMER		7
#define RES_TDT_H_THRESH	8
#define RES_TDT_L_THRESH	9
#define RES_TAP_TIMER		10
#define RES_TOTAL_TIMER		11
#define RES_LAT_TIMER		12
#define RES_WIN_TIMER		13
#define RESUME_ENTRIES		14

#define SENSITIVITY_LEVELS        3
#define SENSITIVITY_LOW_OFFSET    0
#define SENSITIVITY_MEDIUM_OFFSET 1
#define SENSITIVITY_HIGH_OFFSET   2

#define TILT_REPORT_DELAY	500

struct {
	/* cutoff serves dual purpose as valid output delay */
	unsigned int cutoff;
	u8 mask;
} kxtf9_odr_table[] = {
	{
	3,	ODR800}, {
	5,	ODR400}, {
	10,	ODR200}, {
	20,	ODR100}, {
	40,	ODR50}, {
	80,	ODR25},};

struct tap_sensitivity {
	u8 reg_timer_init;
	u8 reg_h_thresh_init;
	u8 reg_l_thresh_init;
	u8 reg_tap_timer_init;
	u8 reg_total_timer_init;
	u8 reg_latency_timer_init;
	u8 reg_window_timer_init;
};

struct kxtf9_data {
	struct i2c_client *client;
	struct kxtf9_platform_data *pdata;
	struct delayed_work input_work;
	struct input_dev *input_dev;
	struct work_struct irq_work;
	struct workqueue_struct *irq_work_queue;
	struct delayed_work force_tilt;

	int hw_initialized;
	unsigned int odr_delay;
	atomic_t enabled;
	atomic_t is_polling;
	int was_polling_at_suspend;
	u8 shift_adj;
	u8 resume_state[RESUME_ENTRIES];
	int irq;
	struct tap_sensitivity ts_regs[SENSITIVITY_LEVELS];
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kxtf9_early_suspend(struct early_suspend *handler);
static void kxtf9_late_resume(struct early_suspend *handler);
#endif

struct kxtf9_data *kxtf9_misc_data;

static int kxtf9_i2c_read(struct kxtf9_data *tf9, u8 *buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = tf9->client->addr,
		 .flags = tf9->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = tf9->client->addr,
		 .flags = (tf9->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};
	do {
		err = i2c_transfer(tf9->client->adapter, msgs, 2);
		if (err != 2)
			msleep(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&tf9->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int kxtf9_i2c_write(struct kxtf9_data *tf9, u8 * buf, int len)
{
	int err;
	int tries = 0;

	struct i2c_msg msgs[] = {
		{
		 .addr = tf9->client->addr,
		 .flags = tf9->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};
	do {
		err = i2c_transfer(tf9->client->adapter, msgs, 1);
		if (err != 1)
			msleep(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&tf9->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int kxtf9_hw_init(struct kxtf9_data *tf9)
{
	int err = -1;
	u8 buf[8];

	buf[0] = CTRL_REG1;
	buf[1] = tf9->resume_state[RES_CTRL_REG1] & (~PC1_ON);
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		goto error;
	buf[0] = DATA_CTRL;
	buf[1] = tf9->resume_state[RES_DATA_CTRL];
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		goto error;
	buf[0] = CTRL_REG3;
	buf[1] = tf9->resume_state[RES_CTRL_REG3];
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		goto error;
	buf[0] = TILT_TIMER;
	buf[1] = tf9->resume_state[RES_TILT_TIMER];
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		goto error;
	buf[0] = WUF_TIMER;
	buf[1] = tf9->resume_state[RES_WUF_TIMER];
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		goto error;
	buf[0] = WUF_THRESH;
	buf[1] = tf9->resume_state[RES_WUF_THRESH];
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		goto error;
	buf[0] = TDT_TIMER;
	buf[1] = tf9->resume_state[RES_TDT_TIMER];
	buf[2] = tf9->resume_state[RES_TDT_H_THRESH];
	buf[3] = tf9->resume_state[RES_TDT_L_THRESH];
	buf[4] = tf9->resume_state[RES_TAP_TIMER];
	buf[5] = tf9->resume_state[RES_TOTAL_TIMER];
	buf[6] = tf9->resume_state[RES_LAT_TIMER];
	buf[7] = tf9->resume_state[RES_WIN_TIMER];
	err = kxtf9_i2c_write(tf9, buf, 7);
	if (err < 0)
		goto error;
	buf[0] = INT_CTRL1;
	buf[1] = tf9->resume_state[RES_INT_CTRL1];
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		goto error;
	buf[0] = CTRL_REG1;
	buf[1] = (tf9->resume_state[RES_CTRL_REG1] | PC1_ON);
	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		goto error;
	tf9->resume_state[RES_CTRL_REG1] = buf[1];
	tf9->hw_initialized = 1;

	msleep(tf9->odr_delay);

	return 0;
error:
	dev_err(&tf9->client->dev, "hw init error 0x%x,0x%x: %d\n",
		buf[0], buf[1], err);

	return err;
}

static void kxtf9_device_power_off(struct kxtf9_data *tf9)
{
	int err;
	u8 buf[2] = { CTRL_REG1, tf9->resume_state[RES_CTRL_REG1] & (~PC1_ON) };

	err = kxtf9_i2c_write(tf9, buf, 1);
	if (err < 0)
		dev_err(&tf9->client->dev, "soft power off failed: %d\n", err);
	disable_irq_nosync(tf9->irq);
	if (tf9->pdata->power_off) {
		tf9->pdata->power_off();
		tf9->hw_initialized = 0;
	}
}

static int kxtf9_device_power_on(struct kxtf9_data *tf9)
{
	int err;
	u8 intrel = INT_REL;

	if (tf9->pdata->power_on) {
		err = tf9->pdata->power_on();
		if (err < 0) {
			dev_err(&tf9->client->dev,
				"power_on failed: %d\n", err);
			return err;
		}
		msleep(25);
	}
	if (!tf9->hw_initialized) {
		err = kxtf9_hw_init(tf9);
		if (err < 0) {
			kxtf9_device_power_off(tf9);
			return err;
		}
	}
	err = kxtf9_i2c_read(tf9, &intrel, 1);
	if (err < 0) {
		dev_err(&tf9->client->dev,
			"error clearing interrupt status: %d\n", err);
		return err;
	}
	enable_irq(tf9->irq);

	return 0;
}

static irqreturn_t kxtf9_isr(int irq, void *dev)
{
	struct kxtf9_data *tf9 = dev;

	disable_irq_nosync(irq);
	queue_work(tf9->irq_work_queue, &tf9->irq_work);

	return IRQ_HANDLED;
}

static void kxtf9_irq_work_func(struct work_struct *work)
{
	int err;
	/* [status][tapdir][tilt_prev][tilt_curr], high bit gesture toggle */
	unsigned int int_status = 0;
	u8 status;
	u8 buf[2];

	struct kxtf9_data *tf9 = container_of(work,
						struct kxtf9_data, irq_work);
	if (gpio_get_value(tf9->pdata->gpio)) {
		status = INT_SRC_REG2;
		err = kxtf9_i2c_read(tf9, &status, 1);
		if (err < 0) {
			dev_err(&tf9->client->dev,
				"int source read error: %d\n", err);
			goto release;
		}
		int_status = status << 24;
		if ((status & TPS) > 0) {
			buf[0] = TILT_POS_CUR;
			err = kxtf9_i2c_read(tf9, buf, 2);
			if (err < 0) {
				dev_err(&tf9->client->dev,
					"tilt read error: %d\n", err);
			} else {
				int_status |= buf[0];
				int_status |= buf[1] << 8;
			}
		}
		if (((status & TDTS0) | (status & TDTS1)) > 0) {
			buf[0] = INT_SRC_REG1;
			err = kxtf9_i2c_read(tf9, buf, 1);
			if (err < 0)
				dev_err(&tf9->client->dev,
					"tap read error: %d\n", err);
			else
				int_status |= buf[0] << 16;
		}
		if (int_status & 0x1FFFFFFF) {
			int_status |= (tf9->pdata->gesture++ & 1) << 31;
			input_report_abs(tf9->input_dev, ABS_MISC, int_status);
			input_sync(tf9->input_dev);
		}
release:
		buf[0] = INT_REL;
		err = kxtf9_i2c_read(tf9, buf, 1);
		if (err < 0)
			dev_err(&tf9->client->dev,
				"error clearing interrupt status: %d\n", err);
	}
	enable_irq(tf9->irq);
}

int kxtf9_update_g_range(struct kxtf9_data *tf9, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 buf[2] = {0,
		(tf9->resume_state[RES_CTRL_REG1] & 0x67) | new_g_range};

	switch (new_g_range) {
	case KXTF9_G_2G:
		shift = SHIFT_ADJ_2G;
		break;
	case KXTF9_G_4G:
		shift = SHIFT_ADJ_4G;
		break;
	case KXTF9_G_8G:
		shift = SHIFT_ADJ_8G;
		break;
	default:
		dev_err(&tf9->client->dev,
			"invalid g range requested: %u\n", new_g_range);
		return -EINVAL;
	}
	if (shift != tf9->shift_adj) {
		if (tf9->shift_adj > shift)
			tf9->resume_state[RES_WUF_THRESH] >>=
						(tf9->shift_adj - shift);
		if (tf9->shift_adj < shift)
			tf9->resume_state[RES_WUF_THRESH] <<=
						(shift - tf9->shift_adj);
		if (atomic_read(&tf9->enabled)) {
			buf[0] = CTRL_REG1;
			buf[1] = tf9->resume_state[RES_CTRL_REG1] & (~PC1_ON);
			err = kxtf9_i2c_write(tf9, buf, 1);
			if (err < 0)
				goto error;
			buf[0] = WUF_THRESH;
			buf[1] = tf9->resume_state[RES_WUF_THRESH];
			err = kxtf9_i2c_write(tf9, buf, 1);
			if (err < 0)
				goto error;
			buf[0] = CTRL_REG1;
			buf[1] = (tf9->resume_state[RES_CTRL_REG1] & 0x67) |
					new_g_range | PC1_ON;
			err = kxtf9_i2c_write(tf9, buf, 1);
			if (err < 0)
				goto error;
			msleep(tf9->odr_delay);
		}
	}
	tf9->resume_state[RES_CTRL_REG1] = buf[1];
	tf9->shift_adj = shift;

	return 0;
error:
	dev_err(&tf9->client->dev, "update g range failed 0x%x,0x%x: %d\n",
		buf[0], buf[1], err);

	return err;
}

int kxtf9_update_odr(struct kxtf9_data *tf9, int poll_interval)
{
	int err = -1;
	int i;
	unsigned int valid_t;
	u8 config[2] = { DATA_CTRL, 0 };
	u8 buf[2] = { CTRL_REG1, tf9->resume_state[RES_CTRL_REG1] & (~PC1_ON) };

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next slower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(kxtf9_odr_table); i++) {
		config[1] = kxtf9_odr_table[i].mask;
		valid_t = kxtf9_odr_table[i].cutoff;
		if (poll_interval < kxtf9_odr_table[i].cutoff)
			break;
	}
	tf9->odr_delay = valid_t;
	tf9->resume_state[RES_DATA_CTRL] = config[1];
	if (atomic_read(&tf9->enabled)) {
		if (tf9->input_dev) {
			cancel_delayed_work_sync(&tf9->input_work);
			atomic_set(&tf9->is_polling, 0);
		}
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			goto error;
		err = kxtf9_i2c_write(tf9, config, 1);
		if (err < 0) {
			buf[0] = config[0];
			buf[1] = config[1];
			goto error;
		}
		buf[1] = tf9->resume_state[RES_CTRL_REG1];
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			goto error;
		msleep(tf9->odr_delay);
		if (tf9->input_dev && !atomic_read(&tf9->is_polling)) {
			schedule_delayed_work(&tf9->input_work,
			      msecs_to_jiffies(poll_interval));
			atomic_set(&tf9->is_polling, 1);
		}
	}

	return 0;
error:
	dev_err(&tf9->client->dev, "update odr write failed 0x%x,0x%x: %d\n",
		buf[0], buf[1], err);

	return err;
}

int kxtf9_update_gesture_sensitivity(struct kxtf9_data *tf9, int index)
{
	int err = -EINVAL;
	u8 buf[8];

	if (index >= SENSITIVITY_LEVELS || index < 0)
		return err;

	tf9->resume_state[RES_TDT_TIMER] =
		tf9->ts_regs[index].reg_timer_init;
	tf9->resume_state[RES_TDT_H_THRESH] =
		tf9->ts_regs[index].reg_h_thresh_init;
	tf9->resume_state[RES_TDT_L_THRESH] =
		tf9->ts_regs[index].reg_l_thresh_init;
	tf9->resume_state[RES_TAP_TIMER] =
		tf9->ts_regs[index].reg_tap_timer_init;
	tf9->resume_state[RES_TOTAL_TIMER] =
		tf9->ts_regs[index].reg_total_timer_init;
	tf9->resume_state[RES_LAT_TIMER] =
		tf9->ts_regs[index].reg_latency_timer_init;
	tf9->resume_state[RES_WIN_TIMER] =
		tf9->ts_regs[index].reg_window_timer_init;
	if (atomic_read(&tf9->enabled)) {
		buf[0] = CTRL_REG1;
		buf[1] = tf9->resume_state[RES_CTRL_REG1] & (~PC1_ON);
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			goto error;
		buf[0] = TDT_TIMER;
		buf[1] = tf9->resume_state[RES_TDT_TIMER];
		buf[2] = tf9->resume_state[RES_TDT_H_THRESH];
		buf[3] = tf9->resume_state[RES_TDT_L_THRESH];
		buf[4] = tf9->resume_state[RES_TAP_TIMER];
		buf[5] = tf9->resume_state[RES_TOTAL_TIMER];
		buf[6] = tf9->resume_state[RES_LAT_TIMER];
		buf[7] = tf9->resume_state[RES_WIN_TIMER];
		err = kxtf9_i2c_write(tf9, buf, 7);
		if (err < 0)
			goto error;
		buf[0] = CTRL_REG1;
		buf[1] = tf9->resume_state[RES_CTRL_REG1] | PC1_ON;
		err = kxtf9_i2c_write(tf9, buf, 1);
		if (err < 0)
			goto error;
		msleep(tf9->odr_delay);
	}

	return 0;
error:
	dev_err(&tf9->client->dev,
		"gesture sensitivity write failure 0x%x,0x%x: %d\n",
		buf[0], buf[1], err);

	return err;
}

static int kxtf9_get_acceleration_data(struct kxtf9_data *tf9, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6] = { XOUT_L };

	err = kxtf9_i2c_read(tf9, acc_data, 6);
	if (err < 0) {
		dev_err(&tf9->client->dev, "accel data read failed: %d\n", err);
		return err;
	}
	xyz[0] = (int) (((acc_data[1]) << 8) | acc_data[0]);
	xyz[1] = (int) (((acc_data[3]) << 8) | acc_data[2]);
	xyz[2] = (int) (((acc_data[5]) << 8) | acc_data[4]);

	xyz[0] = (xyz[0] & 0x8000) ? (xyz[0] | 0xFFFF0000) : (xyz[0]);
	xyz[1] = (xyz[1] & 0x8000) ? (xyz[1] | 0xFFFF0000) : (xyz[1]);
	xyz[2] = (xyz[2] & 0x8000) ? (xyz[2] | 0xFFFF0000) : (xyz[2]);

	xyz[0] >>= tf9->shift_adj;
	xyz[1] >>= tf9->shift_adj;
	xyz[2] >>= tf9->shift_adj;

	return err;
}

static void kxtf9_report_values(struct kxtf9_data *tf9, int *xyz)
{
	input_report_abs(tf9->input_dev, ABS_X, xyz[0]);
	input_report_abs(tf9->input_dev, ABS_Y, xyz[1]);
	input_report_abs(tf9->input_dev, ABS_Z, xyz[2]);
	input_sync(tf9->input_dev);
}

static int kxtf9_enable(struct kxtf9_data *tf9)
{
	int err;

	if (!atomic_cmpxchg(&tf9->enabled, 0, 1)) {
		err = kxtf9_device_power_on(tf9);
		if (err < 0) {
			atomic_set(&tf9->enabled, 0);
			return err;
		}
		if ((tf9->resume_state[RES_CTRL_REG1] & TPE) > 0)
			schedule_delayed_work(&tf9->force_tilt,
				msecs_to_jiffies(TILT_REPORT_DELAY));
	}

	return 0;
}

static int kxtf9_disable(struct kxtf9_data *tf9)
{
	if (atomic_cmpxchg(&tf9->enabled, 1, 0)) {
		cancel_delayed_work_sync(&tf9->force_tilt);
		cancel_delayed_work_sync(&tf9->input_work);
		atomic_set(&tf9->is_polling, 0);
		kxtf9_device_power_off(tf9);
	}

	return 0;
}

static int kxtf9_misc_open(struct inode *inode, struct file *file)
{
	int err;

	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;
	file->private_data = kxtf9_misc_data;

	return 0;
}

static int kxtf9_misc_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	struct kxtf9_data *tf9 = file->private_data;
	int io_int;
	u8  io_u8;
	/* Initial value used to move IC to stand-by */
	u8 ctrl[2] = { CTRL_REG1,
			tf9->resume_state[RES_CTRL_REG1] & (~PC1_ON) };
	int err;

	switch (cmd) {
	case KXTF9_IOCTL_SET_DELAY:
		if (copy_from_user(&io_int, argp, sizeof(io_int)))
			return -EFAULT;
		if (io_int > tf9->pdata->min_interval)
			tf9->pdata->poll_interval = io_int;
		else
			tf9->pdata->poll_interval = tf9->pdata->min_interval;
		err = kxtf9_update_odr(tf9, tf9->pdata->poll_interval);
		if (err < 0)
			return err;
		break;
	case KXTF9_IOCTL_GET_DELAY:
		io_int = tf9->pdata->poll_interval;
		if (copy_to_user(argp, &io_int, sizeof(io_int)))
			return -EFAULT;
		break;
	case KXTF9_IOCTL_SET_ENABLE:
		if (copy_from_user(&io_int, argp, sizeof(io_int)))
			return -EFAULT;
		if (io_int < 0 || io_int > 1)
			return -EINVAL;
		if (io_int)
			kxtf9_enable(tf9);
		else
			kxtf9_disable(tf9);
		break;
	case KXTF9_IOCTL_GET_ENABLE:
		io_int = atomic_read(&tf9->enabled);
		if (copy_to_user(argp, &io_int, sizeof(io_int)))
			return -EFAULT;
		break;
	case KXTF9_IOCTL_SET_G_RANGE:
		if (copy_from_user(&io_u8, argp, sizeof(io_u8)))
			return -EFAULT;
		err = kxtf9_update_g_range(tf9, io_u8);
		if (err < 0)
			return err;
		break;
	case KXTF9_IOCTL_SET_TILT_ENABLE: /* Overlapped set functionality */
		io_u8 = TPE;
		goto process_set_x;
	case KXTF9_IOCTL_SET_TAP_ENABLE:  /* Overlapped set functionality */
		io_u8 = TDTE;
		goto process_set_x;
	case KXTF9_IOCTL_SET_WAKE_ENABLE: /* Overlapped set functionality */
		io_u8 = WUFE;
process_set_x:
		if (copy_from_user(&io_int, argp, sizeof(io_int)))
			return -EFAULT;
		if (io_int == 1)
			tf9->resume_state[RES_CTRL_REG1] |= io_u8;
		else if (io_int == 0)
			tf9->resume_state[RES_CTRL_REG1] &= ~io_u8;
		else
			return -EINVAL;
		err = 0;
		if (atomic_read(&tf9->enabled)) {
			err = kxtf9_i2c_write(tf9, ctrl, 1);
			if (err < 0)
				goto set_x_error;
			ctrl[1] = tf9->resume_state[RES_CTRL_REG1] | PC1_ON;
			err = kxtf9_i2c_write(tf9, ctrl, 1);
			if (err < 0)
				goto set_x_error;
			msleep(tf9->odr_delay);
			if ((io_u8 == TPE) && (io_int == 1))
				schedule_delayed_work(&tf9->force_tilt,
					msecs_to_jiffies(TILT_REPORT_DELAY));
			ctrl[0] = INT_REL;
			err = kxtf9_i2c_read(tf9, ctrl, 1);
		}
		if (err < 0) {
set_x_error:
			dev_err(&tf9->client->dev,
				"set 0x%x enable error: 0x%x,0x%x: %d\n",
				io_u8, ctrl[0], ctrl[1], err);
			return err;
		}
		break;
	case KXTF9_IOCTL_SET_SENSITIVITY:
		if (copy_from_user(&io_int, argp, sizeof(io_int)))
			return -EFAULT;
		err = kxtf9_update_gesture_sensitivity(tf9, io_int - 1);
		if (err < 0)
			return err;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations kxtf9_misc_fops = {
	.owner = THIS_MODULE,
	.open = kxtf9_misc_open,
	.ioctl = kxtf9_misc_ioctl,
};

static struct miscdevice kxtf9_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &kxtf9_misc_fops,
};

static void kxtf9_input_work_func(struct work_struct *work)
{
	struct kxtf9_data *tf9 = container_of((struct delayed_work *)work,
						struct kxtf9_data, input_work);
	int xyz[3] = { 0 };

	if (kxtf9_get_acceleration_data(tf9, xyz) == 0)
		kxtf9_report_values(tf9, xyz);
	schedule_delayed_work(&tf9->input_work,
			      msecs_to_jiffies(tf9->pdata->poll_interval));
}

static void kxtf9_force_tilt_func(struct work_struct *work)
{
	struct kxtf9_data *tf9 = container_of((struct delayed_work *)work,
						struct kxtf9_data, force_tilt);
	int err = -1;
	unsigned int tilt = KXTF9_INT_TILT;
	u8 reg = TILT_POS_CUR;

	err = kxtf9_i2c_read(tf9, &reg, 1);
	if (err < 0) {
		dev_err(&tf9->client->dev, "tilt read error: %d\n", err);
	} else {
		tilt |= reg | (0xFF << 8);
		tilt |= (tf9->pdata->gesture++ & 1) << 31;
		input_report_abs(tf9->input_dev, ABS_MISC, tilt);
		input_sync(tf9->input_dev);
	}
}

#ifdef KXTF9_OPEN_ENABLE
int kxtf9_input_open(struct input_dev *input)
{
	struct kxtf9_data *tf9 = input_get_drvdata(input);

	return kxtf9_enable(tf9);
}

void kxtf9_input_close(struct input_dev *dev)
{
	struct kxtf9_data *tf9 = input_get_drvdata(dev);

	kxtf9_disable(tf9);
}
#endif

static int kxtf9_validate_pdata(struct kxtf9_data *tf9)
{
	if (tf9->pdata->min_interval > tf9->pdata->poll_interval) {
		tf9->pdata->poll_interval = tf9->pdata->min_interval;
		dev_info(&tf9->client->dev,
			"enforcing minimum interval\n");
	}

	return 0;
}

static int kxtf9_input_init(struct kxtf9_data *tf9)
{
	int err;

	INIT_DELAYED_WORK(&tf9->input_work, kxtf9_input_work_func);
	tf9->input_dev = input_allocate_device();
	if (!tf9->input_dev) {
		err = -ENOMEM;
		dev_err(&tf9->client->dev,
			"input device allocate failed: %d\n", err);
		goto err0;
	}
#ifdef kxtf9_OPEN_ENABLE
	tf9->input_dev->open = kxtf9_input_open;
	tf9->input_dev->close = kxtf9_input_close;
#endif
	input_set_drvdata(tf9->input_dev, tf9);

	set_bit(EV_ABS, tf9->input_dev->evbit);
	set_bit(ABS_MISC, tf9->input_dev->absbit);
	input_set_abs_params(tf9->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);

	tf9->input_dev->name = "accelerometer";
	err = input_register_device(tf9->input_dev);
	if (err) {
		dev_err(&tf9->client->dev,
			"unable to register input polled device %s: %d\n",
			tf9->input_dev->name, err);
		goto err1;
	}
	if ((tf9->resume_state[RES_CTRL_REG1] & TPE) > 0)
		schedule_delayed_work(&tf9->force_tilt,
			msecs_to_jiffies(TILT_REPORT_DELAY));

	return 0;
err1:
	input_free_device(tf9->input_dev);
err0:
	return err;
}

static void kxtf9_sensitivity_init(struct kxtf9_data *tf9)
{
	int buf_size = 0;

	buf_size = sizeof(tf9->pdata->sensitivity_low);
	memcpy(tf9->ts_regs + SENSITIVITY_LOW_OFFSET,
			tf9->pdata->sensitivity_low, buf_size);
	buf_size = sizeof(tf9->pdata->sensitivity_medium);
	memcpy(tf9->ts_regs + SENSITIVITY_MEDIUM_OFFSET,
			tf9->pdata->sensitivity_medium, buf_size);
	buf_size = sizeof(tf9->pdata->sensitivity_high);
	memcpy(tf9->ts_regs + SENSITIVITY_HIGH_OFFSET,
			tf9->pdata->sensitivity_high, buf_size);
}

static void kxtf9_input_cleanup(struct kxtf9_data *tf9)
{
	input_unregister_device(tf9->input_dev);
}

static int kxtf9_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct kxtf9_data *tf9;
	int err = -1;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL, exiting\n");
		err = -ENODEV;
		goto err0;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}
	tf9 = kzalloc(sizeof(*tf9), GFP_KERNEL);
	if (tf9 == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for module data: %d\n", err);
		goto err0;
	}

	tf9->client = client;

	INIT_WORK(&tf9->irq_work, kxtf9_irq_work_func);
	tf9->irq_work_queue = create_singlethread_workqueue("kxtf9_wq");
	if (!tf9->irq_work_queue) {
		err = -ENOMEM;
		dev_err(&client->dev, "cannot create work queue: %d\n", err);
		goto err1;
	}
	tf9->pdata = kzalloc(sizeof(*tf9->pdata), GFP_KERNEL);
	if (tf9->pdata == NULL) {
		err = -ENOMEM;
		dev_err(&client->dev,
			"failed to allocate memory for pdata: %d\n", err);
		goto err2;
	}
	memcpy(tf9->pdata, client->dev.platform_data, sizeof(*tf9->pdata));
	err = kxtf9_validate_pdata(tf9);
	if (err < 0) {
		dev_err(&client->dev,
			"failed to validate platform data: %d\n", err);
		goto err3;
	}
	i2c_set_clientdata(client, tf9);
	if (tf9->pdata->init) {
		err = tf9->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto err3;
		}
	}

	kxtf9_sensitivity_init(tf9);
	tf9->irq = gpio_to_irq(tf9->pdata->gpio);
	tf9->resume_state[RES_DATA_CTRL]    = tf9->pdata->data_odr_init;
	tf9->resume_state[RES_CTRL_REG1]    = tf9->pdata->ctrl_reg1_init;
	tf9->resume_state[RES_INT_CTRL1]    = tf9->pdata->int_ctrl_init;
	tf9->resume_state[RES_TILT_TIMER]   = tf9->pdata->tilt_timer_init;
	tf9->resume_state[RES_CTRL_REG3]    = tf9->pdata->engine_odr_init;
	tf9->resume_state[RES_WUF_TIMER]    = tf9->pdata->wuf_timer_init;
	tf9->resume_state[RES_WUF_THRESH]   = tf9->pdata->wuf_thresh_init;
	tf9->resume_state[RES_TDT_TIMER]    = tf9->pdata->tdt_timer_init;
	tf9->resume_state[RES_TDT_H_THRESH] = tf9->pdata->tdt_h_thresh_init;
	tf9->resume_state[RES_TDT_L_THRESH] = tf9->pdata->tdt_l_thresh_init;
	tf9->resume_state[RES_TAP_TIMER]    = tf9->pdata->tdt_tap_timer_init;
	tf9->resume_state[RES_TOTAL_TIMER]  = tf9->pdata->tdt_total_timer_init;
	tf9->resume_state[RES_LAT_TIMER]   = tf9->pdata->tdt_latency_timer_init;
	tf9->resume_state[RES_WIN_TIMER]    = tf9->pdata->tdt_window_timer_init;

	INIT_DELAYED_WORK(&tf9->force_tilt, kxtf9_force_tilt_func);

	err = kxtf9_device_power_on(tf9);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto err4;
	}
	atomic_set(&tf9->enabled, 1);

	err = kxtf9_update_g_range(tf9, tf9->pdata->g_range);
	if (err < 0)
		goto err5;
	err = kxtf9_update_odr(tf9, tf9->pdata->poll_interval);
	if (err < 0)
		goto err5;
	err = kxtf9_input_init(tf9);
	if (err < 0)
		goto err5;

	kxtf9_misc_data = tf9;
	err = misc_register(&kxtf9_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "misc register failed: %d\n", err);
		goto err6;
	}

	kxtf9_device_power_off(tf9);
	atomic_set(&tf9->enabled, 0);

	err = request_irq(tf9->irq, kxtf9_isr, IRQF_TRIGGER_RISING,
		"kxtf9_irq", tf9);
	if (err < 0) {
		dev_err(&client->dev, "request irq failed: %d\n", err);
		goto err7;
	}
	disable_irq_nosync(tf9->irq);

#ifdef CONFIG_HAS_EARLYSUSPEND
	tf9->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	tf9->early_suspend.suspend = kxtf9_early_suspend;
	tf9->early_suspend.resume = kxtf9_late_resume;
	register_early_suspend(&tf9->early_suspend);
#endif

	dev_info(&client->dev, "kxtf9 probed\n");

	return 0;

err7:
	misc_deregister(&kxtf9_misc_device);
err6:
	kxtf9_input_cleanup(tf9);
err5:
	kxtf9_device_power_off(tf9);
err4:
	if (tf9->pdata->exit)
		tf9->pdata->exit();
err3:
	kfree(tf9->pdata);
err2:
	destroy_workqueue(tf9->irq_work_queue);
err1:
	kfree(tf9);
err0:
	return err;
}

static int __devexit kxtf9_remove(struct i2c_client *client)
{
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);

	unregister_early_suspend(&tf9->early_suspend);
	free_irq(tf9->irq, tf9);
	gpio_free(tf9->pdata->gpio);
	misc_deregister(&kxtf9_misc_device);
	kxtf9_input_cleanup(tf9);
	kxtf9_device_power_off(tf9);
	if (tf9->pdata->exit)
		tf9->pdata->exit();
	kfree(tf9->pdata);
	destroy_workqueue(tf9->irq_work_queue);
	kfree(tf9);

	return 0;
}

static int kxtf9_resume(struct i2c_client *client)
{
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);

	return kxtf9_enable(tf9);
}

static int kxtf9_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct kxtf9_data *tf9 = i2c_get_clientdata(client);

	return kxtf9_disable(tf9);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void kxtf9_early_suspend(struct early_suspend *handler)
{
	struct kxtf9_data *tf9;
	tf9 = container_of(handler, struct kxtf9_data, early_suspend);

	tf9->was_polling_at_suspend = atomic_read(&tf9->is_polling);
	kxtf9_suspend(tf9->client, PMSG_SUSPEND);
}

static void kxtf9_late_resume(struct early_suspend *handler)
{
	struct kxtf9_data *tf9;
	int err = -1;
	tf9 = container_of(handler, struct kxtf9_data, early_suspend);

	kxtf9_resume(tf9->client);
	if (atomic_read(&tf9->enabled) && tf9->was_polling_at_suspend) {
		err = kxtf9_update_odr(tf9, tf9->pdata->poll_interval);
		if (err < 0)
			dev_err(&tf9->client->dev,
				"odr kickoff failed: %d\n", err);
	}
}
#endif

static const struct i2c_device_id kxtf9_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, kxtf9_id);

static struct i2c_driver kxtf9_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = kxtf9_probe,
	.remove = __devexit_p(kxtf9_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND
	.resume = kxtf9_resume,
	.suspend = kxtf9_suspend,
#endif
	.id_table = kxtf9_id,
};

static int __init kxtf9_init(void)
{
	pr_info("kxtf9 accelerometer driver\n");
	return i2c_add_driver(&kxtf9_driver);
}

static void __exit kxtf9_exit(void)
{
	i2c_del_driver(&kxtf9_driver);
	return;
}

module_init(kxtf9_init);
module_exit(kxtf9_exit);

MODULE_DESCRIPTION("KXTF9 accelerometer driver");
MODULE_AUTHOR("Kionix");
MODULE_LICENSE("GPL");
