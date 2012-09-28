/*
 * Copyright (C) 2009 Motorola, Inc.
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

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/suspend.h>

#include <linux/lis331dlh.h>

#define NAME				"lis331dlh"

/** Maximum polled-device-reported g value */
#define G_MAX			8000

#define SHIFT_ADJ_2G		4
#define SHIFT_ADJ_4G		3
#define SHIFT_ADJ_8G		2

#define AXISDATA_REG		0x28

/* ctrl 1: pm2 pm1 pm0 dr1 dr0 zenable yenable zenable */
#define CTRL_REG1		0x20	/* power control reg */
#define CTRL_REG2		0x21	/* power control reg */
#define CTRL_REG3		0x22	/* power control reg */
#define CTRL_REG4		0x23	/* interrupt control reg */

#define PM_OFF          	0x00
#define PM_NORMAL       	0x20
#define ENABLE_ALL_AXES 	0x07

#define ODRHALF         	0x40	/* 0.5Hz output data rate */
#define ODR1            	0x60	/* 1Hz output data rate */
#define ODR2            	0x80	/* 2Hz output data rate */
#define ODR5            	0xA0	/* 5Hz output data rate */
#define ODR10           	0xC0	/* 10Hz output data rate */
#define ODR50           	0x00	/* 50Hz output data rate */
#define ODR100          	0x08	/* 100Hz output data rate */
#define ODR400          	0x10	/* 400Hz output data rate */
#define ODR1000         	0x18	/* 1000Hz output data rate */

#define FUZZ			32
#define FLAT			32
#define I2C_RETRY_DELAY		5
#define I2C_RETRIES		5
#define AUTO_INCREMENT		0x80

static struct {
	unsigned int cutoff;
	unsigned int mask;
} odr_table[] = {
	{
	3,	PM_NORMAL | ODR1000}, {
	10,	PM_NORMAL | ODR400}, {
	20,	PM_NORMAL | ODR100}, {
	100,	PM_NORMAL | ODR50}, {
	200,	ODR1000	| ODR10}, {
	500,	ODR1000 | ODR5}, {
	1000,	ODR1000 | ODR2}, {
	2000,	ODR1000 | ODR1}, {
	0,	ODR1000 | ODRHALF},};

struct lis331dlh_data {
	struct i2c_client *client;
	struct lis331dlh_platform_data *pdata;

	struct mutex lock;

	struct delayed_work input_work;
	struct input_dev *input_dev;

	int hw_initialized;
	atomic_t enabled;
	int on_before_suspend;

	u8 shift_adj;
	u8 resume_state[5];
};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct lis331dlh_data *lis331dlh_misc_data;

static struct notifier_block lis331dlh_pm_notifier;

static int lis331dlh_i2c_read(struct lis331dlh_data *lis, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = lis->client->addr,
		 .flags = lis->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = lis->client->addr,
		 .flags = (lis->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(lis->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&lis->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis331dlh_i2c_write(struct lis331dlh_data *lis, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = lis->client->addr,
		 .flags = lis->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(lis->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&lis->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int lis331dlh_hw_init(struct lis331dlh_data *lis)
{
	int err = -1;
	u8 buf[6];

	buf[0] = (AUTO_INCREMENT | CTRL_REG1);
	buf[1] = lis->resume_state[0];
	buf[2] = lis->resume_state[1];
	buf[3] = lis->resume_state[2];
	buf[4] = lis->resume_state[3];
	buf[5] = lis->resume_state[4];
	err = lis331dlh_i2c_write(lis, buf, 5);
	if (err < 0)
		return err;

	lis->hw_initialized = 1;

	return 0;
}

static void lis331dlh_device_power_off(struct lis331dlh_data *lis)
{
	int err;
	u8 buf[2] = { CTRL_REG4, PM_OFF };

	err = lis331dlh_i2c_write(lis, buf, 1);
	if (err < 0)
		dev_err(&lis->client->dev, "soft power off failed\n");

	if (lis->pdata->power_off) {
		lis->pdata->power_off();
		lis->hw_initialized = 0;
	}
}

static int lis331dlh_device_power_on(struct lis331dlh_data *lis)
{
	int err;

	if (lis->pdata->power_on) {
		err = lis->pdata->power_on();
		if (err < 0)
			return err;
	}

	if (!lis->hw_initialized) {
		err = lis331dlh_hw_init(lis);
		if (err < 0) {
			lis331dlh_device_power_off(lis);
			return err;
		}
	}

	return 0;
}

int lis331dlh_update_g_range(struct lis331dlh_data *lis, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 buf[2];

	switch (new_g_range) {
	case LIS331DLH_G_2G:
		shift = SHIFT_ADJ_2G;
		break;
	case LIS331DLH_G_4G:
		shift = SHIFT_ADJ_4G;
		break;
	case LIS331DLH_G_8G:
		shift = SHIFT_ADJ_8G;
		break;
	default:
		return -EINVAL;
	}

	if (atomic_read(&lis->enabled)) {
		/* Set configuration register 4, which contains g range setting
		 *  NOTE: this is a straight overwrite because this driver does
		 *  not use any of the other configuration bits in this
		 *  register.  Should this become untrue, we will have to read
		 *  out the value and only change the relevant bits --XX----
		 *  (marked by X) */
		buf[0] = CTRL_REG4;
		buf[1] = new_g_range;
		err = lis331dlh_i2c_write(lis, buf, 1);
		if (err < 0)
			return err;
	}

	lis->resume_state[3] = new_g_range;
	lis->shift_adj = shift;

	return 0;
}

int lis331dlh_update_odr(struct lis331dlh_data *lis, int poll_interval)
{
	int err = -1;
	int i;
	u8 config[2];

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next lower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(odr_table); i++) {
		config[1] = odr_table[i].mask;
		if (poll_interval < odr_table[i].cutoff)
			break;
	}

	config[1] |= ENABLE_ALL_AXES;

	/* If device is currently enabled, we need to write new
	 *  configuration out to it */
	if (atomic_read(&lis->enabled)) {
		config[0] = CTRL_REG1;
		err = lis331dlh_i2c_write(lis, config, 1);
		if (err < 0)
			return err;
	}

	lis->resume_state[0] = config[1];

	return 0;
}

static int lis331dlh_get_acceleration_data(struct lis331dlh_data *lis, int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware data */
	int hw_d[3] = { 0 };

	acc_data[0] = (AUTO_INCREMENT | AXISDATA_REG);
	err = lis331dlh_i2c_read(lis, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (int) (((acc_data[1]) << 8) | acc_data[0]);
	hw_d[1] = (int) (((acc_data[3]) << 8) | acc_data[2]);
	hw_d[2] = (int) (((acc_data[5]) << 8) | acc_data[4]);

	hw_d[0] = (hw_d[0] & 0x8000) ? (hw_d[0] | 0xFFFF0000) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? (hw_d[1] | 0xFFFF0000) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? (hw_d[2] | 0xFFFF0000) : (hw_d[2]);

	hw_d[0] >>= lis->shift_adj;
	hw_d[1] >>= lis->shift_adj;
	hw_d[2] >>= lis->shift_adj;

	xyz[0] = ((lis->pdata->negate_x) ? (-hw_d[lis->pdata->axis_map_x])
		  : (hw_d[lis->pdata->axis_map_x]));
	xyz[1] = ((lis->pdata->negate_y) ? (-hw_d[lis->pdata->axis_map_y])
		  : (hw_d[lis->pdata->axis_map_y]));
	xyz[2] = ((lis->pdata->negate_z) ? (-hw_d[lis->pdata->axis_map_z])
		  : (hw_d[lis->pdata->axis_map_z]));

	return err;
}

static void lis331dlh_report_values(struct lis331dlh_data *lis, int *xyz)
{
	input_report_abs(lis->input_dev, ABS_X, xyz[0]);
	input_report_abs(lis->input_dev, ABS_Y, xyz[1]);
	input_report_abs(lis->input_dev, ABS_Z, xyz[2]);
	input_sync(lis->input_dev);
}

static int lis331dlh_enable(struct lis331dlh_data *lis)
{
	int err;

	if (!atomic_cmpxchg(&lis->enabled, 0, 1)) {

		err = lis331dlh_device_power_on(lis);
		if (err < 0) {
			atomic_set(&lis->enabled, 0);
			return err;
		}
		schedule_delayed_work(&lis->input_work,
				      msecs_to_jiffies(lis->
						       pdata->poll_interval));
	}

	return 0;
}

static int lis331dlh_disable(struct lis331dlh_data *lis)
{
	if (atomic_cmpxchg(&lis->enabled, 1, 0)) {
		cancel_delayed_work_sync(&lis->input_work);
		lis331dlh_device_power_off(lis);
	}

	return 0;
}

static int lis331dlh_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = lis331dlh_misc_data;

	return 0;
}

static int lis331dlh_misc_ioctl(struct inode *inode, struct file *file,
				unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 buf[4];
	int err;
	int interval;
	struct lis331dlh_data *lis = file->private_data;

	switch (cmd) {
	case LIS331DLH_IOCTL_GET_DELAY:
		interval = lis->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case LIS331DLH_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 200)
			return -EINVAL;

		lis->pdata->poll_interval =
		    max(interval, lis->pdata->min_interval);
		err = lis331dlh_update_odr(lis, lis->pdata->poll_interval);
		/* TODO: if update fails poll is still set */
		if (err < 0)
			return err;

		break;

	case LIS331DLH_IOCTL_SET_ENABLE:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval > 1)
			return -EINVAL;

		if (interval)
			lis331dlh_enable(lis);
		else
			lis331dlh_disable(lis);

		break;

	case LIS331DLH_IOCTL_GET_ENABLE:
		interval = atomic_read(&lis->enabled);
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;

		break;

	case LIS331DLH_IOCTL_SET_G_RANGE:
		if (copy_from_user(&buf, argp, 1))
			return -EFAULT;
		err = lis331dlh_update_g_range(lis, arg);
		if (err < 0)
			return err;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations lis331dlh_misc_fops = {
	.owner = THIS_MODULE,
	.open = lis331dlh_misc_open,
	.ioctl = lis331dlh_misc_ioctl,
};

static struct miscdevice lis331dlh_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &lis331dlh_misc_fops,
};

static void lis331dlh_input_work_func(struct work_struct *work)
{
	struct lis331dlh_data *lis = container_of((struct delayed_work *)work,
						  struct lis331dlh_data,
						  input_work);
	int xyz[3] = { 0 };
	int err;

	mutex_lock(&lis->lock);
	err = lis331dlh_get_acceleration_data(lis, xyz);
	if (err < 0)
		dev_err(&lis->client->dev, "get_acceleration_data failed\n");
	else
		lis331dlh_report_values(lis, xyz);

	schedule_delayed_work(&lis->input_work,
			      msecs_to_jiffies(lis->pdata->poll_interval));
	mutex_unlock(&lis->lock);
}

#ifdef LIS331DLH_OPEN_ENABLE
int lis331dlh_input_open(struct input_dev *input)
{
	struct lis331dlh_data *lis = input_get_drvdata(input);

	return lis331dlh_enable(lis);
}

void lis331dlh_input_close(struct input_dev *dev)
{
	struct lis331dlh_data *lis = input_get_drvdata(dev);

	lis331dlh_disable(lis);
}
#endif

static int lis331dlh_validate_pdata(struct lis331dlh_data *lis)
{
	lis->pdata->poll_interval = max(lis->pdata->poll_interval,
					lis->pdata->min_interval);

	if (lis->pdata->axis_map_x > 2 ||
	    lis->pdata->axis_map_y > 2 || lis->pdata->axis_map_z > 2) {
		dev_err(&lis->client->dev,
			"invalid axis_map value x:%u y:%u z%u\n",
			lis->pdata->axis_map_x, lis->pdata->axis_map_y,
			lis->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (lis->pdata->negate_x > 1 || lis->pdata->negate_y > 1 ||
	    lis->pdata->negate_z > 1) {
		dev_err(&lis->client->dev,
			"invalid negate value x:%u y:%u z:%u\n",
			lis->pdata->negate_x, lis->pdata->negate_y,
			lis->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (lis->pdata->poll_interval < lis->pdata->min_interval) {
		dev_err(&lis->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis331dlh_input_init(struct lis331dlh_data *lis)
{
	int err;

	INIT_DELAYED_WORK(&lis->input_work, lis331dlh_input_work_func);

	lis->input_dev = input_allocate_device();
	if (!lis->input_dev) {
		err = -ENOMEM;
		dev_err(&lis->client->dev, "input device allocate failed\n");
		goto err0;
	}

#ifdef LIS331DLH_OPEN_ENABLE
	lis->input_dev->open = lis331dlh_input_open;
	lis->input_dev->close = lis331dlh_input_close;
#endif

	input_set_drvdata(lis->input_dev, lis);

	set_bit(EV_ABS, lis->input_dev->evbit);

	input_set_abs_params(lis->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(lis->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(lis->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	lis->input_dev->name = "accelerometer";

	err = input_register_device(lis->input_dev);
	if (err) {
		dev_err(&lis->client->dev,
			"unable to register input polled device %s\n",
			lis->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(lis->input_dev);
err0:
	return err;
}

static void lis331dlh_input_cleanup(struct lis331dlh_data *lis)
{
	input_unregister_device(lis->input_dev);
}

static int lis331dlh_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct lis331dlh_data *lis;
	int err = -1;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}

	lis = kzalloc(sizeof(*lis), GFP_KERNEL);
	if (lis == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&lis->lock);
	mutex_lock(&lis->lock);
	lis->client = client;

	lis->pdata = kmalloc(sizeof(*lis->pdata), GFP_KERNEL);
	if (lis->pdata == NULL)
		goto err1;

	memcpy(lis->pdata, client->dev.platform_data, sizeof(*lis->pdata));

	err = lis331dlh_validate_pdata(lis);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, lis);

	if (lis->pdata->init) {
		err = lis->pdata->init();
		if (err < 0)
			goto err1_1;
	}

	memset(lis->resume_state, 0, ARRAY_SIZE(lis->resume_state));

	lis->resume_state[0] = 7;
	lis->resume_state[1] = 0;
	lis->resume_state[2] = 0;
	lis->resume_state[3] = 0;
	lis->resume_state[4] = 0;

	err = lis331dlh_device_power_on(lis);
	if (err < 0)
		goto err2;

	atomic_set(&lis->enabled, 1);

	err = lis331dlh_update_g_range(lis, lis->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto err2;
	}

	err = lis331dlh_update_odr(lis, lis->pdata->poll_interval);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto err2;
	}

	err = lis331dlh_input_init(lis);
	if (err < 0)
		goto err3;

	lis331dlh_misc_data = lis;

	err = misc_register(&lis331dlh_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "lisd_device register failed\n");
		goto err4;
	}

	lis331dlh_device_power_off(lis);

	register_pm_notifier(&lis331dlh_pm_notifier);

	/* As default, do not report information */
	atomic_set(&lis->enabled, 0);

	mutex_unlock(&lis->lock);

	dev_info(&client->dev, "lis331dlh probed\n");

	return 0;

err4:
	lis331dlh_input_cleanup(lis);
err3:
	lis331dlh_device_power_off(lis);
err2:
	if (lis->pdata->exit)
		lis->pdata->exit();
err1_1:
	mutex_unlock(&lis->lock);
	kfree(lis->pdata);
err1:
	kfree(lis);
err0:
	return err;
}

static int __devexit lis331dlh_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct lis331dlh_data *lis = i2c_get_clientdata(client);

	misc_deregister(&lis331dlh_misc_device);
	lis331dlh_input_cleanup(lis);
	lis331dlh_device_power_off(lis);
	if (lis->pdata->exit)
		lis->pdata->exit();
	kfree(lis->pdata);
	kfree(lis);

	return 0;
}

static const struct i2c_device_id lis331dlh_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, lis331dlh_id);

static int lis331dlh_resume(void)
{
	if (lis331dlh_misc_data->on_before_suspend)
		return lis331dlh_enable(lis331dlh_misc_data);
	return 0;
}

static int lis331dlh_suspend(void)
{
	lis331dlh_misc_data->on_before_suspend =
		atomic_read(&lis331dlh_misc_data->enabled);
	return lis331dlh_disable(lis331dlh_misc_data);
}

static int lis331dlh_pm_event(struct notifier_block *this, unsigned long event,
				void *ptr)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		lis331dlh_suspend();
		break;
	case PM_POST_SUSPEND:
		lis331dlh_resume();
	}
	return NOTIFY_DONE;
}

static struct notifier_block lis331dlh_pm_notifier = {
	.notifier_call = lis331dlh_pm_event,
};

static struct i2c_driver lis331dlh_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = lis331dlh_probe,
	.remove = __devexit_p(lis331dlh_remove),
	.id_table = lis331dlh_id,
};

static int __init lis331dlh_init(void)
{
	pr_info(KERN_INFO "LIS331DLH accelerometer driver\n");
	return i2c_add_driver(&lis331dlh_driver);
}

static void __exit lis331dlh_exit(void)
{
	i2c_del_driver(&lis331dlh_driver);
	return;
}

module_init(lis331dlh_init);
module_exit(lis331dlh_exit);

MODULE_DESCRIPTION("lis331dlh accelerometer driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
