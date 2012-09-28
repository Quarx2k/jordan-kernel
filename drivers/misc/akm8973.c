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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/akm8973.h>

#define NAME "akm8973"

#define FUZZ			2
#define FLAT			2
#define I2C_RETRIES		5
#define I2C_RETRY_DELAY		5

#define ENABLED			0x1
#define BUSY			0x2
#define ENABLED_BEFORE_SUSPEND	0x4

struct akm8973_data {
	struct i2c_client *client;
	struct akm8973_platform_data *pdata;

	struct work_struct irq_work;

	struct delayed_work input_work;
	struct input_dev *input_dev;

	int hw_initialized;
	unsigned long flags;

	u8 hxga;
	u8 hyga;
	u8 hzga;

	int state;
};

/*
 * Because misc devices can not carry a pointer from driver register to
 * open, we keep this global.  This limits the driver to a single instance.
 */
struct akm8973_data *akm8973_misc_data;

static inline u8 akm8973_convert_dac_offset(u8 offset)
{
	if (offset < 0x80)
		offset = 0x7f - offset;

	return offset;
}

static int akm8973_convert_adc_offset(u8 offset, u8 value)
{
	if (offset >= 0x80)
		offset = offset - 0x01;

	/* (((steps * 15.625(uT/step) * 1000(uT/nT))
			- (15.625(uT/step) * 1000(uT/nT) * 127))
				+ ((value(uT) - 128(uT)) * 1000(uT/nT))) */
	return ((offset * 15625) - 1984375) +
			((value - 128) * 1000);
}

static int akm8973_i2c_read(struct akm8973_data *akm, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = akm->client->addr,
		 .flags = akm->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = buf,
		 },
		{
		 .addr = akm->client->addr,
		 .flags = (akm->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(akm->client->adapter, msgs, 2);
		if (err != 2)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 2) && (++tries < I2C_RETRIES));

	if (err != 2) {
		dev_err(&akm->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int akm8973_i2c_write(struct akm8973_data *akm, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = akm->client->addr,
		 .flags = akm->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(akm->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&akm->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int akm8973_clear_irq(struct akm8973_data *akm)
{
	u8 buf[4];
	int err;

	buf[0] = AKM8973_REG_TMPS;
	err = akm8973_i2c_read(akm, buf, 4);
	if (err < 0)
		return err;

	return 0;
}

static int akm8973_set_mode(struct akm8973_data *akm, u8 mode)
{
	int err;
	u8 buf[2];

	/*
	 * the AKM8973 will not transition into measure mode
	 * when an irq is pending
	 */
	if (mode == AKM8973_MODE_MEASURE) {
		err = akm8973_clear_irq(akm);
		if (err < 0)
			return err;
	}

	buf[0] = AKM8973_REG_MS1;
	buf[1] = mode;

	return akm8973_i2c_write(akm, buf, 1);
}

static int akm8973_hw_init(struct akm8973_data *akm)
{
	u8 buf[7];
	int err;

	err = akm8973_set_mode(akm, AKM8973_MODE_EEPROM_READ);
	if (err < 0)
		return err;

	buf[0] = AKM8973_REG_EHXGA;
	err = akm8973_i2c_read(akm, buf, 3);
	if (err < 0) {
		akm8973_set_mode(akm, AKM8973_MODE_POWERDOWN);
		return err;
	}

	err = akm8973_set_mode(akm, AKM8973_MODE_POWERDOWN);
	if (err < 0)
		return err;

	akm->hxga = buf[0];
	akm->hyga = buf[1];
	akm->hzga = buf[2];

	buf[0] = AKM8973_REG_HXDA;
	buf[1] = akm8973_convert_dac_offset(akm->pdata->hxda);
	buf[2] = akm8973_convert_dac_offset(akm->pdata->hyda);
	buf[3] = akm8973_convert_dac_offset(akm->pdata->hzda);
	buf[4] = akm->hxga;
	buf[5] = akm->hyga;
	buf[6] = akm->hzga;

	err = akm8973_i2c_write(akm, buf, 6);
	if (err < 0)
		return err;

	akm->hw_initialized = 1;
	return 0;
}

static void akm8973_device_power_off(struct akm8973_data *akm)
{
	if (akm->pdata->power_off) {
		disable_irq_nosync(akm->client->irq);
		akm->pdata->power_off();
		clear_bit(BUSY, &akm->flags);
		akm->hw_initialized = 0;
	}
}

static int akm8973_device_power_on(struct akm8973_data *akm)
{
	int err;

	if (akm->pdata->power_on) {
		err = akm->pdata->power_on();
		if (err < 0)
			return err;
		enable_irq(akm->client->irq);
	}

	if (!akm->hw_initialized) {
		err = akm8973_hw_init(akm);
		if (err < 0) {
			akm8973_device_power_off(akm);
			return err;
		}
	}

	return 0;
}

/* returns 1 if calibration was needed */
static int akm8973_auto_calibrate_axis(struct akm8973_data *akm,
				       u8 *offset, u8 value)
{
	int off = *offset;
	int calibrate = 0;

	if (value < akm->pdata->cal_min_threshold) {
		off++;
		calibrate = 1;
	}

	if (value > akm->pdata->cal_max_threshold) {
		off--;
		calibrate = 1;
	}

	*offset = clamp(off, 0x0, 0xff);

	return calibrate;
}

/* returns 1 if calibration was needed */
static int akm8973_auto_calibrate(struct akm8973_data *akm, u8 *values)
{
	u8 buf[4];
	int err;
	int calibrate = 0;
	u8 hxda = akm->pdata->hxda;
	u8 hyda = akm->pdata->hyda;
	u8 hzda = akm->pdata->hzda;

	if (akm8973_auto_calibrate_axis(akm, &hxda, values[1]))
		calibrate = 1;
	if (akm8973_auto_calibrate_axis(akm, &hyda, values[2]))
		calibrate = 1;
	if (akm8973_auto_calibrate_axis(akm, &hzda, values[3]))
		calibrate = 1;

	if (calibrate) {
		buf[0] = AKM8973_REG_HXDA;
		buf[1] = akm8973_convert_dac_offset(hxda);
		buf[2] = akm8973_convert_dac_offset(hyda);
		buf[3] = akm8973_convert_dac_offset(hzda);

		err = akm8973_i2c_write(akm, buf, 3);
		if (err < 0) {
			dev_err(&akm->client->dev,
				"unable to update offset dacs\n");
			return 0;
		}

		akm->pdata->hxda = hxda;
		akm->pdata->hyda = hyda;
		akm->pdata->hzda = hzda;
	}

	return calibrate;
}

static void akm8973_transform_values(struct akm8973_data *akm,
		u8 *values, int *ivalues)
{
	int tmp;

	/* values = {t,x,y,z} */
	ivalues[0] = values[0];
	ivalues[1] = akm8973_convert_adc_offset(akm->pdata->hxda, values[1]);
	ivalues[2] = akm8973_convert_adc_offset(akm->pdata->hyda, values[2]);
	ivalues[3] = akm8973_convert_adc_offset(akm->pdata->hzda, values[3]);

	/* rotation over x axis */
	if (akm->pdata->xy_swap)
		ivalues[1] = -ivalues[1];

	if (akm->pdata->z_flip)
		ivalues[3] = -ivalues[3];

	/* part orientation on the x,y plane */
	switch (akm->pdata->orientation) {
	case 0:
		break;

	case 90:
		tmp = ivalues[1];
		ivalues[1] = -ivalues[2];
		ivalues[2] = tmp;
		break;

	case 180:
		ivalues[1] = -ivalues[1];
		ivalues[2] = -ivalues[2];
		break;

	case 270:
		tmp = ivalues[1];
		ivalues[1] = ivalues[2];
		ivalues[2] = -tmp;
		break;

	default:
		break;
	}
}

static void akm8973_report_values(struct akm8973_data *akm,
				  int *values)
{
	if (akm->state & AKM8973_MAG) {
		input_report_abs(akm->input_dev, ABS_HAT0X, values[1]);
		input_report_abs(akm->input_dev, ABS_HAT0Y, values[2]);
		input_report_abs(akm->input_dev, ABS_BRAKE, values[3]);
	}

	if (akm->state & AKM8973_TEMP)
		input_report_abs(akm->input_dev, ABS_THROTTLE, values[0]);

	if (akm->state)
		input_sync(akm->input_dev);
}

static void akm8973_irq_work_func(struct work_struct *work)
{
	struct akm8973_data *akm =
	    container_of(work, struct akm8973_data, irq_work);
	u8 buf[4];
	int ibuf[4];
	int err;

	buf[0] = AKM8973_REG_TMPS;
	err = akm8973_i2c_read(akm, buf, 4);
	if (err < 0)
		goto err0;

	/* don't report railed values */
	if (!akm8973_auto_calibrate(akm, buf)) {
		akm8973_transform_values(akm, buf, ibuf);
		akm8973_report_values(akm, ibuf);
	}

err0:
	clear_bit(BUSY, &akm->flags);
	enable_irq(akm->client->irq);
}

static irqreturn_t akm8973_isr(int irq, void *dev_id)
{
	struct akm8973_data *akm = dev_id;
	disable_irq_nosync(akm->client->irq);
	schedule_work(&akm->irq_work);
	return IRQ_HANDLED;
}

static int akm8973_enable(struct akm8973_data *akm)
{
	int err;

	if (!test_and_set_bit(ENABLED, &akm->flags)) {
		err = akm8973_device_power_on(akm);
		if (err < 0) {
			clear_bit(ENABLED, &akm->flags);
			return err;
		}

		schedule_delayed_work(&akm->input_work,
				      msecs_to_jiffies(akm->pdata->
						       poll_interval));
	}

	return 0;
}

static int akm8973_disable(struct akm8973_data *akm)
{
	if (test_and_clear_bit(ENABLED, &akm->flags)) {
		cancel_delayed_work_sync(&akm->input_work);
		akm8973_device_power_off(akm);
	}

	return 0;
}

static int akm8973_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = akm8973_misc_data;

	return 0;
}

static int akm8973_misc_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 buf[4];
	int err;
	int interval;
	struct akm8973_data *akm = file->private_data;

	switch (cmd) {
	case AKM8973_IOCTL_GET_CALI:
		buf[0] = akm->pdata->hxda;
		buf[1] = akm->pdata->hyda;
		buf[2] = akm->pdata->hzda;
		if (copy_to_user(argp, &buf, 3))
			return -EFAULT;
		break;

	case AKM8973_IOCTL_SET_CALI:
		/* leave room for command byte */
		if (test_and_set_bit(BUSY, &akm->flags)) {
			return -EBUSY;
		}

		if (copy_from_user(buf + 1, argp, 3))
			return -EFAULT;

		buf[0] = AKM8973_REG_HXDA;
		err = akm8973_i2c_write(akm, buf, 3);
		if (err < 0)
			return err;

		akm->pdata->hxda = buf[1];
		akm->pdata->hyda = buf[2];
		akm->pdata->hzda = buf[3];
		clear_bit(BUSY, &akm->flags);
		break;

	case AKM8973_IOCTL_GET_DELAY:
		interval = akm->pdata->poll_interval;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EFAULT;
		break;

	case AKM8973_IOCTL_SET_DELAY:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval < 0 || interval > 200)
			return -EINVAL;

		akm->pdata->poll_interval =
		    max(interval, akm->pdata->min_interval);
		break;

	case AKM8973_IOCTL_SET_FLAG:
		if (copy_from_user(&interval, argp, sizeof(interval)))
			return -EFAULT;
		if (interval > 3)
			return -EINVAL;

		akm->state = interval;
		if (akm->state != 0)
			akm8973_enable(akm);
		else
			akm8973_disable(akm);

		break;

	case AKM8973_IOCTL_GET_FLAG:
		interval = akm->state;
		if (copy_to_user(argp, &interval, sizeof(interval)))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations akm8973_misc_fops = {
	.owner = THIS_MODULE,
	.open = akm8973_misc_open,
	.ioctl = akm8973_misc_ioctl,
};

static struct miscdevice akm8973_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &akm8973_misc_fops,
};

static void akm8973_input_work_func(struct work_struct *work)
{
	struct akm8973_data *akm = container_of((struct delayed_work *)work,
						struct akm8973_data,
						input_work);

	/* if we are already measuring, don't start another measurement */
	if (!test_and_set_bit(BUSY, &akm->flags)) {
		akm8973_set_mode(akm, AKM8973_MODE_MEASURE);
	}
	schedule_delayed_work(&akm->input_work,
			      msecs_to_jiffies(akm->pdata->poll_interval));
}

#ifdef AKM8973_OPEN_ENABLE
int akm8973_input_open(struct input_dev *input)
{
	struct akm8973_data *akm = input_get_drvdata(input);

	return akm8973_enable(akm);
}

void akm8973_input_close(struct input_dev *dev)
{
	struct akm8973_data *akm = input_get_drvdata(dev);

	akm8973_disable(akm);
}
#endif

static int akm8973_validate_pdata(struct akm8973_data *akm)
{
	switch (akm->pdata->orientation) {
	case 0:
	case 90:
	case 180:
	case 270:
		break;

	default:
		dev_warn(&akm->client->dev,
			 "part orientation not recognized, defaulting to 0\n");
		akm->pdata->orientation = 0;
		break;
	}

	akm->pdata->poll_interval = max(akm->pdata->poll_interval,
					akm->pdata->min_interval);

	return 0;
}

static int akm8973_input_init(struct akm8973_data *akm)
{
	int err;

	INIT_DELAYED_WORK(&akm->input_work, akm8973_input_work_func);

	akm->input_dev = input_allocate_device();
	if (!akm->input_dev) {
		err = -ENOMEM;
		dev_err(&akm->client->dev, "input device allocate failed\n");
		goto err0;
	}


#ifdef AKM8973_OPEN_ENABLE
	akm->input_dev->open = akm8973_input_open;
	akm->input_dev->close = akm8973_input_close;
#endif

	input_set_drvdata(akm->input_dev, akm);

	set_bit(EV_ABS, akm->input_dev->evbit);

	/* x-axis of raw magnetic vector */
	input_set_abs_params(
			akm->input_dev, ABS_HAT0X, -2000, 2000, FUZZ, FLAT);
	/* y-axis of raw magnetic vector */
	input_set_abs_params(
			akm->input_dev, ABS_HAT0Y, -2000, 2000, FUZZ, FLAT);
	/* z-axis of raw magnetic vector */
	input_set_abs_params(
			akm->input_dev, ABS_BRAKE, -2000, 2000, FUZZ, FLAT);
	/* temperature */
	input_set_abs_params(
			akm->input_dev, ABS_THROTTLE, -30, 85, FUZZ, FLAT);

	akm->input_dev->name = "magnetometer";

	err = input_register_device(akm->input_dev);
	if (err) {
		dev_err(&akm->client->dev,
			"unable to register input polled device %s\n",
			akm->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(akm->input_dev);
err0:
	return err;
}

static void akm8973_input_cleanup(struct akm8973_data *akm)
{
	input_unregister_device(akm->input_dev);
}

static int akm8973_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct akm8973_data *akm;
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

	akm = kzalloc(sizeof(*akm), GFP_KERNEL);
	if (akm == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	akm->client = client;

	akm->pdata = kmalloc(sizeof(*akm->pdata), GFP_KERNEL);
	if (akm->pdata == NULL)
		goto err1;

	memcpy(akm->pdata, client->dev.platform_data, sizeof(*akm->pdata));

	err = akm8973_validate_pdata(akm);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto err1_1;
	}

	i2c_set_clientdata(client, akm);

	INIT_WORK(&akm->irq_work, akm8973_irq_work_func);

	if (akm->pdata->init) {
		err = akm->pdata->init();
		if (err < 0)
			goto err1_1;
	}

	err = akm8973_device_power_on(akm);
	if (err < 0)
		goto err2;

	err = akm8973_input_init(akm);
	if (err < 0)
		goto err3;

	akm8973_misc_data = akm;
	err = misc_register(&akm8973_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "akmd_device register failed\n");
		goto err4;
	}

	err = request_irq(akm->client->irq, akm8973_isr,
			  IRQF_TRIGGER_RISING, "akm8973_irq", akm);
	if (err < 0) {
		dev_err(&client->dev, "failed to request irq %d\n",
			akm->client->irq);
		goto err5;
	}

	akm8973_device_power_off(akm);

	/* As default, do not report information */
	akm->state = 0;
	akm->flags = 0;

	dev_info(&client->dev, "akm8973 probed\n");

	return 0;

err5:
	misc_deregister(&akm8973_misc_device);
err4:
	akm8973_input_cleanup(akm);
err3:
	akm8973_device_power_off(akm);
err2:
	if (akm->pdata->exit)
		akm->pdata->exit();
err1_1:
	kfree(akm->pdata);
err1:
	kfree(akm);
err0:
	return err;
}

static int __devexit akm8973_remove(struct i2c_client *client)
{
	/* TODO: revisit ordering here once _probe order is finalized */
	struct akm8973_data *akm = i2c_get_clientdata(client);

	free_irq(akm->client->irq, akm);
	misc_deregister(&akm8973_misc_device);
	akm8973_input_cleanup(akm);
	akm8973_device_power_off(akm);
	if (akm->pdata->exit)
		akm->pdata->exit();
	kfree(akm->pdata);
	kfree(akm);

	return 0;
}

static int akm8973_resume(struct i2c_client *client)
{
	struct akm8973_data *akm = i2c_get_clientdata(client);

	if (test_and_clear_bit(ENABLED_BEFORE_SUSPEND, &akm->flags))
		return akm8973_enable(akm);

	return 0;
}

static int akm8973_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct akm8973_data *akm = i2c_get_clientdata(client);
	/* it's ok that this check isn't atomic since we are in a suspend
	 * path so userspace is frozen and can't call the ioctl to disable it
	 * in between */
	if (akm->flags & ENABLED)
		set_bit(ENABLED_BEFORE_SUSPEND, &akm->flags);

	return akm8973_disable(akm);
}

static const struct i2c_device_id akm8973_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, akm8973_id);

static struct i2c_driver akm8973_driver = {
	.driver = {
		   .name = NAME,
		   },
	.probe = akm8973_probe,
	.remove = __devexit_p(akm8973_remove),
	.resume = akm8973_resume,
	.suspend = akm8973_suspend,
	.id_table = akm8973_id,
};

static int __init akm8973_init(void)
{
	pr_info(KERN_INFO "AKM8973 magnetometer driver\n");
	return i2c_add_driver(&akm8973_driver);
}

static void __exit akm8973_exit(void)
{
	i2c_del_driver(&akm8973_driver);
	return;
}

module_init(akm8973_init);
module_exit(akm8973_exit);

MODULE_DESCRIPTION("akm8973 magnetometer driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
