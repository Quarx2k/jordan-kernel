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
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/airc.h>

#define NAME		"airc"
#define I2C_RETRIES		10
#define I2C_RETRY_DELAY		5
#define INPUT_DEV_NAME "airc"

struct airc_data {
	struct airc_platform_data *pdata;

	struct work_struct irq_work;

	struct input_dev *input_dev;
	struct i2c_client *client;

	/* hardware access protection */
	struct mutex lock;

	int hw_initialized;
	atomic_t enabled;

};

struct airc_data *airc_misc_data;

static int airc_i2c_read(struct airc_data *airc, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = airc->client->addr,
		 .flags = (airc->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(airc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&airc->client->dev, "read transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int airc_i2c_write(struct airc_data *airc, u8 * buf, int len)
{
	int err;
	int tries = 0;
	struct i2c_msg msgs[] = {
		{
		 .addr = airc->client->addr,
		 .flags = airc->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	do {
		err = i2c_transfer(airc->client->adapter, msgs, 1);
		if (err != 1)
			msleep_interruptible(I2C_RETRY_DELAY);
	} while ((err != 1) && (++tries < I2C_RETRIES));

	if (err != 1) {
		dev_err(&airc->client->dev, "write transfer error\n");
		err = -EIO;
	} else {
		err = 0;
	}

	return err;
}

static int airc_hw_init(struct airc_data *airc)
{
	u8 buf[] = { 0x00, 0x00, 0x00, 0x00 };
	int err;

	u8 init_value[] = { 0x02, 0x00, 0x70 };
	u8 config_value_swipe[] = { 0x06, 0x00, 0x03 };
	u8 config_value_hold[] = { 0x07, 0x00, 0xFF };

	err = airc_i2c_write(airc, &init_value[0], 2);
	if (err < 0)
		return err;

	mdelay(1);

	err = airc_i2c_write(airc, &config_value_swipe[0], 2);
	if (err < 0)
		return err;

	mdelay(1);

	err = airc_i2c_write(airc, &config_value_hold[0], 2);
	if (err < 0)
		return err;

	mdelay(1);

	airc->hw_initialized = 1;
	return 0;
}

static void airc_device_power_off(struct airc_data *airc)
{
	if (airc->pdata->power_off) {
		disable_irq_nosync(airc->client->irq);
		airc->pdata->power_off();
		airc->hw_initialized = 0;
	}
}

static int airc_device_power_on(struct airc_data *airc)
{
	int err;

	if (airc->pdata->power_on) {
		err = airc->pdata->power_on();
		if (err < 0)
			return err;
		enable_irq(airc->client->irq);
	}

	if (!airc->hw_initialized) {
		mdelay(100);
		err = airc_hw_init(airc);
		if (err < 0) {
			airc_device_power_off(airc);
			return err;
		}
	}

	return 0;
}

static void airc_irq_work_func(struct work_struct *work)
{
	int distance;
	int err;
	int i;

	struct airc_data *aircfh = container_of(work,
						struct airc_data, irq_work);

	u8 reg_value[] = { 0x09, 0x00 };
	u8 temp[12];
	temp[0] = 0x00;

	err = airc_i2c_write(aircfh, &reg_value[0], 1);

	err = airc_i2c_read(aircfh, temp, 12);

	distance = ((temp[3]) & 0x03);
	distance = (distance * 256) + temp[2];

	distance = (temp[0] << 8 | distance);

    /* This might be useful for revert back to old firmware
	    distance = (temp[1] << 8 | temp[2]); */

	input_report_abs(aircfh->input_dev, ABS_Z, distance);
	input_sync(aircfh->input_dev);
	enable_irq(aircfh->client->irq);

}

static irqreturn_t airc_isr(int irq, void *dev)
{
	struct airc_data *airc = dev;
	disable_irq_nosync(airc->client->irq);
	schedule_work(&airc->irq_work);
	return IRQ_HANDLED;
}

int airc_enable(struct airc_data *aircfh)
{
	int err;

	if (!atomic_cmpxchg(&aircfh->enabled, 0, 1)) {
		err = airc_device_power_on(aircfh);
		if (err) {
			atomic_set(&aircfh->enabled, 0);
			return err;
		}
	}
	return 0;
}

int airc_disable(struct airc_data *aircfh)
{
	if (atomic_cmpxchg(&aircfh->enabled, 1, 0))
		airc_device_power_off(aircfh);

	return 0;
}

static int airc_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = airc_misc_data;

	return 0;
}

static int airc_misc_ioctl(struct inode *inode, struct file *file,
			   unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 enable;
	struct airc_data *aircfh = file->private_data;

	switch (cmd) {
	case AIRC_IOCTL_SET_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			airc_enable(aircfh);
		else
			airc_disable(aircfh);

		break;

	case AIRC_IOCTL_GET_ENABLE:
		enable = atomic_read(&aircfh->enabled);
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations airc_misc_fops = {
	.owner = THIS_MODULE,
	.open = airc_misc_open,
	.ioctl = airc_misc_ioctl,
};

static struct miscdevice airc_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &airc_misc_fops,
};

#ifdef AIRC_OPEN_ENABLE
int airc_input_open(struct input_dev *input)
{
	struct airc_data *aircfh = input_get_drvdata(input);

	return airc_enable(aircfh);
}

void airc_input_close(struct input_dev *dev)
{
	struct airc_data *aircfh = input_get_drvdata(dev);

	airc_disable(aircfh);
}
#endif

static int airc_input_init(struct airc_data *aircfh)
{
	int err;
	int distance;

	aircfh->input_dev = input_allocate_device();
	if (!aircfh->input_dev) {
		err = -ENOMEM;
		dev_err(&aircfh->client->dev, "input device allocate failed\n");
		goto err0;
	}
#ifdef AIRC_OPEN_ENABLE
	aircfh->input_dev->open = airc_input_open;
	aircfh->input_dev->close = airc_input_close;
#endif

	input_set_drvdata(aircfh->input_dev, aircfh);

	set_bit(EV_ABS, aircfh->input_dev->evbit);
	set_bit(ABS_Z, aircfh->input_dev->absbit);

	aircfh->input_dev->name = INPUT_DEV_NAME;

	err = input_register_device(aircfh->input_dev);
	if (err) {
		dev_err(&aircfh->client->dev,
			"unable to register input polled device %s\n",
			aircfh->input_dev->name);
		goto err1;
	}

	return 0;

err1:
	input_free_device(aircfh->input_dev);
err0:
	return err;
}

static void airc_input_cleanup(struct airc_data *aircfh)
{
	input_unregister_device(aircfh->input_dev);
}

static int airc_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct airc_data *aircfh;
	int err = -1;

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		pr_info(KERN_INFO "platform data is NULL\n");
		err = -ENODEV;
		goto err0;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		pr_info(KERN_INFO "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}

	aircfh = kmalloc(sizeof(*aircfh), GFP_KERNEL);
	if (aircfh == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		pr_info(KERN_INFO "failed to allocate memory\n");
		err = -ENOMEM;
		goto err0;
	}

	mutex_init(&aircfh->lock);
	mutex_lock(&aircfh->lock);
	aircfh->client = client;

	aircfh->pdata = kmalloc(sizeof(*aircfh->pdata), GFP_KERNEL);
	if (aircfh->pdata == NULL) {
		pr_info(KERN_INFO "pdata is null\n");
		goto err1;
	}

	memcpy(aircfh->pdata, client->dev.platform_data,
	       sizeof(*aircfh->pdata));

	i2c_set_clientdata(client, aircfh);

	INIT_WORK(&aircfh->irq_work, airc_irq_work_func);

	if (aircfh->pdata->init) {
		err = aircfh->pdata->init();
		if (err < 0)
			goto err1;
	}

	if (aircfh->pdata->power_on) {
		err = aircfh->pdata->power_on();
		if (err < 0)
			goto err3;
	}

	err = airc_input_init(aircfh);
	if (err < 0)
		goto err4;

	airc_misc_data = aircfh;
	err = misc_register(&airc_misc_device);
	if (err < 0) {
		dev_err(&client->dev, "aircfhd_device register failed\n");
		pr_info(KERN_INFO "aircfhd_device register failed\n");
		goto err5;
	}

	atomic_set(&aircfh->enabled, 0);
	aircfh->hw_initialized = 0;

	err = request_irq(aircfh->client->irq, airc_isr,
			  IRQF_TRIGGER_FALLING, "airc_irq", aircfh);

	if (err < 0) {
		pr_err("%s: request irq failed: %d\n", __func__, err);
		pr_info(KERN_INFO "request irq failed\n");
		goto err6;
	}

	disable_irq_nosync(aircfh->client->irq);

	if (aircfh->pdata->power_off)
		aircfh->pdata->power_off();

	mutex_unlock(&aircfh->lock);

	dev_info(&client->dev, "aircfh probed\n");
	pr_info(KERN_INFO "aircfh probed %d\n", aircfh->hw_initialized);

	airc_enable(aircfh);

	return 0;

err6:
	mutex_unlock(&aircfh->lock);
	misc_deregister(&airc_misc_device);
err5:
	mutex_unlock(&aircfh->lock);
	airc_input_cleanup(aircfh);
err4:
	mutex_unlock(&aircfh->lock);
	if (aircfh->pdata->power_off)
		aircfh->pdata->power_off();
err3:
	mutex_unlock(&aircfh->lock);
	if (aircfh->pdata->exit)
		aircfh->pdata->exit();
err1:
	mutex_unlock(&aircfh->lock);
	kfree(aircfh->pdata);
	kfree(aircfh);
err0:
	return err;
}

static int __devexit airc_remove(struct i2c_client *client)
{
	struct airc_data *airc = i2c_get_clientdata(client);

	free_irq(airc->client->irq, airc);
	misc_deregister(&airc_misc_device);
	airc_input_cleanup(airc);
	airc_device_power_off(airc);
	if (airc->pdata->exit)
		airc->pdata->exit();
	kfree(airc->pdata);
	kfree(airc);

	return 0;
}

static int airc_resume(struct i2c_client *client)
{
	struct airc_data *airc = i2c_get_clientdata(client);

	return airc_enable(airc);
}

static int airc_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct airc_data *airc = i2c_get_clientdata(client);

	return airc_disable(airc);
}

static const struct i2c_device_id airc_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, airc_id);

static struct i2c_driver airc_driver = {
	.probe = airc_probe,
	.remove = __devexit_p(airc_remove),
	.resume = airc_resume,
	.suspend = airc_suspend,
	.driver = {
		   .name = NAME,
		   },
	.id_table = airc_id,
};

static int __init airc_init(void)
{
	pr_info(KERN_INFO "Motorola AIRC driver\n");
	return i2c_add_driver(&airc_driver);
}

static void __exit airc_exit(void)
{
	i2c_del_driver(&airc_driver);
	return;
}

module_init(airc_init);
module_exit(airc_exit);

MODULE_DESCRIPTION("Motorola AIRC Proximity Driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
