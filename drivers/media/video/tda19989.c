/*
 * drivers/media/video/tda19989.c
 *
 * Copyright (C) 2009 Motorola Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/kmod.h>
#include <linux/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/regulator/consumer.h>
#include <media/tda19989_platform.h>
#include <mach/gpio.h>

#include <linux/tda19989.h>

#define DEVICE_NAME	"tda19989"

struct tda19989_data {
	struct mutex mtx; /* Lock for all struct accesses */

	bool opened;

	struct i2c_client *client;

	int  int_gpio;
	bool int_enabled;
	bool waiter;
	wait_queue_head_t int_wait;

	int  pwr_en_gpio;
	bool pwr_enabled;

	spinlock_t int_lock; /* Lock for int_occurred flag */
	bool int_occurred;

	bool exiting;

	int  cec_i2c_dev; /* CEC is not supported if 0 */
	struct regulator *cec_reg;
	char cec_reg_name[TDA19989_CEC_REGULATOR_NAME_SIZE];
	bool cec_in_use;
};

static struct tda19989_data *g_dev;

/*===========================================================================*/

static irqreturn_t hdmi_int_irq(int irq, void *dev_inst)
{
	printk(KERN_DEBUG "hdmi_int_irq\n");

	if (g_dev) {
		spin_lock(&g_dev->int_lock);
		g_dev->int_occurred = true;
		wake_up_interruptible(&g_dev->int_wait);
		spin_unlock(&g_dev->int_lock);
	}

	return IRQ_HANDLED;
}

static int check_int(void)
{
	int rc = 0;
	unsigned long lock_flags;

	spin_lock_irqsave(&g_dev->int_lock, lock_flags);
	if (g_dev->int_occurred) {
		rc = 1;
		g_dev->int_occurred = false;
	} else if (g_dev->exiting || !g_dev->int_enabled) {
		rc = 1;
	}
	spin_unlock_irqrestore(&g_dev->int_lock, lock_flags);

	return rc;
}

static int i2cTda19989_write(struct i2cMsgArg *pArg)
{
	u8 reg;
	u8 length;
	u8 *pData;
	int rc = 0;

	reg = (u8) pArg->firstRegister;
	length = (u8) pArg->lenData;
	pData = (u8 *) &pArg->Data[0];

	if (g_dev->client == NULL) {
		printk(KERN_ERR "I2cTda19989_write no client\n");
		return -EFAULT;
	}

	g_dev->client->addr = (unsigned short) pArg->slaveAddr;

	if ((length > 1) && (pArg->slaveAddr == g_dev->cec_i2c_dev)) {
		/* For CEC, the hardware may require I2C block writes */
		rc = i2c_smbus_write_i2c_block_data(
				g_dev->client, reg, length, pData);
		if (rc != 0) {
			printk(KERN_ERR "i2cTda19989 W(B) err:%d\n", rc);
		}
	} else {
		while (length--) {
			rc = i2c_smbus_write_byte_data(
					g_dev->client, reg, *pData);
			if (rc != 0) {
				printk(KERN_ERR "I2cTda19989 W err:%d\n", rc);
				break;
			}
			reg++;
			pData++;
		}
	}

	/* Work-around for CEC.  Some sort of write timing issue. */
	if (pArg->slaveAddr == g_dev->cec_i2c_dev)
		mdelay(2);

	return ((rc == 0) ? 0 : -EFAULT);
}

static int i2cTda19989_read(struct i2cMsgArg *pArg)
{
	u8 reg;
	u8 length;
	u8 *pData;
	int rc = 0;

	reg = (u8) pArg->firstRegister;
	length = (u8) pArg->lenData;
	pData = (u8 *) &pArg->Data[0];

	if (g_dev->client == NULL) {
		printk(KERN_ERR "I2cTda19989_read no client\n");
		return -EFAULT;
	}

	g_dev->client->addr = (unsigned short) pArg->slaveAddr;

	if ((length > 1) && (pArg->slaveAddr == g_dev->cec_i2c_dev)) {
		/* For CEC, the hardware _requires_ I2C block reads */
		rc = i2c_smbus_read_i2c_block_data(
				g_dev->client, reg, length, pData);
		if ((u8)rc != length) {
			printk(KERN_ERR "i2cTda19989 R(B) err:%d\n", rc);
			return -EFAULT;
		}
	} else {
		while (length--) {
			rc = i2c_smbus_read_byte_data(g_dev->client, reg);
			if (rc < 0) {
				printk(KERN_ERR "i2cTda19989 R err:%d\n", rc);
				return -EFAULT;
			}
			*pData = (u8) rc;
			reg++;
			pData++;
		}
	}

	return 0;
}

static int cec_calibration(void)
{
	int i;
	int prevIntEn = 0;
	struct timeval prevTime;
	struct timeval curTime;
	struct timeval resultTime;

	/* During CEC calibration we need to use the interrupt pin as an
	 * output, so disable the IRQ and release the waiter
	 */
	if (g_dev->int_enabled) {
		prevIntEn = 1;
		disable_irq(OMAP_GPIO_IRQ(g_dev->int_gpio));
		if (g_dev->waiter)
			wake_up_interruptible(&g_dev->int_wait);
		g_dev->int_enabled = false;
	}

	/* Reconfigure the interrupt pin as an output pin */
	gpio_direction_output(g_dev->int_gpio, 0);

	/* The output should transition low and stay low for 10ms +/- 1% */
	gpio_set_value(g_dev->int_gpio, 0);
	do_gettimeofday(&prevTime);
	mdelay(9);
	for (i = 0; i < 500; i++) {
		do_gettimeofday(&curTime);
		resultTime.tv_usec = curTime.tv_usec - prevTime.tv_usec;
		if (resultTime.tv_usec > 9990)
			break;
		udelay(2);
	}
	gpio_set_value(g_dev->int_gpio, 1);
	do_gettimeofday(&curTime);

	/* Reconfigure the pin as an input */
	gpio_direction_input(g_dev->int_gpio);

	resultTime.tv_usec = curTime.tv_usec - prevTime.tv_usec;
	printk(KERN_DEBUG "Time interval: %d\n", (int)resultTime.tv_usec);

	/* Re-enable the IRQ and allow a waiter, if needed */
	if (prevIntEn) {
		enable_irq(OMAP_GPIO_IRQ(g_dev->int_gpio));
		g_dev->int_enabled = true;
	}

	return 0;
}

static int cec_regulator_enable(bool en)
{
	int rc = 0;

	if (g_dev->cec_reg_name[0] == 0 || g_dev->cec_reg == NULL) {
		printk(KERN_DEBUG "tda19989 No CEC Regulator\n");
	} else if (en) {
		if (regulator_enable(g_dev->cec_reg) < 0) {
			printk(KERN_ERR "tda19989 cec reg enable failed\n");
			g_dev->cec_reg = NULL;
			rc = -EFAULT;
		}
	} else {
		if (regulator_disable(g_dev->cec_reg) < 0)
			printk(KERN_ERR "tda19989 cec reg disable failed\n");
	}

	return rc;
}

/*===========================================================================*/

static int tda19989_open(struct inode *inode, struct file *filp)
{
	int rc;
	struct res_handle *cec_rhandle;

	printk(KERN_DEBUG "tda19989_open\n");

	cec_rhandle = NULL;

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&g_dev->mtx);

	/* We only support single open */
	if (g_dev->opened) {
		printk(KERN_ERR "Device already opened\n");
		rc = -EBUSY;
		goto failed;
	}

	g_dev->opened = true;

	g_dev->exiting = false;

	g_dev->waiter = false;
	init_waitqueue_head(&g_dev->int_wait);

	g_dev->pwr_enabled = false;
	rc = gpio_request(g_dev->pwr_en_gpio, "HDMI_PWR_EN");
	if (rc < 0) {
		printk(KERN_ERR "tda19989 GPIO Pwr On request error\n");
		rc = -EFAULT;
		goto failed;
	}
	gpio_direction_output(g_dev->pwr_en_gpio, 0);

	g_dev->int_enabled = false;
	rc = gpio_request(g_dev->int_gpio, "HDMI_INT");
	if (rc < 0) {
		printk(KERN_ERR "tda19989 GPIO INT request error\n");
		rc = -EFAULT;
		goto failed_gpio;
	}
	gpio_direction_input(g_dev->int_gpio);
	set_irq_type(gpio_to_irq(g_dev->int_gpio),
						IRQ_TYPE_EDGE_FALLING);
	rc = request_irq(gpio_to_irq(g_dev->int_gpio), hdmi_int_irq,
					IRQF_TRIGGER_FALLING | IRQF_DISABLED,
					DEVICE_NAME, (void *)NULL);
	if (rc != 0) {
		printk(KERN_ERR	"tda19989 req irq err (%d)\n", rc);
		rc = -EFAULT;
		goto failed_irq;
	}
	disable_irq(OMAP_GPIO_IRQ(g_dev->int_gpio));

	g_dev->cec_in_use = false;
	if (g_dev->cec_reg_name[0] != 0) {
		g_dev->cec_reg = regulator_get(NULL, g_dev->cec_reg_name);
		if (IS_ERR(g_dev->cec_reg)) {
			printk(KERN_ERR "tda19989 cec get regulator failed\n");
			rc = -ENODEV;
			goto failed_irq;
		}
	}

	g_dev->int_occurred = false;

	mutex_unlock(&g_dev->mtx);

	return 0;

failed_irq:
	gpio_free(g_dev->int_gpio);
failed_gpio:
	gpio_free(g_dev->pwr_en_gpio);
failed:
	mutex_unlock(&g_dev->mtx);
	return rc;
}

static int tda19989_release(struct inode *inode, struct file *filp)
{
	printk(KERN_DEBUG "tda19989_release\n");

	if (g_dev == NULL) {
		printk(KERN_ERR "Invalid device\n");
		return -ENODEV;
	}

	mutex_lock(&g_dev->mtx);

	g_dev->opened = false;

	g_dev->exiting = true;

	if (g_dev->int_enabled)
		disable_irq(OMAP_GPIO_IRQ(g_dev->int_gpio));
	free_irq(gpio_to_irq(g_dev->int_gpio), (void *)NULL);
	gpio_direction_input(g_dev->int_gpio);
	gpio_free(g_dev->int_gpio);

	if (g_dev->waiter)
		wake_up_interruptible(&g_dev->int_wait);

	gpio_direction_input(g_dev->pwr_en_gpio);
	gpio_free(g_dev->pwr_en_gpio);

	if (g_dev->cec_in_use) {
		if (g_dev->cec_reg)
			regulator_disable(g_dev->cec_reg);
		g_dev->cec_in_use = false;
	}
	if (g_dev->cec_reg) {
		regulator_put(g_dev->cec_reg);
		g_dev->cec_reg = NULL;
	}

	mutex_unlock(&g_dev->mtx);

	return 0;
}

static ssize_t tda19989_read(struct file *fp, char __user *buf,
						size_t count, loff_t *ppos)
{
	int rc = 0;

	printk(KERN_DEBUG "tda19989_read\n");

	mutex_lock(&g_dev->mtx);

	if (g_dev->exiting) {
		rc = -EINVAL;
		goto exit;
	}

	if (g_dev->waiter) {
		rc = -EBUSY;
		goto exit;
	}

	if (!g_dev->int_enabled) {
		rc = -EAGAIN;
		goto exit;
	}

	printk(KERN_DEBUG "waiting ...\n");
	g_dev->waiter = true;

	mutex_unlock(&g_dev->mtx);

	wait_event_interruptible(g_dev->int_wait, check_int());

	mutex_lock(&g_dev->mtx);

	g_dev->waiter = false;
	printk(KERN_DEBUG "exit waiting\n");

exit:
	mutex_unlock(&g_dev->mtx);

	return rc;
}

static ssize_t tda19989_write(struct file *file, const char __user *buf,
						size_t count, loff_t *ppos)
{
	printk(KERN_DEBUG "tda19989_write\n");
	return 0;
}

static int tda19989_ioctl(struct inode *inode, struct file *filp,
						u_int cmd, u_long arg)
{
	int rc = 0;
	struct i2cMsgArg mArg;
	int en;

	if (unlikely(_IOC_TYPE(cmd) != TDA19989_IOCTL_MAGIC)) {
		printk(KERN_ERR "Bad command value (%d)\n", cmd);
		return -EINVAL;
	}

	mutex_lock(&g_dev->mtx);

	switch (cmd) {
	case TDA19989_I2C_WRITE:
		if (copy_from_user((char *)&mArg, (char *)arg, sizeof(mArg))) {
			printk(KERN_ERR	"tda19989: WRITE copy from error\n");
			rc = -EFAULT;
			break;
		}
		rc = i2cTda19989_write(&mArg);
		break;
	case TDA19989_I2C_READ:
		if (copy_from_user((char *)&mArg, (char *)arg, sizeof(mArg))) {
			printk(KERN_ERR	"tda19989: READ copy from error\n");
			rc = -EFAULT;
			break;
		}
		rc = i2cTda19989_read(&mArg);
		if (copy_to_user((char *)arg, (char *)&mArg, sizeof(mArg))) {
			printk(KERN_ERR	"tda19989: READ copy to error\n");
			rc = -EFAULT;
		}
		break;
	case TDA19989_PWR_ENABLE:
		if (copy_from_user(&en, (int *)arg, sizeof(en))) {
			printk(KERN_ERR	"tda19989: 5V EN copy from error\n");
			rc = -EFAULT;
			break;
		}
		if (en && !g_dev->pwr_enabled) {
			gpio_set_value(g_dev->pwr_en_gpio, 1);
			g_dev->pwr_enabled = true;
		} else if (!en && g_dev->pwr_enabled) {
			gpio_set_value(g_dev->pwr_en_gpio, 0);
			g_dev->pwr_enabled = false;
		}
		break;
	case TDA19989_INT_ENABLE:
		if (copy_from_user(&en, (int *)arg, sizeof(en))) {
			printk(KERN_ERR	"tda19989: INT EN copy from error\n");
			rc = -EFAULT;
			break;
		}
		if (en && !g_dev->int_enabled) {
			enable_irq(OMAP_GPIO_IRQ(g_dev->int_gpio));
			g_dev->int_enabled = true;
		} else if (!en && g_dev->int_enabled) {
			disable_irq(OMAP_GPIO_IRQ(g_dev->int_gpio));
			g_dev->int_enabled = false;
			if (g_dev->waiter)
				wake_up_interruptible(&g_dev->int_wait);
		}
		break;
	case TDA19989_CEC_CAL_TIME:
		if (!g_dev->cec_i2c_dev) {
			printk(KERN_ERR	"tda19989: CEC not supported\n");
			rc = -EINVAL;
			break;
		}

		rc = cec_calibration();
		if (rc != 0) {
			printk(KERN_ERR	"tda19989: CEC cal error (%d)\n", rc);
			rc = -EFAULT;
		}
		break;
	case TDA19989_CEC_PWR_ENABLE:
		if (!g_dev->cec_i2c_dev) {
			printk(KERN_ERR	"tda19989: CEC not supported\n");
			rc = -EINVAL;
			break;
		}

		if (copy_from_user(&en, (int *)arg, sizeof(en))) {
			printk(KERN_ERR	"tda19989: CEC reg copy from error\n");
			rc = -EFAULT;
			break;
		}
		if (en && !g_dev->cec_in_use) {
			g_dev->cec_in_use = true;
			rc = cec_regulator_enable(true);
		} else if (!en && g_dev->cec_in_use) {
			g_dev->cec_in_use = false;
			rc = cec_regulator_enable(false);
		}
		break;
	case TDA19989_CEC_SUPPORT:
		if (!g_dev->cec_i2c_dev) {
			printk(KERN_ERR	"tda19989: CEC not supported\n");
			rc = -EINVAL;
		}
		break;
	default:
		rc = -EINVAL;
		break;
	}

	mutex_unlock(&g_dev->mtx);

	return rc;
}

static const struct file_operations tda19989_fops = {
	.owner = THIS_MODULE,
	.open = tda19989_open,
	.release = tda19989_release,
	.read = tda19989_read,
	.write = tda19989_write,
	.ioctl = tda19989_ioctl,
};

static int i2cTda19989_probe(struct i2c_client *client,
					const struct i2c_device_id *id)
{
	printk(KERN_DEBUG "tda19989 I2C probe\n");

	if (!g_dev) {
		printk(KERN_ERR "tda19989 No device storage\n");
		return -ENODEV;
	} else if (!i2c_check_functionality(client->adapter,
					I2C_FUNC_SMBUS_BYTE_DATA)) {
		printk(KERN_ERR "tda19989 Can't support SMBUS\n");
		return -ENODEV;
	}

	g_dev->client = client;

	return 0;
}

static int i2cTda19989_remove(struct i2c_client *client)
{
	printk(KERN_DEBUG "tda19989 I2C remove\n");

	if (g_dev)
		g_dev->client = 0;

	return 0;
}

static const struct i2c_device_id tda19989_id[] = {
	{ DEVICE_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, tda19989_id);

static struct i2c_driver i2c_driver_tda19989 = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = i2cTda19989_probe,
	.remove = __devexit_p(i2cTda19989_remove),
	.id_table = tda19989_id,
};

static struct miscdevice tda19989_misc_device = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= DEVICE_NAME,
	.fops	= &tda19989_fops,
};

static int tda19989_probe(struct platform_device *pdev)
{
	struct tda19989_platform_data *cfg;
	int rc = 0;

	printk(KERN_DEBUG "tda19989_probe\n");

	if (!pdev->dev.platform_data)
		return -EINVAL;

	cfg = (struct tda19989_platform_data *) pdev->dev.platform_data;

	g_dev = kzalloc(sizeof(struct tda19989_data), GFP_KERNEL);
	if (g_dev == NULL)
		return -ENOMEM;

	mutex_init(&g_dev->mtx);
	spin_lock_init(&g_dev->int_lock);

	g_dev->pwr_en_gpio = cfg->pwr_en_gpio;
	g_dev->int_gpio = cfg->int_gpio;
	g_dev->cec_i2c_dev = cfg->cec_i2c_dev;
	g_dev->cec_reg_name[0] = 0;
	if (cfg->cec_reg_name[0] != 0) {
		cfg->cec_reg_name[TDA19989_CEC_REGULATOR_NAME_SIZE - 1] = 0;
		strcpy(g_dev->cec_reg_name, cfg->cec_reg_name);
	}

	g_dev->client = NULL;
	rc = i2c_add_driver(&i2c_driver_tda19989);
	if (rc) {
		printk(KERN_ERR "tda19989: i2c add dvr failed (%d)\n", rc);
		goto failed_i2c;
	}

	rc = misc_register(&tda19989_misc_device);
	if (rc) {
		printk(KERN_ERR "tda19989: misc register failed (%d)\n", rc);
		goto failed_misc;
	}

	return 0;

failed_misc:
	i2c_del_driver(&i2c_driver_tda19989);
failed_i2c:
	kfree(g_dev);
	g_dev = NULL;

	return rc;
}

static int tda19989_remove(struct platform_device *pdev)
{
	printk(KERN_DEBUG "tda19989_remove\n");

	i2c_del_driver(&i2c_driver_tda19989);
	misc_deregister(&tda19989_misc_device);
	kfree(g_dev);
	g_dev = NULL;

	return 0;
}

#if defined(CONFIG_PM)
static int tda19989_suspend(struct platform_device *pdev, pm_message_t event)
{
	return 0;
}

static int tda19989_resume(struct platform_device *pdev)
{
	return 0;
}
#endif

static struct platform_driver tda19989_driver = {
	.probe		= tda19989_probe,
	.remove		= tda19989_remove,
#if defined(CONFIG_PM)
	.suspend	= tda19989_suspend,
	.resume		= tda19989_resume,
#endif
	.driver		= {
		.name	= DEVICE_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init hdmiTda19989_init(void)
{
	int rc;

	rc = platform_driver_probe(&tda19989_driver, tda19989_probe);
	if (rc != 0) {
		printk(KERN_ERR "tda19989 register/probe failed (%d)\n", rc);
		return -ENODEV;
	}

	return 0;
}

static void __exit hdmiTda19989_exit(void)
{
	platform_driver_unregister(&tda19989_driver);
}

module_init(hdmiTda19989_init);
module_exit(hdmiTda19989_exit);

MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
