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

#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>

#include <linux/sfh7743.h>

/* unit = millimeter */
#define PROXIMITY_NEAR	30		/* prox close threshold is 22-70mm */
#define PROXIMITY_FAR	2147483647	/* (2^31)-1 */
#define NAME		"sfh7743"

/* Suspend and resume is disabled so the sensor can wake the processor */
#undef ENABLE_SUSPEND

struct sfh7743_data {
	struct platform_device *pdev;
	struct sfh7743_platform_data *pdata;

	struct work_struct irq_work;

	struct workqueue_struct *work_queue;
	struct input_dev *input_dev;

	atomic_t enabled;
	int on_before_suspend;

	int irq;
};

struct sfh7743_data *sfh7743_misc_data;

static void sfh7743_report_input(struct sfh7743_data *sfh)
{
	int distance;

	if (gpio_get_value(sfh->pdata->gpio))
		distance = PROXIMITY_NEAR;
	else
		distance = PROXIMITY_FAR;

	input_report_abs(sfh->input_dev, ABS_DISTANCE, distance);
	input_sync(sfh->input_dev);
}

static void sfh7743_device_power_off(struct sfh7743_data *sfh)
{
	if (sfh->pdata->power_off) {
		sfh->pdata->power_off();
		disable_irq(sfh->irq);
		disable_irq_wake(sfh->irq);
	}
}

static int sfh7743_device_power_on(struct sfh7743_data *sfh)
{
	int err;

	if (sfh->pdata->power_on) {
		err = sfh->pdata->power_on();
		if (err < 0)
			return err;
		enable_irq(sfh->irq);
		enable_irq_wake(sfh->irq);
		sfh7743_report_input(sfh);
	}

	return 0;
}

static irqreturn_t sfh7743_isr(int irq, void *dev)
{
	struct sfh7743_data *sfh = dev;

	disable_irq_nosync(irq);
	queue_work(sfh->work_queue, &sfh->irq_work);

	return IRQ_HANDLED;
}

static void sfh7743_irq_work_func(struct work_struct *work)
{
	struct sfh7743_data *sfh = container_of(work,
						struct sfh7743_data, irq_work);

	sfh7743_report_input(sfh);
	enable_irq(sfh->irq);
}

int sfh7743_enable(struct sfh7743_data *sfh)
{
	int err;

	if (!atomic_cmpxchg(&sfh->enabled, 0, 1)) {
		err = sfh7743_device_power_on(sfh);
		if (err) {
			atomic_set(&sfh->enabled, 0);
			return err;
		}
	}
	return 0;
}

int sfh7743_disable(struct sfh7743_data *sfh)
{
	if (atomic_cmpxchg(&sfh->enabled, 1, 0))
		sfh7743_device_power_off(sfh);

	return 0;
}

static int sfh7743_misc_open(struct inode *inode, struct file *file)
{
	int err;
	err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	file->private_data = sfh7743_misc_data;

	return 0;
}

static int sfh7743_misc_ioctl(struct inode *inode, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	u8 enable;
	struct sfh7743_data *sfh = file->private_data;

	switch (cmd) {
	case SFH7743_IOCTL_SET_ENABLE:
		if (copy_from_user(&enable, argp, 1))
			return -EFAULT;
		if (enable > 1)
			return -EINVAL;

		if (enable != 0)
			sfh7743_enable(sfh);
		else
			sfh7743_disable(sfh);

		break;

	case SFH7743_IOCTL_GET_ENABLE:
		enable = atomic_read(&sfh->enabled);
		if (copy_to_user(argp, &enable, 1))
			return -EINVAL;

		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations sfh7743_misc_fops = {
	.owner = THIS_MODULE,
	.open = sfh7743_misc_open,
	.ioctl = sfh7743_misc_ioctl,
};

static struct miscdevice sfh7743_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &sfh7743_misc_fops,
};

#ifdef SFH7743_OPEN_ENABLE
int sfh7743_input_open(struct input_dev *input)
{
	struct sfh7743_data *sfh = input_get_drvdata(input);

	return sfh7743_enable(sfh);
}

void sfh7743_input_close(struct input_dev *dev)
{
	struct sfh7743_data *sfh = input_get_drvdata(dev);

	sfh7743_disable(sfh);
}
#endif

static int sfh7743_input_init(struct sfh7743_data *sfh)
{
	int err;

	sfh->input_dev = input_allocate_device();
	if (!sfh->input_dev) {
		err = -ENOMEM;
		dev_err(&sfh->pdev->dev, "input device allocate failed\n");
		goto err0;
	}
#ifdef SFH7743_OPEN_ENABLE
	sfh->input_dev->open = sfh7743_input_open;
	sfh->input_dev->close = sfh7743_input_close;
#endif

	input_set_drvdata(sfh->input_dev, sfh);

	set_bit(EV_ABS, sfh->input_dev->evbit);
	set_bit(ABS_DISTANCE, sfh->input_dev->absbit);

	sfh->input_dev->name = "proximity";

	err = input_register_device(sfh->input_dev);
	if (err) {
		dev_err(&sfh->pdev->dev,
			"unable to register input polled device %s\n",
			sfh->input_dev->name);
		goto err1;
	}

	sfh7743_report_input(sfh);

	return 0;

err1:
	input_free_device(sfh->input_dev);
err0:
	return err;
}

static void sfh7743_input_cleanup(struct sfh7743_data *sfh)
{
	input_unregister_device(sfh->input_dev);
}

static int sfh7743_probe(struct platform_device *pdev)
{
	struct sfh7743_data *sfh;
	int err = -1;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err0;
	}

	sfh = kmalloc(sizeof(*sfh), GFP_KERNEL);
	if (sfh == NULL) {
		dev_err(&pdev->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}

	sfh->pdata = pdev->dev.platform_data;

	sfh->pdev = pdev;

	sfh->irq = gpio_to_irq(sfh->pdata->gpio);

	INIT_WORK(&sfh->irq_work, sfh7743_irq_work_func);

	sfh->work_queue = create_singlethread_workqueue("sfh7743_wq");
	if (!sfh->work_queue) {
		err = -ENOMEM;
		pr_err("%s: cannot create work queue: %d\n", __func__, err);
		goto err1;
	}

	if (sfh->pdata->init) {
		err = sfh->pdata->init();
		if (err < 0)
			goto err2;
	}

	if (sfh->pdata->power_on) {
		err = sfh->pdata->power_on();
		if (err < 0)
			goto err3;
	}

	err = sfh7743_input_init(sfh);
	if (err < 0)
		goto err4;

	sfh7743_misc_data = sfh;
	err = misc_register(&sfh7743_misc_device);
	if (err < 0) {
		dev_err(&pdev->dev, "sfhd_device register failed\n");
		goto err5;
	}

	atomic_set(&sfh->enabled, 0);

	platform_set_drvdata(pdev, sfh);

	err = request_irq(sfh->irq, sfh7743_isr,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "sfh7743_irq", sfh);

	if (err < 0) {
		pr_err("%s: request irq failed: %d\n", __func__, err);
		goto err6;
	}

	disable_irq_nosync(sfh->irq);

	if (sfh->pdata->power_off)
		sfh->pdata->power_off();

	dev_info(&pdev->dev, "sfh7743 probed\n");

	return 0;

err6:
	misc_deregister(&sfh7743_misc_device);
err5:
	sfh7743_input_cleanup(sfh);
err4:
	if (sfh->pdata->power_off)
		sfh->pdata->power_off();
err3:

	if (sfh->pdata->exit)
		sfh->pdata->exit();
err2:
	destroy_workqueue(sfh->work_queue);
err1:
	kfree(sfh->pdata);
	kfree(sfh);
err0:
	return err;
}

static int __devexit sfh7743_remove(struct platform_device *pdev)
{
	struct sfh7743_data *sfh = platform_get_drvdata(pdev);

	disable_irq_wake(sfh->irq);
	free_irq(sfh->irq, sfh);
	gpio_free(sfh->pdata->gpio);
	sfh7743_device_power_off(sfh);
	input_unregister_device(sfh->input_dev);
	if (sfh->pdata->exit)
		sfh->pdata->exit();
	destroy_workqueue(sfh->work_queue);
	kfree(sfh->pdata);
	kfree(sfh);

	return 0;
}

#ifdef ENABLE_SUSPEND
static int sfh7743_resume(struct platform_device *pdev)
{
	struct sfh7743_data *sfh = platform_get_drvdata(pdev);

	if (sfh->on_before_suspend)
		return sfh7743_enable(sfh);
	return 0;
}

static int sfh7743_suspend(struct platform_device *pdev, pm_message_t mesg)
{
	struct sfh7743_data *sfh = platform_get_drvdata(pdev);
	sfh->on_before_suspend = atomic_read(&sfh->enabled);

	return sfh7743_disable(sfh);
}
#endif

static struct platform_driver sfh7743_driver = {
	.probe = sfh7743_probe,
	.remove = __devexit_p(sfh7743_remove),
#ifdef ENABLE_SUSPEND
	.resume = sfh7743_resume,
	.suspend = sfh7743_suspend,
#endif
	.driver = {
		   .name = NAME,
		   },
};

static int __init sfh7743_init(void)
{
	return platform_driver_register(&sfh7743_driver);
}

static void __exit sfh7743_exit(void)
{
	platform_driver_unregister(&sfh7743_driver);
}

module_init(sfh7743_init);
module_exit(sfh7743_exit);

MODULE_DESCRIPTION("OSRAM SFH7743 Proximity Driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
