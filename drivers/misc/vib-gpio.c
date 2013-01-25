/* drivers/misc/vib-gpio.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 * Copyright (C) 2008 Google, Inc.
 * Author: Mike Lockwood <lockwood@android.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/vib-gpio.h>
#include <linux/workqueue.h>

/* TODO: replace with correct header */
#include "../staging/android/timed_output.h"

struct vib_gpio_data {
	struct timed_output_dev dev;
	struct work_struct vib_work;
	struct hrtimer timer;
	spinlock_t lock;

	struct vib_gpio_platform_data *pdata;

	int vib_power_state;
	int vib_state;
};

struct vib_gpio_data *misc_data;

static void vib_gpio_set(int on)
{
	if (on) {
		if (misc_data->pdata->power_on && !misc_data->vib_power_state) {
			misc_data->pdata->power_on();
			misc_data->vib_power_state = 1;
		}
		gpio_direction_output(misc_data->pdata->gpio,
				      misc_data->pdata->active_low ? 0 : 1);
	} else {
		gpio_direction_output(misc_data->pdata->gpio,
				      misc_data->pdata->active_low ? 1 : 0);
		if (misc_data->pdata->power_on && misc_data->vib_power_state) {
			misc_data->pdata->power_off();
			misc_data->vib_power_state = 0;
		}
	}
}

static void vib_gpio_update(struct work_struct *work)
{
	vib_gpio_set(misc_data->vib_state);
}

static enum hrtimer_restart gpio_timer_func(struct hrtimer *timer)
{
	struct vib_gpio_data *data =
	    container_of(timer, struct vib_gpio_data, timer);
	data->vib_state = 0;
	schedule_work(&data->vib_work);
	return HRTIMER_NORESTART;
}

static int vib_gpio_get_time(struct timed_output_dev *dev)
{
	struct vib_gpio_data *data =
	    container_of(dev, struct vib_gpio_data, dev);

	if (hrtimer_active(&data->timer)) {
		ktime_t r = hrtimer_get_remaining(&data->timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		return 0;
}

static void vib_gpio_enable(struct timed_output_dev *dev, int value)
{
	struct vib_gpio_data *data =
	    container_of(dev, struct vib_gpio_data, dev);
	unsigned long flags;

	spin_lock_irqsave(&data->lock, flags);
	hrtimer_cancel(&data->timer);

	if (value == 0)
		data->vib_state = 0;
	else {
		value = (value > data->pdata->max_timeout ?
				 data->pdata->max_timeout : value);
		data->vib_state = 1;
		hrtimer_start(&data->timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&data->lock, flags);

	schedule_work(&data->vib_work);
}

/* This is a temporary solution until a more global haptics soltion is
 * available for haptics that need to occur in any application */
void vibrator_haptic_fire(int value)
{
	vib_gpio_enable(&misc_data->dev, value);
}

static int vib_gpio_probe(struct platform_device *pdev)
{
	struct vib_gpio_platform_data *pdata = pdev->dev.platform_data;
	struct vib_gpio_data *gpio_data;
	int ret = 0;

	if (!pdata) {
		ret = -EBUSY;
		goto err0;
	}

	gpio_data = kzalloc(sizeof(struct vib_gpio_data), GFP_KERNEL);
	if (!gpio_data) {
		ret = -ENOMEM;
		goto err0;
	}

	gpio_data->pdata = pdata;

	INIT_WORK(&gpio_data->vib_work, vib_gpio_update);

	hrtimer_init(&gpio_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	gpio_data->timer.function = gpio_timer_func;
	spin_lock_init(&gpio_data->lock);

	gpio_data->dev.name = "vibrator";
	gpio_data->dev.get_time = vib_gpio_get_time;
	gpio_data->dev.enable = vib_gpio_enable;
	ret = timed_output_dev_register(&gpio_data->dev);
	if (ret < 0)
		goto err1;

	if (gpio_data->pdata->init)
		ret = gpio_data->pdata->init();
	if (ret < 0)
		goto err2;

	gpio_direction_output(gpio_data->pdata->gpio,
			      gpio_data->pdata->active_low);

	misc_data = gpio_data;
	platform_set_drvdata(pdev, gpio_data);

	vib_gpio_enable(&gpio_data->dev, gpio_data->pdata->initial_vibrate);

	return 0;

err2:
	timed_output_dev_unregister(&gpio_data->dev);
err1:
	kfree(gpio_data->pdata);
	kfree(gpio_data);
err0:
	return ret;
}

static int vib_gpio_remove(struct platform_device *pdev)
{
	struct vib_gpio_data *gpio_data = platform_get_drvdata(pdev);

	if (gpio_data->pdata->exit)
		gpio_data->pdata->exit();

	timed_output_dev_unregister(&gpio_data->dev);

	kfree(gpio_data->pdata);
	kfree(gpio_data);

	return 0;
}

static struct platform_driver vib_gpio_driver = {
	.probe = vib_gpio_probe,
	.remove = vib_gpio_remove,
	.driver = {
		   .name = VIB_GPIO_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init vib_gpio_init(void)
{
	return platform_driver_register(&vib_gpio_driver);
}

static void __exit vib_gpio_exit(void)
{
	platform_driver_unregister(&vib_gpio_driver);
}

module_init(vib_gpio_init);
module_exit(vib_gpio_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("vib gpio driver");
MODULE_LICENSE("GPL");
