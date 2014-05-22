/* drivers/misc/vib-gpio.c
 *
 * Copyright (C) 2013 Motorola, Inc.
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
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>

/* TODO: replace with correct header */
#include "../staging/android/timed_output.h"

struct vib_gpio_data {
	struct timed_output_dev dev;
	struct work_struct vib_work;
	struct hrtimer timer;
	spinlock_t lock;
	struct mutex io_mutex; /* protect GPIO & regulator operations */

	struct regulator *reg;
	int gpio;
	int max_timeout;
	bool active_low;
	int initial_vibrate;

	int vib_power_state;
	int vib_state;
};

static int power_on(struct vib_gpio_data *vib_data)
{
	if (vib_data->reg) {
		dev_dbg(vib_data->dev.dev, "enable regulator\n");
		return regulator_enable(vib_data->reg);
	}
	return 0;
}

static int power_off(struct vib_gpio_data *vib_data)
{
	if (vib_data->reg) {
		dev_dbg(vib_data->dev.dev, "disable regulator\n");
		return regulator_disable(vib_data->reg);
	}
	return 0;
}

static void vib_gpio_set(struct vib_gpio_data *vib_data, int on)
{
	dev_dbg(vib_data->dev.dev, "%s(%d)\n", __func__, on);

	mutex_lock(&(vib_data->io_mutex));

	if (on) {
		if (!vib_data->vib_power_state) {
			power_on(vib_data);
			vib_data->vib_power_state = 1;
		}
		if (vib_data->gpio >= 0)
			gpio_direction_output(vib_data->gpio,
					      vib_data->active_low ?  0 : 1);
	} else {
		if (vib_data->gpio >= 0)
			gpio_direction_output(vib_data->gpio,
					      vib_data->active_low ?  1 : 0);

		if (vib_data->vib_power_state) {
			power_off(vib_data);
			vib_data->vib_power_state = 0;
		}
	}

	mutex_unlock(&(vib_data->io_mutex));
}

static void vib_gpio_update(struct work_struct *work)
{
	struct vib_gpio_data *vib_data;

	vib_data = container_of(work, struct vib_gpio_data, vib_work);
	if (vib_data)
		vib_gpio_set(vib_data, vib_data->vib_state);
}

static enum hrtimer_restart gpio_timer_func(struct hrtimer *timer)
{
	struct vib_gpio_data *vib_data =
	    container_of(timer, struct vib_gpio_data, timer);
	dev_dbg(vib_data->dev.dev, "Timer expired: disabling vibrator\n");
	vib_data->vib_state = 0;
	schedule_work(&vib_data->vib_work);
	return HRTIMER_NORESTART;
}

static int vib_gpio_get_time(struct timed_output_dev *dev)
{
	struct vib_gpio_data *vib_data =
	    container_of(dev, struct vib_gpio_data, dev);

	if (hrtimer_active(&vib_data->timer)) {
		ktime_t r = hrtimer_get_remaining(&vib_data->timer);
		struct timeval t = ktime_to_timeval(r);
		return t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		return 0;
}

static void vib_gpio_enable(struct timed_output_dev *dev, int value)
{
	struct vib_gpio_data *vib_data =
	    container_of(dev, struct vib_gpio_data, dev);
	unsigned long flags;

	dev_dbg(dev->dev, "Enable vibrator for %dms\n", value);

	spin_lock_irqsave(&vib_data->lock, flags);
	hrtimer_cancel(&vib_data->timer);

	if (value == 0)
		vib_data->vib_state = 0;
	else {
		value = (value > vib_data->max_timeout ?
				 vib_data->max_timeout : value);
		vib_data->vib_state = 1;
		hrtimer_start(&vib_data->timer,
			      ktime_set(value / 1000, (value % 1000) * 1000000),
			      HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&vib_data->lock, flags);

	schedule_work(&vib_data->vib_work);
}

static int vib_gpio_probe(struct platform_device *pdev)
{
	struct vib_gpio_data *vib_data;
	struct device_node *np;
	unsigned int prop;
	int ret = 0;

	vib_data = kzalloc(sizeof(struct vib_gpio_data), GFP_KERNEL);
	if (!vib_data) {
		ret = -ENOMEM;
		goto err;
	}

	mutex_init(&(vib_data->io_mutex));

	platform_set_drvdata(pdev, vib_data);

	vib_data->gpio = -1;
	vib_data->active_low = 0;
	vib_data->initial_vibrate = 0;
	vib_data->max_timeout = 1500;
	vib_data->reg = NULL;
#ifdef CONFIG_OF
	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "required device_tree entry not found\n");
		goto free_mem;
	}

	if (!of_property_read_u32(np, "gpio", &prop))
		vib_data->gpio = prop;

	if (!of_property_read_u32(np, "max-timeout", &prop))
		vib_data->max_timeout = prop;

	if (!of_property_read_u32(np, "active-low", &prop))
		vib_data->active_low = prop;

	if (!of_property_read_u32(np, "initial-vibrate", &prop))
		vib_data->initial_vibrate = prop;

#endif
	vib_data->reg = regulator_get(&pdev->dev, "vib-gpio");
	if (IS_ERR(vib_data->reg)) {
		ret = PTR_ERR(vib_data->reg);
		goto free_mem;
	}

	INIT_WORK(&vib_data->vib_work, vib_gpio_update);

	hrtimer_init(&vib_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	vib_data->timer.function = gpio_timer_func;
	spin_lock_init(&vib_data->lock);

	vib_data->dev.name = "vibrator";
	vib_data->dev.get_time = vib_gpio_get_time;
	vib_data->dev.enable = vib_gpio_enable;
	ret = timed_output_dev_register(&vib_data->dev);
	if (ret < 0)
		goto reg_put;

	if (vib_data->gpio >= 0)
		gpio_direction_output(vib_data->gpio,
				      vib_data->active_low);

	vib_gpio_enable(&vib_data->dev, vib_data->initial_vibrate);

	pr_info("vib gpio probe done");
	return 0;

reg_put:
	regulator_put(vib_data->reg);
	mutex_destroy(&(vib_data->io_mutex));
free_mem:
	kfree(vib_data);
err:
	return ret;
}

static int vib_gpio_remove(struct platform_device *pdev)
{
	struct vib_gpio_data *vib_data = platform_get_drvdata(pdev);

	timed_output_dev_unregister(&vib_data->dev);
	regulator_put(vib_data->reg);
	mutex_destroy(&(vib_data->io_mutex));
	kfree(vib_data);

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id vib_gpio_of_match[] = {
	{ .compatible = "mot,vib-gpio" },
	{ }, };
MODULE_DEVICE_TABLE(of, vib_gpio_of_match);
#endif

static struct platform_driver vib_gpio_driver = {
	.probe = vib_gpio_probe,
	.remove = vib_gpio_remove,
	.driver = {
		   .name = "vib-gpio",
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = of_match_ptr(vib_gpio_of_match),
#endif
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

late_initcall(vib_gpio_init);
module_exit(vib_gpio_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("vib gpio driver");
MODULE_LICENSE("GPL");
