/* drivers/misc/vib-pwm.c
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
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
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/vib-pwm.h>
#include <linux/workqueue.h>

#include "../staging/android/timed_output.h"

#define VIB_STATE_IDLE		0
#define VIB_STATE_SQUARE	1
#ifdef CONFIG_VIB_PWM_SWEEP
#define VIB_STATE_SWEEP		2
#endif /* CONFIG_VIB_PWM_SWEEP */

#define VIB_TYPE_SQUARE 0
#ifdef CONFIG_VIB_PWM_SWEEP
#define VIB_TYPE_SWEEP  1
#endif /* CONFIG_VIB_PWM_SWEEP */

struct vib_pwm_data {
	struct timed_output_dev dev;
	struct work_struct vib_work;
	struct hrtimer timer;
	/* Ensure only one vibration at a time */
	struct mutex lock;

	struct vib_pwm_platform_data *pdata;
	int vib_power_state;
	int vib_state;
	int value;
};

static struct vib_pwm_data *misc_data;

static void vib_pwm_set(struct vib_pwm_data *data)
{
	if (data->vib_state != VIB_STATE_IDLE) {
#ifdef CONFIG_VIB_PWM_SWEEP
		if ((data->vib_state == VIB_STATE_SWEEP) &&
			data->pdata->pattern) {
			if (data->vib_power_state != VIB_STATE_SWEEP) {
				data->pdata->pattern(1);
				data->vib_power_state = VIB_STATE_SWEEP;
			}
			hrtimer_start(&data->timer,
				ktime_set(data->value / 1000,
				(data->value % 1000) * 1000000),
				HRTIMER_MODE_REL);
			return;
		}
#endif /* CONFIG_VIB_PWM_SWEEP */
		if (data->pdata->power_on) {
			if (!data->vib_power_state) {
				data->pdata->power_on();
				data->vib_power_state = 1;
			}
			hrtimer_start(&data->timer,
				ktime_set(data->value / 1000,
				(data->value % 1000) * 1000000),
				HRTIMER_MODE_REL);
		}
	} else {
		if (data->pdata->power_off && data->vib_power_state) {
			data->pdata->power_off();
			data->vib_power_state = 0;
		}
	}
}

static void vib_pwm_update(struct work_struct *work)
{
	struct vib_pwm_data *data =
		container_of(work, struct vib_pwm_data, vib_work);
	mutex_lock(&data->lock);
	vib_pwm_set(data);
	mutex_unlock(&data->lock);
}

static enum hrtimer_restart pwm_timer_func(struct hrtimer *timer)
{
	struct vib_pwm_data *data =
	    container_of(timer, struct vib_pwm_data, timer);
	data->vib_state = 0;
	schedule_work(&data->vib_work);
	return HRTIMER_NORESTART;
}

static int vib_pwm_get_time(struct timed_output_dev *dev)
{
	int ret;
	struct vib_pwm_data *data = container_of(dev, struct vib_pwm_data, dev);

	mutex_lock(&data->lock);

	if (hrtimer_active(&data->timer)) {
		ktime_t r = hrtimer_get_remaining(&data->timer);
		struct timeval t = ktime_to_timeval(r);
		ret = t.tv_sec * 1000 + t.tv_usec / 1000;
	} else
		ret = 0;

	mutex_unlock(&data->lock);

	return ret;
}

static void vib_pwm_process_request(struct vib_pwm_data *data,
	int vib_type, int value)
{
	hrtimer_cancel(&data->timer);

	if (value == 0) {
		data->vib_state = VIB_STATE_IDLE;
		data->value = 0;
		vib_pwm_set(data);
	} else {
		if (vib_type == VIB_TYPE_SQUARE)
			data->vib_state = VIB_STATE_SQUARE;
#ifdef CONFIG_VIB_PWM_SWEEP
		else
			data->vib_state = VIB_STATE_SWEEP;
#endif /* CONFIG_VIB_PWM_SWEEP */
		data->value = value;
		schedule_work(&data->vib_work);
	}
}

static void vib_pwm_enable(struct timed_output_dev *dev, int value)
{
	struct vib_pwm_data *data = container_of(dev, struct vib_pwm_data, dev);

	mutex_lock(&data->lock);

#ifdef CONFIG_VIB_PWM_SWEEP
	if (data->vib_state == VIB_STATE_SWEEP)
		goto vib_pwm_enable_exit;
#endif /* CONFIG_VIB_PWM_SWEEP */

	vib_pwm_process_request(data, VIB_TYPE_SQUARE, value);

vib_pwm_enable_exit:
	mutex_unlock(&data->lock);
}

#ifdef CONFIG_VIB_PWM_SWEEP
static ssize_t sweep_show(struct device *dev, struct device_attribute *attr,
		char *buf)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	int remaining = tdev->get_time(tdev);

	return sprintf(buf, "%d\n", remaining);
}

static ssize_t sweep_store(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t size)
{
	struct timed_output_dev *tdev = dev_get_drvdata(dev);
	struct vib_pwm_data *data =
		container_of(tdev, struct vib_pwm_data, dev);
	int value;

	if (sscanf(buf, "%d", &value) == 0)
		return size;

	mutex_lock(&data->lock);

	vib_pwm_process_request(data, VIB_TYPE_SWEEP, value);

	mutex_unlock(&data->lock);

	return size;
}

static DEVICE_ATTR(sweep, S_IRUGO | S_IWUSR, sweep_show, sweep_store);
#endif /* CONFIG_VIB_PWM_SWEEP */

static int vib_pwm_probe(struct platform_device *pdev)
{
	struct vib_pwm_platform_data *pdata = pdev->dev.platform_data;
	struct vib_pwm_data *pwm_data;
	int ret = 0;

	if (!pdata) {
		ret = -EBUSY;
		printk(KERN_ERR "vib-pwm platform data is null\n");
		goto err0;
	}

	pwm_data = kzalloc(sizeof(struct vib_pwm_data), GFP_KERNEL);
	if (!pwm_data) {
		printk(KERN_ERR "vib-pwm: memory allocation failed\n");
		ret = -ENOMEM;
		goto err0;
	}

	pwm_data->pdata = pdata;

	INIT_WORK(&pwm_data->vib_work, vib_pwm_update);

	hrtimer_init(&pwm_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);

	pwm_data->timer.function = pwm_timer_func;
	mutex_init(&pwm_data->lock);

	pwm_data->dev.name = pdata->device_name;
	pwm_data->dev.get_time = vib_pwm_get_time;
	pwm_data->dev.enable = vib_pwm_enable;
	ret = timed_output_dev_register(&pwm_data->dev);
	if (ret < 0) {
		printk(KERN_ERR "timed o/p dev. lvib. registration failed\n");
		goto err1;
	}

	if (pwm_data->pdata->init)
		ret = pwm_data->pdata->init();
	if (ret < 0) {
		printk(KERN_ERR "vib-pwm: platform data not available\n");
		goto err2;
	}

#ifdef CONFIG_VIB_PWM_SWEEP
	ret = device_create_file(pwm_data->dev.dev, &dev_attr_sweep);
	if (ret < 0) {
		printk(KERN_ERR "lvib. sysfs creating file failed\n");
		goto err3;
	}
#endif /* CONFIG_VIB_PWM_SWEEP */

	misc_data = pwm_data;
	platform_set_drvdata(pdev, pwm_data);

	printk(KERN_ALERT "vib-pwm probed\n");
	return 0;
#ifdef CONFIG_VIB_PWM_SWEEP
err3:
	if (pwm_data->pdata->exit)
		pwm_data->pdata->exit();
#endif /* CONFIG_VIB_PWM_SWEEP */
err2:
	timed_output_dev_unregister(&pwm_data->dev);
err1:
	mutex_destroy(&pwm_data->lock);
	kfree(pwm_data);
err0:
	return ret;
}

static int vib_pwm_remove(struct platform_device *pdev)
{
	struct vib_pwm_data *pwm_data = platform_get_drvdata(pdev);

#ifdef CONFIG_VIB_PWM_SWEEP
	device_remove_file(pwm_data->dev.dev, &dev_attr_sweep);
#endif /* CONFIG_VIB_PWM_SWEEP */

	if (pwm_data->pdata->exit)
		pwm_data->pdata->exit();

	timed_output_dev_unregister(&pwm_data->dev);
	mutex_destroy(&pwm_data->lock);
	kfree(pwm_data);
	return 0;
}

static struct platform_driver vib_pwm_driver = {
	.probe = vib_pwm_probe,
	.remove = vib_pwm_remove,
	.driver = {
		   .name = VIB_PWM_NAME,
		   .owner = THIS_MODULE,
		   },
};

static int __init vib_pwm_init(void)
{
	printk(KERN_ERR "vib-pwm: vib_pwm_init\n");
	return platform_driver_register(&vib_pwm_driver);
}

static void __exit vib_pwm_exit(void)
{
	platform_driver_unregister(&vib_pwm_driver);
}

module_init(vib_pwm_init);
module_exit(vib_pwm_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("vib pwm driver");
MODULE_LICENSE("GPL");
