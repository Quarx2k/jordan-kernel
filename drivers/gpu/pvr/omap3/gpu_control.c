/*
 * Author: Alexander Christ aka Blechd0se <alex.christ@hotmail.com>
 *
 * Copyright 2013 Alexander Christ
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

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/earlysuspend.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/cpufreq.h>

#define GPU_CONTROL_VERSION 1

#if defined(SGX530) && (SGX_CORE_REV == 125)
#define SYS_SGX_CLOCK_SPEED		200000000
#else
#define SYS_SGX_CLOCK_SPEED		110666666
#endif

/*
 * Enable or disable gpu_control:
 */
static bool gpu_control_active = true;

/*
 * Our current frequency:
 */
static int current_freq;

/*
 * Locks freq while screen is off to this value;
 */
static int suspend_freq = 5000;

/*
 * Maximum possible frequency:
 */
static int max_freq_val = 2000;

/*
 * Check if suspended or not:
 */
static bool suspend_state = false;


static ssize_t show_gpu_control_active(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", (gpu_control_active ? 1 : 0));
}

static ssize_t store_gpu_control_active(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	unsigned int data;

	if(sscanf(buf, "%u\n", &data) == 1) {
		if (data == 1) {
			pr_info("%s: gpu control enabled\n", __FUNCTION__);
			gpu_control_active = true;
		}
		else if (data == 0) {
			pr_info("%s: gpu control disabled\n", __FUNCTION__);
			gpu_control_active = false;
		}
		else
			pr_info("%s: bad value: %u\n", __FUNCTION__, data);
	} else
		pr_info("%s: unknown input!\n", __FUNCTION__);

	return count;
}


static ssize_t show_max_freq(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", max_freq_val);
}

static ssize_t store_max_freq(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	if (val <= 2500)
		max_freq_val = val;

	pr_info("gpu control overclocked to: %u \n", max_freq_val * 100000);

	return count;
}

define_one_global_rw(max_freq);

int gpu_main_control(int gpu_tmp)
{
	int ret;
	
	// Is it even active?
	if (gpu_control_active) {
		if (suspend_state) {
			ret = suspend_freq * 10000;
		}
		else {
			// Check if overclocked value is our default value;
			if ((max_freq_val * 100000) != current_freq)
				ret = max_freq_val * 100000;
			else
				ret = current_freq;
		}

		printk("!! GPU FREQ IS NOW AT %u \n", ret);
		return ret;
	} 
	else {
	
	ret = SYS_SGX_CLOCK_SPEED;
	
	return ret;
	}
}
EXPORT_SYMBOL(gpu_main_control);

static void gpu_control_early_suspend(struct early_suspend *h)
{
	if (gpu_control_active) {
		suspend_state = true;
	}
}

static void gpu_control_late_resume(struct early_suspend *h)
{
	if (gpu_control_active) {
		suspend_state = false;
	}
}

static struct early_suspend gpu_control_suspend_handler  = {
	.suspend   = gpu_control_early_suspend,
	.resume    = gpu_control_late_resume,
};


static struct kobj_attribute gpu_control_active_attribute = 
	__ATTR(gpu_control_active, 0666,
		show_gpu_control_active,
		store_gpu_control_active);


static struct attribute *gpu_control_attributes[] = 
    {
	&gpu_control_active_attribute.attr,
	&max_freq.attr,
	NULL
    };

static struct attribute_group gpu_control_group = 
    {
	.attrs  = gpu_control_attributes,
    };

static struct kobject *gpu_control_kobj;


static int __init gpu_control_init(void)
{
	int ret;

	register_early_suspend(&gpu_control_suspend_handler);
	
	
	gpu_control_kobj = kobject_create_and_add("gpu_control", kernel_kobj);
	if (!gpu_control_kobj) {
		pr_err("%s gpu_control kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
        }
	
	ret = sysfs_create_group(gpu_control_kobj,
			&gpu_control_group);

        if (ret) {
		pr_info("%s gpu_control sysfs create failed!\n", __FUNCTION__);
		kobject_put(gpu_control_kobj);
	}

	current_freq = SYS_SGX_CLOCK_SPEED;

	// Basic init with current default freq;
	gpu_main_control(current_freq);

	return ret;
}

static void __exit gpu_control_exit(void)
{
	unregister_early_suspend(&gpu_control_suspend_handler);
}

module_init(gpu_control_init);
module_exit(gpu_control_exit);


MODULE_AUTHOR("Alexander Christ <alex.christ@hotmail.de>");
MODULE_DESCRIPTION("'interactive gpu governor' - A gpufreq governor for "
	"Latency sensitive workloads");
MODULE_LICENSE("GPL");
