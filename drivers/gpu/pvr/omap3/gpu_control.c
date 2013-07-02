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
#include <linux/clk.h>
#include "sysconfig.h"

#define GPU_CONTROL_VERSION 1

/*
 * Enable or disable gpu_control:
 */
static bool gpu_control_active = true;

/*
 * Maximum possible frequency:
 */
static int max_freq_val = SYS_SGX_CLOCK_SPEED;

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

void gpu_set_clock(int freq)
{
	int ret;
	struct clk *sgx_fck;

	if (!gpu_control_active) {
		printk("GPU Control is disabled\n");
		return;
	}

	sgx_fck = clk_get(NULL, "sgx_fck");

	ret = clk_set_rate(sgx_fck, freq);

	if (ret) {
		max_freq_val = SYS_SGX_CLOCK_SPEED;
		clk_set_rate(sgx_fck, SYS_SGX_CLOCK_SPEED);
		pr_err("GPU clock rate change failed: %d. Default value restored %d MHz\n", ret, max_freq_val/1000000);
	} else {
		printk("GPU clock changed to: %lu MHz \n", clk_get_rate(sgx_fck)/1000000);
	}

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

	if (val <= 300)
		max_freq_val = val*1000000;

	gpu_set_clock(max_freq_val);
	return count;
}

define_one_global_rw(max_freq);

static void gpu_control_early_suspend(struct early_suspend *h)
{
	int suspend_freq = 100000000; //100 MHz
	gpu_set_clock(suspend_freq);
}

static void gpu_control_late_resume(struct early_suspend *h)
{
	gpu_set_clock(max_freq_val);
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

	return ret;
}

static void __exit gpu_control_exit(void)
{
	unregister_early_suspend(&gpu_control_suspend_handler);
}

module_init(gpu_control_init);
module_exit(gpu_control_exit);


MODULE_AUTHOR("Alexander Christ <alex.christ@hotmail.de>, Nicholas Semendyaev <agent00791@gmail.com>");
MODULE_DESCRIPTION("'interactive gpu governor' - A gpufreq governor for "
	"Latency sensitive workloads");
MODULE_LICENSE("GPL");
