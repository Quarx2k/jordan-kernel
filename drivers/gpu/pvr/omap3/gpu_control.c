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

// Engle, Add proc fs API, start
#include <asm/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/vmalloc.h>

struct proc_dir_entry *gpu_proc_root = NULL;
#define BUF_SIZE 128
// Engle, Add proc fs API, end

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

int gpu_set_clock(int freq)
{
	int ret;
	struct clk *sgx_fck;

	if (!gpu_control_active) {
		printk("GPU Control is disabled\n");
		return -ENOTSUPP;
	}

	sgx_fck = clk_get(NULL, "sgx_fck");
    printk("GPU Control try to set clock rate to %d MHz\n", freq/1000000);
	ret = clk_set_rate(sgx_fck, freq);

	if (ret) {
		//max_freq_val = SYS_SGX_CLOCK_SPEED;
		//clk_set_rate(sgx_fck, SYS_SGX_CLOCK_SPEED);
		pr_err("GPU clock rate change failed: %d. Default value restored %lu MHz\n", ret, clk_get_rate(sgx_fck)/1000000);
	} else {
		printk("GPU clock changed to: %lu MHz \n", clk_get_rate(sgx_fck)/1000000);
	}
	return ret;

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
		val = val*1000000;

	if (gpu_set_clock(val) == 0) {
		max_freq_val = val;
	}
	return count;
}

define_one_global_rw(max_freq);

static void gpu_control_early_suspend(struct early_suspend *h)
{
	int suspend_freq = SYS_SGX_SUSPEND_CLOCK_SPEED;
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

// Engle, Add proc fs API, start

static int proc_cur_rate_read(char *buffer, char **buffer_location,
		off_t offset, int count, int *eof, void *data)
{
	int ret = 0;
	struct clk *sgx_fck;

	if (offset > 0)
		ret = 0;
	else {
		sgx_fck = clk_get(NULL, "sgx_fck");
		if (sgx_fck) {
			ret = scnprintf(buffer, count, "%lu\n", clk_get_rate(sgx_fck));
		}
	}

	return ret;
}

static int proc_max_rate_read(char *buffer, char **buffer_location,
		off_t offset, int count, int *eof, void *data)
{
	int ret;

	if (offset > 0)
		ret = 0;
	else
		ret = scnprintf(buffer, count, "%u\n", max_freq_val);

	return ret;
}

static int proc_max_rate_write(struct file *filp, const char __user *buffer,
		unsigned long len, void *data)
{
	ulong newrate;
	int result;
	char gpu_proc_fs_buf[BUF_SIZE];

	if(!len || len >= BUF_SIZE)
		return -ENOSPC;
	if(copy_from_user(gpu_proc_fs_buf, buffer, len))
		return -EFAULT;
	gpu_proc_fs_buf[len] = 0;
	if((result = strict_strtoul(gpu_proc_fs_buf, 0, &newrate)))
		return result;
	if(max_freq_val != newrate) {
		if (gpu_set_clock(newrate) == 0) {
			max_freq_val = newrate;
		}
	}

	return len;
}

static void init_gpu_proc_fs(void) {
	struct proc_dir_entry *proc_entry;
	gpu_proc_root = proc_mkdir("gpu", NULL);
	proc_entry = create_proc_read_entry("gpu/cur_rate", 0644, NULL, proc_cur_rate_read, NULL);
	proc_entry = create_proc_read_entry("gpu/max_rate", 0644, NULL, proc_max_rate_read, NULL);
	proc_entry->write_proc = proc_max_rate_write;
}

static void exit_gpu_proc_fs(void) {
	remove_proc_entry("gpu/max_rate", NULL);
	remove_proc_entry("gpu/cur_rate", NULL);
	remove_proc_entry("gpu", NULL);
}
// Engle, Add proc fs API, end

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

    init_gpu_proc_fs();
	return ret;
}

static void __exit gpu_control_exit(void)
{
	unregister_early_suspend(&gpu_control_suspend_handler);
	exit_gpu_proc_fs();
}

module_init(gpu_control_init);
module_exit(gpu_control_exit);


MODULE_AUTHOR("Alexander Christ <alex.christ@hotmail.de>, Nicholas Semendyaev <agent00791@gmail.com>");
MODULE_DESCRIPTION("'interactive gpu governor' - A gpufreq governor for "
	"Latency sensitive workloads");
MODULE_LICENSE("GPL");
