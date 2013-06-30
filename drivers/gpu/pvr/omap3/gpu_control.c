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

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/earlysuspend.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <linux/writeback.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#define GPU_CONTROL_VERSION 1

#if defined(SGX530) && (SGX_CORE_REV == 125)
#define SYS_SGX_CLOCK_SPEED		200000000
#define SYS_SGX_CLOCK_SPEED_OC		220000000
#else
#define SYS_SGX_CLOCK_SPEED		110666666
#endif

#define SYS_SGX_CLOCK_SPEED_SUSPEND	50000000

/*
 * Enable or disable gpu_control:
 */
static bool gpu_control_active = true;

/*
 * Set the Default Clock; 200mhz
 */
static int gpu_freq = SYS_SGX_CLOCK_SPEED;

static int current_freq;

static bool suspend_state;

struct miscdevice gpu_control_device =
{
	.minor 	= MISC_DYNAMIC_MINOR,
	.name 	= "gpu_control",
};

int gpu_main_control(int gpu_tmp)
{
	// If we boot up the device, we want to have some sort of default clock;
	if (!gpu_tmp && !suspend_state) {
		gpu_tmp = SYS_SGX_CLOCK_SPEED;
		gpu_freq = gpu_tmp;
	}
	else {
		if(current_freq)
			gpu_freq = current_freq;
	}

	if (suspend_state)
		gpu_freq = SYS_SGX_CLOCK_SPEED_SUSPEND;
	else
		gpu_freq = current_freq;
			
	printk("!! GPU FREQ IS NOW %u \n", gpu_freq);	
	
	return gpu_freq;
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

static ssize_t show_gpu_freq(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", current_freq);
}

static ssize_t store_gpu_freq(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    	int ret;
	long unsigned int val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	current_freq = val;
	return size;
}

static ssize_t gpu_control_status_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", (gpu_control_active ? 1 : 0));
}

static ssize_t gpu_control_status_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if(sscanf(buf, "%u\n", &data) == 1) 
	{
	    if (data == 1) 
		{
		    pr_info("%s: GPUCONTROL enabled\n", __FUNCTION__);

		    gpu_control_active = true;

		} 
	    else if (data == 0) 
		{
		    pr_info("%s: GPUCONTROL disabled\n", __FUNCTION__);

		    gpu_control_active = false;
		} 
	    else 
		{
		    pr_info("%s: invalid input range %u\n", __FUNCTION__, data);
		}
	} 
    else 
	{
	    pr_info("%s: invalid input\n", __FUNCTION__);
	}

    return size;
}

static ssize_t gpu_control_version(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", GPU_CONTROL_VERSION);
}

static DEVICE_ATTR(gpu_control_active, S_IRUGO | S_IWUGO, gpu_control_status_read, gpu_control_status_write);
static DEVICE_ATTR(gpu_current_freq, S_IRUGO | S_IWUGO, show_gpu_freq, store_gpu_freq);
static DEVICE_ATTR(version, S_IRUGO , gpu_control_version, NULL);

static struct attribute *gpu_control_attributes[] = 
    {
	&dev_attr_gpu_control_active.attr,
	&dev_attr_gpu_current_freq.attr,
	&dev_attr_version.attr,
	NULL
    };

static struct attribute_group gpu_control_group = 
    {
	.attrs  = gpu_control_attributes,
    };


static int __init gpu_control_init(void)
{
	int ret;

	register_early_suspend(&gpu_control_suspend_handler);
	
	ret = misc_register(&gpu_control_device);
	
	current_freq = gpu_freq;

	// Basic init with current default freq;
	gpu_main_control(current_freq);
	
	if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, gpu_control_device.name);
	    return 1;
	}

	if (sysfs_create_group(&gpu_control_device.this_device->kobj, &gpu_control_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", gpu_control_device.name);
	}
	
	return 0;
}

device_initcall(gpu_control_init);
