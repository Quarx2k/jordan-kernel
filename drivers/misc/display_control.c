/* drivers/misc/display_control
 *
 * Copyright 2012  Ezekeel
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *
 * Based on the FSync_control by Ezekeel, changed by Blechdose for Motorola Defy
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/miscdevice.h>

#define DISPLAY_CONTROL_VERSION 1

/* This should be improved for both display types */
u8 display_brightness_value = 0x1f;

u8 display_brightness(void)
{
	return display_brightness_value;
}

EXPORT_SYMBOL(display_brightness);

static ssize_t display_status_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", display_brightness_value);
}

static ssize_t display_status_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
        int ret;
        unsigned long val;
        
        ret = strict_strtoul(buf, 0, &val);

        if (ret < 0)
                return ret;

        display_brightness_value = val;

        return size;
}

static ssize_t display_control_version(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", DISPLAY_CONTROL_VERSION);
}

static DEVICE_ATTR(display_brightness_value, S_IRUGO | S_IWUGO, display_status_read, display_status_write);
static DEVICE_ATTR(version, S_IRUGO , display_control_version, NULL);

static struct attribute *display_control_attributes[] = 
    {
	&dev_attr_display_brightness_value.attr,
	&dev_attr_version.attr,
	NULL
    };

static struct attribute_group display_control_group = 
    {
	.attrs  = display_control_attributes,
    };

static struct miscdevice display_control_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "display_control",
    };

static int __init display_control_init(void)
{
    int ret;

    pr_info("%s misc_register(%s)\n", __FUNCTION__, display_control_device.name);

    ret = misc_register(&display_control_device);

    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, display_control_device.name);
	    return 1;
	}

    if (sysfs_create_group(&display_control_device.this_device->kobj, &display_control_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", display_control_device.name);
	}

    return 0;
}

device_initcall(display_control_init);
