/* drivers/misc/writeback.c
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

#define WRITEBACK_VERSION 1

static bool writeback_enabled = true;

bool writeback(void)
{
	return writeback_enabled;
}

EXPORT_SYMBOL(writeback);

static ssize_t writeback_status_read(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", (writeback_enabled ? 1 : 0));
}

static ssize_t writeback_status_write(struct device * dev, struct device_attribute * attr, const char * buf, size_t size)
{
    unsigned int data;

    if(sscanf(buf, "%u\n", &data) == 1) 
	{
	    if (data == 1) 
		{
		    pr_info("%s: writeback enabled\n", __FUNCTION__);

		    writeback_enabled = true;

		} 
	    else if (data == 0) 
		{
		    pr_info("%s: Fwriteback disabled\n", __FUNCTION__);

		    writeback_enabled = false;
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

static ssize_t writeback_version(struct device * dev, struct device_attribute * attr, char * buf)
{
    return sprintf(buf, "%u\n", WRITEBACK_VERSION);
}

static DEVICE_ATTR(writeback_enabled, S_IRUGO | S_IWUGO, writeback_status_read, writeback_status_write);
static DEVICE_ATTR(version, S_IRUGO , writeback_version, NULL);

static struct attribute *writeback_attributes[] = 
    {
	&dev_attr_writeback_enabled.attr,
	&dev_attr_version.attr,
	NULL
    };

static struct attribute_group writeback_group = 
    {
	.attrs  = writeback_attributes,
    };

static struct miscdevice writeback_device = 
    {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "writeback",
    };

static int __init writeback_init(void)
{
    int ret;

    pr_info("%s misc_register(%s)\n", __FUNCTION__, writeback_device.name);

    ret = misc_register(&writeback_device);

    if (ret) 
	{
	    pr_err("%s misc_register(%s) fail\n", __FUNCTION__, writeback_device.name);
	    return 1;
	}

    if (sysfs_create_group(&writeback_device.this_device->kobj, &writeback_group) < 0) 
	{
	    pr_err("%s sysfs_create_group fail\n", __FUNCTION__);
	    pr_err("Failed to create sysfs group for device (%s)!\n", writeback_device.name);
	}

    return 0;
}

device_initcall(writeback_init);
