/*
 * drivers/media/video/hp3a/hp3a.c
 *
 * HP Imaging/3A Driver : Driver implementation for OMAP ISP 3A
 *				functionality.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <media/v4l2-dev.h>

#include "hp3a.h"
#include "hp3a_common.h"

#define	HP3A_DRV_NAME	"hp3a"
#define	HP3A_DRV_SYSFS	"hp3a-omap"

/**
 * Global variables.
 **/
static int hp3a_major = -1;
static struct hp3a_dev *g_device;

/**
 * hp3a_open - Initializes and opens the hp3a device
 * @inode: Inode structure associated with the hp3a driver
 * @filp: File structure associated with the hp3a driver
 *
 * Returns 0 if successful, -EBUSY if its already opened or the ISP module is
 * not available, or -ENOMEM if its unable to allocate the device in kernel
 * space memory.
 **/
static int hp3a_open(struct inode *inode, struct file *file)
{
	struct hp3a_fh *fh;

	dev_info(g_device->dev , "open\n");

	fh = kzalloc(sizeof(struct hp3a_fh), GFP_KERNEL);
	if (unlikely(fh == NULL))
		return -ENOMEM;

	fh->v4l2_dev = -1;
	fh->device = g_device;

	hp3a_framework_start(fh);

	/* Save context in file handle. */
	file->private_data = fh;

	return 0;
}

/**
 * hp3a_release - Releases hp3a device and frees up allocated memory
 * @inode: Inode structure associated with the hp3a driver
 * @filp: File structure associated with the hp3a driver
 *
 * Returns 0 if successful, or -EBUSY if channel is being used.
 **/
static int hp3a_release(struct inode *inode, struct file *file)
{
	struct hp3a_fh *fh = file->private_data;

	dev_info(g_device->dev , "release\n");

	/* Stop framework. */
	hp3a_framework_stop(fh);

	/* Releasing session specific data. */
	file->private_data = NULL;
	kfree(fh);

	return 0;
}

/**
 * hp3a_mmap - Memory maps hp3a module.
 * @file: File structure associated with the hp3a driver
 * @vma: Virtual memory area structure.
 *
 * Returns 0 if successful, or returned value by the videobuf_mmap_mapper()
 * function
 **/
static int hp3a_mmap(struct file *file, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations hp3a_fops = {
	.owner = THIS_MODULE,
	.open = hp3a_open,
	.release = hp3a_release,
	.mmap = hp3a_mmap,
	.unlocked_ioctl = hp3a_unlocked_ioctl,
};

/**
 * hp3a_platform_release - Place holder
 * @device: Structure containing hp3a driver global information
 *
 * This is called when the reference count goes to zero.
 **/
static void hp3a_platform_release(struct device *device)
{
}

/**
 * hp3a_probe - Checks for device presence
 * @device: Structure containing details of the current device.
 *
 * Always returns 0.
 **/
static int __init hp3a_probe(struct platform_device *device)
{
	/* Initialize 3A task. */
	initialize_hp3a_framework(g_device);

	return 0;
}

/**
 * hp3a_remove - Handles the removal of the driver
 * @device: Structure containing details of the current device.
 *
 * Always returns 0.
 **/
static int hp3a_remove(struct platform_device *device)
{
	/* Deinitialize 3A task, this is a blocking call. */
	deinitialize_hp3a_framework();

	return 0;
}

static struct class *hp3a_class;
static struct platform_device hp3a_device = {
	.name = HP3A_DRV_NAME,
	.id = -1,
	.dev = {
		.release = hp3a_platform_release,
	}
};

static struct platform_driver hp3a_driver = {
	.probe = hp3a_probe,
	.remove = hp3a_remove,
	.driver = {
			.owner = THIS_MODULE,
			.name = HP3A_DRV_NAME,
	},
};

/**
 *
 * Driver initialization routine.
 *
 **/
static int __init hp3a_drv_init(void)
{
	int ret;
	struct hp3a_dev *device;

	device = kzalloc(sizeof(struct hp3a_dev), GFP_KERNEL);
	if (!device) {
		dev_err(0 , HP3A_DRV_NAME ": could not allocate memory\n");
		return -ENOMEM;
	}

	device->dev = NULL;

	hp3a_major = register_chrdev(0, HP3A_DRV_SYSFS, &hp3a_fops);
	if (hp3a_major < 0) {
		dev_err(device->dev , "initialization failed. could"
				" not register character device\n");
		ret = -ENODEV;
		goto exit_error_1;
	}

	/* Register driver as a platform driver */
	ret = platform_driver_register(&hp3a_driver);
	if (ret) {
		dev_err(device->dev , "Failed to register platform driver!\n");
		goto exit_error_2;
	}

	/* Register the drive as a platform device */
	ret = platform_device_register(&hp3a_device);
	if (ret) {
		dev_err(device->dev , "Failed to register platform device!\n");
		goto exit_error_3;
	}

	hp3a_class = class_create(THIS_MODULE, HP3A_DRV_NAME);
	if (!hp3a_class) {
		dev_err(device->dev , "Failed to create class!\n");
		goto exit_error_4;
	}

	/* make entry in the devfs */
	device->dev = device_create(hp3a_class, device->dev,
		MKDEV(hp3a_major, 0), NULL, HP3A_DRV_SYSFS);

	dev_info(device->dev , "Registered hp3a driver.\n");

	/* hp3a global data initialization. */
	g_tc.initialized = 0;
	g_tc.hw_initialized = 0;
	/* Save device params for later use. */
	g_device = device;
	/* Returning success. */
	return 0;

exit_error_4:
	platform_device_unregister(&hp3a_device);
exit_error_3:
	platform_driver_unregister(&hp3a_driver);
exit_error_2:
	unregister_chrdev(hp3a_major,  HP3A_DRV_SYSFS);
	hp3a_major = -1;
exit_error_1:
	kfree(device);
	return ret;
}

/**
 *	hp3a exit routine.
 **/
static void __exit hp3a_drv_exit(void)
{
	class_destroy(hp3a_class);
	platform_device_unregister(&hp3a_device);
	platform_driver_unregister(&hp3a_driver);
	unregister_chrdev(hp3a_major,  HP3A_DRV_NAME);
	kfree(g_device);
	hp3a_major = -1;
}

module_init(hp3a_drv_init);
module_exit(hp3a_drv_exit);

/* module specific information */
MODULE_AUTHOR("Hewlett-Packard Co.");
MODULE_DESCRIPTION("HP Imaging/3A Interface Module Library");
MODULE_LICENSE("GPL");
