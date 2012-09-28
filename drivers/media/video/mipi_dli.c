/*
 * driver/media/video/mipi_dli.c
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/fs.h>		/* everything... */
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include "oldisp/ispreg.h"
#include <linux/uaccess.h>

#include <linux/mipi_dli.h>

struct mipi_dli_platform_data {
	struct mutex lock;		    /* Mutex lock */
	unsigned long FrameCount;    /* total frames after last reset */
	unsigned long ECCErrors;       /* ECC errors after last reset */
	unsigned long CRCErrors;       /* CRC errors after last reset */
};

/*Counters for MIPI DLI.*/
unsigned long frame_counter;
unsigned long ecc_counter;
unsigned long crc_counter;

static int mipi_dli_open(struct inode *inode, struct file *file)
{
	printk(KERN_ERR "%s is called.\n", __func__);
	return nonseekable_open(inode, file);
}

static int mipi_dli_release(struct inode *inode, struct file *file)
{
	printk(KERN_ERR "%s is called.\n", __func__);
	return 0;
}

static int mipi_dli_ioctl(struct inode *inode, struct file *file,\
				unsigned int cmd, unsigned long arg)
{
	unsigned long mipi_counter;
	void __user *argp = (void __user *)arg;

	int ret = 0;

	printk(KERN_ERR "mipi_dli_ioctl function start.\n");

	if (copy_from_user(&mipi_counter, argp, sizeof(mipi_counter)))
		return -EFAULT;

	printk(KERN_ERR "mipi_dli_ioctl operation start.\n");

	if (mipi_counter == 0) {
		/*Reset counter.*/
		switch (cmd) {
		case MIPI_DLI_IOCTL_FRAME_COUNT:
			mipi_counter = omap_readl(ISPCSI2_CTX_CTRL2(0));
			mipi_counter = (mipi_counter & 0xFFFF0000) >> 16;
			frame_counter = mipi_counter;
			break;

		case MIPI_DLI_IOCTL_ECC_COUNT:
			ecc_counter = 0;
			break;

		case MIPI_DLI_IOCTL_CRC_COUNT:
			crc_counter = 0;
			break;

		default:
			printk(KERN_ERR "Unexpected parameter.\n");
			ret = -EINVAL;
			break;
		}
	} else {
		/*Get counter.*/
		switch (cmd)	{
		case MIPI_DLI_IOCTL_FRAME_COUNT:
			mipi_counter = omap_readl(ISPCSI2_CTX_CTRL2(0));
			mipi_counter = (mipi_counter & 0xFFFF0000) >> 16;
			mipi_counter -= frame_counter;
			break;

		case MIPI_DLI_IOCTL_ECC_COUNT:
			mipi_counter = ecc_counter;
			break;

		case MIPI_DLI_IOCTL_CRC_COUNT:
			mipi_counter = crc_counter;
			break;

		default:
			printk(KERN_ERR "Unexpected parameter.\n");
			ret = -EINVAL;
			break;
		}

		if (ret != 0) {
			printk(KERN_ERR "mipi_dli cmd fail %d.\n", cmd);
			return -EFAULT;
		} else
			printk(KERN_ERR "mipi_counter = %ld.\n",\
				mipi_counter);

		if (copy_to_user(argp, &mipi_counter, sizeof(mipi_counter)))
			ret = -EFAULT;
	}

	return ret;
}

static const struct file_operations mipi_dli_fops = {
	.owner   = THIS_MODULE,
	.open    = mipi_dli_open,
	.release = mipi_dli_release,
	.ioctl   = mipi_dli_ioctl,
};

static struct miscdevice mipi_dli_device = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = MIPI_DLI_DEVICE_NAME,
  .fops = &mipi_dli_fops,
};


static int __init mipi_dli_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = misc_register(&mipi_dli_device);

	if (ret != 0)
		printk(KERN_ERR "misc_register failed\n");

	printk(KERN_ERR "MIPI DLI tester probe is finished.\n");

	return ret;
}

static int mipi_dli_remove(struct platform_device *pdev)
{
	return 0;
}

static void mipi_dli_shutdown(struct platform_device *pdev)
{
}

static struct platform_driver mipi_dli_driver = {
	.driver		= {
		.name	= "mipi_dli_tester",
	},
	.remove		= __devexit_p(mipi_dli_remove),
	.shutdown	= mipi_dli_shutdown,
};

static int __init mipi_dli_init(void)
{
	int retval = 0;
	retval = platform_driver_probe(&mipi_dli_driver, mipi_dli_probe);
	if (retval != 0) {
		printk(KERN_ERR "failed mipi dli register/probe %d\n", retval);
		return -ENODEV;
	}
	return retval;
}

module_init(mipi_dli_init);

MODULE_AUTHOR("Motorola Corporation");
MODULE_DESCRIPTION("MIPI DLI test Module");
MODULE_LICENSE("GPL");
