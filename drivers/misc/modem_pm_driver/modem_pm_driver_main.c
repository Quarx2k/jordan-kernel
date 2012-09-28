/*
 * Copyright (C) 2009, Motorola, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 * Motorola 2009-Sep-08 - Bug fix for frequency constraint
 * Motorola 2009-Jul-13 - Update for K29 to use Resource Framework
 * Motorola 2009-Jan-28 - Initial Creation
 */

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/semaphore.h>
#include <plat/omap34xx.h>
#include <plat/resource.h>
#include "modem_pm_driver.h"

static char modem_pm_driver_opened;

static struct device my_device;

static struct class *modem_pm_driver_class;
static struct device *modem_pm_driver_dev;

static int modem_pm_driver_major_num;

static DECLARE_MUTEX(modem_pm_driver_lock);


static int modem_pm_driver_init(void);

static void modem_pm_driver_exit(void);

static int modem_pm_driver_open(struct inode *inode, struct file *file);

static int modem_pm_driver_free(struct inode *inode, struct file *file);

static int modem_pm_driver_ioctl(struct inode *inode,
				 struct file *file,
				 unsigned int cmd,
				 unsigned long arg);

/* This structure defines the file operations for the Modem PM Driver */
static const struct file_operations modem_pm_driver_fops = {
	.owner =    THIS_MODULE,
	.ioctl =    modem_pm_driver_ioctl,
	.open =     modem_pm_driver_open,
	.release =  modem_pm_driver_free
};

static int modem_pm_driver_init(void)
{
	int retval = 0;

	/* Register the character device. */
	modem_pm_driver_major_num =
		register_chrdev(0, MODEM_PM_DRIVER_DEV_NAME,
				&modem_pm_driver_fops);

	/* If the character device is registered, continue. */
	if (modem_pm_driver_major_num >= 0) {
		/* Make the character device. */
		modem_pm_driver_class = class_create(THIS_MODULE,
						     MODEM_PM_DRIVER_DEV_NAME);

		if (IS_ERR(modem_pm_driver_class)) {
			unregister_chrdev(modem_pm_driver_major_num,
					  MODEM_PM_DRIVER_DEV_NAME);
			retval = -EFAULT;
		} else {
			modem_pm_driver_dev =
				device_create(modem_pm_driver_class,
					      NULL,
					      MKDEV(modem_pm_driver_major_num,
						    0),
					      NULL,
					      MODEM_PM_DRIVER_DEV_NAME);

			if (IS_ERR(modem_pm_driver_dev)) {
				class_destroy(modem_pm_driver_class);
				unregister_chrdev(modem_pm_driver_major_num,
						  MODEM_PM_DRIVER_DEV_NAME);
				retval = -EFAULT;
			}
		}
	}
	/* If we failed to get a character device, return the error. */
	else
		retval = modem_pm_driver_major_num;

	return retval;
}

static void modem_pm_driver_exit(void)
{
	/* Unregister the character device */
	device_destroy(modem_pm_driver_class,
		       MKDEV(modem_pm_driver_major_num, 0));
	class_destroy(modem_pm_driver_class);
	unregister_chrdev(modem_pm_driver_major_num, MODEM_PM_DRIVER_DEV_NAME);
}

static int modem_pm_driver_open(struct inode *inode, struct file *file)
{
	int retval = -EINTR;

	/* Acquire the mutex */
	if (down_interruptible(&modem_pm_driver_lock) == 0) {
		/* Check if it is not already open */
		if (modem_pm_driver_opened == 0) {
			/* Set the opened state */
			modem_pm_driver_opened = 1;
			retval = 0;
		} else
			retval = -EBUSY;

		/* Release the mutex */
		up(&modem_pm_driver_lock);
	}

	return retval;
}

static int modem_pm_driver_free(struct inode *inode, struct file *file)
{
	int retval = -EINTR;

	/* Acquire the mutex */
	if (down_interruptible(&modem_pm_driver_lock) == 0) {
		modem_pm_driver_opened = 0;
		retval = 0;

		/* Remove the setting of the VDD2 OPP constraint */
		resource_release("vdd2_opp", &my_device);

		/* Remove the setting of the latency constraint */
		resource_release("core_latency", &my_device);

		/* Release the mutex */
		up(&modem_pm_driver_lock);
	}

	return retval;
}

static int modem_pm_driver_ioctl(struct inode *inode,
				 struct file *file,
				 unsigned int cmd,
				 unsigned long arg)
{
	int status = -EFAULT;

	switch (cmd) {
	case MODEM_PM_DRIVER_IOCTL_HANDLE_FREQUENCY_OPP_CONSTRAINT: {
		switch (arg) {
		case MODEM_PM_SHARED_DDR_FREQUENCY_OPP_HIGH: {
			/* Set the target value of the VDD2 OPP constraint  */
			/* Make sure the interconnect is at 150Mhz or above */
			/* Throughput in KiB/s for 150 Mhz = 150 * 1000 * 4 */
			resource_request("vdd2_opp", &my_device, 600000);
			status = 0;
			break;
		}
		case MODEM_PM_SHARED_DDR_FREQUENCY_OPP_NO_VOTE: {
			/* Remove the setting of the VDD2 OPP constraint */
			resource_release("vdd2_opp", &my_device);
			status = 0;
			break;
		}
		default:
			break;
		}
		break;
	}
	case MODEM_PM_DRIVER_IOCTL_HANDLE_LOW_POWER_POLICY_CONSTRAINT: {
		switch (arg) {
		case MODEM_PM_SHARED_DDR_LOW_POWER_POLICY_ON_INACTIVE: {
			/* Set the target value of the latency constraint */
			resource_request("core_latency", &my_device, 9999);
			status = 0;
			break;
		}
		case MODEM_PM_SHARED_DDR_LOW_POWER_POLICY_RET: {
			/* Set the target value of the latency constraint */
			resource_request("core_latency", &my_device, 39999);
			status = 0;
			break;
		}
		case MODEM_PM_SHARED_DDR_LOW_POWER_POLICY_NO_VOTE: {
			/* Remove the setting of the latency constraint */
			resource_release("core_latency", &my_device);
			status = 0;
			break;
		}
		default:
			break;
		}
		break;
	}
	default:
		break;
	}

	return status;
}

/* Module entry points */
module_init(modem_pm_driver_init);
module_exit(modem_pm_driver_exit);

MODULE_DESCRIPTION("Modem PM Driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");

