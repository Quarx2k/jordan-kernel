/*
 *  Copyright (C) 2009 Motorola, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Adds ability to program periodic interrupts from user space that
 *  can wake the phone out of low power modes.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <plat/cpu.h>
#include <linux/proc_fs.h>
#include <linux/semaphore.h>
#include <linux/omap_dev_type.h>

#define OMAP_DEVICE_TYPE_HS_STRING  "HS"
#define OMAP_DEVICE_TYPE_NS_STRING  "NS"

static struct semaphore proc_read_lock;

static int omap_device_type_ioctl(struct inode *inode, struct file *filp,
				 unsigned int cmd, unsigned long arg)
{
	int ret;
	int dev_type;
	switch (cmd) {
	case OMAP_GET_DEVICE_TYPE_IOCTL:
		dev_type =  omap_type();
		printk(KERN_INFO "Device Type : %d\n", dev_type);
		ret = dev_type;
		break;
	default:
		printk(KERN_ERR"Invalid IOCTL Command\n");
		 ret = -EINVAL;
	}
	return ret;
}

static int omap_device_type_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int omap_device_type_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations omap_device_type_fops = {
	.owner = THIS_MODULE,
	.ioctl = omap_device_type_ioctl,
	.open  = omap_device_type_open,
	.release = omap_device_type_release,
};

static struct miscdevice omap_device_type_miscdrv = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "omap_dev_type",
	.fops = &omap_device_type_fops,
};

int omap_device_type_proc_read(char *buf, char **start, off_t offset, int count,
				int *eof, void *data)
{
	int len = 0;
	int type = 0;
	char *dev_type = NULL;

	if (down_interruptible(&proc_read_lock))
		return -ERESTARTSYS;

	switch (omap_type()) {
	case OMAP_DEVICE_TYPE_HS:
		dev_type = OMAP_DEVICE_TYPE_HS_STRING;
		break;
	case OMAP_DEVICE_TYPE_NS:
		dev_type = OMAP_DEVICE_TYPE_NS_STRING;
		break;
	default:
		dev_type = "Unknown";
	}

	len += sprintf(buf, "OMAP Device Type :%s\n", dev_type);
	up(&proc_read_lock);

	return len;
}

static int __init omap_device_type_init(void)
{
	int ret;
	struct proc_dir_entry *entry;

	ret = misc_register(&omap_device_type_miscdrv);
	if (ret < 0) {
		printk(KERN_ERR"Error registering omap_dev_type driver\n");
		goto error;
	}

	entry = create_proc_read_entry("omap_dev_type", 0 , NULL,
				    omap_device_type_proc_read, NULL);

	if (!entry) {
		printk(KERN_ERR"Error creating omep_dev_type proc entry\n");
		ret = -1;
		goto unregister_driver;
	}

	sema_init(&proc_read_lock, 1);
	return 0;

unregister_driver:
	misc_deregister(&omap_device_type_miscdrv);
error:
	return ret;
}


static void __exit omap_device_type_exit(void)
{
	remove_proc_entry("omap_dev_type", NULL);
	misc_deregister(&omap_device_type_miscdrv);
}

module_init(omap_device_type_init);
module_exit(omap_device_type_exit);

MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL")
MODULE_DESCRIPTION("Motorola OMAP Device Type");
