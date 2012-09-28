/* 
 * GPIODev                                       gpiodev.c
 *
 * Copyright (C) 2008-2009 Motorola, Inc.
 */

/*
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 *
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/errno.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <mach/gpio.h>
#include <linux/autoconf.h>
#include <linux/gpiodev.h>
#include <linux/platform_device.h>

MODULE_AUTHOR("Motorola, Inc.");
MODULE_DESCRIPTION("GPIO device for user space accessing GPIO pins");
MODULE_LICENSE("GPL");

/*
 * GPIODEV_DBG macro is for debugging purpose only
 * Should turn it off while releasing official version
 */
#define GPIODEV_DBG 0

#if GPIODEV_DBG
#define trace_msg(fmt, args...)  printk("\n" KERN_ALERT fmt "\n", ##args)
#else
#define trace_msg(fmt, arg...) do { } while(0)
#endif

const unsigned int int_type[GPIODEV_INTTYPE_MAX] = {
	IRQF_TRIGGER_NONE,
	IRQF_TRIGGER_RISING,
	IRQF_TRIGGER_FALLING,
	(IRQF_TRIGGER_HIGH | IRQF_TRIGGER_RISING),
	(IRQF_TRIGGER_LOW | IRQF_TRIGGER_FALLING)
};

#define GET_INT_TYPE(a)    (int_type[((a) & GPIODEV_CONFIG_INT_MASK) >> GPIODEV_CONFIG_INT_MASK_OFFSET])
#define GPIODEV_IS_INTERRUPTABLE(a)    ((((a) & GPIODEV_CONFIG_INT_MASK) != GPIODEV_CONFIG_INT_NONE) ? 1 : 0)

static unsigned long gpiodev_major = 0;
static struct class *gpiodev_class;

static struct gpio_device *gpio_devs;
static int gpio_dev_count;

/*
 * gpiodev_isr handles an interrupt on a GPIO triggered by the parameters
 * provided when setting up the line. The interrupt that occurs here will
 * be for a specific GPIO previously configured for a particular interrupt.
 * The function will wake up any process waiting for this interrupt to be 
 * triggered.
 */
static irqreturn_t gpiodev_isr(int irq, void *param)
{
	struct gpio_device *dev;

	dev = (struct gpio_device *) param;

	dev->flags |= GPIODEV_FLAG_INTERRUPTED;

	disable_irq_nosync(gpio_to_irq(dev->pin_nr));
	wake_up_interruptible(&dev->event_queue);

	trace_msg("gpio%d interrupt occurs", dev->pin_nr);

	return IRQ_HANDLED;
}

/*
 * config_gpio configures the way a GPIO operates and adjusts the
 * current configuration byte appropriately. Any resources in use
 * prior to the reconfiguration will be released and the new configuration
 * will take effect.
 */
static unsigned long config_gpio(struct gpio_device *dev, u32 newconfig)
{
	unsigned long ret = 0;

	trace_msg("Configure gpio%d with config 0x%04x instead of 0x%04x",
		  dev->pin_nr, newconfig, dev->current_config);


	/* If new conig is invalid, release gpio resource */
	if ( newconfig & GPIODEV_CONFIG_INVALID ) {
		/* Free any currently consumed resources */
		if (!(dev->current_config & GPIODEV_CONFIG_INVALID)) {
			if (GPIODEV_IS_INTERRUPTABLE(dev->current_config)) {
				free_irq(gpio_to_irq(dev->pin_nr), dev);
				dev->flags &= ~GPIODEV_FLAG_INTERRUPTED;
			}
			gpio_free(dev->pin_nr);
		}
	}
	else {
		/* If current gpio is invalid, request gpio resource*/
		if (dev->current_config & GPIODEV_CONFIG_INVALID) {
			if (gpio_request(dev->pin_nr, NULL) < 0) {
				printk(KERN_ERR "Failed to request GPIO for %s \n",
			    	   dev->device_name);
				ret = -EBUSY;
				goto end;
			}
		}

		/* 
		 * Configure GPIO with newconfig
		 */
		/* Step 1: direction and level setting */
		if (newconfig & GPIODEV_CONFIG_INPUT) {
			gpio_direction_input(dev->pin_nr);
		} 
		else if (newconfig & GPIODEV_CONFIG_OUTPUT_HIGH) {
			gpio_direction_output(dev->pin_nr, 1);
		}
		else if (newconfig & GPIODEV_CONFIG_OUTPUT_LOW) {
			gpio_direction_output(dev->pin_nr, 0);
		}
		/* Step 2: request interrupt */
		if (GPIODEV_IS_INTERRUPTABLE(newconfig)) {
			if (((newconfig & GPIODEV_CONFIG_INT_MASK) >> GPIODEV_CONFIG_INT_MASK_OFFSET)
				>= GPIODEV_INTTYPE_MAX) {
				ret = -EINVAL;
				goto end;
			}
			else if (request_irq(gpio_to_irq(dev->pin_nr),
					     &gpiodev_isr,
					     GET_INT_TYPE(newconfig),
					     dev->device_name, dev) ) {
				printk(KERN_ERR "Fail to request irq for \"%s\"\n", 
						dev->device_name);
				gpio_free(dev->pin_nr);
				ret = -EBUSY;
				goto end;
			}
		}
	}

	dev->current_config = newconfig;
end:
	return ret;
}


/*
 * gpiodev_open handles a userspace open() to our driver. This function takes
 * the specified device out of the default state by configuring the GPIO.
 */
static int gpiodev_open(struct inode *inode, struct file *filp)
{
	struct gpio_device *dev;
	unsigned long minor;
	int ret = 0;

	minor = MINOR(inode->i_rdev);

	if (minor >= gpio_dev_count)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (dev->flags & GPIODEV_FLAG_OPEN) {
		ret = -EBUSY;
		goto end;
	}

#if GPIODEV_DBG
	if (dev->pin_nr == 111) {
		free_irq(gpio_to_irq(dev->pin_nr), NULL);
		gpio_free(dev->pin_nr);
	}
#endif

	ret = config_gpio(dev, dev->init_config);
	if (ret != 0) 
		goto end;

	dev->flags |= GPIODEV_FLAG_OPEN;
	if (!try_module_get(THIS_MODULE)) {
		ret = -EINVAL;
		module_put(THIS_MODULE);
		goto end;
	}

end:
	mutex_unlock(&dev->lock);
	trace_msg("open function return value 0x%04x", ret);
	return ret;
}

/*
 * gpiodev_close handles a userspace close() on a device previously
 * opened. The close reverses any initialization done on a device.
 */
static int gpiodev_close(struct inode *inode, struct file *filp)
{
	struct gpio_device *dev;
	unsigned long minor;
	int ret = 0;

	minor = MINOR(inode->i_rdev);

	if (minor >= gpio_dev_count)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)) {
		ret = 0;
		goto end;
	}

	if (dev->flags & GPIODEV_FLAG_CONFIGURABLE) {
		ret = config_gpio(dev, GPIODEV_CONFIG_INVALID);
		if (ret != 0)
			goto end;
	}

	dev->flags &= ~GPIODEV_FLAG_OPEN;
	module_put(THIS_MODULE);

end:
	mutex_unlock(&dev->lock);
	trace_msg("close function return value 0x%04x", ret);
	return ret;
}

/*
 * gpiodev_read handles a userspace read() on an open device.
 * The read will only read a single byte. A read on a GPIO
 * configured for output will return the value currently in the output
 * register.
 */
static ssize_t gpiodev_read(struct file *filp, char *buf, size_t count,
			    loff_t * f_pos)
{
	struct gpio_device *dev;
	unsigned long minor;
	unsigned long result;
	unsigned char value;
	int ret = 1;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (minor >= gpio_dev_count)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)) {
		ret = -EBADF;
		goto end;
	}
	else if ((dev->current_config & GPIODEV_CONFIG_OUTPUT_LOW) 
			|| (dev->current_config & GPIODEV_CONFIG_OUTPUT_HIGH)) {
		ret = -EPERM;
		goto end;
	}

	if (count < 1) {
		ret = 0;
		goto end;
	}

	value = gpio_get_value(dev->pin_nr);
	trace_msg("Reading gpio%d with 0x%02x", dev->pin_nr, value);

	result = copy_to_user(buf, &value, 1);
	if (result)
		ret = -EFAULT;

end:
	mutex_unlock(&dev->lock);
	return ret;
}

/*
 * gpiodev_write handles a userspace write() on an open device. The write
 * must be one byte in length or the call will fail. A write on a GPIO
 * configured for input will not affect the line's status.
 */
static ssize_t gpiodev_write(struct file *filp, const char *buf, size_t count,
			     loff_t * f_pos)
{
	struct gpio_device *dev;
	unsigned long minor;
	unsigned long result;
	unsigned char value;
	int ret = 1;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (minor >= gpio_dev_count)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)) {
		ret = -EBADF;
		goto end;
	}
	else if (dev->current_config & GPIODEV_CONFIG_INPUT) {
		ret = -EPERM;
		goto end;
	}

	if (count < 1) {
		ret = 0;
		goto end;
	}

	value = 0;

	result = copy_from_user(&value, buf, 1);
	if (result) {
		ret = -EFAULT;
		goto end;
	}

	if (value > 1) {
		ret = -EINVAL;
		goto end;
	}

	gpio_set_value(dev->pin_nr, value);
	trace_msg("Writing gpio%d with 0x%02x", dev->pin_nr, value);

end:
	mutex_unlock(&dev->lock);
	trace_msg("write function return value = 0x%04x", ret);
	return ret;
}

/*
 * gpiodev_ioctl handles the configuration of a GPIO. The following commands are
 * supported:
 * GPIODEV_GET_CONFIG - returns the 1 byte configuration of the GPIO
 * GPIODEV_SET_CONFIG - sets up the 1 byte configuration of the GPIO
 * GPIODEV_INT_REENABLE - If the line is responsive to interrupts, ints will be
 *                        reenabled.
 * GPIODEV_GET_LOWLEVELCONFIG - Get the low level details of a device. If the device doesn't
 *                              allow this operation an error is returned.
 * GPIODEV_SET_LOWLEVELCONFIG - Set the low level details of a device. If the device
 *                              doesn't allow this operation an error is returned. If
 *                              the low level configuration to be set is invalid an
 *                              error is returned.
 * GPIODEV_INT_POLL - Wait for gpiodev interrupt occurrence.
 *
 */
static int gpiodev_ioctl(struct inode *inode, struct file *filp,
			 unsigned int cmd, unsigned long arg)
{
	GPIODEV_LOWLEVEL_CONFIG llconf;
	struct gpio_device *dev;
	unsigned long minor;
	unsigned long result;
	u32 value;
	int ret = 0;

	minor = MINOR(inode->i_rdev);

	if (minor >= gpio_dev_count)
		return -ENODEV;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)) {
		ret = -EBADF;
		goto end;
	}

	result = 0;

	switch (cmd) {
	case GPIODEV_GET_CONFIG:
		{
			result =
			    copy_to_user((unsigned char *) arg,
					 &dev->current_config, sizeof(u32));
			if (result) {
				ret = -EFAULT;
				goto end;
			}
		}
		break;

	case GPIODEV_SET_CONFIG:
		{
			if (!(dev->flags & GPIODEV_FLAG_CONFIGURABLE)) {
				ret = -EBADF;
				goto end;
			}

			result =
			    copy_from_user(&value, (unsigned char *) arg,
								sizeof(u32));
			if (result) {
				ret = -EFAULT;
				goto end;
			}

			ret = config_gpio(dev, value);
		}
		break;

	case GPIODEV_INT_REENABLE:
		{
			if (!(GPIODEV_IS_INTERRUPTABLE(dev->current_config))) {
				ret = -EPERM;
				goto end;
			}

			dev->flags &= ~GPIODEV_FLAG_INTERRUPTED;
			enable_irq(gpio_to_irq(dev->pin_nr));
		}
		break;

	case GPIODEV_GET_LOWLEVELCONFIG:
		{
			if (!(dev->flags & GPIODEV_FLAG_LOWLEVELACCESS)) {
				ret = -EPERM;
				goto end;
			}

			llconf.config = dev->current_config;

			result = copy_to_user((unsigned int *) arg, &llconf,
					      sizeof(GPIODEV_LOWLEVEL_CONFIG));
			if (result) {
				ret = -EFAULT;
				goto end;
			}
		}
		break;

	case GPIODEV_SET_LOWLEVELCONFIG:
		{
			if (!(dev->flags & GPIODEV_FLAG_LOWLEVELACCESS) ||
				!(dev->flags & GPIODEV_FLAG_CONFIGURABLE)) {
				ret = -EPERM;
				goto end;
			}

			result =
			    copy_from_user(&llconf, (unsigned int *) arg,
					   sizeof(GPIODEV_LOWLEVEL_CONFIG));
			if (result) {
				ret = -EFAULT;
				goto end;
			}

			ret = config_gpio(dev, llconf.config);
		}
		break;

	case GPIODEV_INT_POLL:
		if (!(GPIODEV_IS_INTERRUPTABLE(dev->current_config))) {
			ret = -EPERM;
			goto end;
		}
		else { 
			trace_msg("evoke wait_event_interruptible calling \n");
			wait_event_interruptible(dev->event_queue, (dev->flags & GPIODEV_FLAG_INTERRUPTED) != 0);
			dev->flags &= ~GPIODEV_FLAG_INTERRUPTED;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

end:
	mutex_unlock(&dev->lock);
	return ret;
}

/*
 * gpiodev_poll handles a user's select/poll call. This function will
 * return whatever functionality is currently available on a device.
 * The intended use is to wait for interrupts in a select call. By
 * waiting on an exception, a select/poll call will block until an interrupt
 * occurs.
 */
static unsigned int gpiodev_poll(struct file *filp, poll_table * table)
{
	struct gpio_device *dev;
	unsigned long minor;
	unsigned int mask = 0;

	minor = MINOR(filp->f_dentry->d_inode->i_rdev);

	if (minor >= gpio_dev_count)
		return POLLERR;

	dev = &gpio_devs[minor];

	mutex_lock(&dev->lock);

	if (!(dev->flags & GPIODEV_FLAG_OPEN)
	    || !(GPIODEV_IS_INTERRUPTABLE(dev->current_config))) {
		mask |= POLLNVAL;
		goto end;
	}

	trace_msg("evoke poll_wait calling \n");
	poll_wait(filp, &dev->event_queue, table);
	if (dev->flags & GPIODEV_FLAG_INTERRUPTED)
		mask |= POLLPRI;

end:
	dev->flags &= ~GPIODEV_FLAG_INTERRUPTED;
	mutex_unlock(&dev->lock);
	return mask;
}

static const struct file_operations gpiodev_fops = {
	.owner = THIS_MODULE,
	.open = gpiodev_open,
	.release = gpiodev_close,
	.read = gpiodev_read,
	.write = gpiodev_write,
	.ioctl = gpiodev_ioctl,
	.poll = gpiodev_poll,
};

/*
 * gpiodev_cleanup unregisters the character drivers and removes the /dev
 * entries. If for some reason a GPIO still has a reference count, interrupts
 * are disabled on to effectively 'shut it down.'
 */
static void gpiodev_cleanup(unsigned long device_nr)
{
	struct gpio_device *gpiodev;
	unsigned long index;

	for (index = 0; index < device_nr; index++) {
		gpiodev = &gpio_devs[index];
		device_destroy(gpiodev_class,
				     MKDEV(gpiodev_major, index));
	}

	class_destroy(gpiodev_class);
	unregister_chrdev(gpiodev_major, GPIO_DEVICE_DEV_NAME);

}

static void gpiodev_exit(void)
{
	gpiodev_cleanup(gpio_dev_count);
}

/*
 * gpiodev_init sets up the character driver, configures the GPIOs specified
 * in the gpiodevs list, and creates entries in /dev corresponding to
 * each configured GPIO.
 */
static int __init gpiodev_init(void)
{
	struct gpio_device *gpiodev;
	unsigned long index;
	unsigned long result;

	result = register_chrdev(0, GPIO_DEVICE_DEV_NAME, &gpiodev_fops);
	if (result < 0) {
		printk(KERN_ERR "Failed to register gpiodev %s \n",
		       GPIO_DEVICE_DEV_NAME);

		return result;
	}

	gpiodev_major = result;
	gpiodev_class = class_create(THIS_MODULE, GPIO_DEVICE_DEV_NAME);

	for (index = 0; index < gpio_dev_count; index++) {
		gpiodev = &gpio_devs[index];

		mutex_init(&gpiodev->lock);
		init_waitqueue_head(&gpiodev->event_queue);
		if (IS_ERR(device_create(gpiodev_class,
					       NULL,
					       MKDEV(gpiodev_major, index),
					       NULL,
					       "%s", gpiodev->device_name))) {
			printk(KERN_ERR
			       "Device \"%s\" could not be created properly\n",
			       gpiodev->device_name);
			gpiodev_cleanup(index);
			return -1;
		}
		trace_msg("Created device %s", gpiodev->device_name);
	}

	printk(KERN_INFO "GPIODev init successfully \n");
	return 0;
}

static int __init gpio_device_probe(struct platform_device *pdev)
{
	struct gpio_device_platform_data *data;

	data = pdev->dev.platform_data;
	if (data == NULL) {
		printk(KERN_ERR "gpio_device_probe: No pdata!\n");
		return -ENODEV;
	}

	if ((data->info == NULL) || (data->info_count == 0)) {
		printk(KERN_ERR "gpio_device_probe: incomplete pdata!\n");
		return -ENODEV;
	}

	gpio_devs = data->info;
	gpio_dev_count = data->info_count;

	gpiodev_init();
	return 0;
}

static int gpio_device_remove(struct platform_device *pdev)
{
	gpiodev_exit();
	return 0;
}

static struct platform_driver gpio_device_driver = {
	.probe = gpio_device_probe,
	.remove = gpio_device_remove,
	.driver = {
		.name = GPIO_DEVICE_DEV_NAME,
	},
};

static int __devinit gpio_device_init(void)
{
	return platform_driver_register(&gpio_device_driver);
}

static void __exit gpio_device_exit(void)
{
	platform_driver_unregister(&gpio_device_driver);
}

module_init(gpio_device_init);
module_exit(gpio_device_exit);
