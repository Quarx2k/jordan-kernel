
/*
 *	tcmd_driver.c
 *
 * Copyright (c) 2010 Motorola
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/types.h>
#include <linux/gpio.h>
#include <linux/uaccess.h>
#include <linux/gpio_mapping.h>
#include <linux/tcmd_driver.h>

#define NAME "tcmd_driver"

const struct tcmd_gpio_mapping devtree_gpio_name_map_table[] = {
		{TCMD_GPIO_ISL29030_INT,       "als_int"},
		{TCMD_GPIO_KXTF9_INT,          "accel1_int"},
		{TCMD_GPIO_MMC_DETECT,         "mmc_detect"},
		{TCMD_GPIO_PROX_SFH7743,       "proximity_int"}
	};

static int tcmd_misc_open(struct inode *inode, struct file *file)
{
	int err = nonseekable_open(inode, file);
	if (err < 0)
		return err;

	return 0;
}

static int tcmd_misc_ioctl(struct inode *inode, struct file *file,
							unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int gpio_enum = -1, gpio = -1, irq = -1, gpio_state = -1, index = 0;
	int devtree_gpio_size = (sizeof(devtree_gpio_name_map_table) /
			sizeof(devtree_gpio_name_map_table[0]));
	char *gpio_name = NULL;
	struct tcmd_gpio_set_arg gpio_set_arg;

	if (cmd == TCMD_IOCTL_SET_INT) {
		if (copy_from_user(&gpio_set_arg, argp, 2))
			return -EFAULT;
		gpio_enum = gpio_set_arg.gpio;
		gpio_state = gpio_set_arg.gpio_state;
    }
	else if (copy_from_user(&gpio_enum, argp, sizeof(int)))
		return -EFAULT;

	for (index = 0; index < devtree_gpio_size; index++)	{
		if (devtree_gpio_name_map_table[index].gpio_index
			== gpio_enum) {
			gpio_name =
			(char *)devtree_gpio_name_map_table[index].gpio_name;
			break;
		}
	}

	if (gpio_name == NULL)
		return -EINVAL;

	gpio = get_gpio_by_name(gpio_name);
	if (gpio <= 0)
		return -EINVAL;

	switch (cmd) {
	case TCMD_IOCTL_MASK_INT:
	case TCMD_IOCTL_UNMASK_INT:
		irq = gpio_to_irq(gpio);
		if (irq < 0)
			return -EINVAL;
		break;
	default:
		break;
	}

	switch (cmd) {
	case TCMD_IOCTL_MASK_INT:
		pr_info("tcmd mask interrupt: gpio = %d, irq = %d.\n",
						gpio, irq);
		disable_irq(irq);
		break;
	case TCMD_IOCTL_UNMASK_INT:
		pr_info("tcmd unmask interrupt: gpio = %d, irq = %d.\n",
						gpio, irq);
		enable_irq(irq);
		break;
	case TCMD_IOCTL_READ_INT:
		gpio_state = gpio_get_value(gpio);
		pr_info("tcmd interrupt state: gpio = %d -> %d.\n",
						gpio, gpio_state);
		if (copy_to_user(argp, &gpio_state, sizeof(int)))
			return -EFAULT;
		break;
	case TCMD_IOCTL_SET_INT:
		pr_info("tcmd set interrupt state: gpio = %d -> %d.\n",
						gpio, gpio_state);
		gpio_direction_output(gpio, 0);
		gpio_set_value(gpio, gpio_state);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations tcmd_misc_fops = {
	.owner = THIS_MODULE,
	.open = tcmd_misc_open,
	.ioctl = tcmd_misc_ioctl,
};

static struct miscdevice tcmd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = NAME,
	.fops = &tcmd_misc_fops,
};

static int __init tcmd_init(void)
{
	int error = misc_register(&tcmd_misc_device);
	if (error < 0) {
		pr_err("%s: tcmd misc register failed!\n", __func__);
		return error;
	}

	pr_info("tcmd probe\n");

	return 0;
}

static void __exit tcmd_exit(void)
{
	misc_deregister(&tcmd_misc_device);
}

module_init(tcmd_init);
module_exit(tcmd_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("tcmd");
