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
/*
 * DATE			AUTHOR		 COMMENT
 * -----		-----		 --------
 * Sep 08, 2009		Motorola	 Initial version for omap Android
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/moduleparam.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/wakeup_timer.h>

#define DRV_NAME "clock_32k"

struct timer_list timer_32k;
static struct platform_device *clock_platform_device;
static DEFINE_SPINLOCK(clock_32k_spin_lock);
struct timespec clock_32k_read(void)
{
	static unsigned long long roll_over;
	struct timespec my_time_32k;
	unsigned long long current_time;
	static unsigned long long curr_val;
	static unsigned long long last_val;
	curr_val = sched_clock();
	/* Check if sched_clock returned value has rolled over */
	if (curr_val < last_val) {
		roll_over += (1ULL << (32-15))*1000000000ULL;
		}
	current_time = curr_val + roll_over ;
	my_time_32k = ns_to_timespec(current_time);
	last_val = curr_val;
	/* Restart the 24 hrs timer to be sure that
	   all roll over will be detected */
	mod_timer(&timer_32k, jiffies + 24*60*60*HZ);
	return my_time_32k;
}

static int clock_32k_ioctl(struct inode *inode,
		struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct timespec my_time_32k;
	/* Acquire mutx to be sure that user read and
	*timer read will not happen at same time */

	/* Then evaluate the command and take according action */
	switch (cmd) {
	case IOC_32KHZ_READ:
		spin_lock_bh(&clock_32k_spin_lock);
		my_time_32k = clock_32k_read();
		spin_unlock_bh(&clock_32k_spin_lock);
		ret = __copy_to_user((int *)arg, \
		&my_time_32k, sizeof(my_time_32k));
		break;
	default:
		printk(KERN_ERR "Invalid IOCTL command\n");
		ret = -EINVAL;
	}
	return ret;
}

static int clock_32k_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int clock_32k_free(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations clock_32k_fops = {
	.owner =        THIS_MODULE,
	.ioctl =        clock_32k_ioctl,
	.open =         clock_32k_open,
	.release =      clock_32k_free,
};

static struct miscdevice clock_32k_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "clock_32k",
	.fops	= &clock_32k_fops,
};

void timer_32k_callback(unsigned long param)
{
	spin_lock(&clock_32k_spin_lock);
	clock_32k_read();
	spin_unlock(&clock_32k_spin_lock);
}

static int __devinit clock_32k_probe(struct platform_device *dev)
{
	int ret;
	spin_lock_init(&clock_32k_spin_lock);
	ret = misc_register(&clock_32k_miscdev);
	if (ret != 0)
		goto out;
	init_timer(&timer_32k);
	timer_32k.function = timer_32k_callback;
	timer_32k.data = 0 ;
	timer_32k.expires = jiffies + 24*60*60*HZ;
	add_timer(&timer_32k);
	return 0;
out:
	return ret;
}

static int __devexit clock_32k_remove(struct platform_device *dev)
{
	misc_deregister(&clock_32k_miscdev);
	return 0;
}

static struct platform_driver clock_32k_driver = {
	.probe		= clock_32k_probe,
	.remove		= __devexit_p(clock_32k_remove),
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= DRV_NAME,
		},
};

static int __init clock_32k_init(void)
{
	int err;
	err = platform_driver_register(&clock_32k_driver);
	if (err)
		return err;
	clock_platform_device = platform_device_register_simple(DRV_NAME,
								-1, NULL, 0);
	if (IS_ERR(clock_platform_device)) {
		err = PTR_ERR(clock_platform_device);
		goto unreg_platform_driver;
	}
	return 0;

unreg_platform_driver:
	platform_driver_unregister(&clock_32k_driver);
	return err;
}
static void __exit clock_32k_exit(void)
{
	platform_device_unregister(clock_platform_device);
	platform_driver_unregister(&clock_32k_driver);
}

module_init(clock_32k_init);
module_exit(clock_32k_exit);

MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Motorola 32K sync timer driver");
