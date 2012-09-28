/*
 *  wakeup_timer.h
 *
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
 * Jul 01, 2009		Motorola	 Initial version for omap Android
 * Jul 27, 2009         Motorola         Timer granularity to be in msecs
 * Aug 21, 2009		Motorola	 suspend optimization
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
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/jiffies.h>
#include <linux/time.h>
#include <linux/ktime.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/spinlock_types.h>
#include <linux/wakeup_timer.h>
#include <linux/wakeup_timer_kernel.h>
#include "pm.h"

#define ANDROID_WAKEUP_PRINT_ERROR (1U << 0)
#define ANDROID_WAKEUP_PRINT_SUSPEND (1U << 1)
#define ANDROID_WAKEUP_PRINT_RESUME (1U << 2)
#define ANDROID_WAKEUP_PRINT_CALLBACK (1U << 3)
#define ANDROID_WAKEUP_PRINT_INFO (1U << 4)

static int debug_mask = ANDROID_WAKEUP_PRINT_ERROR;
module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define pr_wakeup(debug_level_mask, args...) \
	do { \
		if (debug_mask & ANDROID_WAKEUP_PRINT_##debug_level_mask) \
			pr_info(args); \
	} while (0)

#define WAKEUP_LATENCY 0

/*
 * Wakeup timer state definition:
 */
enum timer_state {
	/* Timer is inactive */
	TIMER_INACTIVE = 0,
	/* Timer is active, waitting for timeout */
	TIMER_ACTIVE,
	/* Timer is timeout, has wokenup process, waitting for next poll */
	TIMER_PENDING,
	/* Timer is unused. */
	TIMER_INVALID
};

#ifdef CONFIG_HAS_WAKELOCK
static struct wake_lock driver_wake_lock;
#endif

static LIST_HEAD(parent_node);
static DEFINE_SPINLOCK(wakeup_timer_lock);
static bool isSuspended;

inline unsigned long get_expire_time(struct timer_cascade_root *pwkup_cascade)
{
	unsigned long clock;
	unsigned long cur_time;
	long delta;
	unsigned long k32_max;
	unsigned long long temp;

	/* 32K timer max val is 131071999969482ns, or 0x07CFFFFF ms*/
	k32_max = (unsigned long)0x07CFFFFF;
	clock = pwkup_cascade->timer_base_time + pwkup_cascade->period_time;

	temp = sched_clock();
	do_div(temp, NSEC_PER_MSEC);
	cur_time = (unsigned long)temp;
	if (cur_time >= pwkup_cascade->timer_base_time)
		delta = clock - cur_time;
	else {
		pr_wakeup(INFO, "32KHz clock roll-over \n");
		clock = pwkup_cascade->period_time -
			(k32_max - pwkup_cascade->timer_base_time);
		delta = (long)(clock - cur_time);
	}

	if (delta < 0)
		delta = 0;

	return (unsigned long)delta;
}

static unsigned long get_nearest_wakeup_timer_ktime(void)
{
	unsigned long timer_fire;
	unsigned long timer_temp;
	struct timer_cascade_root *pwkup_cascade;

	timer_fire = ULONG_MAX;
	timer_temp = 0;

	list_for_each_entry(pwkup_cascade, &parent_node, node) {
		if (pwkup_cascade->state == TIMER_ACTIVE ||
			pwkup_cascade->state == TIMER_INACTIVE) {
			timer_temp = get_expire_time(pwkup_cascade);
			if (timer_temp < timer_fire)
				timer_fire = timer_temp;
		}
		pr_wakeup(INFO, "type=%d, timer_base=%lu ms, expired_time=%lu,"
			" period=%lu ms, expired_interval=%lu ms, "
			" now=%llu, APP: %s\n",
			pwkup_cascade->cascade_type,
			pwkup_cascade->timer_base_time,
			pwkup_cascade->timer_base_time +
				pwkup_cascade->period_time,
			pwkup_cascade->period_time,
			timer_temp,
			sched_clock(),
			pwkup_cascade->process->comm);
	}
	return timer_fire;
}

static void cascade_start_hrtimer(struct timer_cascade_root *pwkup_cascade)
{
	ktime_t tmp;
	unsigned long expire_time;

	expire_time = get_expire_time(pwkup_cascade);
	tmp = ktime_set((long)expire_time/MSEC_PER_SEC,
			(long)expire_time%MSEC_PER_SEC*NSEC_PER_MSEC);

	hrtimer_start(&(pwkup_cascade->alarm_timer), tmp, HRTIMER_MODE_REL);

	pr_wakeup(INFO, "type=%d, timer_base=%lu ms, "
			"period=%lu ms, now=%llu ns\n",
			pwkup_cascade->cascade_type,
			pwkup_cascade->timer_base_time,
			pwkup_cascade->period_time,
			sched_clock());
}

static enum hrtimer_restart wakeup_timer_callback(struct hrtimer *timer)
{
	unsigned long flags;
	struct timer_cascade_root *pwkup_cascade;
	enum hrtimer_restart ret = HRTIMER_NORESTART;

	pwkup_cascade = container_of(timer,
				struct timer_cascade_root, alarm_timer);
	pr_wakeup(CALLBACK, "type=%d, timer_base=%lu ms, period=%lu ms, "
		"expected_time=%lu ms now=%llu ns\n",
			pwkup_cascade->cascade_type,
			pwkup_cascade->timer_base_time,
			pwkup_cascade->period_time,
			pwkup_cascade->timer_base_time +
				pwkup_cascade->period_time,
			sched_clock());

	if (pwkup_cascade->cascade_type == TYPE_STATUS
			&& pwkup_cascade->callback) {
		spin_lock_irqsave(&wakeup_timer_lock, flags);
		pwkup_cascade->state = TIMER_INVALID;
		list_del(&(pwkup_cascade->node));
		spin_unlock_irqrestore(&wakeup_timer_lock, flags);
		pwkup_cascade->callback();
		return ret;
	}

	spin_lock_irqsave(&wakeup_timer_lock, flags);

	pwkup_cascade->state = TIMER_PENDING;

	if (pwkup_cascade->cascade_type == TYPE_PERIODIC) {
		pwkup_cascade->timer_base_time += pwkup_cascade->period_time;
		cascade_start_hrtimer(pwkup_cascade);
	}
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_timeout(&(pwkup_cascade->wkuptimer_wake_lock), 5 * HZ);
#endif
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);

	wake_up_interruptible(&(pwkup_cascade->cascade_wq));

	return ret;
}

static int cascade_create(struct timer_cascade_root **ppwkup_cascade)
{
	int ret = 0;
	int sz = sizeof(struct timer_cascade_root);

	*ppwkup_cascade = kzalloc(sz, GFP_KERNEL);
	if (*ppwkup_cascade == NULL) {
		ret = -ENOMEM;
		return ret;
	}
	/* Initialize some of the members */
	init_waitqueue_head(&((*ppwkup_cascade)->cascade_wq));

	(*ppwkup_cascade)->process = current;

	hrtimer_init(&((*ppwkup_cascade)->alarm_timer),
			CLOCK_MONOTONIC,
			HRTIMER_MODE_REL);

	(*ppwkup_cascade)->alarm_timer.function = wakeup_timer_callback;

	INIT_LIST_HEAD(&((*ppwkup_cascade)->node));

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&((*ppwkup_cascade)->wkuptimer_wake_lock),
			WAKE_LOCK_SUSPEND, "wakeup_timer");
#endif
	return ret;
}

/* Attach timer request and attache timer cascade and start timers */
static int cascade_attach(struct timer_cascade_root *new_pwkup_cascade)
{
	unsigned long flags;
	struct timer_cascade_root *pwkup_cascade;
	int type;
	int has_period_timer = 0;

	unsigned long new_period_time;
	unsigned long base_first;
	unsigned long cur_time;
	unsigned long long temp;

	base_first = 0;

	new_period_time = new_pwkup_cascade->period_time;
	type = new_pwkup_cascade->cascade_type;

	spin_lock_irqsave(&wakeup_timer_lock, flags);

	temp = sched_clock();
	do_div(temp, NSEC_PER_MSEC);
	cur_time = (unsigned long)temp;

	if (type == TYPE_ONESHOT || type == TYPE_STATUS) {
		/* Directly attach the one shot timer, time based is now. */
		new_pwkup_cascade->timer_base_time = cur_time;
		list_add(&(new_pwkup_cascade->node), &parent_node);
		pr_wakeup(INFO, "type=%d, timer_base=%lu, "
			"period=%lu ms, now=%llu\n",
			new_pwkup_cascade->cascade_type,
			new_pwkup_cascade->timer_base_time,
			new_pwkup_cascade->period_time,
			sched_clock());
	} else if (type == TYPE_PERIODIC) {
		/* Align all period timers with the only one time base */
		list_for_each_entry(pwkup_cascade, &parent_node, node) {
			if (pwkup_cascade->cascade_type == TYPE_ONESHOT)
				continue;
			else if (pwkup_cascade->cascade_type == TYPE_PERIODIC ||
				base_first == 0) {
				base_first = pwkup_cascade->timer_base_time;
				has_period_timer = 1;
				break;
			}
		}

		/* No period cascade yet */
		if (base_first == 0)
			new_pwkup_cascade->timer_base_time = cur_time;
		else
			/* Align base with the first found period timer */
			new_pwkup_cascade->timer_base_time = base_first;

		if (has_period_timer)
			list_add(&(new_pwkup_cascade->node),
				&(pwkup_cascade->node));
		else
			list_add_tail(&(new_pwkup_cascade->node),
				&(pwkup_cascade->node));

		pr_wakeup(INFO, "type=%d, timer_base=%lu ms, "
			"period=%lu ms, now=%llu\n",
			new_pwkup_cascade->cascade_type,
			new_pwkup_cascade->timer_base_time,
			new_pwkup_cascade->period_time,
			sched_clock());
	}
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);

	/* don't start hrtimer if we are suspended */
	if (!isSuspended)
		cascade_start_hrtimer(new_pwkup_cascade);

	return 0;
}

static int cascade_deattach(struct timer_cascade_root *pwkup_cascade)
{

	if (pwkup_cascade->state == TIMER_INVALID)
		return 0;

	pwkup_cascade->state = TIMER_INVALID;
	if (!isSuspended)
		hrtimer_cancel(&(pwkup_cascade->alarm_timer));
	list_del(&(pwkup_cascade->node));

#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&(pwkup_cascade->wkuptimer_wake_lock));
#endif
	return 0;
}

struct timer_cascade_root *wakeup_create_status_timer(int (*callback) (void))
{
	struct timer_cascade_root *new;

	if (cascade_create(&new))
		return NULL;

	new->state = TIMER_INVALID;
	new->cascade_type = TYPE_STATUS;
	new->callback = callback;
	return new;
}
EXPORT_SYMBOL_GPL(wakeup_create_status_timer);

void wakeup_start_status_timer(struct timer_cascade_root *timer,
		unsigned long period)
{
	unsigned long flags;

	spin_lock_irqsave(&wakeup_timer_lock, flags);
	if (timer->state == TIMER_INACTIVE)
		cascade_deattach(timer);

	timer->state = TIMER_INACTIVE;
	timer->cascade_type = TYPE_STATUS;
	timer->period_time = period ;
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);
	/* Now, the timer request is ready but not attached into the list */
	cascade_attach(timer);

	if (isSuspended) {
		/* We are suspended, set wakeup timer to nearest timer. */
		unsigned long expire = get_nearest_wakeup_timer_ktime();
		if (expire == ULONG_MAX) {
			wakeup_timer_seconds = 0;
			wakeup_timer_nseconds = 0;
			return;
		}
		wakeup_timer_seconds = (expire-WAKEUP_LATENCY)/MSEC_PER_SEC;
		wakeup_timer_nseconds = ((expire-WAKEUP_LATENCY)%MSEC_PER_SEC)
					*NSEC_PER_MSEC;
		pr_wakeup(INFO, "set wakeup_timer_seconds: %d.%d \n",
			wakeup_timer_seconds, wakeup_timer_nseconds);
	}
}
EXPORT_SYMBOL_GPL(wakeup_start_status_timer);
extern int wakeup_stop_status_timer(struct timer_cascade_root *timer)
{
	unsigned long flags;
	spin_lock_irqsave(&wakeup_timer_lock, flags);
	cascade_deattach(timer);
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(wakeup_stop_status_timer);
extern int wakeup_del_status_timer(struct timer_cascade_root *timer)
{
	if (!timer)
		return -EINVAL;

	wakeup_stop_status_timer(timer);
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&(timer->wkuptimer_wake_lock));
	wake_lock_destroy(&(timer->wkuptimer_wake_lock));
#endif
	/* Free the memory for the timer request */
	kfree(timer);
	return 0;
}
EXPORT_SYMBOL_GPL(wakeup_del_status_timer);

/*
 * return -1 if no sim status timer pending, 0 if it has expired
 * and time left in ms otherwise
 */
int wakeup_check_status_timer(struct timer_cascade_root *timer)
{
	if (timer->state == TIMER_INACTIVE)
		return get_expire_time(timer);
	return -1;
}
EXPORT_SYMBOL_GPL(wakeup_check_status_timer);

/*
 * Main function to add new timer.
 */
static int wakeup_timer_add
(int type, struct file *filp, unsigned long timer_period)
{
	struct timer_cascade_root *pwkup_cascade;
	unsigned long flags;

	if ((type != TYPE_PERIODIC) && (type != TYPE_ONESHOT))
		return -EINVAL;

	/* Need create new request */
	if (likely(filp->private_data == 0)) {
		if (cascade_create(&pwkup_cascade))
			return -ENOMEM;

		/* Initialize more members in timer request */
		filp->private_data = (void *)pwkup_cascade;
	} else {
		/* Remove the timer request from list */
		pwkup_cascade = (struct timer_cascade_root *)filp->private_data;

		spin_lock_irqsave(&wakeup_timer_lock, flags);
		cascade_deattach(pwkup_cascade);
		spin_unlock_irqrestore(&wakeup_timer_lock, flags);
	}

	spin_lock_irqsave(&wakeup_timer_lock, flags);
	pwkup_cascade->state = TIMER_INACTIVE;
	pwkup_cascade->cascade_type = type;
	pwkup_cascade->period_time = timer_period ;
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);

	/* Now, the timer request is ready but not attached into the list */
	return cascade_attach(pwkup_cascade);
}

/* This function will delete the timer request and free it,
 * also it will delete and free the cascade if necessary
 */
static int wakeup_timer_destroy(struct file *filp)
{
	unsigned long flags;
	struct timer_cascade_root *pwkup_cascade;

	/* No wakeup timer request */
	if (filp->private_data == 0)
		return 0;

	pwkup_cascade = (struct timer_cascade_root *)filp->private_data;

	spin_lock_irqsave(&wakeup_timer_lock, flags);
	cascade_deattach(pwkup_cascade);

#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&(pwkup_cascade->wkuptimer_wake_lock));
	wake_lock_destroy(&(pwkup_cascade->wkuptimer_wake_lock));
#endif
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);

	/* Free the memory for the timer request */
	kfree(pwkup_cascade);

	filp->private_data = 0;

	return 0;
}

/*
 * Only one timer is allowed for each thread
 * Driver will use filp->private_data as the reference for the timer request
 * to avoid going through the whole list
 */
static int wakeup_timer_open(struct inode *inode, struct file *filp)
{
	if (filp->private_data == 0)
		return nonseekable_open(inode, filp);
	else
		return -1;
}

static int wakeup_timer_release(struct inode *inode, struct file *filp)
{
	return wakeup_timer_destroy(filp);
}

/*
 * This function will do strict check, so others will just assume the
 * timer in timerlist are valid
 */
static int wakeup_timer_ioctl(struct inode *inode,
		struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;

	switch (cmd) {
	case IOC_WAKEUP_TIMER_SETPERIOD:
		if (arg <= 0) {
			pr_wakeup(ERROR, "Period Timer Value is not valid.\n");
			return -EINVAL;
		}

		if (arg < 20000) {
			pr_wakeup(ERROR, "Wakeup Timer Period < 20 seconds"
					"is not supported.\n");
			return -EINVAL;
		}

		pr_wakeup(INFO, "Period wake up timer has been set.\n");
		ret = wakeup_timer_add(TYPE_PERIODIC, filp, (unsigned long)arg);
		break;

	case IOC_WAKEUP_TIMER_ONESHOT:
		if (arg <= 0) {
			pr_wakeup(ERROR, "Oneshot Timer Value is not valid.\n");
			return -EINVAL;
		}

		pr_wakeup(INFO, "Oneshot wake up timer has been set.\n");
		ret = wakeup_timer_add(TYPE_ONESHOT, filp, (unsigned long)arg);
		break;

	case IOC_WAKEUP_TIMER_DELETE:
		ret = wakeup_timer_destroy(filp);
		break;

	default:
		pr_wakeup(ERROR, "Invalid IOCTL command\n");
		return -EINVAL;
	}
	return ret;
}

/* The POLL will mark the timer request as ACTIVE so timer
* callback can mark this as pending timer
*/
static unsigned int wakeup_timer_poll(struct file *filp, poll_table *wait)
{
	unsigned long flags;
	unsigned int ret = 0;
	struct timer_cascade_root *pwkup_cascade;

	/* If the timer is not present, do not permit this operation */
	if (filp->private_data == NULL)
		return -EPERM;

	pwkup_cascade = (struct timer_cascade_root *)filp->private_data;
	if (pwkup_cascade->state == TIMER_INVALID)
		return -EPERM;

	/* Check whether the timer has already expired */
	spin_lock_irqsave(&wakeup_timer_lock, flags);
	if (pwkup_cascade->state == TIMER_PENDING) {
		pwkup_cascade->state = TIMER_ACTIVE;
		if (pwkup_cascade->cascade_type == TYPE_ONESHOT)
			cascade_deattach(pwkup_cascade);

		spin_unlock_irqrestore(&wakeup_timer_lock, flags);
		return POLLIN;
	}
	pwkup_cascade->state = TIMER_ACTIVE;
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);

	/* release the wake lock before wait */
#ifdef CONFIG_HAS_WAKELOCK
	wake_unlock(&(pwkup_cascade->wkuptimer_wake_lock));
#endif
	poll_wait(filp, &(pwkup_cascade->cascade_wq), wait);

	/* Check whether the timer has already expired */
	spin_lock_irqsave(&wakeup_timer_lock, flags);
	if (pwkup_cascade->state == TIMER_PENDING) {
		pwkup_cascade->state = TIMER_ACTIVE;
		ret = POLLIN;
		if (pwkup_cascade->cascade_type == TYPE_ONESHOT)
			cascade_deattach(pwkup_cascade);
	}
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);

	return ret;
}

static const struct file_operations wakeup_timer_fops = {
	.owner		= THIS_MODULE,
	.ioctl		= wakeup_timer_ioctl,
	.open		= wakeup_timer_open,
	.release	= wakeup_timer_release,
	.poll		= wakeup_timer_poll,
};

static struct miscdevice wakeup_timer_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "wakeup_timer",
	.fops	= &wakeup_timer_fops
};

static ssize_t wake_timer_list_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int len = 0;

	struct timer_cascade_root *pwkup_cascade;
	unsigned long flags;

	spin_lock_irqsave(&wakeup_timer_lock, flags);
	list_for_each_entry(pwkup_cascade, &parent_node, node) {
	len += sprintf(buf + len,
			"Timer Type: %s, period:%lu ms, base:%lu ms\n",
			(pwkup_cascade->cascade_type) ? "ONESHOT" : "PERIOD",
			pwkup_cascade->period_time,
			pwkup_cascade->timer_base_time);
	}
	spin_unlock_irqrestore(&wakeup_timer_lock, flags);

	WARN_ON(len > PAGE_SIZE);
	return len;
}

static DEVICE_ATTR(wake_timer_list, S_IRUGO | S_IWUSR,
			wake_timer_list_show, NULL);

static int __init wakeup_timer_probe(struct platform_device *pdev)
{
	int err;

	err = device_create_file(&pdev->dev, &dev_attr_wake_timer_list);
	if (err) {
		dev_err(pdev->dev.parent,
			"failed to create timer list attribute, %d\n", err);
	}
#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&driver_wake_lock,
			WAKE_LOCK_SUSPEND,
			"wakeup_driver");
#endif

	return misc_register(&wakeup_timer_miscdev);
}

static int wakeup_timer_remove(struct platform_device *pdev)
{
	return misc_deregister(&wakeup_timer_miscdev);
}

#ifdef CONFIG_SUSPEND


static int wakeup_timer_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	struct timer_cascade_root *pwkup_cascade;
#ifdef CONFIG_HAS_WAKELOCK
	long timeout;
#endif

	unsigned long expire = get_nearest_wakeup_timer_ktime();
	pr_wakeup(SUSPEND, "suspend expire %lu\n", expire);
	if (expire == ULONG_MAX) {
		wakeup_timer_seconds = 0;
		wakeup_timer_nseconds = 0;
		return 0;
	}

	/* If expiration value is less than the total time of
	* "suspend+resume+shedule", then don't get into suspend.
	* Currently we suppose it will no more than 200ms in worst case,
	* right? this might need change later with more statistic.
	*/
	if (expire <= 5000) {
#ifdef CONFIG_HAS_WAKELOCK
		timeout = (HZ*expire)/MSEC_PER_SEC + 1;
		wake_lock_timeout(&driver_wake_lock , timeout);
#endif
		return -EBUSY;
	}

	/* Make sure system is waked up 200ms ahead of timer expiration,
	* then the latency could be off-set and timer could be scheduled
	* at accurate point.
	*/
	wakeup_timer_seconds = (expire-WAKEUP_LATENCY)/MSEC_PER_SEC;
	wakeup_timer_nseconds = ((expire-WAKEUP_LATENCY)%MSEC_PER_SEC)
				*NSEC_PER_MSEC;

	pr_wakeup(SUSPEND, "set wakeup_timer_seconds: %d.%d \n",
		wakeup_timer_seconds, wakeup_timer_nseconds);

	list_for_each_entry(pwkup_cascade, &parent_node, node) {
		hrtimer_cancel(&(pwkup_cascade->alarm_timer));
	}
	isSuspended = true;
	return 0;
}

static int wakeup_timer_resume(struct platform_device *pdev)
{
	struct timer_cascade_root *pwkup_cascade;
	struct timer_cascade_root *tmp;
#ifdef CONFIG_HAS_WAKELOCK
	unsigned long expire;
	long timeout;
#endif
	isSuspended = false;
	pr_wakeup(RESUME, "resume\n");

#ifdef CONFIG_HAS_WAKELOCK
	expire = get_nearest_wakeup_timer_ktime();

	/* if expire >200ms, its not waked up by timer.
	* if expire <50ms, hold a 50ms wakelock to make sure timer is scheduled.
	* if 50~200ms, disable suspend but allow idle by setting wakelock.
	*/
	if (expire < 5000) {
		if (expire <= 50)
			timeout = (HZ*50)/MSEC_PER_SEC + 1;
		else
			timeout = (HZ*expire)/MSEC_PER_SEC + 1;

		wake_lock_timeout(&driver_wake_lock, timeout);
		pr_wakeup(RESUME, "expire in %lu ms, "
				"taking wakelock for %ld tick.\n",
				expire, timeout);
	}
#endif
	list_for_each_entry_safe(pwkup_cascade, tmp, &parent_node, node) {
		cascade_start_hrtimer(pwkup_cascade);
	}
	return 0;
}

#else /* !CONFIG_SUSPEND */

#define wakeup_timer_suspend NULL
#define wakeup_timer_resume  NULL

#endif

static struct platform_driver wakeup_timer_driver = {
	.probe		= wakeup_timer_probe,
	.remove		= wakeup_timer_remove,
	.suspend	= wakeup_timer_suspend,
	.resume		= wakeup_timer_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "wakeup_timer",
	},
};

static struct platform_device *wakeup_timer_device;

static int __init wakeup_timer_init(void)
{
	int ret = 0;

	wakeup_timer_device =
		platform_device_register_simple("wakeup_timer", 0, NULL, 0);

	if (IS_ERR(wakeup_timer_device)) {
		pr_wakeup(ERROR, "Failed to register wakeup_timer device.\n");
		ret = PTR_ERR(wakeup_timer_device);
		return ret;
	}

	ret = platform_driver_register(&wakeup_timer_driver);
	if (ret) {
		pr_wakeup(ERROR, "Failed to register wakeup_timer driver.\n");
		platform_device_unregister(wakeup_timer_device);
	}

	return ret;
}

static void __exit wakeup_timer_exit(void)
{
	platform_driver_unregister(&wakeup_timer_driver);
	platform_device_unregister(wakeup_timer_device);
}

module_init(wakeup_timer_init);
module_exit(wakeup_timer_exit);

MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Motorola wakeup timer driver");
