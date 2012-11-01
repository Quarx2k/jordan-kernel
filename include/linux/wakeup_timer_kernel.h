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
 * DATE             AUTHOR         COMMENT
 * -----            ------         --------
 * Apr 30, 2009     Motorola       Initial version for omap Android
 *
 */

#ifndef WAKEUP_TIMER_KERNEL_H_
#define WAKEUP_TIMER_KERNEL_H_

#include <linux/time.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
/*
 * Wakeup timer type definition:
 * Periodic wakeup timers will ensure the accuracy in period,
 * but the first wakeup time point maybe adjusted
 * oneshot wakeup timers will ensure the accuracy on the desired time point
 */
enum timer_type {
	/* Periodic wakeup timers which will be restarted after it fires */
	TYPE_PERIODIC = 0,
	/* Oneshot wakeup timers which only start for once */
	TYPE_ONESHOT,
	/* scim status timer, very special case */
	TYPE_STATUS
};

struct timer_cascade_root {
	/* Timers cascade link list */
	struct list_head node;
	struct hrtimer alarm_timer;
	unsigned long period_time;
	unsigned long timer_base_time;

	/* wait queue head for this timer cascade */
	wait_queue_head_t cascade_wq;

	/* Periodic or oneshot for timers in this cascade */
	int cascade_type;

	/* current timer status */
	int state;

	/* calling process */
	struct task_struct *process;

	/* callback for kernel status timer */
	int (*callback) (void);

#ifdef CONFIG_HAS_WAKELOCK
	/* Wake lock to prevent suspend so allow app to finish their work */
	struct wake_lock wkuptimer_wake_lock;
#endif
};

extern struct timer_cascade_root *wakeup_create_status_timer(
		int (*callback) (void));
extern void wakeup_start_status_timer(struct timer_cascade_root *timer,
		unsigned long period_time);
extern int wakeup_stop_status_timer(struct timer_cascade_root *timer);
extern int wakeup_del_status_timer(struct timer_cascade_root *timer);
extern int wakeup_check_status_timer(struct timer_cascade_root *timer);

#endif /* WAKEUP_TIMER_H_ */
