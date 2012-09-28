/* drivers/misc/vib-omap-pwm.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 * Copyright (C) 2008 HTC Corporation.
 * Copyright (C) 2007 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/workqueue.h>
#include <linux/clk.h>
#include <mach/dmtimer.h>
#include <mach/gpio.h>
#include <mach/mux.h>

/* TODO: replace with correct header */
#include "../staging/android/timed_output.h"

static struct work_struct vibrator_work;
static struct hrtimer vibe_timer;
static struct omap_dm_timer *pwm_timer;
static spinlock_t vibe_lock;
static int vibe_state;
static int vibe_timer_state;

/* periods in microseconds, old vals: on = 700, off = 467 */
static unsigned long on_period = 1800;
static unsigned long off_period = 1200;
static unsigned long load_reg;
static unsigned long cmp_reg;

static void pwm_timer_init(void)
{
	/* timer pwm setup */
	omap_dm_timer_set_source(pwm_timer,
		OMAP_TIMER_SRC_32_KHZ);

	omap_dm_timer_stop(pwm_timer);
	omap_dm_timer_set_load(pwm_timer, 1, -load_reg);
	omap_dm_timer_set_match(pwm_timer, 1, -cmp_reg);

	omap_dm_timer_set_pwm(pwm_timer, 0, 1,
		OMAP_TIMER_TRIGGER_OVERFLOW_AND_COMPARE);

	omap_dm_timer_write_counter(pwm_timer, -2);
}

static void set_gptimer_pwm_vibrator(int on)
{
	if (pwm_timer == NULL) {
		pr_err(KERN_ERR "vibrator pwm timer is NULL\n");
		return;
	}

	if (on) {
		if(!vibe_timer_state) {
			omap_dm_timer_enable(pwm_timer);
			pwm_timer_init();
			omap_dm_timer_start(pwm_timer);
			vibe_timer_state = 1;
		}
	} else {
		if(vibe_timer_state) {
			omap_dm_timer_stop(pwm_timer);
			omap_dm_timer_disable(pwm_timer);
			vibe_timer_state = 0;
		}
	}
}

static void update_vibrator(struct work_struct *work)
{
	set_gptimer_pwm_vibrator(vibe_state);
}

static void vibrator_enable(struct timed_output_dev *dev, int value)
{
	unsigned long	flags;

	spin_lock_irqsave(&vibe_lock, flags);
	hrtimer_cancel(&vibe_timer);

	if (value == 0)
		vibe_state = 0;
	else {
		value = (value > 15000 ? 15000 : value);
		vibe_state = 1;
		hrtimer_start(&vibe_timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
	}

	spin_unlock_irqrestore(&vibe_lock, flags);

	schedule_work(&vibrator_work);
}

static int vibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&vibe_timer)) {
		ktime_t r = hrtimer_get_remaining(&vibe_timer);
		return r.tv.sec * 1000 + r.tv.nsec / 1000000;
	} else
		return 0;
}

static enum hrtimer_restart vibrator_timer_func(struct hrtimer *timer)
{
	vibe_state = 0;
	schedule_work(&vibrator_work);
	return HRTIMER_NORESTART;
}

static struct timed_output_dev gptimer_pwm_vibrator = {
	.name = "vibrator",
	.get_time = vibrator_get_time,
	.enable = vibrator_enable,
};

void __init vibrator_omap_pwm_init(int initial_vibrate)
{
	INIT_WORK(&vibrator_work, update_vibrator);

	spin_lock_init(&vibe_lock);
	vibe_state = 0;
	vibe_timer_state = 0;
	hrtimer_init(&vibe_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	vibe_timer.function = vibrator_timer_func;

	pwm_timer = omap_dm_timer_request_specific(11);
	if (pwm_timer == NULL) {
		pr_err(KERN_ERR "failed to request vibrator pwm timer\n");
		return;
	}

	/* omap_dm_timer_request_specific enables the timer */
	omap_dm_timer_disable(pwm_timer);

	timed_output_dev_register(&gptimer_pwm_vibrator);

	load_reg = 32768 * (on_period + off_period) / 1000000;
	cmp_reg = 32768 * off_period / 1000000;

	vibrator_enable(NULL, 0);
	if (initial_vibrate)
		vibrator_enable(NULL, initial_vibrate);

	pr_info("vib-omap-pwm initialized\n");
}

MODULE_DESCRIPTION("timed output gptimer pwm vibrator device");
MODULE_LICENSE("GPL");

