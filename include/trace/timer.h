#ifndef _TRACE_TIMER_H
#define _TRACE_TIMER_H

#include <linux/tracepoint.h>

DECLARE_TRACE(timer_itimer_expired,
	TP_PROTO(struct signal_struct *sig),
		TP_ARGS(sig));
DECLARE_TRACE(timer_itimer_set,
	TP_PROTO(int which, struct itimerval *value),
		TP_ARGS(which, value));
DECLARE_TRACE(timer_set,
	TP_PROTO(struct timer_list *timer),
		TP_ARGS(timer));
/*
 * xtime_lock is taken when kernel_timer_update_time tracepoint is reached.
 */
DECLARE_TRACE(timer_update_time,
	TP_PROTO(struct timespec *_xtime, struct timespec *_wall_to_monotonic),
		TP_ARGS(_xtime, _wall_to_monotonic));
DECLARE_TRACE(timer_timeout,
	TP_PROTO(struct task_struct *p),
		TP_ARGS(p));
#endif
