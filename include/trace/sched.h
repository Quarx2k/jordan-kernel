#ifndef _LTTNG_TRACE_SCHED_H
#define _LTTNG_TRACE_SCHED_H

#include <linux/sched.h>
#include <linux/tracepoint.h>

DECLARE_TRACE(sched_kthread_create,
	TP_PROTO(void *fn, int pid),
		TP_ARGS(fn, pid));

#endif /* _LTTNG_TRACE_SCHED_H */
