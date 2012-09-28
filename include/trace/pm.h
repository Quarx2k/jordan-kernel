#ifndef _TRACE_PM_H
#define _TRACE_PM_H

#include <linux/tracepoint.h>

DECLARE_TRACE(pm_idle_entry,
	TP_PROTO(void),
		TP_ARGS());
DECLARE_TRACE(pm_idle_exit,
	TP_PROTO(void),
		TP_ARGS());
DECLARE_TRACE(pm_suspend_entry,
	TP_PROTO(void),
		TP_ARGS());
DECLARE_TRACE(pm_suspend_exit,
	TP_PROTO(void),
		TP_ARGS());

#endif
