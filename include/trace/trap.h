#ifndef _TRACE_TRAP_H
#define _TRACE_TRAP_H

#include <linux/tracepoint.h>

DECLARE_TRACE(trap_entry,
	TP_PROTO(struct pt_regs *regs, long id),
	TP_ARGS(regs, id));
DECLARE_TRACE(trap_exit,
	TP_PROTO(void),
	TP_ARGS());

#endif
