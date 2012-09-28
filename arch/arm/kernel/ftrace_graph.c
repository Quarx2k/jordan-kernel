/*
 * function graph tracing support.
 *
 * Copyright (C) 2010 Motorola
 * Copyright (C) 2009 Tim Bird <tim.bird@am.sony.com>
 *
 * For licencing details, see COPYING.
 *
 * Defines routine needed for ARM return trampoline for tracing
 * function exits.
 */

#include <linux/ftrace.h>

/*
 * Hook the return address and push it in the stack of return addrs
 * in current thread info.
 */
void prepare_ftrace_return(unsigned long *parent, unsigned long self_addr,
			   unsigned long frame_pointer)
{
	unsigned long old;

	struct ftrace_graph_ent trace;
	unsigned long return_hooker = (unsigned long)
				&return_to_handler;

	if (unlikely(atomic_read(&current->tracing_graph_pause)))
		return;

	old = *parent;
	*parent = return_hooker;

	if (ftrace_push_return_trace(old, self_addr, &trace.depth,
		    frame_pointer) == -EBUSY) {
		*parent = old;
		return;
	}

	trace.func = self_addr;

	/* Only trace if the calling function expects to */
	if (!ftrace_graph_entry(&trace)) {
		current->curr_ret_stack--;
		*parent = old;
	}
}
