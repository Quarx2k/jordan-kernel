#ifndef _LTTNG_TRACE_LOCKDEP_H
#define _LTTNG_TRACE_LOCKDEP_H

#include <linux/lockdep.h>
#include <linux/tracepoint.h>

/*
 * lockdep tracing must be very careful with respect to reentrancy.
 *
 * It should not use immediate values for activation because it involves
 * traps called when the code patching is done.
 */
DECLARE_TRACE(lockdep_hardirqs_on,
	TP_PROTO(unsigned long retaddr),
		TP_ARGS(retaddr));
DECLARE_TRACE(lockdep_hardirqs_off,
	TP_PROTO(unsigned long retaddr),
		TP_ARGS(retaddr));
DECLARE_TRACE(lockdep_softirqs_on,
	TP_PROTO(unsigned long retaddr),
		TP_ARGS(retaddr));
DECLARE_TRACE(lockdep_softirqs_off,
	TP_PROTO(unsigned long retaddr),
		TP_ARGS(retaddr));

/* FIXME : some duplication with lockdep TRACE EVENTs */
DECLARE_TRACE(lockdep_lock_acquire,
	TP_PROTO(unsigned long retaddr, unsigned int subclass,
			struct lockdep_map *lock, int trylock, int read,
			int hardirqs_off),
		TP_ARGS(retaddr, subclass, lock, trylock, read, hardirqs_off));
DECLARE_TRACE(lockdep_lock_release,
	TP_PROTO(unsigned long retaddr, struct lockdep_map *lock, int nested),
		TP_ARGS(retaddr, lock, nested));


#endif /* _LTTNG_TRACE_LOCKDEP_H */
