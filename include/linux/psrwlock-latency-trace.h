#ifndef _LINUX_PSRWLOCK_LATENCY_TRACE_H
#define _LINUX_PSRWLOCK_LATENCY_TRACE_H

/*
 * Priority Sifting Reader-Writer Lock Latency Tracer
 *
 * Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 * August 2008
 */

#include <linux/hardirq.h>

#ifdef CONFIG_PSRWLOCK_LATENCY_TEST

extern void psrwlock_profile_latency_reset(void);
extern void psrwlock_profile_latency_print(void);

extern void psrwlock_profile_irq_disable(void);
extern void psrwlock_profile_irq_enable(void);
extern void psrwlock_profile_bh_disable(void);
extern void psrwlock_profile_bh_enable(void);

#define psrwlock_irq_save(flags)				\
do {								\
	local_irq_save(flags);					\
	if (!irqs_disabled_flags(flags))			\
		psrwlock_profile_irq_disable();		\
} while (0)

#define psrwlock_irq_restore(flags)				\
do {								\
	if (irqs_disabled() && !irqs_disabled_flags(flags))	\
		psrwlock_profile_irq_enable();		\
	local_irq_restore(flags);				\
} while (0)

static inline void psrwlock_irq_disable(void)
{
	unsigned long flags;

	local_save_flags(flags);
	local_irq_disable();
	if (!irqs_disabled_flags(flags))
		psrwlock_profile_irq_disable();
}
static inline void psrwlock_irq_enable(void)
{
	if (irqs_disabled())
		psrwlock_profile_irq_enable();
	local_irq_enable();
}
static inline void psrwlock_bh_disable(void)
{
	local_bh_disable();
	if (softirq_count() == SOFTIRQ_OFFSET)
		psrwlock_profile_bh_disable();
}
static inline void psrwlock_bh_enable(void)
{
	if (softirq_count() == SOFTIRQ_OFFSET)
		psrwlock_profile_bh_enable();
	local_bh_enable();
}
static inline void psrwlock_bh_enable_ip(unsigned long ip)
{
	if (softirq_count() == SOFTIRQ_OFFSET)
		psrwlock_profile_bh_enable();
	local_bh_enable_ip(ip);
}

#ifdef CONFIG_PREEMPT
extern void psrwlock_profile_preempt_disable(void);
extern void psrwlock_profile_preempt_enable(void);

static inline void psrwlock_preempt_disable(void)
{
	preempt_disable();
	if (preempt_count() == PREEMPT_OFFSET)
		psrwlock_profile_preempt_disable();
}
static inline void psrwlock_preempt_enable(void)
{
	if (preempt_count() == PREEMPT_OFFSET)
		psrwlock_profile_preempt_enable();
	preempt_enable();
}
static inline void psrwlock_preempt_enable_no_resched(void)
{
	/*
	 * Not exactly true, since we really re-preempt at the next preempt
	 * check, but gives a good idea (lower-bound).
	 */
	if (preempt_count() == PREEMPT_OFFSET)
		psrwlock_profile_preempt_enable();
	preempt_enable_no_resched();
}
#else
#define psrwlock_preempt_disable()		preempt_disable()
#define psrwlock_preempt_enable()		preempt_enable()
#define psrwlock_preempt_enable_no_resched()	preempt_enable_no_resched()
#endif

#endif	/* CONFIG_PSRWLOCK_LATENCY_TEST */
#endif	/* _LINUX_PSRWLOCK_LATENCY_TRACE_H */
