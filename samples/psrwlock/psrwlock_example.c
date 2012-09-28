/*
 * Priority Sifting Reader-Writer Lock Example
 *
 * Copyright 2008 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 */

#include <linux/module.h>
#include <linux/psrwlock.h>

/*
 * Define which execution contexts can access the lock in read or write mode.
 * See psrwlock.h and psrwlock-types.h for details.
 *
 * In this example, the writer is in preemptable context and the readers either
 * in IRQ context, softirq context, non-preemptable context or preemptable
 * context.
 */
#define SAMPLE_ALL_WCTX		PSRW_PRIO_P
#define SAMPLE_ALL_RCTX		(PSR_IRQ | PSR_BH | PSR_NPTHREAD | PSR_PTHREAD)

static DEFINE_PSRWLOCK(sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
CHECK_PSRWLOCK_MAP(sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);

/*
 * Reader in IRQ context.
 */
static void executed_in_irq(void)
{
	psread_lock_irq(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
	/* read structure */
	psread_unlock(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
}

/*
 * Reader in Softirq context.
 */
static void executed_in_bh(void)
{
	psread_lock_bh(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
	/* read structure */
	psread_unlock(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
}

/*
 * Reader in non-preemptable context.
 */
static void executed_inatomic(void)
{
	psread_lock_inatomic(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
	/* read structure */
	psread_unlock(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
}

/*
 * Reader in preemptable context.
 */
static void reader_executed_preemptable(void)
{
	psread_lock(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
	/* read structure */
	psread_unlock(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
}

/*
 * Writer in preemptable context.
 */
static void writer_executed_preemptable(void)
{
	pswrite_lock(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
	/* read structure */
	pswrite_unlock(&sample_rwlock, SAMPLE_ALL_WCTX, SAMPLE_ALL_RCTX);
}

/*
 * Execute readers in all contexts.
 */
static void sample_all_context(void)
{
	local_irq_disable();
	executed_in_irq();
	local_irq_enable();

	local_bh_disable();
	executed_in_bh();
	local_bh_enable();

	preempt_disable();
	executed_inatomic();
	preempt_enable();

	reader_executed_preemptable();

	writer_executed_preemptable();
}


/*
 * In this second example, the writer is in non-preemptable context and the
 * readers either in IRQ context or softirq context only.
 */
static DEFINE_PSRWLOCK(sample_wnp_rbh_rirq_rwlock,
	PSRW_PRIO_P, PSR_IRQ | PSR_BH);
CHECK_PSRWLOCK_MAP(sample_wnp_rbh_rirq_rwlock,
	PSRW_PRIO_P, PSR_IRQ | PSR_BH);

/*
 * Reader in IRQ context.
 */
static void wnp_rbh_rirq_executed_in_irq(void)
{
	psread_lock_irq(&sample_wnp_rbh_rirq_rwlock,
		PSRW_PRIO_P, PSR_IRQ | PSR_BH);
	/* read structure */
	psread_unlock(&sample_wnp_rbh_rirq_rwlock,
		PSRW_PRIO_P, PSR_IRQ | PSR_BH);
}

/*
 * Reader in Softirq context.
 */
static void wnp_rbh_rirq_executed_in_bh(void)
{
	psread_lock_bh(&sample_wnp_rbh_rirq_rwlock,
		PSRW_PRIO_P, PSR_IRQ | PSR_BH);
	/* read structure */
	psread_unlock(&sample_wnp_rbh_rirq_rwlock,
		PSRW_PRIO_P, PSR_IRQ | PSR_BH);
}

/*
 * Writer in preemptable context.
 */
static void wnp_rbh_rirq_writer_executed_non_preemptable(void)
{
	pswrite_lock(&sample_wnp_rbh_rirq_rwlock,
				PSRW_PRIO_P, PSR_IRQ | PSR_BH);
	/* read structure */
	pswrite_unlock(&sample_wnp_rbh_rirq_rwlock,
				PSRW_PRIO_P, PSR_IRQ | PSR_BH);
}

/*
 * Execute readers in all contexts.
 */
static void sample_wnp_rbh_rirq_context(void)
{
	local_irq_disable();
	wnp_rbh_rirq_executed_in_irq();
	local_irq_enable();

	local_bh_disable();
	wnp_rbh_rirq_executed_in_bh();
	local_bh_enable();

	preempt_disable();
	wnp_rbh_rirq_writer_executed_non_preemptable();
	preempt_enable();
}

static int __init init_example(void)
{
	sample_all_context();
	sample_wnp_rbh_rirq_context();

	return 0;
}

static void __exit exit_example(void)
{
}

module_init(init_example)
module_exit(exit_example)

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("psrwlock example");
