#ifndef _LINUX_PSRWLOCK_DEBUG_API_H
#define _LINUX_PSRWLOCK_DEBUG_API_H

#include <linux/lockdep.h>

/*
 * Priority-Sifting reader-writer lock debugging API. Using the slow path.
 */

/* Reader lock */

/*
 * many readers, from irq/softirq/non preemptable and preemptable thread
 * context. Protects against writers.
 */

/*
 * Called from any context.
 * Statically check for preemptable writer to compile-out the check if all the
 * contexts accessing the lock are non-preemptable.
 */
extern void psread_unlock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx);

/*
 * Called from interrupt disabled or interrupt context.
 */
static inline void psread_lock_irq(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_IRQ));
	uc = atomic_read(&rwlock->uc);
	psread_lock_slow_irq(uc, rwlock);
}

static inline int psread_trylock_irq(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_IRQ));
	uc = atomic_read(&rwlock->uc);
	return psread_trylock_slow_irq(uc, rwlock);
}

/*
 * Called from softirq context.
 */

static inline void psread_lock_bh(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_BH));
	uc = atomic_read(&rwlock->uc);
	psread_lock_slow_bh(uc, rwlock);
}

static inline int psread_trylock_bh(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_BH));
	uc = atomic_read(&rwlock->uc);
	return psread_trylock_slow_bh(uc, rwlock);
}


/*
 * Called from non-preemptable thread context.
 */

static inline void psread_lock_inatomic(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_NPTHREAD));
	uc = atomic_read(&rwlock->uc);
	psread_lock_slow_inatomic(uc, rwlock);
}

static inline int psread_trylock_inatomic(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_NPTHREAD));
	uc = atomic_read(&rwlock->uc);
	return psread_trylock_slow_inatomic(uc, rwlock);
}


/*
 * Called from preemptable thread context.
 */

static inline void psread_lock(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_PTHREAD));
	uc = atomic_read(&rwlock->uc);
	psread_lock_slow(uc, rwlock);
}

static inline int psread_lock_interruptible(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_PTHREAD));
	uc = atomic_read(&rwlock->uc);
	return psread_lock_interruptible_slow(uc, rwlock);
}

static inline int psread_trylock(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_PTHREAD));
	uc = atomic_read(&rwlock->uc);
	return psread_trylock_slow(uc, rwlock);
}


/* Writer Lock */

/*
 * ctx is the context map showing which contexts can take the read lock and
 * which context is using the write lock.
 *
 * Write lock use example, where the lock is used by readers in interrupt,
 * preemptable context and non-preemptable context. The writer lock is taken in
 * preemptable context.
 *
 * static DEFINE_PSRWLOCK(lock, PSRW_PRIO_P, PSR_IRQ | PSR_PTHREAD);
 * CHECK_PSRWLOCK_MAP(lock, PSRW_PRIO_P, PSR_IRQ | PSR_PTHREAD);
 *
 *  pswrite_lock(&lock, PSRW_PRIO_P, PSR_IRQ | PSR_PTHREAD);
 *  ...
 *  pswrite_unlock(&lock, PSRW_PRIO_P, PSR_IRQ | PSR_PTHREAD);
 */
static inline
void pswrite_lock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	write_context_disable(wctx, rctx);
	uc = atomic_read(&rwlock->uc);
	pswrite_lock_slow(uc, rwlock);
}

static inline
int pswrite_lock_interruptible(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	write_context_disable(wctx, rctx);
	uc = atomic_read(&rwlock->uc);
	return pswrite_lock_interruptible_slow(uc, rwlock);
}

static inline
int pswrite_trylock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	write_context_disable(wctx, rctx);
	uc = atomic_read(&rwlock->uc);
	return pswrite_trylock_slow(uc, rwlock);
}

static inline
void pswrite_unlock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	uc = atomic_read(&rwlock->uc);
	pswrite_unlock_slow(uc, rwlock);
}

#endif /* _LINUX_PSRWLOCK_DEBUG_API_H */
