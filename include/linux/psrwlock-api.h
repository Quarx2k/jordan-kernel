#ifndef _LINUX_PSRWLOCK_API_H
#define _LINUX_PSRWLOCK_API_H

/* Reader lock */

/*
 * many readers, from irq/softirq/non preemptable and preemptable thread
 * context. Protects against writers.
 *
 * Read lock fastpath :
 *
 * A cmpxchg is used here and _not_ a simple add because a lower-priority reader
 * could block the writer while it is waiting for readers to clear the
 * uncontended path. This would happen if, for instance, the reader gets
 * interrupted between the add and the moment it gets to the slow path.
 */

/*
 * Called from any context.
 * Statically check for preemptable writer to compile-out the check if all the
 * contexts accessing the lock are non-preemptable.
 */
static inline void psread_unlock(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc = atomic_sub_return(UC_READER_OFFSET, &rwlock->uc);
	if (wctx == PSRW_PRIO_P || (rctx & PSR_PTHREAD))
		psrwlock_preempt_check(uc, rwlock);
}

/*
 * Called from interrupt disabled or interrupt context.
 */
static inline void psread_lock_irq(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_IRQ));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return;
	psread_lock_slow_irq(uc, rwlock);
}

static inline int psread_trylock_irq(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_IRQ));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 1;
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
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return;
	psread_lock_slow_bh(uc, rwlock);
}

static inline int psread_trylock_bh(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_BH));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 1;
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
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return;
	psread_lock_slow_inatomic(uc, rwlock);
}

static inline int psread_trylock_inatomic(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_NPTHREAD));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 1;
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
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return;
	psread_lock_slow(uc, rwlock);
}

static inline int psread_lock_interruptible(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_PTHREAD));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 0;
	return psread_lock_interruptible_slow(uc, rwlock);
}

static inline int psread_trylock(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	MAYBE_BUILD_BUG_ON(!(rctx & PSR_PTHREAD));
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_READER_OFFSET);
	if (likely(!uc))
		return 1;
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
	/* no other reader nor writer present, try to take the lock */
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_WRITER);
	if (likely(!uc))
		return;
	else
		pswrite_lock_slow(uc, rwlock);
}

static inline
int pswrite_lock_interruptible(psrwlock_t *rwlock,
		enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	write_context_disable(wctx, rctx);
	/* no other reader nor writer present, try to take the lock */
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_WRITER);
	if (likely(!uc))
		return 0;
	else
		return pswrite_lock_interruptible_slow(uc, rwlock);
}

static inline
int pswrite_trylock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	write_context_disable(wctx, rctx);
	/* no other reader nor writer present, try to take the lock */
	uc = atomic_cmpxchg(&rwlock->uc, 0, UC_WRITER);
	if (likely(!uc))
		return 1;
	else
		return pswrite_trylock_slow(uc, rwlock);
}

static inline
void pswrite_unlock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	unsigned int uc;

	/*
	 * atomic_cmpxchg makes sure we commit the data before reenabling
	 * the lock. Will take the slow path if there are active readers, if
	 * UC_SLOW_WRITER is set or if there are threads in the wait queue.
	 */
	uc = atomic_cmpxchg(&rwlock->uc, UC_WRITER, 0);
	if (likely(uc == UC_WRITER)) {
		write_context_enable(wctx, rctx);
		/*
		 * no need to check preempt because all wait queue masks
		 * were 0. An active wait queue would trigger the slow path.
		 */
		return;
	}
	/*
	 * Go through the slow unlock path to check if we must clear the
	 * UC_SLOW_WRITER bit.
	 */
	pswrite_unlock_slow(uc, rwlock);
}

#endif /* _LINUX_PSRWLOCK_API_H */
