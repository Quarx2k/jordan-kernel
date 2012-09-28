/*
 * Priority Sifting Reader-Writer Lock
 *
 * Priority Sifting Reader-Writer Lock (psrwlock) excludes reader execution
 * contexts one at a time, thus increasing the writer priority in stages. It
 * favors writers against reader threads, but lets higher priority readers in
 * even when there are subscribed writers waiting for the lock at a given lower
 * priority. Very frequent writers could starve reader threads.
 *
 * Copyright 2008 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 */

#include <linux/psrwlock.h>
#include <linux/list.h>
#include <linux/linkage.h>
#include <linux/freezer.h>
#include <linux/module.h>
#include <linux/debug_locks.h>

#include <asm/processor.h>

#ifdef CONFIG_DEBUG_PSRWLOCK
# include "psrwlock-debug.h"
#else
# include "psrwlock.h"
#endif

#ifdef WBIAS_RWLOCK_DEBUG
#define printk_dbg printk
#else
#define printk_dbg(fmt, args...)
#endif

enum preempt_type {
	PSRW_PREEMPT,		/* preemptable */
	PSRW_NON_PREEMPT,	/* non-preemptable */
};

enum lock_type {
	PSRW_READ,
	PSRW_WRITE,
};

enum v_type {
	V_INT,
	V_LONG,
};

static int rwlock_wait(void *vptr, psrwlock_t *rwlock,
		unsigned long mask, unsigned long test_mask,
		unsigned long full_mask, int check_full_mask,
		enum v_type vtype, enum lock_type ltype, long state,
		unsigned long ip);

/***
 * psrwlock_init - initialize the psrwlock
 * @lock: the psrwlock to be initialized
 * @key: the lock_class_key for the class; used by mutex lock debugging
 *
 * Initialize the psrwlock to unlocked state.
 *
 * It is not allowed to initialize an already locked psrwlock.
 */
void
__psrwlock_init(struct psrwlock *lock, const char *name,
		struct lock_class_key *key, u32 rctx, enum psrw_prio wctx)
{
	unsigned int i;

	atomic_set(&lock->uc, 0);
	atomic_set(&lock->ws, 0);
	for (i = 0; i < PSRW_NR_PRIO; i++)
		atomic_long_set(&lock->prio[i], 0);
	lock->rctx_bitmap = rctx;
	lock->wctx = wctx;
	INIT_LIST_HEAD(&lock->wait_list_r);
	INIT_LIST_HEAD(&lock->wait_list_w);

	debug_psrwlock_init(lock, name, key);
}

EXPORT_SYMBOL(__psrwlock_init);

/*
 * Lock out a specific uncontended execution context from the read lock. Wait
 * for the rmask (readers in previous context count) and for the writer count in
 * the new context not to be full before proceeding to subscribe to the new
 * write context.
 *
 * return values :
 * 1 : lock taken
 * 0 : trylock failed
 * < 0 : interrupted
 */
static int _pswrite_lock_ctx_wait_sub(void *v_inout,
		void *vptr, psrwlock_t *rwlock,
		unsigned long wait_mask, unsigned long test_mask,
		unsigned long full_mask, long offset,
		enum v_type vtype, enum lock_type ltype,
		enum preempt_type ptype, int trylock, long state,
		unsigned long ip)
{
	long try = NR_PREEMPT_BUSY_LOOPS;
	unsigned long newv;
	unsigned long v;
	int ret;

	if (vtype == V_LONG)
		v = *(unsigned long *)v_inout;
	else
		v = *(unsigned int *)v_inout;

	printk_dbg("wait sub start v %lX, new %lX, wait_mask %lX, "
		"test_mask %lX, full_mask %lX, offset %lX\n",
		v, v + offset, wait_mask, test_mask, full_mask, offset);

	for (;;) {
		if (v & wait_mask || (v & test_mask) >= full_mask) {
			lock_contended(&rwlock->dep_map, ip);
			if (trylock)
				return 0;
			if (ptype == PSRW_PREEMPT && unlikely(!(--try))) {
				ret = rwlock_wait(vptr, rwlock, wait_mask,
					test_mask, full_mask, 1,
					vtype, ltype, state, ip);
				if (ret < 0)
					return ret;
				try = NR_PREEMPT_BUSY_LOOPS;
			} else
				cpu_relax();	/* Order v reads */
			if (vtype == V_LONG)
				v = atomic_long_read((atomic_long_t *)vptr);
			else
				v = atomic_read((atomic_t *)vptr);
			continue;
		}
		if (vtype == V_LONG)
			newv = atomic_long_cmpxchg((atomic_long_t *)vptr,
				v, v + offset);
		else
			newv = atomic_cmpxchg((atomic_t *)vptr,
				(int)v, (int)v + (int)offset);
		if (likely(newv == v))
			break;
		else {
			if (trylock)
				return 0;
			v = newv;
		}
	}
	printk_dbg("wait sub end v %lX, new %lX, wait_mask %lX, "
		"test_mask %lX, full_mask %lX, offset %lX\n",
		v, v + offset, wait_mask, test_mask, full_mask, offset);
	/* cmpxchg orders memory reads and writes */
	v += offset;
	if (vtype == V_LONG)
		*(unsigned long *)v_inout = v;
	else
		*(unsigned int *)v_inout = v;
	return 1;
}

/*
 * return values :
 * 1 : lock taken
 * 0 : trylock failed
 * < 0 : interrupted
*/
static int _pswrite_lock_ctx_wait(unsigned long v_in, void *vptr,
		psrwlock_t *rwlock, unsigned long wait_mask,
		enum v_type vtype, enum lock_type ltype,
		enum preempt_type ptype, int trylock, long state,
		unsigned long ip)
{
	int try = NR_PREEMPT_BUSY_LOOPS;
	unsigned long v = v_in;
	int ret;

	printk_dbg("wait start v %lX, wait_mask %lX\n", v, wait_mask);
	/* order all read and write memory operations. */
	smp_mb();
	while (v & wait_mask) {
		if (ptype == PSRW_PREEMPT && unlikely(!(--try))) {
			lock_contended(&rwlock->dep_map, ip);
			if (trylock)
				return 0;
			ret = rwlock_wait(vptr, rwlock, wait_mask, 0, 0, 0,
				vtype, ltype, state, ip);
			if (ret < 0)
				return ret;
			try = NR_PREEMPT_BUSY_LOOPS;
		} else
			cpu_relax();	/* Order v reads */
		if (vtype == V_LONG)
			v = atomic_long_read((atomic_long_t *)vptr);
		else
			v = atomic_read((atomic_t *)vptr);
	}
	/* order all read and write memory operations. */
	smp_mb();
	printk_dbg("wait end v %lX, wait_mask %lX\n", v, wait_mask);
	return 1;
}

/*
 * Go into a wait queue.
 *
 * mask, v & full_mask == full_mask are the conditions for which we wait.
 * return values :
 * 1 : woken up
 * < 0 : interrupted
 */
static int rwlock_wait(void *vptr, psrwlock_t *rwlock,
		unsigned long mask, unsigned long test_mask,
		unsigned long full_mask, int check_full_mask,
		enum v_type vtype, enum lock_type ltype, long state,
		unsigned long ip)
{
	struct task_struct *task = current;
	struct psrwlock_waiter waiter;
	unsigned long v;
	int wq_active, ws, ret = 1;

	/*
	 * Busy-loop waiting for the waitqueue mutex.
	 */
	psrwlock_irq_disable();
	ws = atomic_read(&rwlock->ws);
	_pswrite_lock_ctx_wait_sub(&ws, &rwlock->ws, rwlock,
		0, WS_WQ_MUTEX, WS_WQ_MUTEX, WS_WQ_MUTEX,
		V_INT, ltype, PSRW_NON_PREEMPT, 0, TASK_UNINTERRUPTIBLE, ip);

	debug_psrwlock_lock_common(rwlock, &waiter);

	/*
	 * Got the waitqueue mutex, get into the wait queue.
	 */
	wq_active = !list_empty(&rwlock->wait_list_r)
			|| !list_empty(&rwlock->wait_list_w);
	if (!wq_active)
		atomic_add(UC_WQ_ACTIVE, &rwlock->uc);
	/* Set the UC_WQ_ACTIVE flag before testing the condition. */
	smp_mb();
	/*
	 * Before we go to sleep, check that the lock we were expecting
	 * did not free between the moment we last checked for the lock and the
	 * moment we raised the UC_WQ_ACTIVE flag.
	 */
	if (vtype == V_LONG)
		v = atomic_long_read((atomic_long_t *)vptr);
	else
		v = atomic_read((atomic_t *)vptr);
	if (unlikely(!(v & mask || (check_full_mask
			&& (v & test_mask) >= full_mask))))
		goto skip_sleep;
	/*
	 * got a signal ? (not done in TASK_UNINTERRUPTIBLE)
	 */
	if (unlikely(signal_pending_state(state, task))) {
		ret = -EINTR;
		goto skip_sleep;
	}

	debug_psrwlock_add_waiter(rwlock, &waiter, task_thread_info(task));

	/*
	 * Add waiting tasks to the end of the waitqueue (FIFO):
	 * Only one thread will be woken up at a time.
	 */
	if (ltype == PSRW_WRITE)
		list_add_tail(&waiter.list, &rwlock->wait_list_w);
	else
		list_add_tail(&waiter.list, &rwlock->wait_list_r);
	waiter.task = task;
	__set_task_state(task, state);
	smp_mb();	/* Insure memory ordering when clearing the mutex. */


	atomic_sub(WS_WQ_MUTEX, &rwlock->ws);
	psrwlock_irq_enable();

	try_to_freeze();
	schedule();

	/*
	 * Woken up; Busy-loop waiting for the waitqueue mutex.
	 */
	psrwlock_irq_disable();
	ws = atomic_read(&rwlock->ws);
	_pswrite_lock_ctx_wait_sub(&ws, &rwlock->ws, rwlock,
		0, WS_WQ_MUTEX, WS_WQ_MUTEX, WS_WQ_MUTEX,
		V_INT, ltype, PSRW_NON_PREEMPT, 0, TASK_UNINTERRUPTIBLE, ip);
	__set_task_state(task, TASK_RUNNING);
	psrwlock_remove_waiter(rwlock, &waiter, task_thread_info(task));
skip_sleep:
	wq_active = !list_empty(&rwlock->wait_list_r)
			|| !list_empty(&rwlock->wait_list_w);
	if (!wq_active)
		atomic_sub(UC_WQ_ACTIVE, &rwlock->uc);
	smp_mb();	/* Insure memory ordering when clearing the mutex. */
	atomic_sub(WS_WQ_MUTEX, &rwlock->ws);
	psrwlock_irq_enable();
	debug_psrwlock_free_waiter(&waiter);
	return ret;
}

/*
 * Reader lock
 */

#ifdef CONFIG_DEBUG_PSRWLOCK
static int _psread_lock_fast_check(unsigned int uc, psrwlock_t *rwlock,
	unsigned int uc_rmask)
{
	return 0;
}
#else
/*
 * _psread_lock_fast_check
 *
 * Second cmpxchg taken in case of many active readers.
 * Will busy-loop if cmpxchg fails even in trylock mode.
 *
 * First try to get the uncontended lock. If it is non-zero (can be common,
 * since we allow multiple readers), pass the returned cmpxchg v to the loop
 * to try to get the reader lock.
 *
 * trylock will fail if a writer is subscribed or holds the lock, but will
 * spin if there is concurency to win the cmpxchg. It could happen if, for
 * instance, other concurrent reads need to update the roffset or if a
 * writer updated the lock bits which does not contend us. Since many
 * concurrent readers is a common case, it makes sense not to fail is it
 * happens.
 *
 * the non-trylock case will spin for both situations.
 *
 * Busy-loop if the reader count is full.
 */
static int _psread_lock_fast_check(unsigned int uc, psrwlock_t *rwlock,
	unsigned int uc_rmask)
{
	unsigned int newuc;

	/*
	 * This is the second cmpxchg taken in case of many active readers.
	 */
	while (likely(!(uc & (UC_SLOW_WRITER | UC_WRITER))
			&& (uc & UC_READER_MASK) < uc_rmask)) {
		newuc = atomic_cmpxchg(&rwlock->uc, uc, uc + UC_READER_OFFSET);
		if (likely(newuc == uc))
			return 1;
		else
			uc = newuc;
	}
	return 0;
}
#endif

int __psread_lock_slow(psrwlock_t *rwlock,
		unsigned int uc_rmask, atomic_long_t *vptr,
		int trylock, enum preempt_type ptype, long state,
		unsigned long ip)
{
	u32 rctx = rwlock->rctx_bitmap;
	unsigned long v;
	unsigned int uc;
	int ret;
	int subclass = SINGLE_DEPTH_NESTING;	/* TODO : parameter */

	if (unlikely(in_irq() || irqs_disabled()))
		WARN_ON_ONCE(!(rctx & PSR_IRQ) || ptype != PSRW_NON_PREEMPT);
	else if (in_softirq())
		WARN_ON_ONCE(!(rctx & PSR_BH) || ptype != PSRW_NON_PREEMPT);
#ifdef CONFIG_PREEMPT
	else if (in_atomic())
		WARN_ON_ONCE(!(rctx & PSR_NPTHREAD)
			|| ptype != PSRW_NON_PREEMPT);
	else
		WARN_ON_ONCE(!(rctx & PSR_PTHREAD) || ptype != PSRW_PREEMPT);
#else
	else
		WARN_ON_ONCE((!(rctx & PSR_NPTHREAD)
				|| ptype != PSRW_NON_PREEMPT)
				&& (!(rctx & PSR_PTHREAD)
				|| ptype != PSRW_PREEMPT));
#endif

	psrwlock_acquire_read(&rwlock->dep_map, subclass, trylock, ip);

	/*
	 * A cmpxchg read uc, which implies strict ordering.
	 */
	v = atomic_long_read(vptr);
	ret = _pswrite_lock_ctx_wait_sub(&v, vptr, rwlock,
		CTX_WMASK, CTX_RMASK, CTX_RMASK, CTX_ROFFSET,
		V_LONG, PSRW_READ, ptype, trylock, state, ip);
	if (unlikely(ret < 1))
		goto fail;

	/*
	 * We are in! Well, we just have to busy-loop waiting for any
	 * uncontended writer to release its lock.
	 *
	 * In this exact order :
	 * - increment the uncontended readers count.
	 * - decrement the current context reader count we just previously got.
	 *
	 * This makes sure we always count in either the slow path per context
	 * count or the uncontended reader count starting from the moment we got
	 * the slow path count to the moment we will release the uncontended
	 * reader count at the unlock.
	 *
	 * This implies a strict read/write ordering of these two variables.
	 * Reading first "uc" and then "v" is strictly required. The current
	 * reader count can be summed twice in the worse case, but we are only
	 * interested to know if there is _any_ reader left.
	 */
	uc = atomic_read(&rwlock->uc);
	ret = _pswrite_lock_ctx_wait_sub(&uc, &rwlock->uc, rwlock,
		UC_WRITER, UC_READER_MASK, uc_rmask, UC_READER_OFFSET,
		V_INT, PSRW_READ, ptype, trylock, state, ip);
	/*
	 * _pswrite_lock_ctx_wait_sub has a memory barrier
	 */
	atomic_long_sub(CTX_ROFFSET, vptr);
	/*
	 * don't care about v ordering wrt memory operations inside the
	 * read lock. It's uc which holds our read count.
	 */
	if (unlikely(ret < 1))
		goto fail_preempt;

	lock_acquired(&rwlock->dep_map, ip);
	debug_psrwlock_set_owner(rwlock, (void *)-1UL);	/* -1 : all readers */

	/* Success */
	return 1;

	/* Failure */
fail_preempt:
	/* write v before reading uc */
	smp_mb();
	uc = atomic_read(&rwlock->uc);
	psrwlock_preempt_check(uc, rwlock);
fail:
	cpu_relax();
	psrwlock_release(&rwlock->dep_map, 1, ip);
	return ret;

}

/*
 * _psread_lock_slow : read lock slow path.
 *
 * Non-preemptable :
 * Busy-wait for the specific context lock.
 * Preemptable :
 * Busy-wait for the specific context lock NR_PREEMPT_BUSY_LOOPS loops, and then
 * go to the wait queue.
 *
 * _psread_trylock_slow : read trylock slow path.
 *
 * Try to get the read lock. Returns 1 if succeeds, else returns 0.
 */

asmregparm
void _psread_lock_slow_irq(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_HARDIRQ_READER_MASK);
	if (ret)
		return;
	__psread_lock_slow(rwlock, UC_HARDIRQ_READER_MASK,
			&rwlock->prio[PSRW_PRIO_IRQ],
			0, PSRW_NON_PREEMPT, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(_psread_lock_slow_irq);

asmregparm
void _psread_lock_slow_bh(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_SOFTIRQ_READER_MASK);
	if (ret)
		return;
	__psread_lock_slow(rwlock, UC_SOFTIRQ_READER_MASK,
			&rwlock->prio[PSRW_PRIO_BH],
			0, PSRW_NON_PREEMPT, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(_psread_lock_slow_bh);

asmregparm
void _psread_lock_slow_inatomic(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_NPTHREAD_READER_MASK);
	if (ret)
		return;
	__psread_lock_slow(rwlock, UC_NPTHREAD_READER_MASK,
			&rwlock->prio[PSRW_PRIO_NP],
			0, PSRW_NON_PREEMPT, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(_psread_lock_slow_inatomic);

asmregparm
void _psread_lock_slow(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_PTHREAD_READER_MASK);
	if (ret)
		return;
	__psread_lock_slow(rwlock, UC_PTHREAD_READER_MASK,
			&rwlock->prio[PSRW_PRIO_P],
			0, PSRW_PREEMPT, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(_psread_lock_slow);

asmregparm
int _psread_lock_interruptible_slow(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_PTHREAD_READER_MASK);
	if (ret)
		return 0;
	ret = __psread_lock_slow(rwlock, UC_PTHREAD_READER_MASK,
			&rwlock->prio[PSRW_PRIO_P],
			0, PSRW_PREEMPT, TASK_INTERRUPTIBLE, _RET_IP_);
	if (ret < 1)
		return ret;
	return 0;
}
EXPORT_SYMBOL(_psread_lock_interruptible_slow);

asmregparm
int _psread_trylock_slow_irq(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_HARDIRQ_READER_MASK);
	if (ret)
		return 1;
	return __psread_lock_slow(rwlock, UC_HARDIRQ_READER_MASK,
			&rwlock->prio[PSRW_PRIO_IRQ],
			1, PSRW_NON_PREEMPT, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(_psread_trylock_slow_irq);

asmregparm
int _psread_trylock_slow_bh(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_SOFTIRQ_READER_MASK);
	if (ret)
		return 1;
	return __psread_lock_slow(rwlock, UC_SOFTIRQ_READER_MASK,
			&rwlock->prio[PSRW_PRIO_BH],
			1, PSRW_NON_PREEMPT, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(_psread_trylock_slow_bh);

asmregparm
int _psread_trylock_slow_inatomic(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_NPTHREAD_READER_MASK);
	if (ret)
		return 1;
	return __psread_lock_slow(rwlock, UC_NPTHREAD_READER_MASK,
			&rwlock->prio[PSRW_PRIO_NP],
			1, PSRW_NON_PREEMPT, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(_psread_trylock_slow_inatomic);

asmregparm
int _psread_trylock_slow(unsigned int uc, psrwlock_t *rwlock)
{
	int ret;

	ret = _psread_lock_fast_check(uc, rwlock, UC_PTHREAD_READER_MASK);
	if (ret)
		return 1;
	return __psread_lock_slow(rwlock, UC_PTHREAD_READER_MASK,
			&rwlock->prio[PSRW_PRIO_P],
			1, PSRW_PREEMPT, TASK_UNINTERRUPTIBLE, _RET_IP_);
}
EXPORT_SYMBOL(_psread_trylock_slow);


/* Writer lock */

static int _pswrite_lock_out_context(unsigned int *uc_inout,
	atomic_long_t *vptr, psrwlock_t *rwlock,
	enum preempt_type ptype, int trylock, long state, unsigned long ip)
{
	int ret;
	unsigned long v;

	/* lock out read slow paths */
	v = atomic_long_read(vptr);
	ret = _pswrite_lock_ctx_wait_sub(&v, vptr, rwlock,
		0, CTX_WMASK, CTX_WMASK, CTX_WOFFSET,
		V_LONG, PSRW_WRITE, ptype, trylock, state, ip);
	if (unlikely(ret < 1))
		return ret;
	/*
	 * continue when no reader threads left, but keep subscription, will be
	 * removed by next subscription.
	 */
	ret = _pswrite_lock_ctx_wait(v, vptr, rwlock,
		CTX_RMASK, V_LONG, PSRW_WRITE, ptype, trylock, state, ip);
	if (unlikely(ret < 1))
		goto fail_clean_slow;
	/* Wait for uncontended readers and writers to unlock */
	*uc_inout = atomic_read(&rwlock->uc);
	ret = _pswrite_lock_ctx_wait(*uc_inout, &rwlock->uc, rwlock,
		UC_WRITER | UC_READER_MASK,
		V_INT, PSRW_WRITE, ptype, trylock, state, ip);
	if (ret < 1)
		goto fail_clean_slow;
	return 1;

fail_clean_slow:
	atomic_long_sub(CTX_WOFFSET, vptr);
	return ret;
}

static void writer_count_inc(unsigned int *uc, psrwlock_t *rwlock,
		enum preempt_type ptype, unsigned long ip)
{
	unsigned int ws;

	ws = atomic_read(&rwlock->ws);
	/*
	 * Take the mutex and increment the writer count at once.
	 * Never fail.
	 */
	_pswrite_lock_ctx_wait_sub(&ws, &rwlock->ws, rwlock,
		WS_COUNT_MUTEX, WS_MASK, WS_MASK,
		WS_COUNT_MUTEX + WS_OFFSET,
		V_INT, PSRW_WRITE, ptype, 0, TASK_UNINTERRUPTIBLE, ip);
	/* First writer in slow path ? */
	if ((ws & WS_MASK) == WS_OFFSET) {
		atomic_add(UC_SLOW_WRITER, &rwlock->uc);
		*uc += UC_SLOW_WRITER;
	}
	smp_mb();	/* serialize memory operations with mutex */
	atomic_sub(WS_COUNT_MUTEX, &rwlock->ws);
}

static void writer_count_dec(unsigned int *uc, psrwlock_t *rwlock,
		enum preempt_type ptype, unsigned long ip)
{
	unsigned int ws;

	ws = atomic_read(&rwlock->ws);
	/*
	 * Take the mutex and decrement the writer count at once.
	 * Never fail.
	 */
	_pswrite_lock_ctx_wait_sub(&ws, &rwlock->ws, rwlock,
		WS_COUNT_MUTEX, WS_COUNT_MUTEX, WS_COUNT_MUTEX,
		WS_COUNT_MUTEX - WS_OFFSET,
		V_INT, PSRW_WRITE, ptype, 0, TASK_UNINTERRUPTIBLE, ip);
	/* Last writer in slow path ? */
	if (!(ws & WS_MASK)) {
		atomic_sub(UC_SLOW_WRITER, &rwlock->uc);
		*uc -= UC_SLOW_WRITER;
	}
	smp_mb();	/* serialize memory operations with mutex */
	atomic_sub(WS_COUNT_MUTEX, &rwlock->ws);
}

static int __pswrite_lock_slow_common(unsigned int uc, psrwlock_t *rwlock,
		int trylock, long state, unsigned long ip)
{
	struct task_struct *task = current;
	enum psrw_prio wctx = rwlock->wctx;
	u32 rctx = rwlock->rctx_bitmap;
	enum preempt_type ptype;
	unsigned int ws;
	int ret;
	int subclass = SINGLE_DEPTH_NESTING;	/* TODO : parameter */

	write_context_enable(wctx, rctx);

	if (wctx == PSRW_PRIO_IRQ)
		WARN_ON_ONCE(!in_irq() && !irqs_disabled());
	else if (wctx == PSRW_PRIO_BH)
		WARN_ON_ONCE(!in_softirq());
#ifdef CONFIG_PREEMPT
	else if (wctx == PSRW_PRIO_NP)
		WARN_ON_ONCE(!in_atomic());
#endif

	/*
	 * We got here because the MAY_CONTEND bit is set in the uc bitmask. We
	 * are therefore contending with fast-path or other slow-path writers.
	 * A cmpxchg reads uc, which implies strict ordering.
	 */
	if (wctx == PSRW_PRIO_P)
		ptype = PSRW_PREEMPT;
	else
		ptype = PSRW_NON_PREEMPT;

	psrwlock_acquire(&rwlock->dep_map, subclass, trylock, ip);

	/* Increment the slow path writer count */
	writer_count_inc(&uc, rwlock, ptype, ip);

	if (rctx & PSR_PTHREAD) {
		ptype = PSRW_PREEMPT;
		ret = _pswrite_lock_out_context(&uc,
			&rwlock->prio[PSRW_PRIO_P], rwlock,
			ptype, trylock, state, ip);
		if (unlikely(ret < 1))
			goto fail_dec_count;
	}

	/*
	 * lock out non-preemptable threads.
	 */
	if (rctx & PSR_NPTHREAD) {
		if (wctx != PSRW_PRIO_NP)
			psrwlock_preempt_disable();
		ptype = PSRW_NON_PREEMPT;
		ret = _pswrite_lock_out_context(&uc,
			&rwlock->prio[PSRW_PRIO_NP], rwlock,
			ptype, trylock, state, ip);
		if (unlikely(ret < 1))
			goto fail_unsub_pthread;
	}

	/* lock out softirqs */
	if (rctx & PSR_BH) {
		if (wctx != PSRW_PRIO_BH)
			psrwlock_bh_disable();
		ptype = PSRW_NON_PREEMPT;
		ret = _pswrite_lock_out_context(&uc,
			&rwlock->prio[PSRW_PRIO_BH], rwlock,
			ptype, trylock, state, ip);
		if (unlikely(ret < 1))
			goto fail_unsub_npthread;
	}

	/* lock out hardirqs */
	if (rctx & PSR_IRQ) {
		if (wctx != PSRW_PRIO_IRQ)
			psrwlock_irq_disable();
		ptype = PSRW_NON_PREEMPT;
		ret = _pswrite_lock_out_context(&uc,
			&rwlock->prio[PSRW_PRIO_IRQ], rwlock,
			ptype, trylock, state, ip);
		if (unlikely(ret < 1))
			goto fail_unsub_bh;
	}

	/*
	 * Finally, take the mutex.
	 */
	if (rctx & (PSR_NPTHREAD | PSR_BH | PSR_IRQ))
		ptype = PSRW_NON_PREEMPT;
	else
		ptype = PSRW_PREEMPT;
	ws = atomic_read(&rwlock->ws);
	ret = _pswrite_lock_ctx_wait_sub(&ws, &rwlock->ws, rwlock,
		0, WS_LOCK_MUTEX, WS_LOCK_MUTEX, WS_LOCK_MUTEX,
		V_INT, PSRW_WRITE, ptype, trylock, state, ip);
	if (unlikely(ret < 1))
		goto fail_unsub_irq;
	/* atomic_cmpxchg orders writes */

	lock_acquired(&rwlock->dep_map, ip);
	debug_psrwlock_set_owner(rwlock, task_thread_info(task));

	return 1;	/* success */

	/* Failure paths */
fail_unsub_irq:
	if (rctx & PSR_IRQ)
		atomic_long_sub(CTX_WOFFSET, &rwlock->prio[PSRW_PRIO_IRQ]);
fail_unsub_bh:
	if ((rctx & PSR_IRQ) && wctx != PSRW_PRIO_IRQ)
		psrwlock_irq_enable();
	if (rctx & PSR_BH)
		atomic_long_sub(CTX_WOFFSET, &rwlock->prio[PSRW_PRIO_BH]);
fail_unsub_npthread:
	if ((rctx & PSR_BH) && wctx != PSRW_PRIO_BH)
		psrwlock_bh_enable();
	if (rctx & PSR_NPTHREAD)
		atomic_long_sub(CTX_WOFFSET, &rwlock->prio[PSRW_PRIO_NP]);
fail_unsub_pthread:
	if ((rctx & PSR_NPTHREAD) && wctx != PSRW_PRIO_NP)
		psrwlock_preempt_enable();
	if (rctx & PSR_PTHREAD)
		atomic_long_sub(CTX_WOFFSET, &rwlock->prio[PSRW_PRIO_P]);
fail_dec_count:
	if (wctx == PSRW_PRIO_P)
		ptype = PSRW_PREEMPT;
	else
		ptype = PSRW_NON_PREEMPT;
	writer_count_dec(&uc, rwlock, ptype, ip);
	psrwlock_preempt_check(uc, rwlock);
	cpu_relax();
	psrwlock_release(&rwlock->dep_map, 1, ip);
	return ret;
}

/*
 * _pswrite_lock_slow : Writer-biased rwlock write lock slow path.
 *
 * Locks out execution contexts one by one.
 */
asmregparm void _pswrite_lock_slow(unsigned int uc, psrwlock_t *rwlock)
{
	__pswrite_lock_slow_common(uc, rwlock, 0, TASK_UNINTERRUPTIBLE,
				   _RET_IP_);
}
EXPORT_SYMBOL_GPL(_pswrite_lock_slow);

/*
 * pswrite lock, interruptible.
 */
asmregparm int _pswrite_lock_interruptible_slow(unsigned int uc,
		psrwlock_t *rwlock)
{
	int ret;

	ret = __pswrite_lock_slow_common(uc, rwlock, 0, TASK_INTERRUPTIBLE,
				   _RET_IP_);
	if (ret < 1)
		return ret;
	return 0;
}
EXPORT_SYMBOL_GPL(_pswrite_lock_interruptible_slow);

/*
 * _pswrite_trylock_slow : Try to take a write lock.
 */
asmregparm
int _pswrite_trylock_slow(unsigned int uc, psrwlock_t *rwlock)
{
	return __pswrite_lock_slow_common(uc, rwlock, 1, TASK_INTERRUPTIBLE,
				   _RET_IP_);
}
EXPORT_SYMBOL_GPL(_pswrite_trylock_slow);

asmregparm
void _pswrite_unlock_slow(unsigned int uc, psrwlock_t *rwlock)
{
	enum psrw_prio wctx = rwlock->wctx;
	u32 rctx = rwlock->rctx_bitmap;
	enum preempt_type ptype;
	int nested = 1;	/* FIXME : allow nested = 0 ? */

	mutex_release(&rwlock->dep_map, nested, _RET_IP_);
	debug_psrwlock_unlock(rwlock, 0);
	debug_psrwlock_clear_owner(rwlock);

	/*
	 * We get here either :
	 * - From the fast-path unlock, but a slow-path writer has set the
	 *   UC_SLOW_WRITER bit.
	 * - still having the slowpath locks.
	 *
	 * We have to know if we must decrement the WS_OFFSET count.
	 *
	 * uc, received as parameter, was read by an atomic cmpxchg, which
	 * implies strict memory ordering. It orders memory accesses done within
	 * the critical section with the lock.
	 */
	if (uc & UC_WRITER) {
		uc = atomic_sub_return(UC_WRITER, &rwlock->uc);
		write_context_enable(wctx, rctx);
		psrwlock_preempt_check(uc, rwlock);
	} else {
		/*
		 * Release the slow path lock.
		 */
		smp_mb();	/* insure memory order with lock mutex */
		atomic_sub(WS_LOCK_MUTEX, &rwlock->ws);
		if (rctx & PSR_IRQ) {
			atomic_long_sub(CTX_WOFFSET,
				&rwlock->prio[PSRW_PRIO_IRQ]);
			if (wctx != PSRW_PRIO_IRQ)
				psrwlock_irq_enable();
		}
		if (rctx & PSR_BH) {
			atomic_long_sub(CTX_WOFFSET,
				&rwlock->prio[PSRW_PRIO_BH]);
			if (wctx != PSRW_PRIO_BH)
				psrwlock_bh_enable();
		}
		if (rctx & PSR_NPTHREAD) {
			atomic_long_sub(CTX_WOFFSET,
				&rwlock->prio[PSRW_PRIO_NP]);
			if (wctx != PSRW_PRIO_NP)
				psrwlock_preempt_enable();
		}
		if (rctx & PSR_PTHREAD)
			atomic_long_sub(CTX_WOFFSET,
				&rwlock->prio[PSRW_PRIO_P]);

		if (wctx == PSRW_PRIO_P)
			ptype = PSRW_PREEMPT;
		else
			ptype = PSRW_NON_PREEMPT;
		writer_count_dec(&uc, rwlock, ptype, _RET_IP_);
		psrwlock_preempt_check(uc, rwlock);
	}
}
EXPORT_SYMBOL_GPL(_pswrite_unlock_slow);

/*
 * _psrwlock_wakeup : Wake up tasks waiting for a write or read lock.
 *
 * Called from any context (irq/softirq/preempt/non-preempt). Contains a
 * busy-loop; must therefore disable interrupts, but only for a short time.
 */
asmregparm void _psrwlock_wakeup(unsigned int uc, psrwlock_t *rwlock)
{
	unsigned long flags;
	unsigned int ws;
	struct psrwlock_waiter *waiter;

	/*
	 * Busy-loop waiting for the waitqueue mutex.
	 */
	psrwlock_irq_save(flags);
	/*
	 * Pass PSRW_READ since unused in PSRW_NON_PREEMPT.
	 */
	ws = atomic_read(&rwlock->ws);
	_pswrite_lock_ctx_wait_sub(&ws, &rwlock->ws, rwlock,
		0, WS_WQ_MUTEX, WS_WQ_MUTEX, WS_WQ_MUTEX,
		V_INT, PSRW_READ, PSRW_NON_PREEMPT, 0, TASK_UNINTERRUPTIBLE,
		_RET_IP_);
	/*
	 * If there is at least one non-preemptable writer subscribed or holding
	 * higher priority write masks, let it handle the wakeup when it exits
	 * its critical section which excludes any preemptable context anyway.
	 * The same applies to preemptable readers, which are the only ones
	 * which can cause a preemptable writer to sleep.
	 *
	 * The conditions here are all the states in which we are sure to reach
	 * a preempt check without blocking on the lock.
	 */
	uc = atomic_read(&rwlock->uc);
	if (!(uc & UC_WQ_ACTIVE) || uc & UC_READER_MASK
			|| (atomic_long_read(&rwlock->prio[PSRW_PRIO_IRQ])
				& CTX_WMASK)
			|| (atomic_long_read(&rwlock->prio[PSRW_PRIO_BH])
				& CTX_WMASK)
			|| (atomic_long_read(&rwlock->prio[PSRW_PRIO_NP])
				& CTX_WMASK)) {
		smp_mb();	/*
				 * Insure memory ordering when clearing the
				 * mutex.
				 */
		atomic_sub(WS_WQ_MUTEX, &rwlock->ws);
		psrwlock_irq_restore(flags);
		return;
	}

	/*
	 * First do an exclusive wake-up of the first writer if there is one
	 * waiting, else wake-up the readers.
	 */
	if (!list_empty(&rwlock->wait_list_w))
		waiter = list_entry(rwlock->wait_list_w.next,
				    struct psrwlock_waiter, list);
	else
		waiter = list_entry(rwlock->wait_list_r.next,
				    struct psrwlock_waiter, list);
	debug_psrwlock_wake_waiter(rwlock, waiter);
	wake_up_process(waiter->task);
	smp_mb();	/*
			 * Insure global memory order when clearing the mutex.
			 */
	atomic_sub(WS_WQ_MUTEX, &rwlock->ws);
	psrwlock_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(_psrwlock_wakeup);
