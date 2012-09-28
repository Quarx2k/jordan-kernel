/*
 * Priority Sifting Reader-Writer Lock Debug
 *
 * Inspired from kernel/mutex-debug.c.
 *
 * Copyright 2008 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 */

#include <linux/psrwlock.h>
#include <linux/lockdep.h>
#include <linux/sched.h>
#include <linux/poison.h>
#include <linux/module.h>

#include "psrwlock-debug.h"

/*
 * Must be called with lock->wait_lock held.
 */
void debug_psrwlock_set_owner(struct psrwlock *lock,
			      struct thread_info *new_owner)
{
	lock->owner = new_owner;
}

void debug_psrwlock_lock_common(struct psrwlock *lock,
				struct psrwlock_waiter *waiter)
{
	memset(waiter, PSRWLOCK_DEBUG_INIT, sizeof(*waiter));
	waiter->magic = waiter;
	INIT_LIST_HEAD(&waiter->list);
}

void debug_psrwlock_wake_waiter(struct psrwlock *lock,
				struct psrwlock_waiter *waiter)
{
	SMP_DEBUG_LOCKS_WARN_ON(!(atomic_read(&lock->ws) & WS_WQ_MUTEX));
	DEBUG_LOCKS_WARN_ON(list_empty(&lock->wait_list_r) &&
			    list_empty(&lock->wait_list_w));
	DEBUG_LOCKS_WARN_ON(waiter->magic != waiter);
	DEBUG_LOCKS_WARN_ON(list_empty(&waiter->list));
}

void debug_psrwlock_free_waiter(struct psrwlock_waiter *waiter)
{
	DEBUG_LOCKS_WARN_ON(!list_empty(&waiter->list));
	memset(waiter, PSRWLOCK_DEBUG_FREE, sizeof(*waiter));
}

void debug_psrwlock_add_waiter(struct psrwlock *lock,
			       struct psrwlock_waiter *waiter,
                               struct thread_info *ti)
{
	SMP_DEBUG_LOCKS_WARN_ON(!(atomic_read(&lock->ws) & WS_WQ_MUTEX));

	/* Mark the current thread as blocked on the lock: */
	ti->task->psrwlock_blocked_on = waiter;
	waiter->lock = lock;
}

void psrwlock_remove_waiter(struct psrwlock *lock,
			    struct psrwlock_waiter *waiter,
			    struct thread_info *ti)
{
	DEBUG_LOCKS_WARN_ON(list_empty(&waiter->list));
	DEBUG_LOCKS_WARN_ON(waiter->task != ti->task);
	DEBUG_LOCKS_WARN_ON(ti->task->psrwlock_blocked_on != waiter);
	ti->task->psrwlock_blocked_on = NULL;

	list_del_init(&waiter->list);
	waiter->task = NULL;
}

void debug_psrwlock_unlock(struct psrwlock *lock, int rw)
{
	if (unlikely(!debug_locks))
		return;

	DEBUG_LOCKS_WARN_ON(lock->magic != lock);
	if (rw)	/* read */
		DEBUG_LOCKS_WARN_ON(lock->owner != (void *)-1UL);
	else
		DEBUG_LOCKS_WARN_ON(lock->owner != current_thread_info());
	DEBUG_LOCKS_WARN_ON(!lock->wait_list_r.prev && !lock->wait_list_r.next);
	DEBUG_LOCKS_WARN_ON(!lock->wait_list_w.prev && !lock->wait_list_w.next);
}

void debug_psrwlock_init(struct psrwlock *lock, const char *name,
		      struct lock_class_key *key)
{
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	/*
	 * Make sure we are not reinitializing a held lock:
	 */
	debug_check_no_locks_freed((void *)lock, sizeof(*lock));
	lockdep_init_map(&lock->dep_map, name, key, 0);
#endif
	lock->owner = NULL;
	lock->magic = lock;
}

/***
 * psrwlock_destroy - mark a psrwlock unusable
 * @lock: the psrwlock to be destroyed
 *
 * This function marks the psrwlock uninitialized, and any subsequent
 * use of the lock is forbidden. The lock must not be locked when
 * this function is called.
 */
void psrwlock_destroy(struct psrwlock *lock)
{
	DEBUG_LOCKS_WARN_ON(psrwlock_is_locked(lock));
	lock->magic = NULL;
}

EXPORT_SYMBOL(psrwlock_destroy);

void psread_unlock(psrwlock_t *rwlock, enum psrw_prio wctx, u32 rctx)
{
	int nested = 1;	/* TODO support nested = 0 */
	unsigned int uc;

	psrwlock_release(&rwlock->dep_map, nested, _RET_IP_);
	debug_psrwlock_unlock(rwlock, 1);
	debug_psrwlock_clear_owner(rwlock);
	uc = atomic_sub_return(UC_READER_OFFSET, &rwlock->uc);
	if (wctx == PSRW_PRIO_P || (rctx & PSR_PTHREAD))
		psrwlock_preempt_check(uc, rwlock);
}

EXPORT_SYMBOL(psread_unlock);
