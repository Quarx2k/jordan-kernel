#ifndef _LIB_PSRWLOCK_H
#define _LIB_PSRWLOCK_H

/*
 * This must be called with lock->wait_lock held.
 */
extern void
debug_psrwlock_set_owner(struct psrwlock *lock,
			 struct thread_info *new_owner);

static inline void debug_psrwlock_clear_owner(struct psrwlock *lock)
{
	lock->owner = NULL;
}

extern void debug_psrwlock_lock_common(struct psrwlock *lock,
				       struct psrwlock_waiter *waiter);
extern void debug_psrwlock_wake_waiter(struct psrwlock *lock,
				       struct psrwlock_waiter *waiter);
extern void debug_psrwlock_free_waiter(struct psrwlock_waiter *waiter);
extern void debug_psrwlock_add_waiter(struct psrwlock *lock,
				      struct psrwlock_waiter *waiter,
				      struct thread_info *ti);
extern void psrwlock_remove_waiter(struct psrwlock *lock,
				   struct psrwlock_waiter *waiter,
				   struct thread_info *ti);
extern void debug_psrwlock_unlock(struct psrwlock *lock, int rw);
extern void debug_psrwlock_init(struct psrwlock *lock, const char *name,
				struct lock_class_key *key);

#endif /* _LIB_PSRWLOCK_H */
