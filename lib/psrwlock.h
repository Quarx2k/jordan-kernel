/*
 * Psrwlock
 *
 * Internal psrwlock prototypes for !CONFIG_DEBUG_PSRWLOCK config.
 */

#define psrwlock_remove_waiter(lock, waiter, ti) \
		__list_del((waiter)->list.prev, (waiter)->list.next)

#define debug_psrwlock_set_owner(lock, new_owner)		do { } while (0)
#define debug_psrwlock_clear_owner(lock)			do { } while (0)
#define debug_psrwlock_wake_waiter(lock, waiter)		do { } while (0)
#define debug_psrwlock_free_waiter(waiter)			do { } while (0)
#define debug_psrwlock_add_waiter(lock, waiter, ti)	do { } while (0)
#define debug_psrwlock_unlock(lock, rw)			do { } while (0)
#define debug_psrwlock_init(lock, name, key)		do { } while (0)

static inline void
debug_psrwlock_lock_common(struct psrwlock *lock,
			   struct psrwlock_waiter *waiter)
{
}
