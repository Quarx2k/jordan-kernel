#ifndef _LINUX_PSRWLOCK_DEBUG_H
#define _LINUX_PSRWLOCK_DEBUG_H

#include <linux/lockdep.h>

/*
 * Priority-Sifting Reader-Writer Locks : debugging helpers:
 */

#define __DEBUG_PSRWLOCK_INITIALIZER(lockname)				\
	.magic = &lockname,

#define psrwlock_init(psrwlock, _rctx, _wctx)				\
do {									\
	static struct lock_class_key __key;				\
									\
	__psrwlock_init((psrwlock), #psrwlock, &__key, _rctx, _wctx);	\
} while (0)

extern void psrwlock_destroy(struct psrwlock *lock);

#endif /* _LINUX_PSRWLOCK_DEBUG_H */
