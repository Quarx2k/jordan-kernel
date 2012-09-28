#ifndef _LINUX_PSRWLOCK_TYPES_H
#define _LINUX_PSRWLOCK_TYPES_H

/*
 * Priority Sifting Reader-Writer Lock types definition
 *
 * Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 * August 2008
 */

#include <linux/list.h>
#include <linux/linkage.h>
#include <linux/lockdep.h>

#include <asm/atomic.h>

/*
 * This table represents which is the lowest read priority context can be used
 * given the highest read priority context and the context in which the write
 * lock is taken.
 *
 * e.g. given the highest priority context from which we take the read lock is
 * interrupt context (IRQ) and the context where the write lock is taken is
 * non-preemptable (NP), we should never have a reader in context lower than
 * NP.
 *
 * X means : don't !
 *
 * X axis : Priority of writer
 * Y axis : Max priority of reader
 * Maps to :  Minimum priority of a reader.
 *
 * Highest Read Prio / Write Prio    | P     NP    BH    IRQ
 * ------------------------------------------------------------------------
 * P                                 | P     X     X     X
 * NP                                | P     NP    X     X
 * BH                                | P     NP    BH    X
 * IRQ                               | P     NP    BH    IRQ
 *
 * This table is verified by the CHECK_PSRWLOCK_MAP macro.
 */

enum psrw_prio {
	PSRW_PRIO_P,
	PSRW_PRIO_NP,
	PSRW_PRIO_BH,
	PSRW_PRIO_IRQ,
	PSRW_NR_PRIO,
};

/*
 * Possible execution contexts for readers.
 */
#define PSR_PTHREAD	(1U << PSRW_PRIO_P)
#define PSR_NPTHREAD	(1U << PSRW_PRIO_NP)
#define PSR_BH		(1U << PSRW_PRIO_BH)
#define PSR_IRQ		(1U << PSRW_PRIO_IRQ)
#define PSR_NR		PSRW_NR_PRIO
#define PSR_MASK	(PSR_PTHREAD | PSR_NPTHREAD | PSR_BH | PSR_IRQ)

typedef struct psrwlock {
	atomic_t uc;			/* Uncontended word	*/
	atomic_t ws;			/* Writers in the slow path count */
	atomic_long_t prio[PSRW_NR_PRIO]; /* Per priority slow path counts */
	u32 rctx_bitmap;		/* Allowed read execution ctx */
	enum psrw_prio wctx;		/* Allowed write execution ctx */
	struct list_head wait_list_r;	/* Preemptable readers wait queue */
	struct list_head wait_list_w;	/* Preemptable writers wait queue */
#ifdef CONFIG_DEBUG_PSRWLOCK
	struct thread_info	*owner;
	const char 		*name;
	void			*magic;
#endif
#ifdef CONFIG_DEBUG_LOCK_ALLOC
	struct lockdep_map      dep_map;
#endif
} psrwlock_t;

/*
 * This is the control structure for tasks blocked on psrwlock,
 * which resides on the blocked task's kernel stack:
 */
struct psrwlock_waiter {
	struct list_head	list;
	struct task_struct	*task;
#ifdef CONFIG_DEBUG_PSRWLOCK
	struct psrwlock		*lock;
	void			*magic;
#endif
};

#ifdef CONFIG_DEBUG_PSRWLOCK
# include <linux/psrwlock-debug.h>
#else
# define __DEBUG_PSRWLOCK_INITIALIZER(lockname)
# define psrwlock_init(psrwlock, _rctx, _wctx)				\
do {									\
	static struct lock_class_key __key;				\
									\
	__psrwlock_init((psrwlock), #psrwlock, &__key, _rctx, _wctx);	\
} while (0)
# define psrwlock_destroy(psrwlock)		do { } while (0)
#endif

#ifdef CONFIG_DEBUG_LOCK_ALLOC
# define __DEP_MAP_PSRWLOCK_INITIALIZER(lockname)			\
		.dep_map = { .name = #lockname },
#else
# define __DEP_MAP_PSRWLOCK_INITIALIZER(lockname)
#endif

#define __PSRWLOCK_UNLOCKED(x, _wctx, _rctx)				\
	{								\
		.uc = { 0 },						\
		.ws = { 0 },						\
		.prio[0 ... (PSRW_NR_PRIO - 1)] = { 0 },		\
		.rctx_bitmap = (_rctx),					\
		.wctx = (_wctx),					\
		.wait_list_r = LIST_HEAD_INIT((x).wait_list_r),		\
		.wait_list_w = LIST_HEAD_INIT((x).wait_list_w),		\
		__DEBUG_PSRWLOCK_INITIALIZER(x)				\
		__DEP_MAP_PSRWLOCK_INITIALIZER(x)			\
	}

#define DEFINE_PSRWLOCK(x, wctx, rctx)					\
	psrwlock_t x = __PSRWLOCK_UNLOCKED(x, wctx, rctx)

/*
 * Statically check that no reader with priority lower than the writer is
 * possible.
 */
#define CHECK_PSRWLOCK_MAP(x, wctx, rctx)				\
	static inline void __psrwlock_bad_context_map_##x(void)		\
	{								\
		MAYBE_BUILD_BUG_ON((~(~0UL << (wctx))) & (rctx));	\
	}

extern void __psrwlock_init(struct psrwlock *lock, const char *name,
			    struct lock_class_key *key,
			    u32 rctx, enum psrw_prio wctx);

/**
 * psrwlock_is_locked - is the psrwlock locked
 * @lock: the psrwlock to be queried
 *
 * Returns 1 if the psrwlock is locked or if any accessor is waiting for it,
 * else returns 0.
 * Also check the per-priority counts to make sure no reader nor writer is
 * within the per-priority slow path waiting period, where they do not appear
 * in the fastpath "uc".
 */
static inline int psrwlock_is_locked(struct psrwlock *lock)
{
	unsigned int i;

	if (atomic_read(&lock->uc))
		return 1;
	for (i = 0; i < PSRW_NR_PRIO; i++)
		if (atomic_long_read(&lock->prio[i]))
			return 1;
	return 0;
}

#endif /* _LINUX_PSRWLOCK_TYPES_H */
