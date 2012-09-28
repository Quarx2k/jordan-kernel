/*
 * linux/include/asm/timex.h
 *
 * sparc64 architecture timex specifications
 */
#ifndef _ASMsparc64_TIMEX_H
#define _ASMsparc64_TIMEX_H

#include <asm/timer.h>

#define CLOCK_TICK_RATE	1193180 /* Underlying HZ */

/* Getting on the cycle counter on sparc64. */
typedef unsigned long cycles_t;

static inline cycles_t get_cycles(void)
{
	return tick_ops->get_tick();
}

/* get_cycles instruction is synchronized on sparc64 */
static inline void get_cycles_barrier(void)
{
	return;
}

extern unsigned long tb_ticks_per_usec;

static inline cycles_t get_cycles_rate(void)
{
	return (cycles_t)tb_ticks_per_usec * 1000000UL;
}

#define ARCH_HAS_READ_CURRENT_TIMER

#endif
