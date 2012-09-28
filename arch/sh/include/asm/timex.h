/*
 * linux/include/asm-sh/timex.h
 *
 * sh architecture timex specifications
 */
#ifndef __ASM_SH_TIMEX_H
#define __ASM_SH_TIMEX_H

#include <linux/io.h>
#include <cpu/timer.h>

#define CLOCK_TICK_RATE               (HZ * 100000UL)

typedef unsigned long long cycles_t;

static __inline__ cycles_t get_cycles (void)
{
	return 0xffffffff - ctrl_inl(TMU1_TCNT);
}

#endif /* __ASM_SH_TIMEX_H */
