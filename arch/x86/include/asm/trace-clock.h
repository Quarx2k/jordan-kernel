#ifndef _ASM_X86_TRACE_CLOCK_H
#define _ASM_X86_TRACE_CLOCK_H

/*
 * linux/arch/x86/include/asm/trace-clock.h
 *
 * Copyright (C) 2005,2006,2008
 *   Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Trace clock definitions for x86.
 */

#include <linux/timex.h>
#include <asm/system.h>
#include <asm/processor.h>
#include <asm/atomic.h>

/* Minimum duration of a probe, in cycles */
#define TRACE_CLOCK_MIN_PROBE_DURATION 200

extern cycles_t trace_clock_async_tsc_read(void);

extern int _trace_clock_is_sync;
static inline int trace_clock_is_sync(void)
{
	return _trace_clock_is_sync;
}

static inline u32 trace_clock_read32(void)
{
	u32 cycles;

	if (likely(trace_clock_is_sync())) {
		get_cycles_barrier();
		cycles = (u32)get_cycles(); /* only need the 32 LSB */
		get_cycles_barrier();
	} else
		cycles = (u32)trace_clock_async_tsc_read();
	return cycles;
}

static inline u64 trace_clock_read64(void)
{
	u64 cycles;

	if (likely(trace_clock_is_sync())) {
		get_cycles_barrier();
		cycles = get_cycles();
		get_cycles_barrier();
	} else
		cycles = trace_clock_async_tsc_read();
	return cycles;
}

static inline u64 trace_clock_frequency(void)
{
	return (u64)cpu_khz * 1000;
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

extern void get_trace_clock(void);
extern void put_trace_clock(void);

extern void set_trace_clock_is_sync(int state);

#endif /* _ASM_X86_TRACE_CLOCK_H */
