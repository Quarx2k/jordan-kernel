/*
 * Copyright (C) 2005,2008 Mathieu Desnoyers
 *
 * Trace clock MIPS definitions.
 */

#ifndef _ASM_MIPS_TRACE_CLOCK_H
#define _ASM_MIPS_TRACE_CLOCK_H

#include <linux/timex.h>
#include <asm/processor.h>

#define TRACE_CLOCK_MIN_PROBE_DURATION 200

/*
 * Number of hardware clock bits. The higher order bits are expected to be 0.
 * If the hardware clock source has more than 32 bits, the bits higher than the
 * 32nd will be truncated by a cast to a 32 bits unsigned. Range : 1 - 32.
 * (too few bits would be unrealistic though, since we depend on the timer to
 * detect the overflows).
 */
#define TC_HW_BITS			32

/* Expected maximum interrupt latency in ms : 15ms, *2 for security */
#define TC_EXPECTED_INTERRUPT_LATENCY	30

extern u64 trace_clock_read_synthetic_tsc(void);

/*
 * MIPS get_cycles only returns a 32 bits TSC (see timex.h). The assumption
 * there is that the reschedule is done every 8 seconds or so. Given that
 * tracing needs to detect delays longer than 8 seconds, we need a full 64-bits
 * TSC, whic is provided by trace-clock-32-to-64.
*/

static inline u32 trace_clock_read32(void)
{
	return (u32)get_cycles(); /* only need the 32 LSB */
}

static inline u64 trace_clock_read64(void)
{
	return trace_clock_read_synthetic_tsc();
}

static inline u64 trace_clock_frequency(void)
{
	return get_cycles_rate();
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

extern void get_synthetic_tsc(void);
extern void put_synthetic_tsc(void);

static inline void get_trace_clock(void)
{
	get_synthetic_tsc();
}

static inline void put_trace_clock(void)
{
	put_synthetic_tsc();
}

static inline void set_trace_clock_is_sync(int state)
{
}
#endif /* _ASM_MIPS_TRACE_CLOCK_H */
