/*
 * Copyright (C) 2009 Mathieu Desnoyers
 *
 * Trace clock ARM OMAP3 definitions.
 */

#ifndef _ASM_ARM_TRACE_CLOCK_OMAP3_H
#define _ASM_ARM_TRACE_CLOCK_OMAP3_H

#include <linux/clk.h>
#include <linux/timer.h>
#include <plat/clock.h>

/*
 * Number of hardware clock bits. The higher order bits are expected to be 0.
 * If the hardware clock source has more than 32 bits, the bits higher than the
 * 32nd will be truncated by a cast to a 32 bits unsigned. Range : 1 - 32.
 * (too few bits would be unrealistic though, since we depend on the timer to
 * detect the overflows).
 * OMAP3-specific : we clear bit 31 periodically so it never overflows. There
 * is a hardware bug with CP14 and CP15 being executed at the same time a ccnt
 * overflow occurs.
 *
 * Siarhei Siamashka <siarhei.siamashka@nokia.com> :
 * Performance monitoring unit breaks if somebody is accessing CP14/CP15
 * coprocessor register exactly at the same time as CCNT overflows (regardless
 * of the fact if generation of interrupts is enabled or not). A workaround
 * suggested by ARM was to never allow it to overflow and reset it
 * periodically.
 */
#define TC_HW_BITS			31

/* Expected maximum interrupt latency in ms : 15ms, *2 for security */
#define TC_EXPECTED_INTERRUPT_LATENCY	30

/* Resync with 32k clock each 100ms */
#define TC_RESYNC_PERIOD		100

struct tc_cur_freq {
	u64 cur_cpu_freq;	/* in khz */
	/* cur time : (now - base) * (max_freq / cur_freq) + base */
	u32 mul_fact;		/* (max_cpu_freq << 10) / cur_freq */
	u64 hw_base;		/* stamp of last cpufreq change, hw cycles */
	u64 virt_base;		/* same as above, virtual trace clock cycles */
	u64 floor;		/* floor value, so time never go back */
	int need_resync;	/* Need resync after dvfs update ? */
};

/* 32KHz counter per-cpu count save upon PM sleep and cpufreq management */
struct pm_save_count {
	struct tc_cur_freq cf[2];	/* rcu-protected */
	unsigned int index;		/* tc_cur_freq current read index */
	/*
	 * Is fast clock ready to be read ?  Read with preemption off. Modified
	 * only by local CPU in thread and interrupt context or by start/stop
	 * when time is not read concurrently.
	 */
	int fast_clock_ready;

	u64 int_fast_clock;
	struct timer_list clear_ccnt_ms_timer;
	struct timer_list clock_resync_timer;
	u32 ext_32k;
	int refcount;
	u32 init_clock;
	spinlock_t lock;		/* spinlock only sync the refcount */
	/* cpufreq management */
	u64 max_cpu_freq;		/* in khz */
};

DECLARE_PER_CPU(struct pm_save_count, pm_save_count);

extern u64 trace_clock_read_synthetic_tsc(void);
extern void _trace_clock_write_synthetic_tsc(u64 value);
extern unsigned long long cpu_hz;

DECLARE_PER_CPU(int, fast_clock_ready);
extern u64 _trace_clock_read_slow(void);

/*
 * ARM OMAP3 timers only return 32-bits values. We ened to extend it to a
 * 64-bit value, which is provided by trace-clock-32-to-64.
 */
extern u64 trace_clock_async_tsc_read(void);
/*
 * Update done by the architecture upon wakeup.
 */
extern void _trace_clock_write_synthetic_tsc(u64 value);

#ifdef CONFIG_DEBUG_TRACE_CLOCK
DECLARE_PER_CPU(unsigned int, last_clock_nest);
extern void trace_clock_debug(u64 value);
#else
static inline void trace_clock_debug(u64 value)
{
}
#endif

static inline u32 read_ccnt(void)
{
	u32 val;
        __asm__ __volatile__ ("mrc p15, 0, %0, c9, c13, 0" : "=r" (val));
	return val & ~(1 << TC_HW_BITS);
}

static inline u32 trace_clock_read32(void)
{
	u32 val;

	isb();
	val = read_ccnt();
	isb();
	return val;
}

static inline u64 trace_clock_read64(void)
{
	struct pm_save_count *pm_count;
	struct tc_cur_freq *cf;
	u64 val;
#ifdef CONFIG_DEBUG_TRACE_CLOCK
	unsigned long flags;

	local_irq_save(flags);
	per_cpu(last_clock_nest, smp_processor_id())++;
	barrier();
#endif

	preempt_disable();
	pm_count = &per_cpu(pm_save_count, smp_processor_id());
	if (likely(pm_count->fast_clock_ready)) {
		cf = &pm_count->cf[ACCESS_ONCE(pm_count->index)];
		val = max((((trace_clock_read_synthetic_tsc() - cf->hw_base)
		      * cf->mul_fact) >> 10) + cf->virt_base, cf->floor);
	} else
		val = _trace_clock_read_slow();
	trace_clock_debug(val);
	preempt_enable();

#ifdef CONFIG_DEBUG_TRACE_CLOCK
	barrier();
	per_cpu(last_clock_nest, smp_processor_id())--;
	local_irq_restore(flags);
#endif
	return val;
}

static inline u64 trace_clock_frequency(void)
{
	return cpu_hz;
}

static inline u32 trace_clock_freq_scale(void)
{
	return 1;
}

extern void get_trace_clock(void);
extern void put_trace_clock(void);
extern void get_synthetic_tsc(void);
extern void put_synthetic_tsc(void);

extern void resync_trace_clock(void);
extern void save_sync_trace_clock(void);
extern void start_trace_clock(void);
extern void stop_trace_clock(void);

static inline void set_trace_clock_is_sync(int state)
{
}
#endif /* _ASM_MIPS_TRACE_CLOCK_OMAP3_H */
