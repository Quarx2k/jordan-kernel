/*
 * kernel/trace/trace-clock-32-to-64.c
 *
 * (C) Copyright	2006,2007,2008 -
 * 		Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Extends a 32 bits clock source to a full 64 bits count, readable atomically
 * from any execution context.
 *
 * notes :
 * - trace clock 32->64 bits extended timer-based clock cannot be used for early
 *   tracing in the boot process, as it depends on timer interrupts.
 * - The timer is only on one CPU to support hotplug.
 * - We have the choice between schedule_delayed_work_on and an IPI to get each
 *   CPU to write the heartbeat. IPI has been chosen because it is considered
 *   faster than passing through the timer to get the work scheduled on all the
 *   CPUs.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/timex.h>
#include <linux/bitops.h>
#include <linux/trace-clock.h>
#include <linux/smp.h>
#include <linux/sched.h> /* needed due to include order problem on m68k */
#include <linux/math64.h>

#define HW_BITMASK			((1ULL << TC_HW_BITS) - 1)
#define HW_LS32(hw)			((hw) & HW_BITMASK)
#define SW_MS32(sw)			((sw) & ~HW_BITMASK)

static DEFINE_SPINLOCK(synthetic_tsc_lock);
static int synthetic_tsc_refcount;  /* Number of readers */
static int synthetic_tsc_enabled;   /* synth. TSC enabled on all online CPUs */

static DEFINE_PER_CPU(struct timer_list, tsc_timer);
static unsigned int precalc_expire;

struct synthetic_tsc_struct {
	union {
		u64 val;
		struct {
#ifdef __BIG_ENDIAN
			u32 ms32;
			u32 ls32;
#else
			u32 ls32;
			u32 ms32;
#endif
		} sel;
	} tsc[2];
	unsigned int index;	/* Index of the current synth. tsc. */
};

static DEFINE_PER_CPU(struct synthetic_tsc_struct, synthetic_tsc);

/* Called from IPI or timer interrupt */
static void update_synthetic_tsc(void)
{
	struct synthetic_tsc_struct *cpu_synth;
	u32 tsc;

	cpu_synth = &per_cpu(synthetic_tsc, smp_processor_id());
	tsc = trace_clock_read32();		/* Hardware clocksource read */

	if (tsc < HW_LS32(cpu_synth->tsc[cpu_synth->index].sel.ls32)) {
		unsigned int new_index = 1 - cpu_synth->index; /* 0 <-> 1 */
		/*
		 * Overflow
		 * Non atomic update of the non current synthetic TSC, followed
		 * by an atomic index change. There is no write concurrency,
		 * so the index read/write does not need to be atomic.
		 */
		cpu_synth->tsc[new_index].val =
			(SW_MS32(cpu_synth->tsc[cpu_synth->index].val)
				| (u64)tsc) + (1ULL << TC_HW_BITS);
		/*
		 * Ensure the compiler does not reorder index write. It makes
		 * sure all nested interrupts will see the new value before the
		 * new index is written.
		 */
		barrier();
		cpu_synth->index = new_index;	/* atomic change of index */
	} else {
		/*
		 * No overflow : We know that the only bits changed are
		 * contained in the 32 LS32s, which can be written to atomically.
		 */
		cpu_synth->tsc[cpu_synth->index].sel.ls32 =
			SW_MS32(cpu_synth->tsc[cpu_synth->index].sel.ls32) | tsc;
	}
}

/*
 * Should only be called when interrupts are off. Affects only current CPU.
 */
void _trace_clock_write_synthetic_tsc(u64 value)
{
	struct synthetic_tsc_struct *cpu_synth;
	unsigned int new_index;

	cpu_synth = &per_cpu(synthetic_tsc, smp_processor_id());
	new_index = 1 - cpu_synth->index; /* 0 <-> 1 */
	cpu_synth->tsc[new_index].val = value;
	barrier();
	cpu_synth->index = new_index;	/* atomic change of index */
}

/* Called from buffer switch : in _any_ context (even NMI) */
u64 notrace trace_clock_read_synthetic_tsc(void)
{
	struct synthetic_tsc_struct *cpu_synth;
	u64 ret;
	unsigned int index;
	u32 tsc;

	preempt_disable_notrace();
	cpu_synth = &per_cpu(synthetic_tsc, smp_processor_id());
	index = ACCESS_ONCE(cpu_synth->index);	/* atomic read */
	tsc = trace_clock_read32();		/* Hardware clocksource read */

	/* Overflow detection */
	if (unlikely(tsc < HW_LS32(cpu_synth->tsc[index].sel.ls32)))
		ret = (SW_MS32(cpu_synth->tsc[index].val) | (u64)tsc)
			+ (1ULL << TC_HW_BITS);
	else
		ret = SW_MS32(cpu_synth->tsc[index].val) | (u64)tsc;
	preempt_enable_notrace();
	return ret;
}
EXPORT_SYMBOL_GPL(trace_clock_read_synthetic_tsc);

static void synthetic_tsc_ipi(void *info)
{
	update_synthetic_tsc();
}

/*
 * tsc_timer_fct : - Timer function synchronizing synthetic TSC.
 * @data: unused
 *
 * Guarantees at least 1 execution before low word of TSC wraps.
 */
static void tsc_timer_fct(unsigned long data)
{
	update_synthetic_tsc();

	mod_timer_pinned(&per_cpu(tsc_timer, smp_processor_id()),
		  jiffies + precalc_expire);
}

/*
 * precalc_stsc_interval: - Precalculates the interval between the clock
 * wraparounds.
 */
static int __init precalc_stsc_interval(void)
{
	u64 rem_freq, rem_interval;

	precalc_expire =
		__iter_div_u64_rem(HW_BITMASK, (
		  __iter_div_u64_rem(trace_clock_frequency(),
		  HZ * trace_clock_freq_scale(), &rem_freq) << 1
		 )
		 - 1
		 - (TC_EXPECTED_INTERRUPT_LATENCY * HZ / 1000), &rem_interval)
		>> 1;
	WARN_ON(precalc_expire == 0);
	printk(KERN_DEBUG "Synthetic TSC timer will fire each %u jiffies.\n",
		precalc_expire);
	return 0;
}

static void prepare_synthetic_tsc(int cpu)
{
	struct synthetic_tsc_struct *cpu_synth;
	u64 local_count;

	cpu_synth = &per_cpu(synthetic_tsc, cpu);
	local_count = trace_clock_read_synthetic_tsc();
	cpu_synth->tsc[0].val = local_count;
	cpu_synth->index = 0;
	smp_wmb();	/* Writing in data of CPU about to come up */
	init_timer_deferrable(&per_cpu(tsc_timer, cpu));
	per_cpu(tsc_timer, cpu).function = tsc_timer_fct;
	per_cpu(tsc_timer, cpu).expires = jiffies + precalc_expire;
}

static void enable_synthetic_tsc(int cpu)
{
	smp_call_function_single(cpu, synthetic_tsc_ipi, NULL, 1);
	add_timer_on(&per_cpu(tsc_timer, cpu), cpu);
}

static void disable_synthetic_tsc(int cpu)
{
	del_timer_sync(&per_cpu(tsc_timer, smp_processor_id()));
}

/*
 * 	hotcpu_callback - CPU hotplug callback
 * 	@nb: notifier block
 * 	@action: hotplug action to take
 * 	@hcpu: CPU number
 *
 *	Sets the new CPU's current synthetic TSC to the same value as the
 *	currently running CPU.
 *
 * 	Returns the success/failure of the operation. (NOTIFY_OK, NOTIFY_BAD)
 */
static int __cpuinit hotcpu_callback(struct notifier_block *nb,
				unsigned long action,
				void *hcpu)
{
	unsigned int hotcpu = (unsigned long)hcpu;

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		spin_lock(&synthetic_tsc_lock);
		if (synthetic_tsc_refcount)
			prepare_synthetic_tsc(hotcpu);
		spin_unlock(&synthetic_tsc_lock);
		break;
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		spin_lock(&synthetic_tsc_lock);
		if (synthetic_tsc_refcount)
			enable_synthetic_tsc(hotcpu);
		spin_unlock(&synthetic_tsc_lock);
		break;
#ifdef CONFIG_HOTPLUG_CPU
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		spin_lock(&synthetic_tsc_lock);
		if (synthetic_tsc_refcount)
			disable_synthetic_tsc(hotcpu);
		spin_unlock(&synthetic_tsc_lock);
		break;
#endif /* CONFIG_HOTPLUG_CPU */
	}
	return NOTIFY_OK;
}

void get_synthetic_tsc(void)
{
	int cpu;

	spin_lock(&synthetic_tsc_lock);
	if (synthetic_tsc_refcount++)
		goto end;

	synthetic_tsc_enabled = 1;
	for_each_online_cpu(cpu) {
		prepare_synthetic_tsc(cpu);
		enable_synthetic_tsc(cpu);
	}
end:
	spin_unlock(&synthetic_tsc_lock);
}
EXPORT_SYMBOL_GPL(get_synthetic_tsc);

void put_synthetic_tsc(void)
{
	int cpu;

	spin_lock(&synthetic_tsc_lock);
	WARN_ON(synthetic_tsc_refcount <= 0);
	if (synthetic_tsc_refcount != 1 || !synthetic_tsc_enabled)
		goto end;

	for_each_online_cpu(cpu)
		disable_synthetic_tsc(cpu);
	synthetic_tsc_enabled = 0;
end:
	synthetic_tsc_refcount--;
	spin_unlock(&synthetic_tsc_lock);
}
EXPORT_SYMBOL_GPL(put_synthetic_tsc);

/* Called from CPU 0, before any tracing starts, to init each structure */
static int __init init_synthetic_tsc(void)
{
	precalc_stsc_interval();
	hotcpu_notifier(hotcpu_callback, 3);
	return 0;
}

/* Before SMP is up */
early_initcall(init_synthetic_tsc);
