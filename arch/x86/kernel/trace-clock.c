/*
 * arch/x86/kernel/trace-clock.c
 *
 * Trace clock for x86.
 *
 * Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>, October 2008
 */

#include <linux/module.h>
#include <linux/trace-clock.h>
#include <linux/jiffies.h>
#include <linux/timer.h>
#include <linux/cpu.h>

static cycles_t trace_clock_last_tsc;
static DEFINE_PER_CPU(struct timer_list, update_timer);
static DEFINE_SPINLOCK(async_tsc_lock);
static int async_tsc_refcount;	/* Number of readers */
static int async_tsc_enabled;	/* Async TSC enabled on all online CPUs */

int _trace_clock_is_sync = 1;
EXPORT_SYMBOL_GPL(_trace_clock_is_sync);

/*
 * Called by check_tsc_sync_source from CPU hotplug.
 */
void set_trace_clock_is_sync(int state)
{
	_trace_clock_is_sync = state;
}

#if BITS_PER_LONG == 64
static cycles_t read_last_tsc(void)
{
	return trace_clock_last_tsc;
}
#else
/*
 * A cmpxchg64 update can happen concurrently. Based on the assumption that
 * two cmpxchg64 will never update it to the same value (the count always
 * increases), reading it twice insures that we read a coherent value with the
 * same "sequence number".
 */
static cycles_t read_last_tsc(void)
{
	cycles_t val1, val2;

	val1 = trace_clock_last_tsc;
	for (;;) {
		val2 = val1;
		barrier();
		val1 = trace_clock_last_tsc;
		if (likely(val1 == val2))
			break;
	}
	return val1;
}
#endif

/*
 * Support for architectures with non-sync TSCs.
 * When the local TSC is discovered to lag behind the highest TSC counter, we
 * increment the TSC count of an amount that should be, ideally, lower than the
 * execution time of this routine, in cycles : this is the granularity we look
 * for : we must be able to order the events.
 */
notrace cycles_t trace_clock_async_tsc_read(void)
{
	cycles_t new_tsc, last_tsc;

	WARN_ON(!async_tsc_refcount || !async_tsc_enabled);
	rdtsc_barrier();
	new_tsc = get_cycles();
	rdtsc_barrier();
	last_tsc = read_last_tsc();
	do {
		if (new_tsc < last_tsc)
			new_tsc = last_tsc + TRACE_CLOCK_MIN_PROBE_DURATION;
		/*
		 * If cmpxchg fails with a value higher than the new_tsc, don't
		 * retry : the value has been incremented and the events
		 * happened almost at the same time.
		 * We must retry if cmpxchg fails with a lower value :
		 * it means that we are the CPU with highest frequency and
		 * therefore MUST update the value.
		 */
		last_tsc = cmpxchg64(&trace_clock_last_tsc, last_tsc, new_tsc);
	} while (unlikely(last_tsc < new_tsc));
	return new_tsc;
}
EXPORT_SYMBOL_GPL(trace_clock_async_tsc_read);

static void update_timer_ipi(void *info)
{
	(void)trace_clock_async_tsc_read();
}

/*
 * update_timer_fct : - Timer function to resync the clocks
 * @data: unused
 *
 * Fires every jiffy.
 */
static void update_timer_fct(unsigned long data)
{
	(void)trace_clock_async_tsc_read();
	mod_timer_pinned(&per_cpu(update_timer, smp_processor_id()),
			 jiffies + 1);
}

static void enable_trace_clock(int cpu)
{
	init_timer(&per_cpu(update_timer, cpu));
	per_cpu(update_timer, cpu).function = update_timer_fct;
	per_cpu(update_timer, cpu).expires = jiffies + 1;
	smp_call_function_single(cpu, update_timer_ipi, NULL, 1);
	add_timer_on(&per_cpu(update_timer, cpu), cpu);
}

static void disable_trace_clock(int cpu)
{
	del_timer_sync(&per_cpu(update_timer, cpu));
}

/*
 * 	hotcpu_callback - CPU hotplug callback
 * 	@nb: notifier block
 * 	@action: hotplug action to take
 * 	@hcpu: CPU number
 *
 * 	Returns the success/failure of the operation. (NOTIFY_OK, NOTIFY_BAD)
 */
static int __cpuinit hotcpu_callback(struct notifier_block *nb,
				unsigned long action,
				void *hcpu)
{
	unsigned int hotcpu = (unsigned long)hcpu;
	int cpu;

	spin_lock(&async_tsc_lock);
	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		break;
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		/*
		 * trace_clock_is_sync() is updated by set_trace_clock_is_sync()
		 * code, protected by cpu hotplug disable.
		 * It is ok to let the hotplugged CPU read the timebase before
		 * the CPU_ONLINE notification. It's just there to give a
		 * maximum bound to the TSC error.
		 */
		if (async_tsc_refcount && !trace_clock_is_sync()) {
			if (!async_tsc_enabled) {
				async_tsc_enabled = 1;
				for_each_online_cpu(cpu)
					enable_trace_clock(cpu);
			} else {
				enable_trace_clock(hotcpu);
			}
		}
		break;
#ifdef CONFIG_HOTPLUG_CPU
	case CPU_UP_CANCELED:
	case CPU_UP_CANCELED_FROZEN:
		if (!async_tsc_refcount && num_online_cpus() == 1)
			set_trace_clock_is_sync(1);
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		/*
		 * We cannot stop the trace clock on other CPUs when readers are
		 * active even if we go back to a synchronized state (1 CPU)
		 * because the CPU left could be the one lagging behind.
		 */
		if (async_tsc_refcount && async_tsc_enabled)
			disable_trace_clock(hotcpu);
		if (!async_tsc_refcount && num_online_cpus() == 1)
			set_trace_clock_is_sync(1);
		break;
#endif /* CONFIG_HOTPLUG_CPU */
	}
	spin_unlock(&async_tsc_lock);

	return NOTIFY_OK;
}

void get_trace_clock(void)
{
	int cpu;

	if (!trace_clock_is_sync()) {
		printk(KERN_WARNING
			"Trace clock falls back on cache-line bouncing\n"
			"workaround due to non-synchronized TSCs.\n"
			"This workaround preserves event order across CPUs.\n"
			"Please consider disabling Speedstep or PowerNow and\n"
			"using kernel parameters "
			"\"force_tsc_sync=1 idle=poll\"\n"
			"for accurate and fast tracing clock source.\n");
	}

	get_online_cpus();
	spin_lock(&async_tsc_lock);
	if (async_tsc_refcount++ || trace_clock_is_sync())
		goto end;

	async_tsc_enabled = 1;
	for_each_online_cpu(cpu)
		enable_trace_clock(cpu);
end:
	spin_unlock(&async_tsc_lock);
	put_online_cpus();
}
EXPORT_SYMBOL_GPL(get_trace_clock);

void put_trace_clock(void)
{
	int cpu;

	get_online_cpus();
	spin_lock(&async_tsc_lock);
	WARN_ON(async_tsc_refcount <= 0);
	if (async_tsc_refcount != 1 || !async_tsc_enabled)
		goto end;

	for_each_online_cpu(cpu)
		disable_trace_clock(cpu);
	async_tsc_enabled = 0;
end:
	async_tsc_refcount--;
	if (!async_tsc_refcount && num_online_cpus() == 1)
		set_trace_clock_is_sync(1);
	spin_unlock(&async_tsc_lock);
	put_online_cpus();
}
EXPORT_SYMBOL_GPL(put_trace_clock);

static __init int init_unsync_trace_clock(void)
{
	hotcpu_notifier(hotcpu_callback, 4);
	return 0;
}
early_initcall(init_unsync_trace_clock);
