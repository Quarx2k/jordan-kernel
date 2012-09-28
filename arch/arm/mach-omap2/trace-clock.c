/*
 * arch/arm/mach-omap2/trace-clock.c
 *
 * Trace clock for ARM OMAP3
 *
 * Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca> 2009
 */

#include <linux/module.h>
#include <linux/clocksource.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/init.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>

#include <plat/clock.h>
#include <asm/trace-clock.h>

/* depends on CONFIG_OMAP_32K_TIMER */
/* Need direct access to the clock from arch/arm/mach-omap2/timer-gp.c */
static struct clocksource *clock;

DEFINE_PER_CPU(struct pm_save_count, pm_save_count);
EXPORT_PER_CPU_SYMBOL_GPL(pm_save_count);

static void clear_ccnt_ms(unsigned long data);

/* According to timer32k.c, this is a 32768Hz clock, not a 32000Hz clock. */
#define TIMER_32K_FREQ	32768
#define TIMER_32K_SHIFT	15

/*
 * Clear ccnt twice per 31-bit overflow, or 4 times per 32-bits period.
 */
static u32 clear_ccnt_interval;

static DEFINE_SPINLOCK(trace_clock_lock);
static int trace_clock_refcount;

static int print_info_done;

static u32 get_mul_fact(u64 max_freq, u64 cur_freq)
{
	u64 rem;

	BUG_ON(cur_freq == 0);
	return __iter_div_u64_rem(max_freq << 10, cur_freq, &rem);
}

/*
 * Cycle counter management.
 */

static inline void write_pmnc(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c12, 0" : : "r" (val));
}

static inline u32 read_pmnc(void)
{
	u32 val;
	__asm__ __volatile__ ("mrc p15, 0, %0, c9, c12, 0" : "=r" (val));
        return val;
}

static inline void write_ctens(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c12, 1" : : "r" (val));
}

static inline u32 read_ctens(void)
{
	u32 val;
	__asm__ __volatile__ ("mrc p15, 0, %0, c9, c12, 1" : "=r" (val));
	return val;
}

static inline void write_intenc(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c14, 2" : : "r" (val));
}

static inline u32 read_intenc(void)
{
	u32 val;
        __asm__ __volatile__ ("mrc p15, 0, %0, c9, c14, 2" : "=r" (val));
	return val;
}

static inline void write_useren(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c14, 0" : : "r" (val));
}

static inline u32 read_useren(void)
{
	u32 val;
        __asm__ __volatile__ ("mrc p15, 0, %0, c9, c14, 0" : "=r" (val));
	return val;
}

/*
 * Must disable counter before writing to it.
 */
static inline void write_ccnt(u32 val)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c9, c13, 0" : : "r" (val));
}

/*
 * Periodical timer handler, clears ccnt most significant bit each half-period
 * of 31-bit overflow. Makes sure the ccnt never overflows.
 */
static void clear_ccnt_ms(unsigned long data)
{
	struct pm_save_count *pm_count;
	unsigned int cycles;
	unsigned long flags;
	int cpu;

	cpu = smp_processor_id();
	pm_count = &per_cpu(pm_save_count, cpu);

	local_irq_save(flags);

	if (!pm_count->fast_clock_ready)
		goto end;

	isb();	/* clear the pipeline so we can execute ASAP */
	write_ctens(read_ctens() & ~(1 << 31));	/* disable counter */
	cycles = read_ccnt();
	write_ccnt(cycles & ~(1 << 31));
	isb();
	write_ctens(read_ctens() |  (1 << 31));	/* enable counter */
	isb();
end:
	local_irq_restore(flags);

	mod_timer_pinned(&pm_count->clear_ccnt_ms_timer,
		  jiffies + clear_ccnt_interval);
}

/*
 * disabling interrupts to protect against concurrent IPI save/resync.
 */
void save_sync_trace_clock(void)
{
	struct pm_save_count *pm_count;
	unsigned long flags;
	int cpu;

	local_irq_save(flags);
	cpu = smp_processor_id();
	pm_count = &per_cpu(pm_save_count, cpu);
	__raw_spin_lock(&pm_count->lock);

	if (!pm_count->refcount)
		goto end;

	pm_count->ext_32k = clock->read(clock);
	pm_count->int_fast_clock = trace_clock_read64();
end:
	__raw_spin_unlock(&pm_count->lock);

	/*
	 * Only enable slow read after saving the clock values.
	 */
	barrier();
	pm_count->fast_clock_ready = 0;

	/*
	 * Disable counter to ensure there is no overflow while we are
	 * keeping track of time with ext. clock.
	 */
	write_ctens(read_ctens() & ~(1 << 31));	/* disable counter */
	local_irq_restore(flags);
}

/*
 * Called with preemption disabled. Read the external clock source directly
 * and return corresponding time in fast clock source time frame.
 * Called after time is saved and before it is resynced.
 * Also used to periodically resync the drifting dvfs clock on external clock.
 */
u64 _trace_clock_read_slow(void)
{
	struct pm_save_count *pm_count;
	u64 ref_time;
	unsigned int count_32k;
	int cpu;

	cpu = smp_processor_id();
	pm_count = &per_cpu(pm_save_count, cpu);
	WARN_ON_ONCE(!pm_count->refcount);

	/*
	 * Set the timer's value MSBs to the same as current 32K timer.
	 */
	ref_time = pm_count->int_fast_clock;
	if (!pm_count->init_clock)
		count_32k = clock->read(clock);
	else
		count_32k = pm_count->init_clock;

	/*
	 * Delta done on 32-bits, then casted to u64. Must guarantee
	 * that we are called often enough so the difference does not
	 * overflow 32 bits anyway.
	 */
	ref_time += (u64)(count_32k - pm_count->ext_32k)
			* (cpu_hz >> TIMER_32K_SHIFT);
	return ref_time;
}
EXPORT_SYMBOL_GPL(_trace_clock_read_slow);

/*
 * resynchronize the per-cpu fast clock with the last save_sync values and the
 * external clock. Called from PM (thread) context and IPI context.
 */
void resync_trace_clock(void)
{
	struct pm_save_count *pm_count;
	struct tc_cur_freq *new_cf, *cf;
	unsigned int new_index, index;
	u64 ref_time;
	unsigned long flags;
	u32 regval;
	int cpu;

	local_irq_save(flags);
	cpu = smp_processor_id();
	pm_count = &per_cpu(pm_save_count, cpu);
	__raw_spin_lock(&pm_count->lock);

	if (!pm_count->refcount)
		goto end;

	/* Let userspace access performance counter registers */
	regval = read_useren();
	regval |=  (1 << 0);	/* User mode enable */
	write_useren(regval);

	regval = read_intenc();
	regval |=  (1 << 31);	/* CCNT overflow interrupt disable */
	write_intenc(regval);

	regval = read_pmnc();
	regval |=  (1 << 0);	/* Enable all counters */
	regval &= ~(1 << 3);	/* count every cycles */
	regval &= ~(1 << 5);	/* Enable even in non-invasive debug prohib. */
	write_pmnc(regval);

	ref_time = _trace_clock_read_slow();

	if (pm_count->init_clock)
		pm_count->init_clock = 0;

	write_ctens(read_ctens() & ~(1 << 31));	/* disable counter */
	write_ccnt((u32)ref_time & ~(1 << 31));
	write_ctens(read_ctens() |  (1 << 31));	/* enable counter */

	_trace_clock_write_synthetic_tsc(ref_time);

	index = pm_count->index;
	new_index = 1 - index;
	cf = &pm_count->cf[index];
	new_cf = &pm_count->cf[new_index];
	new_cf->hw_base = ref_time;
	new_cf->virt_base = ref_time;
	new_cf->cur_cpu_freq = cpufreq_quick_get(cpu);
	if (new_cf->cur_cpu_freq == 0)
		new_cf->cur_cpu_freq = pm_count->max_cpu_freq;
	new_cf->mul_fact = get_mul_fact(pm_count->max_cpu_freq,
					new_cf->cur_cpu_freq);
	new_cf->floor = max(ref_time, cf->floor);
	barrier();
	pm_count->index = new_index;
	barrier();	/* make clock ready before enabling */
	pm_count->fast_clock_ready = 1;

	/* Delete resync timer if present. Just done its job anyway. */
	if (pm_count->dvfs_count)
		del_timer(&pm_count->clock_resync_timer);
	pm_count->dvfs_count = 0;

	if (unlikely(!print_info_done)) {
		printk(KERN_INFO "Trace clock using cycle counter at %llu HZ\n"
			"saved 32k clk value 0x%08X, "
			"saved cycle counter value 0x%016llX\n"
			"synthetic value (write, read) 0x%016llX, 0x%016llX\n",
			cpu_hz,
			pm_count->ext_32k,
			pm_count->int_fast_clock,
			ref_time, trace_clock_read64());
		printk(KERN_INFO "Reference clock used : %s\n", clock->name);
		print_info_done = 1;
	}
end:
	__raw_spin_unlock(&pm_count->lock);
	local_irq_restore(flags);
}

/*
 * Called with IRQ and FIQ off.
 */
static void resync_on_32k(struct pm_save_count *pm_count, int cpu,
			  unsigned int cached_freq, int new_freq)
{
	struct tc_cur_freq *new_cf, *cf;
	u64 ref_time;
	unsigned int new_index, index;

	index = pm_count->index;

	new_index = 1 - index;
	cf = &pm_count->cf[index];
	new_cf = &pm_count->cf[new_index];
	ref_time = _trace_clock_read_slow();
	new_cf->hw_base = trace_clock_read_synthetic_tsc();
	new_cf->virt_base = ref_time;
	if (cached_freq)
		new_cf->cur_cpu_freq = cf->cur_cpu_freq;
	else {
		new_cf->cur_cpu_freq = new_freq;
		if (new_cf->cur_cpu_freq == 0)
			new_cf->cur_cpu_freq = pm_count->max_cpu_freq;
	}
	new_cf->mul_fact = get_mul_fact(pm_count->max_cpu_freq,
					new_cf->cur_cpu_freq);
	new_cf->floor = max((((new_cf->hw_base - cf->hw_base)
			    * cf->mul_fact) >> 10) + cf->virt_base,
			    cf->floor);
	barrier();
	pm_count->index = new_index;
}

/*
 * Timer to resynchronize with ext. 32k clock after DVFS update (but not too
 * often if flooded by DVFS updates).
 * Necessary to deal with drift caused by DVFS updates.
 * Per-cpu timer added by cpu freq events, single-shot.
 */
static void clock_resync_timer_fct(unsigned long data)
{
	struct pm_save_count *pm_count;
	unsigned long flags;
	int cpu;

	cpu = smp_processor_id();
	pm_count = &per_cpu(pm_save_count, cpu);

	local_irq_save(flags);
	local_fiq_disable();	/* disable fiqs for floor value */

	/* Need to resync if we had more than 1 dvfs event in period */
	if (pm_count->dvfs_count > 1)
		resync_on_32k(pm_count, cpu, 1, 0);
	pm_count->dvfs_count = 0;

	local_fiq_enable();
	local_irq_restore(flags);
}

static void prepare_timer(int cpu)
{
	struct pm_save_count *pm_count;

	pm_count = &per_cpu(pm_save_count, cpu);
	init_timer_deferrable(&pm_count->clear_ccnt_ms_timer);
	pm_count->clear_ccnt_ms_timer.function = clear_ccnt_ms;
	pm_count->clear_ccnt_ms_timer.expires = jiffies + clear_ccnt_interval;

	init_timer_deferrable(&pm_count->clock_resync_timer);
	pm_count->clock_resync_timer.function = clock_resync_timer_fct;
}

static void enable_timer(int cpu)
{
	struct pm_save_count *pm_count;

	pm_count = &per_cpu(pm_save_count, cpu);
	add_timer_on(&pm_count->clear_ccnt_ms_timer, cpu);
}

static void disable_timer_ipi(void *info)
{
	save_sync_trace_clock();
}

static void disable_timer(int cpu)
{
	struct pm_save_count *pm_count;

	pm_count = &per_cpu(pm_save_count, cpu);
	del_timer_sync(&pm_count->clear_ccnt_ms_timer);
	if (pm_count->dvfs_count)
		del_timer_sync(&pm_count->clock_resync_timer);
	smp_call_function_single(cpu, disable_timer_ipi, NULL, 1);
}

static void resync_ipi(void *info)
{
	resync_trace_clock();
}

void _start_trace_clock(void)
{
	struct pm_save_count *pm_count;
	u32 ext_32k;
	u64 old_fast_clock;
	int cpu;

	ext_32k = clock->read(clock);
	old_fast_clock = per_cpu(pm_save_count, 0).int_fast_clock;

	for_each_online_cpu(cpu) {
		pm_count = &per_cpu(pm_save_count, cpu);
		pm_count->ext_32k = ext_32k;
		pm_count->int_fast_clock = old_fast_clock;
		pm_count->refcount = 1;
		pm_count->init_clock = ext_32k;
		pm_count->dvfs_count = 0;
	}

	on_each_cpu(resync_ipi, NULL, 1);

	get_synthetic_tsc();

	for_each_online_cpu(cpu) {
		prepare_timer(cpu);
		enable_timer(cpu);
	}
}

void _stop_trace_clock(void)
{
	struct pm_save_count *pm_count;
	int cpu;

	per_cpu(pm_save_count, 0).int_fast_clock = trace_clock_read64();

	for_each_online_cpu(cpu) {
		pm_count = &per_cpu(pm_save_count, cpu);
		disable_timer(cpu);
		pm_count->refcount = 0;
	}
	put_synthetic_tsc();
}

void start_trace_clock(void)
{
	spin_lock(&trace_clock_lock);
	if (!trace_clock_refcount)
		goto end;
	_start_trace_clock();
end:
	spin_unlock(&trace_clock_lock);
}

void stop_trace_clock(void)
{
	spin_lock(&trace_clock_lock);
	if (!trace_clock_refcount)
		goto end;
	_stop_trace_clock();
end:
	spin_unlock(&trace_clock_lock);
}

/*
 * 	hotcpu_callback - CPU hotplug callback
 * 	@nb: notifier block
 * 	@action: hotplug action to take
 * 	@hcpu: CPU number
 *
 *	Start/stop timers for trace clock upon cpu hotplug.
 *	Also resync the clock.
 *
 * 	Returns the success/failure of the operation. (NOTIFY_OK, NOTIFY_BAD)
 */
static int __cpuinit hotcpu_callback(struct notifier_block *nb,
				unsigned long action,
				void *hcpu)
{
	struct pm_save_count *pm_count;
	unsigned int hotcpu = (unsigned long)hcpu;
	unsigned long flags;

	switch (action) {
	case CPU_UP_PREPARE:
	case CPU_UP_PREPARE_FROZEN:
		spin_lock(&trace_clock_lock);
		if (trace_clock_refcount) {
			pm_count = &per_cpu(pm_save_count, hotcpu);
			local_irq_save(flags);
			pm_count->ext_32k = clock->read(clock);
			pm_count->int_fast_clock = trace_clock_read64();
			local_irq_restore(flags);
			pm_count->refcount = 1;
			pm_count->dvfs_count = 0;
			prepare_timer(hotcpu);
		}
		spin_unlock(&trace_clock_lock);
		break;
	case CPU_ONLINE:
	case CPU_ONLINE_FROZEN:
		spin_lock(&trace_clock_lock);
		if (trace_clock_refcount) {
			resync_trace_clock();
			enable_timer(hotcpu);
		}
		spin_unlock(&trace_clock_lock);
		break;
#ifdef CONFIG_HOTPLUG_CPU
	case CPU_DOWN_PREPARE:
	case CPU_DOWN_PREPARE_FROZEN:
		spin_lock(&trace_clock_lock);
		if (trace_clock_refcount)
			disable_timer(hotcpu);
		spin_unlock(&trace_clock_lock);
		break;
	case CPU_DEAD:
	case CPU_DEAD_FROZEN:
		spin_lock(&trace_clock_lock);
		if (trace_clock_refcount) {
			pm_count = &per_cpu(pm_save_count, hotcpu);
			pm_count->refcount = 0;
		}
		spin_unlock(&trace_clock_lock);
		break;
#endif /* CONFIG_HOTPLUG_CPU */
	}
	return NOTIFY_OK;
}

void get_trace_clock(void)
{
	spin_lock(&trace_clock_lock);
	if (trace_clock_refcount++)
		goto end;
	_start_trace_clock();
end:
	spin_unlock(&trace_clock_lock);
}
EXPORT_SYMBOL_GPL(get_trace_clock);

void put_trace_clock(void)
{
	spin_lock(&trace_clock_lock);
	WARN_ON(trace_clock_refcount <= 0);
	if (trace_clock_refcount != 1)
		goto end;
	_stop_trace_clock();
end:
	trace_clock_refcount--;
	spin_unlock(&trace_clock_lock);
}
EXPORT_SYMBOL_GPL(put_trace_clock);

/*
 * We do not use prechange hook to sample 2 clock values and average because
 * locking wrt other timers can be difficult to get right.
 * A bit more imprecision just increases the drift. We have a periodic timer
 * in place to resynchronize periodically on the 32k clock anyway.
 */
static int cpufreq_trace_clock(struct notifier_block *nb,
			       unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct pm_save_count *pm_count;
	struct tc_cur_freq *new_cf, *cf;
	unsigned long flags;
	unsigned int new_index, index;
	u64 post_val;
	int cpu;

#if 0 /* debug trace_mark */
	trace_mark(test, freq_change,
		   "%s cpu %u oldfreq %u newfreq %u const %u",
		   (val != CPUFREQ_POSTCHANGE) ? "prechange" : "postchange",
		   freq->cpu, freq->old, freq->new,
		   (freq->flags & CPUFREQ_CONST_LOOPS) ? 1 : 0);
#endif

	if (freq->flags & CPUFREQ_CONST_LOOPS)
		return 0;

	if (val != CPUFREQ_POSTCHANGE)
		return 0;

	local_irq_save(flags);
	cpu = smp_processor_id();
	WARN_ON_ONCE(cpu != freq->cpu);
	pm_count = &per_cpu(pm_save_count, cpu);
	__raw_spin_lock(&pm_count->lock);

	if (!pm_count->refcount)
		goto end;

	/*
	 * Disable FIQs to ensure the floor value is indeed the
	 * floor.
	 */
	local_fiq_disable();

	if (!pm_count->dvfs_count) {
		resync_on_32k(pm_count, cpu, 0, freq->new);
		pm_count->clock_resync_timer.expires = jiffies
					+ (TC_RESYNC_PERIOD * HZ / 1000);
		add_timer_on(&pm_count->clock_resync_timer, cpu);
	} else {
		post_val = trace_clock_read_synthetic_tsc();
		/* disable irqs to ensure we are the only value modifier */
		index = pm_count->index;
		new_index = 1 - index;
		cf = &pm_count->cf[index];
		new_cf = &pm_count->cf[new_index];
		new_cf->hw_base = post_val;
		new_cf->virt_base = (((post_val - cf->hw_base)
				      * cf->mul_fact) >> 10) + cf->virt_base;
		new_cf->cur_cpu_freq = freq->new;
		new_cf->mul_fact = get_mul_fact(pm_count->max_cpu_freq,
						freq->new);
		new_cf->floor = max((((post_val - cf->hw_base)
				      * cf->mul_fact) >> 10) + cf->virt_base,
				    cf->floor);
		barrier();
		pm_count->index = new_index;
	}

	local_fiq_enable();
	pm_count->dvfs_count++;
end:
	__raw_spin_unlock(&pm_count->lock);
	local_irq_restore(flags);
	return 0;
}

static struct notifier_block cpufreq_trace_clock_nb = {
	.notifier_call = cpufreq_trace_clock,
};

#ifdef CONFIG_DEBUG_TRACE_CLOCK
/*
 * Clock expected to never overflow and never go backward.
 */
static DEFINE_PER_CPU(u64, last_clock_value);
static DEFINE_PER_CPU(u32, last_ccnt_value);
DEFINE_PER_CPU(unsigned int, last_clock_nest);
EXPORT_PER_CPU_SYMBOL_GPL(last_clock_nest);

static int tc_print_done;

/*
 * Called with interrupts disabled.
 */
void trace_clock_debug(u64 value)
{
	int cpu;

	cpu = smp_processor_id();
	if (unlikely(per_cpu(last_clock_nest, cpu) != 1))
		return;		/* fiq nesting, don't perform racy check */
	if (unlikely(!tc_print_done
		     && (per_cpu(last_clock_value, cpu) > value))) {
		printk(KERN_WARNING "Trace clock going back last %llu new %llu "
				    "diff %llu last_ccnt %u ccnt %u\n",
		       (unsigned long long) per_cpu(last_clock_value, cpu),
		       (unsigned long long) value,
		       (unsigned long long) per_cpu(last_clock_value, cpu)
					    - value,
		       per_cpu(last_ccnt_value, cpu),
		       trace_clock_read32());
		tc_print_done = 1;
	}
	per_cpu(last_clock_value, cpu) = value;
	per_cpu(last_ccnt_value, cpu) = trace_clock_read32();;
}
EXPORT_SYMBOL_GPL(trace_clock_debug);
#endif

static __init int init_trace_clock(void)
{
	int cpu;
	u64 rem;

	clock = get_clocksource_32k();
	/*
	 * clear_ccnt_interval based on the cpu fastest frequency. Never
	 * recomputed.
	 */
	clear_ccnt_interval = __iter_div_u64_rem(HZ * (1ULL << 30), cpu_hz,
						 &rem);
	printk(KERN_INFO "LTTng will clear ccnt top bit every %u jiffies.\n",
		clear_ccnt_interval);
	for_each_possible_cpu(cpu) {
		per_cpu(pm_save_count, cpu).max_cpu_freq =
			__iter_div_u64_rem(cpu_hz, 1000, &rem);
		per_cpu(pm_save_count, cpu).lock =
			(raw_spinlock_t)__RAW_SPIN_LOCK_UNLOCKED;
	}
	hotcpu_notifier(hotcpu_callback, 4);
	cpufreq_register_notifier(&cpufreq_trace_clock_nb,
				  CPUFREQ_TRANSITION_NOTIFIER);
	return 0;
}
__initcall(init_trace_clock);
