/*
 * Priority Sifting Reader-Writer Lock Latency Tracer
 *
 * Copyright 2008 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 */

#include <linux/psrwlock.h>
#include <linux/module.h>
#include <linux/stop_machine.h>
#include <linux/percpu.h>
#include <linux/init.h>
#include <linux/kallsyms.h>

/*
 * Use unsigned long, enough to represent cycle count diff, event on 32-bit
 * arch.
 */

struct psrwlock_latency {
	unsigned long last_disable_cycles, max_latency, min_latency, nr_enable;
	cycles_t total_latency;
	unsigned long max_latency_ip_disable,
		max_latency_ip_enable,
		last_ip_disable;
};

static DEFINE_PER_CPU(struct psrwlock_latency, irq_latency_info);
static DEFINE_PER_CPU(struct psrwlock_latency, softirq_latency_info);
static DEFINE_PER_CPU(struct psrwlock_latency, preempt_latency_info);

static DEFINE_MUTEX(calibration_mutex);
static unsigned long cycles_calibration_min,
		cycles_calibration_avg,
		cycles_calibration_max;

/*
 * Since we are taking the timestamps within the critical section,
 * add the number of cycles it takes to take two consecutive
 * cycles count reads to the total.
 * Returns an unsigned long long for %llu print format.
 */
static unsigned long long calibrate_cycles(cycles_t cycles)
{
	return cycles + cycles_calibration_avg;
}

static void calibrate_get_cycles(void)
{
	int i;
	cycles_t time1, time2;
	unsigned long delay;

	printk(KERN_INFO "** get_cycles calibration **\n");
	cycles_calibration_min = ULLONG_MAX;
	cycles_calibration_avg = 0;
	cycles_calibration_max = 0;

	local_irq_disable();
	for (i = 0; i < 10; i++) {
		get_cycles_barrier();
		time1 = get_cycles();
		get_cycles_barrier();
		get_cycles_barrier();
		time2 = get_cycles();
		get_cycles_barrier();
		delay = time2 - time1;
		cycles_calibration_min = min(cycles_calibration_min, delay);
		cycles_calibration_avg += delay;
		cycles_calibration_max = max(cycles_calibration_max, delay);
	}
	cycles_calibration_avg /= 10;
	local_irq_enable();

	printk(KERN_INFO "get_cycles takes [min,avg,max] %lu,%lu,%lu "
		"cycles, results calibrated on avg\n",
		cycles_calibration_min,
		cycles_calibration_avg,
		cycles_calibration_max);
	printk("\n");
}

static void reset_latency(struct psrwlock_latency *irql)
{
	irql->last_disable_cycles = 0;
	irql->max_latency = 0;
	irql->min_latency = ULONG_MAX;
	irql->total_latency = 0;
	irql->nr_enable = 0;
	irql->max_latency_ip_disable = 0;
	irql->max_latency_ip_enable = 0;
	irql->last_ip_disable = 0;
}

/* can't be in irq disabled section in stop_machine */
static int _psrwlock_profile_latency_reset(void *data)
{
	int cpu = smp_processor_id();

	reset_latency(&per_cpu(irq_latency_info, cpu));
	reset_latency(&per_cpu(softirq_latency_info, cpu));
	reset_latency(&per_cpu(preempt_latency_info, cpu));
	return 0;
}


void psrwlock_profile_latency_reset(void)
{
	mutex_lock(&calibration_mutex);
	printk(KERN_INFO "Writer-biased rwlock latency profiling reset\n");
	calibrate_get_cycles();
	stop_machine(_psrwlock_profile_latency_reset,
			NULL, &cpu_possible_map);
	mutex_unlock(&calibration_mutex);
}
EXPORT_SYMBOL_GPL(psrwlock_profile_latency_reset);

enum irq_latency_type {
	IRQ_LATENCY,
	SOFTIRQ_LATENCY,
	PREEMPT_LATENCY,
};

/*
 * total_irq_latency and nr_irq_enable reads are racy, but it's just an
 * average. Off-by-one is not a big deal.
 */
static void print_latency(const char *typename, enum irq_latency_type type)
{
	struct psrwlock_latency *irql;
	cycles_t avg;
	unsigned long nr_enable;
	int i;

	for_each_online_cpu(i) {
		if (type == IRQ_LATENCY)
			irql = &per_cpu(irq_latency_info, i);
		else if (type == SOFTIRQ_LATENCY)
			irql = &per_cpu(softirq_latency_info, i);
		else
			irql = &per_cpu(preempt_latency_info, i);
		nr_enable = irql->nr_enable;
		if (!nr_enable)
			continue;
		avg = irql->total_latency / (cycles_t)nr_enable;
		printk(KERN_INFO "%s latency for cpu %d "
			"disabled %lu times, "
			"[min,avg,max] %llu,%llu,%llu cycles\n",
			typename, i, nr_enable,
			calibrate_cycles(irql->min_latency),
			calibrate_cycles(avg),
			calibrate_cycles(irql->max_latency));
		printk(KERN_INFO "Max %s latency caused by :\n", typename);
		printk(KERN_INFO "disable : ");
		print_ip_sym(irql->max_latency_ip_disable);
		printk(KERN_INFO "enable : ");
		print_ip_sym(irql->max_latency_ip_enable);
	}
}

void psrwlock_profile_latency_print(void)
{
	mutex_lock(&calibration_mutex);
	printk(KERN_INFO "Writer-biased rwlock latency profiling results\n");
	printk(KERN_INFO "\n");
	print_latency("IRQ", IRQ_LATENCY);
	print_latency("SoftIRQ", SOFTIRQ_LATENCY);
	print_latency("Preemption", PREEMPT_LATENCY);
	printk(KERN_INFO "\n");
	mutex_unlock(&calibration_mutex);
}
EXPORT_SYMBOL_GPL(psrwlock_profile_latency_print);

void psrwlock_profile_irq_disable(void)
{
	struct psrwlock_latency *irql =
		&per_cpu(irq_latency_info, smp_processor_id());

	WARN_ON_ONCE(!irqs_disabled());
	irql->last_ip_disable = _RET_IP_;
	get_cycles_barrier();
	irql->last_disable_cycles = get_cycles();
	get_cycles_barrier();
}
EXPORT_SYMBOL_GPL(psrwlock_profile_irq_disable);

void psrwlock_profile_irq_enable(void)
{
	struct psrwlock_latency *irql;
	unsigned long cur_cycles, diff_cycles;

	get_cycles_barrier();
	cur_cycles = get_cycles();
	get_cycles_barrier();
	irql = &per_cpu(irq_latency_info, smp_processor_id());
	WARN_ON_ONCE(!irqs_disabled());
	if (!irql->last_disable_cycles)
		return;
	diff_cycles = cur_cycles - irql->last_disable_cycles;
	if (diff_cycles > irql->max_latency) {
		irql->max_latency = diff_cycles;
		irql->max_latency_ip_enable = _RET_IP_;
		irql->max_latency_ip_disable = irql->last_ip_disable;
	}
	irql->min_latency = min(irql->min_latency, diff_cycles);
	irql->total_latency += diff_cycles;
	irql->nr_enable++;
}
EXPORT_SYMBOL_GPL(psrwlock_profile_irq_enable);

void psrwlock_profile_bh_disable(void)
{
	struct psrwlock_latency *irql =
		&per_cpu(softirq_latency_info, smp_processor_id());

	WARN_ON_ONCE(!in_softirq());
	irql->last_ip_disable = _RET_IP_;
	get_cycles_barrier();
	irql->last_disable_cycles = get_cycles();
	get_cycles_barrier();
}
EXPORT_SYMBOL_GPL(psrwlock_profile_bh_disable);

void psrwlock_profile_bh_enable(void)
{
	struct psrwlock_latency *irql;
	unsigned long cur_cycles, diff_cycles;

	get_cycles_barrier();
	cur_cycles = get_cycles();
	get_cycles_barrier();
	irql = &per_cpu(softirq_latency_info, smp_processor_id());
	WARN_ON_ONCE(!in_softirq());
	diff_cycles = cur_cycles - irql->last_disable_cycles;
	if (diff_cycles > irql->max_latency) {
		irql->max_latency = diff_cycles;
		irql->max_latency_ip_enable = _RET_IP_;
		irql->max_latency_ip_disable = irql->last_ip_disable;
	}
	irql->min_latency = min(irql->min_latency, diff_cycles);
	irql->total_latency += diff_cycles;
	irql->nr_enable++;
}
EXPORT_SYMBOL_GPL(psrwlock_profile_bh_enable);

#ifdef CONFIG_PREEMPT
void psrwlock_profile_preempt_disable(void)
{
	struct psrwlock_latency *irql =
		&per_cpu(preempt_latency_info, smp_processor_id());

	WARN_ON_ONCE(preemptible());
	irql->last_ip_disable = _RET_IP_;
	get_cycles_barrier();
	irql->last_disable_cycles = get_cycles();
	get_cycles_barrier();
}
EXPORT_SYMBOL_GPL(psrwlock_profile_preempt_disable);

void psrwlock_profile_preempt_enable(void)
{
	struct psrwlock_latency *irql;
	unsigned long cur_cycles, diff_cycles;

	get_cycles_barrier();
	cur_cycles = get_cycles();
	get_cycles_barrier();
	irql = &per_cpu(preempt_latency_info, smp_processor_id());
	WARN_ON_ONCE(preemptible());
	diff_cycles = cur_cycles - irql->last_disable_cycles;
	if (diff_cycles > irql->max_latency) {
		irql->max_latency = diff_cycles;
		irql->max_latency_ip_enable = _RET_IP_;
		irql->max_latency_ip_disable = irql->last_ip_disable;
	}
	irql->min_latency = min(irql->min_latency, diff_cycles);
	irql->total_latency += diff_cycles;
	irql->nr_enable++;
}
EXPORT_SYMBOL_GPL(psrwlock_profile_preempt_enable);
#endif

__init int psrwlock_init(void)
{
	printk(KERN_INFO "psrwlock latency profiling init\n");
	calibrate_get_cycles();
	return 0;
}
device_initcall(psrwlock_init);
