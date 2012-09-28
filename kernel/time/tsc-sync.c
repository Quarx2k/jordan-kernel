/*
 * kernel/time/tsc-sync.c
 *
 * Test TSC synchronization
 *
 * marks the tsc as unstable _and_ keep a simple "_tsc_is_sync" variable, which
 * is fast to read when a simple test must determine which clock source to use
 * for kernel tracing.
 *
 * - CPU init :
 *
 * We check whether all boot CPUs have their TSC's synchronized,
 * print a warning if not and turn off the TSC clock-source.
 *
 * Only two CPUs may participate - they can enter in any order.
 * ( The serial nature of the boot logic and the CPU hotplug lock
 *   protects against more than 2 CPUs entering this code.
 *
 * - When CPUs are up :
 *
 * TSC synchronicity of all CPUs can be checked later at run-time by calling
 * test_tsc_synchronization().
 *
 * Copyright 2007, 2008
 *    Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 */
#include <linux/module.h>
#include <linux/timer.h>
#include <linux/timex.h>
#include <linux/jiffies.h>
#include <linux/trace-clock.h>
#include <linux/cpu.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/cpu.h>

#define MAX_CYCLES_DELTA 1000ULL

/*
 * Number of loops to take care of MCE, NMIs, SMIs.
 */
#define NR_LOOPS	10

static DEFINE_MUTEX(tscsync_mutex);

struct sync_data {
	int nr_waits;
	int wait_sync;
	cycles_t tsc_count;
} ____cacheline_aligned;

/* 0 is master, 1 is slave */
static struct sync_data sync_data[2] = {
	[0 ... 1] = {
		.nr_waits = 3 * NR_LOOPS + 1,
		.wait_sync = 3 * NR_LOOPS + 1,
	},
};

int _tsc_is_sync = 1;
EXPORT_SYMBOL(_tsc_is_sync);

static int force_tsc_sync;
static cycles_t slave_offset;
static int slave_offset_ready;	/* for 32-bits architectures */

static int __init force_tsc_sync_setup(char *str)
{
	force_tsc_sync = simple_strtoul(str, NULL, 0);
	return 1;
}
__setup("force_tsc_sync=", force_tsc_sync_setup);

/*
 * Mark it noinline so we make sure it is not unrolled.
 * Wait until value is reached.
 */
static noinline void tsc_barrier(long this_cpu)
{
	sync_core();
	sync_data[this_cpu].wait_sync--;
	smp_mb();	/* order master/slave sync_data read/write */
	while (unlikely(sync_data[1 - this_cpu].wait_sync >=
			sync_data[this_cpu].nr_waits))
		barrier();	/*
				 * barrier is used because faster and
				 * more predictable than cpu_idle().
				 */
	smp_mb();	/* order master/slave sync_data read/write */
	sync_data[this_cpu].nr_waits--;
	get_cycles_barrier();
	sync_data[this_cpu].tsc_count = get_cycles();
	get_cycles_barrier();
}

/*
 * Worker thread called on each CPU.
 * First wait with interrupts enabled, then wait with interrupt disabled,
 * for precision. We are already bound to one CPU.
 * this_cpu 0 : master
 * this_cpu 1 : slave
 */
static void test_sync(void *arg)
{
	long this_cpu = (long)arg;
	unsigned long flags;

	local_irq_save(flags);
	/* Make sure the instructions are in I-CACHE */
	tsc_barrier(this_cpu);
	tsc_barrier(this_cpu);
	sync_data[this_cpu].wait_sync--;
	smp_mb();	/* order master/slave sync_data read/write */
	while (unlikely(sync_data[1 - this_cpu].wait_sync >=
			sync_data[this_cpu].nr_waits))
		barrier();	/*
				 * barrier is used because faster and
				 * more predictable than cpu_idle().
				 */
	smp_mb();	/* order master/slave sync_data read/write */
	sync_data[this_cpu].nr_waits--;
	/*
	 * Here, only the master will wait for the slave to reach this barrier.
	 * This makes sure that the master, which holds the mutex and will reset
	 * the barriers, waits for the slave to stop using the barrier values
	 * before it continues. This is only done at the complete end of all the
	 * loops. This is why there is a + 1 in original wait_sync value.
	 */
	if (sync_data[this_cpu].nr_waits == 1)
		sync_data[this_cpu].wait_sync--;
	local_irq_restore(flags);
}

/*
 * Each CPU (master and target) must decrement the wait_sync value twice (one
 * for priming in cache), and also once after the get_cycles. After all the
 * loops, one last synchronization is required to make sure the master waits
 * for the slave before resetting the barriers.
 */
static void reset_barriers(void)
{
	int i;

	/*
	 * Wait until slave is done so that we don't overwrite
	 * wait_end_sync prematurely.
	 */
	smp_mb();	/* order master/slave sync_data read/write */
	while (unlikely(sync_data[1].wait_sync >= sync_data[0].nr_waits))
		barrier();	/*
				 * barrier is used because faster and
				 * more predictable than cpu_idle().
				 */
	smp_mb();	/* order master/slave sync_data read/write */

	for (i = 0; i < 2; i++) {
		WARN_ON(sync_data[i].wait_sync != 0);
		WARN_ON(sync_data[i].nr_waits != 1);
		sync_data[i].wait_sync = 3 * NR_LOOPS + 1;
		sync_data[i].nr_waits = 3 * NR_LOOPS + 1;
	}
}

/*
 * Do loops (making sure no unexpected event changes the timing), keep the best
 * one. The result of each loop is the highest tsc delta between the master CPU
 * and the slaves. Stop CPU hotplug when this code is executed to make sure we
 * are concurrency-safe wrt CPU hotplug also using this code.  Test TSC
 * synchronization even if we already "know" CPUs were not synchronized. This
 * can be used as a test to check if, for some reason, the CPUs eventually got
 * in sync after a CPU has been unplugged. This code is kept separate from the
 * CPU hotplug code because the slave CPU executes in an IPI, which we want to
 * keep as short as possible (this is happening while the system is running).
 * Therefore, we do not send a single IPI for all the test loops, but rather
 * send one IPI per loop.
 */
int test_tsc_synchronization(void)
{
	long cpu, master;
	cycles_t max_diff = 0, diff, best_loop, worse_loop = 0;
	int i;

	mutex_lock(&tscsync_mutex);
	get_online_cpus();

	printk(KERN_INFO
	       "checking TSC synchronization across all online CPUs:");

	preempt_disable();
	master = smp_processor_id();
	for_each_online_cpu(cpu) {
		if (master == cpu)
			continue;
		best_loop = (cycles_t)ULLONG_MAX;
		for (i = 0; i < NR_LOOPS; i++) {
			smp_call_function_single(cpu, test_sync,
						(void *)1UL, 0);
			test_sync((void *)0UL);
			diff = abs(sync_data[1].tsc_count
				- sync_data[0].tsc_count);
			best_loop = min(best_loop, diff);
			worse_loop = max(worse_loop, diff);
		}
		reset_barriers();
		max_diff = max(best_loop, max_diff);
	}
	preempt_enable();
	if (max_diff >= MAX_CYCLES_DELTA) {
		printk(KERN_WARNING
			"Measured %llu cycles TSC offset between CPUs,"
			" turning off TSC clock.\n", (u64)max_diff);
		mark_tsc_unstable("check_tsc_sync_source failed");
		_tsc_is_sync = 0;
	} else {
		printk(" passed.\n");
	}
	put_online_cpus();
	mutex_unlock(&tscsync_mutex);
	return max_diff < MAX_CYCLES_DELTA;
}
EXPORT_SYMBOL_GPL(test_tsc_synchronization);

/*
 * Test synchronicity of a single core when it is hotplugged.
 * Source CPU calls into this - waits for the freshly booted target CPU to
 * arrive and then start the measurement:
 */
void __cpuinit check_tsc_sync_source(int cpu)
{
	cycles_t diff, abs_diff,
		 best_loop = (cycles_t)ULLONG_MAX, worse_loop = 0;
	int i;

	/*
	 * No need to check if we already know that the TSC is not synchronized:
	 */
	if (!force_tsc_sync && unsynchronized_tsc()) {
		/*
		 * Make sure we mark _tsc_is_sync to 0 if the TSC is found
		 * to be unsynchronized for other causes than non-synchronized
		 * TSCs across CPUs.
		 */
		_tsc_is_sync = 0;
		set_trace_clock_is_sync(0);
		return;
	}

	printk(KERN_INFO "checking TSC synchronization [CPU#%d -> CPU#%d]:",
			  smp_processor_id(), cpu);

	for (i = 0; i < NR_LOOPS; i++) {
		test_sync((void *)0UL);
		diff = sync_data[1].tsc_count - sync_data[0].tsc_count;
		abs_diff = abs(diff);
		best_loop = min(best_loop, abs_diff);
		worse_loop = max(worse_loop, abs_diff);
		if (force_tsc_sync && best_loop == abs_diff)
			slave_offset = diff;
	}
	reset_barriers();

	if (!force_tsc_sync && best_loop >= MAX_CYCLES_DELTA) {
		printk(" failed.\n");
		printk(KERN_WARNING
			"Measured %llu cycles TSC offset between CPUs,"
			" turning off TSC clock.\n", (u64)best_loop);
		mark_tsc_unstable("check_tsc_sync_source failed");
		_tsc_is_sync = 0;
		set_trace_clock_is_sync(0);
	} else {
		printk(" %s.\n", !force_tsc_sync ? "passed" : "forced");
	}
	if (force_tsc_sync) {
		/* order slave_offset and slave_offset_ready writes */
		smp_wmb();
		slave_offset_ready = 1;
	}
}

/*
 * Freshly booted CPUs call into this:
 */
void __cpuinit check_tsc_sync_target(void)
{
	int i;

	if (!force_tsc_sync && unsynchronized_tsc())
		return;

	for (i = 0; i < NR_LOOPS; i++)
		test_sync((void *)1UL);

	/*
	 * Force slave synchronization if requested.
	 */
	if (force_tsc_sync) {
		unsigned long flags;
		cycles_t new_tsc;

		while (!slave_offset_ready)
			cpu_relax();
		/* order slave_offset and slave_offset_ready reads */
		smp_rmb();
		local_irq_save(flags);
		/*
		 * slave_offset is read when master has finished writing to it,
		 * and is protected by cpu hotplug serialization.
		 */
		new_tsc = get_cycles() - slave_offset;
		write_tsc((u32)new_tsc, (u32)((u64)new_tsc >> 32));
		local_irq_restore(flags);
	}
}
