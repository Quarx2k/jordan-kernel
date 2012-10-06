/*
 * drivers/cpufreq/cpufreq_smoothass.c
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Author: Erasmux
 *
 * Based on the interactive governor By Mike Chan (mike@android.com)
 * which was adaptated to 2.6.29 kernel by Nadlabak (pavel@doshaska.net)
 * 
 * requires to add
 * EXPORT_SYMBOL_GPL(nr_running);
 * at the end of kernel/sched.c
 *
 */

#include &lt;linux/cpu.h&gt;
#include &lt;linux/cpumask.h&gt;
#include &lt;linux/cpufreq.h&gt;
#include &lt;linux/sched.h&gt;
#include &lt;linux/tick.h&gt;
#include &lt;linux/timer.h&gt;
#include &lt;linux/workqueue.h&gt;
#include &lt;linux/moduleparam.h&gt;
#include &lt;asm/cputime.h&gt;
#include &lt;linux/earlysuspend.h&gt;

static void (*pm_idle_old)(void);
static atomic_t active_count = ATOMIC_INIT(0);

struct smoothass_info_s {
	struct cpufreq_policy *cur_policy;
	struct timer_list timer;
	u64 time_in_idle;
	u64 idle_exit_time;
	unsigned int force_ramp_up;
	unsigned int enable;
};
static DEFINE_PER_CPU(struct smoothass_info_s, smoothass_info);

/* Workqueues handle frequency scaling */
static struct workqueue_struct *up_wq;
static struct workqueue_struct *down_wq;
static struct work_struct freq_scale_work;

static u64 freq_change_time;
static u64 freq_change_time_in_idle;

static cpumask_t work_cpumask;
static unsigned int suspended;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down,
 * default is 45ms.
 */
#define DEFAULT_DOWN_RATE_US 45000
static unsigned long down_rate_us;

/*
 * When ramping up frequency with no idle cycles jump to at least this frequency.
 * Zero disables. Set a very high value to jump to policy max freqeuncy.
 */
#define DEFAULT_UP_MIN_FREQ 998400
static unsigned int up_min_freq;

/*
 * When sleep_max_freq&gt;0 the frequency when suspended will be capped
 * by this frequency. Also will wake up at max frequency of policy
 * to minimize wakeup issues.
 * Set sleep_max_freq=0 to disable this behavior.
 */
#define DEFAULT_SLEEP_MAX_FREQ 245760
static unsigned int sleep_max_freq;

/*
 * Sampling rate, I highly recommend to leave it at 2.
 */
#define DEFAULT_SAMPLE_RATE_JIFFIES 2
static unsigned int sample_rate_jiffies;

/*
 * Freqeuncy delta when ramping up.
 * zero disables causes to always jump straight to max frequency.
 */
#define DEFAULT_RAMP_UP_STEP 100000
static unsigned int ramp_up_step;

/*
 * Max frequency delta when ramping down. zero disables.
 */
#define DEFAULT_MAX_RAMP_DOWN 100000
static unsigned int max_ramp_down;

/*
 * CPU freq will be increased if measured load &gt; max_cpu_load;
 */
#define DEFAULT_MAX_CPU_LOAD 60
static unsigned long max_cpu_load;

/*
 * CPU freq will be decreased if measured load &lt; min_cpu_load;
 */
#define DEFAULT_MIN_CPU_LOAD 30
static unsigned long min_cpu_load;


static int cpufreq_governor_smoothass(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SMOOTHASS
static
#endif
struct cpufreq_governor cpufreq_gov_smoothass = {
	.name = "smoothass",
	.governor = cpufreq_governor_smoothass,
	.max_transition_latency = 6000000,
	.owner = THIS_MODULE,
};

static void cpufreq_smoothass_timer(unsigned long data)
{
	u64 delta_idle;
	u64 update_time;
	u64 now_idle;
	struct smoothass_info_s *this_smoothass = &amp;per_cpu(smoothass_info, data);
	struct cpufreq_policy *policy = this_smoothass-&gt;cur_policy;

	now_idle = get_cpu_idle_time_us(data, &amp;update_time);

	if (update_time == this_smoothass-&gt;idle_exit_time)
		return;

	delta_idle = cputime64_sub(now_idle, this_smoothass-&gt;time_in_idle);
	//printk(KERN_INFO "smoothass: t=%llu i=%llu\n",cputime64_sub(update_time,this_smoothass-&gt;idle_exit_time),delta_idle);

	/* Scale up if there were no idle cycles since coming out of idle */
	if (delta_idle == 0) {
		if (policy-&gt;cur == policy-&gt;max)
			return;

		if (nr_running() &lt; 1)
			return;

		this_smoothass-&gt;force_ramp_up = 1;
		cpumask_set_cpu(data, &amp;work_cpumask);
		queue_work(up_wq, &amp;freq_scale_work);
		return;
	}

	/*
	 * There is a window where if the cpu utlization can go from low to high
	 * between the timer expiring, delta_idle will be &gt; 0 and the cpu will
	 * be 100% busy, preventing idle from running, and this timer from
	 * firing. So setup another timer to fire to check cpu utlization.
	 * Do not setup the timer if there is no scheduled work.
	 */
	if (!timer_pending(&amp;this_smoothass-&gt;timer) &amp;&amp; nr_running() &gt; 0) { 
			this_smoothass-&gt;time_in_idle = get_cpu_idle_time_us(
					data, &amp;this_smoothass-&gt;idle_exit_time);
			mod_timer(&amp;this_smoothass-&gt;timer, jiffies + sample_rate_jiffies);
	}

	if (policy-&gt;cur == policy-&gt;min)
		return;

	/*
	 * Do not scale down unless we have been at this frequency for the
	 * minimum sample time.
	 */
	if (cputime64_sub(update_time, freq_change_time) &lt; down_rate_us)
		return;

	cpumask_set_cpu(data, &amp;work_cpumask);
	queue_work(down_wq, &amp;freq_scale_work);
}

static void cpufreq_idle(void)
{
	struct smoothass_info_s *this_smoothass = &amp;per_cpu(smoothass_info, smp_processor_id());
	struct cpufreq_policy *policy = this_smoothass-&gt;cur_policy;

	pm_idle_old();

	if (!cpumask_test_cpu(smp_processor_id(), policy-&gt;cpus))
			return;

	/* Timer to fire in 1-2 ticks, jiffie aligned. */
	if (timer_pending(&amp;this_smoothass-&gt;timer) == 0) {
		this_smoothass-&gt;time_in_idle = get_cpu_idle_time_us(
				smp_processor_id(), &amp;this_smoothass-&gt;idle_exit_time);
		mod_timer(&amp;this_smoothass-&gt;timer, jiffies + sample_rate_jiffies);
	}
}

/*
 * Choose the cpu frequency based off the load. For now choose the minimum
 * frequency that will satisfy the load, which is not always the lower power.
 */
static unsigned int cpufreq_smoothass_calc_freq(unsigned int cpu, struct cpufreq_policy *policy)
{
	unsigned int delta_time;
	unsigned int idle_time;
	unsigned int cpu_load;
	unsigned int new_freq;
	u64 current_wall_time;
	u64 current_idle_time;

	current_idle_time = get_cpu_idle_time_us(cpu, &amp;current_wall_time);

	idle_time = (unsigned int)( current_idle_time - freq_change_time_in_idle );
	delta_time = (unsigned int)( current_wall_time - freq_change_time );

	cpu_load = 100 * (delta_time - idle_time) / delta_time;
	//printk(KERN_INFO "Smoothass calc_freq: delta_time=%u cpu_load=%u\n",delta_time,cpu_load);
	if (cpu_load &lt; min_cpu_load) {
		cpu_load += 100 - max_cpu_load; // dummy load.
		new_freq = policy-&gt;cur * cpu_load / 100;
		if (max_ramp_down &amp;&amp; new_freq &lt; policy-&gt;cur - max_ramp_down)
			new_freq = policy-&gt;cur - max_ramp_down;
		//printk(KERN_INFO "Smoothass calc_freq: %u =&gt; %u\n",policy-&gt;cur,new_freq);
		return new_freq;
	} if (cpu_load &gt; max_cpu_load) {
		if (ramp_up_step)
			new_freq = policy-&gt;cur + ramp_up_step;
		else
			new_freq = policy-&gt;max;
		return new_freq;
	}
	return policy-&gt;cur;
}

/* We use the same work function to sale up and down */
static void cpufreq_smoothass_freq_change_time_work(struct work_struct *work)
{
	unsigned int cpu;
	unsigned int new_freq;
	struct smoothass_info_s *this_smoothass;
	struct cpufreq_policy *policy;
	cpumask_t tmp_mask = work_cpumask;
	for_each_cpu(cpu, tmp_mask) {
		this_smoothass = &amp;per_cpu(smoothass_info, cpu);
		policy = this_smoothass-&gt;cur_policy;

		if (this_smoothass-&gt;force_ramp_up) {
			this_smoothass-&gt;force_ramp_up = 0;

			if (nr_running() == 1) {
				cpumask_clear_cpu(cpu, &amp;work_cpumask);
				return;
			}

			if (policy-&gt;cur == policy-&gt;max)
				return;

			if (ramp_up_step)
				new_freq = policy-&gt;cur + ramp_up_step;
			else
				new_freq = policy-&gt;max;

			if (suspended &amp;&amp; sleep_max_freq) {
				if (new_freq &gt; sleep_max_freq)
					new_freq = sleep_max_freq;
			} else {
				if (new_freq &lt; up_min_freq)
					new_freq = up_min_freq;
			}

		} else {
			new_freq = cpufreq_smoothass_calc_freq(cpu,policy);

			// in suspend limit to sleep_max_freq and
			// jump straight to sleep_max_freq to avoid wakeup problems
			if (suspended &amp;&amp; sleep_max_freq &amp;&amp;
			    (new_freq &gt; sleep_max_freq || new_freq &gt; policy-&gt;cur))
				new_freq = sleep_max_freq;
		}

		if (new_freq &gt; policy-&gt;max)
			new_freq = policy-&gt;max;
		
		if (new_freq &lt; policy-&gt;min)
			new_freq = policy-&gt;min;
		
		__cpufreq_driver_target(policy, new_freq,
					CPUFREQ_RELATION_L);

		freq_change_time_in_idle = get_cpu_idle_time_us(cpu,
							&amp;freq_change_time);

		cpumask_clear_cpu(cpu, &amp;work_cpumask);
	}


}

static ssize_t show_down_rate_us(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%lu\n", down_rate_us);
}

static ssize_t store_down_rate_us(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &amp;input);
	if (res &gt;= 0 &amp;&amp; input &gt;= 1000 &amp;&amp; input &lt;= 100000000)
	  down_rate_us = input;
	return res;
}

static struct freq_attr down_rate_us_attr = __ATTR(down_rate_us, 0644,
		show_down_rate_us, store_down_rate_us);

static ssize_t show_up_min_freq(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", up_min_freq);
}

static ssize_t store_up_min_freq(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &amp;input);
	if (res &gt;= 0 &amp;&amp; input &gt;= 0)
	  up_min_freq = input;
	return res;
}

static struct freq_attr up_min_freq_attr = __ATTR(up_min_freq, 0644,
		show_up_min_freq, store_up_min_freq);

static ssize_t show_sleep_max_freq(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", sleep_max_freq);
}

static ssize_t store_sleep_max_freq(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &amp;input);
	if (res &gt;= 0 &amp;&amp; input &gt;= 0)
	  sleep_max_freq = input;
	return res;
}

static struct freq_attr sleep_max_freq_attr = __ATTR(sleep_max_freq, 0644,
		show_sleep_max_freq, store_sleep_max_freq);

static ssize_t show_sample_rate_jiffies(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", sample_rate_jiffies);
}

static ssize_t store_sample_rate_jiffies(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &amp;input);
	if (res &gt;= 0 &amp;&amp; input &gt; 0 &amp;&amp; input &lt;= 1000)
	  sample_rate_jiffies = input;
	return res;
}

static struct freq_attr sample_rate_jiffies_attr = __ATTR(sample_rate_jiffies, 0644,
		show_sample_rate_jiffies, store_sample_rate_jiffies);

static ssize_t show_ramp_up_step(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", ramp_up_step);
}

static ssize_t store_ramp_up_step(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &amp;input);
	if (res &gt;= 0)
	  ramp_up_step = input;
	return res;
}

static struct freq_attr ramp_up_step_attr = __ATTR(ramp_up_step, 0644,
		show_ramp_up_step, store_ramp_up_step);

static ssize_t show_max_ramp_down(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%u\n", max_ramp_down);
}

static ssize_t store_max_ramp_down(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &amp;input);
	if (res &gt;= 0)
	  max_ramp_down = input;
	return res;
}

static struct freq_attr max_ramp_down_attr = __ATTR(max_ramp_down, 0644,
		show_max_ramp_down, store_max_ramp_down);

static ssize_t show_max_cpu_load(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%lu\n", max_cpu_load);
}

static ssize_t store_max_cpu_load(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &amp;input);
	if (res &gt;= 0 &amp;&amp; input &gt; 0 &amp;&amp; input &lt;= 100)
	  max_cpu_load = input;
	return res;
}

static struct freq_attr max_cpu_load_attr = __ATTR(max_cpu_load, 0644,
		show_max_cpu_load, store_max_cpu_load);

static ssize_t show_min_cpu_load(struct cpufreq_policy *policy, char *buf)
{
	return sprintf(buf, "%lu\n", min_cpu_load);
}

static ssize_t store_min_cpu_load(struct cpufreq_policy *policy, const char *buf, size_t count)
{
        ssize_t res;
	unsigned long input;
	res = strict_strtoul(buf, 0, &amp;input);
	if (res &gt;= 0 &amp;&amp; input &gt; 0 &amp;&amp; input &lt; 100)
	  min_cpu_load = input;
	return res;
}

static struct freq_attr min_cpu_load_attr = __ATTR(min_cpu_load, 0644,
		show_min_cpu_load, store_min_cpu_load);

static struct attribute * smoothass_attributes[] = {
	&amp;down_rate_us_attr.attr,
	&amp;up_min_freq_attr.attr,
	&amp;sleep_max_freq_attr.attr,
	&amp;sample_rate_jiffies_attr.attr,
	&amp;ramp_up_step_attr.attr,
	&amp;max_ramp_down_attr.attr,
	&amp;max_cpu_load_attr.attr,
	&amp;min_cpu_load_attr.attr,
	NULL,
};

static struct attribute_group smoothass_attr_group = {
	.attrs = smoothass_attributes,
	.name = "smoothass",
};

static int cpufreq_governor_smoothass(struct cpufreq_policy *new_policy,
		unsigned int event)
{
	unsigned int cpu = new_policy-&gt;cpu;
	int rc;
	struct smoothass_info_s *this_smoothass = &amp;per_cpu(smoothass_info, cpu);
	
	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!new_policy-&gt;cur))
			return -EINVAL;

		if (this_smoothass-&gt;enable) /* Already enabled */
			break;

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (atomic_inc_return(&amp;active_count) &gt; 1)
			return 0;

		rc = sysfs_create_group(&amp;new_policy-&gt;kobj, &amp;smoothass_attr_group);
		if (rc)
			return rc;
		pm_idle_old = pm_idle;
		pm_idle = cpufreq_idle;

		this_smoothass-&gt;cur_policy = new_policy;
		this_smoothass-&gt;cur_policy-&gt;max = 1401600;
		this_smoothass-&gt;cur_policy-&gt;min = 122880;
		this_smoothass-&gt;cur_policy-&gt;cur = 1401600;
		this_smoothass-&gt;enable = 1;

		// notice no break here!

	case CPUFREQ_GOV_LIMITS:
		if (this_smoothass-&gt;cur_policy-&gt;cur != new_policy-&gt;max)
			__cpufreq_driver_target(new_policy, new_policy-&gt;max, CPUFREQ_RELATION_H);

		break;

	case CPUFREQ_GOV_STOP:
		this_smoothass-&gt;enable = 0;

		if (atomic_dec_return(&amp;active_count) &gt; 1)
			return 0;
		sysfs_remove_group(&amp;new_policy-&gt;kobj,
				&amp;smoothass_attr_group);

		pm_idle = pm_idle_old;
		del_timer(&amp;this_smoothass-&gt;timer);
		break;
	}

	return 0;
}

static void smoothass_suspend(int cpu, int suspend)
{
	struct smoothass_info_s *this_smoothass = &amp;per_cpu(smoothass_info, smp_processor_id());
	struct cpufreq_policy *policy = this_smoothass-&gt;cur_policy;
	unsigned int new_freq;

	if (!this_smoothass-&gt;enable || sleep_max_freq==0) // disable behavior for sleep_max_freq==0
		return;

	if (suspend) {
	    if (policy-&gt;cur &gt; sleep_max_freq) {
			new_freq = sleep_max_freq;
			if (new_freq &gt; policy-&gt;max)
				new_freq = policy-&gt;max;
			if (new_freq &lt; policy-&gt;min)
				new_freq = policy-&gt;min;
			__cpufreq_driver_target(policy, new_freq,
						CPUFREQ_RELATION_H);
		}
	} else { // resume at max speed:
		__cpufreq_driver_target(policy, policy-&gt;max,
					CPUFREQ_RELATION_H);
	}

}

static void smoothass_early_suspend(struct early_suspend *handler) {
	int i;
	suspended = 1;
	for_each_online_cpu(i)
		smoothass_suspend(i,1);
}

static void smoothass_late_resume(struct early_suspend *handler) {
	int i;
	suspended = 0;
	for_each_online_cpu(i)
		smoothass_suspend(i,0);
}

static struct early_suspend smoothass_power_suspend = {
	.suspend = smoothass_early_suspend,
	.resume = smoothass_late_resume,
};

static int __init cpufreq_smoothass_init(void)
{	
	unsigned int i;
	struct smoothass_info_s *this_smoothass;
	down_rate_us = DEFAULT_DOWN_RATE_US;
	up_min_freq = DEFAULT_UP_MIN_FREQ;
	sleep_max_freq = DEFAULT_SLEEP_MAX_FREQ;
	sample_rate_jiffies = DEFAULT_SAMPLE_RATE_JIFFIES;
	ramp_up_step = DEFAULT_RAMP_UP_STEP;
	max_ramp_down = DEFAULT_MAX_RAMP_DOWN;
	max_cpu_load = DEFAULT_MAX_CPU_LOAD;
	min_cpu_load = DEFAULT_MIN_CPU_LOAD;

	suspended = 0;

	/* Initalize per-cpu data: */
	for_each_possible_cpu(i) {
		this_smoothass = &amp;per_cpu(smoothass_info, i);
		this_smoothass-&gt;enable = 0;
		this_smoothass-&gt;force_ramp_up = 0;
		this_smoothass-&gt;time_in_idle = 0;
		this_smoothass-&gt;idle_exit_time = 0;
		// intialize timer:
		init_timer_deferrable(&amp;this_smoothass-&gt;timer);
		this_smoothass-&gt;timer.function = cpufreq_smoothass_timer;
		this_smoothass-&gt;timer.data = i;
	}

	/* Scale up is high priority */
	up_wq = create_rt_workqueue("ksmoothass_up");
	down_wq = create_workqueue("ksmoothass_down");

	INIT_WORK(&amp;freq_scale_work, cpufreq_smoothass_freq_change_time_work);

	register_early_suspend(&amp;smoothass_power_suspend);

	return cpufreq_register_governor(&amp;cpufreq_gov_smoothass);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_SMOOTHASS
pure_initcall(cpufreq_smoothass_init);
#else
module_init(cpufreq_smoothass_init);
#endif

static void __exit cpufreq_smoothass_exit(void)
{
	cpufreq_unregister_governor(&amp;cpufreq_gov_smoothass);
	destroy_workqueue(up_wq);
	destroy_workqueue(down_wq);
}

module_exit(cpufreq_smoothass_exit);

MODULE_AUTHOR ("Erasmux, modified by LeeDrOiD");
MODULE_DESCRIPTION ("'cpufreq_smoothass' - A smart cpufreq governor");
MODULE_LICENSE ("GPL");