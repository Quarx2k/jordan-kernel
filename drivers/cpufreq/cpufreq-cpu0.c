/*
 * Copyright (C) 2012 Freescale Semiconductor, Inc.
 *
 * The OPP code in function cpu0_set_target() is reused from
 * drivers/cpufreq/omap-cpufreq.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt)	KBUILD_MODNAME ": " fmt

#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/opp.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/suspend.h>

static unsigned int transition_latency;
static unsigned int voltage_tolerance; /* in percentage */

static struct device *cpu_dev;
static struct clk *cpu_clk;
static struct regulator *cpu_reg;
static struct cpufreq_frequency_table *freq_table;

static DEFINE_MUTEX(cpu0_cpufreq_lock);
static bool is_suspended;

static int cpu0_verify_speed(struct cpufreq_policy *policy)
{
	return cpufreq_frequency_table_verify(policy, freq_table);
}

static unsigned int cpu0_get_speed(unsigned int cpu)
{
	return clk_get_rate(cpu_clk) / 1000;
}

static int __cpu0_set_target(struct cpufreq_policy *policy,
			   unsigned int target_freq, unsigned int relation)
{
	struct cpufreq_freqs freqs;
	struct opp *opp;
	unsigned long volt = 0, volt_old = 0, tol = 0;
	long freq_Hz, freq_exact;
	unsigned int index;
	int ret;

	ret = cpufreq_frequency_table_target(policy, freq_table, target_freq,
					     relation, &index);
	if (ret) {
		pr_err("failed to match target freqency %d: %d\n",
		       target_freq, ret);
		return ret;
	}

	freq_Hz = clk_round_rate(cpu_clk, freq_table[index].frequency * 1000);
	if (freq_Hz < 0)
		freq_Hz = freq_table[index].frequency * 1000;
	freq_exact = freq_Hz;
	freqs.new = freq_Hz / 1000;
	freqs.old = clk_get_rate(cpu_clk) / 1000;

	if (freqs.old == freqs.new)
		return 0;

	cpufreq_notify_transition(policy, &freqs, CPUFREQ_PRECHANGE);

	if (cpu_reg) {
		rcu_read_lock();
		opp = opp_find_freq_ceil(cpu_dev, &freq_Hz);
		if (IS_ERR(opp)) {
			rcu_read_unlock();
			pr_err("failed to find OPP for %ld\n", freq_Hz);
			freqs.new = freqs.old;
			ret = PTR_ERR(opp);
			goto post_notify;
		}
		volt = opp_get_voltage(opp);
		rcu_read_unlock();
		tol = volt * voltage_tolerance / 100;
		volt_old = regulator_get_voltage(cpu_reg);
	}

	pr_debug("%u MHz, %ld mV --> %u MHz, %ld mV\n",
		 freqs.old / 1000, volt_old ? volt_old / 1000 : -1,
		 freqs.new / 1000, volt ? volt / 1000 : -1);

	/* scaling up?  scale voltage before frequency */
	if (cpu_reg && freqs.new > freqs.old) {
		ret = regulator_set_voltage_tol(cpu_reg, volt, tol);
		if (ret) {
			pr_err("failed to scale voltage up: %d\n", ret);
			freqs.new = freqs.old;
			goto post_notify;
		}
	}

	ret = clk_set_rate(cpu_clk, freq_exact);
	if (ret) {
		pr_err("failed to set clock rate: %d\n", ret);
		if (cpu_reg)
			regulator_set_voltage_tol(cpu_reg, volt_old, tol);
		freqs.new = freqs.old;
		goto post_notify;
	}

	/* scaling down?  scale voltage after frequency */
	if (cpu_reg && freqs.new < freqs.old) {
		ret = regulator_set_voltage_tol(cpu_reg, volt, tol);
		if (ret) {
			pr_err("failed to scale voltage down: %d\n", ret);
			clk_set_rate(cpu_clk, freqs.old * 1000);
			freqs.new = freqs.old;
		}
	}

post_notify:
	cpufreq_notify_transition(policy, &freqs, CPUFREQ_POSTCHANGE);

	return ret;
}

static int cpu0_set_target(struct cpufreq_policy *policy,
			   unsigned int target_freq, unsigned int relation)
{
	int ret;

	mutex_lock(&cpu0_cpufreq_lock);
	if (is_suspended) {
		ret = -EBUSY;
		goto out;
	}

	ret = __cpu0_set_target(policy, target_freq, relation);
out:
	mutex_unlock(&cpu0_cpufreq_lock);
	return  ret;

}

static int cpu0_cpufreq_pm_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	mutex_lock(&cpu0_cpufreq_lock);
	if (event == PM_SUSPEND_PREPARE) {
		struct cpufreq_policy *policy = cpufreq_cpu_get(0);
		is_suspended = true;
		pr_debug("cpu0 cpufreq suspend: setting frequency to %d kHz\n",
			policy->max);
		__cpu0_set_target(policy, policy->max, CPUFREQ_RELATION_L);
		cpufreq_cpu_put(policy);
	} else if (event == PM_POST_SUSPEND) {
		is_suspended = false;
	}
	mutex_unlock(&cpu0_cpufreq_lock);

	return NOTIFY_OK;
}

static struct notifier_block cpu0_cpufreq_pm_notifier = {
	.notifier_call = cpu0_cpufreq_pm_notify,
};

static int cpu0_cpufreq_reboot_notify(struct notifier_block *nb,
	unsigned long event, void *dummy)
{
	struct cpufreq_policy *policy;

	switch(event) {
	case SYS_DOWN:
	case SYS_HALT:
	case SYS_POWER_OFF:
		mutex_lock(&cpu0_cpufreq_lock);
		policy = cpufreq_cpu_get(0);
		is_suspended = true;
		pr_info("cpu0 cpufreq shutdown: setting frequency to %d kHz\n",
				policy->max);
		__cpu0_set_target(policy, policy->max, CPUFREQ_RELATION_L);
		cpufreq_cpu_put(policy);
		mutex_unlock(&cpu0_cpufreq_lock);
		break;
	default:
		pr_warn("cpu0 cpufreq shutdown: INVALID event type %lu\n",
			event);
		break;
	}

	return NOTIFY_OK;
}

static struct notifier_block cpu0_cpufreq_reboot_notifier = {
	.notifier_call = cpu0_cpufreq_reboot_notify,
};

static int cpu0_cpufreq_init(struct cpufreq_policy *policy)
{
	int ret;

	ret = cpufreq_frequency_table_cpuinfo(policy, freq_table);
	if (ret) {
		pr_err("invalid frequency table: %d\n", ret);
		return ret;
	}

	policy->cpuinfo.transition_latency = transition_latency;
	policy->cur = clk_get_rate(cpu_clk) / 1000;

	/*
	 * The driver only supports the SMP configuartion where all processors
	 * share the clock and voltage and clock.  Use cpufreq affected_cpus
	 * interface to have all CPUs scaled together.
	 */
	cpumask_setall(policy->cpus);

	cpufreq_frequency_table_get_attr(freq_table, policy->cpu);

	if (policy->cpu == 0)
		register_pm_notifier(&cpu0_cpufreq_pm_notifier);

	register_reboot_notifier(&cpu0_cpufreq_reboot_notifier);

	return 0;
}

static int cpu0_cpufreq_exit(struct cpufreq_policy *policy)
{
	if (policy->cpu == 0)
		unregister_pm_notifier(&cpu0_cpufreq_pm_notifier);

	unregister_reboot_notifier(&cpu0_cpufreq_reboot_notifier);

	cpufreq_frequency_table_put_attr(policy->cpu);

	return 0;
}

static struct freq_attr *cpu0_cpufreq_attr[] = {
	&cpufreq_freq_attr_scaling_available_freqs,
	NULL,
};

static struct cpufreq_driver cpu0_cpufreq_driver = {
	.flags = CPUFREQ_STICKY,
	.verify = cpu0_verify_speed,
	.target = cpu0_set_target,
	.get = cpu0_get_speed,
	.init = cpu0_cpufreq_init,
	.exit = cpu0_cpufreq_exit,
	.name = "generic_cpu0",
	.attr = cpu0_cpufreq_attr,
};

static int cpu0_cpufreq_probe(struct platform_device *pdev)
{
	struct device_node *np, *parent;
	int ret;

	parent = of_find_node_by_path("/cpus");
	if (!parent) {
		pr_err("failed to find OF /cpus\n");
		return -ENOENT;
	}

	for_each_child_of_node(parent, np) {
		if (of_get_property(np, "operating-points", NULL))
			break;
	}

	if (!np) {
		pr_err("failed to find cpu0 node\n");
		ret = -ENOENT;
		goto out_put_parent;
	}

	cpu_dev = &pdev->dev;
	cpu_dev->of_node = np;

	cpu_reg = devm_regulator_get(cpu_dev, "cpu0");
	if (IS_ERR(cpu_reg)) {
		/*
		 * If cpu0 regulator supply node is present, but regulator is
		 * not yet registered, we should try defering probe.
		 */
		if (PTR_ERR(cpu_reg) == -EPROBE_DEFER) {
			dev_err(cpu_dev, "cpu0 regulator not ready, retry\n");
			ret = -EPROBE_DEFER;
			goto out_put_node;
		}
		pr_warn("failed to get cpu0 regulator: %ld\n",
			PTR_ERR(cpu_reg));
		cpu_reg = NULL;
	}

	cpu_clk = devm_clk_get(cpu_dev, NULL);
	if (IS_ERR(cpu_clk)) {
		ret = PTR_ERR(cpu_clk);
		pr_err("failed to get cpu0 clock: %d\n", ret);
		goto out_put_node;
	}

	ret = of_init_opp_table(cpu_dev);
	if (ret) {
		pr_err("failed to init OPP table: %d\n", ret);
		goto out_put_node;
	}

	ret = opp_init_cpufreq_table(cpu_dev, &freq_table);
	if (ret) {
		pr_err("failed to init cpufreq table: %d\n", ret);
		goto out_put_node;
	}

	of_property_read_u32(np, "voltage-tolerance", &voltage_tolerance);

	if (of_property_read_u32(np, "clock-latency", &transition_latency))
		transition_latency = CPUFREQ_ETERNAL;

	if (cpu_reg) {
		struct opp *opp;
		unsigned long min_uV, max_uV;
		int i;

		/*
		 * OPP is maintained in order of increasing frequency, and
		 * freq_table initialised from OPP is therefore sorted in the
		 * same order.
		 */
		for (i = 0; freq_table[i].frequency != CPUFREQ_TABLE_END; i++)
			;
		rcu_read_lock();
		opp = opp_find_freq_exact(cpu_dev,
				freq_table[0].frequency * 1000, true);
		min_uV = opp_get_voltage(opp);
		opp = opp_find_freq_exact(cpu_dev,
				freq_table[i-1].frequency * 1000, true);
		max_uV = opp_get_voltage(opp);
		rcu_read_unlock();
		ret = regulator_set_voltage_time(cpu_reg, min_uV, max_uV);
		if (ret > 0)
			transition_latency += ret * 1000;
	}

	ret = cpufreq_register_driver(&cpu0_cpufreq_driver);
	if (ret) {
		pr_err("failed register driver: %d\n", ret);
		goto out_free_table;
	}

	of_node_put(np);
	of_node_put(parent);
	return 0;

out_free_table:
	opp_free_cpufreq_table(cpu_dev, &freq_table);
out_put_node:
	of_node_put(np);
out_put_parent:
	of_node_put(parent);
	return ret;
}

static int cpu0_cpufreq_remove(struct platform_device *pdev)
{
	cpufreq_unregister_driver(&cpu0_cpufreq_driver);
	opp_free_cpufreq_table(cpu_dev, &freq_table);

	return 0;
}

static struct platform_driver cpu0_cpufreq_platdrv = {
	.driver = {
		.name	= "cpufreq-cpu0",
		.owner	= THIS_MODULE,
	},
	.probe		= cpu0_cpufreq_probe,
	.remove		= cpu0_cpufreq_remove,
};
module_platform_driver(cpu0_cpufreq_platdrv);

MODULE_AUTHOR("Shawn Guo <shawn.guo@linaro.org>");
MODULE_DESCRIPTION("Generic CPU0 cpufreq driver");
MODULE_LICENSE("GPL");
