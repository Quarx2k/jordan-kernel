/*
 * (C) Copyright	2008 -
 * 		Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * LTTng Ftrace integration module.
 *
 * Per-cpu "tap" (ftrace_cpu_start/ftrace_cpu_stop) will enable the tap for a
 * given CPU on which the probes has been called.
 *
 * System-wide "tap" (ftrace_system_start/ftrace_system_stop) will enable
 * tracing on every CPU as soon as a single CPU hits the start probe. It is left
 * active until the same CPU hits the "stop" probe. Uses per-cpu boolean and
 * global reference counting to make sure we detect when _at least_ one CPU is
 * interested in opening the tap.
 *
 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/module.h>
#include <linux/ftrace.h>
#include <linux/ltt-tracer.h>
#include <linux/marker.h>
#include <asm/atomic.h>

/* per-cpu function tracing activation */
static DEFINE_PER_CPU(int, tracing_cpu);

/* system-wide function tracing activation */
static DEFINE_PER_CPU(int, system_tracing_cpu);
static atomic_t system_trace_refcount __read_mostly;


static notrace
void ltt_tracer_call(unsigned long ip, unsigned long parent_ip)
{
	int cpu = raw_smp_processor_id();
	if (likely(!per_cpu(tracing_cpu, cpu)
			&& !atomic_read(&system_trace_refcount)))
		return;
	trace_mark(function_trace, entry, "ip 0x%lX parent_ip 0x%lX",
		   ip, parent_ip);
}

static notrace
void ltt_tap_marker(const struct marker *mdata, void *probe_data,
		    void *call_data, const char *fmt, va_list *args)
{
	int cpu = raw_smp_processor_id();
	if (likely(!per_cpu(tracing_cpu, cpu)
	    && !atomic_read(&system_trace_refcount)))
		return;
	ltt_vtrace(mdata, probe_data, call_data, fmt, args);
}

struct ltt_available_probe ltt_tap_marker_probe = {
	.name = "ltt_tap_marker",
	.format = NULL,
	.probe_func = ltt_tap_marker,
};

static struct ftrace_ops trace_ops __read_mostly =
{
	.func = ltt_tracer_call,
};

static notrace
void ftrace_cpu_start(const struct marker *mdata, void *probe_data,
		      void *call_data, const char *fmt, va_list *args)
{
	int cpu = raw_smp_processor_id();
	per_cpu(tracing_cpu, cpu) = 1;
}

struct ltt_available_probe ftrace_cpu_start_probe = {
	.name = "ftrace_cpu_start",
	.format = NULL,
	.probe_func = ftrace_cpu_start,
};

static notrace
void ftrace_cpu_stop(const struct marker *mdata, void *probe_data,
		     void *call_data, const char *fmt, va_list *args)
{
	int cpu = raw_smp_processor_id();
	per_cpu(tracing_cpu, cpu) = 0;
}

struct ltt_available_probe ftrace_cpu_stop_probe = {
	.name = "ftrace_cpu_stop",
	.format = NULL,
	.probe_func = ftrace_cpu_stop,
};

static notrace
void ftrace_system_start(const struct marker *mdata, void *probe_data,
			 void *call_data, const char *fmt, va_list *args)
{
	int cpu = raw_smp_processor_id();
	int value = xchg(&per_cpu(system_tracing_cpu, cpu), 1);
	if (!value)
		atomic_inc(&system_trace_refcount);
}

struct ltt_available_probe ftrace_system_start_probe = {
	.name = "ftrace_system_start",
	.format = NULL,
	.probe_func = ftrace_system_start,
};

static notrace
void ftrace_system_stop(const struct marker *mdata, void *probe_data,
			void *call_data, const char *fmt, va_list *args)
{
	int cpu = raw_smp_processor_id();
	int value = xchg(&per_cpu(system_tracing_cpu, cpu), 0);
	if (value)
		atomic_dec(&system_trace_refcount);
}

struct ltt_available_probe ftrace_system_stop_probe = {
	.name = "ftrace_system_stop",
	.format = NULL,
	.probe_func = ftrace_system_stop,
};

static int __init ltt_ftrace_init(void)
{
	int ret;

	printk(KERN_INFO "LTT : ltt-ftrace init\n");
	register_ftrace_function(&trace_ops);
	ret = ltt_probe_register(&ftrace_cpu_start_probe);
	BUG_ON(ret);
	ret = ltt_probe_register(&ftrace_cpu_stop_probe);
	BUG_ON(ret);
	ret = ltt_probe_register(&ftrace_system_start_probe);
	BUG_ON(ret);
	ret = ltt_probe_register(&ftrace_system_stop_probe);
	BUG_ON(ret);
	ret = ltt_probe_register(&ltt_tap_marker_probe);
	BUG_ON(ret);

	/*
	 * Keep a refcount on ourselves, because ftrace forbids freeing
	 * trace_ops.
	 */
	/* __module_get(THIS_MODULE); */

	return 0;
}
module_init(ltt_ftrace_init);

#if 0
/* create file operation to activate/deactivate these probes.
 */
static void __exit ltt_ftrace_exit(void)
{
	int ret;

	printk(KERN_INFO "LTT : ltt-ftrace exit\n");
	ret = ltt_probe_unregister(&ltt_tap_marker_probe);
	BUG_ON(ret);
	ret = ltt_probe_unregister(&ftrace_system_stop_probe);
	BUG_ON(ret);
	ret = ltt_probe_unregister(&ftrace_system_start_probe);
	BUG_ON(ret);
	ret = ltt_probe_unregister(&ftrace_cpu_stop_probe);
	BUG_ON(ret);
	ret = ltt_probe_unregister(&ftrace_cpu_start_probe);
	BUG_ON(ret);
	unregister_ftrace_function(&trace_ops);
}
module_exit(ltt_ftrace_exit);
#endif //0

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("Linux Trace Toolkit Function Tracer Support");
