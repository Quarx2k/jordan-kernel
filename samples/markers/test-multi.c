/* test-multi.c
 *
 * Connects multiple callbacks.
 *
 * (C) Copyright 2007 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 * This file is released under the GPLv2.
 * See the file COPYING for more details.
 */

#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/marker.h>
#include <asm/atomic.h>

struct probe_data {
	const char *name;
	const char *format;
	marker_probe_func *probe_func;
};

atomic_t eventb_count = ATOMIC_INIT(0);

void probe_subsystem_eventa(void *probe_data, void *call_data,
	const char *format, va_list *args)
{
	/* Increment counter */
	atomic_inc(&eventb_count);
}

void probe_subsystem_eventb(void *probe_data, void *call_data,
	const char *format, va_list *args)
{
	/* Increment counter */
	atomic_inc(&eventb_count);
}

void probe_subsystem_eventc(void *probe_data, void *call_data,
	const char *format, va_list *args)
{
	/* Increment counter */
	atomic_inc(&eventb_count);
}

void probe_subsystem_eventd(void *probe_data, void *call_data,
	const char *format, va_list *args)
{
	/* Increment counter */
	atomic_inc(&eventb_count);
}

static struct probe_data probe_array[] =
{
	{	.name = "test_multi",
		.format = MARK_NOARGS,
		.probe_func = (marker_probe_func*)0xa },
	{	.name = "test_multi",
		.format = MARK_NOARGS,
		.probe_func = (marker_probe_func*)0xb },
	{	.name = "test_multi",
		.format = MARK_NOARGS,
		.probe_func = (marker_probe_func*)0xc },
	{	.name = "test_multi",
		.format = MARK_NOARGS,
		.probe_func = (marker_probe_func*)0xd },
	{	.name = "test_multi",
		.format = MARK_NOARGS,
		.probe_func = (marker_probe_func*)0x10 },
	{	.name = "test_multi",
		.format = MARK_NOARGS,
		.probe_func = (marker_probe_func*)0x20 },
	{	.name = "test_multi",
		.format = MARK_NOARGS,
		.probe_func = (marker_probe_func*)0x30 },
};

static int __init probe_init(void)
{
	int result;
	int i;

	for (i = 0; i < ARRAY_SIZE(probe_array); i++) {
		result = marker_probe_register("samples", probe_array[i].name,
				probe_array[i].format,
				probe_array[i].probe_func, (void*)(long)i);
		if (result)
			printk(KERN_INFO "Unable to register probe %s\n",
				probe_array[i].name);
	}
	return 0;
}

static void __exit probe_fini(void)
{
	int result;
	int i;

	for (i = 0; i < ARRAY_SIZE(probe_array); i++) {
		result = marker_probe_unregister("samples", probe_array[i].name,
			probe_array[i].probe_func, (void*)(long)i);
		if (result)
			printk(KERN_INFO "Unable to unregister probe %s\n",
				probe_array[i].name);
	}
	printk(KERN_INFO "Number of event b : %u\n",
		atomic_read(&eventb_count));
	marker_synchronize_unregister();
}

module_init(probe_init);
module_exit(probe_fini);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("SUBSYSTEM Probe");
