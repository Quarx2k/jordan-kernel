#ifndef _LINUX_TRACE_CLOCK_H
#define _LINUX_TRACE_CLOCK_H

/*
 * Trace clock
 *
 * Chooses between an architecture specific clock or an atomic logical clock.
 *
 * Copyright (C) 2007,2008 Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 */

#ifdef CONFIG_HAVE_TRACE_CLOCK
#include <asm/trace-clock.h>
#else
#include <asm-generic/trace-clock.h>
#endif /* CONFIG_HAVE_TRACE_CLOCK */
#endif /* _LINUX_TRACE_CLOCK_H */
