#ifndef _TRACE_KERNEL_H
#define _TRACE_KERNEL_H

#include <linux/tracepoint.h>
#include <linux/kexec.h>

struct kimage;

DECLARE_TRACE(kernel_printk,
	TP_PROTO(unsigned long retaddr),
		TP_ARGS(retaddr));
DECLARE_TRACE(kernel_vprintk,
	TP_PROTO(unsigned long retaddr, char *buf, int len),
		TP_ARGS(retaddr, buf, len));
DECLARE_TRACE(kernel_module_free,
	TP_PROTO(struct module *mod),
		TP_ARGS(mod));
DECLARE_TRACE(kernel_module_load,
	TP_PROTO(struct module *mod),
		TP_ARGS(mod));
DECLARE_TRACE(kernel_panic,
	TP_PROTO(const char *fmt, va_list args),
		TP_ARGS(fmt, args));
DECLARE_TRACE(kernel_kernel_kexec,
	TP_PROTO(struct kimage *image),
		TP_ARGS(image));
DECLARE_TRACE(kernel_crash_kexec,
	TP_PROTO(struct kimage *image, struct pt_regs *regs),
		TP_ARGS(image, regs));

#endif
