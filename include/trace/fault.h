#ifndef _TRACE_FAULT_H
#define _TRACE_FAULT_H

#include <linux/tracepoint.h>

DECLARE_TRACE(page_fault_entry,
	TP_PROTO(struct pt_regs *regs, int trapnr,
			struct mm_struct *mm, struct vm_area_struct *vma,
			unsigned long address, int write_access),
		TP_ARGS(regs, trapnr, mm, vma, address, write_access));
DECLARE_TRACE(page_fault_exit,
	TP_PROTO(int res),
		TP_ARGS(res));
DECLARE_TRACE(page_fault_nosem_entry,
	TP_PROTO(struct pt_regs *regs, int trapnr, unsigned long address),
		TP_ARGS(regs, trapnr, address));
DECLARE_TRACE(page_fault_nosem_exit,
	TP_PROTO(void),
		TP_ARGS());
DECLARE_TRACE(page_fault_get_user_entry,
	TP_PROTO(struct mm_struct *mm, struct vm_area_struct *vma,
			unsigned long address, int write_access),
		TP_ARGS(mm, vma, address, write_access));
DECLARE_TRACE(page_fault_get_user_exit,
	TP_PROTO(int res),
		TP_ARGS(res));
#endif
