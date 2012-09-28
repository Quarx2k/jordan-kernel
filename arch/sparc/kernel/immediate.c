/*
 * Immediate values. Sparc64 architecture optimizations.
 *
 * (C) Copyright 2009 David Miller <davem@davemloft.net>
 * (C) Copyright 2009 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/module.h>
#include <linux/immediate.h>
#include <linux/string.h>
#include <linux/kprobes.h>

#include <asm/system.h>

int arch_imv_update(const struct __imv *imv, int early)
{
	unsigned long imv_vaddr = imv->imv;
	unsigned long var_vaddr = imv->var;
	u32 insn, *ip = (u32 *) imv_vaddr;

	insn = *ip;

#ifdef CONFIG_KPROBES
	switch (imv->size) {
	case 1:
		break;
	default:
		return -EINVAL;
	}

	if (unlikely(!early &&
		     (insn == BREAKPOINT_INSTRUCTION ||
		      insn == BREAKPOINT_INSTRUCTION_2))) {
		printk(KERN_WARNING "Immediate value in conflict with kprobe. "
				    "Variable at %p, "
				    "instruction at %p, size %u\n",
				    ip, (void *)var_vaddr, imv->size);
		return -EBUSY;
	}
#endif

	switch (imv->size) {
	case 1:
		if ((insn & 0x1fff) == *(uint8_t *)var_vaddr)
			return 0;
		insn &= ~0x00001fff;
		insn |= (u32) (*(uint8_t *)var_vaddr);
		break;
	default:
		return -EINVAL;
	}
	*ip = insn;
	flushi(ip);
	return 0;
}
