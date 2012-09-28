/*
 * Dynamic function tracing support.
 *
 * Copyright (C) 2008 Abhishek Sagar <sagar.abhishek@gmail.com>
 *
 * For licencing details, see COPYING.
 *
 * Defines low-level handling of mcount calls when the kernel
 * is compiled with the -pg flag. When using dynamic ftrace, the
 * mcount call-sites get patched lazily with NOP till they are
 * enabled. All code mutation routines here take effect atomically.
 */

#include <linux/ftrace.h>
#include <linux/uaccess.h>

#include <asm/cacheflush.h>
#include <asm/ftrace.h>

#define PC_OFFSET      8
#define BL_OPCODE      0xeb000000
#define BL_OFFSET_MASK 0x00ffffff

#define NOP 0xe1a00000 /* mov r0, r0 */
#define GNU_NOP 0xe8bd4000 /* pop {lr} */

#define GNU_MCOUNT_ADDR ((unsigned long) __gnu_mcount_nc)
#define GNU_FTRACE_ADDR ((unsigned long) ftrace_caller_gnu)

static unsigned long ftrace_nop_replace(struct dyn_ftrace *rec)
{
	return rec->arch.gnu_mcount ? GNU_NOP : NOP;
}

static unsigned long ftrace_caller_addr(struct dyn_ftrace *rec)
{
	return rec->arch.gnu_mcount ? GNU_FTRACE_ADDR : FTRACE_ADDR;
}

/* construct a branch (BL) instruction to addr */
static unsigned long ftrace_call_replace(unsigned long pc, unsigned long addr)
{
	unsigned long bl_insn;
	long offset;

	offset = (long)addr - (long)(pc + PC_OFFSET);
	if (unlikely(offset < -33554432 || offset > 33554428)) {
		/* Can't generate branches that far (from ARM ARM). Ftrace
		 * doesn't generate branches outside of kernel text.
		 */
		WARN_ON_ONCE(1);
		return 0;
	}
	offset = (offset >> 2) & BL_OFFSET_MASK;
	bl_insn = BL_OPCODE | offset;
	return bl_insn;
}

static int ftrace_modify_code(unsigned long pc, unsigned long old,
		       unsigned long new)
{
	unsigned long replaced;

	if (probe_kernel_read(&replaced, (void *)pc, MCOUNT_INSN_SIZE))
		return -EFAULT;

	if (replaced != old)
		return -EINVAL;

	if (probe_kernel_write((void *)pc, &new, MCOUNT_INSN_SIZE))
		return -EPERM;

	flush_icache_range(pc, pc + MCOUNT_INSN_SIZE);

	return 0;
}

int ftrace_update_ftrace_func(ftrace_func_t func)
{
	unsigned long pc, old;
	unsigned long new;

	pc = (unsigned long)&ftrace_call;
	memcpy(&old, &ftrace_call, MCOUNT_INSN_SIZE);
	new = ftrace_call_replace(pc, (unsigned long)func);

	return ftrace_modify_code(pc, old, new);
}

int ftrace_make_call(struct dyn_ftrace *rec, unsigned long addr)
{
	unsigned long new, old;
	unsigned long ip = rec->ip;

	old = ftrace_nop_replace(rec);
	new = ftrace_call_replace(ip, ftrace_caller_addr(rec));

	return ftrace_modify_code(rec->ip, old, new);
}

static int ftrace_detect_make_nop(struct module *mod,
		struct dyn_ftrace *rec, unsigned long addr)
{
	unsigned long ip = rec->ip;
	unsigned long call;
	int ret;

	call = ftrace_call_replace(ip, GNU_MCOUNT_ADDR);
	ret = ftrace_modify_code(ip, call, GNU_NOP);
	if (!ret)
		rec->arch.gnu_mcount = true;
	else if (ret == -EINVAL) {
		call = ftrace_call_replace(ip, addr);
		ret = ftrace_modify_code(ip, call, NOP);
	}

	return ret;
}

int ftrace_make_nop(struct module *mod,
		struct dyn_ftrace *rec, unsigned long addr)
{
	unsigned long ip = rec->ip;
	unsigned long old;
	unsigned long new;

	if (addr == MCOUNT_ADDR)
		return ftrace_detect_make_nop(mod, rec, addr);

	 old = ftrace_call_replace(ip, ftrace_caller_addr(rec));
	 new = ftrace_nop_replace(rec);

	 return ftrace_modify_code(ip, old, new);
}

int __init ftrace_dyn_arch_init(void *data)
{
	*(unsigned long *)data = 0;

	return 0;
}
