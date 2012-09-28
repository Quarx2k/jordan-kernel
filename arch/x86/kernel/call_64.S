/*
 * linux/arch/x86/kernel/call_64.S -- special 64-bits calling conventions
 *
 * Copyright (C) 2008 Mathieu Desnoyers
 */

#include <linux/linkage.h>

/*
 * Called by call_rax_rsi().
 *
 * Move rax to rdi and proceed to the standard call.
 */
.macro TRAMPOLINE_RAX_RSI symbol
ENTRY(asm_\symbol)
	movq	%rax, %rdi
	jmp	_\symbol
END(asm_\symbol)
.endm

/*
 * Called by call_rbx_rsi().
 *
 * Move rbx to rdi and proceed to the standard call.
 */
.macro TRAMPOLINE_RBX_RSI symbol
ENTRY(asm_\symbol)
	movq	%rbx, %rdi
	jmp	_\symbol
END(asm_\symbol)
.endm

TRAMPOLINE_RAX_RSI psread_lock_slow_irq
TRAMPOLINE_RAX_RSI psread_trylock_slow_irq
TRAMPOLINE_RAX_RSI psread_lock_slow_bh
TRAMPOLINE_RAX_RSI psread_trylock_slow_bh
TRAMPOLINE_RAX_RSI psread_lock_slow_inatomic
TRAMPOLINE_RAX_RSI psread_trylock_slow_inatomic
TRAMPOLINE_RAX_RSI psread_lock_slow
TRAMPOLINE_RAX_RSI psread_lock_interruptible_slow
TRAMPOLINE_RAX_RSI psread_trylock_slow

TRAMPOLINE_RAX_RSI pswrite_lock_slow
TRAMPOLINE_RAX_RSI pswrite_lock_interruptible_slow
TRAMPOLINE_RAX_RSI pswrite_trylock_slow
TRAMPOLINE_RAX_RSI pswrite_unlock_slow
TRAMPOLINE_RBX_RSI psrwlock_wakeup
