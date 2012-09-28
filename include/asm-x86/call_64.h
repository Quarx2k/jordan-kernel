#ifndef __ASM_X86_CALL_64_H
#define __ASM_X86_CALL_64_H

/*
 * asm-x86/call_64.h
 *
 * Use rax as first argument for the call. Useful when already returned by the
 * previous instruction, such as cmpxchg.
 * Leave rdi free to mov rax to rdi in the trampoline.
 * Return value in rax.
 *
 * Saving the registers in the original caller because we cannot restore them in
 * the trampoline. Save the same as "SAVE_ARGS".
 *
 * Copyright (C) 2008 Mathieu Desnoyers
 */

#define call_rax_rsi(symbol, rax, rsi)				\
	({							\
		unsigned long ret, modrsi;			\
		asm volatile("callq asm_" #symbol "\n\t"	\
			     : "=a" (ret), "=S" (modrsi)	\
			     : "a" (rax), "S" (rsi)		\
			     : "rdi", "rcx", "rdx",		\
			       "%r8", "%r9", "%r10", "%r11",	\
			       "cc", "memory");			\
		ret;						\
	})

#define call_rbx_rsi(symbol, rbx, rsi)				\
	({							\
		unsigned long ret, modrsi;			\
		asm volatile("callq asm_" #symbol "\n\t"	\
			     : "=a" (ret), "=S" (modrsi)	\
			     : "b" (rbx), "S" (rsi)		\
			     : "rdi", "rcx", "rdx",		\
			       "%r8", "%r9", "%r10", "%r11",	\
			       "cc", "memory");			\
		ret;						\
	})

#define psread_lock_slow_irq(v, rwlock)				\
	call_rax_rsi(psread_lock_slow_irq, v, rwlock)
#define psread_trylock_slow_irq(v, rwlock)			\
	call_rax_rsi(psread_trylock_slow_irq, v, rwlock)
#define psread_lock_slow_bh(v, rwlock)				\
	call_rax_rsi(psread_lock_slow_bh, v, rwlock)
#define psread_trylock_slow_bh(v, rwlock)			\
	call_rax_rsi(psread_trylock_slow_bh, v, rwlock)
#define psread_lock_slow_inatomic(v, rwlock)			\
	call_rax_rsi(psread_lock_slow_inatomic, v, rwlock)
#define psread_trylock_slow_inatomic(v, rwlock)			\
	call_rax_rsi(psread_trylock_slow_inatomic, v, rwlock)
#define psread_lock_slow(v, rwlock)				\
	call_rax_rsi(psread_lock_slow, v, rwlock)
#define psread_lock_interruptible_slow(v, rwlock)		\
	call_rax_rsi(psread_lock_interruptible_slow, v, rwlock)
#define psread_trylock_slow(v, rwlock)				\
	call_rax_rsi(psread_trylock_slow, v, rwlock)

#define pswrite_lock_slow(v, rwlock)				\
	call_rax_rsi(pswrite_lock_slow, v, rwlock)
#define pswrite_lock_interruptible_slow(v, rwlock)		\
	call_rax_rsi(pswrite_lock_interruptible_slow, v, rwlock)
#define pswrite_trylock_slow(v, rwlock)				\
	call_rax_rsi(pswrite_trylock_slow, v, rwlock)
#define pswrite_unlock_slow(v, rwlock)				\
	call_rax_rsi(pswrite_unlock_slow, v, rwlock)
#define psrwlock_wakeup(v, rwlock)				\
	call_rbx_rsi(psrwlock_wakeup, v, rwlock)

#endif
