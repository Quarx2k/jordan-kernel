/*
 * linux/arch/x86/kernel/call_64.c -- special 64-bits calling conventions
 *
 * Export function symbols of special calling convention functions.
 *
 * Copyright (C) 2008 Mathieu Desnoyers
 */

#include <linux/module.h>
#include <asm/call_64.h>

void asm_psread_lock_slow_irq(void);
EXPORT_SYMBOL_GPL(asm_psread_lock_slow_irq);
void asm_psread_trylock_slow_irq(void);
EXPORT_SYMBOL_GPL(asm_psread_trylock_slow_irq);
void asm_psread_lock_slow_bh(void);
EXPORT_SYMBOL_GPL(asm_psread_lock_slow_bh);
void asm_psread_trylock_slow_bh(void);
EXPORT_SYMBOL_GPL(asm_psread_trylock_slow_bh);
void asm_psread_lock_slow_inatomic(void);
EXPORT_SYMBOL_GPL(asm_psread_lock_slow_inatomic);
void asm_psread_trylock_slow_inatomic(void);
EXPORT_SYMBOL_GPL(asm_psread_trylock_slow_inatomic);
void asm_psread_lock_slow(void);
EXPORT_SYMBOL_GPL(asm_psread_lock_slow);
void asm_psread_lock_interruptible_slow(void);
EXPORT_SYMBOL_GPL(asm_psread_lock_interruptible_slow);
void asm_psread_trylock_slow(void);
EXPORT_SYMBOL_GPL(asm_psread_trylock_slow);

void asm_pswrite_lock_slow(void);
EXPORT_SYMBOL_GPL(asm_pswrite_lock_slow);
void asm_pswrite_lock_interruptible_slow(void);
EXPORT_SYMBOL_GPL(asm_pswrite_lock_interruptible_slow);
void asm_pswrite_trylock_slow(void);
EXPORT_SYMBOL_GPL(asm_pswrite_trylock_slow);
void asm_pswrite_unlock_slow(void);
EXPORT_SYMBOL_GPL(asm_pswrite_unlock_slow);
void asm_psrwlock_wakeup(void);
EXPORT_SYMBOL_GPL(asm_psrwlock_wakeup);
