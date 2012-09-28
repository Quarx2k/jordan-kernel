#ifndef _ASM_SPARC_IMMEDIATE_H
#define _ASM_SPARC_IMMEDIATE_H

#include <asm/asm.h>

/*
 * Immediate values. Sparc64 architecture optimizations.
 *
 * (C) Copyright 2009 David Miller <davem@davemloft.net>
 * (C) Copyright 2009 Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 *
 * Dual BSD/GPL v2 license.
 */

struct __imv {
	unsigned long var;
	unsigned long imv;
	unsigned char size;
} __attribute__ ((packed));

#define imv_read(name)							\
	({								\
		__typeof__(name##__imv) value;				\
		BUILD_BUG_ON(sizeof(value) > 8);			\
		switch (sizeof(value)) {				\
		case 1:							\
			asm(".section __imv,\"aw\",@progbits\n\t"	\
					_ASM_UAPTR " %c1, 1f\n\t"	\
					".byte 1\n\t"			\
					".previous\n\t"			\
					"1: mov 0, %0\n\t"		\
				: "=r" (value)				\
				: "i" (&name##__imv));			\
			break;						\
		case 2:							\
		case 4:							\
		case 8:	value = name##__imv;				\
			break;						\
		};							\
		value;							\
	})

#define imv_cond(name)	imv_read(name)
#define imv_cond_end()

extern int arch_imv_update(const struct __imv *imv, int early);

#endif /* _ASM_SPARC_IMMEDIATE_H */
