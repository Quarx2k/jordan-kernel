/*
 * linux/include/asm-arm/arch-omap/omap-serial.h
 *
 * register definitions for omap24xx/omap34xx uart controller.
 *
 * Copyright (C) 2008-2009 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef __OMAP_SERIAL_H__
#define __OMAP_SERIAL_H__

/* UART NUMBERS */
#define MAX_UARTS 			4
#define UART1				(0x0)
#define UART2				(0x1)
#define UART3				(0x2)
#define UART4				(0x3)

#define UART_BASE(uart_no)		(uart_no == UART1)?OMAP_UART1_BASE :\
					(uart_no == UART2)?OMAP_UART2_BASE :\
					 OMAP_UART3_BASE

#define UART_MODULE_BASE(uart_no)	 (UART1 == uart_no ? IO_ADDRESS(OMAP_UART1_BASE) :\
		                                 ( UART2 == uart_no ? IO_ADDRESS(OMAP_UART2_BASE) :\
					         IO_ADDRESS(OMAP_UART3_BASE) ))
extern unsigned int fcr[MAX_UARTS];
#endif /* __OMAP_SERIAL_H__ */

