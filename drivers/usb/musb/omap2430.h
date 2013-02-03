/*
 * Copyright (C) 2005-2006 by Texas Instruments
 *
 * The Inventra Controller Driver for Linux is free software; you
 * can redistribute it and/or modify it under the terms of the GNU
 * General Public License version 2 as published by the Free Software
 * Foundation.
 */

#ifndef __MUSB_OMAP243X_H__
#define __MUSB_OMAP243X_H__

#include <plat/usb.h>

/*
 * OMAP2430-specific definitions
 */

#define OTG_REVISION		0x400

#define OTG_SYSCONFIG		0x404
#	define	MIDLEMODE	12	/* bit position */
#	define	FORCESTDBY		(0 << MIDLEMODE)
#	define	NOSTDBY			(1 << MIDLEMODE)
#	define	SMARTSTDBY		(2 << MIDLEMODE)

#	define	SIDLEMODE		3	/* bit position */
#	define	FORCEIDLE		(0 << SIDLEMODE)
#	define	NOIDLE			(1 << SIDLEMODE)
#	define	SMARTIDLE		(2 << SIDLEMODE)
#	define	SMARTIDLEWKUP		(3 << SIDLEMODE)

#	define	ENABLEWAKEUP		(1 << 2)
#	define	SOFTRST			(1 << 1)
#	define	AUTOIDLE		(1 << 0)

#define OTG_SYSSTATUS		0x408
#	define	RESETDONE		(1 << 0)

#define OTG_INTERFSEL		0x40c
#	define	EXTCP			(1 << 2)
#	define	PHYSEL			0	/* bit position */
#	define	UTMI_8BIT		(0 << PHYSEL)
#	define	ULPI_12PIN		(1 << PHYSEL)
#	define	ULPI_8PIN		(2 << PHYSEL)

#define OTG_SIMENABLE		0x410
#	define	TM1			(1 << 0)

#define OTG_FORCESTDBY		0x414
#	define	ENABLEFORCE		(1 << 0)

#define USBOTGHS_CONTROL	0x33c
#	define	AVALID			(1 << 0)
#	define	BVALID			(1 << 1)
#	define	VBUSVALID		(1 << 2)
#	define	SESSEND			(1 << 3)
#	define	IDDIG			(1 << 4)

#define CONTROL_DEV_CONF	0x300
#	define PHY_PD			(1 << 0)

#define USBA0_OTG_CE_PAD1_USBA0_OTG_DP	0x194
#	define	DP_WAKEUPENABLE		(1 << 30)

#define USBPHY_ANA_CONFIG2	0x54
#	define RTERM_TEST			(3 << 15)

#define USBPHY_TERMINATION_CONTROL	0x0
#	define HS_CODE_SEL			(7 << 11)
#	define HS_CODE_SEL_Eye_Diag_Inc	(3 << 11)

#define OCP2SCP_TIMING	0x18
#	define SYNC2				0xf

static inline
void omap_modify_reg32(u32 *_reg, const u32 _clear_mask,
			const u32 _set_mask)
{
	__raw_writel((__raw_readl(_reg) & ~_clear_mask) | _set_mask, _reg);
}

#define OCP2SCP_REG_BASE_PHY	0x4A0AD000
#define OCP2SCP_REG_LEN			64
#define USBPHY_REG_BASE_PHY		0x4A0AD080
#define USBPHY_REG_LEN			3968
#endif	/* __MUSB_OMAP243X_H__ */
