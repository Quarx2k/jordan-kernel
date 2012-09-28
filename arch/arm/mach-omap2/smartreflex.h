#ifndef __ARCH_ARM_MACH_OMAP3_SMARTREFLEX_H
#define __ARCH_ARM_MACH_OMAP3_SMARTREFLEX_H
/*
 * linux/arch/arm/mach-omap2/smartreflex.h
 *
 * Copyright (C) 2009 Texas Instruments, Inc.
 * Nishanth Menon
 *
 * Copyright (C) 2008 Nokia Corporation
 * Kalle Jokiniemi
 *
 * Copyright (C) 2007 Texas Instruments, Inc.
 * Lesly A M <x0080970@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* SMART REFLEX REG ADDRESS OFFSET */
#define SRCONFIG		0x00
#define SRSTATUS		0x04
#define SENVAL			0x08
#define SENMIN			0x0C
#define SENMAX			0x10
#define SENAVG			0x14
#define AVGWEIGHT		0x18
#define NVALUERECIPROCAL	0x1C
/*OMAP3630SR new interrupt registers*/
#define IRQ_EOI			0x20	/*Not Used*/
#define IRQSTATUS_RAW		0x24
#define IRQSTATUS		0x28
#define IRQENABLE_SET		0x2C
#define IRQENABLE_CLR		0x30
/*OMAP3630SR change offset from 0x20 to 0x34*/
#define SENERROR_36XX		0x34
/*OMAP3630SR change offset from ox24 to 0x38*/
#define ERRCONFIG_36XX		0x38

#define SENERROR_34XX		0x20
#define ERRCONFIG_34XX		0x24
/* SR Modules */
#define SR1	1
#define SR2	2
#define SEN_ENABLE 1

#define VP1_IRQMASK_TRANSDONE	(0x1 << 15)
#define VP2_IRQMASK_TRANSDONE	(0x1 << 21)

/* PRM_VP1_CONFIG */
#define PRM_VP1_CONFIG_ERROROFFSET	(0x00 << 24)
#define SR_ERRGAIN_LOWOPP		(0x0C)
#define SR_ERRGAIN_HIGHOPP		(0x16)
#define SR_ERRMINLIMIT_LOWOPP		(0xF4)
#define SR_ERRMINLIMIT_HIGHOPP		(0xF2)
#define PRM_VP_STATUS_VPINIDLE		(0x1)

#define PRM_VP1_CONFIG_TIMEOUTEN	(0x1 << 3)
#define PRM_VP1_CONFIG_VPENABLE		(0x1)

/* PRM_VP1_VSTEPMIN */
#define PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN	(0x001E << 8)
#define PRM_VP1_VSTEPMIN_VSTEPMIN		(0x01)

/* PRM_VP1_VSTEPMAX */
#define PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX	(0x001E << 8)
#define PRM_VP1_VSTEPMAX_VSTEPMAX		(0x08)

/* PRM_VP1_VLIMITTO */
#define PRM_VP1_VLIMITTO_VDDMAX			(0x44 << 24)
#define PRM_VP1_VLIMITTO_VDDMIN			(0x14 << 16)
#define PRM_VP1_VLIMITTO_TIMEOUT_US		(200)
#define PRM_VP1_VLIMITTO_TIMEOUT_SHIFT		(0)

/* PRM_VP2_CONFIG */
#define PRM_VP2_CONFIG_ERROROFFSET		(0x00 << 24)
#define PRM_VP2_CONFIG_TIMEOUTEN		(0x1 << 3)
#define PRM_VP2_CONFIG_VPENABLE			(0x1)

/* PRM_VP2_VSTEPMIN */
#define PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN	(0x001E << 8)
#define PRM_VP2_VSTEPMIN_VSTEPMIN		(0x01)

/* PRM_VP2_VSTEPMAX */
#define PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX	(0x001E << 8)
#define PRM_VP2_VSTEPMAX_VSTEPMAX		(0x08)

/* PRM_VP2_VLIMITTO */
#define PRM_VP2_VLIMITTO_VDDMAX			(0x42 << 24)
#define PRM_VP2_VLIMITTO_VDDMIN			(0x12 << 16)
#define PRM_VP2_VLIMITTO_TIMEOUT_US		(200)
#define PRM_VP2_VLIMITTO_TIMEOUT_SHIFT		(0)


/* SRCONFIG */
#define SRCLKLENGTH_12MHZ_SYSCLK	0x3C
#define SRCLKLENGTH_13MHZ_SYSCLK	0x41
#define SRCLKLENGTH_19MHZ_SYSCLK	0x60
#define SRCLKLENGTH_26MHZ_SYSCLK	0x82
#define SRCLKLENGTH_38MHZ_SYSCLK	0xC0

#define SRCONFIG_SRCLKLENGTH_SHIFT	12
#define SRCONFIG_34XX_SENNENABLE_SHIFT	5
#define SRCONFIG_34XX_SENPENABLE_SHIFT	3
#define SRCONFIG_36XX_SENNENABLE_SHIFT	1
#define SRCONFIG_36XX_SENPENABLE_SHIFT	0

#define SRCONFIG_SRENABLE		(0x01 << 11)
#define SRCONFIG_SENENABLE		(0x01 << 10)
#define SRCONFIG_ERRGEN_EN		(0x01 << 9)
#define SRCONFIG_MINMAXAVG_EN		(0x01 << 8)

#define SRCONFIG_DELAYCTRL		(0x01 << 2)
#define SRCONFIG_CLKCTRL		(0x00)

/* NVALUERECIPROCAL */
#define NVALUERECIPROCAL_SENPGAIN_SHIFT		20
#define NVALUERECIPROCAL_SENNGAIN_SHIFT		16
#define NVALUERECIPROCAL_RNSENP_SHIFT		8
#define NVALUERECIPROCAL_RNSENN_SHIFT		0

/* ERRCONFIG */
#define SR_CLKACTIVITY_MASK		(0x03 << 20)
#define SR_ERRWEIGHT_MASK		(0x07 << 16)
#define SR_ERRMAXLIMIT_MASK		(0xFF << 8)
#define SR_ERRMINLIMIT_MASK		(0xFF)

#define SR_CLKACTIVITY_IOFF_FOFF	(0x00 << 20)
#define SR_CLKACTIVITY_IOFF_FON		(0x02 << 20)

#define ERRCONFIG_34XX_VPBOUNDINTEN		(0x1 << 31)
#define ERRCONFIG_MCUDISACKINTEN	(0x1 << 23)

/* Status Bits */
#define ERRCONFIG_34XX_VPBOUNDINTST		(0x1 << 30)
#define ERRCONFIG_MCUACCUMINTST		(0x1 << 28)
#define ERRCONFIG_MCUVALIDINTST		(0x1 << 26)
#define ERRCONFIG_MCUBOUNDINTST		(0x1 << 24)
#define ERRCONFIG_MCUDISACKINTST	(0x1 << 22)

#define ERRCONFIG__ERRWEIGHT_MASK			(0x07 << 16)
#define ERRCONFIG__ERRMAXLIMIT_MASK			(0xFF << 8)
#define ERRCONFIG__ERRMINLIMIT_MASK			(0xFF)

#define ERRCONFIG_36XX_VPBOUNDINTEN	(0x1 << 22)
#define ERRCONFIG_36XX_VPBOUNDINTST		(0x1 << 23)

#define ERRCONFIG_WAKEUPENABLE			(0x1 << 26)
#define ERRCONFIG_IDLEMODE_SHIFT			24

/* IRQ */
#define MCUDISABLEACKINT		(0x1 << 0)
#define MCUBOUNDSINT			(0x1 << 1)
#define MCUVALIDINT				(0x1 << 2)
#define MCUACCUMINT				(0x1 << 3)

#define IRQSTATUS_MASK 			(MCUDISABLEACKINT  | MCUBOUNDSINT | \
						   MCUVALIDINT | MCUACCUMINT)


/* WARNING: Ensure all access to errconfig register skips
 * clearing intst bits to ensure that we dont clear status
 * bits unwantedly.. esp vpbound
 */
#define ERRCONFIG_36XX_INTERRUPT_STATUS_MASK (ERRCONFIG_36XX_VPBOUNDINTST |\
					ERRCONFIG_36XX_VPBOUNDINTEN)
#define ERRCONFIG_34XX_INTERRUPT_STATUS_MASK (ERRCONFIG_34XX_VPBOUNDINTST |\
		ERRCONFIG_MCUACCUMINTST | ERRCONFIG_MCUVALIDINTST |\
		ERRCONFIG_MCUBOUNDINTST | ERRCONFIG_MCUDISACKINTST | (0X1<<19))

#define SR1_ERRWEIGHT			(0x04 << 16)
#define SR1_ERRMAXLIMIT			(0x02 << 8)
#define SR1_ERRMINLIMIT			(0xFA)

#define SR1_36XX_ERRMINLIMIT_OPP1	(0xF4)
#define SR1_36XX_ERRMINLIMIT_OPP2	(0xF9)
#define SR1_36XX_ERRMINLIMIT_OPP3	(0xFA)
#define SR1_36XX_ERRMINLIMIT_OPP4	(0xFA)
#define SR1_36XX_ERRMINLIMIT_OPP5	(0xFA)

#define SR1_36XX_ERRGAIN_OPP1	(0x0C)
#define SR1_36XX_ERRGAIN_OPP2	(0x16)
#define SR1_36XX_ERRGAIN_OPP3	(0x23)
#define SR1_36XX_ERRGAIN_OPP4	(0x27)
#define SR1_36XX_ERRGAIN_OPP5	(0x27)

#define SR2_ERRWEIGHT			(0x04 << 16)
#define SR2_ERRMAXLIMIT			(0x02 << 8)
#define SR2_ERRMINLIMIT			(0xFA)

#define SR2_36XX_ERRMINLIMIT_OPP1	(0xF4)
#define SR2_36XX_ERRMINLIMIT_OPP2	(0xF9)

/* T2 SMART REFLEX */
#define R_SRI2C_SLAVE_ADDR		0x12
#define R_VDD1_SR_CONTROL		0x00
#define R_VDD2_SR_CONTROL		0x01

/* VDDs*/
#define PRCM_VDD1	1
#define PRCM_VDD2	2

/*
 * XXX: These should be removed/moved from here once we have a working DVFS
 * implementation in place
 */
#define PHY_TO_OFF_PM_MASTER(p)		(p - 0x36)
#define PHY_TO_OFF_PM_RECIEVER(p)	(p - 0x5b)
#define PHY_TO_OFF_PM_INT(p)		(p - 0x2e)

/* Vmode control */
#define R_DCDC_GLOBAL_CFG		PHY_TO_OFF_PM_RECIEVER(0x61)
/* R_DCDC_GLOBAL_CFG register, SMARTREFLEX_ENABLE values */
#define DCDC_GLOBAL_CFG_ENABLE_SRFLX	0x08

/* DEVICE ID/DPLL ID/CLOCK ID: bits 28-31 for OMAP type */
#define OMAP_TYPE_SHIFT			28
#define OMAP_TYPE_MASK			0xF
/* OPP ID: bits: 0-4 for OPP number */
#define OPP_NO_POS			0
#define OPP_NO_MASK			0x1F
/* OPP ID: bits: 5-6 for VDD */
#define VDD_NO_POS			5
#define VDD_NO_MASK			0x3
/* Other IDs: bits 20-27 for ID type */
/* These IDs have bits 25,26,27 as 1 */
#define OTHER_ID_TYPE_SHIFT		20
#define OTHER_ID_TYPE_MASK		0xFF

#define OTHER_ID_TYPE(X) ((X & OTHER_ID_TYPE_MASK) << OTHER_ID_TYPE_SHIFT)
#define ID_OPP_NO(X)	 ((X & OPP_NO_MASK) << OPP_NO_POS)
#define ID_VDD(X)	 ((X & VDD_NO_MASK) << VDD_NO_POS)
#define OMAP(X)		 ((X >> OMAP_TYPE_SHIFT) & OMAP_TYPE_MASK)
#define get_opp_no(X)	 ((X >> OPP_NO_POS) & OPP_NO_MASK)
#define get_vdd(X)	 ((X >> VDD_NO_POS) & VDD_NO_MASK)

/* XXX: end remove/move */

/* XXX: find more appropriate place for these once DVFS is in place */
extern u32 current_vdd1_opp;
extern u32 current_vdd2_opp;

extern u8 sr_class1p5;
/*
 * Smartreflex module enable/disable interface.
 * NOTE: if smartreflex is not enabled from sysfs, these functions will not
 * do anything.
 */
#ifdef CONFIG_OMAP_SMARTREFLEX
void enable_smartreflex(u8 srid);
void disable_smartreflex(u8 srid);
int sr_voltage_set(u32 target_opp, u32 current_opp,
		    u8 target_vsel, u8 current_vsel);
int sr_vp_disable_both(u32 target_opp, u32 current_opp);
int sr_vp_enable_both(u32 target_opp, u32 current_opp);
int vc_send_command(u8 slave_addr, u8 reg_addr, u8 data, u16 *timeout_us);
int omap_pmic_srinit(void);
u32 omap_pmic_voltage_ramp_delay(u8 srid, u8 target_vsel, u8 current_vsel);
#ifdef CONFIG_OMAP_VC_BYPASS_UPDATE
int omap_pmic_voltage_cmds(u8 srid, u8 target_vsel);
#endif

extern int omap3630_abb_change_active_opp(u32 target_opp_no);

int sr_recalibrate(struct omap_opp *opp, u32 target_opp, u32 current_opp);

#ifdef CONFIG_PM_DEBUG
#define __SR_DEBUG
#endif

#else
static inline void enable_smartreflex(u8 srid)
{
}

static inline void disable_smartreflex(u8 srid)
{
}

static inline int sr_voltage_set(u32 target_opp, u32 current_opp,
				  u8 target_vsel, u8 current_vsel)
{
	return -EINVAL;
}

static inline int sr_vp_disable_both(u32 target_opp, u32 current_opp)
{
	return -EINVAL;
}

static inline int sr_vp_enable_both(u32 target_opp, u32 current_opp)
{
	return -EINVAL;
}

static inline int vc_send_command(u8 slave_addr, u8 reg_addr, u8 data,
				  u16 *timeout_us)
{
	return -EINVAL;
}

#ifdef CONFIG_OMAP_VC_BYPASS_UPDATE
static inline int omap_pmic_voltage_cmds(u8 srid, u8 target_vsel)
{
	return -EINVAL;
}
#endif
int sr_recalibrate(struct omap_opp *opp, u32 target_opp, u32 current_opp)
{
	return -EINVAL;
}
#endif

#include <linux/dcache.h>
#ifdef __SR_DEBUG
int sr_debugfs_create_entries(struct dentry *d);
#else
static inline int sr_debugfs_create_entries(struct dentry *d)
{
	return -EINVAL;
}
#endif

#endif				/* __ARCH_ARM_MACH_OMAP3_SMARTREFLEX_H */
