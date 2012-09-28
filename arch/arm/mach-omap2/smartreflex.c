/*
 * linux/arch/arm/mach-omap3/smartreflex.c
 *
 * OMAP34XX SmartReflex Voltage Control
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/sysfs.h>
#include <linux/kobject.h>
#include <linux/i2c/twl4030.h>
#include <linux/io.h>
#include <linux/debugfs.h>

#include <plat/omap34xx.h>
#include <plat/control.h>
#include <plat/clock.h>
#include <plat/omap-pm.h>
#include <plat/resource.h>
#include <plat/powerdomain.h>
#include <plat/omap-pm.h>

#include "prm.h"
#include "smartreflex.h"
#include "prm-regbits-34xx.h"

/*OMAP3430 Turbo mode: ARM run at 800Mhz*/
/*#define SR_TURBO*/
/*OMAP3430 Highspeed mode: ARM run at 720Mhz*/
#define SR_HIGHSPEED

/* MCUDISACK is expected to happen within 1uSec. */
#define COUNT_TIMEOUT_MCUDISACK		200

/* VPINIDLE is expected to happen within 100uSec. Typical is 2uSec */
#define COUNT_TIMEOUT_VPINIDLE		200

/* Time taken for setting the device - worst case as FS I2C
 * Depends on SMPSWAITIME MIN/MAX Typical is 200uSec
 */
#define COUNT_TIMEOUT_TRANSDONE_SET	400

/* Time to clear out multiple transdone events typical is 3uSec */
#define COUNT_TIMEOUT_TRANSDONE_CLR	50

/* Time For VCBypass mode for TWL4030 derivative chip. */
#define COUNT_TIMEOUT_TWL4030_VCBYPASS	500

/* How many retries to do for I2C errors seen on bus for Forceupdate? */
#define COUNT_RETRY_SMPSNOACK		2

/* The number of steps for NTargets margins */
#define NUM_STEPS_MARGIN			6
/* The number of steps for 1.2G OPP NTargets margins */
#define NUM_STEPS_MARGIN_1_2G 			4
/* The number of steps for 1.0G OPP NTargets margins */
#define NUM_STEPS_MARGIN_1_0G                   2


#define SR_REGADDR(offset)	(sr->srbase_addr + (offset))

/* Which function to use for setting voltage */
#ifdef CONFIG_OMAP_VC_BYPASS_UPDATE
#define SR_CHOSEN_VOLTAGE_UPDATE_MECH  sr_vc_bypass
#else
#define SR_CHOSEN_VOLTAGE_UPDATE_MECH  sr_vp_forceupdate
#endif

static ssize_t omap_sr_vdd_autocomp_show(struct kobject *kobj,
					 struct kobj_attribute *attr,
					 char *buf);
static ssize_t omap_sr_vdd_autocomp_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf, size_t n);

/* 5 values each for the valid OPP please.. */
static u32 __initdata sr1_34xx_test_nvalues[] = {
	/* *INDENT-OFF* */
	0x00000000,	0x00000000,
	0x00AAB48A,	0x00ABA2E6,
	0x00AB90D3
	/* *INDENT-ON* */
};
/* 3 values each for the valid OPP please.. */
static u32 __initdata sr2_34xx_test_nvalues[] = {
	/* *INDENT-OFF* */
	0x00000000,	0x00000000,
	0x00AAC695
	/* *INDENT-ON* */
};

#define SR_N_MOD 0x3
#define SR_P_MOD 0x3

/* 4 values each for the valid OPP please.. */
static u32 __initdata sr1_36xx_test_nvalues[] = {
	/* *INDENT-OFF* */
	0x008985e1,	0x009a90f4,
	0x00aadaba,	0x00aaa58e
	/* *INDENT-ON* */
};

static u32 __initdata sr2_36xx_test_nvalues[] = {
	/* *INDENT-OFF* */
	0x0099eec8,	0x00aaf3cf
	/* *INDENT-ON* */
};

/* Structure for Voltage processor */
struct omap_sr_vp {
	/* Store the commonly used register offsets.
	 * this saves a if condition decision
	 */
	u16 prm_vpx_status_offset;
	u16 prm_vpx_config_offset;
	u16 prm_vpx_stepmin_offset;
	u16 prm_vpx_stepmax_offset;
	u16 prm_vpx_limito_offset;
	u32 prm_vpx_vlimito_timeout;
	u8 prm_vpx_vlimito_shift;
	u16 prm_vpx_voltage_offset;
	u16 prm_vc_cmd_val_offset;
	/* Store the defaults
	 * allowing us to save OCP read
	 * operation
	 */
	u32 vpconfig_value;
	u32 vpstepmin_value;
	u32 vpstepmax_value;
	u32 vplimito_value;
	u32 vpenable_mask;
	u32 irqmask_trans_done;
	u32 irqmask_smps_noack;
};

/* Structure for Smart Reflex */
struct omap_sr {
	u8 srid;
	u8 prcm_vdd;
	char *vdd_name;
	struct kobj_attribute autocom_attr;
	struct omap_opp **omap_opp;
	/* SR activity marker */
	u8 is_sr_reset;
	u8 is_autocomp_active;
	u32 req_opp_no;
	u32 opp_nvalue[6];
	u32 sr_config_value;
	u32 sr_errconfig_value;
	u32 sr_n_mod_mask;
	u8 sr_n_mod_shift;
	u32 sr_p_mod_mask;
	u8 sr_p_mod_shift;
	struct clk *fclk;
	struct clk *iclk;
	void __iomem *srbase_addr;
	char *iclk_name;
	char *fclk_name;
	/* Voltage processor for the specific SR module */
	struct omap_sr_vp vp;
	u8 num_opp;
	u8 opp_boundary;
	u8 last_opp;
	u32 errgain[6];
	u32 errminlimit[6];
};

/* Smart Reflex 1 structure */
static struct omap_sr sr1 = {
	/* *INDENT-OFF* */
	.srid			= SR1,
	.prcm_vdd		= PRCM_VDD1,
	.vdd_name		= "vdd1_opp",
	.omap_opp		= &mpu_opps,
	.autocom_attr		= {
		.attr = {
			 .name = __stringify(sr_vdd1_autocomp),
			 .mode = 0644,
			 },
		.show = omap_sr_vdd_autocomp_show,
		.store = omap_sr_vdd_autocomp_store,
	},
	.is_sr_reset		= 1,
#ifdef CONFIG_OMAP_SMARTREFLEX_CLASS1P5
	.is_autocomp_active	= 1,
#else
	.is_autocomp_active	= 0,
#endif
	.srbase_addr		= (void *)OMAP34XX_SR1_BASE,
	.fclk_name		= "sr1_fck",
	.iclk_name		= "sr_l4_ick",
	/*Adding ERRCONFIG_VPBOUNDINTEN  & ERRCONFIG_VPBOUNDINTST at init */
	.sr_errconfig_value	= SR1_ERRWEIGHT | SR1_ERRMAXLIMIT,
	.sr_n_mod_mask		= OMAP343X_SR1_SENNENABLE_MASK,
	.sr_n_mod_shift		= OMAP343X_SR1_SENNENABLE_SHIFT,
	.sr_p_mod_mask		= OMAP343X_SR1_SENPENABLE_MASK,
	.sr_p_mod_shift		= OMAP343X_SR1_SENPENABLE_SHIFT,
	.vp = {
		.prm_vpx_status_offset	= OMAP3_PRM_VP1_STATUS_OFFSET,
		.prm_vpx_config_offset	= OMAP3_PRM_VP1_CONFIG_OFFSET,
		.prm_vpx_stepmin_offset	= OMAP3_PRM_VP1_VSTEPMIN_OFFSET,
		.prm_vpx_stepmax_offset	= OMAP3_PRM_VP1_VSTEPMAX_OFFSET,
		.prm_vpx_limito_offset	= OMAP3_PRM_VP1_VLIMITTO_OFFSET,
		.prm_vpx_vlimito_timeout = PRM_VP1_VLIMITTO_TIMEOUT_US,
		.prm_vpx_vlimito_shift = PRM_VP1_VLIMITTO_TIMEOUT_SHIFT,
		.prm_vpx_voltage_offset = OMAP3_PRM_VP1_VOLTAGE_OFFSET,
		.prm_vc_cmd_val_offset = OMAP3_PRM_VC_CMD_VAL_0_OFFSET,
		.vpconfig_value		= PRM_VP1_CONFIG_ERROROFFSET |
			PRM_VP1_CONFIG_TIMEOUTEN,
		.vpstepmin_value	= PRM_VP1_VSTEPMIN_SMPSWAITTIMEMIN |
			PRM_VP1_VSTEPMIN_VSTEPMIN,
		.vpstepmax_value	= PRM_VP1_VSTEPMAX_SMPSWAITTIMEMAX |
			PRM_VP1_VSTEPMAX_VSTEPMAX,
		.vplimito_value		= PRM_VP1_VLIMITTO_VDDMAX |
			PRM_VP1_VLIMITTO_VDDMIN,
		.vpenable_mask		= PRM_VP1_CONFIG_VPENABLE,
		.irqmask_trans_done	= VP1_IRQMASK_TRANSDONE,
		.irqmask_smps_noack	= OMAP3430_VP1_NOSMPSACK_ST,
	},
	.num_opp		= 5,
	.opp_boundary		= 3,
	.last_opp		= 0,
	/* *INDENT-ON* */
};

/* Smart Reflex 2 structure */
static struct omap_sr sr2 = {
	/* *INDENT-OFF* */
	.srid			= SR2,
	.prcm_vdd		= PRCM_VDD2,
	.vdd_name		= "vdd2_opp",
	.omap_opp		= &l3_opps,
	.autocom_attr		= {
		.attr = {
			 .name = __stringify(sr_vdd2_autocomp),
			 .mode = 0644,
			 },
		.show = omap_sr_vdd_autocomp_show,
		.store = omap_sr_vdd_autocomp_store,
	},
	.is_sr_reset		= 1,
	.is_autocomp_active	= 0,
	.srbase_addr		= (void *)OMAP34XX_SR2_BASE,
	.fclk_name		= "sr2_fck",
	.iclk_name		= "sr_l4_ick",
	/*Adding ERRCONFIG_VPBOUNDINTEN & ERRCONFIG_VPBOUNDINTST at init */
	.sr_errconfig_value	= SR2_ERRWEIGHT | SR2_ERRMAXLIMIT,
	.sr_n_mod_mask		= OMAP343X_SR2_SENNENABLE_MASK,
	.sr_n_mod_shift		= OMAP343X_SR2_SENNENABLE_SHIFT,
	.sr_p_mod_mask		= OMAP343X_SR2_SENPENABLE_MASK,
	.sr_p_mod_shift		= OMAP343X_SR2_SENPENABLE_SHIFT,
	.vp = {
		.prm_vpx_status_offset	= OMAP3_PRM_VP2_STATUS_OFFSET,
		.prm_vpx_config_offset	= OMAP3_PRM_VP2_CONFIG_OFFSET,
		.prm_vpx_stepmin_offset	= OMAP3_PRM_VP2_VSTEPMIN_OFFSET,
		.prm_vpx_stepmax_offset	= OMAP3_PRM_VP2_VSTEPMAX_OFFSET,
		.prm_vpx_limito_offset	= OMAP3_PRM_VP2_VLIMITTO_OFFSET,
		.prm_vpx_vlimito_timeout = PRM_VP2_VLIMITTO_TIMEOUT_US,
		.prm_vpx_vlimito_shift = PRM_VP2_VLIMITTO_TIMEOUT_SHIFT,
		.prm_vpx_voltage_offset = OMAP3_PRM_VP2_VOLTAGE_OFFSET,
		.prm_vc_cmd_val_offset = OMAP3_PRM_VC_CMD_VAL_1_OFFSET,
		.vpconfig_value		= PRM_VP2_CONFIG_ERROROFFSET |
			PRM_VP2_CONFIG_TIMEOUTEN,
		.vpstepmin_value	= PRM_VP2_VSTEPMIN_SMPSWAITTIMEMIN |
			PRM_VP2_VSTEPMIN_VSTEPMIN,
		.vpstepmax_value	= PRM_VP2_VSTEPMAX_SMPSWAITTIMEMAX |
			PRM_VP2_VSTEPMAX_VSTEPMAX,
		.vplimito_value		= PRM_VP2_VLIMITTO_VDDMAX |
			PRM_VP2_VLIMITTO_VDDMIN,
		.vpenable_mask		= PRM_VP2_CONFIG_VPENABLE,
		.irqmask_trans_done	= VP2_IRQMASK_TRANSDONE,
		.irqmask_smps_noack	= OMAP3430_VP2_NOSMPSACK_ST,
	},
	.num_opp		= 3,
	.opp_boundary		= 3,
	.last_opp		= 0,
	/* *INDENT-ON* */
};

static u32 sr_errgain_lowopp = SR_ERRGAIN_LOWOPP;
static u32 sr_errgain_highopp = SR_ERRGAIN_HIGHOPP;
static u32 sr_errminlimit_lowopp = SR_ERRMINLIMIT_LOWOPP;
static u32 sr_errminlimit_highopp = SR_ERRMINLIMIT_HIGHOPP;
/*********************** OPP Accessor functions ****************************/
/**
 * @brief *get_sr - get SR pointer from an SRID
 *
 * @param srid - vddid
 *
 * @return struct pointer if found, else BUG()s
 */
static inline struct omap_sr *get_sr(u8 srid)
{
	return (srid == 1) ? &sr1 : &sr2;
}

/**
 * @brief *get_sr_from_vdd - get the SR structure indexed by
 * VDD ID
 *
 * @param vddid - vddid
 *
 * @return struct pointer if found, else BUG()s
 */
static inline struct omap_sr *get_sr_from_vdd(u8 vddid)
{
	/* Currently, the SRID and VDDID are the same, misusing it */
	return get_sr(vddid);
}

/**
 * @brief *get_sr_from_vdd_name - get the SR structure from
 * sysfs name
 *
 * @param name -sysfs entry name
 *
 * @return sr struct pointer if found else NULL
 */
static struct omap_sr *get_sr_from_vdd_name(char *name)
{
	int i;
	struct omap_sr *sr;
	for (i = 0; i < 2; i++) {
		sr = get_sr(i + 1);
		if (!strcmp(name, sr->autocom_attr.attr.name))
			return sr;
	}
	/* Nothin found, BUG!! */
	BUG();
	return NULL;
}

/**
 * @brief get_current_opp_number_from_sr - get the current active OPP number
 * from SR pointer
 *
 * @param sr struct pointer
 *
 * @return current OPP ID
 */
static inline int get_current_opp_number_from_sr(struct omap_sr *sr)
{
	return (*sr->omap_opp)[resource_get_level(sr->vdd_name)].opp_id;
}

/**
 * @brief get_vsel_for_opp - get the VSEL value for the SR/OPP combination
 *
 * @param sr  struct pointer
 * @param opp opp number desired for
 *
 * @return vsel value, if bad opp number is given, this will BUG()
 */
static inline u8 get_vsel_for_opp(struct omap_sr *sr, int opp)
{
	/* Dont ask me to derefence nonexistant OPPs!
	 * TODO: add check for valid OPPs here
	 */
	u8 vsel;
	u32 actual_vsel;
	struct omap_sr_vp *vp = &sr->vp;
	BUG_ON(opp > sr->num_opp);

	if (!sr_class1p5) {
		vsel = (*sr->omap_opp)[opp].vsel;
		if (sr->last_opp != 0
		&& (sr->last_opp == (opp+1))
		&& sr->is_autocomp_active) {
			actual_vsel = prm_read_mod_reg(OMAP3430_GR_MOD,
						vp->prm_vpx_voltage_offset);
			if (actual_vsel <= vsel)
				vsel = actual_vsel;
		}
	} else {
		vsel = (*sr->omap_opp)[opp].sr_adjust_vsel;
		if (!sr->is_autocomp_active || !vsel)
			vsel = (*sr->omap_opp)[opp].vsel;
	}
	return vsel;
}

static inline void sr_udelay(u32 delay)
{
	while (delay-- > 0) {
		cpu_relax();
		udelay(1);
	};

}
/****************** SMART REFLEX DEBUGFS ENTRIES *************************/
#ifdef __SR_DEBUG

/**
 * @brief sr_debugfs_set - set variable with value
 *
 * @param data - variable pointer
 * @param val - value to set
 *
 * @return - 0
 */
static int sr_debugfs_set(void *data, u64 val)
{
	u32 *option = data;

	*option = val;

	return 0;
}

/**
 * @brief sr_debugfs_get - setup the params
 *
 * @param data - variable to set
 * @param val - value to return
 *
 * @return - 0
 */
static int sr_debugfs_get(void *data, u64 *val)
{
	u32 *option = data;

	*val = *option;

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sr_debugfs_option_fops, sr_debugfs_get, sr_debugfs_set,
			"%llu\n");

/**
 * @brief sr_debugfs_vselget - return the vsel value
 *
 * @param data - pointer to vp
 * @param val - value we read back
 *
 * @return
 */
static int sr_debugfs_vselget(void *data, u64 *val)
{
	struct omap_sr_vp *vp = (struct omap_sr_vp *)data;

	*val = prm_read_mod_reg(OMAP3430_GR_MOD, vp->prm_vpx_voltage_offset);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(sr_debugfs_option_vsel_fops, sr_debugfs_vselget, NULL,
			"%llu\n");

/* Temporary store of pm_debug directory entry */
static __initdata struct dentry *stored_pm_d;

/**
 * @brief sr_debugfs_create_entries - create the Smart Reflex entries
 * called from pm-debug, this just stores it for SR to use in late_init
 *
 * @param d - parent directory tree
 *
 * @return 0 if all ok, else returns with error
 */
int __init sr_debugfs_create_entries(struct dentry *d)
{
	stored_pm_d = d;
	return 0;
}

/**
 * @brief sr_debugfs_create_entries_late - create the Smart Reflex entries -
 * called as part of init sequence of SR uses the dentry registered early
 *
 * @param d - parent directory tree
 *
 * @return 0 if all ok, else returns with error
 */
static __init int sr_debugfs_create_entries_late(void)
{
	struct dentry *sr_dir;
	struct dentry *sr_sub_dir;

	int count;

	sr_dir = debugfs_create_dir("smartreflex", stored_pm_d);
	if (IS_ERR(sr_dir))
		return PTR_ERR(sr_dir);

	if (!cpu_is_omap3630()) {
		(void)debugfs_create_file("sr_errgain_lowopp",
					  S_IRUGO | S_IWUGO, sr_dir,
					  &sr_errgain_lowopp,
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("sr_errgain_highopp",
					  S_IRUGO | S_IWUGO, sr_dir,
					  &sr_errgain_highopp,
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("sr_errminlimit_lowopp",
					  S_IRUGO | S_IWUGO, sr_dir,
					  &sr_errminlimit_lowopp,
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("sr_errminlimit_highopp",
					  S_IRUGO | S_IWUGO, sr_dir,
					  &sr_errminlimit_highopp,
					  &sr_debugfs_option_fops);
	}
	for (count = 0; count < 2; count++) {
		int i;
		struct omap_sr *sr = get_sr(count + 1);
		char name[] = "SR0";
		char errgain_name[] = "OPP0_errgain";
		char errminlimit_name[] = "OPP0_errminlimit";
		char nval_name[] = "OPP0_nvalue";
		name[2] += sr->srid;
		sr_sub_dir = debugfs_create_dir(name, sr_dir);
		if (IS_ERR(sr_sub_dir))
			continue;
		for (i = 0; i < sr->num_opp; i++) {
			nval_name[3]++;
			(void)debugfs_create_file(nval_name, S_IRUGO | S_IWUGO,
						  sr_sub_dir,
						  &(sr->opp_nvalue[i]),
						  &sr_debugfs_option_fops);
			if (cpu_is_omap3630()) {
				errgain_name[3]++;
				(void)debugfs_create_file(errgain_name,
					S_IRUGO | S_IWUGO, sr_sub_dir,
					&sr->errgain[i],
					&sr_debugfs_option_fops);
				errminlimit_name[3]++;
				(void)debugfs_create_file(errminlimit_name,
					S_IRUGO | S_IWUGO, sr_sub_dir,
					&sr->errminlimit[i],
					&sr_debugfs_option_fops);
				}
		}
		(void)debugfs_create_file("errconfig_value",
					  S_IRUGO | S_IWUGO, sr_sub_dir,
					  &(sr->sr_errconfig_value),
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("config_value", S_IRUGO | S_IWUGO,
					  sr_sub_dir, &(sr->sr_config_value),
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("vpconfig_value", S_IRUGO | S_IWUGO,
					  sr_sub_dir, &(sr->vp.vpconfig_value),
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("vpstepmin_value", S_IRUGO | S_IWUGO,
					  sr_sub_dir, &(sr->vp.vpstepmin_value),
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("vpstepmax_value", S_IRUGO | S_IWUGO,
					  sr_sub_dir, &(sr->vp.vpstepmax_value),
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("vplimito_value", S_IRUGO | S_IWUGO,
					  sr_sub_dir, &(sr->vp.vplimito_value),
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("vplimito_value", S_IRUGO | S_IWUGO,
					  sr_sub_dir, &(sr->vp.vplimito_value),
					  &sr_debugfs_option_fops);
		(void)debugfs_create_file("vsel", S_IRUGO, sr_sub_dir,
				&(sr->vp), &sr_debugfs_option_vsel_fops);
	}

	return 0;
}

#else

static inline int sr_debugfs_create_entries_late(void)
{
	return 0;
}
#endif				/* __SR_DEBUG */

/****************** PMIC WEAK FUNCTIONS FOR TWL4030 derivatives **********/

/**
 * @brief pmic_srinit - Power management IC initialization
 * for smart reflex. The current code is written for TWL4030
 * derivatives, replace in board file if PMIC requires
 * a different sequence
 *
 * @return result of operation
 */
int __weak __init omap_pmic_srinit(void)
{
	int ret = -ENODEV;
#ifdef CONFIG_TWL4030_CORE
	u8 reg;
	/* Enable SR on T2 */
	ret = twl4030_i2c_read_u8(TWL4030_MODULE_PM_RECEIVER, &reg,
				  R_DCDC_GLOBAL_CFG);

	reg |= DCDC_GLOBAL_CFG_ENABLE_SRFLX;
	ret |= twl4030_i2c_write_u8(TWL4030_MODULE_PM_RECEIVER, reg,
				    R_DCDC_GLOBAL_CFG);
#endif				/* End of CONFIG_TWL4030_CORE */
	return ret;
}

/**
 * @brief omap_pmic_voltage_ramp_delay - how much should this pmic ramp delay
 * Various PMICs have different ramp up and down delays. choose to implement
 * in required pmic file to override this function.
 * On TWL4030 derivatives:
 *  T2 SMPS slew rate (min) 4mV/uS, step size 12.5mV,
 *  2us added as buffer.
 *
 * @param srid - which SR is this for other PMIC
 * @param target_vsel - targetted voltage selction
 * @param current_vsel - current voltage selection
 *
 * @return delay in uSeconds
 */
u32 __weak omap_pmic_voltage_ramp_delay(u8 srid, u8 target_vsel,
					u8 current_vsel)
{
	u32 t2_smps_steps = abs(target_vsel - current_vsel);
	u32 t2_smps_delay = ((t2_smps_steps * 125) / 40) + 2;
	return t2_smps_delay;
}

#ifdef CONFIG_OMAP_VC_BYPASS_UPDATE
/**
 * @brief omap_pmic_voltage_cmds - hook for pmic command sequence
 * to be send out which are specific to pmic to set a specific voltage.
 * this should inturn call vc_send_command with the required sequence
 * The current implementation is for TWL4030 derivatives
 *
 * @param srid - which SR is this for?
 * @param target_vsel - what voltage is desired to be set?
 *
 * @return specific value to set.
 */
int __weak omap_pmic_voltage_cmds(u8 srid, u8 target_vsel)
{
	u8 reg_addr = (srid == SR1) ? R_VDD1_SR_CONTROL : R_VDD2_SR_CONTROL;
	u16 timeout = COUNT_TIMEOUT_TWL4030_VCBYPASS;
	return vc_send_command(R_SRI2C_SLAVE_ADDR, reg_addr, target_vsel,
			       &timeout);
}
#endif				/* ifdef CONFIG_OMAP_VC_BYPASS_UPDATE */

/*********************** Voltage Controller functions *************************/

/**
 * @brief vc_send_command - The actual command transmission using
 * Voltage controller on I2C4
 *
 * @param slave_addr - what is the PMIC slave address
 * @param reg_addr  - what is the register address I should be using?
 * @param data - what value do you want to write here?
 * @param timeout_us timeout in uSeconds - returns actual time left
 *
 * @return 0 if all ok, else error value
 */
int vc_send_command(u8 slave_addr, u8 reg_addr, u8 data, u16 *timeout_us)
{
	u32 value;
	u32 count;

	if (unlikely(!timeout_us))
		return -EINVAL;

	/* timeout = timeout_us/10 -> each udelay event
	 * 1 udelay event every 50 iteration, assuming
	 * each iteration is instaneous,
	 * count = (timeout_us/10) * 50 or timeout_us * 5
	 */
	count = *timeout_us * 50;

	value = (data << OMAP3430_DATA_SHIFT) |
	    (reg_addr << OMAP3430_REGADDR_SHIFT) |
	    (slave_addr << OMAP3430_SLAVEADDR_SHIFT);

	prm_write_mod_reg(value, OMAP3430_GR_MOD,
			  OMAP3_PRM_VC_BYPASS_VAL_OFFSET);

	value = prm_set_mod_reg_bits(OMAP3430_VALID, OMAP3430_GR_MOD,
				     OMAP3_PRM_VC_BYPASS_VAL_OFFSET);
	/*
	 * Do continous 50 checks then follow with a 10usec delay,
	 * then check again
	 */
	do {
		value = prm_read_mod_reg(OMAP3430_GR_MOD,
					 OMAP3_PRM_VC_BYPASS_VAL_OFFSET)
		    & OMAP3430_VALID;
		/* should i wait? */
		if (value && (count % 50)) {
			cpu_relax();
			udelay(1);
			*timeout_us -= 1;
		}
		count--;
	} while (value && count);
	if (!count) {
		pr_crit("VC:Command Timedout! slave_addr=0x%02X,reg=0x%02X"
			",value=0x%02X \n", slave_addr, reg_addr, data);
		return -ETIMEDOUT;
	}
	return 0;
}
EXPORT_SYMBOL(vc_send_command);

/**
 * @brief sr_vc_bypass - setup voltage using VC Bypass technique
 *
 * @param target_opp - target opp to go to
 * @param current_opp  - current opp
 * @param target_vsel  - which voltage to go to?
 * @param current_vsel  - current voltage
 *
 * @return -success or failure
 */
#ifdef CONFIG_OMAP_VC_BYPASS_UPDATE
static int sr_vc_bypass(struct omap_sr *sr,
			u32 target_opp_no, u8 target_vsel, u8 current_vsel)
{
	int ret = 0;
	struct omap_sr_vp *vp = &sr->vp;
	u32 vpconfig_value;

	vpconfig_value = vp->vpconfig_value;
	if (cpu_is_omap3630()) {
		vpconfig_value |=
		   (sr_errgain[target_opp_np - 1]) << OMAP3430_ERRORGAIN_SHIFT;
	} else {
		vpconfig_value |=
		   ((target_opp_no < sr->opp_boundary) ? sr_errgain_lowopp :
		      sr_errgain_highopp) << OMAP3430_ERRORGAIN_SHIFT;
	}
	vpconfig_value |= target_vsel << OMAP3430_INITVOLTAGE_SHIFT;
	prm_write_mod_reg(vpconfig_value, OMAP3430_GR_MOD,
			  vp->prm_vpx_config_offset);

	prm_rmw_mod_reg_bits(OMAP3430_VC_CMD_ON_MASK,
			     (target_vsel << OMAP3430_VC_CMD_ON_SHIFT),
			     OMAP3430_GR_MOD, vp->prm_vc_cmd_val_offset);

	/*
	 * Various PMIC might need a set of commands
	 * provide hooks for specific PMICs to implement
	 */
	ret = omap_pmic_voltage_cmds(sr->srid, target_vsel);

	/* delay based on pmic */
	if (!ret)
		sr_udelay(omap_pmic_voltage_ramp_delay(sr->srid,
						    target_vsel, current_vsel));

	WARN_ON(ret);

	return ret;
}
#endif				/* ifdef CONFIG_OMAP_VC_BYPASS_UPDATE */

/********************* SR Private functions ***************************/

/**
 * @brief sr_write_reg - write to Smart Reflex Register
 *
 * @param sr  - pointer to SR structure
 * @param offset - SR reg offset to write to
 * @param value - value to write with
 */
static inline void sr_write_reg(struct omap_sr *sr, unsigned offset, u32 value)
{
	__raw_writel(value, SR_REGADDR(offset));
}

/**
 * @brief sr_modify_reg - Modify a register and write a new value to a field
 *
 * @param sr -pointer to SR structure
 * @param offset -register offset
 * @param mask -mask to clear out
 * @param value -value to write there
 */
static inline void sr_modify_reg(struct omap_sr *sr, unsigned offset, u32 mask,
				 u32 value)
{
	u32 reg_val;

	reg_val = __raw_readl(SR_REGADDR(offset));
	reg_val &= ~mask;
	reg_val |= value;

	__raw_writel(reg_val, SR_REGADDR(offset));
}

/**
 * @brief sr_read_reg - read a SR register
 *
 * @param sr - pointer to SR structure
 * @param offset - register offset
 *
 * @return value in that register
 */
static inline u32 sr_read_reg(struct omap_sr *sr, unsigned offset)
{
	return __raw_readl(SR_REGADDR(offset));
}

/**
 * @brief sr_vplimito_value -return the timeout value based on sysclk
 *		and desired timeout uSec
 *
 * @param sys_clk_speed - internal sysclk freq in hz
 * @param timeout_us - timeout in uSec
 *
 * @return timeout value to use in VPLIMITTO:TIMEOUT reg
 */
static inline u32 sr_vplimito_value(u32 sys_clk_speed, u16 timeout_us)
{
	u32 timeout_val;
	/* prevent round off errors, we will divide by 10 later */
	timeout_val = (sys_clk_speed / 100000);
	timeout_val *= timeout_us;
	timeout_val /= 10;
	return timeout_val;
}

/**
 * @brief sr_clk_enable - SR clock enable
 *
 * @param sr - Structure to SR structure
 *
 * @return - result
 */
static int sr_clk_enable(struct omap_sr *sr)
{
	if (clk_enable(sr->iclk) != 0) {
		pr_crit("SR:Could not enable %s for [%d]\n",
			sr->iclk->name, sr->srid);
		return -EINVAL;
	}
	if (clk_enable(sr->fclk) != 0) {
		pr_crit("SR:Could not enable %s for [%d]\n",
			sr->fclk->name, sr->srid);
		clk_disable(sr->iclk);
		return -EINVAL;
	}
	if (!cpu_is_omap3630()) {
		sr_modify_reg(sr, ERRCONFIG_34XX, SR_CLKACTIVITY_MASK |
				ERRCONFIG_34XX_INTERRUPT_STATUS_MASK,
				SR_CLKACTIVITY_IOFF_FON);
	}
	sr->is_sr_reset = 0;

	return 0;
}

/**
 * @brief sr_clk_disable - SR func clock disable
 *
 * @param sr - pointer to SR structure
 */
static void sr_clk_disable(struct omap_sr *sr)
{
	/* set fclk, iclk- idle */
	if (!cpu_is_omap3630())
		sr_modify_reg(sr, ERRCONFIG_34XX, SR_CLKACTIVITY_MASK |
			ERRCONFIG_34XX_INTERRUPT_STATUS_MASK,
			SR_CLKACTIVITY_IOFF_FOFF);

	clk_disable(sr->fclk);
	clk_disable(sr->iclk);
	sr->is_sr_reset = 1;
}

/****************** Voltage processor functions ***************************/

/**
 * @brief sr_vp_clear_vptransdone - clear vptrans_done event
 *
 * @param sr - sr pointer
 *
 * @return 0 if cleared ok, else 1 if timedout!
 */
static int sr_vp_clear_vptransdone(struct omap_sr *sr)
{
	struct omap_sr_vp *vp = &sr->vp;
	u32 irqstat;
	u32 count = COUNT_TIMEOUT_TRANSDONE_CLR;
	do {
		prm_write_mod_reg(vp->irqmask_trans_done, OCP_MOD,
				  OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
		irqstat = prm_read_mod_reg(OCP_MOD,
					   OMAP3_PRM_IRQSTATUS_MPU_OFFSET) &
		    vp->irqmask_trans_done;
		if (irqstat) {
			cpu_relax();
			udelay(1);
		}
		count--;
	} while (count && irqstat);
	if (!count) {
		pr_crit("SR:VPTransdone[%d]:Timedout\n", sr->srid);
		return -ETIMEDOUT;
	}
	return 0;
}

/**
 * @brief sr_vp_forceupdate - do a forceupdate method
 *		to update the voltage level
 *
 * @param sr - pointer to sr structure
 * @param target_opp_no - targetted opp number
 * @param target_vsel - targetted voltage level
 * @param current_vsel - current voltage level
 *
 * @return 0 if all worked out, else 1
 */
#ifndef CONFIG_OMAP_VC_BYPASS_UPDATE
static int sr_vp_forceupdate(struct omap_sr *sr, u32 target_opp_no,
			     u8 target_vsel, u8 current_vsel)
{
	u32 count, temp;
	u32 irqstat;
	u32 vpconfig_value;
	u32 retry_counter = COUNT_RETRY_SMPSNOACK;

	struct omap_sr_vp *vp = &sr->vp;
	if (!sr_class1p5) {
		temp = prm_read_mod_reg(OMAP3430_GR_MOD,
				vp->prm_vpx_voltage_offset);
		if (current_vsel > target_vsel
		&& temp <= target_vsel
		&& sr->is_autocomp_active)
			goto no_need_forceupdate;
	}


retry_forceupdate:
	/* First clear any pending events in the system */
	if (sr_vp_clear_vptransdone(sr)) {
		pr_crit("SR:forceupdate-transdone1[%d]:timedout\n", sr->srid);
		return -ETIMEDOUT;
	}

	/* Clear initVDD copy trigger bit */
	prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
			       vp->prm_vpx_config_offset);
	/* Clear force bit */
	prm_clear_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
			       vp->prm_vpx_config_offset);

	vpconfig_value = vp->vpconfig_value;
	if (cpu_is_omap3630()) {
		vpconfig_value |= (sr->errgain[target_opp_no - 1]) <<
					OMAP3430_ERRORGAIN_SHIFT;
	} else {
		vpconfig_value |=
		    ((target_opp_no <
		      sr->opp_boundary) ? sr_errgain_lowopp :
			sr_errgain_highopp) << OMAP3430_ERRORGAIN_SHIFT;
	}
	vpconfig_value |= target_vsel << OMAP3430_INITVOLTAGE_SHIFT;

	prm_write_mod_reg(vpconfig_value, OMAP3430_GR_MOD,
			  vp->prm_vpx_config_offset);

	/* Trigger initVDD value copy to voltage processor */
	prm_set_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
			     vp->prm_vpx_config_offset);

	/* Force update of voltage */
	prm_set_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
			     vp->prm_vpx_config_offset);

	/* Now wait for the i2c transactions to complete */
	count = COUNT_TIMEOUT_TRANSDONE_SET;
	irqstat = 0;
	do {
		irqstat = prm_read_mod_reg(OCP_MOD,
					   OMAP3_PRM_IRQSTATUS_MPU_OFFSET) &
		    vp->irqmask_trans_done;
		if (!irqstat) {
			cpu_relax();
			udelay(1);
		}
		count--;
	} while (count && !irqstat);
	if (!count) {
		irqstat = prm_read_mod_reg(OCP_MOD,
					   OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
		if (irqstat & vp->irqmask_smps_noack) {
			WARN(irqstat, "SMPS NO ACK DETECTED[0x%08X]!!"
			     "ATTEMPTING RECOVERY [%d left]\n",
			     irqstat, retry_counter);
			prm_write_mod_reg(irqstat, OCP_MOD,
					  OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
			retry_counter--;
			if (retry_counter)
				goto retry_forceupdate;
		}
		pr_crit("SR:forceupdate-transdone[%d]:timedout-"
			"irqstat=0x%08X\n", sr->srid, irqstat);
		/*BUG();*/
		WARN(1, "SR: Forceupdate 1 BUG!!");
		return -ETIMEDOUT;
	}

	/*
	 * Now we wait for voltage to rise on PMIC
	 */
no_need_forceupdate:
	sr_udelay(omap_pmic_voltage_ramp_delay(sr->srid, target_vsel,
					    current_vsel));

	/* clear that event */
	if (sr_vp_clear_vptransdone(sr)) {
		pr_crit("SR:forceupdate-transdone2[%d]:timedout\n", sr->srid);
		/*BUG();*/
		WARN(1, "SR: Forceupdate 2 BUG!!");
		return -ETIMEDOUT;
	}

	/* Clear initVDD copy trigger bit */
	prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
			       vp->prm_vpx_config_offset);
	/* Clear force bit */
	prm_clear_mod_reg_bits(OMAP3430_FORCEUPDATE, OMAP3430_GR_MOD,
			       vp->prm_vpx_config_offset);

	return 0;
}
#endif				/* ifndef CONFIG_OMAP_VC_BYPASS_UPDATE */

/**
 * @brief sr_vp_enable - enable VP enable code
 *
 * @param sr
 */
static int sr_vp_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 vpconfig_value;
	struct omap_sr_vp *vp = &sr->vp;

	/* Disable VP */
	prm_clear_mod_reg_bits(vp->vpenable_mask, OMAP3430_GR_MOD,
			       vp->prm_vpx_config_offset);
	/* Clear INITVDD */
	prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
			       vp->prm_vpx_config_offset);

	vpconfig_value = vp->vpconfig_value;
	vpconfig_value |= get_vsel_for_opp(sr, target_opp_no) <<
	    OMAP3430_INITVOLTAGE_SHIFT;
	if (cpu_is_omap3630()) {
		vpconfig_value |= (sr->errgain[target_opp_no - 1]) <<
					OMAP3430_ERRORGAIN_SHIFT;
	} else {
		vpconfig_value |=
			    ((target_opp_no <
			      sr->opp_boundary) ? sr_errgain_lowopp :
			      sr_errgain_highopp) << OMAP3430_ERRORGAIN_SHIFT;
	}
	prm_write_mod_reg(vpconfig_value, OMAP3430_GR_MOD,
			  vp->prm_vpx_config_offset);

	prm_write_mod_reg(vp->vpstepmin_value, OMAP3430_GR_MOD,
			  vp->prm_vpx_stepmin_offset);
	prm_write_mod_reg(vp->vpstepmax_value, OMAP3430_GR_MOD,
			  vp->prm_vpx_stepmax_offset);
	prm_write_mod_reg(vp->vplimito_value, OMAP3430_GR_MOD,
			  vp->prm_vpx_limito_offset);

	/* write1 to latch */
	prm_set_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
			     vp->prm_vpx_config_offset);
	/* Enable VP */
	prm_set_mod_reg_bits(vp->vpenable_mask, OMAP3430_GR_MOD,
			     vp->prm_vpx_config_offset);
	/* write2 clear */
	prm_clear_mod_reg_bits(OMAP3430_INITVDD, OMAP3430_GR_MOD,
			       vp->prm_vpx_config_offset);
	return 0;
}

/**
 * @brief sr_vp_disable - disbale Voltage processor
 *
 * @param sr - sr structure
 *
 * @return 0 if all ok, else return -ETIMEDOUT
 */
static int sr_vp_disable(struct omap_sr *sr)
{
	int count;
	u32 v;
	struct omap_sr_vp *vp = &sr->vp;

	v = prm_read_mod_reg(OMAP3430_GR_MOD,
			     vp->prm_vpx_config_offset) & vp->vpenable_mask;
	/* Am i already disabled? */
	if (!v) {
		pr_info("SR[%d] attempt to disable VP when already disabled!\n",
			sr->srid);
		return 0;
	}

	/* Disable VP */
	prm_clear_mod_reg_bits(vp->vpenable_mask, OMAP3430_GR_MOD,
			       vp->prm_vpx_config_offset);

	/* Wait for vp to get idle - clear any events pending */
	count = COUNT_TIMEOUT_MCUDISACK;
	do {
		v = prm_read_mod_reg(OMAP3430_GR_MOD,
				     vp->prm_vpx_status_offset) &
		    PRM_VP_STATUS_VPINIDLE;
		if (!v) {
			cpu_relax();
			udelay(1);
		}
		count--;
	} while (count && !v);
	if (unlikely(!count)) {
		v = prm_read_mod_reg(OCP_MOD, OMAP3_PRM_IRQSTATUS_MPU_OFFSET);
		pr_warning("SR[%d]:vpdisable-vpinidle[opp=%d]:timedout-"
			   "irqstat=0x%08X\n", sr->srid, sr->req_opp_no, v);
		return -ETIMEDOUT;
	}
	return 0;
}

/**
 * @brief sr_vp_configure - configure the basic SR structure
 *
 * @param sr - pointer to SR structure
 */
static void sr_vp_configure(struct omap_sr *sr)
{
	u32 target_opp_no;
	u32 target_vsel;
	u32 prm_vpx_voltage;

	target_opp_no = get_current_opp_number_from_sr(sr);
	target_vsel = get_vsel_for_opp(sr, target_opp_no);
	prm_vpx_voltage = prm_read_mod_reg(OMAP3430_GR_MOD,
					   sr->vp.prm_vpx_voltage_offset);
	sr_vp_enable(sr, target_opp_no);
	if (SR_CHOSEN_VOLTAGE_UPDATE_MECH
	    (sr, target_opp_no, target_vsel, prm_vpx_voltage))
		pr_crit("SR[%d] CONFIGURE VP failed!!\n", sr->srid);
}

/**
 * @brief sr_vp_reset_voltage - reset the voltages back to DVFS values
 *
 * @param srid -SRID
 *
 * @return 0 if ok, else result
 */
static int sr_vp_reset_voltage(u8 srid)
{
	u32 target_opp_no;
	u32 target_vsel;
	u32 prm_vpx_voltage;
	struct omap_sr *sr;

	sr = get_sr(srid);
	target_opp_no = sr->req_opp_no;
	target_vsel = get_vsel_for_opp(sr, target_opp_no);
	prm_vpx_voltage = prm_read_mod_reg(OMAP3430_GR_MOD,
					   sr->vp.prm_vpx_voltage_offset);
	return SR_CHOSEN_VOLTAGE_UPDATE_MECH(sr, target_opp_no,
					     target_vsel, prm_vpx_voltage);

}

/*********************** SR functions *************************/

/**
 * @brief sr_enable - enable smart reflex
 *
 * @param sr - smartreflex structure we are interest
 * @param target_opp_no - target opp we want to switch to
 *
 * @return 0 if all went right, else return err val
 */
static int sr_enable(struct omap_sr *sr, u32 target_opp_no)
{
	u32 value;

	if (!sr->is_sr_reset) {
		pr_info("SR[%d]already enabled\n", sr->srid);
		return -EINVAL;
	}

	sr->req_opp_no = target_opp_no;

	value = sr->opp_nvalue[target_opp_no - 1];
	if (value == 0) {
		pr_info("SR[%d]:OPP%d doesn't support SmartReflex\n",
			sr->srid, target_opp_no);
		return -EINVAL;
	}

	sr_clk_enable(sr);
	/* Start with setting SREnable as 0 */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, 0);

	sr_write_reg(sr, SRCONFIG, sr->sr_config_value);

	sr_write_reg(sr, NVALUERECIPROCAL, value);
	if (cpu_is_omap3630()) {
		value = sr->sr_errconfig_value |
			 (sr->errminlimit[target_opp_no - 1]);
		sr_write_reg(sr, ERRCONFIG_36XX, value);
	} else {
		value = sr->sr_errconfig_value |
			    ((target_opp_no < sr->opp_boundary) ?
				SR_ERRMINLIMIT_LOWOPP :
				SR_ERRMINLIMIT_HIGHOPP);
		sr_write_reg(sr, ERRCONFIG_34XX, value);
	}


	if (cpu_is_omap3630())
		sr_modify_reg(sr, IRQENABLE_SET, MCUBOUNDSINT, MCUBOUNDSINT);

	/* SRCONFIG - enable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, SRCONFIG_SRENABLE);

	return 0;
}

/**
 * @brief sr_disable - disable Smart Reflex
 *
 * @param sr -  pointer to sr structure of interest
 *
 * @return 0 if all went right, else return INVAL/TIMEDOUT
 */
int sr_disable(struct omap_sr *sr)
{
	int count;
	u32 stat;
	u32 value;

	if (sr->is_sr_reset) {
		pr_info("SR[%d]-disable:already Disabled\n", sr->srid);
		return -EINVAL;
	}
	value = sr_read_reg(sr, SRCONFIG) & SRCONFIG_SRENABLE;
	if (!value) {
		pr_info("SR[%d] attempt to disable already disabled SR!\n",
			sr->srid);
		return 0;
	}
	value = sr->opp_nvalue[sr->req_opp_no - 1];
	if (value == 0) {
		pr_info("SR[%d]-disable:"
			"OPP%d doesn't support SmartReflex\n",
			sr->srid, sr->req_opp_no);
		return -EINVAL;
	}
	/* Enable the MCUDISACKINST */
	if (cpu_is_omap3630()) {
		sr_modify_reg(sr, IRQENABLE_SET, MCUDISABLEACKINT,
		      MCUDISABLEACKINT);
	} else {
		sr_modify_reg(sr, ERRCONFIG_34XX, ERRCONFIG_MCUDISACKINTEN |
			      ERRCONFIG_34XX_INTERRUPT_STATUS_MASK,
			      ERRCONFIG_MCUDISACKINTEN);
	}

	/* SRCONFIG - disable SR */
	sr_modify_reg(sr, SRCONFIG, SRCONFIG_SRENABLE, 0);

	/* Disable VPBOUND interrupt and clear any status
	 * before actually waiting for disack
	 * should be done after sr is disabled
	 */
	if (cpu_is_omap3630()) {
		sr_modify_reg(sr, ERRCONFIG_36XX, ERRCONFIG_34XX_VPBOUNDINTEN |
		      ERRCONFIG_36XX_INTERRUPT_STATUS_MASK,
		      ERRCONFIG_36XX_VPBOUNDINTST);
		sr_write_reg(sr, IRQENABLE_CLR, MCUBOUNDSINT);
		sr_write_reg(sr, IRQSTATUS, MCUBOUNDSINT);
	} else {
		sr_modify_reg(sr, ERRCONFIG_34XX,
				ERRCONFIG_34XX_INTERRUPT_STATUS_MASK,
				ERRCONFIG_34XX_VPBOUNDINTST);

	}


	/* Wait for MCUDISACKINTST to be set */
	count = COUNT_TIMEOUT_MCUDISACK;
	do {
		if (cpu_is_omap3630()) {
			stat = sr_read_reg(sr, IRQSTATUS) & MCUDISABLEACKINT;
		} else {
			stat = sr_read_reg(sr, ERRCONFIG_34XX) &
			ERRCONFIG_MCUDISACKINTST;
		}
		if (!stat) {
			cpu_relax();
			udelay(1);
		}
		count--;
	} while (count && !stat);
	/* Clear the event and disable MCUDISACKINST */
	if (cpu_is_omap3630()) {
		sr_write_reg(sr, IRQSTATUS, MCUDISABLEACKINT);
		sr_write_reg(sr, IRQENABLE_CLR, MCUDISABLEACKINT);
	} else {
		sr_modify_reg(sr, ERRCONFIG_34XX, ERRCONFIG_MCUDISACKINTEN |
			      ERRCONFIG_34XX_INTERRUPT_STATUS_MASK,
			      ERRCONFIG_MCUDISACKINTST);

	}

	if (!count) {
		pr_crit("SR[%d]-disable:MCUDIS timedout\n", sr->srid);
		return -ETIMEDOUT;
	}
	sr_clk_disable(sr);
	return 0;
}

/**************** Common enable/disable functionality *********************/

/**
 * @brief srvp_disable - disable SR and VP
 * These functions are based on h/w timeouts and should ideally not fail.
 *
 * @param sr -sr struct
 *
 * @return  result
 */
static int srvp_disable(struct omap_sr *sr)
{
	int ret;
	/* If we dont have an nvalue, dont bother.. */
	if (!sr->opp_nvalue[sr->req_opp_no - 1]) {
		pr_warning("SR[%d]: OPP%d does not support SR to disable\n",
			   sr->srid, sr->req_opp_no);
		return -EINVAL;
	}
	ret = sr_vp_disable(sr);
	if (unlikely(ret)) {
		pr_err("SR[%d]: failed to disable vp:%d\n", sr->srid, ret);
	} else {
		ret = sr_disable(sr);
		if (unlikely(ret))
			pr_err("SR[%d]: failed to disable sr:%d\n",
			       sr->srid, ret);
	}
	/*
	 * Does not make much sense renabling SR as
	 * system is going to be in an invalid state
	 */
	WARN_ON(ret);
	return ret;
}

/**
 * @brief srvp_enable - enable SR and VP
 * These functions are based on h/w timeouts and should ideally not fail.
 *
 * @param sr -sr struct
 * @param target_opp -opp to go to
 *
 * @return result
 */
static int srvp_enable(struct omap_sr *sr, u32 target_opp)
{
	int ret;

	/* If we dont have an nvalue, dont bother.. */
	if (!sr->opp_nvalue[target_opp - 1]) {
		pr_warning("SR[%d]: OPP%d does not support SR to enable\n",
			   sr->srid, target_opp);
		return -EINVAL;
	}
	ret = sr_vp_enable(sr, target_opp);
	if (unlikely(ret)) {
		pr_err("SR[%d]: failed to enable vp:%d\n", sr->srid, ret);
	} else {
		ret = sr_enable(sr, target_opp);
		/* Attempt to recover */
		if (unlikely(ret)) {
			pr_err("SR[%d]: failed to enable sr:%d\n", sr->srid,
			       ret);
			/* nothing we can do if vp_disable fails */
			(void)sr_vp_disable(sr);
		}
	}
	/*
	 * potentially system in an invalid state - warn..
	 */
	WARN_ON(ret);
	return ret;
}

/*********************** DVFS Entry POINTS **********************************/
#define SR_CLASS1P5_LOOP_US	100
#define MAX_STABILIZATION_COUNT 100
#define MAX_LOOP_COUNT		(MAX_STABILIZATION_COUNT * 5)
#define ADJUSTED_VSEL_MARGIN_IN_STEP           3
#define ADJUSTED_VSEL_MARGIN_IN_STEP_1_2G      2
#define MAX_ADJUSTED_VSEL      0x42    /* MAX volt limit to 1.4275V */
int sr_recalibrate(struct omap_opp *opp, u32 t_opp, u32 c_opp)
{
	u32 max_loop_count = MAX_LOOP_COUNT;
	u32 exit_loop_on = 0;
	u32 vdd, target_opp_no;
	u8 new_v = 0;
	u8 high_v = 0;
	u16 current_vsel;
	int ret;
	struct omap_sr_vp *vp;
	struct omap_sr *sr;

	vdd = get_vdd(t_opp);
	target_opp_no = get_opp_no(t_opp);
	sr = get_sr_from_vdd(vdd);
	vp = &sr->vp;

	pr_debug("Calibrate: Entry %s %d:%d %d %d\n", __func__, vdd,
		target_opp_no, sr->is_autocomp_active, sr->is_sr_reset);

	if (!sr->is_autocomp_active) {
		pr_err("%s: %d:%d autocomp was disabled\n", __func__, vdd,
				target_opp_no);
		return 0;
	}
	/* Save Current vsel */
	current_vsel =  prm_read_mod_reg(OMAP3430_GR_MOD,
				vp->prm_vpx_voltage_offset);
	/* Start Smart reflex */
	sr_vp_enable_both(t_opp, c_opp);
	/* We need to wait for SR to stabilize before we start sampling */
	sr_udelay(MAX_STABILIZATION_COUNT * SR_CLASS1P5_LOOP_US);

	/* Ready for recalibration */
	while (max_loop_count) {
		new_v = prm_read_mod_reg(OMAP3430_GR_MOD,
				vp->prm_vpx_voltage_offset);

		/* handle oscillations */
		if (new_v != high_v) {
			high_v = (high_v < new_v) ? new_v : high_v;
			exit_loop_on = MAX_STABILIZATION_COUNT;
		}
		/* wait for one more stabilization loop for us to sample */
		sr_udelay(SR_CLASS1P5_LOOP_US);

		max_loop_count--;
		exit_loop_on--;
		/* Stabilization achieved.. quit */
		if (!exit_loop_on)
			break;
	}
	/*
	 * bad case where we are oscillating.. flag it,
	 * but continue with higher v
	 */
	if (!max_loop_count && exit_loop_on) {
		pr_err("%s: %d:%d exited with voltages 0x%02x 0x%02x\n",
			__func__, vdd, target_opp_no, new_v, high_v);
	}

	opp[target_opp_no].sr_nval = sr_read_reg(sr, SENVAL);
	if (cpu_is_omap3630())
		opp[target_opp_no].sr_err = sr_read_reg(sr, SENERROR_36XX);
	else
		opp[target_opp_no].sr_err = sr_read_reg(sr, SENERROR_34XX);
	/* Stop Smart reflex */
	sr_vp_disable_both(t_opp, c_opp);

	/*
	 * Add 3-Steps margin to the adjusted vsel
	 * Set max volt limit to 1.4375 volt
	 */
	switch (omap_rev_id()) {
	case OMAP_3630:
	default:
		if (omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP4_VDD1) != 0)
			high_v += ADJUSTED_VSEL_MARGIN_IN_STEP;
		break;
	case OMAP_3630_1200:
		if (omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP5_VDD1) != 0) {
			if ((VDD1_OPP == vdd) && (target_opp_no == VDD1_OPP5))
				high_v += ADJUSTED_VSEL_MARGIN_IN_STEP_1_2G;
			else
				high_v += ADJUSTED_VSEL_MARGIN_IN_STEP;
		}
	}

	if (high_v  > MAX_ADJUSTED_VSEL)
		high_v = MAX_ADJUSTED_VSEL;

	opp[target_opp_no].sr_adjust_vsel = high_v;

	/* ForceUpdate sr adjust vsel */
	ret =
		SR_CHOSEN_VOLTAGE_UPDATE_MECH(sr, target_opp_no, high_v,
					current_vsel);
	pr_debug("Calibrate:Exit %s [vdd%d: opp%d] %02x->%02x loops=[%d,%d]\n",
		__func__, vdd, target_opp_no, current_vsel, high_v,
		max_loop_count, exit_loop_on);

	return 0;
}

/**
 * @brief sr_vp_enable_both - enable both vp and sr
 *
 * @param target_opp - targetted op
 * @param current_opp - current opp
 *
 * @return 0 if ok, 1 if not ok
 */
int sr_vp_enable_both(u32 target_opp, u32 current_opp)
{
	struct omap_sr *sr;
	u32 vdd, target_opp_no;
	int ret = 0;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);
	sr = get_sr_from_vdd(vdd);

	if (sr->is_autocomp_active && sr->is_sr_reset) {
		ret = srvp_enable(sr, target_opp_no);
		if (ret) {
			pr_err("SR[%d]:enableboth:"
			       "failed enable SR\n", sr->srid);
		}
	}
	return ret;
}
EXPORT_SYMBOL(sr_vp_enable_both);

/**
 * @brief sr_vp_disable_both - disable both vp and sr
 *
 * @param target_opp - targetted opp
 * @param current_opp - current opp
 *
 * @return 0 if ok, 1 if not ok
 */
int sr_vp_disable_both(u32 target_opp, u32 current_opp)
{
	struct omap_sr *sr;
	u32 vdd;
	int ret = 0;

	vdd = get_vdd(target_opp);
	sr = get_sr_from_vdd(vdd);
	sr->last_opp = get_opp_no(current_opp);;
	if (sr->is_autocomp_active && !sr->is_sr_reset) {
		ret = srvp_disable(sr);
		if (ret) {
			pr_err("SR[%d]:disableboth:"
			       "failed disable SR\n", sr->srid);
		}
	}

	return ret;

}
EXPORT_SYMBOL(sr_vp_disable_both);

/**
 * @brief sr_voltage_set - setup a voltage requested
 *
 * @param target_opp - targetted opp
 * @param current_opp  - current opp
 * @param target_vsel - targeted voltage
 * @param current_vsel - current voltage
 *
 * @return  - result of op -0 if ok, else value
 */

int sr_voltage_set(u32 target_opp, u32 current_opp,
		   u8 target_vsel, u8 current_vsel)
{
	struct omap_sr *sr;
	u8 vdd, target_opp_no, current_opp_no;
	int ret;

	vdd = get_vdd(target_opp);
	target_opp_no = get_opp_no(target_opp);
	current_opp_no = get_opp_no(current_opp);

	sr = get_sr_from_vdd(vdd);

	/* For OMAP3630 only:
	 * Set ABB bypass mode when leaving 1G opp.
	 */
	if (cpu_is_omap3630() && (VDD1_OPP == vdd))
		if (target_opp_no <= VDD1_OPP3 &&
			current_opp_no >= VDD1_OPP4)
			omap3630_abb_change_active_opp(target_opp_no);

	ret =
	    SR_CHOSEN_VOLTAGE_UPDATE_MECH(sr, target_opp_no, target_vsel,
					  current_vsel);

	/* For OMAP3630 only:
	 * Enable ABB FBB mode when going to 1Ghz OPP,
	 */
	if (cpu_is_omap3630() && (VDD1_OPP == vdd))
		if (target_opp_no >= VDD1_OPP4 &&
			current_opp_no <= VDD1_OPP3)
			omap3630_abb_change_active_opp(target_opp_no);

	return ret;
}
EXPORT_SYMBOL(sr_voltage_set);

/*********************** CPUIDLE ENTRY POINTS *********************************/

/**
 * @brief disable_smartreflex - disable SmartReflex before WFI
 *
 * @param srid SRID
 */
void disable_smartreflex(u8 srid)
{
	struct omap_sr *sr = NULL;
	int ret;

	/* Dont do anything in SR Class 1.5 */
	if (sr_class1p5)
		return;

	/* I want to be in irq_disabled context..
	 * else I will die.. find the rootcause and fix it instead
	 */
	BUG_ON(!irqs_disabled());

	sr = get_sr(srid);

	if (sr->is_autocomp_active && !sr->is_sr_reset) {
		sr->last_opp = 0;
		sr->req_opp_no = get_current_opp_number_from_sr(sr);
		ret = srvp_disable(sr);
		if (ret)
			pr_err("SR[%d]:disable_smartreflex:"
			       "failed disable SR\n", sr->srid);
	}
}
EXPORT_SYMBOL(disable_smartreflex);

/**
 * @brief enable_smartreflex - enable smart reflex after WFI is hit
 *
 * @param srid -SR ID to hit
 */
void enable_smartreflex(u8 srid)
{
	struct omap_sr *sr;
	int ret;

	/* Dont do anything in SR Class 1.5 */
	if (sr_class1p5)
		return;
	/* I want to be in irq_disabled context..
	 * else I will die.. find the rootcause and fix it instead
	 */
	BUG_ON(!irqs_disabled());

	sr = get_sr(srid);

	if (sr->is_autocomp_active && sr->is_sr_reset) {
		ret = srvp_enable(sr, get_current_opp_number_from_sr(sr));
		if (ret)
			pr_err("SR[%d]:enable_smartreflex:"
			       "failed enable SR\n", sr->srid);
	}
}
EXPORT_SYMBOL(enable_smartreflex);

/*********************** SYSFS ENTRY POINTS *********************************/
/**
 * @brief omap_sr_vdd_autocomp_show - Sysfs entry for showing SR status
 *
 */
static ssize_t omap_sr_vdd_autocomp_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	struct omap_sr *sr;
	sr = get_sr_from_vdd_name((char *)attr->attr.name);
	return sprintf(buf, "%d\n", sr->is_autocomp_active);
}

/**
 * @brief omap_sr_vdd_autocomp_store - enable/disable SR
 *
 */
static ssize_t omap_sr_vdd_autocomp_store(struct kobject *kobj,
					  struct kobj_attribute *attr,
					  const char *buf, size_t n)
{
	unsigned short value;
	struct omap_sr *sr;
	int ret = 0;
	char *name;

	name = (char *)attr->attr.name;

	sr = get_sr_from_vdd_name(name);

	if (sscanf(buf, "%hu", &value) != 1 || (value > 1)) {
		pr_err("%s: Invalid value[%d]. Use 0 or 1\n", name, value);
		return -EINVAL;
	}
	if (sr->is_autocomp_active == value) {
		pr_info("%s: Already set to %d \n", name, value);
		return n;
	}

	/* dont do anything in SR Class 1.5 */
	if (!sr_class1p5) {
		/*
		 * Sanity check-might happen if SR and dvfs/idle paths collide
		 * but unlikely though..
		 */
		if (unlikely(value ^ sr->is_sr_reset)) {
			pr_warning("%s: Is already set %d Vs %d.\n", name,
				value, sr->is_sr_reset);
			return n;
		}

		if (value) {
			u32 current_vddopp_no =
				get_current_opp_number_from_sr(sr);
			ret = srvp_enable(sr, current_vddopp_no);
		} else {
			ret = srvp_disable(sr);
			/* reset the voltage back to nominal */
			sr_vp_configure(sr);
		}
	}
	if (!ret) {
		sr->is_autocomp_active = value;
		ret = n;
	}

	return ret;
}

/*********************** INIT FUNCTIONS *************************************/

/**
 * @brief srvp_init - configure smart reflex and VP params
 *
 * @param sr  - structure of sr of interest
 */
void ComputeNadj(u32 *Sen_RN, u32 *Sen_Gain, u32 SenGainfuse,
		 u32 SenRNfuse, int Delta)
{
	u32 Nadj, R, G;

	Nadj = ((1 << (SenGainfuse + 8)) / SenRNfuse) + Delta;
	for (G = 0; G < 16; G++) {
		R = (1 << (G+8))/Nadj;
		if (R < 256) {
			*Sen_RN = R;
			*Sen_Gain = G;
		}
	}
}
u32 CalculateRG(u32 NNT_Delta, u32 PNT_Delta)
{
	u32 OPP5_NValue = omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP5_VDD1);
	u32 SenPGain_fuse = (OPP5_NValue & 0x00F00000) >> 0x14;
	u32 SenNGain_fuse = (OPP5_NValue & 0x000F0000) >> 0x10;
	u32 SenPRN_fuse = (OPP5_NValue & 0x0000FF00) >> 0x08;
	u32 SenNRN_fuse = (OPP5_NValue & 0x000000FF);
	u32 SenNRN = 0, SenNGain = 0, SenPRN = 0, SenPGain = 0;

	ComputeNadj(&SenNRN, &SenNGain, SenNGain_fuse, SenNRN_fuse, NNT_Delta);
	ComputeNadj(&SenPRN, &SenPGain, SenPGain_fuse, SenPRN_fuse, PNT_Delta);

	return (SenPGain << 0x14) | (SenNGain << 0x10) |
		(SenPRN << 0x08) | (SenNRN);

}

u32 ApplyAdj(u32 NValue, int NNT_Delta, int PNT_Delta)
{
	u32 SenPGain_fuse = (NValue & 0x00F00000) >> 0x14;
	u32 SenNGain_fuse = (NValue & 0x000F0000) >> 0x10;
	u32 SenPRN_fuse = (NValue & 0x0000FF00) >> 0x08;
	u32 SenNRN_fuse = (NValue & 0x000000FF);

	u32 SenNRN = 0, SenNGain = 0;
	u32 SenPRN = 0, SenPGain = 0;

	ComputeNadj(&SenNRN, &SenNGain, SenNGain_fuse, SenNRN_fuse, NNT_Delta);
	ComputeNadj(&SenPRN, &SenPGain, SenPGain_fuse, SenPRN_fuse, PNT_Delta);

	return (SenPGain << 0x14) | (SenNGain << 0x10) |
		(SenPRN << 0x08) | (SenNRN);
}

static void sr1_init(struct omap_sr *sr)
{
	int i;

	sr->num_opp = omap_pm_get_max_vdd1_opp();
	if (cpu_is_omap3630()) {
		sr->sr_errconfig_value |= ERRCONFIG_36XX_VPBOUNDINTEN |
					ERRCONFIG_36XX_VPBOUNDINTST;
		if (omap_rev_id() == OMAP_3630_1200) {
			sr->opp_nvalue[4] =
			omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP5_VDD1);
				sr->opp_nvalue[4] = ApplyAdj(
					sr->opp_nvalue[4],
					3.0*12.5*NUM_STEPS_MARGIN_1_2G,
					2.6*12.5*NUM_STEPS_MARGIN_1_2G);
		}
		sr->opp_nvalue[3] =
		   omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP4_VDD1);
		if (sr->opp_nvalue[3] != 0x0) {
			pr_info("SR1 : Fused Nvalues for %d OPP\n",
							sr->num_opp);
			sr->opp_nvalue[3] = ApplyAdj(
				sr->opp_nvalue[3],
				3.0*12.5*NUM_STEPS_MARGIN_1_0G,
				2.6*12.5*NUM_STEPS_MARGIN_1_0G);
			sr->opp_nvalue[2] =
			   omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP3_VDD1);
			sr->opp_nvalue[1] =
			   omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP2_VDD1);
			sr->opp_nvalue[0] =
			   omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP1_VDD1);
		} else {
			pr_info("SR1 : Testing Nvalues for %d OPP\n",
							sr->num_opp);
			memcpy(&sr->opp_nvalue, &sr1_36xx_test_nvalues,
				sizeof(sr1_36xx_test_nvalues));

			for (i = 0; i < sr->num_opp; i++)
				sr->opp_nvalue[i] = ApplyAdj(
					sr->opp_nvalue[i],
					3.0*12.5*NUM_STEPS_MARGIN,
					2.6*12.5*NUM_STEPS_MARGIN);
		}

		sr->errminlimit[0] = SR1_36XX_ERRMINLIMIT_OPP1;
		sr->errminlimit[1] = SR1_36XX_ERRMINLIMIT_OPP2;
		sr->errminlimit[2] = SR1_36XX_ERRMINLIMIT_OPP3;
		sr->errminlimit[3] = SR1_36XX_ERRMINLIMIT_OPP4;
		sr->errminlimit[4] = SR1_36XX_ERRMINLIMIT_OPP5;
		sr->errgain[0] = SR1_36XX_ERRGAIN_OPP1;
		sr->errgain[1] = SR1_36XX_ERRGAIN_OPP2;
		sr->errgain[2] = SR1_36XX_ERRGAIN_OPP3;
		sr->errgain[3] = SR1_36XX_ERRGAIN_OPP4;
		sr->errgain[4] = SR1_36XX_ERRGAIN_OPP5;

		if (omap_rev_id() == OMAP_3630_1200) {
			if (sr->opp_nvalue[4]) {
				pr_info("VDD1 - OPP5 -  1.2G Nvalue = 0x%X\n",
					sr->opp_nvalue[4]);
			} else
				pr_info("VDD1 - OPP5 -  Not Fused for \
					 OMAP3630 1.2G Chip.\n");
		}
		pr_info("VDD1 - OPP4 -  1G Nvalue = 0x%X\n",
				sr->opp_nvalue[3]);
		pr_info("VDD1 - OPP3 - 800 Nvalue = 0x%X\n",
				sr->opp_nvalue[2]);
		pr_info("VDD1 - OPP2 - 600 Nvalue = 0x%X\n",
				sr->opp_nvalue[1]);
		pr_info("VDD1 - OPP1 - 300 Nvalue = 0x%X\n",
				sr->opp_nvalue[0]);

	} else { /* if (cpu_is_omap3630()) */
		sr->sr_errconfig_value |= ERRCONFIG_34XX_VPBOUNDINTEN |
					  ERRCONFIG_34XX_VPBOUNDINTST |
					  SR_CLKACTIVITY_IOFF_FON;
		sr->opp_nvalue[0] =
		   omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP1_VDD1);
		if (sr->opp_nvalue[0] != 0x0) {
			pr_info("SR1 : Fused Nvalues for %d OPP\n",
							sr->num_opp);
			if (sr->num_opp == 6)
				sr->opp_nvalue[5] = CalculateRG(730, 434);

#if defined(SR_HIGHSPEED) || defined(SR_TURBO)
			sr->opp_nvalue[4] = CalculateRG(379, 227);
#else /* #if defined(SR_HIGHSPEED) || defined(SR_TURBO) */
			sr->opp_nvalue[4] =
			   omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP5_VDD1);
#endif /* #if defined(SR_HIGHSPEED) || defined(SR_TURBO) */
			sr->opp_nvalue[3] =
			   omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP4_VDD1);
			sr->opp_nvalue[2] =
			   omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP3_VDD1);
			sr->opp_nvalue[1] =
			   omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP2_VDD1);

		} else {
			pr_info("SR1 : Testing Nvalues for %d OPP\n",
					sr->num_opp);
			memcpy(&sr->opp_nvalue, &sr1_34xx_test_nvalues,
					sizeof(sr1_34xx_test_nvalues));
		}
	if (sr->num_opp == 6)
		pr_info("VDD1 - OPP6 - 800 Nvalue = 0x%X\n",
					sr->opp_nvalue[5]);

#if defined(SR_HIGHSPEED) || defined(SR_TURBO)
	pr_info("VDD1 - OPP5 - 720 Nvalue = 0x%X\n",
			sr->opp_nvalue[4]);
#else /* #if defined(SR_HIGHSPEED) || defined(SR_TURBO) */
	pr_info("VDD1 - OPP5 - 600 Nvalue = 0x%X\n",
			sr->opp_nvalue[4]);
#endif /* #if defined(SR_HIGHSPEED) || defined(SR_TURBO) */
	pr_info("VDD1 - OPP4 - 550 Nvalue = 0x%X\n",
			sr->opp_nvalue[3]);
	pr_info("VDD1 - OPP3 - 500 Nvalue = 0x%X\n",
			sr->opp_nvalue[2]);
	pr_info("VDD1 - OPP2 - 250 Nvalue = 0x%X\n",
			sr->opp_nvalue[1]);
	pr_info("VDD1 - OPP1 - 125 Nvalue = 0x%X\n",
			sr->opp_nvalue[0]);
	} /* if (cpu_is_omap3630()) */
}

static void sr2_init(struct omap_sr *sr)
{
	sr->num_opp = omap_pm_get_max_vdd2_opp();
	if (cpu_is_omap3630()) {
		sr->sr_errconfig_value |= ERRCONFIG_36XX_VPBOUNDINTEN |
					  ERRCONFIG_36XX_VPBOUNDINTST;

		if (omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP4_VDD1) != 0x0) {
			pr_info("SR2 : Fused Nvalues for %d OPP\n",
							sr->num_opp);
			sr->opp_nvalue[0] =
			   omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP1_VDD2);
			sr->opp_nvalue[1] =
			   omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP2_VDD2);
		} else {
			pr_info("SR2 : Testing Nvalues for %d OPP\n",
							sr->num_opp);
			memcpy(&sr->opp_nvalue, &sr2_36xx_test_nvalues,
					sizeof(sr2_36xx_test_nvalues));
		}

		sr->errminlimit[0] = SR2_36XX_ERRMINLIMIT_OPP1;
		sr->errminlimit[1] = SR2_36XX_ERRMINLIMIT_OPP2;
		sr->errgain[0] = SR_ERRGAIN_LOWOPP;
		sr->errgain[1] = SR_ERRGAIN_HIGHOPP;
		pr_info("VDD2 - OPP2 - 200 Nvalue = 0x%X\n",
				sr->opp_nvalue[1]);
		pr_info("VDD2 - OPP1 - 100 Nvalue = 0x%X\n",
				sr->opp_nvalue[0]);

	} else { /* if (cpu_is_omap3630()) */
		sr->sr_errconfig_value |= ERRCONFIG_34XX_VPBOUNDINTEN |
					  ERRCONFIG_34XX_VPBOUNDINTST |
					  SR_CLKACTIVITY_IOFF_FON;
		sr->opp_nvalue[0] =
		   omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP1_VDD2);
		if (sr->opp_nvalue[0] != 0x0) {
			pr_info("SR2 : Fused Nvalues for %d OPP\n",
							sr->num_opp);
			sr->opp_nvalue[2] =
			   omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP3_VDD2);
			sr->opp_nvalue[1] =
			   omap_ctrl_readl(OMAP343X_CONTROL_FUSE_OPP2_VDD2);
		} else {
			pr_info("SR2 : Testing Nvalues for %d OPP\n",
							sr->num_opp);
			memcpy(&sr->opp_nvalue, &sr2_34xx_test_nvalues,
					sizeof(sr2_34xx_test_nvalues));
		}
	pr_info("VDD2 - OPP3 - 160 Nvalue = 0x%X\n",
			sr->opp_nvalue[2]);
	pr_info("VDD2 - OPP2 - 080 Nvalue = 0x%X\n",
			sr->opp_nvalue[1]);
	pr_info("VDD2 - OPP1 - 000 Nvalue = 0x%X\n",
			sr->opp_nvalue[0]);
	} /* if (cpu_is_omap3630()) */

}

static void __init srvp_init(struct omap_sr *sr)
{
	struct clk *sys_ck;
	u32 sys_clk_speed;
	u32 clk_len;
	u32 senn_mod, senp_mod;

	/* Grab the clock speed */
	sys_ck = clk_get(NULL, "sys_ck");
	sys_clk_speed = clk_get_rate(sys_ck);
	clk_put(sys_ck);

	switch (sys_clk_speed) {
	case 12000000:
		clk_len = SRCLKLENGTH_12MHZ_SYSCLK;
		break;
	case 13000000:
		clk_len = SRCLKLENGTH_13MHZ_SYSCLK;
		break;
	case 19200000:
		clk_len = SRCLKLENGTH_19MHZ_SYSCLK;
		break;
	case 26000000:
		clk_len = SRCLKLENGTH_26MHZ_SYSCLK;
		break;
	case 38400000:
		clk_len = SRCLKLENGTH_38MHZ_SYSCLK;
		break;
	default:
		pr_err("SR[%d]:Invalid sysclk value: %d\n", sr->srid,
			sys_clk_speed);
		/*BUG();*/
		WARN(1, "SR: srvp_configure BUG!!");
		return;
	}

	if (sr->srid == SR1) {
		sr1_init(sr);
		sr->vp.vplimito_value |= sr_vplimito_value(sys_clk_speed,
				PRM_VP1_VLIMITTO_TIMEOUT_US) <<
				PRM_VP1_VLIMITTO_TIMEOUT_SHIFT;
	} else {
		sr2_init(sr);
		sr->vp.vplimito_value |= sr_vplimito_value(sys_clk_speed,
			PRM_VP2_VLIMITTO_TIMEOUT_US) <<
			PRM_VP2_VLIMITTO_TIMEOUT_SHIFT;
	}
	if (cpu_is_omap3630()) {
		sr->sr_config_value =
			(clk_len << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_ERRGEN_EN | SRCONFIG_SENENABLE |
			(SEN_ENABLE << SRCONFIG_36XX_SENNENABLE_SHIFT) |
			(SEN_ENABLE << SRCONFIG_36XX_SENPENABLE_SHIFT);
	} else {
		if (omap_ctrl_readl(OMAP36XX_CONTROL_FUSE_OPP1_VDD2) != 0x0) {
			senn_mod =
				(omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
				OMAP343X_SR2_SENNENABLE_MASK) >>
				OMAP343X_SR2_SENNENABLE_SHIFT;
			senp_mod =
				(omap_ctrl_readl(OMAP343X_CONTROL_FUSE_SR) &
				OMAP343X_SR2_SENPENABLE_MASK) >>
				OMAP343X_SR2_SENPENABLE_SHIFT;
		} else {
			senn_mod = SR_N_MOD;
			senp_mod = SR_P_MOD;
		}
		sr->sr_config_value =
			(clk_len << SRCONFIG_SRCLKLENGTH_SHIFT) |
			SRCONFIG_ERRGEN_EN | SRCONFIG_SENENABLE |
			(senn_mod << SRCONFIG_34XX_SENNENABLE_SHIFT) |
			(senp_mod << SRCONFIG_34XX_SENPENABLE_SHIFT) |
			SRCONFIG_DELAYCTRL;
	}

	/* Set up nominal voltage */
	sr_vp_configure(sr);
}

/**
 * @brief omap_sr_init - SR initialization
 *
 * @return  0
 */
static int __init omap_sr_init(void)
{
	int ret = 0, i;
	ret = omap_pmic_srinit();
	if (ret) {
		pr_crit("PMIC init failed during SmartReflex initialization."
			"connectivity issues?: %d\n", ret);
		return ret;
	}

#if defined(CONFIG_OMAP_SMARTREFLEX_TESTING_NVALUES) || \
	defined(CONFIG_OMAP_36XX_SMARTREFLEX_TESTING_NVALUES)
	pr_err("Warning: This build uses SmartReflex Testing NVALUES!!..\n");
#endif
	/* Create the userspace control knobs */
	for (i = 0; i < 2; i++) {
		struct omap_sr *sr = get_sr(i + 1);
		sr->fclk = clk_get(NULL, sr->fclk_name);
		sr->iclk = clk_get(NULL, sr->iclk_name);
		sr->srbase_addr = ioremap((u32) sr->srbase_addr, SZ_4K);
		srvp_init(sr);
		if (sysfs_create_file(power_kobj, &sr->autocom_attr.attr))
			pr_warning("SR: sysfs_create_file failed[%d]: %d\n", i,
				   ret);
	}

	if (sr_debugfs_create_entries_late())
		pr_warning("SR: debugfs_create_file failed\n");

	pr_info("SmartReflex driver initialized\n");

	return 0;
}
late_initcall(omap_sr_init);
