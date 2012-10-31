/*
 * pmic-cpcap.c - CPCAP-specific functions for the OPP code
 *
 * Copyright (C) 2011 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */


#include <linux/init.h>
#include <linux/kernel.h>
#include "pm.h"
#include "vc.h"

/**
 * omap_cpcap_vsel_to_vdc - convert CPCAP VSEL value to microvolts DC
 * @vsel: CPCAP VSEL value to convert
 *
 * Returns the microvolts DC that the CPCAP PMIC should generate when
 * programmed with @vsel.
 */
unsigned long omap_cpcap_vsel_to_uv(unsigned char vsel)
{
	if (vsel > 0x44)
		vsel = 0x44;
	return (((vsel * 125) + 6000)) * 100;
}

/**
 * omap_cpcap_uv_to_vsel - convert microvolts DC to CPCAP VSEL value
 * @uv: microvolts DC to convert
 *
 * Returns the VSEL value necessary for the CPCAP PMIC to
 * generate an output voltage equal to or greater than @uv microvolts DC.
 */
unsigned char omap_cpcap_uv_to_vsel(unsigned long uv)
{
	if (uv < 600000)
		uv = 600000;
	else if (uv > 1450000)
		uv = 1450000;
	return DIV_ROUND_UP(uv - 600000, 12500);
}

/* For CPCAP CORE */
static struct omap_voltdm_pmic omap_cpcap_core = {
	.slew_rate		= 4000,
	.step_size		= 12500,
	.on_volt		= 1300000,
	.onlp_volt		= 1300000,
	.ret_volt		= 837500,
	.off_volt		= 600000,
	.volt_setup_time	= 0,
	.switch_on_time		= 1000,
	.vp_erroroffset		= OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin		= 900000,
	.vp_vddmax		= 1350000,
	.vp_timeout_us		= 0x200,
	.i2c_slave_addr		= 0x02,
	.volt_reg_addr		= 0x00,
	.cmd_reg_addr		= 0x01,
	.i2c_high_speed		= false,
	.i2c_scll_low		= 0x60,
	.i2c_scll_high		= 0x26,
	.i2c_hscll_low		= 0x0B,
	.i2c_hscll_high		= 0x00,
	.vsel_to_uv		= omap_cpcap_vsel_to_uv,
	.uv_to_vsel		= omap_cpcap_uv_to_vsel,
};

/* For CPCAP IVA */

static struct omap_voltdm_pmic omap_cpcap_iva = {
	.slew_rate              = 4000,
	.step_size              = 12500,
	.on_volt                = 1125000,
	.onlp_volt              = 1125000,
	.ret_volt               = 837500,
	.off_volt               = 600000,
	.volt_setup_time        = 0,
	.switch_on_time         = 1000,
	.vp_erroroffset         = OMAP4_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin            = OMAP4_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax            = OMAP4_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin              = 900000,
	.vp_vddmax              = 1350000,
	.vp_timeout_us          = 0x200,
	.i2c_slave_addr         = 0x44,
	.volt_reg_addr          = 0x0,
	.cmd_reg_addr           = 0x01,
	.i2c_high_speed         = false,
	.i2c_scll_low           = 0x60,
	.i2c_scll_high          = 0x26,
	.i2c_hscll_low          = 0x0B,
	.i2c_hscll_high         = 0x00,
	.vsel_to_uv             = omap_cpcap_vsel_to_uv,
	.uv_to_vsel             = omap_cpcap_uv_to_vsel,
};


static __initdata struct omap_pmic_map cpcap_map[] = {
	{
		.name = "core",
		.omap_chip = OMAP_CHIP_INIT(CHIP_IS_OMAP3XXX),
		.pmic_data = &omap_cpcap_core,
	},
	{
		.name = "mpu_iva",
		.omap_chip = OMAP_CHIP_INIT(CHIP_IS_OMAP3XXX),
		.pmic_data = &omap_cpcap_iva,
	},
	/* Terminator */
	{	.name = NULL, .pmic_data = NULL},
};

static __initdata struct omap_pmic_description cpcap_desc = {
	.pmic_lp_tshut = 500,	/* T-OFF */
	.pmic_lp_tstart = 500,	/* T-ON */
};

int __init omap_cpcap_init(void)
{
	printk(" __init omap_cpcap_init(void)\n");
	return omap_pmic_register_data(cpcap_map, &cpcap_desc);
}
