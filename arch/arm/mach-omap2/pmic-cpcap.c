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
#include "voltage.h"

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
	return ((((vsel - 1) * 1260) + 60770)) * 10;
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
	printk("Uv:%ld, Set vsel: %ld\n",uv,DIV_ROUND_UP(uv - 607700, 12600) + 1);
	return DIV_ROUND_UP(uv - 607700, 12600) + 1;
}

/* For CPCAP CORE */
static struct omap_voltdm_pmic omap_cpcap_core = {
	.slew_rate		= 4000,
	.step_size		= 12500,
	.on_volt		= 1200000,
	.onlp_volt		= 1000000,
	.ret_volt		= 975000,
	.off_volt		= 600000,
	.volt_setup_time	= 0xfff,
	.switch_on_time		= 1000,
	.vp_erroroffset		= OMAP3_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP3_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP3_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin		= OMAP3630_VP2_VLIMITTO_VDDMIN,
	.vp_vddmax		= OMAP3630_VP2_VLIMITTO_VDDMAX,
	.vp_timeout_us		= OMAP3_VP_VLIMITTO_TIMEOUT_US,
	.i2c_slave_addr		= OMAP3_SRI2C_SLAVE_ADDR,
	.volt_reg_addr		= OMAP3_VDD_CORE_SR_CONTROL_REG,
	.i2c_high_speed		= true,
	.vsel_to_uv		= omap_cpcap_vsel_to_uv,
	.uv_to_vsel		= omap_cpcap_uv_to_vsel,
};

/* For CPCAP IVA */

static struct omap_voltdm_pmic omap_cpcap_iva = {
	.slew_rate		= 4000,
	.step_size		= 12500,
	.on_volt		= 1200000,
	.onlp_volt		= 1000000,
	.ret_volt		= 975000,
	.off_volt		= 600000,
	.volt_setup_time	= 0xfff,
	.switch_on_time         = 1000,
	.vp_erroroffset		= OMAP3_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP3_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP3_VP_VSTEPMAX_VSTEPMAX,
	.vp_vddmin		= OMAP3630_VP2_VLIMITTO_VDDMIN,
	.vp_vddmax		= OMAP3630_VP2_VLIMITTO_VDDMAX,
	.vp_timeout_us          = OMAP3_VP_VLIMITTO_TIMEOUT_US,
	.i2c_slave_addr         = OMAP3_SRI2C_SLAVE_ADDR,
	.volt_reg_addr          = OMAP3_VDD_CORE_SR_CONTROL_REG,
	.i2c_high_speed         = true,
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
