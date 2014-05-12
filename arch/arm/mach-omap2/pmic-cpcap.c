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

#define OMAP3_SRI2C_SLAVE_ADDR		0x12
#define OMAP3_VDD_MPU_SR_CONTROL_REG	0x00
#define OMAP3_VDD_CORE_SR_CONTROL_REG	0x01
#define OMAP3_VP_CONFIG_ERROROFFSET	0x00
#define OMAP3_VP_VSTEPMIN_VSTEPMIN	0x1
#define OMAP3_VP_VSTEPMAX_VSTEPMAX	0x04
#define OMAP3_VP_VLIMITTO_TIMEOUT_US	200

/**
 * omap_cpcap_vsel_to_vdc - convert CPCAP VSEL value to microvolts DC
 * @vsel: CPCAP VSEL value to convert
 *
 * Returns the microvolts DC that the CPCAP PMIC should generate when
 * programmed with @vsel.
 */
unsigned long omap_cpcap_vsel_to_uv(unsigned char vsel)
{
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
	return DIV_ROUND_UP(uv - 600000, 12500);
}

/* For CPCAP CORE */
static struct omap_voltdm_pmic omap_cpcap_core = {
	.slew_rate		= 4000,
	.step_size		= 12500,
	.vp_erroroffset		= OMAP3_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP3_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP3_VP_VSTEPMAX_VSTEPMAX,
	.vddmin			= 600000,
	.vddmax			= 1450000,
	.vp_timeout_us		= OMAP3_VP_VLIMITTO_TIMEOUT_US,
	.i2c_slave_addr		= OMAP3_SRI2C_SLAVE_ADDR,
	.volt_reg_addr		= OMAP3_VDD_MPU_SR_CONTROL_REG,
	.i2c_high_speed		= true,
	.vsel_to_uv		= omap_cpcap_vsel_to_uv,
	.uv_to_vsel		= omap_cpcap_uv_to_vsel,
};

/* For CPCAP IVA */

static struct omap_voltdm_pmic omap_cpcap_iva = {
	.slew_rate		= 4000,
	.step_size		= 12500,
	.vp_erroroffset		= OMAP3_VP_CONFIG_ERROROFFSET,
	.vp_vstepmin		= OMAP3_VP_VSTEPMIN_VSTEPMIN,
	.vp_vstepmax		= OMAP3_VP_VSTEPMAX_VSTEPMAX,
	.vddmin			= 600000,
	.vddmax			= 1450000,
	.vp_timeout_us		= OMAP3_VP_VLIMITTO_TIMEOUT_US,
	.i2c_slave_addr		= OMAP3_SRI2C_SLAVE_ADDR,
	.volt_reg_addr		= OMAP3_VDD_CORE_SR_CONTROL_REG,
	.i2c_high_speed		= true,
	.vsel_to_uv		= omap_cpcap_vsel_to_uv,
	.uv_to_vsel		= omap_cpcap_uv_to_vsel,
};


int __init omap_cpcap_init(void)
{
	struct voltagedomain *voltdm;

	printk(" __init omap_cpcap_init(void)\n");
	voltdm = voltdm_lookup("mpu_iva");
	omap_voltage_register_pmic(voltdm, &omap_cpcap_iva);

	voltdm = voltdm_lookup("core");
	omap_voltage_register_pmic(voltdm, &omap_cpcap_core);

	return 0;
}
