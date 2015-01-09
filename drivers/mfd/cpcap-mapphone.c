/*
 * arch/arm/mach-omap2/board-mapphone-spi.c
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

/* This is converted from the boardfile.  Instead of migrating the
   driver to the new device_tree format, this code is left in place
   to hardcode device settings.  This is driver is for obsolete hardware
   and will be removed in the future
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>

struct cpcap_spi_init_data mapphone_cpcap_spi_init[CPCAP_REG_SIZE + 1] = {
	{CPCAP_REG_ASSIGN1,   0x0101},
	{CPCAP_REG_ASSIGN2,   0x0000},
	{CPCAP_REG_ASSIGN3,   0x0000},
	{CPCAP_REG_ASSIGN4,   0x0000},
	{CPCAP_REG_ASSIGN5,   0x0000},
	{CPCAP_REG_ASSIGN6,   0x0000},
	{CPCAP_REG_UCC1,      0x0000},
	{CPCAP_REG_PC1,       0x010A},
	{CPCAP_REG_PC2,       0x0150},
	{CPCAP_REG_PGC,       0x0000},
	{CPCAP_REG_SDVSPLL,   0xdb04},
	{CPCAP_REG_SI2CC1,    0x0281},
	{CPCAP_REG_Si2CC2,    0x00C4},
	{CPCAP_REG_S1C1,      0x6438},
	{CPCAP_REG_S1C2,      0x3838},
	{CPCAP_REG_S2C1,      0x6434},
	{CPCAP_REG_S2C2,      0x3C14},
	{CPCAP_REG_S3C,       0x0539},
	{CPCAP_REG_S4C1,      0x4034},
	{CPCAP_REG_S4C2,      0x3434},
	{CPCAP_REG_S6C,       0x0000},
	{CPCAP_REG_VWLAN2C,   0x0001},
	{CPCAP_REG_VUSBINT1C, 0x0029},
	{CPCAP_REG_VUSBINT2C, 0x0029},
	{CPCAP_REG_VAUDIOC,   0x0060},
	{CPCAP_REG_CCCC2,     0x002B},
	{CPCAP_REG_ADCC1,     0x9000},
	{CPCAP_REG_ADCC2,     0x4136},
	{CPCAP_REG_USBC1,     0x1201},
	{CPCAP_REG_USBC3,     0x7DFB},
	{CPCAP_REG_UIER2,     0x001F},
	{CPCAP_REG_UIEF2,     0x001F},
	{CPCAP_REG_OWDC,      0x0003},
	{CPCAP_REG_GPIO0,     0x3004},
	{CPCAP_REG_GPIO1,     0x3004},
	{CPCAP_REG_GPIO2,     0x3204},
	{CPCAP_REG_GPIO3,     0x3008},
	{CPCAP_REG_GPIO4,     0x3204},
	{CPCAP_REG_GPIO5,     0x3008},
	{CPCAP_REG_GPIO6,     0x3004},
	{CPCAP_REG_KLC,       0x0000},
	{CPCAP_REG_UNUSED,    0x0000},
};

unsigned short cpcap_regulator_mode_values[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW5]      = 0x0022,
	[CPCAP_VCAM]     = 0x0082,
	[CPCAP_VCSI]     = 0x0043,
	[CPCAP_VDAC]     = 0x0003,
	[CPCAP_VDIG]     = 0x0082,
	[CPCAP_VFUSE]    = 0x0080,
	[CPCAP_VHVIO]    = 0x0013,
	[CPCAP_VSDIO]    = 0x0003,
	[CPCAP_VPLL]     = 0x0042,
	[CPCAP_VRF1]     = 0x0024,
	[CPCAP_VRF2]     = 0x0001,
	[CPCAP_VRFREF]   = 0x0023,
	[CPCAP_VWLAN1]   = 0x0003,
	[CPCAP_VWLAN2]   = 0x000C,
	[CPCAP_VSIM]     = 0x0003,
	[CPCAP_VSIMCARD] = 0x1E00,
	[CPCAP_VVIB]     = 0x0001,
	[CPCAP_VUSB]     = 0x000C,
	[CPCAP_VAUDIO]   = 0x0014,
};

unsigned short cpcap_regulator_off_mode_values[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW5]      = 0x0000,
	[CPCAP_VCAM]     = 0x0000,
	[CPCAP_VCSI]     = 0x0000,
	[CPCAP_VDAC]     = 0x0000,
	[CPCAP_VDIG]     = 0x0000,
	[CPCAP_VFUSE]    = 0x0000,
	[CPCAP_VHVIO]    = 0x0000,
	[CPCAP_VSDIO]    = 0x0000,
	[CPCAP_VPLL]     = 0x0000,
	[CPCAP_VRF1]     = 0x0000,
	[CPCAP_VRF2]     = 0x0000,
	[CPCAP_VRFREF]   = 0x0000,
	[CPCAP_VWLAN1]   = 0x0000,
	[CPCAP_VWLAN2]   = 0x0000,
	[CPCAP_VSIM]     = 0x0000,
	[CPCAP_VSIMCARD] = 0x0000,
	[CPCAP_VVIB]     = 0x0000,
	[CPCAP_VUSB]     = 0x0000,
	[CPCAP_VAUDIO]   = 0x0000,
};

struct regulator_consumer_supply cpcap_vcsi_consumers[] = {
	REGULATOR_SUPPLY("vdds_dsi", "omapdss"),
	REGULATOR_SUPPLY("vdds_dsi", "omapdss_dsi.0"),
};

static struct cpcap_adc_ato mapphone_cpcap_adc_ato = {
	.ato_in = 0x0480,
	.atox_in = 0,
	.adc_ps_factor_in = 0x0200,
	.atox_ps_factor_in = 0,
	.ato_out = 0,
	.atox_out = 0,
	.adc_ps_factor_out = 0,
	.atox_ps_factor_out = 0,
	.ichrg_sense_res = 220,
};

static struct cpcap_leds mapphone_cpcap_leds = {
	.display_led = {
		.display_reg = CPCAP_REG_MDLC,
		.display_mask = 0xFFFF,
		.display_off = 0xFFFA,
		.display_init = 0xB019,
		.poll_intvl = 3000,
	},
	.button_led = {
		.button_reg = CPCAP_REG_BLUEC,
		.button_mask = 0x03FF,
		.button_on = 0x00F5,
		.button_off = 0x00F4,
	},
	.kpad_led = {
		.kpad_reg = CPCAP_REG_KLC,
		.kpad_mask = 0x7FFF,
		.kpad_on = 0x5FF5,
		.kpad_off = 0x5FF0,
	},
	/* To find LUX value from ALS data,
	   below variables are used.
	    * lux_max - LUX maximum value
	    * lux_minimum - LUX minimum value
	    * als_max - Maximum ALS data
	    * als_min - Minimum ALS data */
	.als_data = {
		.lux_max = 5000,
		.lux_min = 100,
		.als_max = 590,
		.als_min = 9,
	},
};

static struct cpcap_platform_data mapphone_cpcap_data = {
	.init = mapphone_cpcap_spi_init,
	.regulator_mode_values = cpcap_regulator_mode_values,
	.regulator_off_mode_values = cpcap_regulator_off_mode_values,
	.adc_ato = &mapphone_cpcap_adc_ato,
	.leds = &mapphone_cpcap_leds,
	.ac_changed = NULL,
	.batt_changed = NULL,
	.usb_changed = NULL,
	.is_umts = 0,
	.hwcfg = {CPCAP_HWCFG0_NONE, CPCAP_HWCFG1_STBY_GPIO},
	.irq_gpio = 0,
};

#ifdef CONFIG_OF

static struct of_regulator_match cpcap_reg_match_table[CPCAP_NUM_REGULATORS] = {
	{ .name = "CPCAP_SW1" },
	{ .name = "CPCAP_SW2" },
	{ .name = "CPCAP_SW3" },
	{ .name = "CPCAP_SW4" },
	{ .name = "CPCAP_SW5" },
	{ .name = "CPCAP_VCAM" },
	{ .name = "CPCAP_VCSI" },
	{ .name = "CPCAP_VDAC" },
	{ .name = "CPCAP_VDIG" },
	{ .name = "CPCAP_VFUSE" },
	{ .name = "CPCAP_VHVIO" },
	{ .name = "CPCAP_VSDIO" },
	{ .name = "CPCAP_VPLL" },
	{ .name = "CPCAP_VRF1" },
	{ .name = "CPCAP_VRF2" },
	{ .name = "CPCAP_VRFREF" },
	{ .name = "CPCAP_VWLAN1" },
	{ .name = "CPCAP_VWLAN2" },
	{ .name = "CPCAP_VSIM" },
	{ .name = "CPCAP_VSIMCARD" },
	{ .name = "CPCAP_VVIB" },
	{ .name = "CPCAP_VUSB" },
	{ .name = "CPCAP_VAUDIO" },
};

struct cpcap_platform_data *cpcap_get_plat_data(struct cpcap_device *cpcap)
{
	struct device_node *np = cpcap->spi->dev.of_node;
	struct device_node *regulators;
	struct of_regulator_match *reg_matches;
	struct regulator_init_data *reg_data;
	unsigned int prop;
	int i, ret;

	for (i = 0; i < CPCAP_REG_SIZE; i++) {
		if (mapphone_cpcap_spi_init[i].reg == CPCAP_REG_UNUSED)
			break;
	}
	mapphone_cpcap_data.init_len = i;

	if (!of_property_read_u32(np, "irq-gpio", &prop))
		mapphone_cpcap_data.irq_gpio = prop;

	regulators = of_find_node_by_name(np, "regulators");
	if (!regulators) {
		dev_err(&cpcap->spi->dev, "regulator node not found\n");
		return NULL;
	}

	reg_matches = cpcap_reg_match_table;

	ret = of_regulator_match(&cpcap->spi->dev, regulators, reg_matches, CPCAP_NUM_REGULATORS);
	of_node_put(regulators);
	if (ret < 0) {
		dev_err(&cpcap->spi->dev, "Error parsing regulator init data: %d\n",
			ret);
		return NULL;
	}

	mapphone_cpcap_data.regulator_init = devm_kzalloc(&cpcap->spi->dev,
			sizeof(struct regulator_init_data) * CPCAP_NUM_REGULATORS, GFP_KERNEL);

	for (i = 0; i < CPCAP_NUM_REGULATORS; i++) {
		if (!reg_matches[i].init_data || !reg_matches[i].of_node)
			continue;

		reg_data = reg_matches[i].init_data;

		switch (i) {
			case CPCAP_VCSI:
				printk(KERN_DEBUG "Patching consumer_supplies of %s\n", reg_data->constraints.name);
				reg_data->consumer_supplies = cpcap_vcsi_consumers;
				reg_data->num_consumer_supplies = ARRAY_SIZE(cpcap_vcsi_consumers);
				break;
			default:
				break;
		}

		mapphone_cpcap_data.regulator_init[i] = *reg_data;
	}

	return &mapphone_cpcap_data;
}
#else
static inline
struct cpcap_platform_data *cpcap_get_plat_data(struct cpcap_device *cpcap)
{
	return NULL;
}
#endif

