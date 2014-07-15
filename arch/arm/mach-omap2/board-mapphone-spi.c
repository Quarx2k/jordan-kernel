/*
 * arch/arm/mach-omap2/board-mapphone-spi.c
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/spi.h>
#include <plat/mcspi.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include <plat/resource.h>
#include <plat/omap34xx.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

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
	{CPCAP_REG_SDVSPLL,   0xDB14},
	{CPCAP_REG_SI2CC1,    0x0281},
	{CPCAP_REG_Si2CC2,    0x00C4},
	{CPCAP_REG_S1C1,      0x6438},
	{CPCAP_REG_S1C2,      0x3838},
	{CPCAP_REG_S2C1,      0x6434},
	{CPCAP_REG_S2C2,      0x3C14},
	{CPCAP_REG_S3C,       0x0539},
	{CPCAP_REG_S4C1,      0x0000},
	{CPCAP_REG_S4C2,      0x0000},
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
	[CPCAP_VCAM]     = 0x0003,
	[CPCAP_VCSI]     = 0x0003,
	[CPCAP_VDAC]     = 0x0003,
	[CPCAP_VDIG]     = 0x0003,
	[CPCAP_VFUSE]    = 0x0080,
	[CPCAP_VHVIO]    = 0x0003,
	[CPCAP_VSDIO]    = 0x0003,
	[CPCAP_VPLL]     = 0x0042,
	[CPCAP_VRF1]     = 0x000C,
	[CPCAP_VRF2]     = 0x0003,
	[CPCAP_VRFREF]   = 0x0003,
	[CPCAP_VWLAN1]   = 0x0003,
	[CPCAP_VWLAN2]   = 0x000C,
	[CPCAP_VSIM]     = 0x0003,
	[CPCAP_VSIMCARD] = 0x1E00,
	[CPCAP_VVIB]     = 0x0001,
	[CPCAP_VUSB]     = 0x000C,
	[CPCAP_VAUDIO]   = 0x0006,
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

#define CPCAP_GPIO 0

#define REGULATOR_CONSUMER(name, device) { .supply = name, .dev = device, }

struct regulator_consumer_supply cpcap_sw5_consumers[] = {
	REGULATOR_CONSUMER("sw5", NULL /* lighting_driver */),
};

struct regulator_consumer_supply cpcap_vcam_consumers[] = {
	REGULATOR_CONSUMER("vcam", NULL /* cpcap_cam_device */),
};

extern struct platform_device mapphone_dss_device;

struct regulator_consumer_supply cpcap_vhvio_consumers[] = {
	REGULATOR_CONSUMER("vhvio", NULL /* lighting_driver */),
#if 0
	REGULATOR_CONSUMER("vhvio", NULL /* lighting_driver */),
	REGULATOR_CONSUMER("vhvio", NULL /* magnetometer */),
	REGULATOR_CONSUMER("vhvio", NULL /* light sensor */),
	REGULATOR_CONSUMER("vhvio", NULL /* accelerometer */),
	REGULATOR_CONSUMER("vhvio", NULL /* display */),
#endif
};

struct regulator_consumer_supply cpcap_vsdio_consumers[] = {
	REGULATOR_CONSUMER("vsdio", NULL),
};

struct regulator_consumer_supply cpcap_vcsi_consumers[] = {
	REGULATOR_CONSUMER("vdds_dsi", &mapphone_dss_device.dev),
};

struct regulator_consumer_supply cpcap_vwlan1_consumers[] = {
	REGULATOR_CONSUMER("vwlan1", NULL /* cpcap_cam_device */),
};

struct regulator_consumer_supply cpcap_vwlan2_consumers[] = {
	REGULATOR_CONSUMER("vwlan2", NULL /* sd slot */),
};

struct regulator_consumer_supply cpcap_vsim_consumers[] = {
	REGULATOR_CONSUMER("vsim", NULL),
};

struct regulator_consumer_supply cpcap_vsimcard_consumers[] = {
	REGULATOR_CONSUMER("vsimcard", NULL),
};

struct regulator_consumer_supply cpcap_vvib_consumers[] = {
	REGULATOR_CONSUMER("vvib", NULL /* vibrator */),
};

struct regulator_consumer_supply cpcap_vusb_consumers[] = {
	REGULATOR_CONSUMER("vusb", NULL /* accy det */),
};

struct regulator_consumer_supply cpcap_vaudio_consumers[] = {
	REGULATOR_CONSUMER("vaudio", NULL /* mic opamp */),
};

struct regulator_consumer_supply cpcap_vfuse_consumers[] = {
    REGULATOR_CONSUMER("vfuse", NULL),
};

struct regulator_consumer_supply cpcap_vrf1_consumers[] = {
    REGULATOR_CONSUMER("vrf1", NULL),
};


static struct regulator_init_data cpcap_regulator[CPCAP_NUM_REGULATORS] = {
	[CPCAP_SW5] = {
		.constraints = {
			.min_uV			= 5050000,
			.max_uV			= 5050000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_sw5_consumers),
		.consumer_supplies	= cpcap_sw5_consumers,
	},
	[CPCAP_VCAM] = {
		.constraints = {
			.min_uV			= 2900000,
			.max_uV			= 2900000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,

		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vcam_consumers),
		.consumer_supplies	= cpcap_vcam_consumers,
	},
	[CPCAP_VCSI] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.boot_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vcsi_consumers),
		.consumer_supplies	= cpcap_vcsi_consumers,
	},
	[CPCAP_VDAC] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
	},
	[CPCAP_VDIG] = {
		.constraints = {
			.min_uV			= 1200000,
			.max_uV			= 1875000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VFUSE] = {
		.constraints = {
			.min_uV			= 1500000,
			.max_uV			= 3150000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
		},
		.num_consumer_supplies  = ARRAY_SIZE(cpcap_vfuse_consumers),
		.consumer_supplies      = cpcap_vfuse_consumers,
	},
	[CPCAP_VHVIO] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.always_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vhvio_consumers),
		.consumer_supplies	= cpcap_vhvio_consumers,
	},
	[CPCAP_VSDIO] = {
		.constraints = {
			.min_uV			= 2900000,
			.max_uV			= 2900000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsdio_consumers),
		.consumer_supplies	= cpcap_vsdio_consumers,
	},
	[CPCAP_VPLL] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1800000,
			.valid_ops_mask		= 0,
			.always_on		= 1,
			.apply_uV		= 1,
		},
	},
	[CPCAP_VRF1] = {
		.constraints = {
			.min_uV			= 2500000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vrf1_consumers),
		.consumer_supplies	= cpcap_vrf1_consumers,
	},
	[CPCAP_VRF2] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VRFREF] = {
		.constraints = {
			.min_uV			= 2500000,
			.max_uV			= 2775000,
			.valid_ops_mask		= 0,
		},
	},
	[CPCAP_VWLAN1] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 1900000,
			.valid_ops_mask		= 0,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vwlan1_consumers),
		.consumer_supplies	= cpcap_vwlan1_consumers,
	},
	[CPCAP_VWLAN2] = {
		.constraints = {
			.min_uV			= 3000000,
			.max_uV			= 3000000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vwlan2_consumers),
		.consumer_supplies	= cpcap_vwlan2_consumers,
	},
	[CPCAP_VSIM] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= 0,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsim_consumers),
		.consumer_supplies	= cpcap_vsim_consumers,
	},
	[CPCAP_VSIMCARD] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= 0,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vsimcard_consumers),
		.consumer_supplies	= cpcap_vsimcard_consumers,
	},
	[CPCAP_VVIB] = {
		.constraints = {
			.min_uV			= 1300000,
			.max_uV			= 3000000,
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vvib_consumers),
		.consumer_supplies	= cpcap_vvib_consumers,
	},
	[CPCAP_VUSB] = {
		.constraints = {
			.min_uV			= 3300000,
			.max_uV			= 3300000,
			.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vusb_consumers),
		.consumer_supplies	= cpcap_vusb_consumers,
	},
	[CPCAP_VAUDIO] = {
		.constraints = {
			.min_uV			= 2775000,
			.max_uV			= 2775000,
			.valid_modes_mask	= (REGULATOR_MODE_NORMAL |
						   REGULATOR_MODE_STANDBY),
			.valid_ops_mask		= REGULATOR_CHANGE_MODE,
			.always_on		= 1,
			.apply_uV		= 1,
		},
		.num_consumer_supplies	= ARRAY_SIZE(cpcap_vaudio_consumers),
		.consumer_supplies	= cpcap_vaudio_consumers,
	},
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
};

static void ac_changed(struct power_supply *ac,
		       struct cpcap_batt_ac_data *ac_state)
{
	static char requested;
	int ret = 0;

	if (!ac || !ac_state)
		return;

	if (ac_state->online) {
		/* To reduce OMAP Vdd1 DC/DC converter output voltage dips as
		 * much as possible, limit Vdd1 to OPP3-OPP5 when the phone is
		 * connected to a charger. */
		if (!requested)
			ret = resource_request("vdd1_opp", ac->dev, VDD1_OPP3);

		if (!ret)
			requested = 1;
	} else if (requested) {
		ret = resource_release("vdd1_opp", ac->dev);

		if (!ret)
			requested = 0;
	}
}

static void batt_changed(struct power_supply *batt,
			 struct cpcap_batt_data *batt_state)
{
	static char requested;
	int ret = 0;

	if (!batt || !batt_state)
		return;

	if (batt_state->batt_temp < 0) {
		/* To reduce OMAP Vdd1 DC/DC converter output voltage dips as
		 * much as possible, limit Vdd1 to OPP3-OPP5 when the
		 * temperature is below 0 degrees C. */
		if (!requested)
			ret = resource_request("vdd1_opp", batt->dev, VDD1_OPP3);

		if (!ret)
			requested = 1;
	} else if (requested) {
		ret = resource_release("vdd1_opp", batt->dev);

		if (!ret)
			requested = 0;
	}
}

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
	.regulator_init = cpcap_regulator,
	.adc_ato = &mapphone_cpcap_adc_ato,
	.leds = &mapphone_cpcap_leds,
	.ac_changed = ac_changed,
	.batt_changed = batt_changed,
	.usb_changed = NULL,
	.is_umts = 1,
};

static struct spi_board_info mapphone_spi_board_info[] __initdata = {
	{
		.modalias = "cpcap",
		.bus_num = 1,
		.chip_select = 0,
		.max_speed_hz = 3000000,
		.controller_data = &mapphone_cpcap_data,
		.mode = SPI_CS_HIGH,
	},
};

#ifdef CONFIG_ARM_OF
struct omap_spi_init_entry {
	u32 reg;
	u32 data;
} __attribute__ ((__packed__));

struct omap_rgt_mode_entry {
	u32 id;
	u16 data;
} __attribute__ ((__packed__));

struct omap_rgt_init_entry {
	u32 id;
	u32 min_uV;
	u32 max_uV;
	u32 valid_ops_mask;
	u8 always_on;
	u8 boot_on;
	u8 apply_uV;
} __attribute__ ((__packed__));

static void regulator_init(void *p_data)
{
	struct omap_rgt_init_entry *p = p_data;
	struct regulator_init_data *p_devs = cpcap_regulator;

	if (p->id < CPCAP_NUM_REGULATORS) {
		p_devs[p->id].constraints.min_uV = p->min_uV;
		p_devs[p->id].constraints.max_uV = p->max_uV;
		p_devs[p->id].constraints.valid_ops_mask = p->valid_ops_mask;
		p_devs[p->id].constraints.always_on = p->always_on;
		p_devs[p->id].constraints.boot_on = p->boot_on;
		p_devs[p->id].constraints.apply_uV = p->apply_uV;
		pr_debug(KERN_DEBUG "CPCAP: Overwrite regulator init [%d]!\n",
				p->id);
	} else {
		printk(KERN_ERR "CPCAP: Too big cpcap regulator count!\n");
	}
}

static void regulator_mode_init(void *p_data)
{
	struct omap_rgt_mode_entry *p = p_data;
	unsigned short *p_devs = cpcap_regulator_mode_values;

	if (p->id < CPCAP_NUM_REGULATORS) {
		p_devs[p->id] = p->data;
		pr_debug(KERN_DEBUG  "CPCAP: Overwrite regulator mode [%d]!\n",
				p->id);
	} else {
		printk(KERN_ERR "CPCAP: Too big cpcap regulator count!\n");
	}
}

static void regulator_off_mode_init(void *p_data)
{
	struct omap_rgt_mode_entry *p = p_data;
	unsigned short *p_devs = cpcap_regulator_off_mode_values;

	if (p->id < CPCAP_NUM_REGULATORS) {
		p_devs[p->id] = p->data;
		pr_debug(KERN_DEBUG  "CPCAP: Overwrite regulator off mode [%d]!\n",
				p->id);
	} else {
		printk(KERN_ERR "CPCAP: Too big cpcap regulator count!\n");
	}
}

static void cpcap_spi_init(void *p_data)
{
	struct omap_spi_init_entry *p = p_data;
	struct cpcap_spi_init_data *p_devs = mapphone_cpcap_spi_init;
	int i = 0;

	for (i = 0; i < CPCAP_REG_SIZE + 1; i++) {
		if (p_devs[i].reg == CPCAP_REG_UNUSED) {
			p_devs[i].reg = p->reg;
			p_devs[i].data = p->data;

			if (i != CPCAP_REG_SIZE)
				p_devs[i + 1].reg = CPCAP_REG_UNUSED;

			pr_debug(KERN_DEBUG  "CPCAP: Add new reg [%d] setting!\n",
					p->reg);
			return;
		}

		if (p_devs[i].reg == p->reg) {
			p_devs[i].data = p->data;

			pr_debug(KERN_DEBUG  "CPCAP: Overwrite reg [%d] setting!\n",
					p->reg);
			return;
		}

		if (i == CPCAP_REG_SIZE)
			printk(KERN_ERR "CPCAP: Too big cpcap reg count!\n");
	}
}

static void __init cpcap_of_init(void)
{
	int size, unit_size, i, count;
	struct device_node *node;
	const void *prop;
	struct device_node *bp_node;
	const void *bp_prop;

	node = of_find_node_by_path(DT_PATH_CPCAP);
	if (node == NULL) {
		printk(KERN_ERR
				"Unable to read node %s from device tree!\n",
				DT_PATH_CPCAP);
		return;
	}

	prop = of_get_property(node, DT_PROP_CPCAP_BUSNUM, NULL);
	if (prop) {
		mapphone_spi_board_info[0].bus_num = *(u16 *)prop;

		printk(KERN_INFO "CPCAP: overwriting bus_num with %d\n", \
			mapphone_spi_board_info[0].bus_num);
	} else
		printk(KERN_INFO "CPCAP: using default bus_num %d\n", \
			mapphone_spi_board_info[0].bus_num);

	unit_size = sizeof(struct omap_spi_init_entry);
	prop = of_get_property(node, DT_PROP_CPCAP_SPIINIT, &size);
	if ((!prop) || (size % unit_size)) {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_CPCAP_SPIINIT);
		of_node_put(node);
		return;
	}

	count = size / unit_size;
	printk(KERN_INFO "cpcap init size = %d\n", count);

	for (i = 0; i < count; i++)
		cpcap_spi_init((struct omap_spi_init_entry *)prop + i);

	unit_size = sizeof(struct omap_rgt_init_entry);
	prop = of_get_property(node, DT_PROP_CPCAP_RGTINIT, &size);
	if ((!prop) || (size % unit_size)) {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_CPCAP_RGTINIT);
		of_node_put(node);
		return;
	}

	count = size / unit_size;
	printk(KERN_INFO "cpcap init size = %d\n", count);

	for (i = 0; i < count; i++)
		regulator_init((struct omap_rgt_init_entry *)prop + i);

	unit_size = sizeof(struct omap_rgt_mode_entry);
	prop = of_get_property(node, DT_PROP_CPCAP_RGTMODE, &size);
	if ((!prop) || (size % unit_size)) {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_CPCAP_RGTMODE);
		of_node_put(node);
		return;
	}

	count = size / unit_size;

	for (i = 0; i < count; i++)
		regulator_mode_init((struct omap_rgt_mode_entry *)prop + i);

	unit_size = sizeof(struct omap_rgt_mode_entry);
	prop = of_get_property(node, DT_PROP_CPCAP_RGTOFFMODE, &size);
	if ((!prop) || (size % unit_size)) {
		printk(KERN_ERR "Read property %s error!\n",
				DT_PROP_CPCAP_RGTOFFMODE);
		of_node_put(node);
		return;
	}

	count = size / unit_size;

	for (i = 0; i < count; i++)
		regulator_off_mode_init((struct omap_rgt_mode_entry *)prop + i);

	of_node_put(node);
	return;
}
#endif

void __init mapphone_spi_init(void)
{
	int irq;
	int ret;
	int i;

#ifdef CONFIG_ARM_OF
	cpcap_of_init();
#endif

	for (i = 0; i < CPCAP_REG_SIZE; i++) {
		if (mapphone_cpcap_spi_init[i].reg == CPCAP_REG_UNUSED)
			break;
	}
	mapphone_cpcap_data.init_len = i;

	ret = gpio_request(CPCAP_GPIO, "cpcap-irq");
	if (ret)
		return;
	ret = gpio_direction_input(CPCAP_GPIO);
	if (ret) {
		gpio_free(CPCAP_GPIO);
		return;
	}

	irq = gpio_to_irq(CPCAP_GPIO);
	set_irq_type(irq, IRQ_TYPE_EDGE_RISING);
	omap_cfg_reg(AF26_34XX_GPIO0);

	mapphone_spi_board_info[0].irq = irq;
	spi_register_board_info(mapphone_spi_board_info,
				ARRAY_SIZE(mapphone_spi_board_info));

	/* regulator_has_full_constraints(); */
}
