/*
 * arch/arm/mach-omap2/board-sholes-spi.c
 *
 * Copyright (C) 2009 Motorola, Inc.
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

#ifdef CONFIG_MFD_CPCAP
extern struct platform_device cpcap_disp_button_led;
extern struct platform_device cpcap_rgb_led;
#endif

struct cpcap_spi_init_data sholes_cpcap_spi_init[] = {
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
	{CPCAP_REG_ADCC1,     0x9000},
	{CPCAP_REG_USBC1,     0x1201},
	{CPCAP_REG_USBC3,     0x7DFB},
	{CPCAP_REG_UIER2,     0x001F},
	{CPCAP_REG_UIEF2,     0x001F},
	{CPCAP_REG_OWDC,      0x0003},
	{CPCAP_REG_GPIO0,     0x3004},
	{CPCAP_REG_GPIO1,     0x3000},
	{CPCAP_REG_GPIO2,     0x3204},
	{CPCAP_REG_GPIO3,     0x3008},
	{CPCAP_REG_GPIO4,     0x3204},
	{CPCAP_REG_GPIO5,     0x3008},
	{CPCAP_REG_GPIO6,     0x3004},
	{CPCAP_REG_MDLC,      0x0000},
	{CPCAP_REG_KLC,       0x0000},
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

#define CPCAP_GPIO 0

#define REGULATOR_CONSUMER(name, device) { .supply = name, .dev = device, }

struct regulator_consumer_supply cpcap_sw5_consumers[] = {
#ifdef CONFIG_MFD_CPCAP
	REGULATOR_CONSUMER("vdd_button_backlight", &cpcap_disp_button_led.dev),
	REGULATOR_CONSUMER("vdd_notification_led", &cpcap_rgb_led.dev),
#endif
};

struct regulator_consumer_supply cpcap_vcam_consumers[] = {
	REGULATOR_CONSUMER("vcam", NULL /* cpcap_cam_device */),
};

extern struct platform_device sholes_dss_device;

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
	REGULATOR_CONSUMER("vdds_dsi", &sholes_dss_device.dev),
};

struct regulator_consumer_supply cpcap_vwlan2_consumers[] = {
	REGULATOR_CONSUMER("vwlan2", NULL /* sd slot */),
};

struct regulator_consumer_supply cpcap_vvib_consumers[] = {
	REGULATOR_CONSUMER("vvib", NULL /* vibrator */),
};

struct regulator_consumer_supply cpcap_vaudio_consumers[] = {
	REGULATOR_CONSUMER("vaudio", NULL /* mic opamp */),
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
	},
	[CPCAP_VSIMCARD] = {
		.constraints = {
			.min_uV			= 1800000,
			.max_uV			= 2900000,
			.valid_ops_mask		= 0,
		},
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
			.valid_ops_mask		= (REGULATOR_CHANGE_VOLTAGE |
						   REGULATOR_CHANGE_STATUS),
			.apply_uV		= 1,
		},
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

static struct cpcap_adc_ato sholes_cpcap_adc_ato = {
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

static struct cpcap_platform_data sholes_cpcap_data = {
	.init = sholes_cpcap_spi_init,
	.init_len = ARRAY_SIZE(sholes_cpcap_spi_init),
	.regulator_mode_values = cpcap_regulator_mode_values,
	.regulator_init = cpcap_regulator,
	.adc_ato = &sholes_cpcap_adc_ato,
	.ac_changed = ac_changed,
	.batt_changed = batt_changed,
	.usb_changed = NULL,
};

static struct spi_board_info sholes_spi_board_info[] __initdata = {
	{
		.modalias = "cpcap",
		.bus_num = 1,
		.chip_select = 0,
		.max_speed_hz = 20000000,
		.controller_data = &sholes_cpcap_data,
		.mode = SPI_CS_HIGH,
	},
};

void __init sholes_spi_init(void)
{
	int irq;
	int ret;

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

	sholes_spi_board_info[0].irq = irq;
	spi_register_board_info(sholes_spi_board_info,
			       ARRAY_SIZE(sholes_spi_board_info));

	regulator_has_full_constraints();
}
