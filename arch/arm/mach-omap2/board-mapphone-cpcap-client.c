/*
 * arch/arm/mach-omap2/board-mapphone-cpcap-client.c
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
#include <linux/spi/cpcap.h>
#include <linux/cpcap_audio_platform_data.h>
#include <linux/pm_dbg.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <plat/omap34xx.h>
#include <linux/leds-cpcap-abmode.h>
#include <linux/leds-cpcap-adb.h>
#include <linux/leds-cpcap-gpio.h>
#include <linux/leds-cpcap-kpb.h>
#include <linux/leds-cpcap-mdb.h>
#include <linux/leds-cpcap-rgb.h>
#include "dt_path.h"
#include <linux/of.h>

/* TODO: Remove after implementing RIL. */
#ifdef CONFIG_MACH_OMAP_MAPPHONE_DEFY
#define MAPPHONE_BP_QSC6085	0x001E0000
#define MAPPHONE_BP_MDM6600	0x001E0001
#define MAPPHONE_BP_MDM9600	0x001E0002
#define MAPPHONE_BP_STE_M570	0x00240000
#define MAPPHONE_BP_W3GLTE	0x0003000F
#endif

/*
 * CPCAP devcies are common for different HW Rev.
 *
 */
static struct platform_device cpcap_3mm5_device = {
	.name   = "cpcap_3mm5",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};

#ifdef CONFIG_CPCAP_USB
static struct platform_device cpcap_usb_device = {
	.name           = "cpcap_usb",
	.id             = -1,
	.dev.platform_data = NULL,
};

static struct platform_device cpcap_usb_det_device = {
	.name   = "cpcap_usb_det",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};
#endif /* CONFIG_CPCAP_USB */

#ifdef CONFIG_TTA_CHARGER
static struct platform_device cpcap_tta_det_device = {
	.name   = "cpcap_tta_charger",
	.id     = -1,
	.dev    = {
		.platform_data  = NULL,
	},
};
#endif


#if defined(CONFIG_SOUND_CPCAP_OMAP) || defined(CONFIG_SND_SOC_CPCAP)
static struct platform_device cpcap_audio_device = {
	.name           = "cpcap_audio",
	.id             = -1,
	.dev.platform_data  = NULL,
};
#endif

static struct platform_device cpcap_bd7885 = {
	.name           = "bd7885",
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
       },
};

static struct platform_device cpcap_vio_active_device = {
	.name		= "cpcap_vio_active",
	.id		= -1,
	.dev		= {
		.platform_data = NULL,
	},
};

#ifdef CONFIG_PM_DBG_DRV
static struct platform_device cpcap_pm_dbg_device = {
	.name		= "cpcap_pm_dbg",
	.id		= -1,
	.dev		= {
		.platform_data = NULL,
	},
};

static struct pm_dbg_drvdata cpcap_pm_dbg_drvdata = {
	.pm_cd_factor = 1000,
};
#endif

static struct platform_device *cpcap_devices[] = {
#ifdef CONFIG_CPCAP_USB
	&cpcap_usb_device,
	&cpcap_usb_det_device,
#endif
#if defined(CONFIG_SOUND_CPCAP_OMAP) || defined(CONFIG_SND_SOC_CPCAP)
	&cpcap_audio_device,
#endif
	&cpcap_3mm5_device,
#ifdef CONFIG_TTA_CHARGER
	&cpcap_tta_det_device,
#endif
#ifdef CONFIG_LEDS_AF_LED
	&cpcap_af_led,
#endif
	&cpcap_bd7885
};


/*
 * CPCAP devcies whose availability depends on HW
 *
 */
static struct platform_device cpcap_lm3554 = {
	.name           = "flash-torch",
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
	},
};

static struct platform_device cpcap_lm3559 = {
	.name           = "flash-torch-3559",
	.id             = -1,
	.dev            = {
		.platform_data  = NULL,
	},
};

static struct cpcap_abmode_config_data abmode_config_data = {
	.abmode_init = CPCAP_ABMODE_INIT,
};

static struct cpcap_adb_led_config_data adb_led_data = {
	.init = CPCAP_ADB_INIT,
	.on = CPCAP_ADB_ON,
	.abmode_config = &abmode_config_data,
	.class_name = CPCAP_ADB_LED_CLASS_NAME,
};

static struct platform_device cpcap_adb_led = {
	.name = CPCAP_ADB_LED_DRV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data  = &adb_led_data,
	},
};

static struct cpcap_gpio_led_config_data gpio_led_data = {
	.reg = CPCAP_REG_GPIO6,
	.init_mask = CPCAP_GPIO_INIT_MASK,
	.init = CPCAP_GPIO_INIT,
	.class_name = CPCAP_GPIO_LED_CLASS_NAME,
};

static struct platform_device cpcap_gpio_led = {
	.name = CPCAP_GPIO_LED_DRV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data  = &gpio_led_data,
	},
};

static struct cpcap_kpb_led_config_data kpb_led_data = {
	.init = CPCAP_KPB_INIT,
	.on = CPCAP_KPB_ON,
	.class_name = CPCAP_KPB_LED_CLASS_NAME,
};

static struct platform_device cpcap_kpb_led = {
	.name = CPCAP_KPB_LED_DRV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data = &kpb_led_data,
	},
};

static struct cpcap_mdb_led_config_data mdb_led_data = {
	.init = CPCAP_MDB_INIT,
	.abmode_config = &abmode_config_data,
	.class_name = CPCAP_MDB_LED_CLASS_NAME,
};

static struct platform_device cpcap_mdb_led = {
	.name = CPCAP_MDB_LED_DRV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data  = &mdb_led_data,
	},
};

static struct cpcap_rgb_led_config_data rgb_led_data = {
	.red_enable = true,
	.green_enable = true,
	.blue_enable = true,
	.blink_enable = true,
	.class_name_red = CPCAP_RGB_LED_RED_CLASS_NAME,
	.class_name_green = CPCAP_RGB_LED_GREEN_CLASS_NAME,
	.class_name_blue = CPCAP_RGB_LED_BLUE_CLASS_NAME,
};

static struct platform_device cpcap_rgb_led = {
	.name = CPCAP_RGB_LED_DRV_NAME,
	.id   = -1,
	.dev  = {
	.platform_data  = &rgb_led_data,
	},
};

static int __init led_cpcap_lm3554_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_LM3554);
	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "device_available", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", "device_available");
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

static int __init led_cpcap_lm3559_init(void)
{
	u8 device_available;
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_LM3559);
	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "device_available", NULL);
	if (prop)
		device_available = *(u8 *)prop;
	else {
		pr_err("Read property %s error!\n", "device_available");
		of_node_put(node);
		return -ENODEV;
	}

	of_node_put(node);
	return device_available;
}

int is_cpcap_vio_supply_converter(void)
{
	struct device_node *node;
	const void *prop;
	int size;

	node = of_find_node_by_path(DT_PATH_CPCAP);
	if (node) {
		prop = of_get_property(node,
				DT_PROP_CPCAP_VIO_SUPPLY_CONVERTER,
				&size);
		if (prop && size)
			return *(u8 *)prop;
	}
	/* The converter is existing by default */
	return 1;
}

#if defined(CONFIG_SOUND_CPCAP_OMAP) || defined(CONFIG_SND_SOC_CPCAP)
static int cpcap_bp_get_type(void)
{
	int ret = 0;
	struct device_node *node;
	const void *prop;
	int size;

	node = of_find_node_by_path("/System@0/Modem@0");
	if (node) {
		prop = of_get_property(node, \
		DT_PROP_MODEM_TYPE, &size);
		if (prop && size)
			ret = *(u32 *)prop;
		of_node_put(node);
	}
	return ret;
}

static void get_cpcap_audio_data(void)
{
	struct device_node *node;
	const void *prop;
	static struct cpcap_audio_pdata data;

	cpcap_audio_device.dev.platform_data = (void *)&data;

	/* read modem-type from device tree to setup data.voice_type */
	switch (cpcap_bp_get_type()) {
	case MAPPHONE_BP_MDM6600:
	case MAPPHONE_BP_MDM9600:
		data.voice_type = VOICE_TYPE_QC;
		break;
	case MAPPHONE_BP_QSC6085:
		data.voice_type = VOICE_TYPE_QC_ANALOG;
		break;
	case MAPPHONE_BP_STE_M570:
		data.voice_type = VOICE_TYPE_STE;
		break;
	case MAPPHONE_BP_W3GLTE:
		data.voice_type = VOICE_TYPE_MOT;
		break;
	default:
		data.voice_type = VOICE_TYPE_NOT_SUPPORT;
		break;
	}

	data.voice_type = VOICE_TYPE_QC; //Check for correct value. We don't have bp_type in devtree

	node = of_find_node_by_path(DT_PATH_AUDIO);
	if (!node) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_AUDIO);
		return;
	}

	prop = of_get_property(node, DT_PROP_AUDIO_STEREO_LOUDSPEAKER, NULL);
	if (prop)
		data.stereo_loudspeaker = (*(int *)prop > 0 ? 1 : 0);
	else {
		data.stereo_loudspeaker = 0;
		pr_err("Read property %s error!\n",
			DT_PROP_AUDIO_STEREO_LOUDSPEAKER);
	}

	prop = of_get_property(node, DT_PROP_AUDIO_MIC3, NULL);
	if (prop)
		data.mic3 = (*(int *)prop > 0 ? 1 : 0);
	else {
		data.mic3 = 0;
		pr_err("Read property %s error!\n",
			DT_PROP_AUDIO_MIC3);
	}
	of_node_put(node);

}
#endif

static int cpcap_abmode_led_init(struct device_node *node)
{
	const void *prop;
	int len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "init", &len);
	if (prop && len)
		abmode_config_data.abmode_init = *(u16 *)prop;

	return 0;
}

static int cpcap_adb_led_init(struct device_node *node)
{
	const void *prop;
	int len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "init", &len);
	if (prop && len)
		adb_led_data.init = *(u16 *)prop;

	prop = of_get_property(node, "on", &len);
	if (prop && len)
		adb_led_data.on = *(u16 *)prop;

	prop = of_get_property(node, "dev_name", &len);
	if (prop && len) {
		strncpy(adb_led_data.class_name, (char *)prop,
			sizeof(adb_led_data.class_name) - 1);
		adb_led_data.class_name[sizeof(adb_led_data.class_name) - 1]
			= '\0';
	}

	cpcap_device_register(&cpcap_adb_led);

	return 0;
}

static int cpcap_gpio_led_init(struct device_node *node)
{
	const void *prop;
	int len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "reg", &len);
	if (prop && len)
		gpio_led_data.reg = *(u32 *)prop;

	prop = of_get_property(node, "init_mask", &len);
	if (prop && len)
		gpio_led_data.init_mask = *(u16 *)prop;

	prop = of_get_property(node, "init", &len);
	if (prop && len)
		gpio_led_data.init = *(u16 *)prop;

	prop = of_get_property(node, "dev_name", &len);
	if (prop && len) {
		strncpy(gpio_led_data.class_name, (char *)prop,
			sizeof(gpio_led_data.class_name) - 1);
		gpio_led_data.class_name[sizeof(gpio_led_data.class_name) - 1]
			= '\0';
	}

	cpcap_device_register(&cpcap_gpio_led);

	return 0;
}

static int cpcap_kpb_led_init(struct device_node *node)
{
	const void *prop;
	int len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "init", &len);
	if (prop && len)
		kpb_led_data.init = *(u16 *)prop;

	prop = of_get_property(node, "on", &len);
	if (prop && len)
		kpb_led_data.on = *(u16 *)prop;

	prop = of_get_property(node, "dev_name", &len);
	if (prop && len) {
		strncpy(kpb_led_data.class_name, (char *)prop,
			sizeof(kpb_led_data.class_name) - 1);
		kpb_led_data.class_name[sizeof(kpb_led_data.class_name) - 1]
			= '\0';
	}

	cpcap_device_register(&cpcap_kpb_led);

	return 0;
}

static int cpcap_mdb_led_init(struct device_node *node)
{
	const void *prop;
	int len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "init", &len);
	if (prop && len)
		mdb_led_data.init = *(u16 *)prop;

	prop = of_get_property(node, "dev_name", &len);
	if (prop && len) {
		strncpy(mdb_led_data.class_name, (char *)prop,
			sizeof(mdb_led_data.class_name) - 1);
		mdb_led_data.class_name[sizeof(mdb_led_data.class_name) - 1]
			= '\0';
	}

	cpcap_device_register(&cpcap_mdb_led);

	return 0;
}

static int cpcap_rgb_led_init(struct device_node *node)
{
	const void *prop;
	int len = 0;

	if (node == NULL)
		return -ENODEV;

	prop = of_get_property(node, "enable_red", &len);
	if (prop && len)
		rgb_led_data.red_enable = *(u8 *)prop;

	prop = of_get_property(node, "enable_green", &len);
	if (prop && len)
		rgb_led_data.green_enable = *(u8 *)prop;

	prop = of_get_property(node, "enable_blue", &len);
	if (prop && len)
		rgb_led_data.blue_enable = *(u8 *)prop;

	prop = of_get_property(node, "enable_blink", &len);
	if (prop && len)
		rgb_led_data.blink_enable = *(u8 *)prop;

	prop = of_get_property(node, "dev_name_red", &len);
	if (prop && len) {
		strncpy(rgb_led_data.class_name_red, (char *)prop,
			sizeof(rgb_led_data.class_name_red) - 1);
		rgb_led_data.class_name_red[
			sizeof(rgb_led_data.class_name_red) - 1]
			= '\0';
	}

	prop = of_get_property(node, "dev_name_green", &len);
	if (prop && len) {
		strncpy(rgb_led_data.class_name_green, (char *)prop,
			sizeof(rgb_led_data.class_name_green) - 1);
		rgb_led_data.class_name_green[
			sizeof(rgb_led_data.class_name_green) - 1]
			= '\0';
	}

	prop = of_get_property(node, "dev_name_blue", &len);
	if (prop && len) {
		strncpy(rgb_led_data.class_name_blue, (char *)prop,
			sizeof(rgb_led_data.class_name_blue) - 1);
		rgb_led_data.class_name_blue[
			sizeof(rgb_led_data.class_name_blue) - 1]
			= '\0';
	}

	cpcap_device_register(&cpcap_rgb_led);

	return 0;
}

static void cpcap_leds_init(void)
{
	struct device_node *node;
	const void *prop;
	int len = 0;

	pr_info("%s Configuring CPCAP LED drivers\n", __func__);

	/* The ABMODE settings are neeeded before the ADB */
	/* and MDB drivers load */
	node = of_find_node_by_name(NULL, "LEDController");
	while (node) {
		prop = of_get_property(node, "type", &len);
		if (prop && len && (*(int *)prop == 0x00210000)) {
			pr_info("CPCAP,ABMODE found.\n");
			cpcap_abmode_led_init(node);
			of_node_put(node);
			break;
		}

		node = of_find_node_by_name(node, "LEDController");
	}

	node = of_find_node_by_name(NULL, "LEDController");
	while (node) {
		pr_info("CPCAP LED found.\n");

		prop = of_get_property(node, "type", &len);
		if (prop && len) {
			pr_info("type = 0X%8X\n", *(int *)prop);

			switch (*(int *)prop) {
			case 0x00210000:
				break;
			case 0x00210001:
				pr_info("CPCAP,ADB found.\n");
				cpcap_adb_led_init(node);
				break;
			case 0x00210002:
				pr_info("CPCAP,GPIO found.\n");
				cpcap_gpio_led_init(node);
				break;
			case 0x00210003:
				pr_info("CPCAP,KPB found.\n");
				cpcap_kpb_led_init(node);
				break;
			case 0x00210004:
				pr_info("CPCAP,MDB found.\n");
				cpcap_mdb_led_init(node);
				break;
			case 0x00210005:
				pr_info("CPCAP,RGB found.\n");
				cpcap_rgb_led_init(node);
				break;
			default:
				pr_err("Unknown CPCAP LED device 0x%8x.\n",
					*(int *)prop);
				break;
			}
		}

		node = of_find_node_by_name(node, "LEDController");
	}
}

#ifdef CONFIG_PM_DBG_DRV
static void get_pm_dbg_drvdata(void)
{
	struct device_node *node;
	const void *prop;
	int size;

	node = of_find_node_by_path("/System@0/PMDbgDevice@0");
	if (node) {
		prop = of_get_property(node,
			"pm_cd_factor",
			&size);
		if (prop && size)
			cpcap_pm_dbg_drvdata.pm_cd_factor = *(u16 *)prop;
	}
}
#endif


void __init mapphone_cpcap_client_init(void)
{
	int i;

#if defined(CONFIG_SOUND_CPCAP_OMAP) || defined(CONFIG_SND_SOC_CPCAP)
	get_cpcap_audio_data();
#endif

	for (i = 0; i < ARRAY_SIZE(cpcap_devices); i++)
		cpcap_device_register(cpcap_devices[i]);

	cpcap_leds_init();

	if (led_cpcap_lm3554_init() > 0)
		cpcap_device_register(&cpcap_lm3554);

	if (led_cpcap_lm3559_init() > 0)
		cpcap_device_register(&cpcap_lm3559);

	if (!is_cpcap_vio_supply_converter())
		cpcap_device_register(&cpcap_vio_active_device);

#ifdef CONFIG_PM_DBG_DRV
	get_pm_dbg_drvdata();
	cpcap_device_register(&cpcap_pm_dbg_device);
	platform_set_drvdata(&cpcap_pm_dbg_device, &cpcap_pm_dbg_drvdata);
#endif
}

