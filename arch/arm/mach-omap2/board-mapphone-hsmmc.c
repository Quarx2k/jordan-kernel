/*
 * linux/arch/arm/mach-omap2/board-mapphone-hsmm.c
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Derived from linux/arch/arm/mach-omap2/board-sdp-hsmmc.c
 *
 * Copyright (C) 2007-2008 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Author: Texas Instruments
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/err.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/gpio_mapping.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/mmc/sdio_func.h>
#include <linux/regulator/consumer.h>

#include <plat/hardware.h>
#include <plat/mmc.h>
#include <plat/board.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include "hsmmc.h"

#include "dt_path.h"
#include <linux/of.h>

/*
 * On the assumption that every product has a microSD detection pin
 * If not supplied from devtree, below default value will be used
 */
#define GPIO_SIGNAL_SD_DET_N 163

#define REG_NAME_LEN	10
static char emmc_regulator_name[REG_NAME_LEN] = "vsdio";
/* no mutex needed at this level for enabling/disabling */
static struct regulator *emmc_regulator;

static char micro_sd_regulator_name[REG_NAME_LEN] = "vwlan2";
static struct regulator *micro_sd_regulator;

/*
 * Save the index of the controller to which tiwlan connected
 * The index is 1 based
 * Will set it first according to devtree, if there is;
 * Else set to CONFIG_TIWLAN_MMC_CONTROLLER
 */
int tiwlan_mmc_controller;

/*
 * for eMMC, we need to take care
 *    Vcc: VSDIO by default (), can be switched off after sleep
 *    VccQ: VIO (1.8V), can't be switched off (no_off)
 */
static int emmc_set_power(struct device *dev, int slot, int power_on,
			  int vdd)
{
	int ret = 0;
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/*
	 * If we don't see a Vcc regulator, assume it's a fixed
	 * voltage always-on regulator.
	 */
	if (!emmc_regulator)
		return 0;

	if (mmc->slots[0].before_set_reg)
		mmc->slots[0].before_set_reg(dev, slot, power_on, vdd);

	/*
	 * Assume Vcc regulator is used only to power the card ... OMAP
	 * VDDS is used to power the pins, optionally with a transceiver to
	 * support cards using voltages other than VDDS (1.8V nominal).  When a
	 * transceiver is used, DAT3..7 are muxed as transceiver control pins.
	 *
	 * In some cases this regulator won't support enable/disable;
	 * e.g. it's a fixed rail for a WLAN chip.
	 *
	 * In other cases vcc_aux switches interface power.  Example, for
	 * eMMC cards it represents VccQ.  Sometimes transceivers or SDIO
	 * chips/cards need an interface voltage rail too.
	 */
	if (power_on) {
		ret = regulator_enable(emmc_regulator);
		if (ret)
			printk(KERN_ERR "%s: enable regulator failed (%d)\n",
				__func__, ret);
	} else {
		/* for the first time unbalanced disable */
		ret = regulator_is_enabled(emmc_regulator);
		if (ret == 1) {
			ret = regulator_disable(emmc_regulator);
			if (ret)
				printk(KERN_ERR "%s: disable regulator"
					" failed(%d)\n",
					__func__, ret);
		}
	}

	if (mmc->slots[0].after_set_reg)
		mmc->slots[0].after_set_reg(dev, slot, power_on, vdd);

	return ret;
}

static int emmc_set_sleep(struct device *dev, int slot, int sleep,
			  int vdd, int cardsleep)
{
	int ret = 0;

	if (!emmc_regulator)
		return 0;

	/* cann't tell between card sleep and regulator sleep here */
	if (sleep)
		ret = regulator_disable(emmc_regulator);
	else
		ret = regulator_enable(emmc_regulator);

	/* ret value not used by host controller now */
	return ret;
}

static int microsd_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	int ret = 0;
	struct omap_mmc_platform_data *mmc = dev->platform_data;

	/*
	 * If we don't see a Vcc regulator, assume it's a fixed
	 * voltage always-on regulator.
	 */
	if (!micro_sd_regulator)
		return 0;

	if (mmc->slots[0].before_set_reg)
		mmc->slots[0].before_set_reg(dev, slot, power_on, vdd);

	if (power_on) {
		ret = regulator_enable(micro_sd_regulator);
		if (ret)
			printk(KERN_ERR "%s: enable regulator failed (%d)\n",
				__func__, ret);
	}
	/*
	 * HACK
	 *
	 * Default gpio GPIO_SIGNAL_SD_DET_N is powered by the same IO rail
	 * as the uSD interface. When we power down the uSD interface,
	 * we lose the reference for that GPIO. Without a reference supply,
	 * it cannot accurately detect edge transitions,
	 * leading to false (or no) interrupts.
	 *
	 * So for those using GPIO_SIGNAL_SD_DET_N, we actually keep the
	 * regulator as always on.
	 */
	else if (mmc->slots[0].switch_pin != GPIO_SIGNAL_SD_DET_N) {
		/* for the first time unbalanced disable */
		ret = regulator_is_enabled(micro_sd_regulator);
		if (ret == 1) {
			ret = regulator_disable(micro_sd_regulator);
			if (ret)
				printk(KERN_ERR "%s: disable regulator"
					" failed(%d)\n", __func__, ret);
		}
	}

	if (mmc->slots[0].after_set_reg)
		mmc->slots[0].after_set_reg(dev, slot, power_on, vdd);

	return 0;
}

static int mapphone_hsmmc_late_init(struct device *dev)
{
	int ret = 0;
	struct platform_device *pdev = container_of(dev,
				struct platform_device, dev);
	struct omap_mmc_platform_data *pdata = dev->platform_data;

	if (pdev->id == 0) {
		micro_sd_regulator = regulator_get(NULL,
			micro_sd_regulator_name);
		if (IS_ERR(micro_sd_regulator)) {
			dev_dbg(dev, "Micro SD regulator missing\n");
			micro_sd_regulator = NULL;
		}

		pdata->slots[0].set_power = microsd_set_power;
	} else if (pdev->id == 1) {
		emmc_regulator = regulator_get(NULL, emmc_regulator_name);
		if (IS_ERR(emmc_regulator)) {
			dev_dbg(dev, "eMMC Vcc regulator missing\n");
			emmc_regulator = NULL;
		}
		pdata->slots[0].set_power = emmc_set_power;
		pdata->slots[0].set_sleep = emmc_set_sleep;
	} else if (pdev->id == 2) {
		pdata->slots[0].set_power = wifi_set_power;
		printk("set_power = wifi_set_power\n");
	}
	return ret;
}

static __init void mapphone_hsmmc_set_late_init(struct device *dev)
{
	struct omap_mmc_platform_data *pdata;

	/* dev can be null if CONFIG_MMC_OMAP_HS is not set */
	if (!dev)
		return;

	pdata = dev->platform_data;
	pdata->init = mapphone_hsmmc_late_init;
}

static struct omap2_hsmmc_info mmc_controllers[] = {
	/*
	 * keep the sequence:
	 * controller for microSD, controller for eMMC, controller for wifi
	 */
	{
		.mmc		= 2,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.nonremovable	= 0,	/* MMC_UNSAFE_RESUME defined */
		.init_delay	= 50,	/* make mmc0 detect after mmc1 */
		.ocr_mask	= MMC_VDD_32_33 | MMC_VDD_33_34 |
					MMC_VDD_165_195,
	},
	{
		.mmc            = 1,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA,
		.gpio_cd        = -EINVAL,
		.gpio_wp        = -EINVAL,
		.no_off		= 1,
		.ocr_mask       = MMC_VDD_30_31,
		.nonremovable	= 1,
#ifdef CONFIG_PM_RUNTIME
		.power_saving   = true, 
#endif
	},
	/* [2]->wifi controller: set the controller id according to devtree */
	{	.mmc		= 3,
		.caps		= MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
		.gpio_cd	= -EINVAL,
		.gpio_wp	= -EINVAL,
		.ocr_mask	= MMC_VDD_165_195,
	},
	{}	/* Terminator */
};

#define MMC_CARD_CONNECT_EXTERNAL	1
#define MMC_CARD_CONNECT_INTERNAL	2
#define MMC_CARD_CONNECT_SDIO		3

#define MMC_PWR_VWLAN2	0x1
#define MMC_PWR_VSDIO	0x2
#define MMC_PWR_VSIMCARD 0x3
int __init mapphone_hsmmc_init(void)
{
	struct device_node *mmc_node;
	const void *mmc_prop;
	struct omap2_hsmmc_info *c;
	int is_found = 0;
	int sd_det_n;

	mmc_node = of_find_node_by_path(DT_PATH_MMC1);
	if (mmc_node) {
		mmc_prop = of_get_property(mmc_node,
			DT_PROP_MMC_PWR_SUPPLY, NULL);
		if (mmc_prop) {
			switch (*(int *)mmc_prop) {
			case MMC_PWR_VSDIO:
				strncpy(micro_sd_regulator_name, "vsdio",
					REG_NAME_LEN);
				break;
			case MMC_PWR_VSIMCARD:
				strncpy(micro_sd_regulator_name, "vsimcard",
					REG_NAME_LEN);
				break;
			default:
				break;
			}
		}
	}

	/* use devtree to change default emmc Vcc (VSDIO) here */

	/* Set wifi's controller id */
	mmc_node = of_find_node_by_path(DT_PATH_MMC3);
	if (mmc_node) {
		mmc_prop = of_get_property(mmc_node,
			DT_PROP_MMC_CARD_CONNECT, NULL);
			if (mmc_prop) {
				tiwlan_mmc_controller =
					*(int *)mmc_prop;
				is_found = 1;
				printk("wifi controller id = %d\n",*(int *)mmc_prop);
			}
	}

	if (!is_found) {
		tiwlan_mmc_controller = 5;
	}
	if (tiwlan_mmc_controller > 0 &&
			tiwlan_mmc_controller <= 5) {
		mmc_controllers[2].mmc = tiwlan_mmc_controller;
	} else {
		tiwlan_mmc_controller = 0;
	}
	
//	mmc_controllers[2].mmc = 3;
	/* set sd_det_n */
	sd_det_n = get_gpio_by_name("sd_det_n");
	if (sd_det_n >= 0)
		mmc_controllers[0].gpio_cd = sd_det_n;
	else
		mmc_controllers[0].gpio_cd = GPIO_SIGNAL_SD_DET_N;

	omap2_hsmmc_init(mmc_controllers);
	for (c = mmc_controllers; c->mmc; c++)
		mapphone_hsmmc_set_late_init(c->dev);

	return 0;
}
