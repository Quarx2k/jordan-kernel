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
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/regulator/consumer.h>

#include <plat/hardware.h>
#include <plat/control.h>
#include <plat/mmc.h>
#include <plat/board.h>
#include <plat/gpio.h>
#include <plat/mux.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#define GPIO_SIGNAL_MMC_DET 163

static const int mmc2_cd_gpio = OMAP_MAX_GPIO_LINES + 1;

static int hsmmc_card_detect(int irq)
{
	return !gpio_get_value_cansleep(GPIO_SIGNAL_MMC_DET);
}

extern int mapphone_wifi_status(int irq);
#ifdef CONFIG_MMC_EMBEDDED_SDIO
extern int mapphone_wifi_status_register(void (*callback)(int card_present,
						void *dev_id), void *dev_id);
#endif

static int wifi_sdio_detect(int irq)
{
	return mapphone_wifi_status(irq);
}

/*
 * MMC Slot Initialization.
 */
static struct regulator *hsmmc_regulator;
static unsigned char hsmmc_regulator_is_on;
DEFINE_MUTEX(regulator_lock);

#define REG_NAME_LEN	10
static char hsmmc_regulator_name[REG_NAME_LEN] = "vwlan2";

static int hsmmc_late_init(struct device *dev)
{
	int ret = 0;

	/*
	 * Configure GPIO parameters for MMC hotplug irq
	 */
	ret = gpio_request(GPIO_SIGNAL_MMC_DET, "mmc_detect");
	if (ret < 0)
		goto err;
	ret = gpio_direction_input(GPIO_SIGNAL_MMC_DET);
	if (ret < 0)
		goto err2;
	hsmmc_regulator = regulator_get(NULL, hsmmc_regulator_name);
	if (IS_ERR(hsmmc_regulator)) {
		dev_dbg(dev, "vwlan2 regulator missing\n");
		ret = PTR_ERR(hsmmc_regulator);
		goto err2;
	}

	return ret;
err2:
	gpio_free(GPIO_SIGNAL_MMC_DET);
err:
	dev_err(dev, "Failed to configure GPIO MMC_DET\n");
	return ret;
}

static void hsmmc_cleanup(struct device *dev)
{
	gpio_free(GPIO_SIGNAL_MMC_DET);
	if (hsmmc_regulator)
		regulator_put(hsmmc_regulator);
}

#ifdef CONFIG_PM
/*
 * To mask and unmask MMC Card Detect Interrupt
 * mask : 1
 * unmask : 0
 */
static int mask_cd_interrupt(int mask)
{
	return 0;
}

static int hsmmc_suspend(struct device *dev, int slot)
{
	int ret = 0;

	disable_irq(gpio_to_irq(GPIO_SIGNAL_MMC_DET));
	ret = mask_cd_interrupt(1);

	return ret;
}

static int hsmmc_resume(struct device *dev, int slot)
{
	int ret = 0;

	enable_irq(gpio_to_irq(GPIO_SIGNAL_MMC_DET));
	ret = mask_cd_interrupt(0);

	return ret;
}
#endif

static int hsmmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	u32 reg;
	int ret = 0;

	if (power_on) {
		reg = omap_ctrl_readl(OMAP2_CONTROL_DEVCONF0);
		reg |= OMAP2_MMCSDIO1ADPCLKISEL;
		omap_ctrl_writel(reg, OMAP2_CONTROL_DEVCONF0);

		reg = omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE);
		reg &= ~OMAP2_PBIASSPEEDCTRL0;
		reg &= ~OMAP2_PBIASLITEPWRDNZ0;
		omap_ctrl_writel(reg, OMAP343X_CONTROL_PBIAS_LITE);

		mutex_lock(&regulator_lock);
		if (!hsmmc_regulator_is_on) {
			hsmmc_regulator_is_on = 1;
			regulator_enable(hsmmc_regulator);
			msleep(4);
		}
		mutex_unlock(&regulator_lock);

		reg = omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE);
		reg |= OMAP2_PBIASLITEPWRDNZ0;
		if ((1 << vdd) <= MMC_VDD_165_195)
			reg &= ~OMAP2_PBIASLITEVMODE0;
		else
			reg |= OMAP2_PBIASLITEVMODE0;
		omap_ctrl_writel(reg, OMAP343X_CONTROL_PBIAS_LITE);

		return ret;
	} else {
		reg = omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE);
		reg &= ~OMAP2_PBIASLITEPWRDNZ0;
		omap_ctrl_writel(reg, OMAP343X_CONTROL_PBIAS_LITE);

		mutex_lock(&regulator_lock);
		if (hsmmc_regulator_is_on) {
			hsmmc_regulator_is_on = 0;
			regulator_disable(hsmmc_regulator);
			msleep(20);
		}
		mutex_unlock(&regulator_lock);

		reg = omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE);
		reg |= (OMAP2_PBIASLITEPWRDNZ0 | OMAP2_PBIASLITEVMODE0);
		omap_ctrl_writel(reg, OMAP343X_CONTROL_PBIAS_LITE);
	}

	return 0;
}

#ifdef CONFIG_MMC_TEST_INSERT_REMOVE
int ex_hsmmc_set_power(struct device *dev, int slot, int power_on,
						int vdd)
{
    hsmmc_set_power(dev, slot, power_on, vdd);
    return 0;
}
EXPORT_SYMBOL(ex_hsmmc_set_power);
#endif

static int emmc_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	unsigned long reg;
	/* No control on VIO */
	/* OMAP2_MMCSDIO2ADPCLKISEL:
	 * 	MMC/SDI/O2 Module Input Clock selection
	 * 	0x0: Input clock is from the external pin
	 * 	0x1: Internal loop-back, module input clock is
	 * 		copied from the module output clock
	 */
	/* fixed to 1.8V, so no need to setup PBIAS */
	/* also set in omap2_mmc_mux() */
	reg = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1);
	reg |= OMAP2_MMCSDIO2ADPCLKISEL;
	omap_ctrl_writel(reg, OMAP343X_CONTROL_DEVCONF1);

	return 0;
}

static int wifi_late_init(struct device *dev)
{
	return 0;
}

static void wifi_cleanup(struct device *dev)
{
}

static int wifi_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	return 0;
}

#ifdef CONFIG_PM
static int wifi_suspend(struct device *dev, int slot)
{
	int ret = 0;
	return ret;
}

static int wifi_resume(struct device *dev, int slot)
{
	int ret = 0;
	return ret;
}
#endif

static struct omap_mmc_platform_data mmc1_data __initdata = {
	.nr_slots			= 1,
	.init				= hsmmc_late_init,
	.cleanup			= hsmmc_cleanup,
#ifdef CONFIG_PM
	.suspend			= hsmmc_suspend,
	.resume				= hsmmc_resume,
#endif
	.dma_mask			= 0xffffffff,
	.init_delay			= 50,
	.slots[0] = {
		.wires			= 4,
		.nonremovable		= 0,	/* MMC_UNSAFE_RESUME defined */
		.power_saving		= 0,
		.set_power		= hsmmc_set_power,
		.name			= "first slot",
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		/* we get car_detect_irq later */
		.card_detect_irq	= 0,
		.card_detect            = hsmmc_card_detect,
	},
};

#if defined(CONFIG_OMAP_HS_MMC2)
static struct omap_mmc_platform_data emmc_data __initdata = {
	.nr_slots			= 1,
	.init				= NULL,
	/* no detection gpio/regulator to setup */
	.cleanup			= NULL,
	.dma_mask			= 0xffffffff,
	/* suspend/resume will be added after bringup */
	.slots[0] = {
		.wires			= 8,
		.nonremovable		= 1,
		.power_saving		= 1,
		.no_off			= 1,
		.set_power		= emmc_set_power,  /* must have */
		.ocr_mask		= MMC_VDD_165_195,
		.name			= "first slot",
		.internal_clock		= 1,
		.card_detect_irq        = 0,
		.card_detect            = NULL,
		/* define .set_sleep to cut Vcc and leave VccQ alone */
	},
};
#endif

#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct sdio_embedded_func wifi_func_array[] __initdata = {
	{
		.f_class        = SDIO_CLASS_NONE,
		.f_maxblksize   = 0,
	},
	{
		.f_class        = SDIO_CLASS_WLAN,
		.f_maxblksize   = 512,
	},
};

static struct embedded_sdio_data mapphone_wifi_emb_data __initdata = {
	.cis    = {
		.vendor         = 0x104c,
		.device         = 0x9066,
		.blksize        = 512,
		.max_dtr        = 24000000,
	},
	.cccr   = {
		.multi_block    = 1,
		.low_speed      = 0,
		.wide_bus       = 1,
		.high_power     = 0,
		.high_speed     = 0,
	},
	.funcs  = wifi_func_array,
	.num_funcs = 2,
};
#endif

static struct omap_mmc_platform_data wifi_data __initdata = {
	.nr_slots			= 1,
	.init				= wifi_late_init,
	.cleanup			= wifi_cleanup,
#ifdef CONFIG_PM
	.suspend			= wifi_suspend,
	.resume				= wifi_resume,
#endif
	.dma_mask			= 0xffffffff,
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.name				= "TIWLAN_SDIO",
#endif
	.slots[0] = {
		.wires			= 4,
		.set_power		= wifi_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",
		.internal_clock		= 1,
		.card_detect_irq        = 0,
		.card_detect            = wifi_sdio_detect,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio		= &mapphone_wifi_emb_data,
		.register_status_notify	= &mapphone_wifi_status_register,
#endif
	},
};

static struct omap_mmc_platform_data *hsmmc_data[OMAP34XX_NR_MMC] __initdata;

#define MMC_CARD_CONNECT_EXTERNAL	1
#define MMC_CARD_CONNECT_INTERNAL	2
#define MMC_CARD_CONNECT_SDIO		3

#define MMC_PWR_VWLAN2	0x1
#define MMC_PWR_VSDIO	0x2

void __init mapphone_hsmmc_init(void)
{
	struct device_node *mmc_node;
	const void *mmc_prop;

#ifdef CONFIG_ARM_OF
	mmc_node = of_find_node_by_path(DT_PATH_MMC1);
	if (mmc_node) {
		mmc_prop = of_get_property(mmc_node,
			DT_PROP_MMC_PWR_SUPPLY, NULL);
		if (mmc_prop) {
			switch (*(int *)mmc_prop) {
			case MMC_PWR_VSDIO:
				strncpy(hsmmc_regulator_name, "vsdio",
					REG_NAME_LEN);
				break;
			default:
				break;
			}
		}
	}
#endif
	hsmmc_data[0] = &mmc1_data;

	mmc1_data.slots[0].card_detect_irq = gpio_to_irq(GPIO_SIGNAL_MMC_DET);

#if defined(CONFIG_OMAP_HS_MMC2)
#ifdef CONFIG_ARM_OF
	mmc_node = of_find_node_by_path(DT_PATH_MMC2);
	if (mmc_node) {
		mmc_prop = of_get_property(mmc_node,
			DT_PROP_MMC_CARD_CONNECT, NULL);
		if (mmc_prop) {
			if (*(int *)mmc_prop == MMC_CARD_CONNECT_SDIO)
				hsmmc_data[1] = &wifi_data;
			else if (*(int *)mmc_prop == MMC_CARD_CONNECT_INTERNAL)
				hsmmc_data[1] = &emmc_data;
		}
	} else
		hsmmc_data[1] = &wifi_data;
#else
	hsmmc_data[1] = &wifi_data;
#endif
#endif

#if defined(CONFIG_OMAP_HS_MMC3)
#ifdef CONFIG_ARM_OF
	mmc_node = of_find_node_by_path(DT_PATH_MMC3);
	if (mmc_node) {
		mmc_prop = of_get_property(mmc_node,
			DT_PROP_MMC_CARD_CONNECT, NULL);
		if (mmc_prop) {
			if (*(int *)mmc_prop == MMC_CARD_CONNECT_SDIO)
				hsmmc_data[2] = &wifi_data;
		}
	}
#endif
#endif
	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);
}
