/*
 * linux/arch/arm/mach-omap2/board-sholes-hsmm.c
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

#define GPIO_SIGNAL_MMC_DET 163

static const int mmc2_cd_gpio = OMAP_MAX_GPIO_LINES + 1;

static int hsmmc_card_detect(int irq)
{
	return !gpio_get_value_cansleep(GPIO_SIGNAL_MMC_DET);
}

#ifdef CONFIG_OMAP_HS_MMC2
extern int sholes_wifi_status(int irq);
#ifdef CONFIG_MMC_EMBEDDED_SDIO
extern int sholes_wifi_status_register(void (*callback)(int card_present,
						void *dev_id), void *dev_id);
#endif

static int hsmmc2_card_detect(int irq)
{
	return sholes_wifi_status(irq);
}
#endif

/*
 * MMC Slot Initialization.
 */
static struct regulator *hsmmc_regulator;
static unsigned char hsmmc_regulator_is_on;
DEFINE_MUTEX(regulator_lock);

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

	hsmmc_regulator = regulator_get(NULL, "vwlan2");
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
		}
		mutex_unlock(&regulator_lock);

		msleep(20);
		reg = omap_ctrl_readl(OMAP343X_CONTROL_PBIAS_LITE);
		reg |= (OMAP2_PBIASLITEPWRDNZ0 | OMAP2_PBIASLITEVMODE0);
		omap_ctrl_writel(reg, OMAP343X_CONTROL_PBIAS_LITE);
	}

	return 0;
}

#if defined(CONFIG_OMAP_HS_MMC2)
static int hsmmc2_late_init(struct device *dev)
{
	return 0;
}

static void hsmmc2_cleanup(struct device *dev)
{
}

static int hsmmc2_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	return 0;
}

#ifdef CONFIG_PM
static int hsmmc2_suspend(struct device *dev, int slot)
{
	int ret = 0;
	return ret;
}

static int hsmmc2_resume(struct device *dev, int slot)
{
	int ret = 0;
	return ret;
}
#endif
#endif

#if defined(CONFIG_OMAP_HS_MMC3)
static int hsmmc3_set_power(struct device *dev, int slot, int power_on,
				int vdd)
{
	/* Power to the slot is hard wired */
	return 0;
}
#endif

static struct omap_mmc_platform_data mmc1_data = {
	.nr_slots			= 1,
	.init				= hsmmc_late_init,
	.cleanup			= hsmmc_cleanup,
#ifdef CONFIG_PM
	.suspend			= hsmmc_suspend,
	.resume				= hsmmc_resume,
#endif
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wires			= 8,
		.set_power		= hsmmc_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",
		/* we get car_detect_irq later */
		.card_detect_irq	= 0,
		.card_detect            = hsmmc_card_detect,
	},
};

#if defined(CONFIG_OMAP_HS_MMC2)
#ifdef CONFIG_MMC_EMBEDDED_SDIO
static struct sdio_embedded_func wifi_func_array[] = {
	{
		.f_class        = SDIO_CLASS_NONE,
		.f_maxblksize   = 0,
	},
	{
		.f_class        = SDIO_CLASS_WLAN,
		.f_maxblksize   = 512,
	},
};

static struct embedded_sdio_data sholes_wifi_emb_data = {
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

static struct omap_mmc_platform_data mmc2_data = {
	.nr_slots			= 1,
	.init				= hsmmc2_late_init,
	.cleanup			= hsmmc2_cleanup,
#ifdef CONFIG_PM
	.suspend			= hsmmc2_suspend,
	.resume				= hsmmc2_resume,
#endif
	.dma_mask			= 0xffffffff,
#ifndef CONFIG_MMC_EMBEDDED_SDIO
	.name				= "TIWLAN_SDIO",
#endif
	.slots[0] = {
		.wires			= 4,
		.set_power		= hsmmc2_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34 |
						MMC_VDD_165_195,
		.name			= "first slot",
		.internal_clock		= 1,
		.card_detect_irq        = 0,
		.card_detect            = hsmmc2_card_detect,
#ifdef CONFIG_MMC_EMBEDDED_SDIO
		.embedded_sdio		= &sholes_wifi_emb_data,
		.register_status_notify	= &sholes_wifi_status_register,
#endif
	},
};
#endif

#if defined(CONFIG_OMAP_HS_MMC3)
static struct omap_mmc_platform_data mmc3_data = {
	.nr_slots			= 1,
	.init				= NULL,
	.cleanup			= NULL,
	.dma_mask			= 0xffffffff,
	.slots[0] = {
		.wires			= 4,
		.set_power		= hsmmc3_set_power,
		.ocr_mask		= MMC_VDD_32_33 | MMC_VDD_33_34,
		.name			= "first slot",

		.card_detect_irq        = 0,
		.card_detect            = NULL,
	},
};
#endif

static struct omap_mmc_platform_data *hsmmc_data[OMAP34XX_NR_MMC];

void __init sholes_hsmmc_init(void)
{
	hsmmc_data[0] = &mmc1_data;

	mmc1_data.slots[0].card_detect_irq = gpio_to_irq(GPIO_SIGNAL_MMC_DET);

#if defined(CONFIG_OMAP_HS_MMC2)
	hsmmc_data[1] = &mmc2_data;
#endif
#if defined(CONFIG_OMAP_HS_MMC3)
	hsmmc_data[2] = &mmc3_data;
#endif
	omap2_init_mmc(hsmmc_data, OMAP34XX_NR_MMC);
}
