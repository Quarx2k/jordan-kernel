/*
 * Toggles a GPIO pin to power down minnow device
 *
 *
 * Copyright (C) 2014 Motorola Mobility LLC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/pinctrl/consumer.h>
#include <linux/usb.h>

#define NUM_GPIOS 1
/*
 * Hold configuration here, cannot be more than one instance of the driver
 * since pm_power_off itself is global.
 */

static int g_gpio_wdi = -1;
static int g_usb_connected = -1;
static struct pinctrl_state *gp_outputstate;
static struct pinctrl_state *gp_tristate;
static struct pinctrl *gp_pctrl;

static void minnow_gpio_poweroff_do_poweroff(void)
{
	BUG_ON(!gpio_is_valid(g_gpio_wdi));

	pr_info("%s, Turning off pmic\n", __func__);

	/* check for USB cable present, if so, wait for
	 some time before going ahead. */

	/* TODO : Check USB status before sleeping
	Please see IKXCLOCK-138*/
	/* sleep for 500 ms */
	msleep(500);

	/* config gpio 143 back from safe mode to reset the device */
	pinctrl_select_state(gp_pctrl, gp_outputstate);

	/* drive it active, also inactive->active edge */
	gpio_direction_output(g_gpio_wdi, 1);

	do {} while (1);

	WARN_ON(1);
}
/* This is not being used right now, will be used once
 IKXCLOCK-138 is implemented */
int pmic_usb_ncb(struct notifier_block *nb, unsigned long val,
			void *priv)
{
	int result = NOTIFY_OK;
	switch (val) {
	case USB_DEVICE_ADD:
	case USB_BUS_ADD:
		g_usb_connected = 1;
		break;
	case USB_DEVICE_REMOVE:
	case USB_BUS_REMOVE:
		g_usb_connected = 0;
		break;
	}
	return result;
}

static struct notifier_block pmic_usb_notifier = {
	.notifier_call = pmic_usb_ncb,
	.priority = INT_MAX /* Need to be called first of all */
};

static int minnow_gpio_poweroff_probe(struct platform_device *pdev)
{
	int ret;
	struct device_node *np;
	enum of_gpio_flags flags;

	/* Lets register right away for usb notifications */
	usb_register_notify(&pmic_usb_notifier);

	/* If a pm_power_off function has already been added, leave it alone */
	if (pm_power_off != NULL) {
		pr_err("%s: pm_power_off function already registered",
		       __func__);
		goto unregister_usb;
	}

	np = pdev->dev.of_node;
	if (!np) {
		pr_err("%s: devtree data not found\n", __func__);
		goto unregister_usb;
	}

	if (of_gpio_count(np) != NUM_GPIOS) {
		pr_err("%s: gpio count is not %d.\n", __func__, NUM_GPIOS);
		goto unregister_usb;
	}

	g_gpio_wdi = of_get_gpio_flags(np, 0, &flags);

	ret = gpio_request_one(g_gpio_wdi, flags,
					"minnow_poweroff-gpio-wdi");
	if (ret) {
		pr_err("%s: Can't request gpios err: %d\n", __func__, ret);
		goto unregister_usb;
	}

	gp_pctrl = devm_pinctrl_get(&(pdev->dev));
	if (IS_ERR(gp_pctrl)) {
		ret = PTR_ERR(gp_pctrl);
		pr_err("%s: no pinctrl handle\n", __func__);
		goto err;
	}

	gp_outputstate = pinctrl_lookup_state(gp_pctrl, "output");
	if (IS_ERR(gp_outputstate)) {
		ret = PTR_ERR(gp_outputstate);
		pr_err("%s: Can't obtain output pinctrl state\n", __func__);
		goto err;
	}

	gp_tristate = pinctrl_lookup_state(gp_pctrl, "tristate");
	if (IS_ERR(gp_tristate)) {
		ret = PTR_ERR(gp_tristate);
		pr_err("%s: Can't obtain tristate pinctrl state\n", __func__);
		goto err;
	}

	ret = pinctrl_select_state(gp_pctrl, gp_tristate);
	if (ret) {
		pr_err("%s: failed to set tristate\n", __func__);
		ret = -EINVAL;
		goto err;
	}


	pm_power_off = &minnow_gpio_poweroff_do_poweroff;
	return 0;

err:
	gpio_free(g_gpio_wdi);
unregister_usb:
	usb_unregister_notify(&pmic_usb_notifier);
	return -ENODEV;
}

static int minnow_gpio_poweroff_remove(struct platform_device *pdev)
{
	gpio_free(g_gpio_wdi);
	if (pm_power_off == &minnow_gpio_poweroff_do_poweroff)
		pm_power_off = NULL;

	usb_unregister_notify(&pmic_usb_notifier);

	return 0;
}

static const struct of_device_id of_minnow_gpio_poweroff_match[] = {
	{ .compatible = "mot,pmic-wdi", },
	{},
};

static struct platform_driver minnow_gpio_poweroff_driver = {
	.probe = minnow_gpio_poweroff_probe,
	.remove = minnow_gpio_poweroff_remove,
	.driver = {
		.name = "minnow-poweroff-gpio",
		.owner = THIS_MODULE,
		.of_match_table = of_minnow_gpio_poweroff_match,
	},
};

module_platform_driver(minnow_gpio_poweroff_driver);

MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("Minnow GPIO poweroff driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:poweroff-gpio");
