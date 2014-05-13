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

#define NUM_GPIOS 2
/*
 * Hold configuration here, cannot be more than one instance of the driver
 * since pm_power_off itself is global.
 */

static int g_gpio_wdi = -1;
static int g_gpio_sys_resetb = -1;
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
	if (g_usb_connected)
		msleep(500);

	/* config gpio 143 back from safe mode to reset the device */
	pinctrl_select_state(gp_pctrl, gp_outputstate);

	/* drive it active, also inactive->active edge */
	gpio_direction_output(g_gpio_wdi, 1);

	do {} while (1);

	WARN_ON(1);
}

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
		/* During poweroff, the USB stack deinits and sends a remove
		 notification even if a USB cable is still plugged in. To
		 address this limitation, once USB cable is plugged in, always
		 assume cable will be left plugged in up until powerdown. This
		 should not impact regular user use-case scenarios since they
		 would never be using USB cable. */
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
	int i, ret;
	struct device_node *np;

	g_usb_connected = 0;
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

	/* Make sure number of GPIOs defined matches the supplied number of
	 * GPIO name strings.
	 */
	if (of_property_count_strings(np, "gpio-names") != NUM_GPIOS) {
		dev_err(&pdev->dev, "GPIO info and name mismatch\n");
		goto unregister_usb;
	}

	for (i = 0; i < NUM_GPIOS; i++) {
		int gpio;
		enum of_gpio_flags flags;
		const char *label;

		gpio = of_get_gpio_flags(np, i, &flags);
		of_property_read_string_index(np, "gpio-names", i, &label);
		ret = gpio_request_one(gpio, flags, label);
		if (ret)
			goto free_gpios;

		if (!strcmp(label, "wdi"))
			g_gpio_wdi = gpio;
		else if (!strcmp(label, "sys_reset"))
			g_gpio_sys_resetb = gpio;
		else {
			dev_info(&pdev->dev, "Unknown gpio: %s (gpio-%d)\n",
				 label, gpio);
			goto free_gpios;
		}
		dev_info(&pdev->dev, "%s: gpio-%d  flags: 0x%x\n",
			 label, gpio, flags);
	}

	gp_pctrl = devm_pinctrl_get(&(pdev->dev));
	if (IS_ERR(gp_pctrl)) {
		ret = PTR_ERR(gp_pctrl);
		pr_err("%s: no pinctrl handle\n", __func__);
		goto free_gpios;
	}

	gp_outputstate = pinctrl_lookup_state(gp_pctrl, "output");
	if (IS_ERR(gp_outputstate)) {
		ret = PTR_ERR(gp_outputstate);
		pr_err("%s: Can't obtain output pinctrl state\n", __func__);
		goto free_gpios;
	}

	gp_tristate = pinctrl_lookup_state(gp_pctrl, "tristate");
	if (IS_ERR(gp_tristate)) {
		ret = PTR_ERR(gp_tristate);
		pr_err("%s: Can't obtain tristate pinctrl state\n", __func__);
		goto free_gpios;
	}

	ret = pinctrl_select_state(gp_pctrl, gp_tristate);
	if (ret) {
		pr_err("%s: failed to set tristate\n", __func__);
		goto free_gpios;
	}


	pm_power_off = &minnow_gpio_poweroff_do_poweroff;
	return 0;

free_gpios:
	if (gpio_is_valid(g_gpio_wdi))
		gpio_free(g_gpio_wdi);
	if (gpio_is_valid(g_gpio_sys_resetb))
		gpio_free(g_gpio_sys_resetb);
unregister_usb:
	usb_unregister_notify(&pmic_usb_notifier);
	return -ENODEV;
}

static int minnow_gpio_poweroff_remove(struct platform_device *pdev)
{
	if (gpio_is_valid(g_gpio_wdi))
		gpio_free(g_gpio_wdi);
	if (gpio_is_valid(g_gpio_sys_resetb))
		gpio_free(g_gpio_sys_resetb);
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
