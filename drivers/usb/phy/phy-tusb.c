/*
 * tusb - TI TUSB transceiver driver, talking to OMAP OTG controller
 *
 * Copyright (C) 2013 Motorola Mobility
 * Copyright (C) 2004-2007 Texas Instruments
 * Copyright (C) 2008 Nokia Corporation
 * Copyright (C) 2009 Google, Inc.
 * Contact: Erik Gilling <konkers@android.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 * Based on twl4030-usb.c
 *
 */

#include <linux/platform_device.h>
#include <linux/usb/otg.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/power_supply.h>
#include <linux/usb/musb-omap.h>

static bool factory_override;

static enum power_supply_property tusb_psy_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

struct tusb_usb {
	struct usb_phy		phy;
	struct device		*dev;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct wake_lock wake_lock;
	int irq_gpio;
	int num_gpios;
	struct gpio *gpio_list;
	struct clk *clk_in;
	bool clk_in_en;
	struct power_supply psy;
	/* device lock used for setting vbus  gpio */
	spinlock_t lock;
};

static int tusb_get_property(struct power_supply *psy,
		enum power_supply_property psp, union power_supply_propval *val)
{
	struct tusb_usb *tusb = container_of(psy, struct tusb_usb, psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = gpio_get_value(tusb->irq_gpio);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int tusb_set_suspend(struct usb_phy *x, int suspend)
{
	return 0;
}

static int tusb_set_peripheral(struct usb_otg *otg,
		struct usb_gadget *gadget)
{
	struct usb_phy	*phy = otg->phy;

	otg->gadget = gadget;
	if (!gadget)
		phy->state = OTG_STATE_UNDEFINED;

	return 0;
}

static int tusb_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct usb_phy	*phy = otg->phy;

	otg->host = host;
	if (!host)
		phy->state = OTG_STATE_UNDEFINED;

	return 0;
}

static int tusb_usb_enable_clkin(struct tusb_usb *tusb, bool enable)
{
	int r = 0;
	if ((tusb->clk_in_en != enable) && (tusb->clk_in != NULL)) {
		if (enable)
			r = clk_prepare_enable(tusb->clk_in);
		else
			clk_disable_unprepare(tusb->clk_in);
		if (!r)
			tusb->clk_in_en = enable;
	}
	return r;
}

static int tusb_enable(struct tusb_usb *tusb, bool enable)
{
	int i;
	unsigned long flags;

	dev_info(tusb->dev, "USB Reset [%s]\n", enable ? "Enable" : "Disable");

	spin_lock_irqsave(&tusb->lock, flags);

	/* enable external clock at first */
	if (enable)
		tusb_usb_enable_clkin(tusb, true);

	for (i = 0; i < tusb->num_gpios; i++) {
		if (!(tusb->gpio_list[i].flags & GPIOF_DIR_IN)) {
			bool default_level =
				!!(tusb->gpio_list[i].flags & GPIOF_INIT_HIGH);
			gpio_set_value(tusb->gpio_list[i].gpio,
			    enable ^ default_level);
		}
	}

	/* disable external clock at last */
	if (!enable)
		tusb_usb_enable_clkin(tusb, false);

	spin_unlock_irqrestore(&tusb->lock, flags);

	return 0;
}

static int tusb_phy_reset(struct usb_phy *x, int enable)
{
	int retval = 0;
	struct tusb_usb	*tusb = dev_get_drvdata(x->dev);
	bool vbus_state = gpio_get_value(tusb->irq_gpio);

	dev_info(tusb->dev, "USB: VBUS state %d\n", vbus_state);

	/* only do this when  vbus is on */
	if (vbus_state)
		retval = tusb_enable(tusb, (bool)enable);

	return retval;
}

static int tusb_usb_set_vbus(struct usb_otg *otg, bool enabled)
{
	return 0;
}

static int tusb_usb_start_srp(struct usb_otg *otg)
{
	return 0;
}

static void tusb_usb_work_func(struct work_struct *work)
{
	struct tusb_usb *tusb =
		container_of(work, struct tusb_usb, work);
	bool vbus_state = gpio_get_value(tusb->irq_gpio);

	if (vbus_state) {
		atomic_notifier_call_chain(&tusb->phy.notifier,
			USB_EVENT_VBUS, NULL);
		tusb->phy.last_event = USB_EVENT_VBUS;
		omap_musb_mailbox(OMAP_MUSB_VBUS_VALID);
	} else {
		atomic_notifier_call_chain(&tusb->phy.notifier,
			USB_EVENT_NONE, NULL);
		tusb->phy.last_event = USB_EVENT_NONE;
		omap_musb_mailbox(OMAP_MUSB_VBUS_OFF);
	}

	power_supply_changed(&tusb->psy);

	/* Output GPIOs are driven based on the vbus_state input
	   and the factory_override flag:

		VBUS | FactoryOverride | Output
		-----|-----------------|-------
		  0  |        0        | default (off)
		  0  |        1        | !default (on)
		  1  |        0        | !default (on)
		  1  |        1        | !default (on)
	 */
	dev_info(tusb->dev, "%s USB phy [%d/%d]\n",
		 (vbus_state | factory_override) ? "Enable" : "Disable",
		 vbus_state, factory_override);

	tusb_enable(tusb, (vbus_state | factory_override));

	if (wake_lock_active(&tusb->wake_lock))
		wake_unlock(&tusb->wake_lock);
}

static irqreturn_t tusb_usb_isr(int irq, void *data)
{
	struct tusb_usb *tusb = data;
	wake_lock(&tusb->wake_lock);
	queue_work(tusb->workqueue, &tusb->work);

	return IRQ_HANDLED;
}

static int tusb_parse_of(struct platform_device *pdev, struct tusb_usb *tusb)
{
	int gpio_count;
	int i;
	struct device_node *np = pdev->dev.of_node;
	enum of_gpio_flags flags;
	char *clkin;

	if (!tusb)
		return -EINVAL;

	gpio_count = of_gpio_count(np);

	if (!gpio_count) {
		dev_err(&pdev->dev, "No GPIOS defined in device tree\n");
		return -EINVAL;
	}

	/* Make sure number of GPIOs defined matches the supplied number of
	 * GPIO name strings.
	 */
	if (gpio_count != of_property_count_strings(np, "gpio-names")) {
		dev_err(&pdev->dev, "GPIO info and name mismatch\n");
		return -EINVAL;
	}

	tusb->gpio_list = devm_kzalloc(&pdev->dev,
				sizeof(struct gpio) * gpio_count,
				GFP_KERNEL);
	if (!tusb->gpio_list)
		return -ENOMEM;

	tusb->num_gpios = gpio_count;
	for (i = 0; i < gpio_count; i++) {
		tusb->gpio_list[i].gpio = of_get_gpio_flags(np, i, &flags);
		tusb->gpio_list[i].flags = flags;
		of_property_read_string_index(np, "gpio-names", i,
					      &tusb->gpio_list[i].label);

		dev_dbg(&pdev->dev, "%s: gpio-%d  flags: 0x%lx\n",
			tusb->gpio_list[i].label, tusb->gpio_list[i].gpio,
			tusb->gpio_list[i].flags);
	}

	clkin = (char *)of_get_property(np, "clk_in", NULL);
	if (clkin) {
		tusb->clk_in = clk_get(NULL, clkin);
		if (IS_ERR(tusb->clk_in)) {
			dev_err(&pdev->dev,
				"Failed get external clock %s!\n", clkin);
			i = PTR_ERR(tusb->clk_in);
			tusb->clk_in = NULL;
			return i;
		}
	}

	return 0;
}

static DEVICE_BOOL_ATTR(factory_override, 0664, factory_override);

static int  tusb_usb_probe(struct platform_device *pdev)
{
	struct tusb_usb	*tusb;
	struct usb_otg	*otg;
	struct device_node *np;
	struct power_supply *psy;
	int i, ret, irqnum = -1;
	int size, count;
	const char *string;

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "devtree data not found\n");
		return -EINVAL;
	}
	tusb = devm_kzalloc(&pdev->dev, sizeof(*tusb), GFP_KERNEL);
	if (!tusb) {
		dev_err(&pdev->dev, "unable to allocate memory for tusb PHY\n");
		return -ENOMEM;
	}
	factory_override = 0;

	ret = tusb_parse_of(pdev, tusb);
	if (ret) {
		dev_err(&pdev->dev, "Error parsing device tree\n");
		return ret;
	}

	ret = gpio_request_array(tusb->gpio_list, tusb->num_gpios);
	if (ret) {
		dev_err(&pdev->dev, "failed to request GPIOs\n");
		return ret;
	}

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg) {
		dev_err(&pdev->dev, "unable to allocate memory for tusb OTG\n");
		ret = -ENOMEM;
		goto free_gpios;
	}

	wake_lock_init(&tusb->wake_lock, WAKE_LOCK_SUSPEND, "tusb");

	tusb->workqueue = create_workqueue("tusb");
	if (!tusb->workqueue) {
		dev_err(&pdev->dev, "unable to create workqueue\n");
		ret = -ENOMEM;
		goto destroy_wakelock;
	}
	INIT_WORK(&tusb->work, tusb_usb_work_func);

	ret = device_create_file(&pdev->dev, &(dev_attr_factory_override.attr));
	if (ret != 0) {
		dev_err(&pdev->dev, "sysfs file creation failed.\n");
		goto remove_workqueue;
	}

	tusb->dev		= &pdev->dev;
	tusb->phy.dev		= tusb->dev;
	tusb->phy.label		= "tusb";
	tusb->phy.set_suspend	= tusb_set_suspend;
	tusb->phy.hw_reset	= tusb_phy_reset;
	tusb->phy.otg		= otg;
	tusb->phy.type		= USB_PHY_TYPE_USB2;
	tusb->phy.last_event	= USB_EVENT_NONE;
	ATOMIC_INIT_NOTIFIER_HEAD(&tusb->phy.notifier);

	otg->set_host		= tusb_set_host;
	otg->set_peripheral	= tusb_set_peripheral;
	otg->set_vbus		= tusb_usb_set_vbus;
	otg->start_srp		= tusb_usb_start_srp;
	otg->phy		= &tusb->phy;

	ret = usb_add_phy_dev(&tusb->phy);
	if (ret != 0) {
		dev_err(&pdev->dev, "declaring USB phy failed.\n");
		goto remove_file;
	}

	psy = &tusb->psy;
	psy->name = "ac";
	psy->type = POWER_SUPPLY_TYPE_MAINS;
	psy->properties = tusb_psy_properties;
	psy->num_properties = ARRAY_SIZE(tusb_psy_properties);
	psy->get_property = tusb_get_property;

	count = of_property_count_strings(np, "supplied_to");
	if (count > 0) {
		size = count * sizeof(*psy->supplied_to);
		psy->supplied_to = devm_kzalloc(&pdev->dev, size,
						     GFP_KERNEL);
		if (!psy->supplied_to) {
			dev_err(&pdev->dev, "Failed to alloc supplied_to\n");
			goto remove_file;
		}

		/* Make copies of the DT strings for const-correctness */
		for (i = 0; i < count; i++) {
			if (of_property_read_string_index(np, "supplied_to", i,
							  &string)) {
				dev_err(&pdev->dev, "Failed to read supplied_to"
					" supplied_to[%d]\n", i);
				goto remove_file;
			}
			psy->supplied_to[i] = kstrdup(string,  GFP_KERNEL);
			if (!psy->supplied_to[i]) {
				dev_err(&pdev->dev, "Failed to alloc space for"
					" supplied_to[%d]\n", i);
				goto remove_file;
			}
		}
		psy->num_supplicants = count;
	}

	ret = power_supply_register(&pdev->dev, psy);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to register power supply\n");
		goto remove_phy;
	}

	for (i = 0; i < tusb->num_gpios; i++) {
		if (!strcmp(tusb->gpio_list[i].label, "tusb-irq")) {
			tusb->irq_gpio = tusb->gpio_list[i].gpio;
			irqnum = gpio_to_irq(tusb->irq_gpio);
			ret = request_irq(irqnum,
					tusb_usb_isr,
					IRQF_DISABLED |
					IRQF_TRIGGER_RISING |
					IRQF_TRIGGER_FALLING,
					"tsub-irq", tusb);
			if (ret) {
				dev_err(&pdev->dev, "failed to get irq\n");
				goto unregister_power_supply;
			}
		}
	}

	if (irqnum >= 0) {
		ret = enable_irq_wake(irqnum);
		if (ret) {
			dev_err(&pdev->dev, "unable to enable irq wake\n");
			ret = -EINVAL;
			goto free_irq;
		}
	} else {
		dev_err(&pdev->dev, "no IRQ configured\n");
		goto remove_phy;
	}

	platform_set_drvdata(pdev, tusb);

	/* initialize PHY to correct state */
	queue_work(tusb->workqueue, &tusb->work);

	spin_lock_init(&tusb->lock);

	return 0;

free_irq:
	free_irq(irqnum, tusb);
	cancel_work_sync(&tusb->work);
unregister_power_supply:
	power_supply_unregister(psy);
remove_phy:
	usb_remove_phy(&tusb->phy);
remove_file:
	device_remove_file(&pdev->dev, &(dev_attr_factory_override.attr));
remove_workqueue:
	destroy_workqueue(tusb->workqueue);
destroy_wakelock:
	wake_lock_destroy(&tusb->wake_lock);
free_gpios:
	gpio_free_array(tusb->gpio_list, tusb->num_gpios);

	return ret;
}

static int tusb_usb_remove(struct platform_device *pdev)
{
	struct tusb_usb	*tusb = platform_get_drvdata(pdev);
	int irqnum = gpio_to_irq(tusb->irq_gpio);

	free_irq(irqnum, tusb);
	cancel_work_sync(&tusb->work);
	power_supply_unregister(&tusb->psy);
	usb_remove_phy(&tusb->phy);
	device_remove_file(&pdev->dev, &(dev_attr_factory_override.attr));
	destroy_workqueue(tusb->workqueue);
	wake_lock_destroy(&tusb->wake_lock);
	gpio_free_array(tusb->gpio_list, tusb->num_gpios);

	platform_set_drvdata(pdev, NULL);

	return 0;
}
#ifdef CONFIG_OF
static const struct of_device_id tusb_usb_id_table[] = {
	{ .compatible = "ti,tusb-usb" },
	{}
};
MODULE_DEVICE_TABLE(of, tusb_usb_id_table);
#endif

static const struct platform_device_id tusb_usb_platform_id_table[] = {
	{"tusb-usb", 0},
	{},
};
MODULE_DEVICE_TABLE(of, tusb_usb_platform_id_table);

static struct platform_driver tusb_usb_driver = {
	.probe		= tusb_usb_probe,
	.remove		= (tusb_usb_remove),
	.driver		= {
		.name	= "tusb_usb",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(tusb_usb_id_table),
#endif
	},
	.id_table = tusb_usb_platform_id_table,
};

module_platform_driver(tusb_usb_driver);

MODULE_ALIAS("platform:tusb_usb stub");
MODULE_DESCRIPTION("TUSB transceiver driver");
MODULE_LICENSE("GPL");
