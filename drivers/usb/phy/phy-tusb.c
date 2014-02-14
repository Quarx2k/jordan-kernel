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

static bool factory_override;

struct tusb_usb {
	struct usb_phy		phy;
	struct device		*dev;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct wake_lock wake_lock;
	int irq_gpio;
	int num_gpios;
	struct gpio *gpio_list;
};

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
	int i;

	if (vbus_state) {
		atomic_notifier_call_chain(&tusb->phy.notifier,
			USB_EVENT_VBUS, NULL);
	} else {
		atomic_notifier_call_chain(&tusb->phy.notifier,
			USB_EVENT_NONE, NULL);
	}

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
	for (i = 0; i < tusb->num_gpios; i++) {
		if (!(tusb->gpio_list[i].flags & GPIOF_DIR_IN)) {
			bool default_level =
				!!(tusb->gpio_list[i].flags & GPIOF_INIT_HIGH);
			gpio_set_value(tusb->gpio_list[i].gpio,
			    (vbus_state | factory_override) ^ default_level);
		}
	}

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

	return 0;
}

static DEVICE_BOOL_ATTR(factory_override, 0664, factory_override);

static int  tusb_usb_probe(struct platform_device *pdev)
{
	struct tusb_usb	*tusb;
	struct usb_otg	*otg;
	struct device_node *np;
	int i, ret, irqnum = -1;

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
				goto free_gpios;
			}
		}
	}

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg) {
		dev_err(&pdev->dev, "unable to allocate memory for tusb OTG\n");
		ret = -ENOMEM;
		goto free_irq;
	}

	wake_lock_init(&tusb->wake_lock, WAKE_LOCK_SUSPEND, "tusb");

	tusb->workqueue = create_workqueue("tusb");
	INIT_WORK(&tusb->work, tusb_usb_work_func);

	if (irqnum >= 0) {
		ret = enable_irq_wake(irqnum);
		if (ret) {
			dev_err(&pdev->dev, "unable to enable irq wake\n");
			ret = -EINVAL;
			goto cancel_work;
		}
	} else {
		dev_err(&pdev->dev, "no IRQ configured\n");
		goto cancel_work;
	}

	ret = device_create_file(&pdev->dev, &(dev_attr_factory_override.attr));
	if (ret != 0) {
		dev_err(&pdev->dev, "sysfs file creation failed.\n");
		goto disable_irq;
	}

	tusb->dev		= &pdev->dev;
	tusb->phy.dev		= tusb->dev;
	tusb->phy.label		= "tusb";
	tusb->phy.set_suspend	= tusb_set_suspend;
	tusb->phy.otg		= otg;
	tusb->phy.type		= USB_PHY_TYPE_USB2;
	ATOMIC_INIT_NOTIFIER_HEAD(&tusb->phy.notifier);

	otg->set_host		= tusb_set_host;
	otg->set_peripheral	= tusb_set_peripheral;
	otg->set_vbus		= tusb_usb_set_vbus;
	otg->start_srp		= tusb_usb_start_srp;
	otg->phy		= &tusb->phy;

	usb_add_phy_dev(&tusb->phy);

	platform_set_drvdata(pdev, tusb);

	return 0;

disable_irq:
	disable_irq_wake(irqnum);
cancel_work:
	cancel_work_sync(&tusb->work);
	destroy_workqueue(tusb->workqueue);
	wake_lock_destroy(&tusb->wake_lock);
free_irq:
	free_irq(irqnum, tusb);
free_gpios:
	gpio_free_array(tusb->gpio_list, tusb->num_gpios);

	return ret;
}

static int tusb_usb_remove(struct platform_device *pdev)
{
	struct tusb_usb	*tusb = platform_get_drvdata(pdev);
	int irqnum = gpio_to_irq(tusb->irq_gpio);

	device_remove_file(&pdev->dev, &(dev_attr_factory_override.attr));
	disable_irq_wake(irqnum);
	cancel_work_sync(&tusb->work);
	destroy_workqueue(tusb->workqueue);

	if (wake_lock_active(&tusb->wake_lock))
		wake_unlock(&tusb->wake_lock);
	wake_lock_destroy(&tusb->wake_lock);
	free_irq(irqnum, tusb);
	gpio_free_array(tusb->gpio_list, tusb->num_gpios);

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

static int __init tusb_usb_init(void)
{
	return platform_driver_register(&tusb_usb_driver);
}
subsys_initcall(tusb_usb_init);

static void __exit tusb_usb_exit(void)
{
	platform_driver_unregister(&tusb_usb_driver);
}
module_exit(tusb_usb_exit);

MODULE_ALIAS("platform:tusb_usb stub");
MODULE_DESCRIPTION("TUSB transceiver driver");
MODULE_LICENSE("GPL");
