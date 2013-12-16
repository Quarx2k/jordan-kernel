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
#include <linux/module.h>

struct tusb_usb {
	struct usb_phy		phy;
	struct device		*dev;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct wake_lock wake_lock;
	int			irq;
	int			irq_gpio;
	int			resetn_gpio;
	int			cs_gpio;
	int			csn_gpio;
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
	int vbus_stat = gpio_get_value(tusb->irq_gpio);

	if (vbus_stat) {
		if (tusb->resetn_gpio >= 0)
			gpio_set_value(tusb->resetn_gpio, 1);
		if (tusb->cs_gpio >= 0)
			gpio_set_value(tusb->cs_gpio, 1);
		if (tusb->csn_gpio >= 0)
			gpio_set_value(tusb->csn_gpio, 0);
	} else {
		if (tusb->resetn_gpio >= 0)
			gpio_set_value(tusb->resetn_gpio, 0);
		if (tusb->cs_gpio >= 0)
			gpio_set_value(tusb->cs_gpio, 0);
		if (tusb->csn_gpio >= 0)
			gpio_set_value(tusb->csn_gpio, 1);
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

static int  tusb_usb_probe(struct platform_device *pdev)
{
	struct tusb_usb	*tusb;
	struct usb_otg	*otg;
	struct device_node *np;
	int gpio;

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
	tusb->irq = -1;
	tusb->irq_gpio = -1;
	tusb->resetn_gpio = -1;
	tusb->cs_gpio = -1;
	tusb->csn_gpio = -1;

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg) {
		dev_err(&pdev->dev, "unable to allocate memory for tusb OTG\n");
		return -ENOMEM;
	}

	wake_lock_init(&tusb->wake_lock, WAKE_LOCK_SUSPEND, "tusb");

	tusb->workqueue = create_workqueue("tusb");
	INIT_WORK(&tusb->work, tusb_usb_work_func);

	if (!of_property_read_u32(np, "resetn-gpio", &gpio)) {
		if (!gpio_request(gpio, "tusb-resetn")) {
			if (!gpio_direction_output(gpio, 1))
				tusb->resetn_gpio = gpio;
			else
				gpio_free(gpio);
		}
	}

	if (!of_property_read_u32(np, "cs-gpio", &gpio)) {
		if (!gpio_request(gpio, "tusb-cs")) {
			if (!gpio_direction_output(gpio, 1))
				tusb->cs_gpio = gpio;
			else
				gpio_free(gpio);
		}
	}

	if (!of_property_read_u32(np, "csn-gpio", &gpio)) {
		if (!gpio_request(gpio, "tusb-csn")) {
			if (!gpio_direction_output(gpio, 0))
				tusb->csn_gpio = gpio;
			else
				gpio_free(gpio);
		}
	}

	if (!of_property_read_u32(np, "irq-gpio", &gpio)) {
		tusb->irq_gpio = gpio;
		if (!gpio_request(tusb->irq_gpio, "tusb-irq")) {
			if (!gpio_direction_input(tusb->irq_gpio)) {
				tusb->irq = gpio_to_irq(tusb->irq_gpio);
				irq_set_irq_type(tusb->irq,
						 IRQ_TYPE_EDGE_RISING |
						 IRQ_TYPE_EDGE_FALLING);

				if (request_irq(tusb->irq, tusb_usb_isr,
						IRQF_DISABLED |
						IRQF_TRIGGER_RISING,
						"tsub-irq", tusb)) {
					gpio_free(tusb->irq_gpio);
					tusb->irq = -1;
					tusb->irq_gpio = -1;
				}
			} else {
				gpio_free(tusb->irq_gpio);
			}
		}
	}
	if (tusb->irq == -1)
		dev_info(&pdev->dev, "Unable to get enable irq;"
				     " no interrupt support\n");

	tusb->dev		= &pdev->dev;
	tusb->phy.dev		= tusb->dev;
	tusb->phy.label		= "tusb";
	tusb->phy.set_suspend	= tusb_set_suspend;
	tusb->phy.otg		= otg;
	tusb->phy.type		= USB_PHY_TYPE_USB2;


	otg->set_host		= tusb_set_host;
	otg->set_peripheral	= tusb_set_peripheral;
	otg->set_vbus		= tusb_usb_set_vbus;
	otg->start_srp		= tusb_usb_start_srp;
	otg->phy		= &tusb->phy;

	usb_add_phy_dev(&tusb->phy);

	platform_set_drvdata(pdev, tusb);

	return 0;
}

static int tusb_usb_remove(struct platform_device *pdev)
{
	struct tusb_usb	*tusb = platform_get_drvdata(pdev);


	if (tusb->irq >= 0) {
		disable_irq_wake(tusb->irq);
		free_irq(tusb->irq, tusb);
		gpio_free(tusb->irq_gpio);
	}

	if (tusb->csn_gpio >= 0)
		gpio_free(tusb->csn_gpio);
	if (tusb->cs_gpio >= 0)
		gpio_free(tusb->csn_gpio);
	if (tusb->resetn_gpio >= 0)
		gpio_free(tusb->csn_gpio);

	cancel_work_sync(&tusb->work);
	destroy_workqueue(tusb->workqueue);

	if (wake_lock_active(&tusb->wake_lock))
		wake_unlock(&tusb->wake_lock);
	wake_lock_destroy(&tusb->wake_lock);

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
