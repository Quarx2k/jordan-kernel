/*
 * cpcap_usb - CPCAP USB transceiver, talking to OMAP OTG controller
 *
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/usb/otg.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/of.h>
#include <linux/of_gpio.h>

enum linkstat {
	USB_LINK_UNKNOWN = 0,
	USB_LINK_NONE,
	USB_LINK_VBUS,
	USB_LINK_ID,
};

struct cpcap_usb {
	//struct otg_transceiver	otg;
	struct usb_phy		phy;
	struct device		*dev;

	struct cpcap_device	*cpcap;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	int			irq;
	u8			linkstat;
	u8			asleep;
	bool			irq_enabled;
};

static int cpcap_set_suspend(struct usb_phy *x, int suspend)
{
	return 0;
}


static int cpcap_set_peripheral(struct usb_otg *otg,
		struct usb_gadget *gadget)
{
    struct usb_phy	*phy = otg->phy;

	otg->gadget = gadget;
	if (!gadget)
		phy->state = OTG_STATE_UNDEFINED;

	return 0;
}

static int cpcap_set_host(struct usb_otg *otg, struct usb_bus *host)
{
	struct usb_phy	*phy = otg->phy;

	otg->host = host;
	if (!host)
		phy->state = OTG_STATE_UNDEFINED;

	return 0;
}


static int cpcap_usb_setup(struct cpcap_usb *cpcap)
{
	unsigned short mask;
	unsigned short value;
	int r;

	if (cpcap->cpcap == NULL) {
		printk("Cpcap device is null, let's try again ;p\n");
		return 0;
	}

	r = cpcap_regacc_read(cpcap->cpcap, CPCAP_REG_INTS2, &value);

	if (value & CPCAP_BIT_SE1_S)
		mask = CPCAP_BIT_VBUSEN_SPI | CPCAP_BIT_VBUSPU_SPI |
			CPCAP_BIT_SUSPEND_SPI | CPCAP_BIT_ULPI_SPI_SEL;
	else
		mask = CPCAP_BIT_VBUSEN_SPI | CPCAP_BIT_VBUSPU_SPI |
			CPCAP_BIT_DMPD_SPI | CPCAP_BIT_DPPD_SPI |
			CPCAP_BIT_SUSPEND_SPI | CPCAP_BIT_PU_SPI |
			CPCAP_BIT_ULPI_SPI_SEL;

	r = cpcap_regacc_write(cpcap->cpcap, CPCAP_REG_USBC3, 0x0, mask);
	if (r < 0) {
		dev_err(cpcap->dev,
			"Can't disable SPI control of CPCAP transceiver\n");
		return r;
	}

	printk("CPCAP USB SETUP FINISH\n");
	return 0;
}
static int cpcap_usb_set_vbus(struct usb_otg *otg, bool enabled)
{
	return 0;
}
static int cpcap_usb_start_srp(struct usb_otg *otg)
{
	return 0;
}
static int  cpcap_usb_probe(struct platform_device *pdev)
{

	struct cpcap_usb	*cpcap;
	struct usb_otg			*otg;
	int err;
	cpcap = devm_kzalloc(&pdev->dev, sizeof(*cpcap), GFP_KERNEL);
	if (!cpcap) {
		dev_err(&pdev->dev, "unable to allocate memory for cpcap PHY\n");
		return -ENOMEM;
	}

	otg = devm_kzalloc(&pdev->dev, sizeof(*otg), GFP_KERNEL);
	if (!otg) {
		dev_err(&pdev->dev, "unable to allocate memory for USB OTG\n");
		return -ENOMEM;
	}

	printk("CPCAP USB PROBE START\n");

	cpcap->dev		= &pdev->dev;
	cpcap->phy.dev		= cpcap->dev;
	cpcap->phy.label		= "cpcap";
	cpcap->phy.set_suspend	= cpcap_set_suspend;
	cpcap->phy.otg		= otg;
	cpcap->phy.type		= USB_PHY_TYPE_USB2;
	cpcap->asleep			= 1;
	cpcap->cpcap			= pdev->dev.platform_data;


	otg->set_host		= cpcap_set_host;
	otg->set_peripheral	= cpcap_set_peripheral;
	otg->set_vbus		= cpcap_usb_set_vbus;
	otg->start_srp		= cpcap_usb_start_srp;
	otg->phy		= &cpcap->phy;

	printk("CPCAP USB PROBE usb_add_phy_dev\n");
	spin_lock_init(&cpcap->lock);
	usb_add_phy_dev(&cpcap->phy);

	printk("CPCAP USB PROBE platform_set_drvdata\n");
	platform_set_drvdata(pdev, cpcap);

	printk("CPCAP USB PROBE cpcap_usb_setup\n");
	err = cpcap_usb_setup(cpcap);
	if (err < 0)
		goto err0;
	printk("CPCAP USB PROBE END\n");
	return 0;

err0:
	printk("ERROR IN CPCAP USB PROBE\n");
	usb_remove_phy(&cpcap->phy);
	devm_kfree(&pdev->dev,cpcap);
	devm_kfree(&pdev->dev,otg);
	return err;
}

static int  cpcap_usb_remove(struct platform_device *pdev)
{
	struct cpcap_usb *cpcap = platform_get_drvdata(pdev);
	devm_kfree(&pdev->dev,cpcap);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id cpcap_usb_id_table[] = {
	{ .compatible = "mot,cpcap-usb" },
	{}
};
MODULE_DEVICE_TABLE(of, cpcap_usb_id_table);
#endif

static const struct platform_device_id cpcap_usb_platform_id_table[] = {
	{"cpcap-usb", 0},
	{},
};
MODULE_DEVICE_TABLE(of, cpcap_usb_platform_id_table);


static struct platform_driver cpcap_usb_driver = {
	.probe		= cpcap_usb_probe,
	.remove		= (cpcap_usb_remove),
	.driver		= {
		.name	= "cpcap_usb",
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(cpcap_usb_id_table),
#endif
	},
};

module_platform_driver(cpcap_usb_driver);

MODULE_ALIAS("platform:cpcap_usb");
MODULE_DESCRIPTION("CPCAP USB transceiver driver");
MODULE_LICENSE("GPL");
