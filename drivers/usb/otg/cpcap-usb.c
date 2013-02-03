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
#include <linux/slab.h>

enum linkstat {
	USB_LINK_UNKNOWN = 0,
	USB_LINK_NONE,
	USB_LINK_VBUS,
	USB_LINK_ID,
};

struct cpcap_usb {
	struct otg_transceiver	otg;
	struct device		*dev;

	struct cpcap_device	*cpcap;

	/* for vbus reporting with irqs disabled */
	spinlock_t		lock;

	int			irq;
	u8			linkstat;
	u8			asleep;
	bool			irq_enabled;
};

static int cpcap_set_suspend(struct otg_transceiver *x, int suspend)
{
	return 0;
}

static int cpcap_set_power(struct otg_transceiver *x, unsigned int mA)
{
	struct cpcap_usb *cpcap;

	if (!x)
		return -ENODEV;

	cpcap = dev_get_drvdata(x->dev);
	cpcap_batt_set_usb_prop_curr(cpcap->cpcap, mA);

	return 0;
}

static int cpcap_set_peripheral(struct otg_transceiver *x,
		struct usb_gadget *gadget)
{
	struct cpcap_usb *cpcap;

	if (!x)
		return -ENODEV;

	cpcap = dev_get_drvdata(x->dev);
	cpcap->otg.gadget = gadget;
	if (!gadget)
		cpcap->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int cpcap_set_host(struct otg_transceiver *x, struct usb_bus *host)
{
	struct cpcap_usb *cpcap;

	if (!x)
		return -ENODEV;

	cpcap = dev_get_drvdata(x->dev);
	cpcap->otg.host = host;
	if (!host)
		cpcap->otg.state = OTG_STATE_UNDEFINED;

	return 0;
}

static int cpcap_usb_setup(struct cpcap_usb *cpcap)
{
	unsigned short mask;
	unsigned short value;
	int r;

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
	return 0;
}
static int __init cpcap_usb_probe(struct platform_device *pdev)
{
	struct cpcap_usb	*cpcap;
	int err;
	printk("CPCAP usb probe entered\n");
	cpcap = kzalloc(sizeof *cpcap, GFP_KERNEL);
	if (!cpcap)
		return -ENOMEM;

	cpcap->dev			= &pdev->dev;
	cpcap->otg.dev			= cpcap->dev;
	cpcap->otg.label		= "cpcap";
	cpcap->otg.set_host		= cpcap_set_host;
	cpcap->otg.set_peripheral	= cpcap_set_peripheral;
	cpcap->otg.set_suspend		= cpcap_set_suspend;
	cpcap->otg.set_power		= cpcap_set_power;
	cpcap->asleep			= 1;
	cpcap->cpcap			= pdev->dev.platform_data;

	/* init spinlock for workqueue */
	spin_lock_init(&cpcap->lock);

	ATOMIC_INIT_NOTIFIER_HEAD(&cpcap->otg.notifier);

	otg_set_transceiver(&cpcap->otg);

	platform_set_drvdata(pdev, cpcap);

	err = cpcap_usb_setup(cpcap);
	if (err < 0)
		goto err0;

	dev_info(&pdev->dev, "Initialized CPCAP USB module\n");
	return 0;

err0:
	otg_set_transceiver(NULL);
	kfree(cpcap);
	return err;
}

static int __exit cpcap_usb_remove(struct platform_device *pdev)
{
	struct cpcap_usb *cpcap = platform_get_drvdata(pdev);
	kfree(cpcap);

	return 0;
}

static struct platform_driver cpcap_usb_driver = {
	.probe		= cpcap_usb_probe,
	.remove		= __exit_p(cpcap_usb_remove),
	.driver		= {
		.name	= "cpcap_usb",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_usb_init(void)
{
	return cpcap_driver_register(&cpcap_usb_driver);
}
subsys_initcall(cpcap_usb_init);

static void __exit cpcap_usb_exit(void)
{
	platform_driver_unregister(&cpcap_usb_driver);
}
module_exit(cpcap_usb_exit);




MODULE_ALIAS("platform:cpcap_usb");
MODULE_DESCRIPTION("CPCAP USB transceiver driver");
MODULE_LICENSE("GPL");
