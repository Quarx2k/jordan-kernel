/*
 * kernel/arch/arm/mach-omap2/board-mapphone-usb.c
 *
 * Copyright (C) 2010-2012 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mm.h>
#include <asm/mach-types.h>
#include <linux/of.h>
#include "dt_path.h"
#include <mach/board-mapphone.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/gpio_mapping.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <linux/usb/otg.h>
#include <linux/usb/composite.h>
#include <plat/omap-pm.h>

#if defined(CONFIG_USB_MUSB_OTG)
#include <linux/spi/cpcap.h>
#include <linux/usb/musb.h>
#include <linux/usb/composite.h>
#endif

#include <plat/common.h>
#include "cm-regbits-34xx.h"
#include "clock.h"
#include "dvfs.h"

#define MAPPHONE_BP_READY2_AP_GPIO      59
#define MAPPHONE_IPC_USB_SUSP_GPIO	142 //95
#define DIE_ID_REG_BASE			(L4_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET		0x218

void cpcap_musb_notifier_call(unsigned long event);

static struct platform_device android_usb_platform_device = {
	.name	= "android_gadget",
	.id	= -1,
	.dev	= {
	},
};

static int cpcap_usb_connected_probe(struct platform_device *pdev)
{
	printk("USB Connected!\n");
	cpcap_musb_notifier_call(USB_EVENT_VBUS);
return 0;
}

static int cpcap_usb_connected_remove(struct platform_device *pdev)
{
printk("USB Disconnected!\n");
	cpcap_musb_notifier_call(USB_EVENT_NONE);
return 0;
}

static struct platform_driver cpcap_usb_connected_driver = {
	.probe		= cpcap_usb_connected_probe,
	.remove		= cpcap_usb_connected_remove,
	.driver		= {
		.name	= "cpcap_usb_connected",
		.owner	= THIS_MODULE,
	},
};

#define USB_FUNC_NAME_SIZE     20

struct omap_usb_pid_entry {
	char name[USB_FUNC_NAME_SIZE];
	u16 usb_pid;
} __attribute__ ((__packed__));

void trim_usb_name_string(char *s)
{
int i;

	/* ignore all characters behind space key */
	for (i = 0; i < USB_FUNC_NAME_SIZE; i++) {
		if (' ' == s[i]) {
			s[i] = '\0';
			return;
		}
	}

	printk(KERN_ERR "Gadget Driver - usb function name is too long!\n");
}

#define MODELNO_MAX_LEN 16
char cmdline_modelno[MODELNO_MAX_LEN] = {};

int __init board_modelno_init(char *s)
{
	strlcpy(cmdline_modelno, s, MODELNO_MAX_LEN);
	return 1;
}
__setup("androidboot.modelno=", board_modelno_init);

void mapphone_gadget_init(void)
{
	platform_device_register(&android_usb_platform_device);
	platform_driver_register(&cpcap_usb_connected_driver);

}

/* USBHost Related  Board/Platform Data Starts Here*/

static int mapphone_usb_fsport_startup(void)
{
	int r;
	r = gpio_request(MAPPHONE_IPC_USB_SUSP_GPIO, "ipc_usb_susp");
	if (r < 0) {
		printk(KERN_WARNING "Could not request GPIO %d"
		" for IPC_USB_SUSP\n",
		MAPPHONE_IPC_USB_SUSP_GPIO);
		return r;
	}
	gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 0);
	printk(KERN_INFO "%s - Configured GPIO 95 for USB Suspend \n",
			__func__);
	return 0;
}

static int mapphone_usb_fsport_suspend(int on)
{
	pr_debug("FSUSB PHY %s\n", on ? "suspended" : "resumed");
	if (on)
		gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 1);
	else {
		gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 0);
		/* Delay 100 usec before enabling clocks */
		udelay(100);
	}

	return 0;
}

static struct clk *ipc_ext_usb_clk;

static void mapphone_hsusb_ext_ts_init(int clk_num)
{
	int gpio, ret;
	char clk_name[16];

	gpio = get_gpio_by_name("ipc_ext_usb_ts_cs");
	if (gpio >= 0) {
		ret = gpio_request(gpio, "ipc ext usb ts cs");
		if (ret) {
			printk(KERN_ERR "Cannot request GPIO %d\n", gpio);
			return;
		}
		gpio_direction_output(gpio, 1);
	}

	if (clk_num >= 0 && clk_num < 6) {
		snprintf(clk_name, sizeof(clk_name), "auxclk%d_ck", clk_num);
		ipc_ext_usb_clk = clk_get(NULL, clk_name);
		if (!ipc_ext_usb_clk) {
			printk(KERN_ERR "Cannot get auxclk%d_ck\n", clk_num);
			goto failed_clk1;
		}

		ret = clk_enable(ipc_ext_usb_clk);
		if (ret) {
			printk(KERN_ERR "Cannot enable auxclk%d_ck\n", clk_num);
			goto failed_clk2;
		}
	}

	printk(KERN_INFO "External USB Transceiver for IPC init done\n");
	return;

failed_clk2:
	clk_put(ipc_ext_usb_clk);
	ipc_ext_usb_clk = 0;
failed_clk1:
	gpio_free(gpio);
}

static struct usbhs_omap_board_data usbhs_bdata  = {
	.port_mode[0] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[1] = OMAP_USBHS_PORT_MODE_UNUSED,
	.port_mode[2] = OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM, //7, added to DT
	.phy_reset  = false,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = -EINVAL,
	.reset_gpio_port[2]  = -EINVAL,
	.ehci_phy_vbus_not_used = false,
	.es2_compatibility = true,
};


void __init mapphone_usbhost_init(void)
{
	struct device_node *node;
	const void *prop;
	int i, size;
	int feature_usbhost = 1;
	int feature_ipc_ohci_phy = 0;
	int feature_ipc_ehci_phy = 0;
	int ipc_hsusb_aux_clk = -1;

	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (node) {
		prop = of_get_property(node, "feature_usbhost_en", &size);
		if (prop && size) {
			feature_usbhost = *(u8 *)prop;
			printk(KERN_NOTICE "%s USBHost in the Platform \n",
				feature_usbhost ? "Enabling" : "Disabling");
		}
		of_node_put(node);
	}

	if (!feature_usbhost)
		return;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node) {
		prop = of_get_property(node, DT_PROP_CHOSEN_USBHS_PORT0, &size);
		if (prop && size) {
			usbhs_bdata.port_mode[0] = *(u8 *)prop;
			printk(KERN_NOTICE "Overriding USBHS Port0 Mode : %d\n",
				*(u8 *)prop);
		}
		prop = of_get_property(node, DT_PROP_CHOSEN_USBHS_PORT1, &size);
		if (prop && size) {
			usbhs_bdata.port_mode[1] = *(u8 *)prop;
			printk(KERN_NOTICE "Overriding USBHS Port1 Mode : %d\n",
				*(u8 *)prop);
		}
		prop = of_get_property(node, DT_PROP_CHOSEN_USBHS_PORT2, &size);
		if (prop && size) {
			usbhs_bdata.port_mode[2] = *(u8 *)prop;
			printk(KERN_NOTICE "Overriding USBHS Port2 Mode : %d\n",
				*(u8 *)prop);
		}
		prop = of_get_property(node,
				DT_PROP_CHOSEN_IPC_USB_TS_CLK_SRC, &size);
		if (prop && size) {
			ipc_hsusb_aux_clk = *(int *)prop;
			printk(KERN_NOTICE "Used AUX Clock Out Pin : %d\n",
				*(u8 *)prop);
		}
		of_node_put(node);
	}

	for (i = 0; i < OMAP3_HS_USB_PORTS; i++) {
		if (usbhs_bdata.port_mode[i] ==
					OMAP_OHCI_PORT_MODE_PHY_4PIN_DPDM)
			feature_ipc_ohci_phy = 1;
		else if (usbhs_bdata.port_mode[i] == OMAP_EHCI_PORT_MODE_PHY)
			feature_ipc_ehci_phy = 1;
	}

	if (feature_ipc_ohci_phy) {
		usbhs_bdata.ohci_phy_suspend = mapphone_usb_fsport_suspend;
		mapphone_usb_fsport_startup();
	}

	if (feature_ipc_ehci_phy) {
		usbhs_bdata.ehci_phy_vbus_not_used = true;
		mapphone_hsusb_ext_ts_init(ipc_hsusb_aux_clk);
		for (i = 0; i < OMAP3_HS_USB_PORTS; i++) {
			if (usbhs_bdata.port_mode[i] == OMAP_EHCI_PORT_MODE_PHY)
				usbhs_bdata.transceiver_clk[i] =
					ipc_ext_usb_clk;
		}
	}

	usbhs_init(&usbhs_bdata);
}
