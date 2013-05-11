/*
 * kernel/arch/arm/mach-omap2/board-mapphone-usb.c
 *
 * Copyright (C) 2010-2011 Motorola, Inc.
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

#include <plat/board-mapphone.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <linux/gpio_mapping.h>
#include <plat/mux.h>
#include <plat/usb.h>
#include <linux/usb/android_composite.h>


#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#include "cm-regbits-34xx.h"
#include "clock.h"


#define MAPPHONE_BP_READY2_AP_GPIO      59
#define MAPPHONE_IPC_USB_SUSP_GPIO	142
#define DIE_ID_REG_BASE			(L4_WK_34XX_PHYS + 0xA000)
#define DIE_ID_REG_OFFSET		0x218
#define MAX_USB_SERIAL_NUM		17
#define MAPPHONE_VENDOR_ID		0x22B8
#define MAPPHONE_PRODUCT_ID		0x41D9
#define MAPPHONE_ADB_PRODUCT_ID		0x41DB
#define MAPPHONE_RNDIS_PRODUCT_ID	0x41E4
#define MAPPHONE_RNDIS_ADB_PRODUCT_ID	0x41E5
#define FACTORY_PRODUCT_ID		0x41E3
#define FACTORY_ADB_PRODUCT_ID		0x41E2

#ifdef CONFIG_USB_MOT_ANDROID
#define MAPPHONE_PHONE_PORTAL_PRODUCT_ID               0x41D8
#define MAPPHONE_PHONE_PORTAL_ADB_PRODUCT_ID           0x41DA
#define MAPPHONE_MTP_PRODUCT_ID                        0x41D6
#define MAPPHONE_MTP_ADB_PRODUCT_ID                    0x41DC
#endif

#define BOOT_MODE_MAX_LEN 30
static char boot_mode[BOOT_MODE_MAX_LEN+1];
int __init board_boot_mode_init(char *s)
{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);
	printk(KERN_INFO "boot_mode=%s\n", boot_mode);
	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);

static int cpcap_usb_connected_probe(struct platform_device *pdev)
{
	android_usb_set_connected(1);
	return 0;
}

static int cpcap_usb_connected_remove(struct platform_device *pdev)
{
	android_usb_set_connected(0);
	return 0;
}

static struct platform_device android_usb_platform_device = {
	.name	= "android_gadget",
	.id	= -1,
	.dev	= {
	},
};

static struct platform_driver cpcap_usb_connected_driver = {
	.probe		= cpcap_usb_connected_probe,
	.remove		= cpcap_usb_connected_remove,
	.driver		= {
		.name	= "cpcap_usb_connected",
		.owner	= THIS_MODULE,
	},
};


void mapphone_gadget_init(void)
{
	platform_device_register(&android_usb_platform_device);
	platform_driver_register(&cpcap_usb_connected_driver);
}


#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)

static int mapphone_usb_port_startup(struct platform_device *dev, int port)
{
	int r;

	if (port == 2) {
		r = gpio_request(MAPPHONE_IPC_USB_SUSP_GPIO, "ipc_usb_susp");
		if (r < 0) {
			printk(KERN_WARNING "Could not request GPIO %d"
			       " for IPC_USB_SUSP\n",
			       MAPPHONE_IPC_USB_SUSP_GPIO);
			return r;
		}
		gpio_direction_output(MAPPHONE_IPC_USB_SUSP_GPIO, 0);
	} else {
		return -EINVAL;
	}
	return 0;
}

static void mapphone_usb_port_shutdown(struct platform_device *dev, int port)
{
	if (port == 2)
		gpio_free(MAPPHONE_IPC_USB_SUSP_GPIO);
}


static void mapphone_usb_port_suspend(struct platform_device *dev,
				    int port, int suspend)
{
	if (port == 2)
		gpio_set_value(MAPPHONE_IPC_USB_SUSP_GPIO, suspend);
}

static int omap_usbhost_bus_check_ctrl_standby(void);
static struct ehci_hcd_omap_platform_data usb_platform_data = {
	.port_data = {
		{ .flags = 0x0, }, /* disabled */
		{ .flags = 0x0, }, /* disabled */
		{
			.flags = EHCI_HCD_OMAP_FLAG_ENABLED |
			EHCI_HCD_OMAP_FLAG_AUTOIDLE |
			EHCI_HCD_OMAP_FLAG_NOBITSTUFF,
			.mode = EHCI_HCD_OMAP_MODE_UTMI_PHY_4PIN,
			.startup = mapphone_usb_port_startup,
			.shutdown = mapphone_usb_port_shutdown,
			.suspend = mapphone_usb_port_suspend,
		},
	},
	.usbhost_standby_status = omap_usbhost_bus_check_ctrl_standby,
};

static struct resource ehci_resources[] = {
	{
		.start	= OMAP34XX_EHCI_BASE,
		.end	= OMAP34XX_EHCI_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP34XX_UHH_CONFIG_BASE,
		.end	= OMAP34XX_UHH_CONFIG_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP34XX_USBTLL_BASE,
		.end	= OMAP34XX_USBTLL_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{         /* general IRQ */
		.start	= INT_34XX_EHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 ehci_dmamask = ~(u32)0;
static struct platform_device ehci_device = {
	.name		= "ehci-omap",
	.id		= 0,
	.dev = {
		.dma_mask		= &ehci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data		= &usb_platform_data,
	},
	.num_resources	= ARRAY_SIZE(ehci_resources),
	.resource	= ehci_resources,
};
#endif

static int omap_usbhost_bus_check_ctrl_standby(void)
{
	u32 val;

	val = cm_read_mod_reg(OMAP3430ES2_USBHOST_MOD, CM_IDLEST);
	if (val & OMAP3430ES2_ST_USBHOST_STDBY_MASK)
		return 1;
	else
		return 0;
}

#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)


static struct resource ohci_resources[] = {
	[0] = {
		.start	= OMAP34XX_OHCI_BASE,
		.end	= OMAP34XX_OHCI_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP34XX_UHH_CONFIG_BASE,
		.end	= OMAP34XX_UHH_CONFIG_BASE + SZ_1K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= OMAP34XX_USBTLL_BASE,
		.end	= OMAP34XX_USBTLL_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{         /* general IRQ */
		.start	= INT_34XX_OHCI_IRQ,
		.flags	= IORESOURCE_IRQ,
	}
};

static u64 ohci_dmamask = ~(u32)0;

#ifdef CONFIG_USB_SERIAL_VIATELECOM_CBP
static struct omap_usb_config dummy_usb_config_via = {
       .port_data = {
		{
			.flags = EHCI_HCD_OMAP_FLAG_ENABLED |
			EHCI_HCD_OMAP_FLAG_AUTOIDLE |
			EHCI_HCD_OMAP_FLAG_NOBITSTUFF,
			.mode = EHCI_HCD_OMAP_MODE_UTMI_TLL_2PIN,
		},
		{ .flags = 0x0, }, /* disabled */
		{ .flags = 0x0, }, /* disabled */
	},
};
#endif

static struct omap_usb_config dummy_usb_config = {
	.port_data = {
		{ .flags = 0x0, }, /* disabled */
		{ .flags = 0x0, }, /* disabled */
		{
			.flags = EHCI_HCD_OMAP_FLAG_ENABLED |
			EHCI_HCD_OMAP_FLAG_AUTOIDLE |
			EHCI_HCD_OMAP_FLAG_NOBITSTUFF,
			.mode = EHCI_HCD_OMAP_MODE_UTMI_PHY_4PIN,
			.startup = mapphone_usb_port_startup,
			.shutdown = mapphone_usb_port_shutdown,
			.suspend = mapphone_usb_port_suspend,
		},
	},
	.usbhost_standby_status = omap_usbhost_bus_check_ctrl_standby,
	.usb_remote_wake_gpio = MAPPHONE_BP_READY2_AP_GPIO,
};

static struct platform_device ohci_device = {
	.name		= "ohci",
	.id		= 0,
	.dev = {
		.dma_mask		= &ohci_dmamask,
		.coherent_dma_mask	= 0xffffffff,
		.platform_data	= &dummy_usb_config,
	},
	.num_resources	= ARRAY_SIZE(ohci_resources),
	.resource	= ohci_resources,
};


extern void set_cdma_modem_interface(unsigned int number);

void mapphone_init_modem_interface(void)
{
	struct device_node *node;
	const void *prop;
	int rwkup_gpio = get_gpio_by_name("bp2ap_usb_rwkup");

	if (rwkup_gpio < 0)
		dummy_usb_config.usb_remote_wake_gpio =
			MAPPHONE_BP_READY2_AP_GPIO;
	else
		dummy_usb_config.usb_remote_wake_gpio = rwkup_gpio;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}
	prop = of_get_property(node, DT_PROP_CHOSEN_MODEM_IFACE_NUM, NULL);
	if (prop) {
		pr_err("Setting the Modem Interface num to %d\n", *(u8 *)prop);
		set_cdma_modem_interface(*(u8 *)prop);
	} else
		set_cdma_modem_interface(0);

	of_node_put(node);
	return;
}
#endif /* OHCI specific data */

void __init mapphone_ehci_init(void)
{
	if (!strcmp(boot_mode, "charger"))
		return;

#ifdef CONFIG_USB_SERIAL_VIATELECOM_CBP
	if (mapphone_bp_get_type() == MAPPHONE_BP_VIACBP71) {
		printk(KERN_INFO "VIA BP is chosen\n");
		ohci_device.dev.platform_data  = &dummy_usb_config_via;
	}
#endif
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	if (is_cdma_phone())
		mapphone_init_modem_interface();
#endif
#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_EHCI_HCD_MODULE)
	if (!is_cdma_phone()) {
		usb_platform_data.port_data[2].mode =
			EHCI_HCD_OMAP_MODE_ULPI_TLL_SDR;
		platform_device_register(&ehci_device);
	}
#endif
#if defined(CONFIG_USB_OHCI_HCD) || defined(CONFIG_USB_OHCI_HCD_MODULE)
	if (is_cdma_phone())
		platform_device_register(&ohci_device);
#endif
}
