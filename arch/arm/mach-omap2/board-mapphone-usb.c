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

static char device_serial[MAX_USB_SERIAL_NUM];



static char *usb_functions_ums[] = {
	"usb_mass_storage",
};

static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_rndis[] = {
	"rndis",
};

static char *usb_functions_rndis_adb[] = {
	"rndis",
	"adb",
};

static char *usb_functions_all[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	"acm",
	"usbnet",
	"mtp",
#elif defined(CONFIG_USB_ANDROID_ACM)
	"acm",
#endif
#ifdef CONFIG_USB_ANDROID_RNDIS
	"rndis",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_SERIAL_VIATELECOM_CBP
	"ets",
#endif
};

#ifdef CONFIG_USB_MOT_ANDROID
static char *usb_functions_phone_portal[] = {
	"acm",
	"usbnet",
	"mtp",
};

static char *usb_functions_phone_portal_adb[] = {
	"acm",
	"usbnet",
	"mtp",
	"adb",
};

static char *usb_functions_mtp[] = {
	"mtp",
};

static char *usb_functions_mtp_adb[] = {
	"mtp",
	"adb",
};
#endif


static struct android_usb_product usb_products[] = {
#ifdef CONFIG_USB_MOT_ANDROID
	{
		.product_id     = MAPPHONE_PHONE_PORTAL_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_phone_portal),
		.functions      = usb_functions_phone_portal,
	},
	{
		.product_id     = MAPPHONE_PHONE_PORTAL_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_phone_portal_adb),
		.functions      = usb_functions_phone_portal_adb,
	},
	{
		.product_id     = MAPPHONE_MTP_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp),
		.functions      = usb_functions_mtp,
	},
	{
		.product_id     = MAPPHONE_MTP_ADB_PRODUCT_ID,
		.num_functions  = ARRAY_SIZE(usb_functions_mtp_adb),
		.functions      = usb_functions_mtp_adb,
	},
#endif
	{
		.product_id	= MAPPHONE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums),
		.functions	= usb_functions_ums,
	},
	{
		.product_id	= MAPPHONE_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
#ifdef CONFIG_USB_ANDROID_RNDIS
	{
		.product_id	= MAPPHONE_RNDIS_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis),
		.functions	= usb_functions_rndis,
	},
	{
		.product_id	= MAPPHONE_RNDIS_ADB_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_rndis_adb),
		.functions	= usb_functions_rndis_adb,
	},
#endif
};



/* standard android USB platform data */
static struct android_usb_platform_data andusb_plat = {
	.vendor_id      = 0x22b8,
	.product_id     = 0x41DA,
	.product_name   = "A853",
	.manufacturer_name	= "Motorola",
	.serial_number		= device_serial,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
};


static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data	= &andusb_plat,
	},
};

static struct usb_mass_storage_platform_data usbms_plat = {
	.vendor			= "Motorola",
	.product		= "A853",
	.release		= 1,
	.nluns			= 1,
	.cdrom_lun_num		= 1,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &usbms_plat,
	},
};


#ifdef CONFIG_USB_ANDROID_RNDIS
static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID	= 0x22b8,
	.vendorDescr	= "Motorola",
};

static struct platform_device rndis_device = {
	.name	= "rndis",
	.id	= -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};
#endif


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

static struct platform_driver cpcap_usb_connected_driver = {
	.probe		= cpcap_usb_connected_probe,
	.remove		= cpcap_usb_connected_remove,
	.driver		= {
		.name	= "cpcap_usb_connected",
		.owner	= THIS_MODULE,
	},
};

#ifdef CONFIG_ARM_OF
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

void __init usb_pid_mapping_init(void)
{
	struct device_node *node;
	const void *prop;
	int i, size, unit_size;
	char name[USB_FUNC_NAME_SIZE];

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		printk(KERN_ERR
			"Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}

	unit_size = sizeof(struct omap_usb_pid_entry);
	prop = of_get_property(node, DT_PROP_CHOSEN_USB_PIDS, &size);
	if ((!prop) || (size % unit_size)) {
		printk(KERN_ERR "Read property %s error!\n",
			DT_PROP_CHOSEN_USB_PIDS);
			of_node_put(node);
		return;
	}

	for (i = 0; i < size / unit_size; i++) {
		struct omap_usb_pid_entry *p =
		(struct omap_usb_pid_entry *) prop;

		memcpy((void *) name, p->name, USB_FUNC_NAME_SIZE);
		trim_usb_name_string(name);
		android_usb_set_pid(name, p->usb_pid);
		prop += unit_size;
	}

	of_node_put(node);
	printk(KERN_INFO "DT overwrite of  USB PID's done!\n");
}

void mapphone_init_cdrom_lun_num(void)
{
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		 return;
	}
	prop = of_get_property(node, DT_PROP_CHOSEN_USB_CDROM_LUN_NUM, NULL);
	if (prop) {
		pr_err("USB Overwrite nLuns %d\n", *(char *)prop);
		usbms_plat.cdrom_lun_num = *(char *)prop;
	}

	of_node_put(node);
	return;
}

void mapphone_init_nluns(void)
{
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}
	prop = of_get_property(node, DT_PROP_CHOSEN_USB_NLUNS, NULL);
	if (prop) {
		pr_err("USB Overwrite nLuns %d\n", *(char *)prop);
		usbms_plat.nluns = *(char *)prop;
	}

	of_node_put(node);
	return;
}

void mapphone_get_product_name(void)
{
	struct device_node *node;
	const void *prop;

	node = of_find_node_by_path(DT_PATH_CHOSEN);
	if (node == NULL) {
		pr_err("Unable to read node %s from device tree!\n",
			DT_PATH_CHOSEN);
		return;
	}

	prop = of_get_property(node, DT_PROP_CHOSEN_USB_PROD_NAME, NULL);
	if (prop) {
		andusb_plat.product_name = (char *)prop;
		usbms_plat.product = (char *)prop;
	} else {
		pr_err("Read property %s error!\n",
		       DT_PROP_CHOSEN_USB_PROD_NAME);
	}

	of_node_put(node);
	return;
}

#endif

void mapphone_get_serial_number(void)
{
	unsigned int val[2];
	unsigned int reg;
#ifdef CONFIG_USB_ANDROID_RNDIS
	int i;
	char *src;
#endif

	reg = DIE_ID_REG_BASE + DIE_ID_REG_OFFSET;
	val[0] = omap_readl(reg);
	val[1] = omap_readl(reg + 4);

	snprintf(device_serial, MAX_USB_SERIAL_NUM, "%08X%08X", val[1], val[0]);
#ifdef CONFIG_USB_ANDROID_RNDIS
	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	src = device_serial;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}
#endif
}


void mapphone_gadget_init(void)
{
	mapphone_get_serial_number();
#ifdef CONFIG_ARM_OF
	mapphone_get_product_name();
	/* Initialize the USB nluns from device tree */
	mapphone_init_nluns();
	mapphone_init_cdrom_lun_num();
	/* Initialize the USB PID's from the device tree */
	usb_pid_mapping_init();
#endif
	platform_device_register(&usb_mass_storage_device);
#ifdef CONFIG_USB_ANDROID_RNDIS
	platform_device_register(&rndis_device);
#endif
	platform_device_register(&androidusb_device);
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
#endif /* OHCI specific data */

#ifdef CONFIG_USB_QSC6085_CDMA_MODEM 
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
#endif

void __init mapphone_ehci_init(void)
{
	if (!strcmp(boot_mode, "charger"))
		return;

	omap_cfg_reg(AF5_34XX_GPIO142);		/*  IPC_USB_SUSP      */
	omap_cfg_reg(AD1_3430_USB3FS_PHY_MM3_RXRCV);
	omap_cfg_reg(AD2_3430_USB3FS_PHY_MM3_TXDAT);
	omap_cfg_reg(AC1_3430_USB3FS_PHY_MM3_TXEN_N);
	omap_cfg_reg(AE1_3430_USB3FS_PHY_MM3_TXSE0);

#ifdef CONFIG_USB_SERIAL_VIATELECOM_CBP
	if (mapphone_bp_get_type() == MAPPHONE_BP_VIACBP71) {
		printk(KERN_INFO "VIA BP is chosen\n");
		ohci_device.dev.platform_data  = &dummy_usb_config_via;
	}
#endif
#ifdef CONFIG_USB_QSC6085_CDMA_MODEM 
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
