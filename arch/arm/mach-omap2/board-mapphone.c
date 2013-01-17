/*
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bootmem.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/input.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/mtd/nand.h>
#include <linux/of_fdt.h>
#include <linux/of.h>
#include <linux/led-lm3530.h>
#include <linux/wl12xx.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/board.h>
#include <plat/gpmc-smc91x.h>
#include <plat/usb.h>
#include <plat/system.h>
#include <plat/mux.h>
#include <plat/hdq.h>

#include <mach/board-mapphone.h>

#include "board-flash.h"
#include "mux.h"
#include "sdram-toshiba-hynix-numonyx.h"
#include "omap_ion.h"
#include "dt_path.h"
#include "pm.h"
#include "hsmmc.h"

#ifdef CONFIG_EMU_UART_DEBUG
#include <plat/board-mapphone-emu_uart.h>
#endif
#include <../drivers/w1/w1_family.h> /* for W1_EEPROM_DS2502 */

#define MAPPHONE_POWER_OFF_GPIO 176
#define MAPPHONE_WIFI_PMENA_GPIO 186
#define MAPPHONE_WIFI_IRQ_GPIO 65

char *bp_model = "CDMA";
static char boot_mode[BOOT_MODE_MAX_LEN+1];

int __init board_boot_mode_init(char *s)
{
	strncpy(boot_mode, s, BOOT_MODE_MAX_LEN);
	boot_mode[BOOT_MODE_MAX_LEN] = '\0';
	pr_debug("boot_mode=%s\n", boot_mode);
	return 1;
}
__setup("androidboot.mode=", board_boot_mode_init);

/* Flat dev tree address */
#define ATAG_FLAT_DEV_TREE_ADDRESS 0xf100040A
struct tag_flat_dev_tree_address {
	u32 address;
	u32 size;
};

static u32 fdt_start_address;
static u32 fdt_size;

/* process flat device tree for hardware configuration */
static int __init parse_tag_flat_dev_tree_address(const struct tag *tag)
{
	struct tag_flat_dev_tree_address *fdt_addr =
		(struct tag_flat_dev_tree_address *)&tag->u;

	if (fdt_addr->size) {
		fdt_start_address = (u32)phys_to_virt(fdt_addr->address);
		fdt_size = fdt_addr->size;
	}

	/*have_of = 1;*/
	printk(KERN_INFO
		"flat_dev_tree_virt_address=0x%08x, flat_dev_tree_address=0x%08x, flat_dev_tree_size == 0x%08X\n",
		fdt_start_address,
		fdt_addr->address,
		fdt_addr->size);

	return 0;
}

__tagtable(ATAG_FLAT_DEV_TREE_ADDRESS, parse_tag_flat_dev_tree_address);

static struct attribute *mapphone_properties_attrs[] = {
	//&mapphone_virtual_keys_attr.attr,
	NULL,
};

static struct attribute_group mapphone_properties_attr_group = {
	.attrs = mapphone_properties_attrs,
};

static struct omap2_hdq_platform_config mapphone_hdq_data = {
	.mode = OMAP_SDQ_MODE,
	.id = W1_EEPROM_DS2502,
};

static int __init omap_hdq_init(void)
{
	omap_hdq_dev.dev.platform_data = &mapphone_hdq_data;
	return platform_device_register(&omap_hdq_dev);
}

static struct wl12xx_platform_data mapphone_wlan_data __initdata = {
	.irq = -1, /* OMAP_GPIO_IRQ(GPIO_WIFI_IRQ),*/
	.board_ref_clock = WL12XX_REFCLOCK_26,
	.board_tcxo_clock = 1,
};

int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	static int power_state;
	pr_debug("Powering %s wifi", (power_on ? "on" : "off"));
	if (power_on == power_state)
		return 0;
	power_state = power_on;
	if (power_on) {
		gpio_set_value(MAPPHONE_WIFI_IRQ_GPIO, 1);
		mdelay(15);
		gpio_set_value(MAPPHONE_WIFI_IRQ_GPIO, 0);
		mdelay(1);
		gpio_set_value(MAPPHONE_WIFI_IRQ_GPIO, 1);
		mdelay(70);
	} else
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 0);
	return 0;
}
static void mapphone_wifi_init(void)
{
	int ret;
	ret = gpio_request(MAPPHONE_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0)
		goto out;
	gpio_direction_output(MAPPHONE_WIFI_PMENA_GPIO, 0);
	mapphone_wlan_data.irq = OMAP_GPIO_IRQ(MAPPHONE_WIFI_IRQ_GPIO);
	if (wl12xx_set_platform_data(&mapphone_wlan_data))
		pr_err("Error setting wl12xx data\n");
out:
	return;
}

static void __init mapphone_bp_model_init(void)
{
#ifdef CONFIG_OMAP_RESET_CLOCKS
	struct clk *clkp;
#endif
	struct device_node *bp_node;
	const void *bp_prop;

	if ((bp_node = of_find_node_by_path(DT_PATH_CHOSEN))) {
		if ((bp_prop = of_get_property(bp_node, \
			DT_PROP_CHOSEN_BP, NULL)))
			bp_model = (char *)bp_prop;
		printk("BP MODEL:%s\n",bp_model);
		of_node_put(bp_node);
	}
#ifdef CONFIG_OMAP_RESET_CLOCKS
	/* Enable sad2d iclk */
	clkp = clk_get(NULL, "sad2d_ick");
	if (clkp) {
             clk_enable(clkp);
             printk("sad2d_ick enabled\n");
	}
#endif
}

static void mapphone_pm_power_off(void)
{
	printk(KERN_INFO "mapphone_pm_power_off start...\n");
	local_irq_disable();

	/* config gpio 176 back from safe mode to reset the device */
	omap_writew(0x4, 0x480021D2);
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 0);

	do {} while (1);

	local_irq_enable();
}

static void mapphone_pm_reset(void)
{
	arch_reset('h', NULL);
}

static int cpcap_charger_connected_probe(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_reset;
	return 0;
}

static int cpcap_charger_connected_remove(struct platform_device *pdev)
{
	pm_power_off = mapphone_pm_power_off;
	return 0;
}

static struct platform_driver cpcap_charger_connected_driver = {
	.probe          = cpcap_charger_connected_probe,
	.remove         = cpcap_charger_connected_remove,
	.driver         = {
		.name   = "cpcap_charger_connected",
		.owner  = THIS_MODULE,
	},
};

static void __init mapphone_power_off_init(void)
{
	gpio_request(MAPPHONE_POWER_OFF_GPIO, "mapphone power off");
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 1);

	/* config gpio176 into safe mode with the pull up enabled to avoid
	 * glitch at reboot */
	omap_writew(0x1F, 0x480021D2);
	pm_power_off = mapphone_pm_power_off;

	platform_driver_register(&cpcap_charger_connected_driver);
}


static void __init mapphone_voltage_init(void)
{
	/* cpcap is the default power supply for core and iva */
	omap_cpcap_init();
}

static void __init omap_mapphone_init_early(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(JEDEC_JESD209A_sdrc_params,
				   JEDEC_JESD209A_sdrc_params);

	if (fdt_start_address) {
		void *mem;
		mem = __alloc_bootmem(fdt_size, __alignof__(int), 0);
		BUG_ON(!mem);
		memcpy(mem, (const void *)fdt_start_address, fdt_size);
		initial_boot_params = (struct boot_param_header *)mem;
		pr_info("Unflattening device tree: 0x%08x\n", (u32)mem);
		unflatten_device_tree();
	}
}

static void __init omap_mapphone_init(void)
{
	struct kobject *properties_kobj = NULL;
	int ret = 0;
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				 &mapphone_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");

	mapphone_bp_model_init();
	mapphone_voltage_init();
	mapphone_gpio_mapping_init();
	mapphone_i2c_init();
	mapphone_padconf_init();
	omap_register_ion();
	mapphone_spi_init();
	mapphone_cpcap_client_init();
	mapphone_panel_init();
	mapphone_als_init();
	omap_hdq_init();
	mapphone_wifi_init();
	mapphone_power_off_init();
	mapphone_hsmmc_init();
	omap_enable_smartreflex_on_init();
	//omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	//omap_board_config = sdp_config;
	//omap_board_config_size = ARRAY_SIZE(sdp_config);
	//enable_board_wakeup_source();
	//usbhs_init(&usbhs_bdata);
#ifdef CONFIG_EMU_UART_DEBUG
	/* emu-uart function will override devtree iomux setting */
	activate_emu_uart();
#endif
}

static void __init mapphone_reserve(void)
{
#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif
	omap_reserve();
}

MACHINE_START(MAPPHONE, "mapphone_")
	.boot_params	= 0x80C00100,
	.reserve	= mapphone_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_mapphone_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap_mapphone_init,
	.timer		= &omap_timer,
MACHINE_END
