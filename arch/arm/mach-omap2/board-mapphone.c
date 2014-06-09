/*
 * Copyright (C) 2012 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bootmem.h>
#include <linux/platform_device.h>
#include <linux/spi/cpcap.h>
#include <linux/gpio.h>
#include <linux/of_fdt.h>
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#include <linux/wl12xx.h>
#include <linux/regulator/machine.h>
#include <linux/wakelock.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>

#include <plat/common.h>
#include <plat/usb.h>
#include <plat/system.h>
#include <plat/hdq.h>
#include <plat/omap-serial.h>
#include "omap_ram_console.h"
#include "omap2plus-cpufreq.h"

#include <mach/board-mapphone.h>

#include "sdram-toshiba-hynix-numonyx.h"
#include "omap_ion.h"
#include "dt_path.h"
#include "pm.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "prm-regbits-34xx.h"

#ifdef CONFIG_EMU_UART_DEBUG
#include <plat/board-mapphone-emu_uart.h>
#endif
#include <../drivers/w1/w1_family.h> /* for W1_EEPROM_DS2502 */

#define WILINK_UART_DEV_NAME "/dev/ttyO1"

#ifdef CONFIG_OMAP_RAM_CONSOLE
#define MAPPHONE_RAM_CONSOLE_START  PLAT_PHYS_OFFSET + 0xE000000
/* XXX: size is ridiculously large, but system won't boot with smaller */
#define MAPPHONE_RAM_CONSOLE_SIZE    SZ_1M
#endif

#define MAPPHONE_POWER_OFF_GPIO 176
#define MAPPHONE_WIFI_PMENA_GPIO 186
#define MAPPHONE_WIFI_IRQ_GPIO 65
#define MAPPHONE_BT_RESET_GPIO 21 //get_gpio_by_name("bt_reset_b")

/* CPCAP Defines */
#define CPCAP_SMPS_VOL_OPP1        0x02
#define CPCAP_SMPS_VOL_OPP2        0x03

/* SMPS I2C voltage control register Address*/
#define CPCAP_SRI2C_VDD_CONTROL        0x00
/* SMPS I2C Address for VDD1 */
#define CPCAP_SRI2C_SLAVE_ADDR_VDD1    0x1
/* SMPS I2C Address for VDD2 */
#define CPCAP_SRI2C_SLAVE_ADDR_VDD2    0x2
/* SMPS I2C voltage control register Address, used for SR command */
#define CPCAP_SMPS_VOL_CNTL        0x01

#define ATAG_FLAT_DEV_TREE_ADDRESS 0xf100040A

char *bp_model = "UMTS";

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

	printk(KERN_INFO
		"flat_dev_tree_virt_address=0x%08x, flat_dev_tree_address=0x%08x, flat_dev_tree_size == 0x%08X\n",
		fdt_start_address,
		fdt_addr->address,
		fdt_addr->size);

	return 0;
}

__tagtable(ATAG_FLAT_DEV_TREE_ADDRESS, parse_tag_flat_dev_tree_address);

static struct omap2_hdq_platform_config mapphone_hdq_data = {
	.mode = OMAP_SDQ_MODE,
	.id = W1_EEPROM_DS2502,
};

static int __init omap_hdq_init(void)
{
	omap_hdq_dev.dev.platform_data = &mapphone_hdq_data;
	return platform_device_register(&omap_hdq_dev);
}

static struct cpuidle_params mapphone_cpuidle_params_table[] = {
        /* C1 */
        {74 + 78, 152, 1},
        /* C2 */
        {165 + 90, 255, 0},
        /* C3 */
        {163 + 180, 345, 1},
        /* C4 */
        {2852 + 605, 3457, 0},
        /* C5 */
        {800 + 366, 2120, 1},
        /* C6 */
        {4080 + 801, 4881, 0},
        /* C7 */
        {4300 + 8794, 159000, 1},
};

static struct omap_cpufreq_platform_data cpufreq_pdata = {
        .max_nominal_freq = 1000000,
};

static struct omap_musb_board_data musb_board_data = {
	.interface_type         = MUSB_INTERFACE_ULPI,
#ifdef CONFIG_USB_MUSB_OTG
	.mode                   = MUSB_OTG,
#elif defined(CONFIG_USB_MUSB_HDRC_HCD)
	.mode                   = MUSB_HOST,
#elif defined(CONFIG_USB_GADGET_MUSB_HDRC)
	.mode                   = MUSB_PERIPHERAL,
#endif
	.power                  = 100,
};
static bool uart_req;
static struct wake_lock st_wk_lock;
/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
	int port_id = 0;
	int err = 0;
	if (uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_disable(port_id);
		if (!err)
			uart_req = false;
	}
	wake_unlock(&st_wk_lock);
	return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
	int port_id = 0;
	int err = 0;
	if (!uart_req) {
		sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
		err = omap_serial_ext_uart_enable(port_id);
		if (!err)
			uart_req = true;
	}
	wake_lock(&st_wk_lock);	
	return err;
}

static int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
	return 0;
}
static int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

/* wl127x BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = MAPPHONE_BT_RESET_GPIO, 
	.dev_name = WILINK_UART_DEV_NAME,
	.flow_cntrl = 1,
	.baud_rate = 3686400,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_asleep = plat_uart_disable,
	.chip_awake = plat_uart_enable,
	.chip_enable = plat_uart_enable,
	.chip_disable = plat_uart_disable,
};
static struct platform_device wl127x_device = {
	.name           = "kim",
	.id             = -1,
	.dev.platform_data = &wilink_pdata,
};
static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static struct platform_device *mapphone_devices[] __initdata = {
	&wl127x_device,
	&btwilink_device,
};

static struct wl12xx_platform_data mapphone_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(MAPPHONE_WIFI_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_26,
};

int wifi_set_power(struct device *dev, int slot, int power_on, int vdd)
{
	static int power_state;

	if (power_on == power_state)
		return 0;

	power_state = power_on;

	if (power_on) {
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 1);
	} else {
		gpio_set_value(MAPPHONE_WIFI_PMENA_GPIO, 0);
	}
	return 0;
}

static void mapphone_wifi_init(void)
{
	int ret;

	ret = gpio_request(MAPPHONE_WIFI_PMENA_GPIO, "wifi_pmena");
	if (ret < 0) {
		printk(KERN_ERR "%s: can't reserve GPIO: %d\n", __func__,
			MAPPHONE_WIFI_PMENA_GPIO);
		gpio_free(MAPPHONE_WIFI_PMENA_GPIO);
	}
	gpio_direction_output(MAPPHONE_WIFI_PMENA_GPIO, 0);

	if (wl12xx_set_platform_data(&mapphone_wlan_data))
	{
		pr_err("Error setting wl12xx data\n");
	}

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

static void __init mapphone_power_off_init(void)
{
	gpio_request(MAPPHONE_POWER_OFF_GPIO, "mapphone power off");
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 1);

	/* config gpio176 into safe mode with the pull up enabled to avoid
	 * glitch at reboot */
	omap_writew(0x1F, 0x480021D2);
	pm_power_off = mapphone_pm_power_off;
}

static struct prm_setup_vc mapphone_prm_setup = {
	.clksetup = 0x4c,
	.voltsetup_time1 = 0x94,
	.voltsetup_time2 = 0x94,
	.voltoffset = 0x0,
	.voltsetup2 = 0x0,
	.vdd0_on = 0x65,
	.vdd0_onlp = 0x45,
	.vdd0_ret = 0x19,
	.vdd0_off = 0x00,
	.vdd1_on = 0x65,
	.vdd1_onlp = 0x45,
	.vdd1_ret = 0x19,
	.vdd1_off = 0x00,
	.i2c_slave_ra = (CPCAP_SRI2C_SLAVE_ADDR_VDD2 <<
			OMAP3430_SMPS_SA1_SHIFT) |
			(CPCAP_SRI2C_SLAVE_ADDR_VDD1 <<
			OMAP3430_SMPS_SA0_SHIFT),
	.vdd_vol_ra = (CPCAP_SRI2C_VDD_CONTROL << OMAP3430_VOLRA1_SHIFT) |
			(CPCAP_SRI2C_VDD_CONTROL << OMAP3430_VOLRA0_SHIFT),
	/* vdd_vol_ra controls both cmd and vol, set the address equal */
	.vdd_cmd_ra = (CPCAP_SMPS_VOL_CNTL << OMAP3430_CMDRA1_SHIFT) |
		(CPCAP_SMPS_VOL_CNTL << OMAP3430_CMDRA0_SHIFT),
	.vdd_ch_conf = OMAP3430_CMD1 | OMAP3430_RACEN0 |
			OMAP3430_PRM_VC_CH_CONF_SA1 | OMAP3430_RACEN1 |
			OMAP3430_RAV1 | OMAP3430_RAC1, OMAP3430_GR_MOD,
	.vdd_i2c_cfg = OMAP3430_MCODE_SHIFT | OMAP3430_HSEN,
};

static void __init mapphone_voltage_init(void)
{
	/* cpcap is the default power supply for core and iva */
	omap_cpcap_init();

	omap3_pm_init_vc(&mapphone_prm_setup);
	/* Set CPCAP SW1/SW2 I2C CNTL Reg to 0x45 (PSM/PSM mode, VPLL enabled)
	 * to avoid extra current drain in active case before hit RET once
	 */
	omap3_bypass_cmd(CPCAP_SRI2C_SLAVE_ADDR_VDD1,
			CPCAP_SMPS_VOL_CNTL, 0x45);
	omap3_bypass_cmd(CPCAP_SRI2C_SLAVE_ADDR_VDD1,
			CPCAP_SMPS_VOL_OPP1, 0x13);
	omap3_bypass_cmd(CPCAP_SRI2C_SLAVE_ADDR_VDD1,
			CPCAP_SMPS_VOL_OPP2, 0x32);

	/* SW2, OPP1 for RET Voltage --- 0.8375V,
	 * OPP2 for ON Voltge --- 1.175V(OPP3)
	 */
	omap3_bypass_cmd(CPCAP_SRI2C_SLAVE_ADDR_VDD2,
			CPCAP_SMPS_VOL_CNTL, 0x45);
	omap3_bypass_cmd(CPCAP_SRI2C_SLAVE_ADDR_VDD2,
			CPCAP_SMPS_VOL_OPP1, 0x13);
	omap3_bypass_cmd(CPCAP_SRI2C_SLAVE_ADDR_VDD2,
			CPCAP_SMPS_VOL_OPP2, 0x2E);
}


/* Platform device structure for the SIM driver */
struct platform_device sim_device = {
	.name = "sim",
	.id = 1,
};

static void mapphone_sim_init(void)
{
	if (platform_device_register(&sim_device))
		printk(KERN_ERR" SIM device registration failed.\n");
}

void __init mapphone_create_board_props(void)
{
	struct kobject *board_props_kobj;
	int ret = 0;

	board_props_kobj = kobject_create_and_add("board_properties", NULL);
	if (!board_props_kobj)
		goto err_board_obj;

	if (mapphone_touch_vkey_prop_attr_group) {
		ret = sysfs_create_group(board_props_kobj,
				mapphone_touch_vkey_prop_attr_group);
		if (ret)
			goto err_board_obj;
	}

err_board_obj:
	if (!board_props_kobj || ret)
		pr_err("failed to create board_properties\n");

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
#ifdef CONFIG_OMAP_RESET_CLOCKS
	struct clk *clkp;
#endif
	/*
	 * This will allow unused regulator to be shutdown. This flag
	 * should be set in the board file. Before regulators are registered.
	 */
	regulator_has_full_constraints();
	omap3_pm_init_cpuidle(mapphone_cpuidle_params_table);
	omap_cpufreq_set_platform_data(&cpufreq_pdata);
	omap_serial_init();
	mapphone_voltage_init();
	mapphone_gpio_mapping_init();
	mapphone_i2c_init();
	mapphone_padconf_init();
	omap_register_ion();
	platform_add_devices(mapphone_devices, ARRAY_SIZE(mapphone_devices));
	wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	mapphone_spi_init();
	mapphone_cpcap_client_init();
	mapphone_panel_init();
	mapphone_als_init();
	omap_hdq_init();
	mapphone_wifi_init();
	usb_musb_init(&musb_board_data);
	mapphone_usbhost_init();
	mapphone_power_off_init();
	mapphone_hsmmc_init();
	mapphone_create_board_props();
	mapphone_gadget_init();
	mapphone_sim_init();
#ifdef CONFIG_EMU_UART_DEBUG
	/* emu-uart function will override devtree iomux setting */
	activate_emu_uart();
#endif

#ifdef CONFIG_OMAP_RESET_CLOCKS
	/* Enable sad2d iclk */
	clkp = clk_get(NULL, "sad2d_ick");
	if (clkp) {
             clk_enable(clkp);
             printk("sad2d_ick enabled\n");
	}
#endif

}

static void __init mapphone_reserve(void)
{
#ifdef CONFIG_OMAP_RAM_CONSOLE
        omap_ram_console_init(MAPPHONE_RAM_CONSOLE_START,
                        MAPPHONE_RAM_CONSOLE_SIZE);
#endif

#ifdef CONFIG_ION_OMAP
	omap_ion_init();
#endif
	omap_reserve();
}

MACHINE_START(MAPPHONE, "mapphone_umts")
	.boot_params	= 0x80C00100,
	.reserve	= mapphone_reserve,
	.map_io		= omap3_map_io,
	.init_early	= omap_mapphone_init_early,
	.init_irq	= omap_init_irq,
	.init_machine	= omap_mapphone_init,
	.timer		= &omap_timer,
MACHINE_END
