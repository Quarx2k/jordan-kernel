/*
 * linux/arch/arm/mach-omap2/board-mapphone.c
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * Modified from mach-omap3/board-3430sdp.c
 *
 * Copyright (C) 2007 Texas Instruments
 *
 * Modified from mach-omap2/board-generic.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/reboot.h>
#include <linux/qtouch_obp_ts.h>
#include <linux/led-cpcap-lm3554.h>
#include <linux/led-cpcap-lm3559.h>
#include <linux/led-lm3530.h>
#include <linux/wl127x-rfkill.h>
#include <linux/wl127x-test.h>
#include <linux/gpio_mapping.h>
#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/bootinfo.h>

#include <plat/board-mapphone.h>
#include <plat/board-mapphone-sensors.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/usb.h>
#include <plat/gpmc.h>
#include <linux/delay.h>
#include <plat/control.h>
#include <plat/hdq.h>
#include <mach/system.h>
#include <linux/wakelock.h>

#include "cm-regbits-34xx.h"

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#include "pm.h"
#include "prm-regbits-34xx.h"
#include "smartreflex.h"
#include "omap3-opp.h"
#include "sdram-toshiba-hynix-numonyx.h"
#include "prcm-common.h"
#include "cm.h"
#include "clock.h"

#ifdef CONFIG_KEYBOARD_ADP5588
#include <linux/i2c/adp5588.h>
#endif

#ifdef CONFIG_MOT_KEYBOARD_ADP5588
#include <linux/adp5588_keypad.h>
#endif

#ifdef CONFIG_VIDEO_OMAP3
#include <media/v4l2-int-device.h>
#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
#include <media/mt9p012.h>
#endif
#if defined(CONFIG_VIDEO_OV8810) || defined(CONFIG_VIDEO_OV8810_MODULE)
#include <media/ov8810.h>
#endif
#if defined(CONFIG_VIDEO_OV5650) || defined(CONFIG_VIDEO_OV5650_MODULE)
#include <media/ov5650.h>
#endif
#if defined(CONFIG_VIDEO_CAM_ISE) || defined(CONFIG_VIDEO_CAM_ISE_MODULE)
#include <media/camise.h>
#endif


#if defined(CONFIG_LEDS_BD7885)
#include <linux/leds-bd7885.h>
#endif
#if defined(CONFIG_LEDS_BU9847)
#include <linux/leds-bu9847.h>
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
#include <../drivers/media/video/hplens.h>
#endif
#endif

#ifdef CONFIG_EMU_UART_DEBUG
#include <plat/board-mapphone-emu_uart.h>
#endif

#ifdef CONFIG_INPUT_ALS_IR_ISL29030
#include <linux/isl29030.h>
#endif

#define ATMXT_NAME	"placeholder"
#define CYTTSP_NAME	"placeholder"

#define MAPPHONE_IPC_USB_SUSP_GPIO	142
#define MAPPHONE_LM_3530_INT_GPIO	92
#define MAPPHONE_AKM8973_INT_GPIO	175
#define MAPPHONE_AKM8975_INT_GPIO       175
#define MAPPHONE_AUDIO_PATH_GPIO	143
#define MAPPHONE_BP_READY2_AP_GPIO	59
#define MAPPHONE_POWER_OFF_GPIO		176
#define MAPPHONE_BPWAKE_STROBE_GPIO	157
#define MAPPHONE_APWAKE_TRIGGER_GPIO	141
#define MAPPHONE_AIRC_INT_GPIO        180

#define MAPPHONE_MMCPROBE_ENABLED 0

#define CAMERA_FLASH_ID 0

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

#define I2C_BUS_MAX_DEVICES 5
#define I2C_MAX_DEV_NAME_LEN 16
#define I2C_BUS_PROP_NAME_LEN 12

char *bp_model = "CDMA";

static struct omap_opp mapphone_omap3430_mpu_rate_table[] = {
	{0, 0, 0, 0},
	/*OPP1*/
	{S125M, VDD1_OPP1, 0x20, 0x0},
	/*OPP2*/
	{S250M, VDD1_OPP2, 0x27, 0x0},
	/*OPP3*/
	{S500M, VDD1_OPP3, 0x32, 0x0},
	/*OPP4*/
	{S550M, VDD1_OPP4, 0x38, 0x0},
	/*OPP5*/
	{S720M, VDD1_OPP5, 0x3E, 0x0},
};

#define S80M 80000000
#define S160M 160000000

static struct omap_opp mapphone_omap3430_l3_rate_table[] = {
	{0, 0, 0, 0},
	/*OPP1*/
	{0, VDD2_OPP1, 0x20, 0x0},
	/*OPP2*/
	{S80M, VDD2_OPP2, 0x27, 0x0},
	/*OPP3*/
	{S160M, VDD2_OPP3, 0x2E, 0x0},
};

static struct omap_opp mapphone_omap3430_dsp_rate_table[] = {
	{0, 0, 0, 0},
	/*OPP1*/
	{S90M, VDD1_OPP1, 0x20, 0x0},
	/*OPP2*/
	{S180M, VDD1_OPP2, 0x27, 0x0},
	/*OPP3*/
	{S360M, VDD1_OPP3, 0x32, 0x0},
	/*OPP4*/
	{S400M, VDD1_OPP4, 0x38, 0x0},
	/*OPP5*/
	{S520M, VDD1_OPP5, 0x3E, 0x0},
};

static struct omap_opp mapphone_omap3630_mpu_rate_table[] = {
	{0, 0, 0, 0},
	/*Add headroom for CPCAP IR drop*/
	/*OPP1,CPCAP 1.0125v*/
	{S300M, VDD1_OPP1, 0x21, 0x0},
	/*OPP2,CPCAP 1.2v*/
	{S600M, VDD1_OPP2, 0x30, 0x0},
	/*OPP3,CPCAP 1.325v*/
	{S800M, VDD1_OPP3, 0x3A, 0x0},
	/*OPP4,CPCAP 1.375v*/
	{S1000M, VDD1_OPP4, 0x3E, 0x0},
	/*OPP5,CPCAP 1.375v*/
	{S1200M, VDD1_OPP5, 0x3E, 0x0},
};


static struct omap_opp mapphone_omap3630_l3_rate_table[] = {
	{0, 0, 0, 0},
	/*OPP1*/
	{S100M, VDD2_OPP1, 0x20, 0x0},
	/*OPP2*/
	{S200M, VDD2_OPP2, 0x30, 0x0},
};

static struct omap_opp mapphone_omap3630_dsp_rate_table[] = {
	{0, 0, 0, 0},
	/*OPP1,CPCAP 1.0125v*/
	{S260M, VDD1_OPP1, 0x21, 0x0},
	/*OPP2,CPCAP 1.2v*/
	{S520M, VDD1_OPP2, 0x30, 0x0},
	/*OPP3,CPCAP 1.325v*/
	{S660M, VDD1_OPP3, 0x3A, 0x0},
	/*OPP4,CPCAP 1.375v*/
	{S800M, VDD1_OPP4, 0x3E, 0x0},
	/*OPP5,CPCAP 1.375v*/
	{S65M, VDD1_OPP5, 0x3E, 0x0},
};

static struct cpuidle_params mapphone_cpuidle_params_table[] = {
	/* C1 */
	{1, 0, 12, 15},
	/* C2 */
	{1, 0, 18, 20},
	/* C3 */
	{1, 150, 260, 500},
	/* C4 */
	{0, 1600, 1850, 4000},
	/* C5 */
	{1, 310, 2850, 5000},
	/* C6 */
	{0, 1800, 4450, 10000},
	/* C7 */
	{0, 10000, 30000, 300000},
};

static void __init mapphone_init_irq(void)
{
	if (cpu_is_omap3630()) {
		omap2_init_common_hw(JEDEC_JESD209A_sdrc_params,
			JEDEC_JESD209A_sdrc_params,
			mapphone_omap3630_mpu_rate_table,
			mapphone_omap3630_dsp_rate_table,
			mapphone_omap3630_l3_rate_table);
	} else{
		omap2_init_common_hw(JEDEC_JESD209A_sdrc_params,
			JEDEC_JESD209A_sdrc_params,
			mapphone_omap3430_mpu_rate_table,
			mapphone_omap3430_dsp_rate_table,
			mapphone_omap3430_l3_rate_table);
	}
	omap3_pm_init_cpuidle(mapphone_cpuidle_params_table);
	omap_init_irq();
#ifdef CONFIG_OMAP3_PM
	scm_clk_init();
#endif
	omap_gpio_init();
}


#if defined(CONFIG_VIDEO_MIPI_DLI_TEST)
static struct platform_device mapphone_mipi_dli_device = {
	.name = "mipi_dli_tester",
	.id = -1,
};
#endif

/* Platform device structure for the SIM driver */
struct platform_device sim_device = {
	.name = "sim",
	.id = 1,
};

static bool sim_available = 1;

bool is_sim_available(void)
{
	return sim_available ? 1 : 0;
}
EXPORT_SYMBOL(is_sim_available);

static void mapphone_sim_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;

	/*
	 * load sim driver if failure to open DT,
	 * default consumption: sim card is available.
	 */
	node = of_find_node_by_path(DT_PATH_SIM_DEV);
	if (node) {
		prop = of_get_property(node,
			DT_PROP_SIM_DEV_AVAILABILITY, NULL);
		if (prop)
			sim_available = *(bool *)prop;
		else
			printk(KERN_ERR"Read property %s error!\n",
				DT_PROP_SIM_DEV_AVAILABILITY);
		of_node_put(node);
	}
#endif

	printk(KERN_INFO"SIM device %s.\n",
		sim_available ? "enabled" : "disabled");

	if (!sim_available)
		return;
	if (platform_device_register(&sim_device))
		printk(KERN_ERR" SIM device registration failed.\n");
}

static int __init mapphone_audio_init(void)
{
	struct device_node *dt_node;
	const void *dt_prop;
	unsigned int is_uart_en = 0;
	int hs_switch = -1;

	gpio_request(MAPPHONE_AUDIO_PATH_GPIO, "mapphone audio path");
	gpio_direction_output(MAPPHONE_AUDIO_PATH_GPIO, 1);

	/* Enable headset audio unless uart debug is enabled in devtree */
	dt_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (NULL != dt_node) {
		dt_prop = of_get_property(dt_node,
				DT_HIGH_LEVEL_FEATURE_HEADSET_UART_EN, NULL);
		if (NULL != dt_prop) {
			is_uart_en = *(u8 *)dt_prop;
			printk(KERN_INFO "feature_headset_uart_en %d\n",
								is_uart_en);

			/* Get the headset switch gpio number from devtree */
			hs_switch = get_gpio_by_name("headset_uart_switch");
			if (hs_switch < 0)
				return -EINVAL;

			/* configure headset switch gpio as output and
			   direction based on devtree setting */
			gpio_request(hs_switch,
					"mapphone audio headset uart switch");

			if (is_uart_en == 0) {
				/* route audio out headset jack */
				gpio_direction_output(hs_switch, 1);
			} else {
				/* route kernel uart out headset jack */
				gpio_direction_output(hs_switch, 0);
			}
		}
	}

	return 0;
}

static struct omap_uart_config mapphone_uart_config __initdata = {
	.enabled_uarts = ((1 << 0) | (1 << 1) | (1 << 2)),
};

static struct omap_board_config_kernel mapphone_config[] __initdata = {
	{OMAP_TAG_UART,		&mapphone_uart_config },
};

int mapphone_touch_reset(void)
{
	int reset_pin;
	int retval = 0;

	reset_pin = get_gpio_by_name("touch_panel_rst");
	if (reset_pin < 0) {
		printk(KERN_ERR "%s: Cannot acquire reset pin.\n", __func__);
		retval = reset_pin;
	} else {
		gpio_direction_output(reset_pin, 1);  /* Legacy support */
		msleep(1);  /* Legacy support */
		gpio_set_value(reset_pin, 0);
		msleep(QTM_OBP_SLEEP_RESET_HOLD);  /* Legacy val */
		gpio_set_value(reset_pin, 1);
		msleep(QTM_OBP_SLEEP_WAIT_FOR_HW_RESET);  /* Legacy support */
	}

	return retval;
}
EXPORT_SYMBOL(mapphone_touch_reset);

static struct qtouch_ts_platform_data mapphone_ts_platform_data;

static ssize_t mapphone_virtual_keys_show(struct kobject *kobj,
					struct kobj_attribute *attr, char *buf)
{
	int key_num;
	int string_loc = 0;
	int num_chars;

	for (key_num = 0; key_num < mapphone_ts_platform_data.vkeys.count; key_num++) {
		if (key_num != 0) {
			num_chars = sprintf((buf + string_loc), ":");
			string_loc += num_chars;
		}

		num_chars = sprintf((buf + string_loc),
			__stringify(EV_KEY) ":%d:%d:%d:%d:%d",
			mapphone_ts_platform_data.vkeys.keys[key_num].code,
			mapphone_ts_platform_data.vkeys.keys[key_num].center_x,
			mapphone_ts_platform_data.vkeys.keys[key_num].center_y,
			mapphone_ts_platform_data.vkeys.keys[key_num].width,
			mapphone_ts_platform_data.vkeys.keys[key_num].height);
		string_loc += num_chars;
	}

	sprintf((buf + string_loc), "\n");

	return string_loc;
}

static struct kobj_attribute mapphone_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.qtouch-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &mapphone_virtual_keys_show,
};

static struct attribute *mapphone_properties_attrs[] = {
	&mapphone_virtual_keys_attr.attr,
	NULL,
};

static struct attribute_group mapphone_properties_attr_group = {
	.attrs = mapphone_properties_attrs,
};

static struct i2c_board_info __initdata mapphone_i2c_bus1_master_board_info[];
static struct i2c_board_info __initdata mapphone_i2c_bus2_master_board_info[];

/* Legacy support */
static void mapphone_legacy_qtouch_init(void)
{
	int len = 0;
	struct device_node *touch_node;
	const void *touch_prop;
	const uint32_t *touch_val;

	printk(KERN_INFO "%s: Selecting legacy qtouch driver.\n",
		__func__);

	touch_node = of_find_node_by_path(DT_PATH_TOUCH);
	if (touch_node == NULL) {
		printk(KERN_INFO "%s: No device tree data available.\n",
			__func__);
		goto mapphone_legacy_qtouch_init_ret;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_KEYMAP, &len);
	if (touch_prop && len && (0 == len % sizeof(struct vkey))) {
			mapphone_ts_platform_data.vkeys.count =
				len / sizeof(struct vkey);
			mapphone_ts_platform_data.vkeys.keys =
				(struct vkey *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_I2C_ADDRESS,
		&len);
	if (touch_prop) {
		mapphone_i2c_bus1_master_board_info[0].addr =
			*((int *)touch_prop);
	}

	touch_prop = of_get_property(touch_node,
		DT_PROP_TOUCH_BOOT_I2C_ADDRESS, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.boot_i2c_addr =
			*((int *)touch_prop);
	}

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_CHECKSUM, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.nv_checksum = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FLAGS, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.flags = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MIN_X, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.abs_min_x = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MAX_X, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.abs_max_x = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MIN_Y, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.abs_min_y = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MAX_Y, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.abs_max_y = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MIN_P, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.abs_min_p = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MAX_P, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.abs_max_p = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MIN_W, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.abs_min_w = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_ABS_MAX_W, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.abs_max_w = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FUZZ_X, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.fuzz_x = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FUZZ_Y, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.fuzz_y = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FUZZ_P, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.fuzz_p = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_FUZZ_W, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.fuzz_w = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_X_DELTA, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.x_delta = *touch_val;

	touch_val = of_get_property(touch_node, DT_PROP_TOUCH_Y_DELTA, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.y_delta = *touch_val;

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T15, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.key_array.cfg =
			(struct qtm_touch_keyarray_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_KEY_ARRAY_MAP,
		&len);
	if (touch_prop) {
		mapphone_ts_platform_data.key_array.keys =
			(struct qtouch_key *)touch_prop;
	}

	touch_val = of_get_property(touch_node,
		DT_PROP_TOUCH_KEY_ARRAY_COUNT, &len);
	if (touch_val && len)
		mapphone_ts_platform_data.key_array.num_keys = *touch_val;

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T7, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.power_cfg =
			*(struct qtm_gen_power_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T8, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.acquire_cfg =
			*(struct qtm_gen_acquire_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T9, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.multi_touch_cfg =
			*(struct qtm_touch_multi_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T17, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.linear_tbl_cfg =
			*(struct qtm_proci_linear_tbl_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T18, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.comms_config_cfg =
			*(struct spt_comms_config_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T19, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.gpio_pwm_cfg =
			*(struct qtm_spt_gpio_pwm_cfg *)touch_prop;
	}


	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T20, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.grip_suppression_cfg =
		*(struct qtm_proci_grip_suppression_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T22, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.noise_suppression_cfg =
			*(struct qtm_procg_noise_suppression_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T23, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.touch_proximity_cfg =
			*(struct qtm_touch_proximity_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T24, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.one_touch_gesture_proc_cfg =
		*(struct qtm_proci_one_touch_gesture_proc_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T25, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.self_test_cfg =
			*(struct qtm_spt_self_test_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T27, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.two_touch_gesture_proc_cfg =
		*(struct qtm_proci_two_touch_gesture_proc_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T28, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.cte_config_cfg =
			*(struct qtm_spt_cte_config_cfg *)touch_prop;
	}

	touch_prop = of_get_property(touch_node, DT_PROP_TOUCH_T36, &len);
	if (touch_prop) {
		mapphone_ts_platform_data.noise1_suppression_cfg =
			*(struct qtm_proci_noise1_suppression_cfg *)touch_prop;
	}

	of_node_put(touch_node);

	mapphone_i2c_bus1_master_board_info[0].platform_data =
		&mapphone_ts_platform_data;

mapphone_legacy_qtouch_init_ret:
	return;
}

static void mapphone_touch_init(void)
{
	int retval = 0;
	struct device_node *dtnode;
	const void *prop;
	int len = 0;
	int pin = 0;
	int err = 0;

	int touch_pwr_en_gpio = 0;  /* Legacy support */

	pin = get_gpio_by_name("touch_panel_rst");
	if (pin >= 0) {
		err = gpio_request(pin, "touch_reset");
		if (err >= 0) {
			err = gpio_direction_output(pin, 1);  /* Legacy val */
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config reset.\n",
						__func__);
				retval = err;
				goto touch_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: Reset GPIO request failed:\n",
					__func__);
			retval = err;
			goto touch_init_fail;
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire reset pin.\n", __func__);
		retval = pin;
		goto touch_init_fail;
	}

	pin = get_gpio_by_name("touch_panel_int");
	if (pin >= 0) {
		err = gpio_request(pin, "touch_irq");
		if (err >= 0) {
			err = gpio_direction_input(pin);
			if (err < 0) {
				printk(KERN_ERR "%s: Unable to config irq.\n",
						__func__);
				retval = err;
				goto touch_init_fail;
			}
		} else {
			printk(KERN_ERR "%s: IRQ GPIO request failed.\n",
					__func__);
			retval = err;
			goto touch_init_fail;
		}
	} else {
		printk(KERN_ERR "%s: Cannot acquire irq pin.\n", __func__);
		goto touch_init_fail;
	}
	mapphone_i2c_bus1_master_board_info[0].irq = gpio_to_irq(pin);

	/* Legacy support */
	touch_pwr_en_gpio = get_gpio_by_name("touch_pwr_en");
	if (touch_pwr_en_gpio >= 0) {
		gpio_request(touch_pwr_en_gpio, "mapphone touch power enable");
		gpio_direction_output(touch_pwr_en_gpio, 1);
	}


	dtnode = of_find_node_by_path("/System@0/I2C@0/Touch@0/Driver@0");
	if (dtnode == NULL) {
		/* Normally this is a fatal error */
		mapphone_legacy_qtouch_init();  /* Legacy support */
		goto touch_init_pass;  /* Legacy support */
	}

	prop = of_get_property(dtnode, "touch_driver_name", &len);
	if (prop == NULL) {
		printk(KERN_ERR "%s: Driver name missing.\n", __func__);
		goto touch_init_fail;
	} else if (len >= I2C_NAME_SIZE) {
		printk(KERN_ERR "%s: Driver name is too long.\n", __func__);
		goto touch_init_fail;
	}
	strncpy((char *)&mapphone_i2c_bus1_master_board_info[0].type,
			(char *)prop, I2C_NAME_SIZE);
	mapphone_i2c_bus1_master_board_info[0].type[I2C_NAME_SIZE-1] = '\0';

	dtnode = of_find_node_by_path("/System@0/I2C@0/Touch@0/IC@0");
	if (dtnode == NULL) {
		printk(KERN_ERR "%s: IC@0 node is missing.\n", __func__);
		goto touch_init_fail;
	}
	prop = of_get_property(dtnode, "i2c_addr", &len);
	if (prop == NULL) {
		printk(KERN_ERR "%s: I2C address is missing.\n", __func__);
		goto touch_init_fail;
	}
	mapphone_i2c_bus1_master_board_info[0].addr = *((int *)prop);

	dtnode = of_find_node_by_path("/System@0/I2C@0/Touch@0/Driver@0");
	prop = of_get_property(dtnode, "touch_driver_name", &len);
	if (strcmp((char *)prop, ATMXT_NAME) == 0) {
		printk(KERN_ERR "%s: Touch driver %s is not yet supported.\n",
				__func__, ATMXT_NAME);
		goto touch_init_fail;

	} else if (strcmp((char *)prop, CYTTSP_NAME) == 0) {
		printk(KERN_ERR "%s: Touch driver %s is not yet supported.\n",
				__func__, CYTTSP_NAME);
		goto touch_init_fail;

	} else {
		printk(KERN_ERR "%s: Invalid driver name found: %s.\n",
				__func__, (char *)prop);
		goto touch_init_fail;
	}

	goto touch_init_pass;

touch_init_fail:
	printk(KERN_ERR "%s: Touch init failed with error code %d.\n",
			__func__, err);
	return;

touch_init_pass:
	printk(KERN_INFO "%s: Touch init successful.\n", __func__);
	return;
}

static struct lm3530_platform_data omap3430_als_light_data;

static void __init mapphone_als_init(void)
{
	int lm3530_int_gpio = MAPPHONE_LM_3530_INT_GPIO;
	int lm3530_reset_gpio;
#ifdef CONFIG_ARM_OF
	struct device_node *als_node;
	const u8 *als_val;
	int len = 0;
	als_node = of_find_node_by_path(DT_LCD_BACKLIGHT);
	if (als_node != NULL) {
		als_val = of_get_property(als_node, DT_PROP_POWERUP_GEN_CNFG,
									&len);
		if (als_val && len)
			omap3430_als_light_data.power_up_gen_config = *als_val;
		else
			pr_err("%s: Cann't get powerup gen cnfg\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_GEN_CNFG, &len);
		if (als_val && len)
			omap3430_als_light_data.gen_config = *als_val;
		else
			pr_err("%s: Cann't get gen cnfg\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ALS_CNFG, &len);
		if (als_val && len)
			omap3430_als_light_data.als_config = *als_val;
		else
			pr_err("%s: Cann't get als cnfg\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_BRIGHTNESS_RAMP,
									&len);
		if (als_val && len)
			omap3430_als_light_data.brightness_ramp = *als_val;
		else
			pr_err("%s: Cann't get brightness ramp", __func__);

		als_val = of_get_property(als_node, DT_PROP_ALS_ZONE_INFO,
									&len);
		if (als_val && len)
			omap3430_als_light_data.als_zone_info = *als_val;
		else
			pr_err("%s: Cann't get als zone info\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ALS_RESISTOR_SEL,
									&len);
		if (als_val && len)
			omap3430_als_light_data.als_resistor_sel = *als_val;
		else
			pr_err("%s: Cann't get als resistor sel\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_BRIGHTNESS_CTRL,
									&len);
		if (als_val && len)
			omap3430_als_light_data.brightness_control = *als_val;
		else
			pr_err("%s: Cann't get brightness control\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZB0, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_boundary_0 = *als_val;
		else
			pr_err("%s: Cann't get zone boundary 0\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZB1, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_boundary_1 = *als_val;
		else
			pr_err("%s: Cann't get zone boundary 1\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZB2, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_boundary_2 = *als_val;
		else
			pr_err("%s: Cann't get zone boundary 2\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZB3, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_boundary_3 = *als_val;
		else
			pr_err("%s: Cann't get zone boundary 3\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT0, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_0 = *als_val;
		else
			pr_err("%s: Cann't get zone target 0\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT1, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_1 = *als_val;
		else
			pr_err("%s: Cann't get zone target 1\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT2, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_2 = *als_val;
		else
			pr_err("%s: Cann't get zone target 2\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT3, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_3 = *als_val;
		else
			pr_err("%s: Cann't get zone target 3\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ZT4, &len);
		if (als_val && len)
			omap3430_als_light_data.zone_target_4 = *als_val;
		else
			pr_err("%s: Cann't get zone target 4\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_MANUAL_CURRENT,
									&len);
		if (als_val && len)
			omap3430_als_light_data.manual_current = *als_val;
		else
			pr_err("%s: Cann't get manual current\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_UPPER_CURR_SEL,
									&len);
		if (als_val && len)
			omap3430_als_light_data.upper_curr_sel = *als_val;
		else
			pr_err("%s: Cann't get upper curr sel\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_LOWER_CURR_SEL,
									&len);
		if (als_val && len)
			omap3430_als_light_data.lower_curr_sel = *als_val;
		else
			pr_err("%s: Cann't get lower curr sel\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_LENS_LOSS_COEFF,
									&len);
		if (als_val && len)
			omap3430_als_light_data.lens_loss_coeff = *als_val;
		else
			pr_err("%s: Cann't get lens loss coeff\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_MANUAL_ALS_CONFIG,
									&len);
		if (als_val && len)
			omap3430_als_light_data.manual_als_config = *als_val;
		else
			pr_err("%s: Cann't get manual als config\n", __func__);

		als_val = of_get_property(als_node, DT_PROP_ALS_ENABLED, &len);
		if (als_val && len)
			omap3430_als_light_data.als_enabled = *als_val;
		else
			pr_err("%s: Cann't get manual als config\n", __func__);
		of_node_put(als_node);
	}

	lm3530_int_gpio = get_gpio_by_name("lm3530_int");
	if (lm3530_int_gpio < 0) {
		printk(KERN_DEBUG"mapphone_als_init: cann't get lm3530_int from device_tree\n");
		lm3530_int_gpio = MAPPHONE_LM_3530_INT_GPIO;
	} else {
		mapphone_i2c_bus1_master_board_info[1].irq =
				 OMAP_GPIO_IRQ(lm3530_int_gpio);
		mapphone_i2c_bus2_master_board_info[3].irq =
				 OMAP_GPIO_IRQ(lm3530_int_gpio);
	}
	lm3530_reset_gpio = get_gpio_by_name("lm3530_reset");
	if (lm3530_int_gpio >= 0) {
		gpio_request(lm3530_reset_gpio, "LED reset");
		gpio_direction_output(lm3530_reset_gpio, 1);
		msleep(10);
	}
#endif
	printk(KERN_INFO "%s:Initializing\n", __func__);
	gpio_request(lm3530_int_gpio, "mapphone als int");
	gpio_direction_input(lm3530_int_gpio);
	omap_cfg_reg(AC27_34XX_GPIO92);
}

static struct vkey mapphone_touch_vkeys[] = {
	{
		.code		= KEY_BACK,
		.center_x	= 32,
		.center_y	= 906,
		.width		= 63,
		.height		= 57,
	},
	{
		.code		= KEY_MENU,
		.center_x	= 162,
		.center_y	= 906,
		.width		= 89,
		.height		= 57,
	},
	{
		.code		= KEY_HOME,
		.center_x	= 292,
		.center_y	= 906,
		.width		= 89,
		.height		= 57,
	},
	{
		.code		= KEY_SEARCH,
		.center_x	= 439,
		.center_y	= 906,
		.width		= 63,
		.height		= 57,
	},
};

static struct qtm_touch_keyarray_cfg mapphone_key_array_data[] = {
	{
		.ctrl		= 0,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 0,
		.y_size		= 0,
		.aks_cfg	= 0,
		.burst_len	= 0,
		.tch_det_thr	= 0,
		.tch_det_int	= 0,
		.rsvd1		= 0,
		.rsvd2		= 0,
	},
	{
		.ctrl		= 0,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 0,
		.y_size		= 0,
		.aks_cfg	= 0,
		.burst_len	= 0,
		.tch_det_thr	= 0,
		.tch_det_int	= 0,
		.rsvd1		= 0,
		.rsvd2		= 0,
	},
};

static struct qtouch_ts_platform_data mapphone_ts_platform_data = {
	.flags		= (QTOUCH_SWAP_XY |
			   QTOUCH_USE_MULTITOUCH |
			   QTOUCH_CFG_BACKUPNV |
			   QTOUCH_EEPROM_CHECKSUM),
	.irqflags		= (IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW),
	.abs_min_x		= 20,
	.abs_max_x		= 1004,
	.abs_min_y		= 0,
	.abs_max_y		= 960,
	.abs_min_p		= 0,
	.abs_max_p		= 255,
	.abs_min_w		= 0,
	.abs_max_w		= 15,
	.x_delta		= 400,
	.y_delta		= 250,
	.nv_checksum		= 0xb834,
	.fuzz_x			= 0,
	.fuzz_y			= 0,
	.fuzz_p			= 2,
	.fuzz_w			= 2,
	.boot_i2c_addr		= 0x5f,
	.hw_reset		= mapphone_touch_reset,
	.key_array = {
		.cfg		= mapphone_key_array_data,
		.keys		= NULL,
		.num_keys	= 0,
	},
	.power_cfg	= {
		.idle_acq_int	= 0xff,
		.active_acq_int	= 0xff,
		.active_idle_to	= 0x01,
	},
	.acquire_cfg	= {
		.charge_time	= 12,
		.atouch_drift	= 5,
		.touch_drift	= 20,
		.drift_susp	= 20,
		.touch_autocal	= 0x96,
		.sync		= 0,
		.atch_cal_suspend_time	= 0,
		.atch_cal_suspend_thres	= 0,
	},
	.multi_touch_cfg	= {
		.ctrl		= 0x0b,
		.x_origin	= 0,
		.y_origin	= 0,
		.x_size		= 12,
		.y_size		= 7,
		.aks_cfg	= 0,
		.burst_len	= 0x40,
		.tch_det_thr	= 0x12,
		.tch_det_int	= 0x2,
		.orient		= 0x00,
		.mrg_to		= 25,
		.mov_hyst_init	= 5,
		.mov_hyst_next	= 5,
		.mov_filter	= 0,
		.num_touch	= 4,
		.merge_hyst	= 0,
		.merge_thresh	= 3,
		.amp_hyst       = 0,
		.x_res		= 0x0000,
		.y_res		= 0x0000,
		.x_low_clip	= 0x00,
		.x_high_clip	= 0x00,
		.y_low_clip	= 0x00,
		.y_high_clip	= 0x00,
		.x_edge_ctrl	= 0,
		.x_edge_dist	= 0,
		.y_edge_ctrl	= 0,
		.y_edge_dist	= 0,
	},
	.linear_tbl_cfg = {
		.ctrl		= 0x01,
		.x_offset	= 0x0000,
		.x_segment = {
			0x48, 0x3f, 0x3c, 0x3E,
			0x3f, 0x3e, 0x3e, 0x3e,
			0x3f, 0x42, 0x41, 0x3f,
			0x41, 0x40, 0x41, 0x46
		},
		.y_offset = 0x0000,
		.y_segment = {
			0x44, 0x38, 0x37, 0x3e,
			0x3e, 0x41, 0x41, 0x3f,
			0x42, 0x41, 0x42, 0x42,
			0x41, 0x3f, 0x41, 0x45
		},
	},
	.comms_config_cfg = {
		.ctrl		= 0,
		.command	= 0,
	},
	.gpio_pwm_cfg = {
		.ctrl			= 0,
		.report_mask		= 0,
		.pin_direction		= 0,
		.internal_pullup	= 0,
		.output_value		= 0,
		.wake_on_change		= 0,
		.pwm_enable		= 0,
		.pwm_period		= 0,
		.duty_cycle_0		= 0,
		.duty_cycle_1		= 0,
		.duty_cycle_2		= 0,
		.duty_cycle_3		= 0,
		.trigger_0		= 0,
		.trigger_1		= 0,
		.trigger_2		= 0,
		.trigger_3		= 0,
	},
	.grip_suppression_cfg = {
		.ctrl		= 0x00,
		.xlogrip	= 0x00,
		.xhigrip	= 0x00,
		.ylogrip	= 0x00,
		.yhigrip	= 0x00,
		.maxtchs	= 0x00,
		.reserve0	= 0x00,
		.szthr1		= 0x00,
		.szthr2		= 0x00,
		.shpthr1	= 0x00,
		.shpthr2	= 0x00,
		.supextto	= 0x00,
	},
	.noise_suppression_cfg = {
		.ctrl			= 0,
		.outlier_filter_len	= 0,
		.reserve0		= 0,
		.gcaf_upper_limit	= 0,
		.gcaf_lower_limit	= 0,
		.gcaf_low_count		= 0,
		.noise_threshold	= 0,
		.reserve1		= 0,
		.freq_hop_scale		= 0,
		.burst_freq_0		= 0,
		.burst_freq_1		= 0,
		.burst_freq_2		= 0,
		.burst_freq_3		= 0,
		.burst_freq_4		= 0,
		.idle_gcaf_valid	= 0,
	},
	.touch_proximity_cfg = {
		.ctrl			= 0,
		.x_origin		= 0,
		.y_origin		= 0,
		.x_size			= 0,
		.y_size			= 0,
		.reserve0		= 0,
		.blen			= 0,
		.tch_thresh		= 0,
		.tch_detect_int		= 0,
		.average		= 0,
		.rate			= 0,
	},
	.one_touch_gesture_proc_cfg = {
		.ctrl			= 0,
		.reserve0		= 0,
		.gesture_enable		= 0,
		.pres_proc		= 0,
		.tap_time_out		= 0,
		.flick_time_out		= 0,
		.drag_time_out		= 0,
		.short_press_time_out	= 0,
		.long_press_time_out	= 0,
		.repeat_press_time_out	= 0,
		.flick_threshold	= 0,
		.drag_threshold		= 0,
		.tap_threshold		= 0,
		.throw_threshold	= 0,
	},
	.self_test_cfg = {
		.ctrl			= 0,
		.command		= 0,
		.high_signal_limit_0	= 0,
		.low_signal_limit_0	= 0,
		.high_signal_limit_1	= 0,
		.low_signal_limit_1	= 0,
		.high_signal_limit_2	= 0,
		.low_signal_limit_2	= 0,
	},
	.two_touch_gesture_proc_cfg = {
		.ctrl			= 0,
		.reserved0		= 0,
		.reserved1		= 0,
		.gesture_enable		= 0,
		.rotate_threshold	= 0,
		.zoom_threshold		= 0,
	},
	.cte_config_cfg = {
		.ctrl			= 1,
		.command		= 0,
		.mode			= 3,
		.idle_gcaf_depth	= 4,
		.active_gcaf_depth	= 8,
		.voltage		= 0,
	},
	.noise1_suppression_cfg = {
		.ctrl		= 0x01,
		.version	= 0x01,
		.atch_thr	= 0x64,
		.duty_cycle	= 0x08,
		.drift_thr	= 0x00,
		.clamp_thr	= 0x00,
		.diff_thr	= 0x00,
		.adjustment	= 0x00,
		.average	= 0x0000,
		.temp		= 0x00,
		.offset = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
		},
		.bad_chan = {
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00
		},
		.x_short	= 0x00,
	},
	.vkeys			= {
		.count		= ARRAY_SIZE(mapphone_touch_vkeys),
		.keys		= mapphone_touch_vkeys,
	},
};

static struct lm3530_platform_data omap3430_als_light_data = {
	.power_up_gen_config = 0x0b,
	.gen_config = 0x3b,
	.als_config = 0x6c,
	.brightness_ramp = 0x00,
	.als_zone_info = 0x00,
	.als_resistor_sel = 0x31,
	.brightness_control = 0x00,
	.zone_boundary_0 = 0x02,
	.zone_boundary_1 = 0x10,
	.zone_boundary_2 = 0x43,
	.zone_boundary_3 = 0xfc,
	.zone_target_0 = 0x51,
	.zone_target_1 = 0x6c,
	.zone_target_2 = 0x6c,
	.zone_target_3 = 0x6c,
	.zone_target_4 = 0x7e,
	.manual_current = 0x2f,
	.upper_curr_sel = 6,
	.lower_curr_sel = 3,
	.lens_loss_coeff = 6,
	.manual_als_config = 0x64,
	.als_enabled = 1,
};

static struct lm3554_platform_data mapphone_camera_flash_3554 = {
	.flags	= 0x0,
	.torch_brightness_def = 0xa0,
	.flash_brightness_def = 0x78,
	.flash_duration_def = 0x28,
	.config_reg_1_def = 0xe0,
	.config_reg_2_def = 0xf0,
	.vin_monitor_def = 0x00,
	.gpio_reg_def = 0x0,
};

static struct lm3559_platform_data mapphone_camera_flash_3559;

static struct lm3559_platform_data mapphone_camera_flash_3559 = {
	.flags		= (LM3559_PRIVACY | LM3559_TORCH |
				   LM3559_FLASH | LM3559_FLASH_LIGHT |
				   LM3559_MSG_IND | LM3559_ERROR_CHECK),
	.enable_reg_def = 0x00,
	.gpio_reg_def = 0x00,
	.adc_delay_reg_def = 0xc0,
	.vin_monitor_def = 0xff,
	.torch_brightness_def = 0x5b,
	.flash_brightness_def = 0xaa,
	.flash_duration_def = 0x0f,
	.flag_reg_def = 0x00,
	.config_reg_1_def = 0x6a,
	.config_reg_2_def = 0x00,
	.privacy_reg_def = 0x10,
	.msg_ind_reg_def = 0x00,
	.msg_ind_blink_reg_def = 0x1f,
	.pwm_reg_def = 0x00,
	.torch_enable_val = 0x1a,
	.flash_enable_val = 0x1b,
	.privacy_enable_val = 0x19,
	.pwm_val = 0x02,
	.msg_ind_val = 0xa0,
	.msg_ind_blink_val = 0x1f,
};

#ifdef CONFIG_SENSORS_AIRC
extern struct airc_platform_data mapphone_airc_data;
#endif
#ifdef CONFIG_INPUT_ALS_IR_ISL29030
extern struct isl29030_platform_data isl29030_pdata;
#endif

static struct i2c_board_info __initdata
	mapphone_i2c_bus1_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus2_board_info[I2C_BUS_MAX_DEVICES];
static struct i2c_board_info __initdata
	mapphone_i2c_bus3_board_info[I2C_BUS_MAX_DEVICES];

extern struct adp8870_backlight_platform_data adp8870_pdata;

static struct i2c_board_info __initdata
	mapphone_i2c_bus1_master_board_info[] = {
	{
		I2C_BOARD_INFO(QTOUCH_TS_NAME, 0x11),  /* Legacy val */
		.platform_data = NULL,
		.irq = OMAP_GPIO_IRQ(99),  /* Legacy val */
	},
	{
		I2C_BOARD_INFO(LD_LM3530_NAME, 0x38),
		.platform_data = &omap3430_als_light_data,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_LM_3530_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("adp8870", 0x2B),
		.platform_data = &adp8870_pdata,
	},
#ifdef CONFIG_SENSORS_AIRC
	{
		I2C_BOARD_INFO("airc", 0x50),
		.platform_data = &mapphone_airc_data,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_AIRC_INT_GPIO),
	},
#endif
#ifdef CONFIG_INPUT_ALS_IR_ISL29030
	{
		I2C_BOARD_INFO(LD_ISL29030_NAME, 0x44),
		.platform_data = &isl29030_pdata,
	},
#endif
};

extern struct lis331dlh_platform_data mapphone_lis331dlh_data;
extern struct akm8975_platform_data mapphone_akm8975_pdata;

static struct i2c_board_info __initdata
	mapphone_i2c_bus2_master_board_info[] = {
	{
		I2C_BOARD_INFO("akm8973", 0x1C),
		.irq = OMAP_GPIO_IRQ(MAPPHONE_AKM8973_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("lis331dlh", 0x19),
		.platform_data = &mapphone_lis331dlh_data,
	},
	{
		I2C_BOARD_INFO("kxtf9", 0x0F),
		.platform_data = &mapphone_kxtf9_data,
	},
	{
		I2C_BOARD_INFO(LD_LM3530_NAME, 0x38),
		.platform_data = &omap3430_als_light_data,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_LM_3530_INT_GPIO),
	},
	{
		I2C_BOARD_INFO("akm8975", 0x0C),
		.platform_data = &mapphone_akm8975_pdata,
		.irq = OMAP_GPIO_IRQ(MAPPHONE_AKM8975_INT_GPIO),
	},
};

#ifdef CONFIG_KEYBOARD_ADP5588
extern struct adp5588_kpad_platform_data mapphone_adp5588_pdata;
#endif

static struct i2c_board_info __initdata
	mapphone_i2c_bus3_master_board_info[] = {
	{
		I2C_BOARD_INFO("lm3554_led", 0x53),
		.platform_data = &mapphone_camera_flash_3554,
	},

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
	{
		I2C_BOARD_INFO("mt9p012", 0x36),
		.platform_data = &mapphone_mt9p012_platform_data,
	},
#endif

#ifdef CONFIG_VIDEO_OMAP3_HPLENS
	{
		I2C_BOARD_INFO("HP_GEN_LENS", 0x04),
		.platform_data = &mapphone_hplens_platform_data,
	},
#endif

#ifdef CONFIG_HDMI_TDA19989
	{
		I2C_BOARD_INFO("tda19989", 0x70),
	},
#endif

#ifdef CONFIG_KEYBOARD_ADP5588
	{
		I2C_BOARD_INFO("sholes-keypad", 0x34),
		.platform_data = &mapphone_adp5588_pdata,
		.irq = OMAP_GPIO_IRQ(26),
	},
#endif

#ifdef CONFIG_MOT_KEYBOARD_ADP5588
	{
		I2C_BOARD_INFO(ADP5588_KEYPAD_NAME, ADP5588_I2C_ADDRESS),
		.platform_data = &mapphone_adp5588_pdata,
	},
#endif

#if defined(CONFIG_VIDEO_OV8810)
	{
		I2C_BOARD_INFO("ov8810", OV8810_I2C_ADDR),
		.platform_data = &mapphone_ov8810_platform_data,
	},
#endif
#if defined(CONFIG_VIDEO_OV5650)
	{
		I2C_BOARD_INFO("ov5650", OV5650_I2C_ADDR),
		.platform_data = &mapphone_ov5650_platform_data,
	},
#endif
#if defined(CONFIG_VIDEO_CAM_ISE)
	{
		I2C_BOARD_INFO("camise", CAMISE_I2C_ADDR),
		.platform_data = &mapphone_camise_platform_data,
	},
#endif
#ifdef CONFIG_VIDEO_OMAP3_HPLENS
	{
		I2C_BOARD_INFO("HP_GEN_LENS", 0x04),
		.platform_data = &mapphone_hplens_platform_data,
	},
#endif

#if defined(CONFIG_LEDS_BD7885)
	{
		I2C_BOARD_INFO(BD7885_DEVICE_NAME, BD7885_SLAVE_ADDR),
	},
#endif	/* CONFIG_LEDS_BD7885 */

#if defined(CONFIG_LEDS_BU9847)
	{
		I2C_BOARD_INFO(BU9847_DEVICE_NAME, BU9847_SLAVE_ADDR),
	},
#endif/*CONFIG_LEDS_BU9847*/

#ifdef CONFIG_MOT_KEYBOARD_ADP5588
	{
		I2C_BOARD_INFO(ADP5588_KEYPAD_NAME, ADP5588_I2C_ADDRESS),
		.platform_data = &mapphone_adp5588_pdata,
	},
#endif
	/* LM3559 must be the last element in the array,
		new devices need to be added above */
	{
		I2C_BOARD_INFO("lm3559_led", 0x53),
		.platform_data = &mapphone_camera_flash_3559,
	},
};


static struct i2c_board_info *get_board_info
(
	char *dev_name,
	int bus_num,
	struct i2c_board_info *board_info_table,
	int size
)
{
	int i;
	char *entry_name;

	if (dev_name != NULL && board_info_table) {
		/* search for the name in the table */
		for (i = 0; i < size; i++) {
			entry_name = \
				board_info_table[i].type;
			if ( strncmp(\
					entry_name, \
					dev_name,   \
					strlen(entry_name)) == 0)
				return &board_info_table[i];
		}
	}

	return NULL;
}

void initialize_device_specific_data(void)
{
#ifdef CONFIG_ARM_OF
	u8 dev_available = 0;
	struct device_node *node;
	int len = 0;
	const uint32_t *val;

	/* Check camera flash led type */
	/* LM3559 */
	node = of_find_node_by_path(DT_PATH_LM3559);
	if (node != NULL) {
		val =
			of_get_property(node, "device_available", &len);
		if (val && len)
			dev_available =  *(u8 *)val;
	}

	if (dev_available) {
		val =
			of_get_property(
				node, "lm3559_flags", &len);
		if (val && len)
			mapphone_camera_flash_3559.flags = *val;
		else
			pr_err("%s: Can't get flags\n", __func__);
	}

	/* LM3554 */
	node = of_find_node_by_path(DT_PATH_LM3554);
	if (node != NULL) {
		val =
			of_get_property(node, "device_available", &len);
		if (val && len)
			mapphone_camera_flash_3554.flags = *val;
		val = of_get_property(node, "flash_duration_def", &len);
		if (val && len)
			mapphone_camera_flash_3554.flash_duration_def = *val;
	}

#if defined(CONFIG_VIDEO_MIPI_DLI_TEST)
	/* MIPI DLI */
	node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (node != NULL) {
		val =
		of_get_property(node, "feature_mipi_cam", NULL);
		if (NULL != val) {
			if (*val) {
				platform_device_register(
					&mapphone_mipi_dli_device);
				printk(KERN_INFO "Enabling MIPI DLI Test");
			}
		}
	}
#endif

#endif /*CONFIG_ARM_OF*/
}

static int initialize_i2c_bus_info
(
	int bus_num,
	struct i2c_board_info *board_info,
	int info_size,
	struct i2c_board_info *master_board_info,
	int master_info_size
)
{
	int dev_cnt = 0;
#ifdef CONFIG_ARM_OF
	struct device_node *bus_node;
	const void *feat_prop;
	char *device_names;
	char dev_name[I2C_MAX_DEV_NAME_LEN];
	int device_name_len, i, j;
	struct i2c_board_info *master_entry;
	char prop_name[I2C_BUS_PROP_NAME_LEN];

	j = 0;

	bus_node = of_find_node_by_path(DT_PATH_I2C);
	if (bus_node == NULL || board_info == NULL)
		return dev_cnt;

	snprintf(prop_name, I2C_BUS_PROP_NAME_LEN,
		"bus%1ddevices", bus_num);

	feat_prop = of_get_property(bus_node,
			prop_name, NULL);
	if (NULL != feat_prop) {
	if (bus_num==3) {
		device_names = "camise,HP_GEN_LENS,lm3554_led,mt9p012" 

	} else {
		device_names = (char *)feat_prop;
	}
		printk(KERN_INFO
			"I2C-%d devices: %s\n", bus_num, device_names);
		device_name_len = strlen(device_names);

		memset(dev_name, 0x0, I2C_MAX_DEV_NAME_LEN);

		for (i = 0; i < device_name_len; i++) {

			if (device_names[i] != '\0' &&
				device_names[i] != ',')
				dev_name[j++] = device_names[i];
			/* parse for ',' in string */
			if (device_names[i] == ',' ||
				(i == device_name_len-1)) {

				if (dev_cnt < info_size) {
					master_entry =
						get_board_info(dev_name,
							bus_num,
							master_board_info,
							master_info_size);
					if (master_entry != NULL) {
						memcpy(
							&board_info[dev_cnt++],
							master_entry,
							sizeof(
							struct i2c_board_info));
						printk(KERN_INFO
							"%s -> I2C bus-%d\n",
							master_entry->type,
							bus_num);

					}
					j = 0;
					memset(
							dev_name,
							0x0,
							I2C_MAX_DEV_NAME_LEN);
				}
			}
		}
	}
#endif
	return dev_cnt;
}

static int __init mapphone_i2c_init(void)
{
	int i2c_bus_devices = 0;

	initialize_device_specific_data();

	/* Populate I2C bus 1 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			1, mapphone_i2c_bus1_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_bus1_master_board_info,
			ARRAY_SIZE(mapphone_i2c_bus1_master_board_info));
	if (i2c_bus_devices != 0)
		omap_register_i2c_bus(
			1, 400,
			mapphone_i2c_bus1_board_info, i2c_bus_devices);

	/* Populate I2C bus 2 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			2, mapphone_i2c_bus2_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_bus2_master_board_info,
			ARRAY_SIZE(mapphone_i2c_bus2_master_board_info));
	if (i2c_bus_devices != 0)
		omap_register_i2c_bus(
			2, 400,
			mapphone_i2c_bus2_board_info, i2c_bus_devices);

	/* Populate I2C bus 3 devices */
	i2c_bus_devices = initialize_i2c_bus_info(
			3, mapphone_i2c_bus3_board_info,
			I2C_BUS_MAX_DEVICES,
			mapphone_i2c_bus3_master_board_info,
			ARRAY_SIZE(mapphone_i2c_bus3_master_board_info));
	if (i2c_bus_devices != 0)
		omap_register_i2c_bus(
			3, 400,
			mapphone_i2c_bus3_board_info, i2c_bus_devices);

	return 0;
}

arch_initcall(mapphone_i2c_init);

extern void __init mapphone_spi_init(void);
extern void __init mapphone_flash_init(void);
extern void __init mapphone_gpio_iomux_init(void);


static void __init mapphone_sdrc_init(void)
{
	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_cfg_reg(H16_34XX_SDRC_CKE0);
	omap_cfg_reg(H17_34XX_SDRC_CKE1);
}

static void __init mapphone_serial_init(void)
{
	int bpwake_strobe_gpio = MAPPHONE_BPWAKE_STROBE_GPIO;
#ifdef CONFIG_ARM_OF
	struct device_node *uart_node;
	const void *uart_prop;
	struct device_node *dt_node;
	const void *dt_prop;

	uart_node = of_find_node_by_path(DT_PATH_UART);
	if (uart_node) {
		uart_prop = of_get_property(uart_node,
					DT_PROP_UART_HW_FLOW_CONTROL, NULL);
		if (uart_prop)
			omap_serial_ctsrts_init((unsigned char *) uart_prop);

		uart_prop = of_get_property(uart_node,
					DT_PROP_UART_PORT_FOR_GPS, NULL);
		if (uart_prop)
			omap_uart_set_gps_port(*(int *) uart_prop);

		uart_prop = of_get_property(uart_node,
					DT_PROP_UART_PADCONF_FOR_UART0, NULL);
		if (uart_prop)
			omap_uart_set_uart0_padconf(*(int *) uart_prop);

		of_node_put(uart_node);
	}

	/* Disable ttyS2 if uart debug is disabled in the device tree.
	 * This disables the serial console, preventing headset static
	 * that occured when the kernel wrote to the serial console.
	 */
	dt_node = of_find_node_by_path(DT_HIGH_LEVEL_FEATURE);
	if (NULL != dt_node) {
		dt_prop = of_get_property(dt_node,
				DT_HIGH_LEVEL_FEATURE_HEADSET_UART_EN, NULL);
		if (NULL != dt_prop)
			if (*(u8 *)dt_prop == 0)
				mapphone_uart_config.enabled_uarts &= ~(1 << 2);

		of_node_put(dt_node);
	}
#endif

	omap_cfg_reg(AA8_34XX_UART1_TX);
	omap_cfg_reg(Y8_34XX_UART1_RX);
	omap_cfg_reg(AA9_34XX_UART1_RTS);
	omap_cfg_reg(W8_34XX_UART1_CTS);
	omap_cfg_reg(AA25_34XX_UART2_TX);
	omap_cfg_reg(AD25_34XX_UART2_RX);
	omap_cfg_reg(AB25_34XX_UART2_RTS);
	omap_cfg_reg(AB26_34XX_UART2_CTS);
	bpwake_strobe_gpio = get_gpio_by_name("ipc_bpwake_strobe");
	if (bpwake_strobe_gpio < 0)
		bpwake_strobe_gpio = MAPPHONE_BPWAKE_STROBE_GPIO;
	omap_serial_init(bpwake_strobe_gpio, 0x01);
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


#ifdef CONFIG_OMAP_SMARTREFLEX
/* TODO : Implement CPCAP init */
int __init omap_pmic_srinit(void)
{
	printk(KERN_INFO "\nMAPPHONE PMIC SR init...\n");
	return 0;
}
/**
 * @brief omap_pmic_voltage_ramp_delay - how much should this pmic ramp delay
 * Various PMICs have different ramp up and down delays.
 *  CPCAP SMPS slew rate (min) 13mV/uS, step size 12.5mV,
 *  2us added as buffer
 *  removed time to send bypass command 46us + waiting delay.
 *
 * @param target_vsel - targetted voltage selction
 * @param current_vsel - current voltage selection
 *
 * @return delay in uSeconds
 */
#define COUNT_TIMEOUT_VCBYPASS   150
u32 omap_pmic_voltage_ramp_delay(u8 srid, u8 target_vsel, u8 current_vsel)
{
	u32 cpcap_smps_steps, cpcap_smps_delay;
	u8 slave_addr = (srid == SR1) ? CPCAP_SRI2C_SLAVE_ADDR_VDD1 :
			CPCAP_SRI2C_SLAVE_ADDR_VDD2;
	u16 timeout = COUNT_TIMEOUT_VCBYPASS;
	int ret = -1;

	ret = vc_send_command(slave_addr, CPCAP_SMPS_VOL_OPP2,
			target_vsel,
			&timeout);
	if (target_vsel < current_vsel)
		return 0;

	cpcap_smps_steps = abs(target_vsel - current_vsel);
	cpcap_smps_delay = ((cpcap_smps_steps * 125) / 130) + 2;
	if (!ret) {
		if (cpcap_smps_delay > (COUNT_TIMEOUT_VCBYPASS - timeout))
			cpcap_smps_delay -= (COUNT_TIMEOUT_VCBYPASS - timeout);
		else
			return 0;
	}

	return cpcap_smps_delay;
}
#endif
#ifdef CONFIG_OMAP_VC_BYPASS_UPDATE
/* VCBypass mode for CPCAP chip. */
int omap_pmic_voltage_cmds(u8 srid, u8 target_vsel)
{
	u8 slave_addr = (srid == SR1) ? CPCAP_SRI2C_SLAVE_ADDR_VDD1 :
			CPCAP_SRI2C_SLAVE_ADDR_VDD2;
	u16 timeout = COUNT_TIMEOUT_VCBYPASS;

	return vc_send_command(slave_addr, CPCAP_SRI2C_VDD_CONTROL,
			target_vsel, &timeout);
 }
#endif         /* ifdef CONFIG_OMAP_VC_BYPASS_UPDATE */

/* Mapphone specific PM */

extern void omap_uart_block_sleep(int num);
static struct wake_lock baseband_wakeup_wakelock;
static irqreturn_t mapphone_bpwake_irqhandler(int irq, void *unused)
{
	omap_uart_block_sleep(0);
	/*
	 * uart_block_sleep keeps uart clock active for 500 ms,
	 * prevent suspend for 1 sec to be safe
	 */
	wake_lock_timeout(&baseband_wakeup_wakelock, HZ);
	return IRQ_HANDLED;
}

static int mapphone_bpwake_probe(struct platform_device *pdev)
{
	int rc;

	int apwake_trigger_gpio;
	apwake_trigger_gpio = get_gpio_by_name("ipc_apwake_trigger");
	if (apwake_trigger_gpio < 0)
		apwake_trigger_gpio = MAPPHONE_APWAKE_TRIGGER_GPIO;

	gpio_request(apwake_trigger_gpio, "BP -> AP IPC trigger");
	gpio_direction_input(apwake_trigger_gpio);

	wake_lock_init(&baseband_wakeup_wakelock, WAKE_LOCK_SUSPEND, "bpwake");

	rc = request_irq(gpio_to_irq(apwake_trigger_gpio),
			 mapphone_bpwake_irqhandler,
			 IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			 "Remote Wakeup", NULL);
	if (rc) {
		wake_lock_destroy(&baseband_wakeup_wakelock);
		printk(KERN_ERR
		       "Failed requesting APWAKE_TRIGGER irq (%d)\n", rc);
		return rc;
	}
	enable_irq_wake(gpio_to_irq(apwake_trigger_gpio));
	return 0;
}

static int mapphone_bpwake_remove(struct platform_device *pdev)
{
	int apwake_trigger_gpio ;

	wake_lock_destroy(&baseband_wakeup_wakelock);
	apwake_trigger_gpio = get_gpio_by_name("ipc_apwake_trigger");
	if (apwake_trigger_gpio < 0)
		apwake_trigger_gpio = MAPPHONE_APWAKE_TRIGGER_GPIO;
	free_irq(gpio_to_irq(apwake_trigger_gpio), NULL);
	return 0;
}

static int mapphone_bpwake_suspend(struct platform_device *pdev,
					pm_message_t state)
{
	return 0;
}

static int mapphone_bpwake_resume(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver mapphone_bpwake_driver = {
	.probe		= mapphone_bpwake_probe,
	.remove		= mapphone_bpwake_remove,
	.suspend	= mapphone_bpwake_suspend,
	.resume		= mapphone_bpwake_resume,
	.driver		= {
		.name		= "mapphone_bpwake",
		.owner		= THIS_MODULE,
	},
};

static struct platform_device mapphone_bpwake_device = {
	.name		= "mapphone_bpwake",
	.id		= -1,
	.num_resources	= 0,
};

/* Choose cold or warm reset
 *    RST_TIME1>4ms will trigger CPCAP to trigger a system cold reset */
static void mapphone_pm_set_reset(char cold)
{
	if (cold) {
		/* Configure RST_TIME1 to 6ms  */
		prm_rmw_mod_reg_bits(OMAP_RSTTIME1_MASK,
		0xc8<<OMAP_RSTTIME1_SHIFT,
		OMAP3430_GR_MOD,
		OMAP3_PRM_RSTTIME_OFFSET);
	} else {
		/* Configure RST_TIME1 to 30us  */
		prm_rmw_mod_reg_bits(OMAP_RSTTIME1_MASK,
		0x01<<OMAP_RSTTIME1_SHIFT,
		OMAP3430_GR_MOD,
		OMAP3_PRM_RSTTIME_OFFSET);
	}
}

static int mapphone_pm_reboot_call(struct notifier_block *this,
			unsigned long code, void *cmd)
{
	int result = NOTIFY_DONE;

	if (code == SYS_RESTART) {
		/* set cold reset */
		mapphone_pm_set_reset(1);
	}

	return result;
}

static struct notifier_block mapphone_pm_reboot_notifier = {
	.notifier_call = mapphone_pm_reboot_call,
};

#ifdef CONFIG_MEM_DUMP

#define WARMRESET 1
#define COLDRESET 0

static unsigned long reset_status = COLDRESET ;
#endif


/* This is VIO HW issue WA
 * Issue: The VIO (SW3 @CPCAP) is set to LP mode in retention mode.
 * When VIO is set to LP in retention, it switch to PFM which can
 * only source a few mA of current. Since WiFi/BT can wakeup
 * independent of OMAP, it outdraws the VIO supply in LP mode,
 * causing voltage to dip significantly which eventually crashes the WiFi.
 * VIO also source eMMC, this cause the MMC port to crashes,
 * which then eventually propagates to phone freeze and panic.
 *
 * WA: set VIO active when Wifi or BI is on to prevent volt dip.
 * Only Apply the WA on HW without DC-DC converter put in VIO supply.
 */
static DEFINE_MUTEX(vio_access);
static struct cpcap_device *misc_cpcap;

static int cpcap_vio_active_probe(struct platform_device *pdev)
{
	misc_cpcap = pdev->dev.platform_data;
	return 0;
}

static struct platform_driver cpcap_vio_active_driver = {
	.probe  = cpcap_vio_active_probe,
	.driver = {
		.name   = "cpcap_vio_active",
		.owner  = THIS_MODULE,
	},
};

void change_vio_mode(int source, int value)
{
	static int bt_request;
	static int wifi_request;

	/*If have DC-DC converter, skip it*/
	if (misc_cpcap == NULL)
		return;

	mutex_lock(&vio_access);
	if (source == 0)
		bt_request = value;
	else if (source == 1)
		wifi_request = value;
	else {
		printk(KERN_ERR "unknown source to vio ");
		mutex_unlock(&vio_access);
		return;
	}

	if (bt_request | wifi_request)
		cpcap_regacc_write(misc_cpcap, CPCAP_REG_S3C,
				0, CPCAP_BIT_SW3STBY);
	else
		cpcap_regacc_write(misc_cpcap, CPCAP_REG_S3C,
				CPCAP_BIT_SW3STBY, CPCAP_BIT_SW3STBY);

	mutex_unlock(&vio_access);
}

static void mapphone_pm_init(void)
{
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

	/* Configure BP <-> AP wake pins */
	omap_cfg_reg(AA21_34XX_GPIO157_OUT);
	omap_cfg_reg(AE6_34XX_GPIO141_DOWN);

	platform_device_register(&mapphone_bpwake_device);
	platform_driver_register(&mapphone_bpwake_driver);
	cpcap_driver_register(&cpcap_vio_active_driver);

#ifdef CONFIG_MEM_DUMP
	if (reset_status == COLDRESET)
		mapphone_pm_set_reset(1);
	else
		mapphone_pm_set_reset(0);
#else
	/* set cold reset, will move to warm reset once ready */
	mapphone_pm_set_reset(1);
#endif
	register_reboot_notifier(&mapphone_pm_reboot_notifier);
}

#ifdef CONFIG_MEM_DUMP
static struct proc_dir_entry *proc_entry ;

ssize_t reset_proc_read(char *page, char **start, off_t off, \
   int count, int *eof, void *data)
{
	int len ;
    /* don't visit offset */
	if (off > 0) {
		*eof = 1 ;
		return 0 ;
	}
	len = snprintf(page, sizeof(page), "%x\n", (unsigned int)reset_status) ;
	return len ;
}

ssize_t reset_proc_write(struct file *filp, const char __user *buff, \
  unsigned long len, void *data)
{
#define MAX_UL_LEN 8
	char k_buf[MAX_UL_LEN] ;
	int count = min((unsigned long)MAX_UL_LEN, len) ;
	int ret ;

	if (copy_from_user(k_buf, buff, count)) {
		ret = -EFAULT ;
		goto err ;
	} else{
		if (k_buf[0] == '0') {
			reset_status = COLDRESET;
			mapphone_pm_set_reset(1);
			printk(KERN_ERR"switch to cold reset\n");
		} else if (k_buf[0] == '1') {
			reset_status = WARMRESET;
			mapphone_pm_set_reset(0);
			printk(KERN_ERR"switch to warm reset\n");
		} else{
			ret = -EFAULT;
			goto err;
		}
	return count ;
	}
err:
	return ret ;
}

static void  reset_proc_init(void)
{
	proc_entry = create_proc_entry("reset_proc", 0660, NULL);
	if (proc_entry == NULL) {
		printk(KERN_INFO"Couldn't create proc entry\n") ;
	} else{
		proc_entry->read_proc = reset_proc_read ;
		proc_entry->write_proc = reset_proc_write ;
		/* proc_entry->owner = THIS_MODULE ; */
	}
}

int __init warmreset_init(char *s)
{
	/* configure to warmreset */
	reset_status = WARMRESET;
	mapphone_pm_set_reset(0);
	return 1;
}
__setup("warmreset_debug=", warmreset_init);
#endif

static void __init config_wlan_gpio(void)
{
	/* WLAN PE and IRQ */
	omap_cfg_reg(AE22_34XX_GPIO186_OUT);
	omap_cfg_reg(J8_3430_GPIO65);
}

static void __init config_mmc2_init(void)
{
	u32 val;

	/* MMC2 */
	omap_cfg_reg(AE2_34XX_MMC2_CLK);
	omap_cfg_reg(AG5_34XX_MMC2_CMD);
	omap_cfg_reg(AH5_34XX_MMC2_DAT0);
	omap_cfg_reg(AH4_34XX_MMC2_DAT1);
	omap_cfg_reg(AG4_34XX_MMC2_DAT2);
	omap_cfg_reg(AF4_34XX_MMC2_DAT3);

	/* Set internal loopback clock */
	val = omap_ctrl_readl(OMAP343X_CONTROL_DEVCONF1);
	omap_ctrl_writel((val | OMAP2_MMCSDIO2ADPCLKISEL),
				OMAP343X_CONTROL_DEVCONF1);
}

/* must match value in drivers/w1/w1_family.h */
#define W1_EEPROM_DS2502        0x89
static struct omap2_hdq_platform_config mapphone_hdq_data = {
	.mode = OMAP_SDQ_MODE,
	.id = W1_EEPROM_DS2502,
};

static int __init omap_hdq_init(void)
{
	omap_cfg_reg(J25_34XX_HDQ_SIO);
	omap_hdq_device.dev.platform_data = &mapphone_hdq_data;
	return platform_device_register(&omap_hdq_device);
}

static int mapphone_wl1271_init(void);
static int mapphone_wl1271_release(void);
static int mapphone_wl1271_enable(void);
static int mapphone_wl1271_disable(void);

static struct wl127x_rfkill_platform_data mapphone_wl1271_pdata = {
	.bt_nshutdown_gpio = -1,
	.pwr_ctl = -1,
	.fm_enable_gpio = -1,
	.bt_hw_init = mapphone_wl1271_init,
	.bt_hw_release = mapphone_wl1271_release,
	.bt_hw_enable = mapphone_wl1271_enable,
	.bt_hw_disable = mapphone_wl1271_disable,
};

static struct wl127x_test_platform_data mapphone_wl1271_test_pdata;

#define is_BTWAKE_present() (mapphone_wl1271_test_pdata.btwake_gpio >= 0)
#define is_BTHOSTWAKE_present() (mapphone_wl1271_test_pdata.hostwake_gpio >= 0)

static int mapphone_wl1271_init(void)
{
	int rc = 0;

	/* wl1271 BT chip init sequence */
	gpio_direction_output(mapphone_wl1271_pdata.bt_nshutdown_gpio, 0);
	msleep(5);
	gpio_set_value(mapphone_wl1271_pdata.bt_nshutdown_gpio, 1);
	msleep(10);
	gpio_set_value(mapphone_wl1271_pdata.bt_nshutdown_gpio, 0);
	msleep(5);

	/* Reserve BT wake and hostwake GPIOs */
	if (is_BTWAKE_present()) {
		rc = gpio_request(mapphone_wl1271_test_pdata.btwake_gpio,
					"wl127x_wake_gpio");
		if (unlikely(rc))
			return rc;
	}

	if (is_BTHOSTWAKE_present()) {
		rc = gpio_request(mapphone_wl1271_test_pdata.hostwake_gpio,
					"wl127x_hostwake_gpio");
		if (unlikely(rc))
			return rc;
	}

	if (is_BTWAKE_present())
		gpio_direction_output(mapphone_wl1271_test_pdata.btwake_gpio,
					1);
	if (is_BTHOSTWAKE_present())
		gpio_direction_input(mapphone_wl1271_test_pdata.hostwake_gpio);

	return 0;
}

static int mapphone_wl1271_release(void)
{
	if (is_BTWAKE_present())
		gpio_free(mapphone_wl1271_test_pdata.btwake_gpio);

	if (is_BTHOSTWAKE_present())
		gpio_free(mapphone_wl1271_test_pdata.hostwake_gpio);

	return 0;
}

static int mapphone_wl1271_enable(void)
{
	/* FIXME
	 * Change vio mode dynamically if necessary
	 */
	change_vio_mode(0, 1);
	return 0;
}

static int mapphone_wl1271_disable(void)
{
	/* FIXME
	 * Change vio mode dynamically if necessary
	 */
	change_vio_mode(0, 0);
	return 0;
}

static struct platform_device mapphone_wl1271_device = {
	.name = "wl127x-rfkill",
	.id = 0,
	.dev.platform_data = &mapphone_wl1271_pdata,
};

static struct platform_device mapphone_wl1271_test_device = {
	.name = "wl127x-test",
	.id = 0,
	.dev.platform_data = &mapphone_wl1271_test_pdata,
};

static void __init mapphone_bt_init(void)
{
#ifdef CONFIG_ARM_OF
	int bt_enable_gpio;
	int bt_wake_gpio;
	int bt_host_wake_gpio;

	bt_enable_gpio = get_gpio_by_name("bt_reset_b");
	if (bt_enable_gpio < 0) {
		printk(KERN_DEBUG "mapphone_bt_init: cannot retrieve bt_reset_b gpio from device tree\n");
		bt_enable_gpio = -1;
	}
	mapphone_wl1271_pdata.bt_nshutdown_gpio = bt_enable_gpio;

	/* if no DC-DC converter, expose rfkill */
	if (!is_cpcap_vio_supply_converter()) {
		printk(KERN_DEBUG "mapphone_bt_init: misc_cpcap not null\n");
		mapphone_wl1271_pdata.pwr_ctl = 1;
	}

	bt_wake_gpio = get_gpio_by_name("bt_wake_b");
	if (bt_wake_gpio < 0) {
		printk(KERN_DEBUG "mapphone_bt_init: cannot retrieve bt_wake_b gpio from device tree\n");
		bt_wake_gpio = -1;
	}
	mapphone_wl1271_test_pdata.btwake_gpio = bt_wake_gpio;

	bt_host_wake_gpio = get_gpio_by_name("bt_host_wake_b");
	if (bt_host_wake_gpio < 0) {
		printk(KERN_DEBUG "mapphone_bt_init: cannot retrieve bt_host_wake_b gpio from device tree\n");
		bt_host_wake_gpio = -1;
	}
	mapphone_wl1271_test_pdata.hostwake_gpio = bt_host_wake_gpio;
#endif

	/* The 3 mux settings below are default; device tree will overwrite */

	/* Mux setup for Bluetooth chip-enable */
	omap_cfg_reg(T3_34XX_GPIO179);

	/* Mux setup for BT wake GPIO and hostwake GPIO */
	if (is_BTWAKE_present())
		omap_cfg_reg(AF21_34XX_GPIO8_OUT);
	if (is_BTHOSTWAKE_present())
		omap_cfg_reg(W7_34XX_GPIO178_DOWN);

	platform_device_register(&mapphone_wl1271_device);
	platform_device_register(&mapphone_wl1271_test_device);
}


static struct omap_vout_config mapphone_vout_platform_data = {
	.max_width = 1280,
	.max_height = 720,
	.max_buffer_size = 0x1C3000,
	.num_buffers = 9, /* 8 for camera/video playback, 1 for HDMI */
	.num_devices = 2,
	.device_ids = {1, 2},
};

static struct platform_device mapphone_vout_device = {
	.name = "omapvout",
	.id = -1,
	.dev = {
		.platform_data = &mapphone_vout_platform_data,
	},
};
static void __init mapphone_vout_init(void)
{
#ifdef CONFIG_ARM_OF
	struct device_node *panel_node;
	const void *panel_prop;
	struct omap_vout_config *platform_data;

	panel_node = of_find_node_by_path(DT_PATH_VIDEO_OUT);

	if (panel_node != NULL) {
		platform_data = (struct omap_vout_config *)
			mapphone_vout_device.dev.platform_data;

		panel_prop = of_get_property(panel_node, "max_width", NULL);
		if (panel_prop)
			platform_data->max_width = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "max_height", NULL);
		if (panel_prop)
			platform_data->max_height = *(u16 *)panel_prop;

		panel_prop = of_get_property(panel_node, "max_buffer_size",
						 NULL);
		if (panel_prop)
			platform_data->max_buffer_size = *(u32 *)panel_prop;

		of_node_put(panel_node);
	}
#endif
	platform_device_register(&mapphone_vout_device);
}

#ifdef CONFIG_ANDROID_RAM_CONSOLE
#define RAM_CONSOLE_START   0x8E000000
#define RAM_CONSOLE_SIZE    0x20000
static struct resource ram_console_resource = {
       .start  = RAM_CONSOLE_START,
       .end    = (RAM_CONSOLE_START + RAM_CONSOLE_SIZE - 1),
       .flags  = IORESOURCE_MEM,
};

static struct platform_device ram_console_device = {
       .name = "ram_console",
       .id = 0,
       .num_resources  = 1,
       .resource       = &ram_console_resource,
};

static inline void mapphone_ramconsole_init(void)
{
	platform_device_register(&ram_console_device);
}

static inline void omap2_ramconsole_reserve_sdram(void)
{
	reserve_bootmem(RAM_CONSOLE_START, RAM_CONSOLE_SIZE, 0);
}
#else
static inline void mapphone_ramconsole_init(void) {}

static inline void omap2_ramconsole_reserve_sdram(void) {}
#endif


static struct platform_device mapphone_sgx_device = {
       .name                   = "pvrsrvkm",
       .id             = -1,
};
static struct platform_device mapphone_omaplfb_device = {
	.name			= "omaplfb",
	.id			= -1,
};


static void __init mapphone_sgx_init(void)
{
	platform_device_register(&mapphone_sgx_device);
	platform_device_register(&mapphone_omaplfb_device);
}

static void __init mapphone_bp_model_init(void)
{
#ifdef CONFIG_OMAP_RESET_CLOCKS
	struct clk *clkp;
#endif

#ifdef CONFIG_ARM_OF
	struct device_node *bp_node;
	const void *bp_prop;

	if ((bp_node = of_find_node_by_path(DT_PATH_CHOSEN))) {
		if ((bp_prop = of_get_property(bp_node, \
			DT_PROP_CHOSEN_BP, NULL)))
			bp_model = (char *)bp_prop;

		of_node_put(bp_node);
	}
#endif
#ifdef CONFIG_OMAP_RESET_CLOCKS
	/* Enable sad2d iclk */
	clkp = clk_get(NULL, "sad2d_ick");
	if (clkp)
		clk_enable(clkp);
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
	.probe		= cpcap_charger_connected_probe,
	.remove		= cpcap_charger_connected_remove,
	.driver		= {
		.name	= "cpcap_charger_connected",
		.owner	= THIS_MODULE,
	},
};

static void __init mapphone_power_off_init(void)
{
	gpio_request(MAPPHONE_POWER_OFF_GPIO, "mapphone power off");
	gpio_direction_output(MAPPHONE_POWER_OFF_GPIO, 1);
	omap_cfg_reg(AB1_34XX_GPIO176_OUT);

	/* config gpio176 into safe mode with the pull up enabled to avoid
	 * glitch at reboot */
	omap_writew(0x1F, 0x480021D2);
	pm_power_off = mapphone_pm_power_off;

	platform_driver_register(&cpcap_charger_connected_driver);
}

static void __init mapphone_init(void)
{
	int ret = 0;
	struct kobject *properties_kobj = NULL;

	omap_board_config = mapphone_config;
	omap_board_config_size = ARRAY_SIZE(mapphone_config);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
				 &mapphone_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");

	mapphone_bp_model_init();
	mapphone_padconf_init();
#ifdef CONFIG_EMU_UART_DEBUG
	/* emu-uart function will override devtree iomux setting */
	activate_emu_uart();
#endif
	mapphone_gpio_mapping_init();
	mapphone_ramconsole_init();
	mapphone_mdm_ctrl_init();
	mapphone_spi_init();
	mapphone_cpcap_client_init();
	mapphone_flash_init();
	mapphone_serial_init();
	mapphone_als_init();
	mapphone_panel_init();
	mapphone_sensors_init();
	mapphone_camera_init();
	mapphone_touch_init();
	mapphone_audio_init();
	usb_musb_init();
	mapphone_ehci_init();
	mapphone_sdrc_init();
	mapphone_pm_init();
	config_mmc2_init();
	config_wlan_gpio();
	omap_hdq_init();
	mapphone_bt_init();
#ifdef mapphone_MMCPROBE_ENABLED
	mapphone_mmcprobe_init();
#else
	mapphone_hsmmc_init();
#endif
	mapphone_vout_init();
	mapphone_sgx_init();
	mapphone_power_off_init();
	mapphone_gadget_init();
	mapphone_sim_init();
#ifdef CONFIG_MEM_DUMP
    reset_proc_init();
#endif
}

static void __init mapphone_map_io(void)
{
	omap2_ramconsole_reserve_sdram();
	omap2_set_globals_343x();
	omap2_map_common_io();
}

MACHINE_START(MAPPHONE, "mapphone_")
	/* Maintainer: Motorola, Inc. */
	.phys_io	= 0x48000000,
	.io_pg_offst	= ((0xfa000000) >> 18) & 0xfffc,
	.boot_params	= 0x80C00100,
	.map_io		= mapphone_map_io,
	.init_irq	= mapphone_init_irq,
	.init_machine	= mapphone_init,
	.timer		= &omap_timer,
MACHINE_END

