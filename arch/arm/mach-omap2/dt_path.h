/*
 * File: arch/arm/plat-omap/include/mach/dt_path.h
 *
 * Copyright (C) 2009-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */



#ifndef _MACH_DT_PATH_H
#define _MACH_DT_PATH_H
#ifdef __KERNEL__

/* Chosen */
#define DT_PATH_CHOSEN		"/Chosen@0"
#define DT_PROP_CHOSEN_BP	"bp_model"
#define DT_PROP_CHOSEN_BP_LEN	16
#define DT_PROP_CHOSEN_BP_COUNT "bp_count"
#define DT_PROP_CHOSEN_OMAP3630REV       "omap3630_rev"
#define DT_PROP_CHOSEN_USB_PROD_NAME "usb_id_prod_name"
#define DT_PROP_CHOSEN_USB_PIDS "usb_pids"
#define DT_PROP_CHOSEN_USB_NLUNS "usb_nluns"
#define DT_PROP_CHOSEN_USB_CDROM_LUN_NUM "usb_cdrom_lun_num"
#define DT_PROP_CHOSEN_MODEM_IFACE_NUM "modem_interface_num"
#define DT_PROP_CHOSEN_USBHS_PORT0 "usbhs_port0_mode"
#define DT_PROP_CHOSEN_USBHS_PORT1 "usbhs_port1_mode"
#define DT_PROP_CHOSEN_USBHS_PORT2 "usbhs_port2_mode"
#define DT_PROP_CHOSEN_IPC_USB_TS_CLK_SRC "ipc_usb_ts_clk_src_num"
#define DT_PROP_CHOSEN_MUSBHS_EXTPOWER "musbhs_ext_pwr_limit"

/* Keypad Node */
#define DT_PATH_KEYPAD		"/System@0/Keypad@0"
#define DT_PROP_KEYPAD_ROWS	"rows"
#define DT_PROP_KEYPAD_COLS	"columns"
#define DT_PROP_KEYPAD_ROWREG	"rowregister"
#define DT_PROP_KEYPAD_COLREG	"columnregister"
#define DT_PROP_KEYPAD_MAPNUM	"mapnum"
#define DT_PROP_KEYPAD_MAPS	"maps"
#define DT_PROP_KEYPAD_CLOSED_MAPS "closed_maps"
#define DT_PROP_KEYPAD_NAME	"name"
#define DT_PROP_KEYPAD_ADP5588  "adp5588_keypad"

/* Keyreset Node */
#define DT_PATH_KEYRESET	"/System@0/Keyreset@0"
#define DT_PROP_KEYRESET_DOWN	"keydown"
#define DT_PROP_KEYRESET_UP	"keyup"
#define DT_PROP_KEYRESET_CRASH	"crash"

/* GPIODev Node */
#define DT_PATH_GPIOGEV		"/System@0/GPIODev@0"
#define DT_PROP_GPIODEV_INIT	"init"

/* MUX Node */
#define DT_PATH_MUX		"/System@0/IOMUX@0"
#define DT_PROP_MUX_PAD	"padinit"
#define DT_PROP_MUX_PADWKUPS	"padwkupsinit"
#define DT_PROP_MUX_OFFMODE		"offmodeinit"
#define DT_PROP_MUX_OFFMODEWKUPS	"offmodewkupsinit"

/* Touch Node */
#define DT_PATH_TOUCH			"/System@0/I2C@0/TouchOBP@0"
#define DT_PROP_TOUCH_KEYMAP		"touch_key_map"
#define DT_PROP_TOUCH_I2C_ADDRESS       "i2c,address"
#define DT_PROP_TOUCH_BOOT_I2C_ADDRESS  "boot_i2c_address"
#define DT_PROP_TOUCH_KEYMAP		"touch_key_map"
#define DT_PROP_TOUCH_NUM_TOUCH_KEYS	"number_of_touch_keys"
#define DT_PROP_TOUCH_FLAGS		"touchobp-flags"
#define DT_PROP_TOUCH_CHECKSUM		"nv_checksum"
#define DT_PROP_TOUCH_ABS_MIN_X		"abs_min_x"
#define DT_PROP_TOUCH_ABS_MAX_X		"abs_max_x"
#define DT_PROP_TOUCH_ABS_MIN_Y		"abs_min_y"
#define DT_PROP_TOUCH_ABS_MAX_Y		"abs_max_y"
#define DT_PROP_TOUCH_ABS_MIN_P		"abs_min_p"
#define DT_PROP_TOUCH_ABS_MAX_P		"abs_max_p"
#define DT_PROP_TOUCH_ABS_MIN_W		"abs_min_w"
#define DT_PROP_TOUCH_ABS_MAX_W		"abs_max_w"
#define DT_PROP_TOUCH_FUZZ_X		"fuzz_x"
#define DT_PROP_TOUCH_FUZZ_Y		"fuzz_y"
#define DT_PROP_TOUCH_FUZZ_P		"fuzz_p"
#define DT_PROP_TOUCH_FUZZ_W		"fuzz_w"
#define DT_PROP_TOUCH_X_DELTA           "x_delta"
#define DT_PROP_TOUCH_Y_DELTA           "y_delta"
#define DT_PROP_TOUCH_KEY_ARRAY_MAP	"key_array_map"
#define DT_PROP_TOUCH_KEY_ARRAY_COUNT	"key_array_count"
#define DT_PROP_TOUCH_T7		"obj_t7"	/* power_cfg */
#define DT_PROP_TOUCH_T8		"obj_t8"	/* acquire_cfg */
#define DT_PROP_TOUCH_T9		"obj_t9"	/* multi_touch_cfg */
#define DT_PROP_TOUCH_T15		"obj_t15"	/* key_array */
#define DT_PROP_TOUCH_T17		"obj_t17"	/* linear_tbl_cfg */
#define DT_PROP_TOUCH_T18		"obj_t18"	/* comms_config_cfg */
#define DT_PROP_TOUCH_T19		"obj_t19"	/* gpio_pwm_cfg */
#define DT_PROP_TOUCH_T20		"obj_t20"	/* grip_suppression_cfg */
#define DT_PROP_TOUCH_T22		"obj_t22"	/* noise_suppression_cfg */
#define DT_PROP_TOUCH_T23		"obj_t23"	/* touch_proximity_cfg */
#define DT_PROP_TOUCH_T24		"obj_t24"	/* one_touch_gesture_proc_cfg */
#define DT_PROP_TOUCH_T25		"obj_t25"	/* self_test_cfg */
#define DT_PROP_TOUCH_T27		"obj_t27"	/* two_touch_gesture_proc_cfg */
#define DT_PROP_TOUCH_T28		"obj_t28"	/* cte_config_cfg */
#define DT_PROP_TOUCH_T36		"obj_t36"	/* noise1_suppression_cfg */

/* LM3559 Node */
#define DT_PATH_LM3559			"/System@0/I2C@0/LEDLM3559@0"

/* LM3554 Node */
#define DT_PATH_LM3554			"/System@0/I2C@0/LEDLM3554@0"

/* BD7885 Node */
#define DT_PATH_BD7885			"/System@0/I2C@0/XENONBD7885@0"

/* I2C Node */
#define DT_PATH_I2C           "/System@0/I2C@0"

/* GPIO Node */
#define DT_PATH_GPIO        "/System@0/GPIO@0"
#define DT_PROP_GPIO_MAP    "signalmap"

/* CPCAP Node */
#define DT_PATH_CPCAP			"/System@0/SPI@0/PowerIC@0"
#define DT_PROP_CPCAP_BUSNUM	"bus_num"
#define DT_PROP_CPCAP_SPIINIT	"spiinit"
#define DT_PROP_CPCAP_VIO_SUPPLY_CONVERTER "vio_supply_converter"
#define DT_PROP_CPCAP_RGTINIT	"regulator_init"
#define DT_PROP_CPCAP_RGTMODE	"regulator_mode"
#define DT_PROP_CPCAP_RGTOFFMODE "regulator_off_mode"

/* Display panel Node */
#define DT_PATH_DISPLAY1	"/System@0/Display@0"
#define DT_PATH_DISPLAY2	"/System@0/Display@1"

/* Lighting Node */
#define DT_BACKLIGHT		"/System@0/DisplayBacklight@0"
#define DT_PROP_TABLET_LCD	"tablet_lcd"
#define DT_PROP_RUTH_LCD	"ruth_lcd"

/* HallEffect Node */
#define DT_HALLEFFECT_DOCK		"/System@0/GPIO@0/HallEffect@0"
#define DT_PROP_DEV_AVAILABLE	"device_available"
#define DT_PROP_DEV_NORTH_IS_DESK	"north_is_desk"


#define DT_HOME_LED		"/System@0/ButtonBacklight@0"
#define DT_PROP_TABLET_BUTTON	"tablet_button_led"
#define DT_PROP_RUTH_BUTTON	"ruth_button_led"

#define DT_KPAD_LED		"/System@0/KeypadBacklight@0"
#define DT_PROP_TABLET_KPAD_LED	"tablet_kpad_led"
#define DT_PROP_RUTH_KPAD_LED	"ruth_kpad_led"
#define DT_PROP_ADP5588_KPAD_LED "adp5588_kpad_led"

#define DT_NOTIFICATION_LED	"/System@0/NotificationLED@0"
#define DT_PROP_TABLET_RGB_LED	"tablet_rgb_led"
#define DT_PROP_RUTH_RGB_LED	"ruth_rgb_led"

#define DT_PATH_PROX                    "/System@0/I2C@0/Proximity@0"
#define DT_PROP_ISL29030_CONF           "configure"
#define DT_PROP_ISL29030_INT_CNTL       "interrupt_cntrl"
#define DT_PROP_ISL29030_PROX_LOW_TH    "prox_lower_threshold"
#define DT_PROP_ISL29030_PROX_HIGH_TH   "prox_higher_threshold"
#define DT_PROP_ISL29030_XTALK_V_COV_TH "crosstalk_vs_covered_threshold"
#define DT_PROP_ISL29030_DEF_PROX_NOISE "default_prox_noise_floor"
#define DT_PROP_ISL29030_NUM_SAMP_NOISE "num_samples_for_noise_floor"
#define DT_PROP_ISL29030_LENS_PERCENT   "lens_percent_t"
#define DT_PROP_ISL29030_REGULATOR      "regulator"

#define DT_LCD_BACKLIGHT	"/System@0/I2C@0/LCDBacklight@0"
#define DT_PROP_POWERUP_GEN_CNFG	"power_up_gen_config"
#define DT_PROP_GEN_CNFG	"gen_config"
#define DT_PROP_ALS_CNFG	"als_config"
#define DT_PROP_BRIGHTNESS_RAMP		"brightness_ramp"
#define DT_PROP_ALS_ZONE_INFO		"als_zone_info"
#define DT_PROP_ALS_RESISTOR_SEL    "als_resistor_sel"
#define DT_PROP_BRIGHTNESS_CTRL		"brightness_control"
#define DT_PROP_ZB0		"zone_boundary_0"
#define DT_PROP_ZB1		"zone_boundary_1"
#define DT_PROP_ZB2		"zone_boundary_2"
#define DT_PROP_ZB3		"zone_boundary_3"
#define DT_PROP_ZT0		"zone_target_0"
#define DT_PROP_ZT1		"zone_target_1"
#define DT_PROP_ZT2		"zone_target_2"
#define DT_PROP_ZT3		"zone_target_3"
#define DT_PROP_ZT4		"zone_target_4"
#define DT_PROP_MANUAL_CURRENT		"manual_current"
#define DT_PROP_UPPER_CURR_SEL		"upper_curr_sel"
#define DT_PROP_LOWER_CURR_SEL		"lower_curr_sel"
#define DT_PROP_LENS_LOSS_COEFF		"lens_loss_coeff"
#define DT_PROP_MANUAL_ALS_CONFIG	"manual_als_config"
#define DT_PROP_ALS_ENABLED		"als_enabled"

/* Video out Node */
#define DT_PATH_VIDEO_OUT	"/System@0/VideoOut@0"

/* Accelerometer Node */
#define DT_PATH_ACCELEROMETER   "/System@0/I2C@0/Accelerometer@0"
#define DT_PROP_ACCELEROMETER_AXIS_MAP_X	"axis_map_x"
#define DT_PROP_ACCELEROMETER_AXIS_MAP_Y	"axis_map_y"
#define DT_PROP_ACCELEROMETER_AXIS_MAP_Z	"axis_map_z"
#define DT_PROP_ACCELEROMETER_NEGATE_X	"negate_x"
#define DT_PROP_ACCELEROMETER_NEGATE_Y	"negate_y"
#define DT_PROP_ACCELEROMETER_NEGATE_Z	"negate_z"
#define DT_PROP_ACCELEROMETER_SENS_LOW "sensitivity_low"
#define DT_PROP_ACCELEROMETER_SENS_MEDIUM "sensitivity_medium"
#define DT_PROP_ACCELEROMETER_SENS_HIGH "sensitivity_high"

/* MMC Node */
#define DT_PATH_MMC1		"/System@0/SDHC@0/SDHCSLOT@0"
#define DT_PATH_MMC2		"/System@0/SDHC@0/SDHCSLOT@1"
#define DT_PATH_MMC3		"/System@0/SDHC@0/SDHCSLOT@2"
#define DT_PROP_MMC_CARD_CONNECT	"card_connect"
#define DT_PROP_MMC_PWR_SUPPLY		"pwr_supply"
#define DT_PROP_MMC_HOST_CAPABILITY	"host_capability"
#define DT_PROP_MMC_CARD_CAPABILITY	"card_capability"
#define DT_PROP_MMC_CARD_DETECTION	"card_detection"

/* Feature Node */
#define DT_HIGH_LEVEL_FEATURE	"/System@0/Feature@0"
#define DT_HIGH_LEVEL_FEATURE_HEADSET_UART_EN "feature_headset_uart_en"
#define DT_HIGH_LEVEL_FEATURE_NO_NAND "feature_no_nand"

/* Sim Card Node */
#define DT_PATH_SIM_DEV	"/System@0/SimDevice@0"
#define DT_PROP_SIM_DEV_AVAILABILITY "sim_availability"

/* UART Node */
#define DT_PATH_UART	"/System@0/UART@0"
#define DT_PROP_UART_HW_FLOW_CONTROL	"hw_flow_control"
#define DT_PROP_UART_PORT_FOR_GPS		"gps_port"
#define DT_PROP_UART_PADCONF_FOR_UART0    "uart0_padconf"

/* Audio Node */
#define DT_PATH_AUDIO	"/System@0/Audio@0"
#define DT_PROP_AUDIO_ANALOG_DOWNLINK	"analog_downlink"
#define DT_PROP_AUDIO_STEREO_LOUDSPEAKER "stereo_loudspeaker"
#define DT_PROP_AUDIO_MIC3 "mic3"
#define DT_PROP_AUDIO_I2S_BP "i2s_bp"

/* Modem Node */
#define DT_PATH_MODEM	"/System@0/Modem@0"
#define DT_PROP_MODEM_TYPE	"type"

#endif
#endif
