/*
 * arch/arm/mach-omap2/board-mapphone-modem.c
 *
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/gpio_mapping.h>
#include <linux/mdm_ctrl.h>

#ifdef CONFIG_ARM_OF
#include <mach/dt_path.h>
#include <asm/prom.h>
#endif

#define MAPPHONE_AP_TO_BP_FLASH_EN_GPIO	157
#define MAPPHONE_AP_TO_BP_PSHOLD_GPIO	138
#define MAPPHONE_BP_RESOUT_GPIO		139
#define MAPPHONE_BP_PWRON_GPIO		137
#define MAPPHONE_BP_READY_AP_GPIO	141
#define MAPPHONE_BP_READY2_AP_GPIO	59
#define MAPPHONE_BP_READY3_AP_GPIO	14
#define MAPPHONE_AP_READY_BP_GPIO	167
#define MAPPHONE_AP_READY2_BP_GPIO	110
#define MAPPHONE_AP_READY3_BP_GPIO	42
#define MAPPHONE_AP_RESET_BP_GPIO	128
#define MAPPHONE_AP_WAKE_BP_GPIO	143
#define MAPPHONE_BP_WAKE_AP_GPIO	29

#define MAPPHONE_BP_QSC6085	0x001E0000
#define MAPPHONE_BP_MDM6600	0x001E0001

/* BP boot mode */
#define BP_BOOT_INVALID			0
#define BP_BOOT_NORMAL_MODE		1
#define BP_BOOT_FLASH_MODE		2
#define BP_BOOT_UNKNOWN_MODE	3

struct mapphone_mdm_gpios {
	int ap_to_bp_flash_en;
	int ap_to_bp_pshold;
	int bp_pwron;
	int bp_resout;
	int bp_ready_ap;
	int bp_ready2_ap;
	int bp_ready3_ap;	/* MDM6600 */
	int ap_ready_bp;	/* MDM6600 */
	int ap_ready2_bp;	/* MDM6600 */
	int ap_ready3_bp;	/* MDM6600 */
	int ap_reset_bp;	/* MDM6600 */
	int ap_wake_bp; 	/* MDM6600 */
	int bp_wake_ap; 	/* MDM6600 */
};

struct mapphone_modem {
	int type;
	struct mapphone_mdm_gpios gpio;
};

static struct mapphone_modem mapphone_mdm0;
static struct mdm_ctrl_platform_data mdm_ctrl_platform_data;

static struct platform_device mdm_ctrl_platform_device = {
	.name = MDM_CTRL_MODULE_NAME,
	.id = -1,
	.dev = {
		.platform_data = &mdm_ctrl_platform_data,
	},
};

/* QSC6085 Control Functions */
static int qsc6085_mdm_ctrl_power_off(void)
{
	int i;
	int pd_return = 0;

	/* Check to see if the modem is already powered down */
	if (!gpio_get_value(mapphone_mdm0.gpio.bp_resout)) {
		pr_info("%s: Modem already powered down.\n", __func__);
		pd_return = 1;
		return pd_return;
	}

	/* Disable ability to notify clients of bp reset activity */
	disable_irq(gpio_to_irq(mapphone_mdm0.gpio.bp_resout));

	pr_info("%s: Initiate modem power down...\n", __func__);
	/* Press modem Power Button */
	gpio_set_value(mapphone_mdm0.gpio.bp_pwron, 1);
	mdelay(100);
	gpio_set_value(mapphone_mdm0.gpio.bp_pwron, 0);
	/* Wait up to 5 seconds for the modem to properly power down */
	for (i = 0; i < 10; i++) {
		if (!gpio_get_value(mapphone_mdm0.gpio.bp_resout)) {
			pr_info("%s: Modem power down success.\n", __func__);
			pd_return = 2;
			break;
		} else
			mdelay(500);
	}

	if (!pd_return) {
		/* Pull power from the modem */
		pr_info("%s: Modem pd failure.  Pull power.\n", __func__);
		gpio_set_value(mapphone_mdm0.gpio.ap_to_bp_pshold, 1);
		mdelay(5);
		gpio_set_value(mapphone_mdm0.gpio.ap_to_bp_pshold, 0);
	}

	return pd_return;
}

static int qsc6085_mdm_ctrl_power_on(int bp_mode)
{
	int pu_result = MDM_CTRL_BP_MODE_INVALID;
	int status, status_prev = 0;
	int i;
	int pu_success = 0;

	pr_info("%s: powering up modem...bp_mode = %d...\n",
		__func__, bp_mode);

	if (bp_mode != MDM_CTRL_BP_MODE_NORMAL &&
	    bp_mode != MDM_CTRL_BP_MODE_FLASH)
		return pu_result;

	/* Make sure the modem isn't already powered on */
	if (gpio_get_value(mapphone_mdm0.gpio.bp_resout) == 1) {
		pr_info("%s: modem already on, exiting\n", __func__);
		return pu_result;
	}

	/* Modem is off, setup boot mode (flash or normal) */
	if (bp_mode == MDM_CTRL_BP_MODE_NORMAL) {
		pr_info("%s: normal mode requested\n", __func__);
		gpio_set_value(mapphone_mdm0.gpio.ap_to_bp_flash_en, 0);
	} else if (bp_mode == MDM_CTRL_BP_MODE_FLASH) {
		pr_info("%s: flash mode requested\n", __func__);
		gpio_set_value(mapphone_mdm0.gpio.ap_to_bp_flash_en, 1);
	}

	/* Enable the bp power supply */
	gpio_set_value(mapphone_mdm0.gpio.ap_to_bp_pshold, 0);
	msleep(100);

	/* Press the bp "power button" */
	gpio_set_value(mapphone_mdm0.gpio.bp_pwron, 1);
	msleep(200);
	gpio_set_value(mapphone_mdm0.gpio.bp_pwron, 0);

	/* Wait up to 5 seconds for the BP to be powered up */
	for (i = 0; i < 10; i++) {
		status = gpio_get_value(mapphone_mdm0.gpio.bp_resout);
		if (status == 1 && status_prev != 1) {  /* debounce */
			status_prev = status;
			mdelay(5);
			continue;
		}

		if (status == 1 && status_prev == 1) {
			pr_info("%s: modem power up complete\n",
				__func__);
			pu_success = 1;
			break;
		}
		pr_info("%s: waiting for modem to power on...\n",
				__func__);
		msleep(500);
		status_prev = status = 0;
	}

	if (!pu_success) {
		pr_info("%s: power up failure. Modem is off.\n", __func__);
		return 0;
	}

	/* Attempt to verify power up mode.  This isn't 100% accurate as
	   BP_READY_AP is owned by uart and BP_READY2_AP is owned by usb.
	   So their status during boot mode detection is not 100% reliable. */
	for (i = 0; i < 20; i++) {
		if (gpio_get_value(mapphone_mdm0.gpio.bp_ready_ap)) {
			/* Confirm still in normal mode */
			if (pu_result == MDM_CTRL_BP_MODE_NORMAL) {
				pr_info("%s: boot mode normal confirmed\n",
					__func__);
				break;
			}
			pr_info("%s: boot mode normal detected, verifying...\n",
				__func__);
			pu_result = MDM_CTRL_BP_MODE_NORMAL;
		} else if (gpio_get_value(mapphone_mdm0.gpio.bp_ready2_ap)) {
			/* Confirm still in flash mode */
			if (pu_result == MDM_CTRL_BP_MODE_FLASH) {
				pr_info("%s: boot mode flash confirmed\n",
					__func__);
				break;
			}
			pr_info("%s: boot mode flash detected, verifying....\n",
				__func__);
			pu_result = MDM_CTRL_BP_MODE_FLASH;
		}

		msleep(100);
	}

	if (pu_result == bp_mode) {
		pr_info("%s: boot mode[%d] verified!\n", __func__, pu_result);
		/* Add some wait time for devices to enumerate */
		msleep(2000);
	} else if (pu_result == MDM_CTRL_BP_MODE_INVALID) {
		pr_info("%s: boot mode[%d], unconfirmed\n", __func__, bp_mode);
		pu_result = MDM_CTRL_BP_MODE_UNKNOWN;
	}

	return pu_result;
}

static unsigned int qsc6085_get_bp_status(void)
{
	unsigned int status = 0;

    /* Not 100% accurate, but this is the best that can be done */
	if (gpio_get_value(mapphone_mdm0.gpio.bp_resout)) {
		if (gpio_get_value(mapphone_mdm0.gpio.bp_ready2_ap)) {
			pr_info("%s Mode is in flash state", __func__);
			status = MDM_CTRL_BP_STATUS_RDL;
		} else if (gpio_get_value(mapphone_mdm0.gpio.bp_ready_ap)) {
			pr_info("%s Mode is in normal state", __func__);
			status = MDM_CTRL_BP_STATUS_AWAKE;
		}
	} else {
		pr_info("%s Mode is in power off state", __func__);
		status = MDM_CTRL_BP_STATUS_UNKNOWN;
	}

	return status;
}

/* MDM6600 Control Functions */
static void mdm6600_set_ap_status(unsigned int status)
{
	gpio_set_value(mapphone_mdm0.gpio.ap_ready_bp, (status & 0x1));
	gpio_set_value(mapphone_mdm0.gpio.ap_ready2_bp, (status >> 1) & 0x1);
	gpio_set_value(mapphone_mdm0.gpio.ap_ready3_bp, (status >> 2) & 0x1);
}

static unsigned int mdm6600_get_bp_status(void)
{
	unsigned int status = 0;
	unsigned int bp_status[3];

	bp_status[0] = gpio_get_value(mapphone_mdm0.gpio.bp_ready_ap);
	bp_status[1] = gpio_get_value(mapphone_mdm0.gpio.bp_ready2_ap);
	bp_status[2] = gpio_get_value(mapphone_mdm0.gpio.bp_ready3_ap);

	status = ((bp_status[2] & 0x1) << 2) |
			((bp_status[1] & 0x1) << 1) |
			(bp_status[0] & 0x1);
	return status;
}

static void mdm6600_set_two_wakes(int bp_mode)
{
	int value = 0;

	if (bp_mode == MDM_CTRL_BP_MODE_FLASH)
		value = 3;

	gpio_direction_output(
		mapphone_mdm0.gpio.bp_wake_ap, (value & 0x1));
	gpio_direction_output(
		mapphone_mdm0.gpio.ap_wake_bp, ((value >> 1) & 0x1));
}

static int mdm6600_mdm_ctrl_power_off(void)
{
	unsigned int bp_status;
	int i;
	int pd_return = 0;

	pr_info("%s: Initiate BP modem power down...\n", __func__);

	if (!gpio_get_value(mapphone_mdm0.gpio.bp_resout)) {
		pr_info("%s: Modem already powered down.\n", __func__);
		pd_return = 1;
		return pd_return;
	}

	/* Request BP shutdown through AP status lines */
	mdm6600_set_ap_status(MDM_CTRL_AP_STATUS_BP_SHUTDOWN);
	mdelay(100);
	pr_info("%s: AP status set to shutdown req...\n", __func__);

	/* Keep BP_PWRON asserted until BP shutdown ack */
	gpio_set_value(mapphone_mdm0.gpio.bp_pwron, 1);
	pr_info("%s: BP requested to shutdown, Waiting for Ack...\n", __func__);

	/* Wait up to 5 seconds for BP to properly power down */
	for (i = 0; i < 10; i++) {
		bp_status = mdm6600_get_bp_status();

		pr_info("%s: BP status is: 0x%x \n", __func__,  bp_status);
		if (bp_status == MDM_CTRL_BP_STATUS_SHUTDOWN_ACK) {
			pr_info("%s: BP power down success.\n", __func__);
			pd_return = 2;
			break;
		} else
			mdelay(500);   /* wait 0.5 sec */
	}

	gpio_set_value(mapphone_mdm0.gpio.bp_pwron, 0);

	if (!pd_return) {
		/* BP failed to power down, pull out the battery*/
		pr_info("%s: BP PWR DN failure,pull Battery. \n", __func__);
		/* TODO Forcefully pull power from MDM6600 */
		/* Assert AP_RESET_BP high */
		gpio_set_value(mapphone_mdm0.gpio.ap_reset_bp, 1);
		mdelay(7000);
		gpio_set_value(mapphone_mdm0.gpio.ap_reset_bp, 0);
	}

	return pd_return;
}

static int mdm6600_mdm_ctrl_power_on(int bp_mode)
{
	int i;
	int bp_status = 0;
	int bp_status_prev = -1;
	int pu_result = MDM_CTRL_BP_MODE_INVALID;

	pr_info("%s: powering up modem...bp_mode = %d...\n",
		__func__, bp_mode);

	mdm6600_set_two_wakes(bp_mode);
	msleep(200);

	/* Request BP startup */
	mdm6600_set_ap_status(MDM_CTRL_AP_STATUS_NO_BYPASS);

	gpio_set_value(mapphone_mdm0.gpio.ap_reset_bp, 0);

	gpio_set_value(mapphone_mdm0.gpio.bp_pwron, 1);
	msleep(500);
	gpio_set_value(mapphone_mdm0.gpio.bp_pwron, 0);

	/* Wait for the modem to acknowledge powerup */
	for (i = 0; i < 100; i++) {
		bp_status = mdm6600_get_bp_status();
		if (bp_status != bp_status_prev) {  /* debounce */
			bp_status_prev = bp_status;
			mdelay(5);
			continue;
		}

		if (bp_status == MDM_CTRL_BP_STATUS_AWAKE) {
			pr_info("%s: modem power up complete in normal mode\n",
				__func__);
			pu_result = MDM_CTRL_BP_MODE_NORMAL;
			break;
		} else if (bp_status == MDM_CTRL_BP_STATUS_RDL) {
			pr_info("%s: modem power up complete in flash mode\n",
				__func__);
			pu_result = MDM_CTRL_BP_MODE_FLASH;
			break;
		} else
			msleep(50);
	}

	if (pu_result == MDM_CTRL_BP_MODE_INVALID) {
		pu_result = MDM_CTRL_BP_MODE_UNKNOWN;
		pr_info("%s: modem did not acknowledge powerup: status=%d\n",
		__func__, bp_status);
	}
	return pu_result;

}

static int mdm6600_mdm_ctrl_bp_reset(int bp_mode)
{
	int bp_status = 0;
	int result = 0;

	pr_info("%s: bp reset...bp_mode = %d\n", __func__, bp_mode);

	mdm6600_set_two_wakes(bp_mode);

	gpio_set_value(mapphone_mdm0.gpio.ap_reset_bp, 1);

	/* BP will power down in ~(3 + .25)secs after resetting */
	msleep(4000);

	bp_status = mdm6600_get_bp_status();

	if (bp_status == MDM_CTRL_BP_STATUS_AWAKE) {
		pr_info("%s: bp reset to power up bp_status = %d...\n",
			__func__, bp_status);
		result = 1;
	} else
		pr_info("%s: bp reset to power down bp_status = %d...\n",
		__func__, bp_status);

	return result;
}

int mapphone_bp_get_type(void)
{
	int ret = 0;
#ifdef CONFIG_ARM_OF
	struct device_node *node;
	const void *prop;
	int size;
	node = of_find_node_by_path(DT_PATH_MODEM);
	if (node) {
		prop = of_get_property(node, \
			DT_PROP_MODEM_TYPE, &size);
		if (prop && size)
			ret = *(u32 *)prop;
		of_node_put(node);
	}
#endif /* CONFIG_ARM_OF */
	printk(KERN_INFO "Modem0 is 0x%08x\n", ret);
	return ret;
}

static int mapphone_qsc6085_init_gpio(void)
{
	mapphone_mdm0.gpio.ap_to_bp_flash_en =
		get_gpio_by_name("ipc_o_bp_flash_en");
	if (mapphone_mdm0.gpio.ap_to_bp_flash_en < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_bp_flash_en "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.ap_to_bp_flash_en =
			MAPPHONE_AP_TO_BP_FLASH_EN_GPIO;
	}

	mapphone_mdm0.gpio.ap_to_bp_pshold =
		get_gpio_by_name("ipc_o_bp_pshold");
	if (mapphone_mdm0.gpio.ap_to_bp_pshold < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_bp_pshold "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.ap_to_bp_pshold =
			MAPPHONE_AP_TO_BP_PSHOLD_GPIO;
	}

	mapphone_mdm0.gpio.bp_pwron =
		get_gpio_by_name("ipc_o_bp_pwron");
	if (mapphone_mdm0.gpio.bp_pwron < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_bp_pwron "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.bp_pwron =
			MAPPHONE_BP_PWRON_GPIO;
	}

	mapphone_mdm0.gpio.bp_resout =
		get_gpio_by_name("ipc_i_bp_resout");
	if (mapphone_mdm0.gpio.bp_resout < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_i_bp_resout "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.bp_resout =
			MAPPHONE_BP_RESOUT_GPIO;
	}

	mapphone_mdm0.gpio.bp_ready_ap =
		get_gpio_by_name("ipc_i_bp_ready");
	if (mapphone_mdm0.gpio.bp_ready_ap < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_i_bp_ready "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.bp_ready_ap =
			MAPPHONE_BP_READY_AP_GPIO;
	}

	mapphone_mdm0.gpio.bp_ready2_ap =
		get_gpio_by_name("ipc_i_bp_ready2");
	if (mapphone_mdm0.gpio.bp_ready2_ap < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_i_bp_ready2 "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.bp_ready2_ap =
			MAPPHONE_BP_READY2_AP_GPIO;
	}

	gpio_request(mapphone_mdm0.gpio.bp_ready2_ap, "BP Flash Ready");
	gpio_direction_input(mapphone_mdm0.gpio.bp_ready2_ap);

	gpio_request(mapphone_mdm0.gpio.bp_resout, "BP Reset Output");
	gpio_direction_input(mapphone_mdm0.gpio.bp_resout);

	gpio_request(mapphone_mdm0.gpio.bp_pwron, "BP Power On");
	gpio_direction_output(mapphone_mdm0.gpio.bp_pwron, 0);

	gpio_request(mapphone_mdm0.gpio.ap_to_bp_pshold, "AP to BP PS Hold");
	gpio_direction_output(mapphone_mdm0.gpio.ap_to_bp_pshold, 0);

	mdm_ctrl_platform_data.ap_to_bp_flash_en_gpio =
		mapphone_mdm0.gpio.ap_to_bp_flash_en;
	mdm_ctrl_platform_data.bp_pwron_gpio =
		mapphone_mdm0.gpio.bp_pwron;
	mdm_ctrl_platform_data.ap_to_bp_pshold_gpio =
		mapphone_mdm0.gpio.ap_to_bp_pshold;
	mdm_ctrl_platform_data.bp_resout_gpio =
		mapphone_mdm0.gpio.bp_resout;
	mdm_ctrl_platform_data.bp_ready_ap_gpio =
		mapphone_mdm0.gpio.bp_ready_ap;
	mdm_ctrl_platform_data.bp_ready2_ap_gpio =
		mapphone_mdm0.gpio.bp_ready2_ap;
	return 0;
}

static int mapphone_mdm6600_init_gpio(void)
{
	/* All QSC6085 GPIOs are also used by MDM6600 */
	mapphone_qsc6085_init_gpio();

	mapphone_mdm0.gpio.bp_ready3_ap =
		get_gpio_by_name("ipc_i_bp_ready3");
	if (mapphone_mdm0.gpio.bp_ready3_ap < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_i_bp_ready3 "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.bp_ready3_ap =
			MAPPHONE_BP_READY3_AP_GPIO;
	}

	mapphone_mdm0.gpio.ap_ready_bp =
		get_gpio_by_name("ipc_o_ap_ready");
	if (mapphone_mdm0.gpio.ap_ready_bp < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_ap_ready "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.ap_ready_bp =
			MAPPHONE_AP_READY_BP_GPIO;
	}

	mapphone_mdm0.gpio.ap_ready2_bp =
		get_gpio_by_name("ipc_o_ap_ready2");
	if (mapphone_mdm0.gpio.ap_ready2_bp < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_ap_ready2 "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.ap_ready2_bp =
			MAPPHONE_AP_READY2_BP_GPIO;
	}

	mapphone_mdm0.gpio.ap_ready3_bp =
		get_gpio_by_name("ipc_o_ap_ready3");
	if (mapphone_mdm0.gpio.ap_ready3_bp < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_ap_ready3 "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.ap_ready3_bp =
			MAPPHONE_AP_READY3_BP_GPIO;
	}

	mapphone_mdm0.gpio.ap_reset_bp =
		get_gpio_by_name("ipc_o_ap_reset_bp");
	if (mapphone_mdm0.gpio.ap_reset_bp < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_o_ap_reset_bp "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.ap_reset_bp =
			MAPPHONE_AP_RESET_BP_GPIO;
	}

	mapphone_mdm0.gpio.ap_wake_bp =
		get_gpio_by_name("ipc_bpwake_strobe");
	if (mapphone_mdm0.gpio.ap_wake_bp < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_bpwake_strobe "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.ap_wake_bp =
			MAPPHONE_AP_WAKE_BP_GPIO;
	}

	mapphone_mdm0.gpio.bp_wake_ap =
		get_gpio_by_name("ipc_apwake_trigger");
	if (mapphone_mdm0.gpio.bp_wake_ap < 0) {
		printk(KERN_DEBUG "%s: can't get ipc_apwake_trigger "
				"from device_tree\n", __func__);
		mapphone_mdm0.gpio.bp_wake_ap =
			MAPPHONE_BP_WAKE_AP_GPIO;
	}

	gpio_request(mapphone_mdm0.gpio.ap_reset_bp, "AP Reset BP");
	gpio_direction_output(mapphone_mdm0.gpio.ap_reset_bp, 0);

	gpio_request(mapphone_mdm0.gpio.bp_wake_ap, "BP AWAKE AP");

	gpio_request(mapphone_mdm0.gpio.ap_wake_bp, "AP AWAKE BP");

	return 0;
}

static int mapphone_mdm6600_mdm_ctrl_init(void)
{
	mapphone_mdm6600_init_gpio();
	mdm_ctrl_platform_data.power_off = mdm6600_mdm_ctrl_power_off;
	mdm_ctrl_platform_data.power_on = mdm6600_mdm_ctrl_power_on;
	mdm_ctrl_platform_data.reset = mdm6600_mdm_ctrl_bp_reset;
	mdm_ctrl_platform_data.set_ap_status = mdm6600_set_ap_status;
	mdm_ctrl_platform_data.get_bp_status = mdm6600_get_bp_status;
	return 0;
}

static int mapphone_qsc6085_mdm_ctrl_init(void)
{
	mapphone_qsc6085_init_gpio();
	mdm_ctrl_platform_data.power_off = qsc6085_mdm_ctrl_power_off;
	mdm_ctrl_platform_data.power_on = qsc6085_mdm_ctrl_power_on;
	mdm_ctrl_platform_data.reset = NULL;
	mdm_ctrl_platform_data.set_ap_status = NULL;
	mdm_ctrl_platform_data.get_bp_status = qsc6085_get_bp_status;
	return 0;
}

int __init mapphone_mdm_ctrl_init(void)
{
	int ret = 0;
	mapphone_mdm0.type = mapphone_bp_get_type();
	if (!mapphone_mdm0.type)
		return -ENODEV;

	switch (mapphone_mdm0.type) {
	case MAPPHONE_BP_MDM6600:
		ret = mapphone_mdm6600_mdm_ctrl_init();
		break;
	case MAPPHONE_BP_QSC6085:
		ret = mapphone_qsc6085_mdm_ctrl_init();
		break;
	default:
		ret = -ENODEV;
		break;
	}
	if (!ret)
		ret = platform_device_register(&mdm_ctrl_platform_device);
	return ret;
}
