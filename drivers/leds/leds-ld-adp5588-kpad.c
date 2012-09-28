/*
 * Copyright (C) 2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/i2c/adp5588.h>
#include <linux/gpio.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#ifdef CONFIG_MOT_KEYBOARD_ADP5588
#include <linux/adp5588_keypad.h>
#endif

#define ADP5588_GPIO_LED1 230
#define ADP5588_GPIO_LED2 231

static void ld_adp5588_kpad_store(struct led_classdev *led_cdev,
			enum led_brightness brightness);
static int ld_adp5588_kpad_register(struct device *dev);
static void ld_adp5588_kpad_unregister(void);
static int ld_adp5588_kpad_suspend(struct platform_device *dev,
				pm_message_t state);
static int ld_adp5588_kpad_resume(struct platform_device *dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ld_adp5588_kpad_early_suspend(struct early_suspend *h);
static void ld_adp5588_kpad_late_resume(struct early_suspend *h);
#endif


/* Stored state of the ADP5588 keypad backlight leds:
   bit 0: Segment 1 (EL_EN1) state (1 - ON, 0 - OFF)
   bit 1: Segment 2 (EL_EN2) state (1 - ON, 0 - OFF)
*/
static int ld_adp5588_mask;

static int ld_adp5588_suspend_transition;

static struct led_classdev ld_adp5588_keypad_class_dev[] = {
	{
		.name           = "keyboard-backlight",
		.brightness_set = ld_adp5588_kpad_store
	},
	{
		.name           = "keyboard1-backlight",
		.brightness_set = ld_adp5588_kpad_store
	},
	{
		.name           = "keyboard2-backlight",
		.brightness_set = ld_adp5588_kpad_store
	}
};

#define NUM_LED_REGIONS (sizeof(ld_adp5588_keypad_class_dev) \
			/ sizeof(ld_adp5588_keypad_class_dev[0]))

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend_data = {
    .level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
    .suspend = ld_adp5588_kpad_early_suspend,
    .resume = ld_adp5588_kpad_late_resume,
};
#endif

static int leds_mask;

/* SET backlight drive register. Bit0=EL_EN1, Bit1=EL_EN2 */
static int adp5588_set_backlight(uint8_t mask)
{
	gpio_set_value(ADP5588_GPIO_LED1, mask & 0x01);
	gpio_set_value(ADP5588_GPIO_LED2, mask & 0x02);

	leds_mask = mask;
	return 0;
}

/* GET backlight drive register value. Bit0=EL_EN1, Bit1=EL_EN2 */
static int adp5588_get_backlight(void)
{
	return leds_mask;
}

/* This function is called by led_update_brightness() in leds class driver
   upon a write onto /sys/class/leds/keyboard-backlight
   to turn on/off both keypad backlight segments (EL_EN1 and EL_EN2), or upon
   a write onto /sys/class/leds/keyboard1-backlight or
   /sys/class/leds/keyboard2-backlight to turn on/off one of the keypad
   backlight segments individually (EL_EN1 or EL_EN2).
*/
static void ld_adp5588_kpad_store(struct led_classdev *led_cdev,
				enum led_brightness brightness)
{
	int ret;
	int mask;
	int segment;

	if (strstr(led_cdev->name, "1"))
		segment = 0x01;
	else {
		if (strstr(led_cdev->name, "2"))
			segment = 0x02;
		else
			segment = 0x03;
	}

	mask = adp5588_get_backlight();

	if (mask < 0) {
		printk(KERN_ERR
			"adp5588: %s: adp5588_get_backlight() failed: %d\n",
			__func__, mask);
		return;
	}

	if (brightness == LED_OFF)
		mask &= ~segment;
	else
		mask |= segment;

	ret = adp5588_set_backlight(mask);

	if (ret < 0) {
		printk(KERN_ERR
			"adp5588: %s: adp5588_set_backlight() failed: %d\n",
			__func__, ret);
		return;
	}

	if (!ld_adp5588_suspend_transition) {
		if (segment == 0x03) {
			ld_adp5588_keypad_class_dev[1].brightness = brightness;
			ld_adp5588_keypad_class_dev[2].brightness = brightness;
		}
	}
}


static int ld_adp5588_kpad_register(struct device *dev)
{
	int i, ret;

	for (i = 0; i < NUM_LED_REGIONS; i++) {
		ret = led_classdev_register(dev,
			&ld_adp5588_keypad_class_dev[i]);

		if (ret) {
			printk(KERN_ERR "%s: unable to register %s LED: %d\n",
				__func__, ld_adp5588_keypad_class_dev[i].name,
				ret);

			ld_adp5588_kpad_unregister();

			return ret;
		}
	}

	return 0;
}


static void ld_adp5588_kpad_unregister(void)
{
	int i;

	for (i = 0; i < NUM_LED_REGIONS; i++)
		led_classdev_unregister(&ld_adp5588_keypad_class_dev[i]);
}


static int ld_adp5588_kpad_remove(struct platform_device *pdev)
{
	ld_adp5588_kpad_unregister();
	return 0;
}


static int ld_adp5588_kpad_probe(struct platform_device *pdev)
{
	int ret;
	struct adp5588_leds_platform_data *pdata = pdev->dev.platform_data;

	if (!pdata->use_leds) {
		printk(KERN_ERR	"ADP5588 keyboard led device "
			" declared unavailable\n");
		return -ENODEV;
	}

	ld_adp5588_kpad_register(&pdev->dev);

	ret = gpio_request(ADP5588_GPIO_LED1, "adp5588-led1");
	ret |= gpio_request(ADP5588_GPIO_LED2, "adp5588-led2");

	if (ret) {
		printk(KERN_ERR "ADP5588 gpio_request ret = %d\n", ret);
		return -ENODEV;
	}

	gpio_direction_output(ADP5588_GPIO_LED1, 0);
	gpio_direction_output(ADP5588_GPIO_LED2, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&early_suspend_data);
#endif

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ld_adp5588_kpad_early_suspend(struct early_suspend *h)
{
	ld_adp5588_kpad_suspend(NULL, PMSG_SUSPEND);
}

static void ld_adp5588_kpad_late_resume(struct early_suspend *h)
{
	ld_adp5588_kpad_resume(NULL);
}
#endif

static int ld_adp5588_kpad_suspend(struct platform_device *pdev,
				pm_message_t state)
{
	ld_adp5588_mask = adp5588_get_backlight();

	ld_adp5588_suspend_transition = 1;

	led_classdev_suspend(&ld_adp5588_keypad_class_dev[0]);
	led_classdev_suspend(&ld_adp5588_keypad_class_dev[1]);
	led_classdev_suspend(&ld_adp5588_keypad_class_dev[2]);

	switch (ld_adp5588_mask) {
	case 0x00:
		ld_adp5588_keypad_class_dev[0].brightness = 0;
		ld_adp5588_keypad_class_dev[1].brightness = 0;
		ld_adp5588_keypad_class_dev[2].brightness = 0;
		break;
	case 0x01:
		ld_adp5588_keypad_class_dev[0].brightness = 0;
		ld_adp5588_keypad_class_dev[1].brightness = 1;
		ld_adp5588_keypad_class_dev[2].brightness = 0;
		break;
	case 0x02:
		ld_adp5588_keypad_class_dev[0].brightness = 0;
		ld_adp5588_keypad_class_dev[1].brightness = 0;
		ld_adp5588_keypad_class_dev[2].brightness = 1;
		break;
	case 0x03:
		ld_adp5588_keypad_class_dev[0].brightness = 1;
		ld_adp5588_keypad_class_dev[1].brightness = 1;
		ld_adp5588_keypad_class_dev[2].brightness = 1;
		break;
	default:
		printk(KERN_ERR "%s: adp5588_get_backlight() failed: %d\n",
			__func__, ld_adp5588_mask);
	}

	ld_adp5588_suspend_transition = 0;

	return 0;
}

static int ld_adp5588_kpad_resume(struct platform_device *pdev)
{
	ld_adp5588_suspend_transition = 1;

	if (ld_adp5588_keypad_class_dev[1].brightness !=
		ld_adp5588_keypad_class_dev[2].brightness) {
		/* Resume dev 0 without restoring its original brightness */
		ld_adp5588_keypad_class_dev[0].flags &= ~LED_SUSPENDED;

		/* Let dev 1 and 2 to restore their original brightness */
		led_classdev_resume(&ld_adp5588_keypad_class_dev[1]);
		led_classdev_resume(&ld_adp5588_keypad_class_dev[2]);
	} else {
		led_classdev_resume(&ld_adp5588_keypad_class_dev[2]);
		led_classdev_resume(&ld_adp5588_keypad_class_dev[1]);
		led_classdev_resume(&ld_adp5588_keypad_class_dev[0]);
	}

	ld_adp5588_suspend_transition = 0;

	return 0;
}


static struct platform_driver ld_adp5588_kpad_driver = {
	.probe		= ld_adp5588_kpad_probe,
	.remove		= ld_adp5588_kpad_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= ld_adp5588_kpad_suspend,
	.resume		= ld_adp5588_kpad_resume,
#endif
	.driver = {
		.name = ADP5588_BACKLIGHT_NAME,
		.owner = THIS_MODULE,
	},
};


static int __init ld_adp5588_kpad_init(void)
{
	int ret;

	ret = platform_driver_register(&ld_adp5588_kpad_driver);

	if (ret) {
		printk(KERN_ERR "%s: driver register failed: %d\n",
			__func__, ret);
		return ret;
	}

	ld_adp5588_mask = 0;
	ld_adp5588_suspend_transition = 0;

	return 0;
}


static void __exit ld_adp5588_kpad_exit(void)
{
	platform_driver_unregister(&ld_adp5588_kpad_driver);
}


module_init(ld_adp5588_kpad_init);
module_exit(ld_adp5588_kpad_exit);


MODULE_DESCRIPTION("ADP5588 Keypad Lighting driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
