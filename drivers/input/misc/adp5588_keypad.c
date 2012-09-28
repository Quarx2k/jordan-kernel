/*
 * Copyright (C) 2008-2010 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

/* Linux driver for ADP5588 keypad controller */

#undef DEBUG
#undef ADP5588_DEMO_BOARD_RINGER_SW
#undef ADP5588_DEMO_BOARD_RESET
#undef ADP5588_DEMO_BOARD_VREG

#include <linux/module.h>
#include <linux/device.h>
#include <linux/sysdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/hrtimer.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/sysfs.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio_event.h>
#include <linux/gpio.h>
#include <linux/gpio_mapping.h>
#include <linux/switch.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <linux/reboot.h>
#include <linux/syscalls.h>

#include <linux/adp5588_keypad.h>
#include <linux/regulator/consumer.h>

/******************************************************************************
*  Registers IDs
******************************************************************************/
#define ADP5588_REG_DEV_ID          0x00  /* Device ID */
#define ADP5588_REG_CONFIG          0x01  /* Config register */
#define ADP5588_REG_INT_STAT        0x02  /* Interrupt Status */
#define ADP5588_REG_KEY_LCK_EC_STAT 0x03  /* Key Lock and Event Counter */
#define ADP5588_REG_KEY_EVENTA      0x04  /* Key Event Register A */
#define ADP5588_REG_KEY_EVENTB      0x05  /* Key Event Register B */
#define ADP5588_REG_KEY_EVENTC      0x06  /* Key Event Register C */
#define ADP5588_REG_KEY_EVENTD      0x07  /* Key Event Register D */
#define ADP5588_REG_KEY_EVENTE      0x08  /* Key Event Register E */
#define ADP5588_REG_KEY_EVENTF      0x09  /* Key Event Register F */
#define ADP5588_REG_KEY_EVENTG      0x0A  /* Key Event Register G */
#define ADP5588_REG_KEY_EVENTH      0x0B  /* Key Event Register H */
#define ADP5588_REG_KEY_EVENTI      0x0C  /* Key Event Register I */
#define ADP5588_REG_KEY_EVENTJ      0x0D  /* Key Event Register J */
#define ADP5588_REG_KP_LCK_TMR      0x0E  /* Keypad Lock1 to Lock2 Timer */
#define ADP5588_REG_UNLOCK1         0x0F  /* Unlock Key1 */
#define ADP5588_REG_UNLOCK2         0x10  /* Unlock Key2 */
#define ADP5588_REG_GPIO_INT_STAT1  0x11  /* GPIO Interrupt Status */
#define ADP5588_REG_GPIO_INT_STAT2  0x12  /* GPIO Interrupt Status */
#define ADP5588_REG_GPIO_INT_STAT3  0x13  /* GPIO Interrupt Status */
#define ADP5588_REG_GPIO_DAT_STAT1  0x14  /* GPIO Data Status */
#define ADP5588_REG_GPIO_DAT_STAT2  0x15  /* GPIO Data Status */
#define ADP5588_REG_GPIO_DAT_STAT3  0x16  /* GPIO Data Status */
#define ADP5588_REG_GPIO_DAT_OUT1   0x17  /* GPIO DATA OUT */
#define ADP5588_REG_GPIO_DAT_OUT2   0x18  /* GPIO DATA OUT */
#define ADP5588_REG_GPIO_DAT_OUT3   0x19  /* GPIO DATA OUT */
#define ADP5588_REG_GPIO_INT_EN1    0x1A  /* GPIO Interrupt Enable */
#define ADP5588_REG_GPIO_INT_EN2    0x1B  /* GPIO Interrupt Enable */
#define ADP5588_REG_GPIO_INT_EN3    0x1C  /* GPIO Interrupt Enable */
#define ADP5588_REG_KP_GPIO1        0x1D  /* Keypad or GPIO Selection */
#define ADP5588_REG_KP_GPIO2        0x1E  /* Keypad or GPIO Selection */
#define ADP5588_REG_KP_GPIO3        0x1F  /* Keypad or GPIO Selection */
#define ADP5588_REG_GPI_EM1         0x20  /* GPI Event Mode 1 */
#define ADP5588_REG_GPI_EM2         0x21  /* GPI Event Mode 2 */
#define ADP5588_REG_GPI_EM3         0x22  /* GPI Event Mode 3 */
#define ADP5588_REG_GPIO_DIR1       0x23  /* GPIO Data Direction */
#define ADP5588_REG_GPIO_DIR2       0x24  /* GPIO Data Direction */
#define ADP5588_REG_GPIO_DIR3       0x25  /* GPIO Data Direction */
#define ADP5588_REG_GPIO_INT_LVL1   0x26  /* GPIO Edge/Level Detect */
#define ADP5588_REG_GPIO_INT_LVL2   0x27  /* GPIO Edge/Level Detect */
#define ADP5588_REG_GPIO_INT_LVL3   0x28  /* GPIO Edge/Level Detect */
#define ADP5588_REG_DEBOUNCE_DIS_1  0x29  /* Debounce Disable */
#define ADP5588_REG_DEBOUNCE_DIS_2  0x2A  /* Debounce Disable */
#define ADP5588_REG_DEBOUNCE_DIS_3  0x2B  /* Debounce Disable */
#define ADP5588_REG_GPIO_PULL1      0x2C  /* GPIO Pull Disable */
#define ADP5588_REG_GPIO_PULL2      0x2D  /* GPIO Pull Disable */
#define ADP5588_REG_GPIO_PULL3      0x2E  /* GPIO Pull Disable */
#define ADP5588_REG_CMP_CFG_STAT    0x30  /* Comparator Config and Status */
/* Registers 0x30 and up are not used (no light sensor) */

/******************************************************************************
*  Chip Configuration
******************************************************************************/
/* No auto incr|Overflow ON|INT re-assert|Key event INT */
#define ADP5588_CONFIG_DEVICE 0x31

/* Key lock Disable */
#define ADP5588_CONFIG_KEY_LOCK 0x00

/* Set EL_EN_1 and EL_EN_2 line to 0 by default */
#define ADP5588_CONFIG_GPIO_DAT_OUT3 0x00

/* Enable GPIO Interrupt for RINGER/SILENT KEY */
#define ADP5588_CONFIG_GPIO_INT2 0x80

/* Row0 to Row7 are KP Matrix */
#define ADP5588_CONFIG_KP_GPIO1 0xff

/* Row0 to Row7 are KP Matrix */
#define ADP5588_CONFIG_KP_GPIO1_M 0xfe

/* Row3 and Row5 only are KP Matrix */
#define ADP5588_CONFIG_KP_GPIO1_M_LOCK 0x28

/* Col0 to Col6 are KP Matrix | Col7  is GPIO */
#define ADP5588_CONFIG_KP_GPIO2 0x7f

/* Col0 and Col2 only are KP Matrix */
#define ADP5588_CONFIG_KP_GPIO2_M_LOCK 0x05

/* COl8 & Col9 are GPIOs */
#define ADP5588_CONFIG_KP_GPIO3 0x00

/* GPI on Col7 (RINGER/SILENT KEY) event is part of event FIFO */
#define ADP5588_CONFIG_GPI_EM2 0x80

/* COl8 & Col9 GPIOs are outputs (EL_EN_1 and EL_EN_2) */
#define ADP5588_CONFIG_GPIO_DIR3 0x03

/* GPI on Col7 (RINGER/SILENT KEY) INT is LOW */
#define ADP5588_CONFIG_INT_LVL2 0x00

/* GPI on Col7 (RINGER/SILENT KEY) pull UP */
#define ADP5588_CONFIG_GPIO_PULL2 0x00

/******************************************************************************
*  Keypad Configuration
******************************************************************************/
#define AD5588_RINGERSILENT_SW 0x70   /* RINGER/SILENT switch is GPI Col 7 */

/******************************************************************************
*  Key Data FLAGS
******************************************************************************/
#define ADP5588_KEY_RELEASE    0x80   /* Key release BIT */
#define ADP5588_MAX_KEY_CODE   0x50   /* MAX Key code */
#define ADP5588_KEY_CODE       0x7f   /* Key code mask  */
#define ADP5588_MAX_GPIO_EVENT 0x72   /* MAX GPIO EVENT  code */
#define ADP5588_KEY_FIFO_EMPTY 0x00   /* Queue is empty */
#define ADP5588_RINGER_KEY_BIT 0x80   /* RINGER/SILENT in REG_GPIO_DAT_STAT2 */


struct ADP5588_cfg {
	uint16_t reg;
	uint8_t  data;
};

struct ADP5588_CMD_T {
	uint8_t reg;
	uint8_t data;
};

static struct ADP5588_cfg adp5588_init_seq[] =
{
	/* NO I2C auto increment|Overflow ON| INT  re-assert|Key event INT */
	{ ADP5588_REG_CONFIG, ADP5588_CONFIG_DEVICE },

	/* Key lock Disable*/
	{ ADP5588_REG_KEY_LCK_EC_STAT, ADP5588_CONFIG_KEY_LOCK },

	/* Set EL_EN_1 and EL_EN_2 line to 0 by default*/
	{ ADP5588_REG_GPIO_DAT_OUT3, ADP5588_CONFIG_GPIO_DAT_OUT3 },

	/* Enable GPIO Interrupt for RINGER/SILENT KEY*/
	{ ADP5588_REG_GPIO_INT_EN2, ADP5588_CONFIG_GPIO_INT2 },

	/*  Configure  8x7 Keypad  Matrix*/

	/*  Row0 to Row7  are KP Matrix*/
	{ ADP5588_REG_KP_GPIO1, ADP5588_CONFIG_KP_GPIO1 },

	/*  Col0 to Col6  are KP Matrix | Col7  is GPIO*/
	{ ADP5588_REG_KP_GPIO2, ADP5588_CONFIG_KP_GPIO2 },

	/* COl8 & Col9  are GPIOs */
	{ ADP5588_REG_KP_GPIO3, ADP5588_CONFIG_KP_GPIO3 },

	/* GPI  on Col7 ( RINGER/SILENT KEY ) event is part of event FIFO */
	{ ADP5588_REG_GPI_EM2, ADP5588_CONFIG_GPI_EM2 },

	/* COl8 & Col9  GPIOs  are outputs  ( EL_EN_1 and EL_EN_2 ) */
	{ ADP5588_REG_GPIO_DIR3, ADP5588_CONFIG_GPIO_DIR3 },

	/* GPI  on Col7 ( RINGER/SILENT KEY ) INT is LOW  */
	{ ADP5588_REG_GPIO_INT_LVL2, ADP5588_CONFIG_INT_LVL2 },

	/* GPI  on Col7 ( RINGER/SILENT KEY ) pull UP */
	{ ADP5588_REG_GPIO_PULL2, ADP5588_CONFIG_GPIO_PULL2 },

	/* Clear  Interrupt */
	{ ADP5588_REG_INT_STAT, 0x05 },

	/* Must end with 0 reg */
	{ 0, 0x00 },
};

static const char adp5588_dev_name[] = ADP5588_KEYPAD_NAME;

static struct adp5588_platform_data *adp5588_pdata;
static struct input_dev             *adp5588_input_dev;
static uint16_t                     *adp5588_keymap;

#define I2C_MUTEX_LOCK()    mutex_lock(&adp5588_pdata->mutex)
#define I2C_MUTEX_UNLOCK()  mutex_unlock(&adp5588_pdata->mutex)
#define clk_busy_wait(time) msleep_interruptible(time/1000)

#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp5588_early_suspend(struct early_suspend *handler);
static void adp5588_early_resume(struct early_suspend *handler);
#endif


static int adp5588_read_register(struct i2c_client *client,
				uint8_t address, uint8_t *value)
{
	uint8_t data[1];
	int rc;

	I2C_MUTEX_LOCK();

	data[0] = address;

	rc = i2c_master_send(client, data, 1);

	if (rc < 0) {
		printk(KERN_ERR "%s: i2c_master_send error %d\n",
			__func__, rc);

		I2C_MUTEX_UNLOCK();

		return rc;
	}

	clk_busy_wait(1000);

	*value = 0;

	rc = i2c_master_recv(client, value, 1);

	if (rc < 0)
		printk(KERN_ERR "%s: i2c_master_recv error %d\n",
			__func__, rc);

	I2C_MUTEX_UNLOCK();

	return rc;
}


static int adp5588_write_register(struct i2c_client *client,
				uint8_t address, uint8_t value)
{
	int rc;
	struct ADP5588_CMD_T msg;

	msg.reg = address;
	msg.data = value;

	I2C_MUTEX_LOCK();

	rc = i2c_master_send(client, (uint8_t *)&msg, sizeof(msg));

	if (rc < 0)
		printk(KERN_ERR "%s: i2c_master_send error %d\n",
			__func__, rc);

	I2C_MUTEX_UNLOCK();

	return rc;
}


/* adp5588 write configuration */
static int adp5588_write_config(struct i2c_client *client,
				struct ADP5588_cfg *cfg)
{
	int i;
	int ret = 0;
	struct ADP5588_CMD_T msg;

	if (!client || !cfg)
		return -1;

	I2C_MUTEX_LOCK();

	for (i = 0; cfg[i].reg != 0; i++) {
		msg.reg = cfg[i].reg;
		msg.data = cfg[i].data;

		ret = i2c_master_send(client, (uint8_t *)&msg, sizeof(msg));

		if (ret < 0)
			printk(KERN_ERR
				"%s: i2c_master_send error %d\n",
				__func__, ret);

		clk_busy_wait(1000);
	}

	I2C_MUTEX_UNLOCK();

	return ret;
}


/* adp5588 initial configuration */
static int adp5588_config(struct i2c_client *client)
{
	clk_busy_wait(1000);

	return adp5588_write_config(client, adp5588_init_seq);
}


static int adp5588_clear_irq(struct i2c_client *client)
{
	/* clear  key event & key lock irq */
	return adp5588_write_register(client, ADP5588_REG_INT_STAT, 0x5);
}


/* SET backlight drive register. Bit0=EL_EN1, Bit1=EL_EN2 */
int adp5588_set_backlight(uint8_t mask)
{
	int rc;

	if (!adp5588_pdata) {
		printk(KERN_ERR "%s: Driver not ready\n", __func__);
		return -ENODEV;
	}

	if (!adp5588_pdata->client) {
		printk(KERN_ERR "%s: No I2C support\n", __func__);
		return -ENOTSUPP;
	}

	dev_dbg(&adp5588_pdata->client->dev, "%s: Set mask=%x\n",
		__func__, mask);

	rc = adp5588_write_register(adp5588_pdata->client,
			ADP5588_REG_GPIO_DAT_OUT3, mask);

	if (rc < 0)
		return rc;

	adp5588_pdata->leds_mask = mask;

	return 0;
}
EXPORT_SYMBOL(adp5588_set_backlight);


/* GET backlight drive register value. Bit0=EL_EN1, Bit1=EL_EN2 */
int adp5588_get_backlight(void)
{
	if (!adp5588_pdata) {
		printk(KERN_ERR "%s: Driver not ready\n", __func__);
		return -ENODEV;
	}

	dev_dbg(&adp5588_pdata->client->dev, "%s: mask=%x\n",
		__func__, adp5588_pdata->leds_mask);

	return adp5588_pdata->leds_mask;
}
EXPORT_SYMBOL(adp5588_get_backlight);


#ifdef ADP5588_DEMO_BOARD_RINGER_SW
static void adp5588_simulate_ringer_sw(struct adp5588_platform_data *kp)
{
	kp->ringer_switch = !kp->ringer_switch;

	input_report_switch(adp5588_input_dev,
		SW_HEADPHONE_INSERT, kp->ringer_switch);

	printk(KERN_INFO "%s: RINGER/SILENT is %d\n",
		__func__, adp5588_pdata->ringer_switch);

}
#endif


static void adp5588_reset(void)
{
	printk(KERN_INFO "%s: ADP5588: RESET asserted\n", __func__);

#ifdef ADP5588_DEMO_BOARD_RESET
	clk_busy_wait(2000);
#else
	/* Assert ADP5588 RESET */
	gpio_set_value(adp5588_pdata->reset_gpio, 0);

	/* This will keep RESET asserted for ~15 msec. */
	clk_busy_wait(1000);

	/* Deassert ADP5588 RESET */
	gpio_set_value(adp5588_pdata->reset_gpio, 1);

	/* Allow ADP5588 to come out of reset */
	clk_busy_wait(1000);
#endif

	printk(KERN_INFO "%s: ADP5588: RESET deasserted\n", __func__);
}


static void adp5588_work_func(struct work_struct *work)
{
	int rc;
	int i;
	int row;
	int col;
	int key_index;
	uint8_t scan_code;
	uint8_t reg = ADP5588_REG_KEY_EVENTA;
	struct adp5588_platform_data *kp =
			container_of(work, struct adp5588_platform_data, work);
	struct i2c_client *client = kp->client;

	I2C_MUTEX_LOCK();

	/* set  read address */
	rc = i2c_master_send(client, &reg, 1);

	if (rc < 0) {
		printk(KERN_ERR "%s: i2c_master_send error %d\n",
			__func__, rc);
		/* I2c Write error. Exit now, enable IRQ */
		/* and read again if IRQ pin  still low  */
		goto adp5588_work_func_exit;
	}

	/* read scancodes until queue is empty */
	i = 0;

	do {
		scan_code = ADP5588_KEY_FIFO_EMPTY;

		rc = i2c_master_recv(client, &scan_code, 1);

		if (rc < 0) {
			printk(KERN_ERR
				" %s: i2c_master_recv error %d\n",
				__func__, rc);
			break;
		}

		if (scan_code != ADP5588_KEY_FIFO_EMPTY) {
			kp->last_key = ADP5588_KEY_CODE & scan_code;
			kp->last_key_state =
				!!(ADP5588_KEY_RELEASE & scan_code);

			dev_dbg(&client->dev,
				"ADP5588 got scancode %d, "
				"keycode 0x%x ( %d ) state %d \n",
				scan_code, kp->last_key,
				kp->last_key, kp->last_key_state);

			if (kp->last_key == AD5588_RINGERSILENT_SW) {
				/* got RINGER/SILENT switch event */
				adp5588_pdata->ringer_switch =
					kp->last_key_state;

				dev_dbg(&client->dev,
					"RINGER/SILENT switch=%d\n",
					kp->last_key_state);

				input_report_switch(adp5588_input_dev,
					SW_HEADPHONE_INSERT,
					adp5588_pdata->ringer_switch);
			} else {
				/* got regular key event */
				row = (kp->last_key - 1) / ADP5588_COL_NUM_MAX;
				col = (kp->last_key - 1) % ADP5588_COL_NUM_MAX;
				key_index = (col * ADP5588_ROW_NUM_MAX) + row;

#ifdef ADP5588_DEMO_BOARD_RINGER_SW
				/* If demo-board, simulate ringer sw */
				/* through the key at row 6 col 5.   */
				if ((row == 6) && (col == 5)) {
					adp5588_simulate_ringer_sw(kp);
				} else {
#endif
				dev_dbg(&client->dev,
					"col=%d row=%d index=%d "
					"key=0x%02x\n",
					col, row, key_index,
					adp5588_keymap[key_index]);

				input_report_key(adp5588_input_dev,
					adp5588_keymap[key_index],
					kp->last_key_state);
#ifdef ADP5588_DEMO_BOARD_RINGER_SW
				}
#endif
			}

			/* Sync is not required for key event */
			/* input_sync(adp5588_input_dev);     */
		} else {
			if (i == 0) {
				/* IRQ without the code must be key */
				/* release HW problem. Wait 5ms     */
				clk_busy_wait(5000);
			}
		}
	} while (scan_code && (i++ < 10));

adp5588_work_func_exit:

	I2C_MUTEX_UNLOCK();

	/* Clear IRQ and enable it */
	if (kp->use_irq) {
		adp5588_clear_irq(client);
		enable_irq(client->irq);
	}
}


/* adp5588 timer handler */
static enum hrtimer_restart adp5588_timer_func(struct hrtimer *timer)
{
	struct adp5588_platform_data *kp =
		container_of(timer, struct adp5588_platform_data, timer);

	schedule_work(&kp->work);

	hrtimer_start(&kp->timer, ktime_set(0, 125000000), HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}


/* adp5588 interrupt handler */
static irqreturn_t adp5588_irq_handler(int irq, void *dev_id)
{
	struct adp5588_platform_data *kp =
		(struct adp5588_platform_data *)dev_id;

	disable_irq_nosync(irq);
	schedule_work(&kp->work);

	return IRQ_HANDLED;
}


/* This function is called by gpio_event_call_all_func(),
   since gpio_event owns the input device.
*/
int gpio_event_adp5588_func(struct gpio_event_input_devs *input_devs,
	struct gpio_event_info *info, void **data, int func)
{
	int i;
	int key_count;
	uint16_t keycode;
	struct gpio_event_matrix_info *mi;

	if (func != GPIO_EVENT_FUNC_INIT)
		return 0;

	mi = container_of(info, struct gpio_event_matrix_info, info);

	if (mi->keymap == NULL) {
		printk(KERN_ERR
		"adp5588: %s: Missing keymap table\n", __func__);
		return -EINVAL;
	}

	if ((mi->noutputs > ADP5588_COL_NUM_MAX) ||
		(mi->ninputs > ADP5588_ROW_NUM_MAX)) {
		printk(KERN_ERR
		"adp5588: %s: Invalid keymap size: "
		"mi->noutputs=%d mi->ninputs=%d\n",
		__func__, mi->noutputs, mi->ninputs);
			return -EINVAL;
	}

	if (input_devs->count != 1) {
		printk(KERN_ERR
		"adp5588: %s: Invalid device count: "
		"input_devs->count=%d\n",
		__func__, input_devs->count);
		return -EINVAL;
	}

	adp5588_input_dev = input_devs->dev[0];
	adp5588_keymap = (uint16_t *)mi->keymap;

	key_count = mi->ninputs * mi->noutputs;

	for (i = 0; i < key_count; i++) {
		keycode = mi->keymap[i] & MATRIX_KEY_MASK;

		if (keycode && (keycode <= KEY_MAX))
			input_set_capability(adp5588_input_dev,
					EV_KEY, keycode);
	}

	input_set_capability(adp5588_input_dev, EV_SW, SW_HEADPHONE_INSERT);

	return 0;
}


/* This function is called by i2c_probe */
static int adp5588_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
#ifdef ADP5588_DEMO_BOARD_VREG
	static struct regulator *regulator_vwlan1;
#endif
	uint8_t reg;
	int ret;
	int err = 0;
	unsigned long request_flags = IRQF_TRIGGER_LOW;
	struct platform_device *adp5588_keypad_leds_ptr;

	adp5588_pdata = client->dev.platform_data;
	adp5588_keypad_leds_ptr = adp5588_pdata->leds_device;

	if (!adp5588_pdata->use_adp5588) {
		printk(KERN_ERR
			"ADP5588 keypad device declared unavailable\n");
		return -ENODEV;
	}

#ifdef ADP5588_DEMO_BOARD_VREG
	regulator_vwlan1 = regulator_get(NULL, "vwlan1");

	if (IS_ERR(regulator_vwlan1)) {
		printk(KERN_ERR
			"%s: Cannot get regulator_vwlan1, err=%ld\n",
			__func__, PTR_ERR(regulator_vwlan1));
		return PTR_ERR(regulator_vwlan1);
	}

	if (regulator_enable(regulator_vwlan1) != 0) {
		printk(KERN_ERR
			"%s: Cannot enable regulator_vwlan1\n", __func__);
		return -EIO;
	}

	printk(KERN_INFO "%s: VWLAN1 turned on\n", __func__);
#endif

	mutex_init(&adp5588_pdata->mutex);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR
			"%s: No i2c support\n", __func__);
		return -ENOTSUPP;
	}

	adp5588_pdata->client = client;
	i2c_set_clientdata(client, adp5588_pdata);

	printk(KERN_INFO "%s: ADP5588: RESET_GPIO=%d INT_GPIO=%d\n",
		__func__, adp5588_pdata->reset_gpio, adp5588_pdata->int_gpio);

	adp5588_reset();

	/* Configure APD5588 chip */
	adp5588_config(client);

	/* Get Chip ID */
	adp5588_read_register(client,
		ADP5588_REG_DEV_ID, &adp5588_pdata->dev_id);

	printk(KERN_INFO "%s: ADP5588 Keypad DevID=0x%x, Name=%s\n",
		__func__, adp5588_pdata->dev_id, adp5588_dev_name);

	INIT_WORK(&adp5588_pdata->work, adp5588_work_func);

	if (adp5588_input_dev == NULL) {
		printk(KERN_ERR "%s: Can't allocate input device \n", __func__);
		return -ENODEV;
	}

	adp5588_input_dev->name = adp5588_dev_name;
	adp5588_input_dev->id.bustype = BUS_I2C;
	adp5588_input_dev->id.vendor = adp5588_pdata->dev_id >> 4;
	adp5588_input_dev->id.product = adp5588_pdata->dev_id | 0xf;
	adp5588_input_dev->id.version = 0x0100;

	if (client->irq) {
		/* Override INT GPIO with the one configured in device_tree */
		client->irq = adp5588_pdata->int_gpio;

		err = request_irq(client->irq, adp5588_irq_handler,
				request_flags, ADP5588_KEYPAD_NAME,
				adp5588_pdata);

		if (err == 0) {
			adp5588_pdata->use_irq = 1;
			err = set_irq_wake(client->irq, 1);
		} else {
			printk(KERN_ERR "%s: request irq %d failed\n",
				__func__, client->irq);
			adp5588_pdata->use_irq = 0;
			free_irq(client->irq, adp5588_pdata);
		}
	}

	if (!adp5588_pdata->use_irq) {
		hrtimer_init(&adp5588_pdata->timer,
			CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		adp5588_pdata->timer.function = adp5588_timer_func;
		hrtimer_start(&adp5588_pdata->timer,
			ktime_set(1, 0), HRTIMER_MODE_REL);
	}

	printk(KERN_INFO "ADP5588 : Start keypad %s in %s mode irq %d \n",
		adp5588_input_dev->name,
		adp5588_pdata->use_irq ? "interrupt" : "polling", client->irq);

	if (adp5588_read_register(client,
		ADP5588_REG_GPIO_DAT_STAT2, &reg) < 0) {
		adp5588_pdata->ringer_switch = 0;
		printk(KERN_ERR " %s: Can't read RINGER/SILENT "
			"switch state. Defaulted it to OFF\n",
			__func__);
	} else {
		adp5588_pdata->ringer_switch = !(ADP5588_RINGER_KEY_BIT & reg);
	}

	printk(KERN_INFO "%s: RINGER/SILENT switch is %s\n",
		__func__, adp5588_pdata->ringer_switch ? "ON" : "OFF");

	input_report_switch(adp5588_input_dev,
		SW_HEADPHONE_INSERT, adp5588_pdata->ringer_switch);
	input_sync(adp5588_input_dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	/* Level should be higher than keypad backlight driver! */
	adp5588_pdata->early_suspend.level =
		EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	adp5588_pdata->early_suspend.suspend = adp5588_early_suspend;
	adp5588_pdata->early_suspend.resume = adp5588_early_resume;
	register_early_suspend(&(adp5588_pdata->early_suspend));
#endif

	adp5588_pdata->leds_mask = 0;

	/* Add the keypad leds device */
	ret = platform_add_devices(&adp5588_keypad_leds_ptr, 1);

	if (ret < 0) {
		printk(KERN_ERR "%s: platform_add_devices() failed: %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}


static int adp5588_remove(struct i2c_client *client)
{
	struct adp5588_platform_data *kp = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s: enter.\n", __func__);

	if (kp->use_irq)
		free_irq(client->irq, kp);
	else
		hrtimer_cancel(&kp->timer);

#ifndef ADP5588_DEMO_BOARD_RESET
	gpio_free(kp->reset_gpio);
#endif

	if (adp5588_input_dev)
		input_unregister_device(adp5588_input_dev);

	if (kp->leds_device)
		platform_device_unregister(adp5588_pdata->leds_device);

/*	kfree(kp);*/

	return 0;
}


static int adp5588_suspend(struct i2c_client *client, pm_message_t mesg)
{
	client = client;
	mesg = mesg;

	return 0;
}


static int adp5588_resume(struct i2c_client *client)
{
	adp5588_write_config(client, adp5588_init_seq);

	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void adp5588_early_suspend(struct early_suspend *handler)
{
	adp5588_suspend(adp5588_pdata->client, PMSG_SUSPEND);
}


static void adp5588_early_resume(struct early_suspend *handler)
{
	adp5588_resume(adp5588_pdata->client);
}
#endif


static const struct i2c_device_id adp5588_id[] = {
	{ ADP5588_KEYPAD_NAME, 0 },
	{ }
};

/* This is the driver that will be inserted */
static struct i2c_driver adp5588_keypad_driver = {
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= adp5588_suspend,
	.resume		= adp5588_resume,
#endif
	.probe		= adp5588_probe,
	.remove		= adp5588_remove,
	.id_table	= adp5588_id,
	.driver		= {
		.name = ADP5588_KEYPAD_NAME,
	},
};


static int __devinit adp5588_init(void)
{
	return i2c_add_driver(&adp5588_keypad_driver);
}


static void __exit adp5588_exit(void)
{
	i2c_del_driver(&adp5588_keypad_driver);
}


module_init(adp5588_init);
module_exit(adp5588_exit);

MODULE_DESCRIPTION("ADP5588 KEYPAD DRIVER");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
