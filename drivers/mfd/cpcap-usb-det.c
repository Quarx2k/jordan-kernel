/*
 * Copyright (C) 2007 - 2010 Motorola, Inc.
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

#include <linux/errno.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include <linux/regulator/consumer.h>

#include <linux/spi/cpcap.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/spi.h>

#define CPCAP_SENSE4_LS		8
#define CPCAP_BIT_DP_S_LS	(CPCAP_BIT_DP_S << CPCAP_SENSE4_LS)
#define CPCAP_BIT_DM_S_LS	(CPCAP_BIT_DM_S << CPCAP_SENSE4_LS)

#define SENSE_USB           (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_USB_FLASH     (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_FACTORY       (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

/* This Sense mask is needed because on TI the CHRGCURR1 interrupt is not always
 * set.  In Factory Mode the comparator follows the Charge current only. */
#define SENSE_FACTORY_COM   (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S)

#define SENSE_CHARGER_FLOAT (CPCAP_BIT_ID_FLOAT_S  | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_SE1_S       | \
			     CPCAP_BIT_DM_S_LS     | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_CHARGER       (CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
				 CPCAP_BIT_SE1_S       | \
				 CPCAP_BIT_DM_S_LS     | \
				 CPCAP_BIT_DP_S_LS)

#define SENSE_OTG_CABLE (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_CHRGCURR1_S)

#define SENSE_OTG_DEVICE (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_VBUSVLD_S   | \
			     CPCAP_BIT_SESSVLD_S   | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_DP_S_LS)

#define SENSE_OTG (CPCAP_BIT_ID_GROUND_S | \
			     CPCAP_BIT_CHRGCURR1_S | \
			     CPCAP_BIT_SESSVLD_S)


#define UNDETECT_TRIES		5

enum cpcap_det_state {
	CONFIG,
	SAMPLE_1,
	SAMPLE_2,
	IDENTIFY,
	USB,
	FACTORY,
    USB_DEVICE,
};

enum cpcap_accy {
	CPCAP_ACCY_USB,
	CPCAP_ACCY_FACTORY,
	CPCAP_ACCY_CHARGER,
	CPCAP_ACCY_USB_DEVICE,
	CPCAP_ACCY_NONE,

	/* Used while debouncing the accessory. */
	CPCAP_ACCY_UNKNOWN,
};

static const char *accy_names[8] = {
	"USB",
	"FACTORY",
	"CHARGER",
	"USB DEVICE",
	"NONE",
	"UNKNOWN",
};

struct cpcap_usb_det_data {
	struct cpcap_device *cpcap;
	struct delayed_work work;
	unsigned short sense;
	unsigned short prev_sense;
	enum cpcap_det_state state;
	enum cpcap_accy usb_accy;
	struct platform_device *usb_dev;
	struct platform_device *usb_connected_dev;
	struct platform_device *charger_connected_dev;
	struct regulator *regulator;
	struct wake_lock wake_lock;
	unsigned char is_vusb_enabled;
	unsigned char undetect_cnt;
};

static const char *accy_devices[] = {
	"cpcap_usb_charger",
	"cpcap_factory",
	"cpcap_charger",
};

/* Expects values from 0 to 2: 0=no_log, 1=basic_log, 2=max_log */
static int cpcap_usb_det_debug = 0;

#ifdef CONFIG_USB_TESTING_POWER
static int testing_power_enable = -1;
module_param(testing_power_enable, int, 0644);
MODULE_PARM_DESC(testing_power_enable, "Enable factory cable power "
	"supply function for testing");
#endif

static void vusb_enable(struct cpcap_usb_det_data *data)
{
	if (!data->is_vusb_enabled) {
		wake_lock(&data->wake_lock);
		regulator_enable(data->regulator);
		data->is_vusb_enabled = 1;
	}
}

static void vusb_disable(struct cpcap_usb_det_data *data)
{
	if (data->is_vusb_enabled) {
		wake_unlock(&data->wake_lock);
		regulator_disable(data->regulator);
		data->is_vusb_enabled = 0;
	}
}

static void dump_sense_bits(struct cpcap_usb_det_data *data)
{
	if (CPCAP_BIT_CHRGCURR1_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_CHRGCURR1_S\n");
	if (CPCAP_BIT_DM_S_LS & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_DM_S_LS\n");
	if (CPCAP_BIT_DP_S_LS & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_DP_S_LS)\n");
	if (CPCAP_BIT_ID_FLOAT_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_ID_FLOAT_S\n");
	if (CPCAP_BIT_ID_GROUND_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_ID_GROUND_S\n");
	if (CPCAP_BIT_SE1_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_SE1_S\n");
	if (CPCAP_BIT_SESSVLD_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_SESSVLD_S\n");
	if (CPCAP_BIT_VBUSVLD_S & data->sense)
		pr_info("cpcap_usb_det: SenseBit = CPCAP_BIT_VBUSVLD_S\n");

	if (SENSE_CHARGER == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_CHARGER\n");
	if (SENSE_CHARGER_FLOAT == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_CHARGER_FLOAT\n");
	if (SENSE_FACTORY == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_FACTORY\n");
	if (SENSE_FACTORY_COM == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_FACTORY_COM\n");
	if (SENSE_USB == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_USB\n");
	if (SENSE_USB_FLASH == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_USB_FLASH\n");
	if (SENSE_OTG_CABLE == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_OTG_CABLE\n");
	if (SENSE_OTG_DEVICE == data->sense)
		pr_info("cpcap_usb_det: Sense Pattern = SENSE_OTG_DEVICE\n");

}

static int get_sense(struct cpcap_usb_det_data *data)
{
	int retval = -EFAULT;
	unsigned short value;
	struct cpcap_device *cpcap;

	if (!data)
		return -EFAULT;
	cpcap = data->cpcap;

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS1, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT1,
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_FLOAT_I |
				      CPCAP_BIT_ID_GROUND_I),
				     (CPCAP_BIT_CHRG_DET_I |
				      CPCAP_BIT_ID_FLOAT_I |
				      CPCAP_BIT_ID_GROUND_I));
	if (retval)
		return retval;

	data->sense = value & (CPCAP_BIT_ID_FLOAT_S |
			       CPCAP_BIT_ID_GROUND_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS2, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT2,
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I),
				    (CPCAP_BIT_CHRGCURR1_I |
				     CPCAP_BIT_VBUSVLD_I |
				     CPCAP_BIT_SESSVLD_I |
				     CPCAP_BIT_SE1_I));
	if (retval)
		return retval;

	data->sense |= value & (CPCAP_BIT_CHRGCURR1_S |
				CPCAP_BIT_VBUSVLD_S |
				CPCAP_BIT_SESSVLD_S |
				CPCAP_BIT_SE1_S);

	retval = cpcap_regacc_read(cpcap, CPCAP_REG_INTS4, &value);
	if (retval)
		return retval;

	/* Clear ASAP after read. */
	retval = cpcap_regacc_write(cpcap, CPCAP_REG_INT4,
				     (CPCAP_BIT_DP_I |
				      CPCAP_BIT_DM_I),
				     (CPCAP_BIT_DP_I |
				      CPCAP_BIT_DM_I));
	if (retval)
		return retval;

	data->sense |= (value & (CPCAP_BIT_DP_S |
			       CPCAP_BIT_DM_S)) << CPCAP_SENSE4_LS;

	if (cpcap_usb_det_debug && data->state > SAMPLE_2) {
		pr_info("cpcap_usb_det: SenseBits = 0x%04x\n", data->sense);
		if (cpcap_usb_det_debug > 1) {
		    dump_sense_bits(data);
		}
	}

	return 0;
}

static int configure_hardware(struct cpcap_usb_det_data *data,
			      enum cpcap_accy accy)
{
	int retval;

	/* Take control of pull up from ULPI. */
	retval  = cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     CPCAP_BIT_PU_SPI,
				     CPCAP_BIT_PU_SPI);
	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
				    CPCAP_BIT_DP150KPU,
				    (CPCAP_BIT_DP150KPU | CPCAP_BIT_DP1K5PU |
				     CPCAP_BIT_DM1K5PU | CPCAP_BIT_DPPD |
				     CPCAP_BIT_DMPD));

	switch (accy) {
	case CPCAP_ACCY_USB:
	case CPCAP_ACCY_FACTORY:
		/* Disable VBus PullDown */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		/* Enable USB xceiver */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2,
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		/* Give USB driver control of pull up via ULPI. */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     0,
					     CPCAP_BIT_PU_SPI |
					     CPCAP_BIT_DMPD_SPI |
					     CPCAP_BIT_DPPD_SPI |
					     CPCAP_BIT_SUSPEND_SPI |
					     CPCAP_BIT_ULPI_SPI_SEL);

		if ((data->cpcap->vendor == CPCAP_VENDOR_ST) &&
			(data->cpcap->revision == CPCAP_REVISION_2_0))
				vusb_enable(data);

		break;

	case CPCAP_ACCY_CHARGER:
		/* Disable Reverse Mode */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     0, CPCAP_BIT_RVRSMODE);
		/* Enable VBus PullDown */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		break;

	case CPCAP_ACCY_USB_DEVICE:
		/* Remove VBus PullDown */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		/* Enable Reverse Mode */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     CPCAP_BIT_RVRSMODE,
					     CPCAP_BIT_RVRSMODE);
		/* enable USB xceiver */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, 
					     CPCAP_BIT_USBXCVREN,
					     CPCAP_BIT_USBXCVREN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     0,
					     CPCAP_BIT_DMPD_SPI |
					     CPCAP_BIT_DPPD_SPI |
					     CPCAP_BIT_SUSPEND_SPI |
					     CPCAP_BIT_ULPI_SPI_SEL);
		/* disable VBUS standby */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3, 0,
					     CPCAP_BIT_VBUSSTBY_EN);
		break;

	case CPCAP_ACCY_UNKNOWN:
		/* Remove VBus PullDown */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1, 0,
					     CPCAP_BIT_VBUSPD);
		/* Disable Reverse Mode */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     0, CPCAP_BIT_RVRSMODE);
		break;

	case CPCAP_ACCY_NONE:
	default:
		/* Disable Reverse Mode */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_CRM,
					     0, CPCAP_BIT_RVRSMODE);
	//	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_VUSBC, 0,
	//				     CPCAP_BIT_VBUS_SWITCH);
		/* Enable VBus PullDown */
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC1,
					     CPCAP_BIT_VBUSPD,
					     CPCAP_BIT_VBUSPD);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC2, 0,
					     CPCAP_BIT_USBXCVREN);
		retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
					     CPCAP_BIT_DMPD_SPI |
					     CPCAP_BIT_DPPD_SPI |
					     CPCAP_BIT_SUSPEND_SPI |
					     CPCAP_BIT_ULPI_SPI_SEL |
					     CPCAP_BIT_VBUSSTBY_EN,
					     CPCAP_BIT_DMPD_SPI |
					     CPCAP_BIT_DPPD_SPI |
					     CPCAP_BIT_SUSPEND_SPI |
					     CPCAP_BIT_ULPI_SPI_SEL |
					     CPCAP_BIT_VBUSSTBY_EN);
		break;
	}

	if (retval != 0)
		retval = -EFAULT;

	return retval;
}

extern void cpcap_musb_notifier_call(unsigned char event);

static void notify_accy(struct cpcap_usb_det_data *data, enum cpcap_accy accy)
{
	if (cpcap_usb_det_debug > 1)
		pr_info("cpcap_usb_det %s: accy=%s\n", __func__, accy_names[accy]);

	dev_info(&data->cpcap->spi->dev, "notify_accy: accy=%d\n", accy);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL)) {
		platform_device_del(data->usb_dev);
		data->usb_dev = NULL;
	}

	if ((accy == CPCAP_ACCY_USB) || (accy == CPCAP_ACCY_FACTORY)) {
		printk("USB connected!\n");
		cpcap_musb_notifier_call(1);
	} else if (accy == CPCAP_ACCY_USB_DEVICE) {
		printk("OTG connected!\n");
		cpcap_musb_notifier_call(2);
	}

	configure_hardware(data, accy);
	data->usb_accy = accy;

	if (accy != CPCAP_ACCY_NONE) {
		if (accy != CPCAP_ACCY_USB_DEVICE)
			data->usb_dev = platform_device_alloc(accy_devices[accy], -1);
		if (data->usb_dev) {
			data->usb_dev->dev.platform_data = data->cpcap;
			platform_device_add(data->usb_dev);
		}
	} else
		vusb_disable(data);

	if ((accy == CPCAP_ACCY_USB) || (accy == CPCAP_ACCY_FACTORY) || (accy == CPCAP_ACCY_USB_DEVICE)) {
		if (!data->usb_connected_dev) {
			data->usb_connected_dev =
			    platform_device_alloc("cpcap_usb_connected", -1);
			platform_device_add(data->usb_connected_dev);
		}
	} else if (data->usb_connected_dev) {
		platform_device_del(data->usb_connected_dev);
		data->usb_connected_dev = NULL;
	}

	if (accy == CPCAP_ACCY_NONE || accy == CPCAP_ACCY_CHARGER) {
		printk("USB disconnected!\n");
		cpcap_musb_notifier_call(0);
	}

	if (accy == CPCAP_ACCY_CHARGER) {
		if (!data->charger_connected_dev) {
			data->charger_connected_dev =
			    platform_device_alloc("cpcap_charger_connected",
						  -1);
			platform_device_add(data->charger_connected_dev);
		}
	} else if (data->charger_connected_dev) {
		platform_device_del(data->charger_connected_dev);
		data->charger_connected_dev = NULL;
	}
}

static void detection_work(struct work_struct *work)
{
	struct cpcap_usb_det_data *data =
		container_of(work, struct cpcap_usb_det_data, work.work);

	switch (data->state) {
	case CONFIG:
		vusb_enable(data);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DPI);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_DMI);
		cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SESSVLD);

		configure_hardware(data, CPCAP_ACCY_UNKNOWN);

		data->undetect_cnt = 0;
		data->state = SAMPLE_1;
		schedule_delayed_work(&data->work, msecs_to_jiffies(11));
		break;

	case SAMPLE_1:
		get_sense(data);
		data->state = SAMPLE_2;
		schedule_delayed_work(&data->work, msecs_to_jiffies(100));
		break;

	case SAMPLE_2:
		data->prev_sense = data->sense;
		get_sense(data);

		if (data->prev_sense != data->sense) {
			/* Stay in this state */
			data->state = SAMPLE_2;
			schedule_delayed_work(&data->work,
					      msecs_to_jiffies(100));
		} else if (!(data->sense & CPCAP_BIT_SE1_S) &&
			   (data->sense & CPCAP_BIT_ID_FLOAT_S) &&
			   !(data->sense & CPCAP_BIT_ID_GROUND_S) &&
			   !(data->sense & CPCAP_BIT_SESSVLD_S)) {
			/* cable may not be fully inserted: wait a bit more & try again... */
			if (cpcap_usb_det_debug > 1)
				pr_info("cpcap_usb_det: SAMPLE_2 cable may not be fully inserted\n");
			data->state = IDENTIFY;
			schedule_delayed_work(&data->work,
					      msecs_to_jiffies(100));
		} else {
			/* cable connected: try to identify what was connected... */
			if (cpcap_usb_det_debug > 1)
				pr_info("cpcap_usb_det: cable connected.\n");
			data->state = IDENTIFY;
			schedule_delayed_work(&data->work, 0);
		}
		break;

	case IDENTIFY:
		get_sense(data);
		data->state = CONFIG;

		if ((data->sense == SENSE_USB) ||
		    (data->sense == SENSE_USB_FLASH)) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: USB or USB_FLASH\n");
			notify_accy(data, CPCAP_ACCY_USB);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			/* Special handling of USB cable undetect. */
			data->state = USB;
		} else if ((data->sense == SENSE_FACTORY) ||
			   (data->sense == SENSE_FACTORY_COM)) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: FACTORY Cable\n");
#ifdef CONFIG_USB_TESTING_POWER
			if (testing_power_enable > 0) {
				notify_accy(data, CPCAP_ACCY_NONE);
				cpcap_irq_unmask(data->cpcap,
					CPCAP_IRQ_CHRG_DET);
				cpcap_irq_unmask(data->cpcap,
					CPCAP_IRQ_CHRG_CURR1);
				cpcap_irq_unmask(data->cpcap,
				CPCAP_IRQ_VBUSVLD);
				break;
			}
#endif
			notify_accy(data, CPCAP_ACCY_FACTORY);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);

			/* Special handling of factory cable undetect. */
			data->state = FACTORY;
		} else if ((data->sense == SENSE_CHARGER_FLOAT) ||
			   (data->sense == SENSE_CHARGER)) {
			notify_accy(data, CPCAP_ACCY_CHARGER);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

			data->state = CONFIG;
		} else if (data->sense & CPCAP_BIT_ID_GROUND_S) {
			if (cpcap_usb_det_debug)
				pr_info("cpcap_usb_det: OTG cable attached\n");
			data->state = USB_DEVICE;

			/* mask VBUSVLD, CHGDET, SESSVLD as Reverse Mode enable may raise these */
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SESSVLD);

			notify_accy(data, CPCAP_ACCY_USB_DEVICE);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

		} else if ((data->sense & CPCAP_BIT_VBUSVLD_S) &&
				(data->usb_accy == CPCAP_ACCY_NONE)) {
			data->state = CONFIG;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DPI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_DMI);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);

		} else {
			notify_accy(data, CPCAP_ACCY_NONE);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);

			/* When a charger is unpowered by unplugging from the
			 * wall, VBUS voltage will drop below CHRG_DET (3.5V)
			 * until the ICHRG bits are cleared.  Once ICHRG is
			 * cleared, VBUS will rise above CHRG_DET, but below
			 * VBUSVLD (4.4V) briefly as it decays.  If the charger
			 * is re-powered while VBUS is within this window, the
			 * VBUSVLD interrupt is needed to trigger charger
			 * detection.
			 *
			 * VBUSVLD must be masked before going into suspend.
			 * See cpcap_usb_det_suspend() for details.
			 */
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);

			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDFLOAT);
		}
		break;

	case USB:
		get_sense(data);

		if ((data->sense & CPCAP_BIT_SE1_S) ||
			(data->sense & CPCAP_BIT_ID_GROUND_S)) {
				data->state = CONFIG;
				schedule_delayed_work(&data->work, 0);
		} else if (!(data->sense & CPCAP_BIT_VBUSVLD_S)) {
			if (data->undetect_cnt++ < UNDETECT_TRIES) {
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_CHRG_DET);
				cpcap_irq_mask(data->cpcap,
					       CPCAP_IRQ_CHRG_CURR1);
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_SE1);
				cpcap_irq_mask(data->cpcap, CPCAP_IRQ_IDGND);
				data->state = USB;
				schedule_delayed_work(&data->work,
						      msecs_to_jiffies(100));
			} else {
				data->state = CONFIG;
				schedule_delayed_work(&data->work, 0);
			}
		} else {
			data->state = USB;
			data->undetect_cnt = 0;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_IDGND);
		}
		break;

	case FACTORY:
		get_sense(data);

		/* The removal of a factory cable can only be detected if a
		 * charger is attached.
		 */
		if (data->sense & CPCAP_BIT_SE1_S) {
#ifdef CONFIG_TTA_CHARGER
			enable_tta();
#endif
			data->state = CONFIG;
			schedule_delayed_work(&data->work, 0);
		} else {
			data->state = FACTORY;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SE1);
		}
		break;

	case USB_DEVICE:
		get_sense(data);

		if (!(data->sense & CPCAP_BIT_ID_GROUND_S)) {
			pr_info("cpcap_usb_det: OTG cable detached\n");
			data->state = CONFIG;
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_VBUSVLD);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_CHRG_DET);
			cpcap_irq_unmask(data->cpcap, CPCAP_IRQ_SESSVLD);

			notify_accy(data, CPCAP_ACCY_NONE);
			schedule_delayed_work(&data->work, 0);
		}
		break;

	default:
		/* This shouldn't happen.  Need to reset state machine. */
		vusb_disable(data);
		data->state = CONFIG;
		schedule_delayed_work(&data->work, 0);
		break;
	}
}

static void int_handler(enum cpcap_irqs int_event, void *data)
{
	struct cpcap_usb_det_data *usb_det_data = data;
	schedule_delayed_work(&(usb_det_data->work), 0);
}

static int __init cpcap_usb_det_probe(struct platform_device *pdev)
{
	int retval;
	struct cpcap_usb_det_data *data;

	if (pdev->dev.platform_data == NULL) {
		dev_err(&pdev->dev, "no platform_data\n");
		return -EINVAL;
	}

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->cpcap = pdev->dev.platform_data;
	data->state = CONFIG;
	platform_set_drvdata(pdev, data);
	INIT_DELAYED_WORK(&data->work, detection_work);
	data->usb_accy = CPCAP_ACCY_NONE;
	wake_lock_init(&data->wake_lock, WAKE_LOCK_SUSPEND, "usb");
	data->undetect_cnt = 0;

	data->regulator = regulator_get(NULL, "vusb");
	if (IS_ERR(data->regulator)) {
		dev_err(&pdev->dev, "Could not get regulator for cpcap_usb\n");
		retval = PTR_ERR(data->regulator);
		goto free_mem;
	}
	regulator_set_voltage(data->regulator, 3300000, 3300000);

	/* Clear the interrupts so they are in a known state when starting detection. */
	retval = cpcap_irq_clear(data->cpcap, CPCAP_IRQ_CHRG_DET);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_SE1);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_IDGND);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_VBUSVLD);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_IDFLOAT);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_DPI);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_DMI);
	retval |= cpcap_irq_clear(data->cpcap, CPCAP_IRQ_SESSVLD);

	/* Register the interrupt handler, please be aware this will enable the
	   interrupts. */
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_DET,
				    int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_CHRG_CURR1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SE1,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDGND,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_VBUSVLD,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_IDFLOAT,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DPI,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_DMI,
				     int_handler, data);
	retval |= cpcap_irq_register(data->cpcap, CPCAP_IRQ_SESSVLD,
				     int_handler, data);

	/* Now that HW initialization is done, give USB control via ULPI. */
	retval |= cpcap_regacc_write(data->cpcap, CPCAP_REG_USBC3,
				     0, CPCAP_BIT_ULPI_SPI_SEL);

	if (retval != 0) {
		dev_err(&pdev->dev, "Initialization Error\n");
		retval = -ENODEV;
		goto free_irqs;
	}

	dev_info(&pdev->dev, "CPCAP USB detection device probed\n");

	/* Perform initial detection */
	detection_work(&(data->work.work));

	return 0;

free_irqs:
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDFLOAT);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DPI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SESSVLD);
	regulator_put(data->regulator);
free_mem:
	wake_lock_destroy(&data->wake_lock);
	kfree(data);

	return retval;
}

static int __exit cpcap_usb_det_remove(struct platform_device *pdev)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);

	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_DET);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_CHRG_CURR1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SE1);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDGND);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_VBUSVLD);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_IDFLOAT);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DPI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_DMI);
	cpcap_irq_free(data->cpcap, CPCAP_IRQ_SESSVLD);

	configure_hardware(data, CPCAP_ACCY_NONE);
	cancel_delayed_work_sync(&data->work);

	if ((data->usb_accy != CPCAP_ACCY_NONE) && (data->usb_dev != NULL))
		platform_device_del(data->usb_dev);

	vusb_disable(data);
	regulator_put(data->regulator);

	wake_lock_destroy(&data->wake_lock);

	kfree(data);
	return 0;
}

#ifdef CONFIG_PM
static int cpcap_usb_det_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct cpcap_usb_det_data *data = platform_get_drvdata(pdev);

	/* VBUSVLD cannot be unmasked when entering suspend. If left
	 * unmasked, a false interrupt will be received, keeping the
	 * device out of suspend. The interrupt does not need to be
	 * unmasked when resuming from suspend since the use case
	 * for having the interrupt unmasked is over.
	 */
	cpcap_irq_mask(data->cpcap, CPCAP_IRQ_VBUSVLD);

	return 0;
}
#else
#define cpcap_usb_det_suspend NULL
#endif

static struct platform_driver cpcap_usb_det_driver = {
	.probe		= cpcap_usb_det_probe,
	.remove		= __exit_p(cpcap_usb_det_remove),
	.suspend	= cpcap_usb_det_suspend,
	.driver		= {
		.name	= "cpcap_usb_det",
		.owner	= THIS_MODULE,
	},
};

static int __init cpcap_usb_det_init(void)
{
	return cpcap_driver_register(&cpcap_usb_det_driver);
}
/* The CPCAP USB detection driver must be started later to give the MUSB
 * driver time to complete its initialization. */
late_initcall(cpcap_usb_det_init);

static void __exit cpcap_usb_det_exit(void)
{
	platform_driver_unregister(&cpcap_usb_det_driver);
}
module_exit(cpcap_usb_det_exit);

MODULE_ALIAS("platform:cpcap_usb_det");
MODULE_DESCRIPTION("CPCAP USB detection driver");
MODULE_LICENSE("GPL");
