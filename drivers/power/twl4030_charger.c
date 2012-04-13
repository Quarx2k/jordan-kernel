/*
 * TWL4030/TPS65950 BCI (Battery Charger Interface) driver
 *
 * Copyright (C) 2010 Gražvydas Ignotas <notasas@gmail.com>
 *
 * based on twl4030_bci_battery.c by TI
 * Copyright (C) 2008 Texas Instruments, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c/twl.h>
#include <linux/power_supply.h>
#include <linux/notifier.h>
#include <linux/usb/otg.h>
#include <linux/i2c/twl4030-madc.h>


/* charger constants */
#define NO_PW_CONN              0
#define AC_PW_CONN              0x01
#define USB_PW_CONN             0x02

#define T2_BATTERY_VOLT         0x04
#define T2_BATTERY_TEMP         0x06
#define T2_BATTERY_CUR          0x08

#define TWL4030_BCIMSTATEC	0x02
#define TWL4030_BCIICHG		0x08
#define TWL4030_BCIVAC		0x0a
#define TWL4030_BCIVBUS		0x0c
#define TWL4030_BCIMFSTS4	0x10
#define TWL4030_BCICTL1		0x23

#define TWL4030_BCIAUTOWEN	BIT(5)
#define TWL4030_CONFIG_DONE	BIT(4)
#define TWL4030_BCIAUTOUSB	BIT(1)
#define TWL4030_BCIAUTOAC	BIT(0)
#define TWL4030_CGAIN		BIT(5)
#define TWL4030_USBFASTMCHG	BIT(2)
#define TWL4030_STS_VBUS	BIT(7)
#define TWL4030_STS_USB_ID	BIT(2)

/* TWL4030_MODULE_USB */
#define REG_POWER_CTRL          0x0AC
#define OTG_EN                  0x020
#define REG_PHY_CLK_CTRL        0x0FE
#define REG_PHY_CLK_CTRL_STS    0x0FF
#define PHY_DPLL_CLK            0x01

#define REG_BCICTL1             0x023
#define REG_BCICTL2             0x024
#define CGAIN                   0x020
#define ITHEN                   0x010
#define ITHSENS                 0x007

/* BCI interrupts */
#define TWL4030_WOVF		BIT(0) /* Watchdog overflow */
#define TWL4030_TMOVF		BIT(1) /* Timer overflow */
#define TWL4030_ICHGHIGH	BIT(2) /* Battery charge current high */
#define TWL4030_ICHGLOW		BIT(3) /* Battery cc. low / FSM state change */
#define TWL4030_ICHGEOC		BIT(4) /* Battery current end-of-charge */
#define TWL4030_TBATOR2		BIT(5) /* Battery temperature out of range 2 */
#define TWL4030_TBATOR1		BIT(6) /* Battery temperature out of range 1 */
#define TWL4030_BATSTS		BIT(7) /* Battery status */

#define TWL4030_VBATLVL		BIT(0) /* VBAT level */
#define TWL4030_VBATOV		BIT(1) /* VBAT overvoltage */
#define TWL4030_VBUSOV		BIT(2) /* VBUS overvoltage */
#define TWL4030_ACCHGOV		BIT(3) /* Ac charger overvoltage */

#define TWL4030_MSTATEC_USB		BIT(4)
#define TWL4030_MSTATEC_AC		BIT(5)
#define TWL4030_MSTATEC_MASK		0x0f
#define TWL4030_MSTATEC_QUICK1		0x02
#define TWL4030_MSTATEC_QUICK7		0x07
#define TWL4030_MSTATEC_COMPLETE1	0x0b
#define TWL4030_MSTATEC_COMPLETE4	0x0e

 /* Interrupt masks for BCIIMR1 */
#define BATSTS_IMR1             0x080
#define VBATLVL_IMR1            0x001

/* Interrupt mask registers for int1*/
#define REG_BCIIMR1A            0x002
#define REG_BCIIMR2A            0x003

/* Interrupt edge detection register */
#define REG_BCIEDR1             0x00A
#define REG_BCIEDR2             0x00B
#define REG_BCIEDR3             0x00C


/* BCIEDR2 */
#define BATSTS_EDRRISIN         0x080
#define BATSTS_EDRFALLING       0x040

/* BCIEDR3 */
#define VBATLVL_EDRRISIN        0x02

/* Boot BCI register */
#define REG_BOOT_BCI            0x007
#define REG_CTRL1               0x00
#define REG_SW1SELECT_MSB       0x07
#define SW1_CH9_SEL             0x02
#define REG_CTRL_SW1            0x012
#define SW1_TRIGGER             0x020
#define EOC_SW1                 0x002
#define REG_GPCH9               0x049
#define REG_STS_HW_CONDITIONS   0x0F
#define STS_VBUS                0x080
#define STS_CHG                 0x02
#define REG_BCIMSTATEC          0x02
#define REG_BCIMFSTS4           0x010
#define REG_BCIMFSTS2           0x00E
#define REG_BCIMFSTS3           0x00F
#define REG_BCIMFSTS1           0x001
#define USBFASTMCHG             0x004
#define BATSTSPCHG              0x004
#define BATSTSMCHG              0x040
#define VBATOV4                 0x020
#define VBATOV3                 0x010
#define VBATOV2                 0x008
#define VBATOV1                 0x004
#define REG_BB_CFG              0x012
#define BBCHEN                  0x010

#define AC_STATEC               0x20
#define BCIMSTAT_MASK           0x03F

/* Step size and prescaler ratio */
#define TEMP_STEP_SIZE          147
#define TEMP_PSR_R              100

#define VOLT_STEP_SIZE          588
#define VOLT_PSR_R              100

#define CURR_STEP_SIZE          147
#define CURR_PSR_R1             44

#define BK_VOLT_STEP_SIZE       441
#define BK_VOLT_PSR_R           100


static bool allow_usb = 1;
module_param(allow_usb, bool, 1);
MODULE_PARM_DESC(allow_usb, "Allow USB charge drawing default current");

/* Ptr to thermistor table */
int *therm_tbl;

struct twl4030_bci {
	struct device		*dev;

	int			voltage_uV;
	int			bk_voltage_uV;
	int			current_uA;
	int			temp_C;
	int			charge_rsoc;
	int			charge_status;
	int			capacity;

	struct power_supply	ac;
	struct power_supply	usb;
	struct power_supply	bat;
	struct power_supply	bk_bat;
	struct otg_transceiver	*transceiver;
	struct notifier_block	otg_nb;
	struct work_struct	work;
	int			irq_chg;
	int			irq_bci;

	unsigned long		event;
	struct delayed_work	twl4030_bci_monitor_work;
	struct delayed_work	twl4030_bk_bci_monitor_work;
};

static int usb_charger_flag;

static inline int clear_n_set(u8 mod_no, u8 clear, u8 set, u8 reg);
/*
 * clear and set bits on an given register on a given module
 */
static int twl4030_clear_set(u8 mod_no, u8 clear, u8 set, u8 reg)
{
	u8 val = 0;
	int ret;

	ret = twl_i2c_read_u8(mod_no, &val, reg);
	if (ret)
		return ret;

	val &= ~clear;
	val |= set;

	return twl_i2c_write_u8(mod_no, val, reg);
}

static int twl4030_bci_read(u8 reg, u8 *val)
{
	return twl_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, val, reg);
}

static int twl4030_clear_set_boot_bci(u8 clear, u8 set)
{
	return twl4030_clear_set(TWL4030_MODULE_PM_MASTER, 0,
			TWL4030_CONFIG_DONE | TWL4030_BCIAUTOWEN | set,
			TWL4030_PM_MASTER_BOOT_BCI);
}

static int twl4030bci_read_adc_val(u8 reg)
{
	int ret, temp;
	u8 val;

	/* read MSB */
	ret = twl4030_bci_read(reg + 1, &val);
	if (ret)
		return ret;

	temp = (int)(val & 0x03) << 8;

	/* read LSB */
	ret = twl4030_bci_read(reg, &val);
	if (ret)
		return ret;

	return temp | val;
}

/*
 * Check if VBUS power is present
 */
static int twl4030_bci_have_vbus(struct twl4030_bci *bci)
{
	int ret;
	u8 hwsts;

	ret = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &hwsts,
			      TWL4030_PM_MASTER_STS_HW_CONDITIONS);
	if (ret < 0)
		return 0;

	dev_dbg(bci->dev, "check_vbus: HW_CONDITIONS %02x\n", hwsts);

	/* in case we also have STS_USB_ID, VBUS is driven by TWL itself */
	if ((hwsts & TWL4030_STS_VBUS) && !(hwsts & TWL4030_STS_USB_ID))
		return 1;

	return 0;
}

/*
 * Returns an integer value, that means,
 * NO_PW_CONN  no power supply is connected
 * AC_PW_CONN  if the AC power supply is connected
 * USB_PW_CONN  if the USB power supply is connected
 * AC_PW_CONN + USB_PW_CONN if USB and AC power supplies are both connected
 *
 * Or < 0 on failure.
 */
static int twl4030charger_presence(void)
{
	int ret = 0;
	u8 hwsts = 0;

	ret = twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &hwsts,
		REG_STS_HW_CONDITIONS);
	if (ret) {
		pr_err("twl4030_bci: error reading STS_HW_CONDITIONS\n");
		return ret;
	}

	ret = (hwsts & STS_CHG) ? AC_PW_CONN : NO_PW_CONN;
	ret += (hwsts & STS_VBUS) ? USB_PW_CONN : NO_PW_CONN;

	if (ret & USB_PW_CONN)
		usb_charger_flag = 1;
	else
		usb_charger_flag = 0;

	return ret;

}


/*
 * Enable/Disable USB Charge funtionality.
 */
static int twl4030_charger_enable_usb(struct twl4030_bci *bci, bool enable)
{
	int ret;

	if (enable) {
		/* Check for USB charger conneted */
		if (!twl4030_bci_have_vbus(bci))
			return -ENODEV;

		ret = twl4030charger_presence();
		if (ret < 0)
			return ret;

		/*
		 * Until we can find out what current the device can provide,
		 * require a module param to enable USB charging.
		 */
		if (!allow_usb) {
			dev_warn(bci->dev, "USB charging is disabled.\n");
			return -EACCES;
		}

		/* forcing the field BCIAUTOUSB (BOOT_BCI[1]) to 1 */
		ret = twl4030_clear_set_boot_bci(0, TWL4030_BCIAUTOUSB);
		if (ret < 0)
			return ret;

		/* forcing USBFASTMCHG(BCIMFSTS4[2]) to 1 */
		ret = twl4030_clear_set(TWL4030_MODULE_MAIN_CHARGE, 0,
			TWL4030_USBFASTMCHG, TWL4030_BCIMFSTS4);
	} else {
		ret = twl4030charger_presence();
		ret = twl4030_clear_set_boot_bci(TWL4030_BCIAUTOUSB, 0);
	}

	return ret;
}

/*
 * Enable/Disable hardware battery level event notifications.
 */
static int twl4030battery_hw_level_en(int enable)
{
	int ret;

	if (enable) {
		/* unmask VBATOV interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, VBATLVL_IMR1,
			0, REG_BCIIMR2A);
		if (ret)
			return ret;

		/* configuring interrupt edge detection for VBATOv */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			VBATLVL_EDRRISIN, REG_BCIEDR3);
		if (ret)
			return ret;
	} else {
		/* mask VBATOV interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			VBATLVL_IMR1, REG_BCIIMR2A);
		if (ret)
			return ret;
	}

	return 0;
}

/*
 * Enable/disable hardware battery presence event notifications.
 */
static int twl4030battery_hw_presence_en(int enable)
{
	int ret;

	if (enable) {
		/* unmask BATSTS interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, BATSTS_IMR1,
			0, REG_BCIIMR1A);
		if (ret)
			return ret;

		/* configuring interrupt edge for BATSTS */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			BATSTS_EDRRISIN | BATSTS_EDRFALLING, REG_BCIEDR2);
		if (ret)
			return ret;
	} else {
		/* mask BATSTS interrupt for INT1 */
		ret = clear_n_set(TWL4030_MODULE_INTERRUPTS, 0,
			BATSTS_IMR1, REG_BCIIMR1A);
		if (ret)
			return ret;
	}

	return 0;
}



/*
 * Enable/Disable AC Charge funtionality.
 */
static int twl4030_charger_enable_ac(bool enable)
{
	int ret;

	if (enable)
		ret = twl4030_clear_set_boot_bci(0, TWL4030_BCIAUTOAC);
	else
		ret = twl4030_clear_set_boot_bci(TWL4030_BCIAUTOAC, 0);

	return ret;
}

/*
 * TWL4030 CHG_PRES (AC charger presence) events
 */
static irqreturn_t twl4030_charger_interrupt(int irq, void *arg)
{
	struct twl4030_bci *bci = arg;

	dev_dbg(bci->dev, "CHG_PRES irq\n");
	power_supply_changed(&bci->ac);
	power_supply_changed(&bci->usb);
	return IRQ_HANDLED;
}

/*
 * TWL4030 BCI monitoring events
 */
static irqreturn_t twl4030_bci_interrupt(int irq, void *arg)
{
	struct twl4030_bci *bci = arg;
	u8 irqs1, irqs2;
	int ret;

	ret = twl_i2c_read_u8(TWL4030_MODULE_INTERRUPTS, &irqs1,
			TWL4030_INTERRUPTS_BCIISR1A);
	if (ret < 0)
		return IRQ_HANDLED;

	ret = twl_i2c_read_u8(TWL4030_MODULE_INTERRUPTS, &irqs2,
			TWL4030_INTERRUPTS_BCIISR2A);
	if (ret < 0)
		return IRQ_HANDLED;

	dev_dbg(bci->dev, "BCI irq %02x %02x\n", irqs2, irqs1);

	if (irqs1 & (TWL4030_ICHGLOW | TWL4030_ICHGEOC)) {
		/* charger state change, inform the core */
		power_supply_changed(&bci->ac);
		power_supply_changed(&bci->usb);
	}

	if (irqs1 & TWL4030_BATSTS)
		dev_crit(bci->dev, "battery disconnected\n");

	if (irqs2 & TWL4030_VBATOV)
		dev_crit(bci->dev, "VBAT overvoltage\n");

	if (irqs2 & TWL4030_VBUSOV)
		dev_crit(bci->dev, "VBUS overvoltage\n");

	if (irqs2 & TWL4030_ACCHGOV)
		dev_crit(bci->dev, "Ac charger overvoltage\n");

	return IRQ_HANDLED;
}

static void twl4030_bci_usb_work(struct work_struct *data)
{
	struct twl4030_bci *bci = container_of(data, struct twl4030_bci, work);

	switch (bci->event) {
	case USB_EVENT_VBUS:
	case USB_EVENT_CHARGER:
		twl4030_charger_enable_usb(bci, true);
		break;
	case USB_EVENT_NONE:
		twl4030_charger_enable_usb(bci, false);
		break;
	}
}

static int twl4030_bci_usb_ncb(struct notifier_block *nb, unsigned long val,
			       void *priv)
{
	struct twl4030_bci *bci = container_of(nb, struct twl4030_bci, otg_nb);

	dev_dbg(bci->dev, "OTG notify %lu\n", val);

	bci->event = val;
	schedule_work(&bci->work);

	return NOTIFY_OK;
}

/*
 * TI provided formulas:
 * CGAIN == 0: ICHG = (BCIICHG * 1.7) / (2^10 - 1) - 0.85
 * CGAIN == 1: ICHG = (BCIICHG * 3.4) / (2^10 - 1) - 1.7
 * Here we use integer approximation of:
 * CGAIN == 0: val * 1.6618 - 0.85
 * CGAIN == 1: (val * 1.6618 - 0.85) * 2
 */
static int twl4030_charger_get_current(void)
{
	int curr;
	int ret;
	u8 bcictl1;

	curr = twl4030bci_read_adc_val(TWL4030_BCIICHG);
	if (curr < 0)
		return curr;

	ret = twl4030_bci_read(TWL4030_BCICTL1, &bcictl1);
	if (ret)
		return ret;

	ret = (curr * 16618 - 850 * 10000) / 10;
	if (bcictl1 & TWL4030_CGAIN)
		ret *= 2;

	return ret;
}

/*
 * Returns the main charge FSM state
 * Or < 0 on failure.
 */
static int twl4030bci_state(struct twl4030_bci *bci)
{
	int ret;
	u8 state;
	ret = twl4030_bci_read(TWL4030_BCIMSTATEC, &state);
	if (ret) {
		pr_err("twl4030_bci: error reading BCIMSTATEC\n");
		return ret;
	}

	dev_dbg(bci->dev, "state: %02x\n", state);
	return state;
}

static int twl4030_bci_state_to_status(int state)
{
	state &= TWL4030_MSTATEC_MASK;
	if (TWL4030_MSTATEC_QUICK1 <= state && state <= TWL4030_MSTATEC_QUICK7)
		return POWER_SUPPLY_STATUS_CHARGING;
	else if (TWL4030_MSTATEC_COMPLETE1 <= state &&
					state <= TWL4030_MSTATEC_COMPLETE4)
		return POWER_SUPPLY_STATUS_FULL;
	else
		return POWER_SUPPLY_STATUS_NOT_CHARGING;
}

static int twl4030_bci_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct twl4030_bci *bci = dev_get_drvdata(psy->dev->parent);
	int is_charging;
	int state;
	int ret;

	state = twl4030bci_state(bci);
	if (state < 0)
		return state;

	if (psy->type == POWER_SUPPLY_TYPE_USB) {
		is_charging = state & TWL4030_MSTATEC_USB;
	} else {
		is_charging = state & TWL4030_MSTATEC_AC;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		if (is_charging)
			val->intval = twl4030_bci_state_to_status(state);
		else
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		/* charging must be active for meaningful result */
		if (!is_charging)
			return -ENODATA;
		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			ret = twl4030bci_read_adc_val(TWL4030_BCIVBUS);
			if (ret < 0)
				return ret;
			/* BCIVBUS uses ADCIN8, 7/1023 V/step */
			val->intval = ret * 6843;
		} else {
			ret = twl4030bci_read_adc_val(TWL4030_BCIVAC);
			if (ret < 0)
				return ret;
			/* BCIVAC uses ADCIN11, 10/1023 V/step */
			val->intval = ret * 9775;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		if (!is_charging)
			return -ENODATA;
		/* current measurement is shared between AC and USB */
		ret = twl4030_charger_get_current();
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = is_charging &&
			twl4030_bci_state_to_status(state) !=
				POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/*
 * Sets and clears bits on an given register on a given module
 */
static inline int clear_n_set(u8 mod_no, u8 clear, u8 set, u8 reg)
{
	int ret;
	u8 val = 0;

	/* Gets the initial register value */
	ret = twl_i2c_read_u8(mod_no, &val, reg);
	if (ret)
		return ret;

	/* Clearing all those bits to clear */
	val &= ~(clear);

	/* Setting all those bits to set */
	val |= set;

	/* Update the register */
	ret = twl_i2c_write_u8(mod_no, val, reg);
	if (ret)
		return ret;

	return 0;
}


static enum power_supply_property twl4030_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
};

static enum power_supply_property twl4030_bci_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
};

static enum power_supply_property twl4030_bk_bci_battery_props[] = {
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
};


/*
 * Returns the main charge FSM status
 * Or < 0 on failure.
 */
static int twl4030bci_status(void)
{
	int ret;
	u8 status = 0;

	ret = twl_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE,
		&status, REG_BCIMSTATEC);
	if (ret) {
		pr_err("twl4030_bci: error reading BCIMSTATEC\n");
		return ret;
	}

	return (int) (status & BCIMSTAT_MASK);
}

static int read_bci_val(u8 reg)
{
	int ret, temp;
	u8 val = 0;

	/* reading MSB */
	ret = twl_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		reg + 1);
	if (ret)
		return ret;

	temp = ((int)(val & 0x03)) << 8;

	/* reading LSB */
	ret = twl_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		reg);
	if (ret)
		return ret;

	return temp | val;
}


/*
 * Settup the twl4030 BCI module to measure battery
 * temperature
 */
static int twl4030battery_temp_setup(void)
{
	int ret;

	/* Enabling thermistor current */
	ret = clear_n_set(TWL4030_MODULE_MAIN_CHARGE, 0, ITHEN,
		REG_BCICTL1);
	if (ret)
		return ret;

	return 0;
}

/*
 * Settup the twl4030 BCI module to enable backup
 * battery charging.
 */
static int twl4030backupbatt_voltage_setup(void)
{
	int ret;

	/* Starting backup batery charge */
	ret = clear_n_set(TWL4030_MODULE_PM_RECEIVER, 0, BBCHEN,
		REG_BB_CFG);
	if (ret)
		return ret;

	return 0;
}

/*
 * Return battery voltage
 * Or < 0 on failure.
 */
static int twl4030battery_voltage(void)
{
	int volt;
	u8 hwsts;
	struct twl4030_madc_request req;

	twl_i2c_read_u8(TWL4030_MODULE_PM_MASTER, &hwsts,
		REG_STS_HW_CONDITIONS);

	if ((hwsts & STS_CHG) || (hwsts & STS_VBUS)) {
		/* AC or USB charger connected
		 * BCI Module requests MADC for info about BTEM,VBUS,ICHG,VCHG
		 * every 50ms. This info is made available through BCI reg
		 */
		volt = read_bci_val(T2_BATTERY_VOLT);
		return (volt * VOLT_STEP_SIZE) / VOLT_PSR_R;
	} else {
		/* No charger present.
		 * BCI registers is not automatically updated.
		 * Request MADC for information - 'SW1 software conversion req'
		 */
		req.channels = (1 << 12);
		req.do_avg = 0;
		req.method = TWL4030_MADC_SW1;
		req.active = 0;
		req.func_cb = NULL;
		twl4030_madc_conversion(&req);
		volt = (u16)req.rbuf[12];
		return volt;
	}
}


/*
 * Return the battery current
 * Or < 0 on failure.
 */
static int twl4030battery_current(void)
{
	int ret, curr = read_bci_val(T2_BATTERY_CUR);
	u8 val = 0;

	ret = twl_i2c_read_u8(TWL4030_MODULE_MAIN_CHARGE, &val,
		REG_BCICTL1);
	if (ret)
		return ret;

	if (val & CGAIN) /* slope of 0.44 mV/mA */
		return (curr * CURR_STEP_SIZE) / CURR_PSR_R1;
	else /* slope of 0.88 mV/mA */
		return (curr * CURR_STEP_SIZE) / CURR_PSR_R2;
}


/*
 * Return battery capacity
 * Or < 0 on failure.
 */
static int twl4030battery_capacity(struct twl4030_bci *di)
{
	int ret = 0;
	/*
	 * need to get the correct percentage value per the
	 * battery characteristics. Approx values for now.
	 */
	if (di->voltage_uV < 3230)
		ret = 5;
	else if ((di->voltage_uV < 3270 && di->voltage_uV > 3230))
		ret = 10;
	else if ((di->voltage_uV < 3340 && di->voltage_uV > 3270))
		ret = 20;
	else if ((di->voltage_uV < 3440 && di->voltage_uV > 3340))
		ret = 35;
	else if ((di->voltage_uV < 3550 && di->voltage_uV > 3440))
		ret = 50;
	else if ((di->voltage_uV < 3650 && di->voltage_uV > 3550))
		ret = 60;
	else if ((di->voltage_uV < 3700 && di->voltage_uV > 3650))
		ret = 65;
	else if ((di->voltage_uV < 3750 && di->voltage_uV > 3700))
		ret = 70;
	else if ((di->voltage_uV < 3790 && di->voltage_uV > 3750))
		ret = 75;
	else if ((di->voltage_uV < 3830 && di->voltage_uV > 3790))
		ret = 80;
	else if (di->voltage_uV > 3830)
		ret = 90;

	return ret;
}
/*
 * Return the battery backup voltage
 * Or < 0 on failure.
 */
static int twl4030backupbatt_voltage(void)
{
	struct twl4030_madc_request req;
	int temp;

	req.channels = (1 << 9);
	req.do_avg = 0;
	req.method = TWL4030_MADC_SW1;
	req.active = 0;
	req.func_cb = NULL;
	twl4030_madc_conversion(&req);
	temp = (u16)req.rbuf[9];

	return  (temp * BK_VOLT_STEP_SIZE) / BK_VOLT_PSR_R;
}


static void
twl4030_bk_bci_battery_read_status(struct twl4030_bci *di)
{
	di->bk_voltage_uV = twl4030backupbatt_voltage();
}


static void twl4030_bk_bci_battery_work(struct work_struct *work)
{
	struct twl4030_bci *di = container_of(work,
		struct twl4030_bci,
		twl4030_bk_bci_monitor_work.work);

	twl4030_bk_bci_battery_read_status(di);
	schedule_delayed_work(&di->twl4030_bk_bci_monitor_work, 500);
}


static void twl4030_bci_battery_read_status(struct twl4030_bci *di)
{
	di->voltage_uV = twl4030battery_voltage();
	di->current_uA = twl4030_charger_get_current();
	di->capacity = twl4030battery_capacity(di);
}

static void twl4030_bci_battery_update_status(struct twl4030_bci *di)
{
	int old_charge_source = di->charge_rsoc;
	int old_charge_status = di->charge_status;
	int old_capacity = di->capacity;
	static int stable_count;

	twl4030_bci_battery_read_status(di);
	di->charge_status = POWER_SUPPLY_STATUS_UNKNOWN;
	if (power_supply_am_i_supplied(&di->bat))
		di->charge_status = POWER_SUPPLY_STATUS_CHARGING;
	else
		di->charge_status = POWER_SUPPLY_STATUS_DISCHARGING;
	/*
	 * Since Charger interrupt only happens for AC plug-in
	 * and not for usb plug-in, we use the next update
	 * cycle to update the status of the power_supply
	 * change to user space.
	 * Read the current usb_charger_flag
	 * compare this with the value from the last
	 * update cycle to determine if there was a
	 * change.
	 */

	di->charge_rsoc = usb_charger_flag;

	/*
	 * Battery voltage fluctuates, when we are on a threshold
	 * level, we do not want to keep informing user of the
	 * capacity fluctuations, or he the battery progress will
	 * keep moving.
	 */
	if (old_capacity != di->capacity)
		stable_count = 0;
	else
		stable_count++;
	/*
	 * Send uevent to user space to notify
	 * of some battery specific events.
	 * Ac plugged in, USB plugged in and Capacity
	 * level changed.
	 * called every 100 jiffies = 0.7 seconds
	 * 20 stable cycles means capacity did not change
	 * in the last 15 seconds.
	 */
	if ((old_charge_status != di->charge_status)
			|| (stable_count == 20)
			|| (old_charge_source !=  di->charge_rsoc)) {
		power_supply_changed(&di->bat);
	}
}



static void twl4030_bci_battery_work(struct work_struct *work)
{
	struct twl4030_bci *di = container_of(work,
			struct twl4030_bci, twl4030_bci_monitor_work.work);

	twl4030_bci_battery_update_status(di);
	schedule_delayed_work(&di->twl4030_bci_monitor_work, 100);
}


#define to_twl4030_bk_bci_device_info(x) container_of((x), \
		struct twl4030_bci, bk_bat);

static int twl4030_bk_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl4030_bci *di = to_twl4030_bk_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->bk_voltage_uV;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


#define to_twl4030_bci_device_info(x) container_of((x), \
			struct twl4030_bci, bat);


static void twl4030_bci_battery_external_power_changed
				(struct power_supply *psy)
{
	struct twl4030_bci *di = to_twl4030_bci_device_info(psy);

	cancel_delayed_work(&di->twl4030_bci_monitor_work);
	schedule_delayed_work(&di->twl4030_bci_monitor_work, 0);
}


static int twl4030_bci_battery_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct twl4030_bci *di;
	int status = 0;

	di = to_twl4030_bci_device_info(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = di->charge_status;
		return 0;
	default:
		break;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = di->voltage_uV;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = di->current_uA;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		val->intval = di->temp_C * 10;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		status = twl4030bci_status();
		if ((status & AC_STATEC) == AC_STATEC)
			val->intval = POWER_SUPPLY_TYPE_MAINS;
		else if (usb_charger_flag)
			val->intval = POWER_SUPPLY_TYPE_USB;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = twl4030battery_capacity(di);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}


static char *twl4030_bci_supplied_to[] = {
	"twl4030_battery",
};

static int __init twl4030_bci_probe(struct platform_device *pdev)
{
	struct twl4030_bci_platform_data *pdata = pdev->dev.platform_data;
	struct twl4030_bci *bci;
	int ret;
	int reg;

	therm_tbl = pdata->battery_tmp_tbl;

	bci = kzalloc(sizeof(*bci), GFP_KERNEL);
	if (bci == NULL)
		return -ENOMEM;

	bci->dev = &pdev->dev;
	bci->irq_chg = platform_get_irq(pdev, 0);
	bci->irq_bci = platform_get_irq(pdev, 1);

	platform_set_drvdata(pdev, bci);

	bci->ac.name = "twl4030_ac";
	bci->ac.type = POWER_SUPPLY_TYPE_MAINS;
	bci->ac.properties = twl4030_charger_props;
	bci->ac.num_properties = ARRAY_SIZE(twl4030_charger_props);
	bci->ac.get_property = twl4030_bci_get_property;

	ret = power_supply_register(&pdev->dev, &bci->ac);
	if (ret) {
		dev_err(&pdev->dev, "failed to register ac: %d\n", ret);
		goto fail_register_ac;
	}

	bci->usb.name = "twl4030_usb";
	bci->usb.type = POWER_SUPPLY_TYPE_USB;
	bci->usb.properties = twl4030_charger_props;
	bci->usb.num_properties = ARRAY_SIZE(twl4030_charger_props);
	bci->usb.get_property = twl4030_bci_get_property;


	ret = power_supply_register(&pdev->dev, &bci->usb);
	if (ret) {
		dev_err(&pdev->dev, "failed to register usb: %d\n", ret);
		goto fail_register_usb;
	}

	bci->bat.name = "twl4030_battery";
	bci->bat.supplied_to = twl4030_bci_supplied_to;
	bci->bat.num_supplicants = ARRAY_SIZE(twl4030_bci_supplied_to);
	bci->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	bci->bat.properties = twl4030_bci_battery_props;
	bci->bat.num_properties = ARRAY_SIZE(twl4030_bci_battery_props);
	bci->bat.get_property = twl4030_bci_battery_get_property;
	bci->bat.external_power_changed =
			twl4030_bci_battery_external_power_changed;

	bci->bk_bat.name = "twl4030_bk_battery";
	bci->bk_bat.type = POWER_SUPPLY_TYPE_BATTERY;
	bci->bk_bat.properties = twl4030_bk_bci_battery_props;
	bci->bk_bat.num_properties = ARRAY_SIZE(twl4030_bk_bci_battery_props);
	bci->bk_bat.get_property = twl4030_bk_bci_battery_get_property;
	bci->bk_bat.external_power_changed = NULL;


	/* settings for temperature sensing */
	ret = twl4030battery_temp_setup();
	if (ret)
		goto fail_temp_setup;

	/* enabling GPCH09 for read back battery voltage */
	ret = twl4030backupbatt_voltage_setup();
	if (ret)
		goto fail_voltage_setup;

	ret = request_threaded_irq(bci->irq_chg, NULL,
			twl4030_charger_interrupt, 0, pdev->name, bci);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not request irq %d, status %d\n",
			bci->irq_chg, ret);
		goto fail_chg_irq;
	}

	ret = request_threaded_irq(bci->irq_bci, NULL,
			twl4030_bci_interrupt, 0, pdev->name, bci);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not request irq %d, status %d\n",
			bci->irq_bci, ret);
		goto fail_bci_irq;
	}

	ret = power_supply_register(&pdev->dev, &bci->bat);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register main battery\n");
		goto fail_register_batt;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&bci->twl4030_bci_monitor_work,
					twl4030_bci_battery_work);
	schedule_delayed_work(&bci->twl4030_bci_monitor_work, 0);

	ret = power_supply_register(&pdev->dev, &bci->bk_bat);
	if (ret) {
		dev_dbg(&pdev->dev, "failed to register backup battery\n");
		goto fail_register_bk_batt;
	}

	INIT_DELAYED_WORK_DEFERRABLE(&bci->twl4030_bk_bci_monitor_work,
				twl4030_bk_bci_battery_work);
	schedule_delayed_work(&bci->twl4030_bk_bci_monitor_work, 500);

	INIT_WORK(&bci->work, twl4030_bci_usb_work);

	bci->transceiver = otg_get_transceiver();
	if (bci->transceiver != NULL) {
		bci->otg_nb.notifier_call = twl4030_bci_usb_ncb;
		otg_register_notifier(bci->transceiver, &bci->otg_nb);
	}

	/* Enable interrupts now. */
	reg = ~(TWL4030_ICHGLOW | TWL4030_ICHGEOC | TWL4030_TBATOR2 |
			TWL4030_TBATOR1 | TWL4030_BATSTS);
	ret = twl_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, reg,
				TWL4030_INTERRUPTS_BCIIMR1A);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to unmask interrupts: %d\n", ret);
		goto fail_unmask_interrupts;
	}

	reg = ~(TWL4030_VBATOV | TWL4030_VBUSOV | TWL4030_ACCHGOV);
	ret = twl_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, reg,
				TWL4030_INTERRUPTS_BCIIMR2A);
	if (ret < 0)
		dev_warn(&pdev->dev, "failed to unmask interrupts: %d\n", ret);

	twl4030_charger_enable_ac(true);
	twl4030_charger_enable_usb(bci, true);
	twl4030battery_hw_level_en(true);
	twl4030battery_hw_presence_en(true);

	return 0;

fail_unmask_interrupts:
	if (bci->transceiver != NULL) {
		otg_unregister_notifier(bci->transceiver, &bci->otg_nb);
		otg_put_transceiver(bci->transceiver);
	}
	free_irq(bci->irq_bci, bci);
fail_bci_irq:
	free_irq(bci->irq_chg, bci);
fail_chg_irq:
	power_supply_unregister(&bci->usb);
fail_register_bk_batt:
	power_supply_unregister(&bci->bk_bat);
fail_register_batt:
	power_supply_unregister(&bci->bat);
fail_register_usb:
	power_supply_unregister(&bci->ac);
fail_register_ac:
	platform_set_drvdata(pdev, NULL);
fail_voltage_setup:
fail_temp_setup:
	twl4030_charger_enable_ac(false);
	twl4030_charger_enable_usb(bci, false);
	twl4030battery_hw_level_en(false);
	twl4030battery_hw_presence_en(false);
	kfree(bci);

	return ret;
}

static int __exit twl4030_bci_remove(struct platform_device *pdev)
{
	struct twl4030_bci *bci = platform_get_drvdata(pdev);

	twl4030_charger_enable_ac(false);
	twl4030_charger_enable_usb(bci, false);
	twl4030battery_hw_level_en(false);
	twl4030battery_hw_presence_en(false);

	/* mask interrupts */
	twl_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, 0xff,
			TWL4030_INTERRUPTS_BCIIMR1A);
	twl_i2c_write_u8(TWL4030_MODULE_INTERRUPTS, 0xff,
			TWL4030_INTERRUPTS_BCIIMR2A);

	if (bci->transceiver != NULL) {
		otg_unregister_notifier(bci->transceiver, &bci->otg_nb);
		otg_put_transceiver(bci->transceiver);
	}
	free_irq(bci->irq_bci, bci);
	free_irq(bci->irq_chg, bci);
	power_supply_unregister(&bci->usb);
	power_supply_unregister(&bci->ac);
	power_supply_unregister(&bci->bat);
	power_supply_unregister(&bci->bk_bat);
	platform_set_drvdata(pdev, NULL);
	kfree(bci);

	return 0;
}

static struct platform_driver twl4030_bci_driver = {
	.driver	= {
		.name	= "twl4030_bci",
		.owner	= THIS_MODULE,
	},
	.remove	= __exit_p(twl4030_bci_remove),
};

static int __init twl4030_bci_init(void)
{
	return platform_driver_probe(&twl4030_bci_driver, twl4030_bci_probe);
}
module_init(twl4030_bci_init);

static void __exit twl4030_bci_exit(void)
{
	platform_driver_unregister(&twl4030_bci_driver);
}
module_exit(twl4030_bci_exit);

MODULE_AUTHOR("Gražydas Ignotas");
MODULE_DESCRIPTION("TWL4030 Battery Charger Interface driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:twl4030_bci");
