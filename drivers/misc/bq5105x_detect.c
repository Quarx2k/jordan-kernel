/*
 * Copyright (C) 2014 Motorola Mobility LLC
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

#include <linux/alarmtimer.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/m4sensorhub.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>
#include <linux/wakeup_source_notify.h>

/*
 * Detect when a device is placed on a wireless charger and report this to user
 * space. Use two gpio lines for the detection, chg_gpio and det_gpio. The gpios
 * are in the following states depending on the device state and h/w revision:
 *
 *    Docked   Charging   chg_gpio   det_gpio (new h/w)     det_gpio (old h/w)
 *      Y         Y         High       High                   High
 *      Y         N         Low        Periodic high pulse    High
 *      N         N/A       Low        Low                    High
 *
 * For new h/w, if charging is stopped while the device is docked, det_gpio may
 * stay high for some time before the periodic pulse starts.
 *
 * For new h/w, the state of det_gpio alone is sufficinet to determine docked
 * state. We have to use state of chg_gpo for compatibality with old h/w.
 *
 * Provide an option to use motion to report undocking. If this option is
 * enabled, then udocking is reported only if motion has been detected.
 */

/*
 * from device tree node
 * chg_gpio, det_gpio	detection lines
 * undocked_delay	time to wait for a pulse on det_gpio line
 * chg_name		name of the wireless charger power_supply device
 * switch_name		name of the wireless charger switch device
 * supplied_to		batteries where this charger supplies power
 * num_supplicants	number of entries in the supplied_to array, the value
 *			is derived from	from the supplied_to array
 * uevent_wakelock_timeout	time to hold wake_lock to ensure user space has
 *				processed new docked state
 * old_hw		h/w where det_gpio is always high
 * use_motion		use motion to detect undocking
 */
struct bq5105x_detect_dts {
	int chg_gpio;
	int det_gpio;
	unsigned long undocked_delay;
	const char* chg_name;
	const char* switch_name;
	char **supplied_to;
	size_t num_supplicants;
	unsigned long uevent_wakelock_timeout;
	bool old_hw;
	bool use_motion;
};

struct bq5105x_detect {
	const struct bq5105x_detect_dts *dts_data;
	struct platform_device *pdev;
	bool docked;		/* based on chg_gpio and det_gpio states */
	bool reported_docked;	/* based on docked state and motion */
	unsigned int chg_irq;
	unsigned int det_irq;
	bool det_irq_enabled;
	struct power_supply charger;
	struct switch_dev *sdev;
	struct workqueue_struct *wq;
	struct work_struct chg_irq_work;
	struct work_struct det_irq_work;
	struct work_struct undocked_work;
	struct alarm undocked_alarm;
	ktime_t alarm_time;
	/* Separate wakelock for each work item */
	struct wake_lock chg_irq_wakelock;
	struct wake_lock det_irq_wakelock;
	struct wake_lock undocked_wakelock;
	/* Wakelock to allow user space process new docked state */
	struct wake_lock uevent_wakelock;
	/* Wakelock to prevent suspend while pulse is present on det_gpio */
	struct wake_lock pulse_wakelock;
	u32 disable_pulse_wakelock; /* for disabling via debugfs */
#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
#endif
	/* For using motion to detect undocking */
	struct m4sensorhub_data *m4shub;
	bool in_motion; /* has there been some motion lately ? */
	struct mutex motion_mutex;
};

static enum power_supply_property bq5105x_detect_chg_prop[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int bq5105x_detect_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct bq5105x_detect *chip = container_of(psy, struct bq5105x_detect,
						   charger);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		/* TODO: Atomic access to the state */
		val->intval = chip->reported_docked;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}


static void bq5105x_detect_m4_track_motion(struct bq5105x_detect *chip, bool en)
{
	int ret;

	if (!chip->m4shub) {
		dev_dbg(&chip->pdev->dev, "m4 sensor hub is not ready to track motion\n");
		return;
	}

	dev_dbg(&chip->pdev->dev, "motion tracking is %s\n", en ? "on" : "off");

	if (en) {
		ret = m4sensorhub_irq_enable(chip->m4shub,
					     M4SH_IRQ_MOTION_DETECTED);
		if (ret < 0) {
			dev_err(&chip->pdev->dev, "failed to enable motion detection\n");
			goto err_m4_track;
		}

		ret = m4sensorhub_irq_enable(chip->m4shub,
					     M4SH_IRQ_STILL_DETECTED);
		if (ret < 0) {
			dev_err(&chip->pdev->dev, "failed to enable still mode detection\n");
			goto err_m4_track;
		}
	} else {
		ret = m4sensorhub_irq_disable(chip->m4shub,
					      M4SH_IRQ_MOTION_DETECTED);
		if (ret < 0) {
			dev_err(&chip->pdev->dev, "failed to disable motion detection\n");
			goto err_m4_track;
		}

		ret = m4sensorhub_irq_disable(chip->m4shub,
					      M4SH_IRQ_STILL_DETECTED);
		if (ret < 0) {
			dev_err(&chip->pdev->dev, "failed to disable still mode detection\n");
			goto err_m4_track;
		}
	}

	return;

err_m4_track:
	/* Unregister from interrupts and assume to be in motion */
	chip->in_motion = true;
	m4sensorhub_irq_unregister(chip->m4shub, M4SH_IRQ_MOTION_DETECTED);
	m4sensorhub_irq_unregister(chip->m4shub, M4SH_IRQ_STILL_DETECTED);
	chip->m4shub = NULL;
}

static bool bq5105x_detect_m4_motion(struct bq5105x_detect *chip)
{
	bool motion;
	unsigned char val;
	int ret;

	/*
	 * In motion if sensor hub is not ready or error occured when reading
	 * the current state.
	 */
	if (!chip->m4shub) {
		dev_dbg(&chip->pdev->dev, "m4 sensor hub is not ready to report motion state\n");
		ret = -1;
	} else {
		ret = m4sensorhub_reg_read(chip->m4shub,
					   M4SH_REG_POWER_DEVICESTATE, &val);
	}

	motion = (ret < 0 || val);
	dev_dbg(&chip->pdev->dev, "current motion=%d\n", motion);
	return motion;
}

static void bq5105x_detect_report(struct bq5105x_detect *chip, bool docked)
{
	if (chip->reported_docked != docked) {
		dev_dbg(&chip->pdev->dev, "report docked=%d\n", docked);
#ifdef CONFIG_WAKEUP_SOURCE_NOTIFY
		wakeup_source_notify_subscriber(docked
						? DISPLAY_WAKE_EVENT_DOCKON
						: DISPLAY_WAKE_EVENT_DOCKOFF);
#endif
		chip->reported_docked = docked;
		power_supply_changed(&chip->charger);
		switch_set_state(chip->sdev, docked);
		if (chip->dts_data->use_motion) {
			/* Track motion state while docked */
			if (docked) {
				bq5105x_detect_m4_track_motion(chip, true);
				chip->in_motion =
					bq5105x_detect_m4_motion(chip);
			} else {
				bq5105x_detect_m4_track_motion(chip, false);
			}
		}
		/* TODO: Release wakelock when switch state is read */
		wake_lock_timeout(&chip->uevent_wakelock,
				  chip->dts_data->uevent_wakelock_timeout);
	}
}

static void bq5105x_detect_set_docked(struct bq5105x_detect *chip, bool docked)
{
	if (chip->docked != docked) {
		dev_dbg(&chip->pdev->dev, "docked=%d\n", docked);
		if (chip->dts_data->use_motion) {
			/* Report undocked only if there has been some motion */
			mutex_lock(&chip->motion_mutex);
			chip->docked = docked;
			if (chip->docked || chip->in_motion)
				bq5105x_detect_report(chip, docked);
			mutex_unlock(&chip->motion_mutex);
		} else {
			chip->docked = docked;
			bq5105x_detect_report(chip, docked);
		}
	}
}

static void bq5105x_detect_chg_irq_work(struct work_struct *work)
{
	struct bq5105x_detect *chip = container_of(work, struct bq5105x_detect,
						   chg_irq_work);
	/*
	 * When chg_gpio is high, declare docked and do not monitor det_gpio
	 * line. When chg_gpio is low, monitor pulses on det_gpio line after it
	 * transitions to low unless old h/w is used. Detect presense of a pulse
	 * by setting an alarm. If det_gpio goes from high to low before the
	 * alarm goes off, the pulse is present. If there is no pulse, declare
	 * un-docked.
	 */
	if (gpio_get_value_cansleep(chip->dts_data->chg_gpio)) {
		dev_dbg(&chip->pdev->dev, "charging\n");
		if (chip->det_irq_enabled) {
			disable_irq(chip->det_irq);
			chip->det_irq_enabled = false;
		}
		cancel_work_sync(&chip->det_irq_work);
		wake_unlock(&chip->pulse_wakelock);
		wake_unlock(&chip->det_irq_wakelock);
		alarm_cancel(&chip->undocked_alarm);
		cancel_work_sync(&chip->undocked_work);
		wake_unlock(&chip->undocked_wakelock);
		bq5105x_detect_set_docked(chip, true);
	} else {
		dev_dbg(&chip->pdev->dev, "not charging\n");
		if (!chip->dts_data->old_hw) {
			 /* Less power is drawn with wakelock when looking for a
			  * pulse */
			if (!chip->disable_pulse_wakelock)
				wake_lock(&chip->pulse_wakelock);

			if (!chip->det_irq_enabled) {
				enable_irq(chip->det_irq);
				chip->det_irq_enabled = true;
			}
			if (!gpio_get_value_cansleep(chip->dts_data->det_gpio))
				alarm_start_relative(&chip->undocked_alarm,
						     chip->alarm_time);
		} else {
			bq5105x_detect_set_docked(chip, false);
		}
	}

	wake_unlock(&chip->chg_irq_wakelock);
}

static irqreturn_t bq5105x_detect_chg_irq(int irq, void *devid)
{
	struct bq5105x_detect *chip = devid;
	wake_lock(&chip->chg_irq_wakelock);
	queue_work(chip->wq, &chip->chg_irq_work);
	return IRQ_HANDLED;
}

static void bq5105x_detect_det_irq_work(struct work_struct *work)
{
	struct bq5105x_detect *chip = container_of(work, struct bq5105x_detect,
						   det_irq_work);
	if (alarm_cancel(&chip->undocked_alarm)) {
		dev_dbg(&chip->pdev->dev, "pulse\n");
		bq5105x_detect_set_docked(chip, true);
	} else {
		dev_dbg(&chip->pdev->dev, "first pulse\n");
	}

	alarm_start_relative(&chip->undocked_alarm, chip->alarm_time);
	/* Less power is drawn if wakelock is held while pulse is present */
	if (!chip->disable_pulse_wakelock)
		wake_lock(&chip->pulse_wakelock);
	wake_unlock(&chip->det_irq_wakelock);
}

static irqreturn_t bq5105x_detect_det_irq(int irq, void *devid)
{
	struct bq5105x_detect *chip = devid;
	wake_lock(&chip->det_irq_wakelock);
	queue_work(chip->wq, &chip->det_irq_work);
	return IRQ_HANDLED;
}

static void bq5105x_detect_undocked_work(struct work_struct *work)
{
	struct bq5105x_detect *chip = container_of(work, struct bq5105x_detect,
						   undocked_work);
	dev_dbg(&chip->pdev->dev, "no pulse\n");
	bq5105x_detect_set_docked(chip, false);
	if (gpio_get_value_cansleep(chip->dts_data->det_gpio))
		dev_dbg(&chip->pdev->dev,
			"timer expired, but det_gpio is high\n");
	wake_unlock(&chip->pulse_wakelock);
	wake_unlock(&chip->undocked_wakelock);
}

static enum alarmtimer_restart
bq5105x_detect_undocked_alarm(struct alarm *alrm, ktime_t now)
{
	struct bq5105x_detect *chip = container_of(alrm, struct bq5105x_detect,
						   undocked_alarm);
	wake_lock(&chip->undocked_wakelock);
	queue_work(chip->wq, &chip->undocked_work);
	return ALARMTIMER_NORESTART;
}

static void
bq5105x_detect_m4_motion_changed(enum m4sensorhub_irqs int_event, void *data)
{
	struct bq5105x_detect *chip = (struct bq5105x_detect *)data;

	mutex_lock(&chip->motion_mutex);

	/* Assume in motion if enabling/disabling IRQs has previously failed */
	if (chip->m4shub)
		chip->in_motion = (int_event == M4SH_IRQ_MOTION_DETECTED);
	else
		chip->in_motion = true;

	dev_dbg(&chip->pdev->dev, "new motion=%d\n", chip->in_motion);

	/* Report undocked if in motion and undocking has been detected */
	if (chip->in_motion && !chip->docked)
		bq5105x_detect_report(chip, false);
	mutex_unlock(&chip->motion_mutex);
}

static int bq5105x_detect_m4_init(struct init_calldata *p_arg)
{
	struct bq5105x_detect *chip = (struct bq5105x_detect *) p_arg->p_data;
	int ret;

	mutex_lock(&chip->motion_mutex);
	/*
	 * Register motion and still mode detection callbacks. If docked,
	 * determine the current state and enable callbacks to detect future
	 * state changes.
	 */

	chip->m4shub = p_arg->p_m4sensorhub_data;
	if (!chip->m4shub) {
		dev_err(&chip->pdev->dev, "invalid handle to sensor hub\n");
		ret = -EINVAL;
		goto err_m4_handle;
	}

	ret = m4sensorhub_irq_register(chip->m4shub, M4SH_IRQ_MOTION_DETECTED,
				       bq5105x_detect_m4_motion_changed,
				       (void *)chip, 0);
	if (ret < 0) {
		dev_err(&chip->pdev->dev,
			"failed to register motion detection\n");
		goto err_m4_motion;
	}

	ret = m4sensorhub_irq_register(chip->m4shub, M4SH_IRQ_STILL_DETECTED,
				       bq5105x_detect_m4_motion_changed,
				       (void *)chip, 0);
	if (ret < 0) {
		dev_err(&chip->pdev->dev,
			"failed to register still mode detection\n");
		goto err_m4_still;
	}

	dev_dbg(&chip->pdev->dev, "m4 sensor hub is ready\n");

	if (chip->docked) {
		bq5105x_detect_m4_track_motion(chip, true);
		chip->in_motion = bq5105x_detect_m4_motion(chip);
	}

	mutex_unlock(&chip->motion_mutex);
	return 0;

err_m4_still:
	m4sensorhub_irq_unregister(chip->m4shub, M4SH_IRQ_MOTION_DETECTED);
err_m4_motion:
	chip->m4shub = NULL;
err_m4_handle:
	mutex_unlock(&chip->motion_mutex);
	return ret;
}

static struct bq5105x_detect_dts *
of_bq5105x_detect(struct platform_device *pdev)
{
	struct bq5105x_detect_dts *dts_data;
	struct device_node *np = pdev->dev.of_node;
	u32 val;
	int i, size, count;
	const char *string;

	if (!np) {
		dev_err(&pdev->dev, "devtree data not found\n");
		return NULL;
	}

	dts_data = devm_kzalloc(&pdev->dev, sizeof(*dts_data), GFP_KERNEL);
	if (!dts_data) {
		dev_err(&pdev->dev, "failed to alloc dts data\n");
		return NULL;
	}

	/* Required properties */
	dts_data->chg_gpio = of_get_named_gpio(np, "charge-gpio", 0);
	if (!gpio_is_valid(dts_data->chg_gpio)) {
		dev_err(&pdev->dev, "no valid charge-gpio property\n");
		return NULL;
	}

	dts_data->det_gpio = of_get_named_gpio(np, "detect-gpio", 0);
	if (!gpio_is_valid(dts_data->det_gpio)) {
		dev_err(&pdev->dev, "no valid detect-gpio property\n");
		return NULL;
	}

	if (of_property_read_u32(np, "undocked-delay,ms", &val) !=0) {
		dev_err(&pdev->dev, "no valid 'undocked-delay,ms' property\n");
		return NULL;
	}
	dts_data->undocked_delay = val;

	/* Optional properties */
	of_property_read_string(np, "charger-name", &dts_data->chg_name);
	of_property_read_string(np, "switch-name", &dts_data->switch_name);

	count = of_property_count_strings(np, "supplied_to");
	if (count > 0) {
		size = count * sizeof(*dts_data->supplied_to);
		dts_data->supplied_to = devm_kzalloc(&pdev->dev, size,
						     GFP_KERNEL);
		if (!dts_data->supplied_to) {
			dev_err(&pdev->dev, "Failed to alloc supplied_to\n");
			return NULL;
		}

		/* Make copies of the DT strings for const-correctness */
		for (i = 0; i < count; i++) {
			if (of_property_read_string_index(np, "supplied_to", i,
							  &string)) {
				dev_err(&pdev->dev, "Failed to read supplied_to"
					" supplied_to[%d]\n", i);
				goto free;
			}
			dts_data->supplied_to[i] = kstrdup(string,  GFP_KERNEL);
			if (!dts_data->supplied_to[i]) {
				dev_err(&pdev->dev, "Failed to alloc space for"
					" supplied_to[%d]\n", i);
				goto free;
			}
		}
		dts_data->num_supplicants = count;
	}

	if (of_property_read_u32(np, "uevent-wakelock-timeout,ms", &val) !=0 ) {
		dev_err(&pdev->dev,
			"no valid 'uevent-wakelock-timeout,ms' property\n");
		goto free;
	}
	dts_data->uevent_wakelock_timeout = msecs_to_jiffies(val);

	dts_data->old_hw = of_property_read_bool(np, "old-hw");

	dts_data->use_motion = of_property_read_bool(np, "use-motion");

	return dts_data;

free:
	for (i = 0; i < count; i++)
		kfree(dts_data->supplied_to[i]);
	return NULL;
}

#ifdef CONFIG_DEBUG_FS
static int bq5105x_detect_debugfs_create(struct bq5105x_detect *chip)
{
	chip->debugfs_root = debugfs_create_dir(dev_name(&chip->pdev->dev),
						NULL);
	if (!chip->debugfs_root)
		return -ENOMEM;

	if (!debugfs_create_bool("disable_pulse_wakelock", S_IRUGO | S_IWUSR,
				 chip->debugfs_root,
				 &chip->disable_pulse_wakelock))
		goto err_debugfs;

	return 0;
err_debugfs:
	debugfs_remove_recursive(chip->debugfs_root);
	chip->debugfs_root = NULL;
	return -ENOMEM;
}
#endif

static int bq5105x_detect_probe(struct platform_device *pdev)
{
	int i, ret;
	struct bq5105x_detect *chip;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "failed to alloc driver structure\n");
		return -ENOMEM;
	}

	chip->dts_data = of_bq5105x_detect(pdev);
	if (!chip->dts_data) {
		dev_err(&pdev->dev, "failed to parse devtree node\n");
		return -ENODEV;
	}

	chip->pdev = pdev;

	ret = gpio_request_one(chip->dts_data->chg_gpio,
			       GPIOF_IN | GPIOF_EXPORT, NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to request charge gpio pin: %d\n",
			ret);
		goto err_chg_gpio;
	}

	ret = gpio_request_one(chip->dts_data->det_gpio,
			       GPIOF_IN | GPIOF_EXPORT, NULL);
	if (ret) {
		dev_err(&pdev->dev, "failed to request detect gpio pin: %d\n",
			ret);
		goto err_det_gpio;
	}

	chip->charger.name = chip->dts_data->chg_name ?
					chip->dts_data->chg_name : "bq5105x";
	chip->charger.type = POWER_SUPPLY_TYPE_WIRELESS;
	chip->charger.properties = bq5105x_detect_chg_prop;
	chip->charger.num_properties = ARRAY_SIZE(bq5105x_detect_chg_prop);
	chip->charger.get_property = bq5105x_detect_get_property;
	chip->charger.supplied_to = chip->dts_data->supplied_to;
	chip->charger.num_supplicants = chip->dts_data->num_supplicants;
	ret = power_supply_register(&pdev->dev, &chip->charger);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to register power supply: %d\n",
			ret);
		goto err_pwr_supply;
	}

	if (chip->dts_data->switch_name ) {
		chip->sdev = devm_kzalloc(&pdev->dev, sizeof(*chip->sdev),
					  GFP_KERNEL);
		if (!chip->sdev) {
			dev_err(&pdev->dev, "failed to alloc switch device\n");
			ret = -ENOMEM;
			goto err_switch;
		}

		chip->sdev->name = chip->dts_data->switch_name;
		ret = switch_dev_register(chip->sdev);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register switch device:"
				" %d\n", ret);
			goto err_switch;
		}
	}

	/* Use ordered workqueue to process irqs and alarms one at a time */
	chip->wq = alloc_ordered_workqueue("bq5105x-detect", 0);
	if (!chip->wq) {
		dev_err(&pdev->dev, "failed to alloc workqueue\n");
		ret = -ENOMEM;
		goto err_workqueue;
	}

	INIT_WORK(&chip->chg_irq_work, bq5105x_detect_chg_irq_work);
	INIT_WORK(&chip->det_irq_work, bq5105x_detect_det_irq_work);
	INIT_WORK(&chip->undocked_work, bq5105x_detect_undocked_work);

	alarm_init(&chip->undocked_alarm, ALARM_BOOTTIME,
		   bq5105x_detect_undocked_alarm);
	chip->alarm_time = ns_to_ktime(chip->dts_data->undocked_delay *
				       NSEC_PER_MSEC);

	wake_lock_init(&chip->chg_irq_wakelock, WAKE_LOCK_SUSPEND, "chg-irq");
	wake_lock_init(&chip->det_irq_wakelock, WAKE_LOCK_SUSPEND, "det-irq");
	wake_lock_init(&chip->undocked_wakelock, WAKE_LOCK_SUSPEND,
		       "undocked-alarm");
	wake_lock_init(&chip->uevent_wakelock, WAKE_LOCK_SUSPEND,
		       "docked-state-uevent");
	wake_lock_init(&chip->pulse_wakelock, WAKE_LOCK_SUSPEND, "det-pulse");

	if (chip->dts_data->use_motion) {
		/* Initialize mutex before irqs are enabled and register with
		 * sensor hub to be called when it is setup and running */
		mutex_init(&chip->motion_mutex);
		ret = m4sensorhub_register_initcall(bq5105x_detect_m4_init,
						    (void *)chip);
		if (ret < 0) {
			dev_err(&pdev->dev, "failed to register with sensor hub\n");
			goto err_m4shub;
		}
	}

	/* Do not enable det_gpio interrupt on old h/w. Moreover, if the IRQ got
	 * processed on old h/w, the detection logic will be broken */
	if (!chip->dts_data->old_hw) {
		/*
		 * Request det_irq before chg_irq to set det_irq_enabled flag
		 * before it can be accessed from chg_irq work function. Use the
		 * flag to ensure correct nesteness of enabling and disabling
		 * det_irq.
		 */
		chip->det_irq = gpio_to_irq(chip->dts_data->det_gpio);
		chip->det_irq_enabled = true;
		ret = request_any_context_irq(chip->det_irq,
			bq5105x_detect_det_irq,
			IRQF_TRIGGER_FALLING, dev_name(&pdev->dev), chip);
		if (ret < 0) {
			dev_err(&pdev->dev,
				"failed to request detect gpio irq: %d\n", ret);
			goto err_det_irq;
		}
	}

	chip->chg_irq = gpio_to_irq(chip->dts_data->chg_gpio);
	ret = request_any_context_irq(chip->chg_irq, bq5105x_detect_chg_irq,
				IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
				dev_name(&pdev->dev), chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to request charge gpio irq: %d\n",
			ret);
		goto err_chg_irq;
	}

#ifdef CONFIG_DEBUG_FS
	if (bq5105x_detect_debugfs_create(chip))
		dev_warn(&pdev->dev, "cannot create debugfs\n");
#endif

	platform_set_drvdata(pdev, chip);

	/* Set initial state */
	queue_work(chip->wq, &chip->chg_irq_work);

	return 0;

err_chg_irq:
	if (!chip->dts_data->old_hw)
		free_irq(chip->det_irq, chip);
err_det_irq:
	if (chip->dts_data->use_motion)
		m4sensorhub_unregister_initcall(bq5105x_detect_m4_init);
err_m4shub:
	alarm_cancel(&chip->undocked_alarm);
	destroy_workqueue(chip->wq);
	/* Destroy mutex after workqueue since work functions use the mutex */
	if (chip->dts_data->use_motion)
		mutex_destroy(&chip->motion_mutex);
	wake_lock_destroy(&chip->chg_irq_wakelock);
	wake_lock_destroy(&chip->det_irq_wakelock);
	wake_lock_destroy(&chip->undocked_wakelock);
	wake_lock_destroy(&chip->uevent_wakelock);
	wake_lock_destroy(&chip->pulse_wakelock);
err_workqueue:
	switch_dev_unregister(chip->sdev);
err_switch:
	power_supply_unregister(&chip->charger);
err_pwr_supply:
	gpio_free(chip->dts_data->det_gpio);
err_det_gpio:
	gpio_free(chip->dts_data->chg_gpio);
err_chg_gpio:
	for (i = 0; i < chip->dts_data->num_supplicants; i++)
		kfree(chip->dts_data->supplied_to[i]);
	return ret;
}

static int bq5105x_detect_remove(struct platform_device *pdev)
{
	struct bq5105x_detect* chip = platform_get_drvdata(pdev);
	int i;
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(chip->debugfs_root);
#endif
	free_irq(chip->chg_irq, chip);
	free_irq(chip->det_irq, chip);
	alarm_cancel(&chip->undocked_alarm);
	destroy_workqueue(chip->wq);
	wake_lock_destroy(&chip->chg_irq_wakelock);
	wake_lock_destroy(&chip->det_irq_wakelock);
	wake_lock_destroy(&chip->undocked_wakelock);
	wake_lock_destroy(&chip->uevent_wakelock);
	wake_lock_destroy(&chip->pulse_wakelock);
	switch_dev_unregister(chip->sdev);
	power_supply_unregister(&chip->charger);
	gpio_free(chip->dts_data->det_gpio);
	gpio_free(chip->dts_data->chg_gpio);
	for (i = 0; i < chip->dts_data->num_supplicants; i++)
		kfree(chip->dts_data->supplied_to[i]);

	platform_set_drvdata(pdev, NULL);
	return 0;
}

static const struct of_device_id bq5105x_detect_of_match[] = {
	{ .compatible = "mmi,bq5105x-detect", },
	{ }
};
MODULE_DEVICE_TABLE(of, bq5105x_detect_of_match);

static struct platform_driver bq5105x_detect_driver = {
	.probe = bq5105x_detect_probe,
	.remove = bq5105x_detect_remove,
	.driver = {
		.name = "bq5105x-detect",
		.owner = THIS_MODULE,
		.of_match_table = bq5105x_detect_of_match,
	},
};

module_platform_driver(bq5105x_detect_driver);

MODULE_ALIAS("platform:bq5105x_detect");
MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("BQ5105x Detection Driver");
MODULE_LICENSE("GPL");
