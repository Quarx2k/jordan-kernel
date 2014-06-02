/*
 * RTC device/driver based on SensorHub
 * Copyright (C) 2014 Motorola Mobility LLC
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/rtc.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/m4sensorhub.h>
#include <linux/m4sensorhub/m4sensorhub_registers.h>
#include <linux/m4sensorhub/m4sensorhub_irqs.h>

#define SECONDS_IN_DAY (24*60*60)
#define DRIVER_NAME "rtc-sensorhub"

struct rtc_sensorhub_private_data {
	struct rtc_device         *p_rtc;
	struct m4sensorhub_data   *p_m4sensorhub_data;
	struct rtc_wkalrm         next_alarm_set;

	unsigned long   m4_seconds_cached;
	unsigned long   m4_boot_cached;
	unsigned long   sys_seconds_cached;
	unsigned long   sys_boot_cached;
};

static int rtc_sensorhub_rtc_alarm_irq_enable(struct device *p_dev,
					unsigned int enable)
{
	int err = 0;
	struct platform_device *p_platdev = to_platform_device(p_dev);
	struct rtc_sensorhub_private_data *p_priv_data =
				platform_get_drvdata(p_platdev);

	dev_dbg(p_dev, "enable is %u\n", enable);

	if (!(p_priv_data->p_m4sensorhub_data)) {
		dev_err(p_dev, "Cannot %s alarm--RTC hardware not ready yet\n",
			(enable ? "set" : "clear"));
		return -EIO;
	}

	if (enable == 1) {
		err = m4sensorhub_irq_enable(p_priv_data->p_m4sensorhub_data,
				M4SH_IRQ_AP_ALARM_EXPIRED);
		if (err < 0)
			dev_err(p_dev, "Enabling IRQ failed (%d)\n", err);
	} else {
		err = m4sensorhub_irq_disable(p_priv_data->p_m4sensorhub_data,
				M4SH_IRQ_AP_ALARM_EXPIRED);
		if (err < 0)
			dev_err(p_dev, "Disabling IRQ failed (%d)\n", err);
	}

	return err;
}

static int rtc_sensorhub_rtc_read_alarm(struct device *p_dev,
	struct rtc_wkalrm *p_alrm)
{
	struct platform_device *p_platdev = to_platform_device(p_dev);
	struct rtc_sensorhub_private_data *p_priv_data =
						platform_get_drvdata(p_platdev);
	struct rtc_time rtc = p_alrm->time;

	memcpy(p_alrm, &(p_priv_data->next_alarm_set),
			sizeof(struct rtc_wkalrm));

	dev_info(p_dev, "alarm read for %d-%02d-%02d %02d:%02d:%02d UTC\n",
		rtc.tm_year + 1900, rtc.tm_mon + 1, rtc.tm_mday,
		rtc.tm_hour, rtc.tm_min, rtc.tm_sec);

	return 0;
}

static int rtc_sensorhub_rtc_set_alarm(struct device *p_dev,
	struct rtc_wkalrm *p_alrm)
{
	struct platform_device *p_platdev = to_platform_device(p_dev);
	struct rtc_sensorhub_private_data *p_priv_data =
						platform_get_drvdata(p_platdev);
	struct m4sensorhub_data *p_m4_drvdata =
			p_priv_data->p_m4sensorhub_data;
	struct rtc_time rtc = p_alrm->time;

	struct timeval tv_current;
	unsigned long requested_time, time_diff;
	int ret;

	dev_info(p_dev, "alarm requested for %d-%02d-%02d %02d:%02d:%02d UTC\n",
		rtc.tm_year + 1900, rtc.tm_mon + 1, rtc.tm_mday,
		rtc.tm_hour, rtc.tm_min, rtc.tm_sec);

	if (!p_m4_drvdata) {
		dev_err(p_dev, "M4 not ready, ignore func call\n");
		return -EIO;
	}

	rtc_tm_to_time(&rtc, &requested_time);
	do_gettimeofday(&tv_current);

	/* make sure alarm requested is for future*/
	if (requested_time < tv_current.tv_sec) {
		dev_err(p_dev, "alarm in past, rejecting\n");
		return -EINVAL;
	}

	time_diff = requested_time - tv_current.tv_sec;
	if (time_diff >= SECONDS_IN_DAY || time_diff <= 0) {
		dev_err(p_dev,
			"requested alarm out of range, rejecting alarm\n");
		return -EINVAL;
	}

	if (m4sensorhub_reg_getsize(p_m4_drvdata, M4SH_REG_GENERAL_APALARM) !=
		m4sensorhub_reg_write(p_m4_drvdata, M4SH_REG_GENERAL_APALARM,
		(char *)&time_diff, m4sh_no_mask)) {
		dev_err(p_dev, "Failed to set M4 alarm!\n");
		return -EIO;
	}

	ret = rtc_sensorhub_rtc_alarm_irq_enable(p_dev, p_alrm->enabled);
	if (ret < 0) {
		dev_err(p_dev, "failed enabling irq for alarm\n");
		return ret;
	}

	/* Store the info abt this alarm in our local datastructure */
	memcpy(&(p_priv_data->next_alarm_set), p_alrm,
		sizeof(struct rtc_wkalrm));
	return 0;
}

static int rtc_sensorhub_get_rtc_from_m4(struct rtc_time *p_tm,
			struct m4sensorhub_data *p_m4_drvdata)
{
	u32 seconds = 0;

	if (m4sensorhub_reg_getsize(p_m4_drvdata, M4SH_REG_GENERAL_UTC) !=
		m4sensorhub_reg_read(p_m4_drvdata, M4SH_REG_GENERAL_UTC,
		(char *)&seconds)) {
		pr_err("%s: Failed to get M4 clock!\n", DRIVER_NAME);
		return -EIO;
	}

	rtc_time_to_tm(seconds, p_tm);
	return 0;
}

static int rtc_sensorhub_rtc_read_time(struct device *p_dev,
	struct rtc_time *p_tm)
{
	int err = 0;
	struct platform_device *p_platdev = to_platform_device(p_dev);
	struct rtc_sensorhub_private_data *p_priv_data =
						platform_get_drvdata(p_platdev);

	if (!(p_priv_data->p_m4sensorhub_data)) {
		if ((p_priv_data->sys_seconds_cached != 0) &&
		(p_priv_data->sys_boot_cached != 0)) {
			dev_err(p_dev,
				"Using saved set RTC time (%lu)\n",
				p_priv_data->sys_seconds_cached +
				get_seconds() - p_priv_data->sys_boot_cached);
			/* Use saved settime request (will go to M4) */
			rtc_time_to_tm(p_priv_data->sys_seconds_cached +
				get_seconds() - p_priv_data->sys_boot_cached,
				p_tm);
		} else {
			dev_err(p_dev,
				"Using cached M4 RTC time (%lu)\n",
				p_priv_data->m4_seconds_cached +
				get_seconds() - p_priv_data->m4_boot_cached);
			/* Use cached time from M4 */
			rtc_time_to_tm(p_priv_data->m4_seconds_cached +
				get_seconds() - p_priv_data->m4_boot_cached,
				p_tm);
		}
		return 0;
	}

	err = rtc_sensorhub_get_rtc_from_m4(p_tm,
		p_priv_data->p_m4sensorhub_data);

	return err;
}

static int rtc_sensorhub_rtc_set_time(struct device *p_dev,
	struct rtc_time *p_tm)
{
	unsigned long sec = 0;
	struct platform_device *p_platdev = to_platform_device(p_dev);
	struct rtc_sensorhub_private_data *p_priv_data =
						platform_get_drvdata(p_platdev);
	struct m4sensorhub_data *p_m4_drvdata =
			p_priv_data->p_m4sensorhub_data;

	/* M4 expects the UTC time in seconds from Jan 1, 1970,
	basically epoch_time in seconds */
	rtc_tm_to_time(p_tm, &sec);

	if (!(p_m4_drvdata)) {
		dev_err(p_dev,
			"Saving set time request (%lu)\n", sec);
		p_priv_data->sys_seconds_cached = sec;
		p_priv_data->sys_boot_cached = get_seconds();
		return 0;
	}

	/* M4 accepts time as u32*/
	if (m4sensorhub_reg_getsize(p_m4_drvdata, M4SH_REG_GENERAL_UTC) !=
		m4sensorhub_reg_write(p_m4_drvdata, M4SH_REG_GENERAL_UTC,
		(char *)&sec, m4sh_no_mask)) {
			dev_err(p_dev,
				"set time, but failed to set M4 clock!\n");
			return -EIO;
	}

	return 0;
}

static const struct rtc_class_ops rtc_sensorhub_rtc_ops = {
	.read_time = rtc_sensorhub_rtc_read_time,
	.set_time = rtc_sensorhub_rtc_set_time,
	.read_alarm = rtc_sensorhub_rtc_read_alarm,
	.set_alarm = rtc_sensorhub_rtc_set_alarm,
	.alarm_irq_enable = rtc_sensorhub_rtc_alarm_irq_enable,
};

static int rtc_sensorhub_preflash(struct init_calldata *p_arg)
{
	int err = 0;
	uint32_t seconds = 0;
	int size = 0;
	struct rtc_sensorhub_private_data *rtcpd = NULL;
	struct m4sensorhub_data *m4 = NULL;
	struct timespec tv = {.tv_sec = 0, .tv_nsec = 0};
	struct rtc_time rtc;

	if (p_arg == NULL) {
		pr_err("%s: No callback data received.\n", __func__);
		err = -ENODATA;
		goto rtc_sensorhub_preflash_fail;
	} else if (p_arg->p_data == NULL) {
		pr_err("%s: No private data received.\n", __func__);
		err = -ENODATA;
		goto rtc_sensorhub_preflash_fail;
	} else if (p_arg->p_m4sensorhub_data == NULL) {
		pr_err("%s: M4 data is NULL.\n", __func__);
		err = -ENODATA;
		goto rtc_sensorhub_preflash_fail;
	}

	/*
	 * NOTE: We don't want to save the M4 data struct;
	 *       M4 will be reflashed after this callback returns
	 *       (thus the struct will become invalid but we have
	 *        no way to communicate that to the rest of the driver).
	 */
	rtcpd = p_arg->p_data;
	m4 = p_arg->p_m4sensorhub_data;
	size = m4sensorhub_reg_getsize(m4, M4SH_REG_GENERAL_UTC);
	err = m4sensorhub_reg_read(m4, M4SH_REG_GENERAL_UTC, (char *)&seconds);
	if (err < 0) {
		pr_err("%s: Failed to read RTC seconds from M4.\n", __func__);
		goto rtc_sensorhub_preflash_fail;
	} else if (err != size) {
		pr_err("%s: Read %d bytes instead of %d.\n",
			__func__, err, size);
	}

	rtcpd->m4_seconds_cached = seconds;
	rtcpd->m4_boot_cached = get_seconds();

	/* Set the time of day (external processes rely on it) */
	tv.tv_sec = seconds;
	err = do_settimeofday(&tv);
	if (err < 0) {
		pr_err("%s: Failed to set time of day (err=%d)\n",
			__func__, err);
	} else {
		rtc_time_to_tm(seconds, &rtc);
		pr_info("%s: %s %d-%02d-%02d %02d:%02d:%02d UTC (%u)\n",
			__func__, "setting system clock to",
			rtc.tm_year + 1900, rtc.tm_mon + 1, rtc.tm_mday,
			rtc.tm_hour, rtc.tm_min, rtc.tm_sec,
			(unsigned int) tv.tv_sec);
	}

rtc_sensorhub_preflash_fail:
	return err;
}

static void rtc_handle_sensorhub_irq(enum m4sensorhub_irqs int_event,
	void *p_data)
{
	struct rtc_sensorhub_private_data *p_priv_data =
			(struct rtc_sensorhub_private_data *)(p_data);

	pr_info("%s: RTC alarm fired\n", DRIVER_NAME);
	rtc_update_irq(p_priv_data->p_rtc, 1, RTC_AF | RTC_IRQF);
}

static int rtc_sensorhub_init(struct init_calldata *p_arg)
{
	struct rtc_time rtc;
	int err;
	struct timespec tv = {.tv_sec = 0, .tv_nsec = 0};
	struct rtc_sensorhub_private_data *p_priv_data =
			(struct rtc_sensorhub_private_data *)(p_arg->p_data);
	uint32_t seconds = 0;

	p_priv_data->p_m4sensorhub_data = p_arg->p_m4sensorhub_data;

	if ((p_priv_data->sys_seconds_cached != 0) &&
		(p_priv_data->sys_boot_cached != 0)) {
		pr_err("%s: Setting M4 to a saved time request\n", __func__);
		seconds = p_priv_data->sys_seconds_cached +
			get_seconds() - p_priv_data->sys_boot_cached;
		if (m4sensorhub_reg_getsize(p_priv_data->p_m4sensorhub_data,
			M4SH_REG_GENERAL_UTC) != m4sensorhub_reg_write(
			p_priv_data->p_m4sensorhub_data, M4SH_REG_GENERAL_UTC,
			(char *)&seconds, m4sh_no_mask)) {
				pr_err("%s: Failed to set M4 RTC\n",
					__func__);
				return 0;
		}
		/*
		 * We don't write directly to tv_sec here because we want
		 * to print below the system clock we are setting,
		 * which means we need to populate rtc anyway.
		 */
		rtc_time_to_tm(seconds, &rtc);
	} else {
		/* read RTC time from M4 and set the system time */
		err = rtc_sensorhub_get_rtc_from_m4(&rtc,
					p_priv_data->p_m4sensorhub_data);
		if (err) {
			pr_err("%s: get_rtc failed\n", DRIVER_NAME);
			return 0;
		}
	}

	rtc_tm_to_time(&rtc, &tv.tv_sec);
	err = do_settimeofday(&tv);
	if (err) {
		pr_err("%s: settimeofday failed (err=%d)\n", DRIVER_NAME, err);
	} else {
		pr_info("%s: %s %d-%02d-%02d %02d:%02d:%02d UTC (%u)\n",
			__func__, "setting system clock to",
			rtc.tm_year + 1900, rtc.tm_mon + 1, rtc.tm_mday,
			rtc.tm_hour, rtc.tm_min, rtc.tm_sec,
			(unsigned int) tv.tv_sec);
	}

	/* register an irq handler*/
	err = m4sensorhub_irq_register(p_priv_data->p_m4sensorhub_data,
		M4SH_IRQ_AP_ALARM_EXPIRED, rtc_handle_sensorhub_irq,
		p_priv_data, 1);

	if (err < 0)
		pr_err("%s: irq register failed\n", DRIVER_NAME);

	return err;
}

static int rtc_sensorhub_probe(struct platform_device *p_platdev)
{
	int err;
	struct rtc_device *p_rtc;
	struct rtc_sensorhub_private_data *p_priv_data;

	p_priv_data = kzalloc(sizeof(*p_priv_data),
					GFP_KERNEL);
	if (!p_priv_data)
		return -ENOMEM;

	p_priv_data->p_m4sensorhub_data = NULL;
	p_priv_data->next_alarm_set.enabled = false;
	/* Set the private data before registering this driver with RTC core
	since hctosys will call rtc interface right away, we need to make sure
	our private data is set by this time */
	platform_set_drvdata(p_platdev, p_priv_data);

	err = device_init_wakeup(&p_platdev->dev, true);
	if (err) {
		dev_err(&(p_platdev->dev), "failed to init as wakeup\n");
		goto err_free_priv_data;
	}

	p_rtc = devm_rtc_device_register(&p_platdev->dev, "rtc_sensorhub",
		&rtc_sensorhub_rtc_ops, THIS_MODULE);

	if (IS_ERR(p_rtc)) {
		err = PTR_ERR(p_rtc);
		goto err_disable_wakeup;
	}

	p_priv_data->p_rtc = p_rtc;

	err = m4sensorhub_register_preflash_callback(rtc_sensorhub_preflash,
		p_priv_data);
	if (err < 0) {
		dev_err(&(p_platdev->dev),
			"Failed to register M4 preflash callback\n");
		goto err_unregister_rtc;
	}

	err = m4sensorhub_register_initcall(rtc_sensorhub_init, p_priv_data);
	if (err) {
		dev_err(&(p_platdev->dev), "can't register init with m4\n");
		goto err_unregister_preflash_callback;
	}

	return 0;

err_unregister_preflash_callback:
	m4sensorhub_unregister_preflash_callback(rtc_sensorhub_preflash);
err_unregister_rtc:
	devm_rtc_device_unregister(&p_platdev->dev, p_rtc);
	kfree(p_rtc);
err_disable_wakeup:
	device_init_wakeup(&p_platdev->dev, false);
err_free_priv_data:
	kfree(p_priv_data);
	return err;
}

static int rtc_sensorhub_remove(struct platform_device *p_platdev)
{
	struct rtc_sensorhub_private_data *p_priv_data =
						platform_get_drvdata(p_platdev);
	struct rtc_device *p_rtc = p_priv_data->p_rtc;
	device_init_wakeup(&p_platdev->dev, false);
	devm_rtc_device_unregister(&p_platdev->dev, p_rtc);
	m4sensorhub_unregister_initcall(rtc_sensorhub_init);
	m4sensorhub_irq_disable(
			p_priv_data->p_m4sensorhub_data,
			M4SH_IRQ_AP_ALARM_EXPIRED);
	m4sensorhub_irq_unregister(
			p_priv_data->p_m4sensorhub_data,
			M4SH_IRQ_AP_ALARM_EXPIRED);
	kfree(p_priv_data->p_rtc);
	kfree(p_priv_data);
	return 0;
}

static const struct of_device_id of_rtc_sensorhub_match[] = {
	{ .compatible = "mot,rtc_from_sensorhub", },
	{},
};

static struct platform_driver rtc_sensorhub_driver = {
	.probe	= rtc_sensorhub_probe,
	.remove = rtc_sensorhub_remove,
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_rtc_sensorhub_match,
	},
};

module_platform_driver(rtc_sensorhub_driver);

MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("SensorHub RTC driver/device");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtc_sensorhub");
