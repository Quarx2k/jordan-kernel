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
#include <linux/err.h>
#include <linux/rtc.h>
#include <linux/platform_device.h>
#include <linux/m4sensorhub.h>
#include <linux/m4sensorhub/m4sensorhub_registers.h>

static bool g_m4ready;

static int rtc_sensorhub_rtc_read_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	pr_err("%s:\n", __func__);

	if (!g_m4ready) {
		pr_err("%s: ignore func call\n", __func__);
		return -EIO;
	}
	/* TODO : implement a real handler*/
	return 0;
}

static int rtc_sensorhub_rtc_set_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	pr_err("%s:\n", __func__);

	if (!g_m4ready) {
		pr_err("%s: ignore func call\n", __func__);
		return -EIO;
	}
	/* TODO : implement a real handler*/
	return 0;
}

static int rtc_sensorhub_get_rtc_from_m4(struct rtc_time *p_tm)
{
	struct m4sensorhub_data *p_m4_drvdata;
	u32 seconds;
	p_m4_drvdata = m4sensorhub_client_get_drvdata();

	if (m4sensorhub_reg_getsize(p_m4_drvdata,
		M4SH_REG_GENERAL_UTC) != m4sensorhub_reg_read(
		p_m4_drvdata, M4SH_REG_GENERAL_UTC,
		(char *)&seconds)) {
		pr_err("%s: Failed get M4 clock!\n", __func__);
		return -EIO;
	}

	rtc_time_to_tm(seconds, p_tm);
	return 0;
}

static int rtc_sensorhub_rtc_read_time(struct device *dev,
	struct rtc_time *p_tm)
{
	int err;

	if (!g_m4ready) {
		pr_err("%s: func called, RTC hardware not ready\n", __func__);
		/* M4 driver is not yet ready, just give the time since boot
		and treat boot as start of epoch */
		rtc_time_to_tm(get_seconds(), p_tm);
		return 0;
	}

	err = rtc_sensorhub_get_rtc_from_m4(p_tm);

	return err;
}

static int rtc_sensorhub_rtc_set_time(struct device *dev,
	struct rtc_time *p_tm)
{
	u32 seconds;
	unsigned long sec;
	struct m4sensorhub_data *p_m4_drvdata;

	if (!g_m4ready) {
		pr_err("%s: ignore func call\n", __func__);
		return 0;
	}
	p_m4_drvdata = m4sensorhub_client_get_drvdata();

	/* M4 expects the UTC time in seconds from Jan 1, 1970,
	basically epoch_time in seconds */
	rtc_tm_to_time(p_tm, &sec);

	/* M4 accepts time as u32*/
	seconds = (u32) sec;

	if (m4sensorhub_reg_getsize(p_m4_drvdata,
		M4SH_REG_GENERAL_UTC) != m4sensorhub_reg_write(
		p_m4_drvdata, M4SH_REG_GENERAL_UTC,
		(char *)&seconds, m4sh_no_mask)) {
			pr_err("%s: Failed set M4 clock!\n", __func__);
			return -EIO;
	}

	return 0;
}

static int rtc_sensorhub_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	/* TODO : implement a real handler*/
	dev_info(dev, "%s, secs = %lu\n", __func__, secs);

	if (!g_m4ready) {
		pr_err("%s: ignore func call\n", __func__);
		return -EIO;
	}
	return 0;
}

static int rtc_sensorhub_rtc_proc(struct device *dev, struct seq_file *seq)
{
	/* TODO : implement a real handler*/
	struct platform_device *plat_dev = to_platform_device(dev);
	char buffer[50];
	int length;
	pr_err("%s:\n", __func__);

	seq_puts(seq, "sensorhub\t\t: yes\n");

	length = sprintf(buffer, "id\t\t: %d\n", plat_dev->id);
	if (length < 49) {
		buffer[length] = 0;
		seq_puts(seq, buffer);
	}

	return 0;
}

static int rtc_sensorhub_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enable)
{
	/* TODO : implement a real handler*/
	if (!g_m4ready) {
		pr_err("%s: ignore func call\n", __func__);
		return 0;
	}
	return 0;
}

static const struct rtc_class_ops rtc_sensorhub_rtc_ops = {
	.proc = rtc_sensorhub_rtc_proc,
	.read_time = rtc_sensorhub_rtc_read_time,
	.set_time = rtc_sensorhub_rtc_set_time,
	.read_alarm = rtc_sensorhub_rtc_read_alarm,
	.set_alarm = rtc_sensorhub_rtc_set_alarm,
	.set_mmss = rtc_sensorhub_rtc_set_mmss,
	.alarm_irq_enable = rtc_sensorhub_rtc_alarm_irq_enable,
};

static ssize_t rtc_sensorhub_irq_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	/* TODO : implement a real handler*/
	if (!g_m4ready) {
		pr_err("%s: ignore func call\n", __func__);
		return -EIO;
	}
	return sprintf(buf, "%d\n", 42);
}
static ssize_t rtc_sensorhub_irq_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	/* TODO : implement a real handler*/
	pr_err("%s:\n", __func__);
	if (!g_m4ready) {
		pr_err("%s: ignore func call\n", __func__);
		return -EIO;
	}
	return count;
}

static DEVICE_ATTR(irq, S_IRUGO | S_IWUSR, rtc_sensorhub_irq_show,
						rtc_sensorhub_irq_store);

static int rtc_sensorhub_init(struct m4sensorhub_data *m4sensorhub)
{
	struct rtc_time rtc;
	int err;
	struct timespec tv;

	g_m4ready = true;

	/* read RTC time from M4 and set the system time */
	err = rtc_sensorhub_get_rtc_from_m4(&rtc);
	if (err) {
		pr_err("%s: get_rtc failed\n", __func__);
		return 0;
	}

	rtc_tm_to_time(&rtc, &tv.tv_sec);

	err = do_settimeofday(&tv);
	if (err) {
		pr_err("%s: settimeofday failed\n", __func__);
		return 0;
	}

	pr_info("setting system clock to "
		"%d-%02d-%02d %02d:%02d:%02d UTC (%u)\n",
		rtc.tm_year + 1900, rtc.tm_mon + 1, rtc.tm_mday,
		rtc.tm_hour, rtc.tm_min, rtc.tm_sec,
		(unsigned int) tv.tv_sec);

	return 0;
}

static int rtc_sensorhub_probe(struct platform_device *plat_dev)
{
	int err;
	struct rtc_device *rtc;

	err = device_init_wakeup(&plat_dev->dev, true);
	if (err) {
		pr_err("%s: failed to init as wakeup\n", __func__);
		return err;
	}

	rtc = devm_rtc_device_register(&plat_dev->dev, "rtc_sensorhub",
				&rtc_sensorhub_rtc_ops, THIS_MODULE);

	if (IS_ERR(rtc)) {
		err = PTR_ERR(rtc);
		goto err_disable_wakeup;
	}

	err = device_create_file(&plat_dev->dev, &dev_attr_irq);
	if (err)
		goto err_unregister_rtc;

	err = m4sensorhub_register_initcall(rtc_sensorhub_init);
	if (err) {
		pr_err("%s: can't register init with m4\n", __func__);
		goto err_remove_file;
	}

	platform_set_drvdata(plat_dev, rtc);

	return 0;

err_remove_file:
	device_remove_file(&plat_dev->dev, &dev_attr_irq);
err_unregister_rtc:
	devm_rtc_device_unregister(&plat_dev->dev, rtc);
err_disable_wakeup:
	device_init_wakeup(&plat_dev->dev, false);
	return err;
}

static int rtc_sensorhub_remove(struct platform_device *plat_dev)
{
	struct rtc_device *rtc = platform_get_drvdata(plat_dev);
	device_remove_file(&plat_dev->dev, &dev_attr_irq);
	device_init_wakeup(&plat_dev->dev, false);
	devm_rtc_device_unregister(&plat_dev->dev, rtc);
	m4sensorhub_unregister_initcall(rtc_sensorhub_init);
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
		.name = "rtc-sensorhub",
		.owner = THIS_MODULE,
		.of_match_table = of_rtc_sensorhub_match,
	},
};

module_platform_driver(rtc_sensorhub_driver);

MODULE_AUTHOR("Motorola Mobility LLC");
MODULE_DESCRIPTION("SensorHub RTC driver/device");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:rtc_sensorhub");
