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

static int rtc_sensorhub_rtc_read_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	/* TODO : implement a real handler*/
	return 0;
}

static int rtc_sensorhub_rtc_set_alarm(struct device *dev,
	struct rtc_wkalrm *alrm)
{
	/* TODO : implement a real handler*/
	return 0;
}

static int rtc_sensorhub_rtc_read_time(struct device *dev,
	struct rtc_time *tm)
{
	/* TODO : implement a real handler*/
	rtc_time_to_tm(get_seconds(), tm);
	return 0;
}

static int rtc_sensorhub_rtc_set_time(struct device *dev,
	struct rtc_time *tm)
{
	/* TODO : implement a real handler*/
	pr_err("%s:\n", __func__);
	return 0;
}

static int rtc_sensorhub_rtc_set_mmss(struct device *dev, unsigned long secs)
{
	/* TODO : implement a real handler*/
	dev_info(dev, "%s, secs = %lu\n", __func__, secs);
	return 0;
}

static int rtc_sensorhub_rtc_proc(struct device *dev, struct seq_file *seq)
{
	/* TODO : implement a real handler*/
	struct platform_device *plat_dev = to_platform_device(dev);
	char buffer[50];
	int length;

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
	pr_err("%s:\n", __func__);
	return sprintf(buf, "%d\n", 42);
}
static ssize_t rtc_sensorhub_irq_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	/* TODO : implement a real handler*/
	pr_err("%s:\n", __func__);
	return count;
}

static DEVICE_ATTR(irq, S_IRUGO | S_IWUSR, rtc_sensorhub_irq_show,
						rtc_sensorhub_irq_store);

static int rtc_sensorhub_probe(struct platform_device *plat_dev)
{
	int err;
	struct rtc_device *rtc;

	rtc = devm_rtc_device_register(&plat_dev->dev, "rtc_sensorhub",
				&rtc_sensorhub_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		err = PTR_ERR(rtc);
		return err;
	}

	err = device_create_file(&plat_dev->dev, &dev_attr_irq);
	if (err)
		goto err;

	platform_set_drvdata(plat_dev, rtc);

	return 0;

err:
	return err;
}

static int rtc_sensorhub_remove(struct platform_device *plat_dev)
{
	device_remove_file(&plat_dev->dev, &dev_attr_irq);

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
