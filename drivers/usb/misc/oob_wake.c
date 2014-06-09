/*
 * Copyright (C) 2011 Motorola, Inc.
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
 * 02111-1307  USA
 */
#include <linux/device.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/oob_wake.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif /* CONFIG_HAS_WAKELOCK */

#define GPIO_MAX_NAME 30

/* list of oob_wake_info devices */
static LIST_HEAD(dev_list);
static DEFINE_MUTEX(dev_list_lock);

struct oob_wake_info {
	unsigned int irq;
	char name[GPIO_MAX_NAME];
	__le16 vendor;
	__le16 product;
#ifdef CONFIG_HAS_WAKELOCK
	struct wake_lock wake_lock;
#endif /* CONFIG_HAS_WAKELOCK */
	struct list_head node;
	struct usb_interface *intf; /* limit one interface per device */
	struct delayed_work wake_work;
};

static void oob_wake_work(struct work_struct *work);

/* adds a single usb interface to the device to be woken up by the
 * out of band interrupt.  Only "unique wake events" are added.
 * Meaning interfaces that coming from the same device and bus
 * will be considered equivalent and only the first will be added.
 */
int oob_wake_register(struct usb_interface *intf)
{
	struct list_head *ptr;
	struct oob_wake_info *info;
	struct usb_device *udev = interface_to_usbdev(intf);

	mutex_lock(&dev_list_lock);
	list_for_each(ptr, &dev_list) {
		info = list_entry(ptr, struct oob_wake_info, node);
		if ((udev->descriptor.idVendor == info->vendor) &&
			(udev->descriptor.idProduct == info->product)) {
			if (!info->intf) {
				info->intf = intf;
				enable_irq(info->irq);
				enable_irq_wake(info->irq);
			}
			break;
		}
	}
	mutex_unlock(&dev_list_lock);

	return 0;
}
EXPORT_SYMBOL(oob_wake_register);

/* removes the given interface from the device to be woken up by
 * the out of band interrupt */
void oob_wake_unregister(struct usb_interface *intf)
{
	struct list_head *ptr;
	struct oob_wake_info *info;
	struct usb_device *udev = interface_to_usbdev(intf);

	mutex_lock(&dev_list_lock);
	list_for_each(ptr, &dev_list) {
		info = list_entry(ptr, struct oob_wake_info, node);
		if ((udev->descriptor.idVendor == info->vendor) &&
			(udev->descriptor.idProduct == info->product)) {
			if (info->intf == intf) {
				disable_irq_wake(info->irq);
				disable_irq_nosync(info->irq);
				info->intf = NULL;
			}
			break;
		}
	}
	mutex_unlock(&dev_list_lock);
}
EXPORT_SYMBOL(oob_wake_unregister);

/* wake up the usb bus if needed */
static void wake_interface(struct oob_wake_info *info)

{
	struct usb_interface *intf = info->intf;
	long ret;
	struct usb_device *udev = interface_to_usbdev(intf);

	pr_debug("%s: called- status %d\n", __func__,
		 intf->dev.power.is_suspended);

	/*
	 * We check again since device state could have changed by the time
	 * the delayed work gets scheduled.
	 */
	if (udev->state == USB_STATE_NOTATTACHED) {
		pr_err("%s: device has disconnected\n", __func__);
		return;
	}

	device_lock(&intf->dev);

	/* Don't proceed until we're awake enough to unsuspend */
	if (intf->dev.power.is_suspended) {
		device_unlock(&intf->dev);
		if (printk_ratelimit())
			pr_debug("%s: pm state %d, retry..\n",
					__func__, intf->dev.power.is_suspended);
		wake_lock_timeout(&info->wake_lock, HZ);
		schedule_delayed_work(&info->wake_work, msecs_to_jiffies(20));
		return;
	}

	pr_debug("%s: Call usb_autopm\n", __func__);
	ret = usb_autopm_get_interface(intf);
	if (ret == 0)
		usb_autopm_put_interface_async(intf);
	else
		pr_err("%s: get interface returned err %ld\n",
						__func__, ret);
	usb_mark_last_busy(udev);
	device_unlock(&intf->dev);
	/* Give USB long enough to take it's own wakelock. */
	wake_lock_timeout(&info->wake_lock, HZ/4);
}

/* Wake the interface for the associated device (vendor/product) */
static void oob_wake_work(struct work_struct *work)
{
	struct delayed_work *dwork = container_of(work,
					struct delayed_work,
					work);
	struct oob_wake_info *info = container_of(dwork,
					struct oob_wake_info,
					wake_work);
	struct usb_interface *intf;

	intf = info->intf;
	if (intf)
		wake_interface(info);

	return;
}

static irqreturn_t oob_wake_isr(int irq, void *data)
{
	struct oob_wake_info *info = (struct oob_wake_info *) data;
	struct usb_device *udev = interface_to_usbdev(info->intf);
#ifdef CONFIG_HAS_WAKELOCK
	pr_debug("%s: take 1 sec wakelock %s\n", __func__,
					info->wake_lock.name);
	wake_lock_timeout(&info->wake_lock, 1 * HZ);
#endif /* CONFIG_HAS_WAKELOCK */

	if (udev->state == USB_STATE_NOTATTACHED)
		pr_err("%s: device has disconnected\n", __func__);
	else
		schedule_delayed_work(&info->wake_work, 0);

	return IRQ_HANDLED;
}

static int __devinit oob_wake_probe(struct platform_device *pdev)
{
	struct oob_wake_platform_data *pdata = pdev->dev.platform_data;
	struct oob_wake_info *info;
	int err = 0;

	pr_info("%s: %s\n", __func__, dev_name(&pdev->dev));
	info = kzalloc(sizeof(struct oob_wake_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->irq = platform_get_irq(pdev, 0);
	info->vendor = pdata->vendor;
	info->product = pdata->product;

	platform_set_drvdata(pdev, info);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&info->wake_lock, WAKE_LOCK_SUSPEND,
			dev_name(&pdev->dev));
#endif /* CONFIG_HAS_WAKELOCK */

	snprintf(info->name, GPIO_MAX_NAME, "%s-%s",
		dev_name(&pdev->dev), "host-wake");
	INIT_DELAYED_WORK(&info->wake_work, oob_wake_work);
	err = request_irq(info->irq, oob_wake_isr,
		IRQ_TYPE_EDGE_FALLING | IRQF_ONESHOT, info->name, info);
	if (err) {
		pr_err("%s: error requesting host wake irq\n", __func__);
		return err;
	}

	/* start out disabled */
	disable_irq(info->irq);

	mutex_lock(&dev_list_lock);
	list_add(&info->node, &dev_list);
	mutex_unlock(&dev_list_lock);

	return 0;
}

static void __devexit oob_wake_shutdown(struct platform_device *pdev)
{
	struct list_head *ptr;
	struct list_head *next;
	struct oob_wake_info *entry;
	struct oob_wake_info *info = platform_get_drvdata(pdev);

	pr_info("%s: %s\n", __func__, dev_name(&pdev->dev));
	if (info) {
		mutex_lock(&dev_list_lock);
		list_for_each_safe(ptr, next, &dev_list) {
			entry = list_entry(ptr, struct oob_wake_info, node);
			if (entry == info)
				list_del(&entry->node);
		}
		if (info->intf)
			disable_irq_wake(info->irq);
		mutex_unlock(&dev_list_lock);

		free_irq(info->irq, info);
		cancel_delayed_work_sync(&info->wake_work);
#ifdef CONFIG_HAS_WAKELOCK
		wake_lock_destroy(&info->wake_lock);
#endif /* CONFIG_HAS_WAKELOCK */
		kfree(info);

		platform_set_drvdata(pdev, NULL);
	}
}

static struct platform_driver oob_wake_driver = {
	.probe = oob_wake_probe,
	.shutdown = __devexit_p(oob_wake_shutdown),
	.driver = {
		.name = "oob-wake",
		.owner = THIS_MODULE,
	},
};

static int __init oob_wake_init(void)
{
	pr_info("%s: initializing %s\n", __func__, oob_wake_driver.driver.name);

	return platform_driver_register(&oob_wake_driver);
}

static void __exit oob_wake_exit(void)
{
	pr_info("%s: exiting %s\n", __func__, oob_wake_driver.driver.name);
	return platform_driver_unregister(&oob_wake_driver);
}

module_init(oob_wake_init);
module_exit(oob_wake_exit);

MODULE_AUTHOR("Jim Wylder <james.wylder@motorola.com>");
MODULE_DESCRIPTION("USB Out-of-Bounds Wake");
MODULE_LICENSE("GPL");
