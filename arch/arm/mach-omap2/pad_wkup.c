/*
 * OMAP3 Pad Wakeup Handler
 *
 * Copyright (C) 2014-2008 Motorola, LLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/err.h>
#include <linux/wakeup_reason.h>

#include "soc.h"
#include "common.h"
#include "mux34xx.h"
#include "prm3xxx.h"
#include "iomap.h"

#include "pad_wkup.h"

#define WAKEUPEVENT		0x8000
struct offmode_wkup {
	struct list_head node;
	u32 irq;
	u32 pad;
	u32 pad_shift;
	bool handle; /* if true generate irq otherwise for debug only */
};
LIST_HEAD(offmode_wkup_list);

static void omap_register_pad_wkup(struct device *dev, u32 pad, u32 irq,
		bool handle)
{
	struct offmode_wkup *wkup;

	pr_info("register pad (0x%04x, %u, %d)\n", pad, irq, handle);

	wkup = devm_kzalloc(dev, sizeof(struct offmode_wkup), GFP_KERNEL);
	if (wkup) {
		wkup->irq = irq;
		wkup->pad = pad & 0xFFFFFFFC;
		wkup->pad_shift = pad % 4 ? 16 : 0;
		wkup->handle = handle;
		list_add_tail(&wkup->node, &offmode_wkup_list);
	}
}

static inline int is_pad_wkup(const struct offmode_wkup *wkup)
{
	int pad = __raw_readl(
		OMAP2_L4_IO_ADDRESS(OMAP3_CONTROL_PADCONF_MUX_PBASE)+ wkup->pad);
	pad = pad >> wkup->pad_shift;

	return (pad & WAKEUPEVENT) ? 1 : 0;
}

/* prcm_handle_pad_wkup:
 *
 *   map i/o pad wkup bits to interrupts.  The primary function is
 *   to generate interrupts.  If the handle flag is set, then generate
 *   interrupt.  If the wkup bit is set on multiple pads, interrupts
 *   are generated for all.   The secondary function is to log the
 *   Typically there should only be one wakeup event, but if there
 *   are multiple wakeup reasons, only the first in the list will be
 *   anointed "the" wakeup reason and logged.  This means the order
 *   items are listed in the device tree will control which irq
 *   wins in the case of a tie.
 */
void prcm_handle_pad_wkup(void)
{
	struct offmode_wkup *wkup;
	int wkup_irq = -1;

	list_for_each_entry(wkup, &offmode_wkup_list, node) {
		if (is_pad_wkup(wkup)) {
			pr_info("%s IRQ = %d\n", __func__, wkup->irq);
			if (wkup_irq < 0)
				wkup_irq = wkup->irq;
			if (wkup->handle)
				generic_handle_irq(wkup->irq);
		}
	}
	if (wkup_irq >= 0)
		log_wakeup_reason(wkup_irq);

}

#ifdef CONFIG_OMAP3_PAD_WKUP_IO
static irqreturn_t omap3_pad_wkup_handle_irq(int irq, void *unused)
{
	prcm_handle_pad_wkup();

	return IRQ_HANDLED;
}
#endif

static int omap3_pad_wkup_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	int ndx = 0;
	uint32_t pad, irq, handle;
	int ret = 0;

	if (!node)
		return -ENODEV;

	do {
		if (of_property_read_u32_index(node, "ti,pad_irq", ndx, &pad))
			break;
		ndx++;
		if (of_property_read_u32_index(node, "ti,pad_irq", ndx, &irq))
			break;
		ndx++;
		if (of_property_read_u32_index(node, "ti,pad_irq", ndx,
				&handle))
			break;
		ndx++;
		omap_register_pad_wkup(&pdev->dev, pad, irq, handle);
	} while (1);

#ifdef CONFIG_OMAP3_PAD_WKUP_IO
	if (ndx > 1) {
		dev_info(&pdev->dev, "request pad_wkup_io\n");
		ret = request_irq(omap_prcm_event_to_irq("io"),
				omap3_pad_wkup_handle_irq,
				IRQF_SHARED | IRQF_NO_SUSPEND,
				"pad_wkup_io", &pdev->dev);

		if (ret)
			pr_warning("wkup: Failed to setup pad_wkup_io irq %d\n",
				ret);
	}
#endif

	return ret;
}

static int omap3_pad_wkup_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id omap3_pad_wkup_table[] = {
	{ .compatible = "ti,pad-wkup", },
	{ },
};
MODULE_DEVICE_TABLE(of, omap3_pad_wkup_table);

static struct platform_driver omap3_pad_wkup_driver = {
	.probe = omap3_pad_wkup_probe,
	.remove = omap3_pad_wkup_remove,
	.driver = {
		.name = "omap3_pad_wkup",
		.owner = THIS_MODULE,
		.of_match_table = omap3_pad_wkup_table,
	},
};

static int __init omap_pad_wkup_init(void)
{
	return platform_driver_register(&omap3_pad_wkup_driver);
}
omap_late_initcall(omap_pad_wkup_init);
