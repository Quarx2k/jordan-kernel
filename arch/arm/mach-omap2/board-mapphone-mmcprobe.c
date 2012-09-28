/*
 * arch/arm/mach-omap2/board-mapphone-mmcprobe.c
 *
 * Copyright (C) 2007-2009 Google Inc.
 * Copyright (C) 2009 Motorola Inc.
 *
 * San Mehat (san@android.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/mm.h>
#include <linux/bootmem.h>
#include <linux/reboot.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <plat/board-mapphone.h>
#include <plat/hardware.h>
#include <plat/gpio.h>
#include <plat/mux.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/control.h>
#include <linux/delay.h>

#define CH_BASE_GPIO 120

void mapphone_mmcprobe_strobe(int ch, int rpt)
{
	int i;

	for (i = 0; i < rpt; i++) {
		gpio_direction_output(CH_BASE_GPIO + ch, 1);
		udelay(10);
		gpio_direction_output(CH_BASE_GPIO + ch, 0);
		udelay(10);
	}
}

void mapphone_mmcprobe_set(int ch, int lvl)
{
	gpio_direction_output(CH_BASE_GPIO + ch, lvl);
}

void __init mapphone_mmcprobe_init(void)
{
	int i;

	printk(KERN_INFO "mapphone_mmcprobe: MMC subsystem ganked for debug\n");

	omap_ctrl_writew(0x04, 0x144); /* MMC1_CLK/GPIO120 */
	omap_ctrl_writew(0x04, 0x146); /* MMC1_CMD/GPIO121 */
	omap_ctrl_writew(0x04, 0x148); /* MMC1_DAT0/GPIO122 */
	omap_ctrl_writew(0x04, 0x14a); /* MMC1_DAT1/GPIO123 */
	omap_ctrl_writew(0x04, 0x14c); /* MMC1_DAT2/GPIO124 */
	omap_ctrl_writew(0x04, 0x14e); /* MMC1_DAT3/GPIO125 */

	gpio_request(120, "mmcprobe-ch1");
	gpio_request(121, "mmcprobe-ch2");
	gpio_request(122, "mmcprobe-ch3");
	gpio_request(123, "mmcprobe-ch4");
	gpio_request(124, "mmcprobe-ch5");
	gpio_request(125, "mmcprobe-ch6");

	for (i = 0; i < 5; i++)
		mapphone_mmcprobe_strobe(i, 1 + i);
}
