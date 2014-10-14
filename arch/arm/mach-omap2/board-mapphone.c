/*
 * linux/arch/arm/mach-omap2/board-mapphone.c
 *
 * Copyright (C) 2013 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/input/touch_platform.h>
#include <linux/usb/musb.h>
#include <linux/usb/phy.h>
#include <linux/usb/nop-usb-xceiv.h>
#include <linux/ti_wilink_st.h>
#include <linux/clk-provider.h>
#include <linux/clkdev.h>
#include "mux.h"
#include "common.h"
#include "dss-common.h"
#include "control.h"

#include "sdram-toshiba-hynix-numonyx.h"

static int platform_wilink_kim_suspend(struct platform_device *pdev,
		pm_message_t msg);
static int platform_wilink_kim_resume(struct platform_device *pdev);

static struct of_device_id omap_dt_match_table[] __initdata = {
	{ .compatible = "simple-bus", },
	{ .compatible = "ti,omap-infra", },
	{ }
};

static const char *omap3_gp_boards_compat[] __initdata = {
	"mot,omap3-mapphone",
	NULL,
};

struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = 83,
	.dev_name = "/dev/ttyO1",
	.port_index = 1,
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = platform_wilink_kim_suspend,
	.resume = platform_wilink_kim_resume,
};

static struct platform_device wl18xx_device = {
	.name              = "kim",
	.id                = -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device hci_tty_device = {
	.name = "hci_tty",
	.id = -1,
};

static int platform_wilink_kim_suspend(struct platform_device *pdev,
		pm_message_t msg)
{
	return 0;
}

static int platform_wilink_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static inline void __init mapphone_init_btwilink(void)
{
	platform_device_register(&wl18xx_device);
	platform_device_register(&hci_tty_device);
}

static void __init mapphone_init_gpio_clock(void)
{
	struct of_phandle_args clkspec;
	struct clk *clk;
	struct clk_lookup *cl;
	clkspec.np = of_find_compatible_node(NULL, NULL, "gpio-clock");
	if (clkspec.np) {
		of_gpio_clk_setup(clkspec.np);
		clk = of_clk_get_from_provider(&clkspec);
		if (!IS_ERR(clk)) {
			cl = clkdev_alloc(clk, clkspec.np->name, NULL);
			if (cl)
				clkdev_add(cl);
		}
	}
}
extern void mapphone_gadget_init(void);
extern void mapphone_cpcap_client_init(void);

static void __init mapphone_init(void)
{
	struct clk *clkp;

	of_platform_populate(NULL, omap_dt_match_table, NULL, NULL);
	mapphone_init_gpio_clock();
	omap_sdrc_init(JEDEC_JESD209A_sdrc_params, JEDEC_JESD209A_sdrc_params);
	omap3_enable_usim_buffer(); /* Needed for GPIOs in USIM block */

	/* Enable sad2d iclk */
	clkp = clk_get(NULL, "sad2d_ick");
	if (clkp) {
             clk_enable(clkp);
             printk("sad2d_ick enabled\n");
	}

	omap_minnow_display_init();
	mapphone_init_btwilink();
	mapphone_cpcap_client_init();
	usb_bind_phy("musb-hdrc.1.auto", 0, "cpcap_usb");
	usb_musb_init(NULL);
}

MACHINE_START(MAPPHONE, "mapphone_umts")
	.atag_offset    = 0x100,
	.reserve        = omap_reserve,
	.map_io         = omap3_map_io,
	.init_early     = omap3630_init_early,
	.init_irq       = omap_intc_of_init,
	.handle_irq     = omap3_intc_handle_irq,
	.init_machine   = mapphone_init,
	.init_late      = omap3630_init_late,
	.init_time      = omap3_sync32k_timer_init,
	.dt_compat	= omap3_gp_boards_compat,
	.restart        = omap3xxx_restart,
MACHINE_END
