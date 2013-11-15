/*
 * linux/arch/arm/mach-omap2/board-minnow.h
 *
 * Copyright (C) 2013 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/i2c.h>

extern void __init minnow_bt_init(void);
extern void __init minnow_spi_init(void);
extern void __init minnow_sensors_init(void);
extern void __init minnow_touch_init(struct i2c_board_info *i2c_info);
extern void __init minnow_cpcap_client_init(void);

#if defined(CONFIG_MFD_M4SENSORHUB) || defined(CONFIG_MFD_M4SENSORHUB_MODULE)
extern int m4sensorhub_stillmode_exit(void);
/* m4_ctrl == 0 OMAP and 1 Sensorhub */
extern int m4sensorhub_set_display_control(int m4_ctrl, int gpio_mipi_mux);
#endif
