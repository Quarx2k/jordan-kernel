/*
 * linux/drivers/mmc/host/omap_hsmmc_raw.h
 *
 * Raw omap hsmmc controller driver
 *
 * Copyright (C) 2010 Motorola
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

extern int raw_mmc_panic_probe(struct raw_hd_struct *rhd, int type);
extern int raw_mmc_panic_write(struct raw_hd_struct *rhd, char *buf,
		unsigned int offset, unsigned int len);
extern int raw_mmc_panic_erase(struct raw_hd_struct *rhd, unsigned int offset,
		unsigned int len);
