
/*
 * board-mapphone-padconf.h
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __BOARD_MAPPHONE_PADCONF_H
#define __BOARD_MAPPHONE_PADCONF_H

/* core control module padconf registers are at 0x48002030 - 0x480021E2 */
#define OMAP343X_PADCONF_CORE_CTRL_BASE (OMAP343X_CTRL_BASE + 0x030)
#define OMAP343X_PADCONF_CORE_CTRL_TOP  (OMAP343X_CTRL_BASE + 0x1E2)

/* etk padconf registers are at 0x480025D8 - 0x480025FA */
#define OMAP343X_PADCONF_CORE_ETK_BASE  (OMAP343X_CTRL_BASE + 0x5D8)
#define OMAP343X_PADCONF_CORE_ETK_TOP   (OMAP343X_CTRL_BASE + 0x5FA)

/* d2d padconf registers are at 0x480021E4 - 0x48002264 */
#define OMAP343X_PADCONF_CORE_D2D_BASE  (OMAP343X_CTRL_BASE + 0x1E4)
#define OMAP343X_PADCONF_CORE_D2D_TOP   (OMAP343X_CTRL_BASE + 0x266)

/* wakeup module padconf registers are at 0x48002A00 - 0x48002A26 */
#define OMAP343X_PADCONF_WKUP_BASE  (OMAP343X_CTRL_BASE + 0xA00)
#define OMAP343X_PADCONF_WKUP_TOP   (OMAP343X_CTRL_BASE + 0xA26)

/* d2d wakeup module padconf registers are at 0x48002A4C - 0x48002A50 */
#define OMAP343X_PADCONF_WKUP_D2D_BASE  (OMAP343X_CTRL_BASE + 0xA4C)
#define OMAP343X_PADCONF_WKUP_D2D_TOP   (OMAP343X_CTRL_BASE + 0xA50)

/* OMAP3630 GPIO extension padconf registers are at 0x48002A54 - 0x48002A5A */
#define OMAP343X_PADCONF_3630_GPIO_BASE  (OMAP343X_CTRL_BASE + 0xA54)
#define OMAP343X_PADCONF_3630_GPIO_TOP   (OMAP343X_CTRL_BASE + 0xA5A)

#define OMAP343X_PADCONF_OFF_WAKEUP_ENABLED (1 << 14)

#define OMAP343X_PADCONF_OFF_PULL_UP	    (1 << 13)
#define OMAP343X_PADCONF_OFF_PULL_DOWN	    (0 << 13)

#define OMAP343X_PADCONF_OFF_PUD_ENABLED    (1 << 12)
#define OMAP343X_PADCONF_OFF_PUD_DISABLED   (0 << 12)

#define OMAP343X_PADCONF_OFF_OUTPUT_HIGH    (1 << 11)
#define OMAP343X_PADCONF_OFF_OUTPUT_LOW	    (0 << 11)

#define OMAP343X_PADCONF_OFF_OUTPUT_ENABLED (1 << 10)
#define OMAP343X_PADCONF_OFF_OUTPUT_DISABLED	(0 << 10)

#define OMAP343X_PADCONF_OFFMODE_ENABLED    (1 << 9)
#define OMAP343X_PADCONF_OFFMODE_DISABLED   (0 << 9)

#define OMAP343X_PADCONF_INPUT_ENABLED	    (1 << 8)

#define OMAP343X_PADCONF_PULL_UP	    (1 << 4)
#define OMAP343X_PADCONF_PULL_DOWN	    (0 << 4)

#define OMAP343X_PADCONF_PUD_ENABLED	    (1 << 3)
#define OMAP343X_PADCONF_PUD_DISABLED	    (0 << 3)

#define OMAP343X_PADCONF_MUXMODE0   (0x00)
#define OMAP343X_PADCONF_MUXMODE1   (0x01)
#define OMAP343X_PADCONF_MUXMODE2   (0x02)
#define OMAP343X_PADCONF_MUXMODE3   (0x03)
#define OMAP343X_PADCONF_MUXMODE4   (0x04)
#define OMAP343X_PADCONF_MUXMODE5   (0x05)
#define OMAP343X_PADCONF_MUXMODE6   (0x06)
#define OMAP343X_PADCONF_MUXMODE7   (0x07)

#define OMAP343X_PADCONF_SETTING_MASK (OMAP343X_PADCONF_OFF_WAKEUP_ENABLED | \
		OMAP343X_PADCONF_OFF_PULL_UP | \
		OMAP343X_PADCONF_OFF_PUD_ENABLED | \
		OMAP343X_PADCONF_OFF_OUTPUT_HIGH | \
		OMAP343X_PADCONF_OFF_OUTPUT_ENABLED | \
		OMAP343X_PADCONF_OFFMODE_ENABLED | \
		OMAP343X_PADCONF_INPUT_ENABLED | \
		OMAP343X_PADCONF_PULL_UP | \
		OMAP343X_PADCONF_PUD_ENABLED | \
		OMAP343X_PADCONF_MUXMODE7)


#ifdef CONFIG_ARM_OF
#define OMAP343X_PAD_MASK (OMAP343X_PADCONF_MUXMODE7 | \
		OMAP343X_PADCONF_INPUT_ENABLED | \
		OMAP343X_PADCONF_PUD_ENABLED | \
		OMAP343X_PADCONF_PULL_UP)

#define OMAP343X_OFFMODE_MASK (OMAP343X_PADCONF_OFFMODE_ENABLED | \
		OMAP343X_PADCONF_OFF_OUTPUT_ENABLED | \
		OMAP343X_PADCONF_OFF_OUTPUT_HIGH | \
		OMAP343X_PADCONF_OFF_PUD_ENABLED | \
		OMAP343X_PADCONF_OFF_PULL_UP | \
		OMAP343X_PADCONF_OFF_WAKEUP_ENABLED)

#define MAKE_OMAP343X_PAD_VALUE(mode, input_en, pull) \
		(((u16) mode) | ((u16) input_en) << 8 | ((u16) pull) << 3)

#define MAKE_OMAP343X_OFFMODE_VALUE(offmode, offout, offpull, offwkup) \
		(((u16) (offmode) << 9) | ((u16) (offout) << 10) | \
		((u16) (offpull) << 12) | ((u16) (offwkup) << 14))

struct dt_operation {
	const char *path;
	const char *prop;
	u32 prop_unit_size;
	void (*callback) (const void *p_data);
	u32 name_size;
} __attribute__ ((__packed__));

struct mux_conf_entry {
	u16 offset;
	u8 mode;
	u8 input_en;
	u8 pull_type;
} __attribute__ ((__packed__));

struct mux_offmode_conf_entry {
	u16 offset;
	u8 offmode_en;
	u8 offout_type;
	u8 offpull_type;
	u8 offwkup_en;
} __attribute__ ((__packed__));
#endif

#endif
