/*
 * Support for the TI OMAPXX Tablet Application board.
 *
 * Copyright (C) 2012 Texas Instruments
 *
 * Author: Dan Murphy <dmurphy@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 */
#include <linux/kernel.h>
#include <plat/omap_apps_brd_id.h>

bool omap_is_board_version(int req_board_version)
{
	return true;
}

int omap_get_board_version(void)
{
	return -1;
}

int omap_get_board_id(void)
{
	return -1;
}

__init int omap_init_board_version(int forced_rev)
{
	return -1;
}
