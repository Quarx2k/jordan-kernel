/*
 * isp_mem_process.h
 *
 * Top level public header file for ISP Control module in
 * TI's OMAP3 Camera ISP
 *
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Contributors:
 * 	Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

#include <plat/isp_user.h>

#ifndef OMAP_ISP_MEM_PROCESS_H
#define OMAP_ISP_MEM_PROCESS_H

int isp_process_mem_data(struct isp_mem_data *data);

int isp_resize_mem_data(struct isp_mem_resize_data *data);

void isp_mem_process_init(void);

#endif

