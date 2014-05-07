/*
 * drivers/media/video/hp3a/hp3a.h
 *
 * HP Imaging/3A Driver : Top level header.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#ifndef	__HP3A_H_INCLUDED
#define  __HP3A_H_INCLUDED

#include <linux/types.h>

/**
 * hp3a framework notification functions.
 **/
void hp3a_hw_enabled(u8 enable);
void hp3a_ccdc_done(void);
void hp3a_frame_done(void);
void hp3a_stream_on(void);
void hp3a_stream_off(void);
void hp3a_ccdc_start(void);
void hp3a_update_wb(void);
int hp3a_hist_busy(void);
int hp3a_af_busy(void);
void hp3a_set_sensor_sync(unsigned char exposure, unsigned char gain);
#endif	/* __HP3A_H_INCLUDED */
