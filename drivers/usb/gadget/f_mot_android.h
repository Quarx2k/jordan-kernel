/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2009 Motorola, Inc.
 * Author:
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __F_MOT_ANDROID_H
#define __F_MOT_ANDROID_H

#define MSC_TYPE_FLAG         0x01
#define ADB_TYPE_FLAG         0x02
#define ETH_TYPE_FLAG         0x04
#define MTP_TYPE_FLAG         0x08
#define ACM_TYPE_FLAG         0x10
#define CDROM_TYPE_FLAG       0x20
#define ECM_TYPE_FLAG         0x40
#define RNDIS_TYPE_FLAG       0x80
#define ETS_TYPE_FLAG         0x100

void usb_interface_enum_cb(int flag);
void usb_data_transfer_callback(void);
void mode_switch_cb(int mode);
void adb_mode_change_cb(void);

#endif /* __F_MOT_ANDROID_H */
