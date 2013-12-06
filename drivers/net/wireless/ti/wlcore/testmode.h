/*
 * This file is part of wl1271
 *
 * Copyright (C) 2010 Nokia Corporation
 *
 * Contact: Luciano Coelho <luciano.coelho@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#ifndef __TESTMODE_H__
#define __TESTMODE_H__

#ifdef __KERNEL__
#include <net/mac80211.h>

int wl1271_tm_cmd(struct ieee80211_hw *hw, void *data, int len);
#endif

enum wl1271_tm_commands {
	WL1271_TM_CMD_UNSPEC,
	WL1271_TM_CMD_TEST,
	WL1271_TM_CMD_INTERROGATE,
	WL1271_TM_CMD_CONFIGURE,
	WL1271_TM_CMD_NVS_PUSH,		/* Not in use. Keep to not break ABI */
	WL1271_TM_CMD_SET_PLT_MODE,
	WL1271_TM_CMD_RECOVER,		/* Not in use. Keep to not break ABI */
	WL1271_TM_CMD_GET_MAC,

	WL1271_TM_CMD_SMART_CONFIG_START,
	WL1271_TM_CMD_SMART_CONFIG_STOP,
	WL1271_TM_CMD_SMART_CONFIG_SET_GROUP_KEY,

	__WL1271_TM_CMD_AFTER_LAST
};
#define WL1271_TM_CMD_MAX (__WL1271_TM_CMD_AFTER_LAST - 1)

enum wl1271_tm_attrs {
	WL1271_TM_ATTR_UNSPEC,
	WL1271_TM_ATTR_CMD_ID,
	WL1271_TM_ATTR_ANSWER,
	WL1271_TM_ATTR_DATA,
	WL1271_TM_ATTR_IE_ID,
	WL1271_TM_ATTR_PLT_MODE,

	WL1271_TM_ATTR_SMART_CONFIG_EVENT,
	WL1271_TM_ATTR_FREQ,
	WL1271_TM_ATTR_PSK,
	WL1271_TM_ATTR_SSID,
	WL1271_TM_ATTR_GROUP_ID,
	WL1271_TM_ATTR_GROUP_KEY,

	__WL1271_TM_ATTR_AFTER_LAST
};
#define WL1271_TM_ATTR_MAX (__WL1271_TM_ATTR_AFTER_LAST - 1)

enum wlcore_tm_attr_smart_config_event {
	WLCORE_TM_SC_EVENT_SYNC,
	WLCORE_TM_SC_EVENT_DECODE,
};

#endif /* __WL1271_TESTMODE_H__ */
