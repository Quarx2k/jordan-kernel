/*
 * Copyright (C) 2014 Motorola Mobility LLC.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef WAKEUP_SOURCEE_NOTIFY_H
#define WAKEUP_SOURCE_NOTIFY_H

#ifdef __KERNEL__

enum wakeup_request_type {
	WAKEUP_DISPLAY,
};
#define WAKEUP_EVENT_START(type)	((type) << 8)
#define GET_WAKEUP_EVENT_TYPE(event)	((event) >> 8)

enum display_wakeup_request {
	DISPLAY_WAKE_EVENT_BEGIN = WAKEUP_EVENT_START(WAKEUP_DISPLAY),
	DISPLAY_WAKE_EVENT_POWERKEY = DISPLAY_WAKE_EVENT_BEGIN,
	DISPLAY_WAKE_EVENT_TOUCH,
	DISPLAY_WAKE_EVENT_GESTURE,
	DISPLAY_WAKE_EVENT_GESTURE_VIEWON,
	DISPLAY_WAKE_EVENT_GESTURE_VIEWOFF,
	DISPLAY_WAKE_EVENT_DOCKON,
	DISPLAY_WAKE_EVENT_DOCKOFF,
	DISPLAY_WAKE_EVENT_END
};

#define	notify_display_wakeup(reason)	\
	wakeup_source_notify_subscriber(DISPLAY_WAKE_EVENT_##reason)

extern void wakeup_source_register_notify(struct notifier_block *nb);
extern void wakeup_source_unregister_notify(struct notifier_block *nb);
extern void wakeup_source_notify_subscriber(unsigned long event);
#endif /* __KERNEL__ */

#endif /* WAKEUP_SOURCE_NOTIFY_H */
