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

enum display_wakeup_request {
	DISPLAY_WAKE_EVENT,
};

extern void wakeup_source_register_notify(struct notifier_block *nb);
extern void wakeup_source_unregister_notify(struct notifier_block *nb);
extern void wakeup_source_notify_subscriber(unsigned long event);
#endif /* __KERNEL__ */

#endif /* WAKEUP_SOURCE_NOTIFY_H */
