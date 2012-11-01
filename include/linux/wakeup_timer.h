/*
 *  wakeup_timer.h
 *
 *  Copyright (C) 2009 Motorola, Inc.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Adds ability to program periodic interrupts from user space that
 *  can wake the phone out of low power modes.
 *
 */
 /*
 * DATE             AUTHOR         COMMENT
 * -----            ------         --------
 * Apr 30, 2009     Motorola       Initial version for omap Android
 *
 */

#ifndef WAKEUP_TIMER_H_
#define WAKEUP_TIMER_H_

#include <linux/time.h>

/*
 * We only allow one thread to have one wakeup timer
 * For periodic timer, it can just use IOC_WAKEUP_TIMER_SETPERIOD
 * to set the perioid and after that,
 * the thread can only use poll on this driver to wait for timer fire.
 * For oneshot timer, the app should use IOC_WAKEUP_TIMER_ONESHOT to
 * reset the time out every time after the timer fires
 * It should call IOC_WAKEUP_TIMER_DELETE in time to remove the timer
 * if it only wants for one time wakeup
 */
/* Set a periodic wakeup timers request */
#define IOC_WAKEUP_TIMER_SETPERIOD		_IOW('w', 0x1, int)
/* Set a oneshot wakeup timer request */
#define IOC_WAKEUP_TIMER_ONESHOT		_IOW('w', 0x2, int)
/* Selete the wakeup timer request */
#define IOC_WAKEUP_TIMER_DELETE			_IO('w', 0x3)

/* Operations for /dev/clock_32k device */
#define IOC_32KHZ_READ                          _IOW('w', 0x1, int)

#endif /* WAKEUP_TIMER_H_ */
