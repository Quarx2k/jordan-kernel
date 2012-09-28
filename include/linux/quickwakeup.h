/* include/linux/quickwakeup.h
 *
 * Copyright (C) 2009 Motorola.
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

struct quickwakeup_ops {
	struct list_head list;
	int (*qw_callback) (void);
	int (*qw_check)(void);
	int checked;
};

#ifdef CONFIG_QUICK_WAKEUP

int quickwakeup_register(struct quickwakeup_ops *ops);
int quickwakeup_check(void);
int quickwakeup_execute(void);
void quickwakeup_unregister(struct quickwakeup_ops *ops);

#else
static int quickwakeup_register(struct quickwakeup_ops *ops) { return 0; };
void quickwakeup_unregister(struct quickwakeup_ops *ops) {};
#endif
