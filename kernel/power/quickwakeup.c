/* kernel/power/quickwakeup.c
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

#include <linux/slab.h>
#include <linux/quickwakeup.h>
#include <linux/module.h>

static LIST_HEAD(qw_head);

int quickwakeup_register(struct quickwakeup_ops *ops)
{
	list_add(&ops->list, &qw_head);
	return 0;
}
EXPORT_SYMBOL_GPL(quickwakeup_register);

void quickwakeup_unregister(struct quickwakeup_ops *ops)
{
	list_del(&ops->list);
}
EXPORT_SYMBOL_GPL(quickwakeup_unregister);
int quickwakeup_check(void)
{
	int ret = 0;
	struct quickwakeup_ops *index;

	list_for_each_entry(index, &qw_head, list) {
		index->checked = index->qw_check();
		ret |= index->checked;
	}
	return ret;
}

int quickwakeup_execute(void)
{
	int ret = 0;
	int count = 0;
	struct quickwakeup_ops *index;

	list_for_each_entry(index, &qw_head, list) {
		if (index->checked) {
			ret = index->qw_callback();
			index->checked = 0;
			if (ret != 0)
				return ret;
			count++;
		}
	}
	if (!count)
		return -1;
	return 0;
}
