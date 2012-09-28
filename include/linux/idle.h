/*
 * include/linux/idle.h - generic idle definition
 *
 */
#ifndef _LINUX_IDLE_H_
#define _LINUX_IDLE_H_

#include <linux/notifier.h>

enum idle_val {
	IDLE_START = 1,
	IDLE_END = 2,
};

int notify_idle(enum idle_val val);
void register_idle_notifier(struct notifier_block *n);
void unregister_idle_notifier(struct notifier_block *n);

#endif /* _LINUX_IDLE_H_ */
