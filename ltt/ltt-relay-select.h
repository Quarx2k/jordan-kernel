#ifndef _LTT_LTT_RELAY_SELECT_H
#define _LTT_LTT_RELAY_SELECT_H

#ifdef CONFIG_LTT_RELAY_LOCKLESS
#include "ltt-relay-lockless.h"
#elif defined(CONFIG_LTT_RELAY_IRQOFF)
#include "ltt-relay-irqoff.h"
#elif defined(CONFIG_LTT_RELAY_LOCKED)
#include "ltt-relay-locked.h"
#else
#error "At least one LTTng relay transport should be selected."
#endif

#endif /* _LTT_LTT_RELAY_SELECT_H */
