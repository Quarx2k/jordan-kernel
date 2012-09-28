#ifndef _TRACE_RCU_H
#define _TRACE_RCU_H

#include <linux/tracepoint.h>
#include <linux/rcupdate.h>

DECLARE_TRACE(rcu_classic_callback,
	TP_PROTO(struct rcu_head *head),
	TP_ARGS(head));

DECLARE_TRACE(rcu_classic_call_rcu,
	TP_PROTO(struct rcu_head *head, unsigned long ip),
	TP_ARGS(head, ip));

DECLARE_TRACE(rcu_classic_call_rcu_bh,
	TP_PROTO(struct rcu_head *head, unsigned long ip),
	TP_ARGS(head, ip));

DECLARE_TRACE(rcu_preempt_callback,
	TP_PROTO(struct rcu_head *head),
	TP_ARGS(head));

DECLARE_TRACE(rcu_preempt_call_rcu,
	TP_PROTO(struct rcu_head *head, unsigned long ip),
	TP_ARGS(head, ip));

DECLARE_TRACE(rcu_preempt_call_rcu_sched,
	TP_PROTO(struct rcu_head *head, unsigned long ip),
	TP_ARGS(head, ip));

DECLARE_TRACE(rcu_tree_callback,
	TP_PROTO(struct rcu_head *head),
	TP_ARGS(head));

DECLARE_TRACE(rcu_tree_call_rcu,
	TP_PROTO(struct rcu_head *head, unsigned long ip),
	TP_ARGS(head, ip));

DECLARE_TRACE(rcu_tree_call_rcu_bh,
	TP_PROTO(struct rcu_head *head, unsigned long ip),
	TP_ARGS(head, ip));

#endif
