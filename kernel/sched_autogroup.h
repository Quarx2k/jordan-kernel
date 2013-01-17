#ifdef CONFIG_SCHED_AUTOGROUP

static inline struct task_group *
autogroup_task_group(struct task_struct *p, struct task_group *tg);

#else /* !CONFIG_SCHED_AUTOGROUP */

static inline void autogroup_init(struct task_struct *init_task) {  }

static inline struct task_group *
autogroup_task_group(struct task_struct *p, struct task_group *tg)
{
	return tg;
}

#ifdef CONFIG_SCHED_DEBUG
static inline int autogroup_path(struct task_group *tg, char *buf, int buflen)
{
	return 0;
}
#endif

#endif /* CONFIG_SCHED_AUTOGROUP */
