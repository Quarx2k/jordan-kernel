/*
 *  fs/proc/socinfo.c
 *
 *  Copyright (C) 2010 Nokia Corporation
 *
 *  Contact: Eduardo Valentin <eduardo.valen...@nokia.com>
 *
 *  proc socinfo file
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/socinfo.h>

/*
 * Function pointer to soc core code which knows how to grab soc info
 */
static int (*socinfo_show)(struct seq_file *, void *);
static void *socinfo_data;
static DEFINE_MUTEX(socinfo_mutex);

/**
 * register_socinfo_show() - register a call back to provide SoC information
 * @show:      The function callback. It is expected to be in the same format
 *             as the .show of struct seq_operations.
 * @data:      A void * which will be passed as argument when show is called.
 *
 * This function will store the reference for a function and its data. The show
 * argument will be called when filling up the seq_file of /proc/socinfo.
 * Usually, this function should be called just once, while executing the SoC
 * core initialization code.
 */
void register_socinfo_show(int (*show)(struct seq_file *, void *), void *data)
{
	mutex_lock(&socinfo_mutex);
	socinfo_show = show;
	socinfo_data = data;
	mutex_unlock(&socinfo_mutex);
}

static int socinfo_show_local(struct seq_file *sfile, void *data)
{
	int r;

	/* Just fall back to those who know how to grab the info */
	mutex_lock(&socinfo_mutex);
	if (socinfo_show)
		r = socinfo_show(sfile, socinfo_data);
	else
		r = seq_printf(sfile, "No data\n");
	mutex_unlock(&socinfo_mutex);

	return r;
}

static int socinfo_open(struct inode *inode, struct file *file)
{
	return single_open(file, socinfo_show_local, NULL);
}

static const struct file_operations proc_socinfo_operations = {
	.owner          = THIS_MODULE,
	.open           = socinfo_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __init proc_socinfo_init(void)
{
	if (!proc_create("socinfo", 0, NULL, &proc_socinfo_operations)) {
		pr_info("Failed to create /proc/socinfo\n");
		return -ENOMEM;
	}

	return 0;
}
module_init(proc_socinfo_init);
