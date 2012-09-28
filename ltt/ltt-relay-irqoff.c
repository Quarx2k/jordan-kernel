/*
 * ltt/ltt-relay-irqoff.c
 *
 * (C) Copyright 2005-2008 - Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * LTTng irqoff buffer space management (reader/writer).
 *
 * Author:
 *	Mathieu Desnoyers (mathieu.desnoyers@polymtl.ca)
 *
 * Inspired from LTT :
 *  Karim Yaghmour (karim@opersys.com)
 *  Tom Zanussi (zanussi@us.ibm.com)
 *  Bob Wisniewski (bob@watson.ibm.com)
 * And from K42 :
 *  Bob Wisniewski (bob@watson.ibm.com)
 *
 * Changelog:
 *  08/10/08, Cleanup.
 *  27/05/05, Modular redesign and rewrite.
 *
 * Userspace reader semantic :
 * while (poll fd != POLLHUP) {
 *   - ioctl RELAY_GET_SUBBUF_SIZE
 *   while (1) {
 *     - ioctl GET_SUBBUF
 *     - splice 1 subbuffer worth of data to a pipe
 *     - splice the data from pipe to disk/network
 *     - ioctl PUT_SUBBUF, check error value
 *       if err val < 0, previous subbuffer was corrupted.
 *   }
 * }
 *
 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/time.h>
#include <linux/ltt-tracer.h>
#include <linux/ltt-relay.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/rcupdate.h>
#include <linux/timer.h>
#include <linux/sched.h>
#include <linux/bitops.h>
#include <linux/fs.h>
#include <linux/smp_lock.h>
#include <linux/debugfs.h>
#include <linux/stat.h>
#include <linux/cpu.h>
#include <linux/pipe_fs_i.h>
#include <linux/splice.h>
#include <asm/atomic.h>
#include <asm/local.h>

#include "ltt-relay-irqoff.h"

#if 0
#define printk_dbg(fmt, args...) printk(fmt, args)
#else
#define printk_dbg(fmt, args...)
#endif

struct ltt_reserve_switch_offsets {
	long begin, end, old;
	long begin_switch, end_switch_current, end_switch_old;
	size_t before_hdr_pad, size;
};

static int ltt_relay_create_buffer(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_chan,
		struct rchan_buf *buf,
		unsigned int cpu,
		unsigned int n_subbufs);

static void ltt_relay_destroy_buffer(struct ltt_channel_struct *ltt_chan,
		unsigned int cpu);

static void ltt_force_switch(struct rchan_buf *buf,
		enum force_switch_mode mode);

static const struct file_operations ltt_file_operations;

static void ltt_buffer_begin(struct rchan_buf *buf,
			u64 tsc, unsigned int subbuf_idx)
{
	struct ltt_channel_struct *channel =
		(struct ltt_channel_struct *)buf->chan->private_data;
	struct ltt_subbuffer_header *header =
		(struct ltt_subbuffer_header *)
			ltt_relay_offset_address(buf,
				subbuf_idx * buf->chan->subbuf_size);

	header->cycle_count_begin = tsc;
	header->lost_size = 0xFFFFFFFF; /* for debugging */
	header->buf_size = buf->chan->subbuf_size;
	ltt_write_trace_header(channel->trace, header);
}

/*
 * offset is assumed to never be 0 here : never deliver a completely empty
 * subbuffer. The lost size is between 0 and subbuf_size-1.
 */
static void ltt_buffer_end(struct rchan_buf *buf,
		u64 tsc, unsigned int offset, unsigned int subbuf_idx)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	struct ltt_subbuffer_header *header =
		(struct ltt_subbuffer_header *)
			ltt_relay_offset_address(buf,
				subbuf_idx * buf->chan->subbuf_size);

	header->lost_size = SUBBUF_OFFSET((buf->chan->subbuf_size - offset),
				buf->chan);
	header->cycle_count_end = tsc;
	header->events_lost = local_read(&ltt_buf->events_lost);
	header->subbuf_corrupt = local_read(&ltt_buf->corrupted_subbuffers);
}

static struct dentry *ltt_create_buf_file_callback(const char *filename,
		struct dentry *parent, int mode,
		struct rchan_buf *buf)
{
	struct ltt_channel_struct *ltt_chan;
	int err;
	struct dentry *dentry;

	ltt_chan = buf->chan->private_data;
	err = ltt_relay_create_buffer(ltt_chan->trace, ltt_chan,
					buf, buf->cpu,
					buf->chan->n_subbufs);
	if (err)
		return ERR_PTR(err);

	dentry = debugfs_create_file(filename, mode, parent, buf,
			&ltt_file_operations);
	if (!dentry)
		goto error;
	if (buf->cpu == 0)
		buf->ascii_dentry = ltt_ascii_create(ltt_chan->trace, ltt_chan);
	return dentry;
error:
	ltt_relay_destroy_buffer(ltt_chan, buf->cpu);
	return NULL;
}

static int ltt_remove_buf_file_callback(struct dentry *dentry)
{
	struct rchan_buf *buf = dentry->d_inode->i_private;
	struct ltt_channel_struct *ltt_chan = buf->chan->private_data;

	ltt_ascii_remove(ltt_chan, buf->ascii_dentry);
	debugfs_remove(dentry);
	ltt_relay_destroy_buffer(ltt_chan, buf->cpu);

	return 0;
}

/*
 * Wake writers :
 *
 * This must be done after the trace is removed from the RCU list so that there
 * are no stalled writers.
 */
static void ltt_relay_wake_writers(struct ltt_channel_buf_struct *ltt_buf)
{

	if (waitqueue_active(&ltt_buf->write_wait))
		wake_up_interruptible(&ltt_buf->write_wait);
}

/*
 * This function should not be called from NMI interrupt context
 */
static void ltt_buf_unfull(struct rchan_buf *buf,
		unsigned int subbuf_idx,
		long offset)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	ltt_relay_wake_writers(ltt_buf);
}

/*
 * Reader API.
 */
static unsigned long get_offset(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	return local_read(&ltt_buf->offset);
}

static unsigned long get_consumed(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	return atomic_long_read(&ltt_buf->consumed);
}

static int _ltt_open(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	if (!atomic_long_add_unless(&ltt_buf->active_readers, 1, 1))
		return -EBUSY;
	ltt_relay_get_chan(buf->chan);
	return 0;
}

static int _ltt_release(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	ltt_relay_put_chan(buf->chan);
	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
	atomic_long_dec(&ltt_buf->active_readers);
	return 0;
}

static int is_finalized(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	return ltt_buf->finalized;
}

/*
 * Promote compiler barrier to a smp_mb().
 * For the specific LTTng case, this IPI call should be removed if the
 * architecture does not reorder writes.  This should eventually be provided by
 * a separate architecture-specific infrastructure.
 */
static void remote_mb(void *info)
{
	smp_mb();
}

static int get_subbuf(struct rchan_buf *buf, unsigned long *consumed)
{
	struct ltt_channel_struct *ltt_channel =
		(struct ltt_channel_struct *)buf->chan->private_data;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	long consumed_old, consumed_idx, commit_count, write_offset;
	int ret;

	consumed_old = atomic_long_read(&ltt_buf->consumed);
	consumed_idx = SUBBUF_INDEX(consumed_old, buf->chan);
	commit_count = local_read(&ltt_buf->commit_count[consumed_idx]);
	/*
	 * Make sure we read the commit count before reading the buffer
	 * data and the write offset. Correct consumed offset ordering
	 * wrt commit count is insured by the use of cmpxchg to update
	 * the consumed offset.
	 * smp_call_function_single can fail if the remote CPU is offline,
	 * this is OK because then there is no wmb to execute there.
	 * If our thread is executing on the same CPU as the on the buffers
	 * belongs to, we don't have to synchronize it at all. If we are
	 * migrated, the scheduler will take care of the memory barriers.
	 * Normally, smp_call_function_single() should ensure program order when
	 * executing the remote function, which implies that it surrounds the
	 * function execution with :
	 * smp_mb()
	 * send IPI
	 * csd_lock_wait
	 *                recv IPI
	 *                smp_mb()
	 *                exec. function
	 *                smp_mb()
	 *                csd unlock
	 * smp_mb()
	 *
	 * However, smp_call_function_single() does not seem to clearly execute
	 * such barriers. It depends on spinlock semantic to provide the barrier
	 * before executing the IPI and, when busy-looping, csd_lock_wait only
	 * executes smp_mb() when it has to wait for the other CPU.
	 *
	 * I don't trust this code. Therefore, let's add the smp_mb() sequence
	 * required ourself, even if duplicated. It has no performance impact
	 * anyway.
	 *
	 * smp_mb() is needed because smp_rmb() and smp_wmb() only order read vs
	 * read and write vs write. They do not ensure core synchronization. We
	 * really have to ensure total order between the 3 barriers running on
	 * the 2 CPUs.
	 */
#ifdef LTT_NO_IPI_BARRIER
	/*
	 * Local rmb to match the remote wmb to read the commit count before the
	 * buffer data and the write offset.
	 */
	smp_rmb();
#else
	if (raw_smp_processor_id() != buf->cpu) {
		smp_mb();	/* Total order with IPI handler smp_mb() */
		smp_call_function_single(buf->cpu, remote_mb, NULL, 1);
		smp_mb();	/* Total order with IPI handler smp_mb() */
	}
#endif
	write_offset = local_read(&ltt_buf->offset);
	/*
	 * Check that the subbuffer we are trying to consume has been
	 * already fully committed.
	 */
	if (((commit_count - buf->chan->subbuf_size)
	     & ltt_channel->commit_count_mask)
	    - (BUFFER_TRUNC(consumed_old, buf->chan)
	       >> ltt_channel->n_subbufs_order)
	    != 0) {
		return -EAGAIN;
	}
	/*
	 * Check that we are not about to read the same subbuffer in
	 * which the writer head is.
	 */
	if ((SUBBUF_TRUNC(write_offset, buf->chan)
	   - SUBBUF_TRUNC(consumed_old, buf->chan))
	   == 0) {
		return -EAGAIN;
	}

	ret = update_read_sb_index(buf, consumed_idx);
	if (ret)
		return ret;

	*consumed = consumed_old;
	return 0;
}

static int put_subbuf(struct rchan_buf *buf, unsigned long consumed)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	long consumed_new, consumed_old;

	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);

	consumed_old = consumed;
	consumed_new = SUBBUF_ALIGN(consumed_old, buf->chan);
	WARN_ON_ONCE(RCHAN_SB_IS_NOREF(buf->rchan_rsb.pages));
	RCHAN_SB_SET_NOREF(buf->rchan_rsb.pages);

	spin_lock(&ltt_buf->full_lock);
	if (atomic_long_cmpxchg(&ltt_buf->consumed, consumed_old,
				consumed_new)
	    != consumed_old) {
		/* We have been pushed by the writer. */
		spin_unlock(&ltt_buf->full_lock);
		/*
		 * We exchanged the subbuffer pages. No corruption possible
		 * even if the writer did push us. No more -EIO possible.
		 */
		return 0;
	} else {
		/* tell the client that buffer is now unfull */
		int index;
		long data;
		index = SUBBUF_INDEX(consumed_old, buf->chan);
		data = BUFFER_OFFSET(consumed_old, buf->chan);
		ltt_buf_unfull(buf, index, data);
		spin_unlock(&ltt_buf->full_lock);
	}
	return 0;
}

static unsigned long get_n_subbufs(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
	return buf->chan->n_subbufs;
}

static unsigned long get_subbuf_size(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
	return buf->chan->subbuf_size;
}

static void switch_buffer(unsigned long data)
{
	struct ltt_channel_buf_struct *ltt_buf =
		(struct ltt_channel_buf_struct *)data;
	struct rchan_buf *buf = ltt_buf->rbuf;

	if (buf)
		ltt_force_switch(buf, FORCE_ACTIVE);

	ltt_buf->switch_timer.expires += ltt_buf->switch_timer_interval;
	add_timer_on(&ltt_buf->switch_timer, smp_processor_id());
}

static void start_switch_timer(struct ltt_channel_struct *ltt_channel)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	int cpu;

	if (!ltt_channel->switch_timer_interval)
		return;

	// TODO : hotplug
	for_each_online_cpu(cpu) {
		struct ltt_channel_buf_struct *ltt_buf;
		struct rchan_buf *buf;

		buf = rchan->buf[cpu];
		ltt_buf = buf->chan_private;
		buf->random_access = 1;
		ltt_buf->switch_timer_interval =
			ltt_channel->switch_timer_interval;
		init_timer(&ltt_buf->switch_timer);
		ltt_buf->switch_timer.function = switch_buffer;
		ltt_buf->switch_timer.expires = jiffies +
					ltt_buf->switch_timer_interval;
		ltt_buf->switch_timer.data = (unsigned long)ltt_buf;
		add_timer_on(&ltt_buf->switch_timer, cpu);
	}
}

/*
 * Cannot use del_timer_sync with add_timer_on, so use an IPI to locally
 * delete the timer.
 */
static void stop_switch_timer_ipi(void *info)
{
	struct ltt_channel_buf_struct *ltt_buf =
		(struct ltt_channel_buf_struct *)info;

	del_timer(&ltt_buf->switch_timer);
}

static void stop_switch_timer(struct ltt_channel_struct *ltt_channel)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	int cpu;

	if (!ltt_channel->switch_timer_interval)
		return;

	// TODO : hotplug
	for_each_online_cpu(cpu) {
		struct ltt_channel_buf_struct *ltt_buf;
		struct rchan_buf *buf;

		buf = rchan->buf[cpu];
		ltt_buf = buf->chan_private;
		smp_call_function(stop_switch_timer_ipi, ltt_buf, 1);
		buf->random_access = 0;
	}
}

static struct ltt_channel_buf_access_ops ltt_channel_buf_accessor = {
	.get_offset   = get_offset,
	.get_consumed = get_consumed,
	.get_subbuf = get_subbuf,
	.put_subbuf = put_subbuf,
	.is_finalized = is_finalized,
	.get_n_subbufs = get_n_subbufs,
	.get_subbuf_size = get_subbuf_size,
	.open = _ltt_open,
	.release = _ltt_release,
	.start_switch_timer = start_switch_timer,
	.stop_switch_timer = stop_switch_timer,
};

/**
 *	ltt_open - open file op for ltt files
 *	@inode: opened inode
 *	@file: opened file
 *
 *	Open implementation. Makes sure only one open instance of a buffer is
 *	done at a given moment.
 */
static int ltt_open(struct inode *inode, struct file *file)
{
	int ret;
	struct rchan_buf *buf = inode->i_private;

	ret = _ltt_open(buf);
	if (!ret)
		ret = ltt_relay_file_operations.open(inode, file);
	return ret;
}

/**
 *	ltt_release - release file op for ltt files
 *	@inode: opened inode
 *	@file: opened file
 *
 *	Release implementation.
 */
static int ltt_release(struct inode *inode, struct file *file)
{
	struct rchan_buf *buf = inode->i_private;
	int ret;

	_ltt_release(buf);
	ret = ltt_relay_file_operations.release(inode, file);
	WARN_ON(ret);
	return ret;
}

/**
 *	ltt_poll - file op for ltt files
 *	@filp: the file
 *	@wait: poll table
 *
 *	Poll implementation.
 */
static unsigned int ltt_poll(struct file *filp, poll_table *wait)
{
	unsigned int mask = 0;
	struct inode *inode = filp->f_dentry->d_inode;
	struct rchan_buf *buf = inode->i_private;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	if (filp->f_mode & FMODE_READ) {
		poll_wait_set_exclusive(wait);
		poll_wait(filp, &ltt_buf->read_wait, wait);

		WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
		if (SUBBUF_TRUNC(local_read(&ltt_buf->offset),
							buf->chan)
		  - SUBBUF_TRUNC(atomic_long_read(&ltt_buf->consumed),
							buf->chan)
		  == 0) {
			if (ltt_buf->finalized)
				return POLLHUP;
			else
				return 0;
		} else {
			struct rchan *rchan = buf->chan;
			if (SUBBUF_TRUNC(local_read(&ltt_buf->offset),
					buf->chan)
			  - SUBBUF_TRUNC(atomic_long_read(
						&ltt_buf->consumed),
					buf->chan)
			  >= rchan->alloc_size)
				return POLLPRI | POLLRDBAND;
			else
				return POLLIN | POLLRDNORM;
		}
	}
	return mask;
}

/**
 *	ltt_ioctl - control on the debugfs file
 *
 *	@inode: the inode
 *	@filp: the file
 *	@cmd: the command
 *	@arg: command arg
 *
 *	This ioctl implements three commands necessary for a minimal
 *	producer/consumer implementation :
 *	RELAY_GET_SUBBUF
 *		Get the next sub buffer that can be read. It never blocks.
 *	RELAY_PUT_SUBBUF
 *		Release the currently read sub-buffer. Parameter is the last
 *		put subbuffer (returned by GET_SUBBUF).
 *	RELAY_GET_N_BUBBUFS
 *		returns the number of sub buffers in the per cpu channel.
 *	RELAY_GET_SUBBUF_SIZE
 *		returns the size of the sub buffers.
 */
static int ltt_ioctl(struct inode *inode, struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	struct rchan_buf *buf = inode->i_private;
	u32 __user *argp = (u32 __user *)arg;

	switch (cmd) {
	case RELAY_GET_SUBBUF:
	{
		unsigned long consumed;
		int ret;

		ret = get_subbuf(buf, &consumed);
		if (ret)
			return ret;
		else
			return put_user((u32)consumed, argp);
		break;
	}
	case RELAY_PUT_SUBBUF:
	{
		struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
		u32 uconsumed_old;
		int ret;
		long consumed_old;

		ret = get_user(uconsumed_old, argp);
		if (ret)
			return ret; /* will return -EFAULT */

		consumed_old = atomic_long_read(&ltt_buf->consumed);
		consumed_old = consumed_old & (~0xFFFFFFFFL);
		consumed_old = consumed_old | uconsumed_old;
		ret = put_subbuf(buf, consumed_old);
		if (ret)
			return ret;
		break;
	}
	case RELAY_GET_N_SUBBUFS:
		return put_user((u32)get_n_subbufs(buf), argp);
		break;
	case RELAY_GET_SUBBUF_SIZE:
		return put_user((u32)get_subbuf_size(buf), argp);
		break;
	default:
		return -ENOIOCTLCMD;
	}
	return 0;
}

#ifdef CONFIG_COMPAT
static long ltt_compat_ioctl(struct file *file, unsigned int cmd,
		unsigned long arg)
{
	long ret = -ENOIOCTLCMD;

	lock_kernel();
	ret = ltt_ioctl(file->f_dentry->d_inode, file, cmd, arg);
	unlock_kernel();

	return ret;
}
#endif

static void ltt_relay_pipe_buf_release(struct pipe_inode_info *pipe,
				   struct pipe_buffer *pbuf)
{
}

static struct pipe_buf_operations ltt_relay_pipe_buf_ops = {
	.can_merge = 0,
	.map = generic_pipe_buf_map,
	.unmap = generic_pipe_buf_unmap,
	.confirm = generic_pipe_buf_confirm,
	.release = ltt_relay_pipe_buf_release,
	.steal = generic_pipe_buf_steal,
	.get = generic_pipe_buf_get,
};

static void ltt_relay_page_release(struct splice_pipe_desc *spd, unsigned int i)
{
}

/*
 *	subbuf_splice_actor - splice up to one subbuf's worth of data
 */
static int subbuf_splice_actor(struct file *in,
			       loff_t *ppos,
			       struct pipe_inode_info *pipe,
			       size_t len,
			       unsigned int flags)
{
	struct rchan_buf *buf = in->private_data;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	unsigned int poff, subbuf_pages, nr_pages;
	struct page *pages[PIPE_BUFFERS];
	struct partial_page partial[PIPE_BUFFERS];
	struct splice_pipe_desc spd = {
		.pages = pages,
		.nr_pages = 0,
		.partial = partial,
		.flags = flags,
		.ops = &ltt_relay_pipe_buf_ops,
		.spd_release = ltt_relay_page_release,
	};
	long consumed_old, consumed_idx, roffset;
	unsigned long bytes_avail;

	/*
	 * Check that a GET_SUBBUF ioctl has been done before.
	 */
	WARN_ON(atomic_long_read(&ltt_buf->active_readers) != 1);
	consumed_old = atomic_long_read(&ltt_buf->consumed);
	consumed_old += *ppos;
	consumed_idx = SUBBUF_INDEX(consumed_old, buf->chan);

	/*
	 * Adjust read len, if longer than what is available.
	 * Max read size is 1 subbuffer due to get_subbuf/put_subbuf for
	 * protection.
	 */
	bytes_avail = buf->chan->subbuf_size;
	WARN_ON(bytes_avail > buf->chan->alloc_size);
	len = min_t(size_t, len, bytes_avail);
	subbuf_pages = bytes_avail >> PAGE_SHIFT;
	nr_pages = min_t(unsigned int, subbuf_pages, PIPE_BUFFERS);
	roffset = consumed_old & PAGE_MASK;
	poff = consumed_old & ~PAGE_MASK;
	printk_dbg(KERN_DEBUG "SPLICE actor len %zu pos %zd write_pos %ld\n",
		len, (ssize_t)*ppos, local_read(&ltt_buf->offset));

	for (; spd.nr_pages < nr_pages; spd.nr_pages++) {
		unsigned int this_len;
		struct page *page;

		if (!len)
			break;
		printk_dbg(KERN_DEBUG "SPLICE actor loop len %zu roffset %ld\n",
			len, roffset);

		this_len = PAGE_SIZE - poff;
		page = ltt_relay_read_get_page(buf, roffset);
		spd.pages[spd.nr_pages] = page;
		spd.partial[spd.nr_pages].offset = poff;
		spd.partial[spd.nr_pages].len = this_len;

		poff = 0;
		roffset += PAGE_SIZE;
		len -= this_len;
	}

	if (!spd.nr_pages)
		return 0;

	return splice_to_pipe(pipe, &spd);
}

static ssize_t ltt_relay_file_splice_read(struct file *in,
				      loff_t *ppos,
				      struct pipe_inode_info *pipe,
				      size_t len,
				      unsigned int flags)
{
	ssize_t spliced;
	int ret;

	ret = 0;
	spliced = 0;

	printk_dbg(KERN_DEBUG "SPLICE read len %zu pos %zd\n",
		len, (ssize_t)*ppos);
	while (len && !spliced) {
		ret = subbuf_splice_actor(in, ppos, pipe, len, flags);
		printk_dbg(KERN_DEBUG "SPLICE read loop ret %d\n", ret);
		if (ret < 0)
			break;
		else if (!ret) {
			if (flags & SPLICE_F_NONBLOCK)
				ret = -EAGAIN;
			break;
		}

		*ppos += ret;
		if (ret > len)
			len = 0;
		else
			len -= ret;
		spliced += ret;
	}

	if (spliced)
		return spliced;

	return ret;
}

static void ltt_relay_print_subbuffer_errors(
		struct ltt_channel_struct *ltt_chan,
		long cons_off, unsigned int cpu)
{
	struct rchan *rchan = ltt_chan->trans_channel_data;
	struct ltt_channel_buf_struct *ltt_buf = rchan->buf[cpu]->chan_private;
	long cons_idx, commit_count, write_offset;

	cons_idx = SUBBUF_INDEX(cons_off, rchan);
	commit_count = local_read(&ltt_buf->commit_count[cons_idx]);
	/*
	 * No need to order commit_count and write_offset reads because we
	 * execute after trace is stopped when there are no readers left.
	 */
	write_offset = local_read(&ltt_buf->offset);
	printk(KERN_WARNING
		"LTT : unread channel %s offset is %ld "
		"and cons_off : %ld (cpu %u)\n",
		ltt_chan->channel_name, write_offset, cons_off, cpu);
	/* Check each sub-buffer for non filled commit count */
	if (((commit_count - rchan->subbuf_size) & ltt_chan->commit_count_mask)
	    - (BUFFER_TRUNC(cons_off, rchan) >> ltt_chan->n_subbufs_order)
	    != 0)
		printk(KERN_ALERT
			"LTT : %s : subbuffer %lu has non filled "
			"commit count %lu.\n",
			ltt_chan->channel_name, cons_idx, commit_count);
	printk(KERN_ALERT "LTT : %s : commit count : %lu, subbuf size %zd\n",
			ltt_chan->channel_name, commit_count,
			rchan->subbuf_size);
}

static void ltt_relay_print_errors(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_chan, int cpu)
{
	struct rchan *rchan = ltt_chan->trans_channel_data;
	struct ltt_channel_buf_struct *ltt_buf = rchan->buf[cpu]->chan_private;
	long cons_off;

	/*
	 * Can be called in the error path of allocation when
	 * trans_channel_data is not yet set.
	 */
	if (!rchan)
		return;
	for (cons_off = atomic_long_read(&ltt_buf->consumed);
			(SUBBUF_TRUNC(local_read(&ltt_buf->offset),
				      rchan)
			 - cons_off) > 0;
			cons_off = SUBBUF_ALIGN(cons_off, rchan))
		ltt_relay_print_subbuffer_errors(ltt_chan, cons_off, cpu);
}

static void ltt_relay_print_buffer_errors(struct ltt_channel_struct *ltt_chan,
		unsigned int cpu)
{
	struct ltt_trace_struct *trace = ltt_chan->trace;
	struct rchan *rchan = ltt_chan->trans_channel_data;
	struct ltt_channel_buf_struct *ltt_buf = rchan->buf[cpu]->chan_private;

	if (local_read(&ltt_buf->events_lost))
		printk(KERN_ALERT
			"LTT : %s : %ld events lost "
			"in %s channel (cpu %u).\n",
			ltt_chan->channel_name,
			local_read(&ltt_buf->events_lost),
			ltt_chan->channel_name, cpu);
	if (local_read(&ltt_buf->corrupted_subbuffers))
		printk(KERN_ALERT
			"LTT : %s : %ld corrupted subbuffers "
			"in %s channel (cpu %u).\n",
			ltt_chan->channel_name,
			local_read(&ltt_buf->corrupted_subbuffers),
			ltt_chan->channel_name, cpu);

	ltt_relay_print_errors(trace, ltt_chan, cpu);
}

static void ltt_relay_remove_dirs(struct ltt_trace_struct *trace)
{
	ltt_ascii_remove_dir(trace);
	debugfs_remove(trace->dentry.trace_root);
}

/*
 * Create ltt buffer.
 */
static int ltt_relay_create_buffer(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_chan, struct rchan_buf *buf,
		unsigned int cpu, unsigned int n_subbufs)
{
	struct ltt_channel_buf_struct *ltt_buf;
	unsigned int j;

	ltt_buf = kzalloc_node(sizeof(*ltt_buf), GFP_KERNEL, cpu_to_node(cpu));
	if (!ltt_buf)
		return -ENOMEM;

	ltt_buf->commit_count =
		kzalloc_node(ALIGN(sizeof(*ltt_buf->commit_count) * n_subbufs,
				   1 << INTERNODE_CACHE_SHIFT),
			GFP_KERNEL, cpu_to_node(cpu));
	if (!ltt_buf->commit_count) {
		kfree(ltt_buf);
		return -ENOMEM;
	}

#ifdef CONFIG_LTT_VMCORE
	ltt_buf->commit_seq =
		kzalloc_node(ALIGN(sizeof(*ltt_buf->commit_seq) * n_subbufs,
				   1 << INTERNODE_CACHE_SHIFT),
			GFP_KERNEL, cpu_to_node(cpu));
	if (!ltt_buf->commit_seq) {
		kfree(ltt_buf->commit_count);
		kfree(ltt_buf);
		return -ENOMEM;
	}
#endif

	buf->chan_private = ltt_buf;

	kref_get(&trace->kref);
	kref_get(&trace->ltt_transport_kref);
	local_set(&ltt_buf->offset, ltt_subbuffer_header_size());
	atomic_long_set(&ltt_buf->consumed, 0);
	atomic_long_set(&ltt_buf->active_readers, 0);
	for (j = 0; j < n_subbufs; j++)
		local_set(&ltt_buf->commit_count[j], 0);
	init_waitqueue_head(&ltt_buf->write_wait);
	init_waitqueue_head(&ltt_buf->read_wait);
	spin_lock_init(&ltt_buf->full_lock);

	RCHAN_SB_CLEAR_NOREF(buf->rchan_wsb[0].pages);
	ltt_buffer_begin(buf, trace->start_tsc, 0);
	/* atomic_add made on local variable on data that belongs to
	 * various CPUs : ok because tracing not started (for this cpu). */
	local_add(ltt_subbuffer_header_size(), &ltt_buf->commit_count[0]);

	local_set(&ltt_buf->events_lost, 0);
	local_set(&ltt_buf->corrupted_subbuffers, 0);
	ltt_buf->finalized = 0;
	ltt_buf->rbuf = buf;

	return 0;
}

static void ltt_relay_destroy_buffer(struct ltt_channel_struct *ltt_chan,
		unsigned int cpu)
{
	struct ltt_trace_struct *trace = ltt_chan->trace;
	struct rchan *rchan = ltt_chan->trans_channel_data;
	struct ltt_channel_buf_struct *ltt_buf = rchan->buf[cpu]->chan_private;

	kref_put(&ltt_chan->trace->ltt_transport_kref,
		ltt_release_transport);
	ltt_relay_print_buffer_errors(ltt_chan, cpu);
#ifdef CONFIG_LTT_VMCORE
	kfree(ltt_buf->commit_seq);
#endif
	kfree(ltt_buf->commit_count);
	kfree(ltt_buf);
	kref_put(&trace->kref, ltt_release_trace);
	wake_up_interruptible(&trace->kref_wq);
}

/*
 * Create channel.
 */
static int ltt_relay_create_channel(const char *trace_name,
		struct ltt_trace_struct *trace, struct dentry *dir,
		const char *channel_name, struct ltt_channel_struct *ltt_chan,
		unsigned int subbuf_size, unsigned int n_subbufs,
		int overwrite)
{
	char *tmpname;
	unsigned int tmpname_len;
	int err = 0;

	tmpname = kmalloc(PATH_MAX, GFP_KERNEL);
	if (!tmpname)
		return EPERM;
	if (overwrite) {
		strncpy(tmpname, LTT_FLIGHT_PREFIX, PATH_MAX-1);
		strncat(tmpname, channel_name,
			PATH_MAX-1-sizeof(LTT_FLIGHT_PREFIX));
	} else {
		strncpy(tmpname, channel_name, PATH_MAX-1);
	}
	strncat(tmpname, "_", PATH_MAX-1-strlen(tmpname));

	ltt_chan->trace = trace;
	ltt_chan->overwrite = overwrite;
	ltt_chan->n_subbufs_order = get_count_order(n_subbufs);
	ltt_chan->commit_count_mask = (~0UL >> ltt_chan->n_subbufs_order);
	ltt_chan->trans_channel_data = ltt_relay_open(tmpname,
			dir,
			subbuf_size,
			n_subbufs,
			&trace->callbacks,
			ltt_chan,
			overwrite);
	tmpname_len = strlen(tmpname);
	if (tmpname_len > 0) {
		/* Remove final _ for pretty printing */
		tmpname[tmpname_len-1] = '\0';
	}
	if (ltt_chan->trans_channel_data == NULL) {
		printk(KERN_ERR "LTT : Can't open %s channel for trace %s\n",
				tmpname, trace_name);
		goto relay_open_error;
	}

	ltt_chan->buf_access_ops = &ltt_channel_buf_accessor;

	err = 0;
	goto end;

relay_open_error:
	err = EPERM;
end:
	kfree(tmpname);
	return err;
}

static int ltt_relay_create_dirs(struct ltt_trace_struct *new_trace)
{
	struct dentry *ltt_root_dentry;
	int ret;

	ltt_root_dentry = get_ltt_root();
	if (!ltt_root_dentry)
		return ENOENT;

	new_trace->dentry.trace_root = debugfs_create_dir(new_trace->trace_name,
			ltt_root_dentry);
	put_ltt_root();
	if (new_trace->dentry.trace_root == NULL) {
		printk(KERN_ERR "LTT : Trace directory name %s already taken\n",
				new_trace->trace_name);
		return EEXIST;
	}
	ret = ltt_ascii_create_dir(new_trace);
	if (ret)
		printk(KERN_WARNING "LTT : Unable to create ascii output file "
				    "for trace %s\n", new_trace->trace_name);

	new_trace->callbacks.create_buf_file = ltt_create_buf_file_callback;
	new_trace->callbacks.remove_buf_file = ltt_remove_buf_file_callback;

	return 0;
}

/*
 * LTTng channel flush function.
 *
 * Must be called when no tracing is active in the channel, because of
 * accesses across CPUs.
 */
static notrace void ltt_relay_buffer_flush(struct rchan_buf *buf)
{
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;

	ltt_buf->finalized = 1;
	ltt_force_switch(buf, FORCE_FLUSH);
}

static void ltt_relay_async_wakeup_chan(struct ltt_channel_struct *ltt_channel)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	unsigned int i;

	for_each_possible_cpu(i) {
		struct ltt_channel_buf_struct *ltt_buf;

		if (!rchan->buf[i])
			continue;

		ltt_buf = rchan->buf[i]->chan_private;
		if (ltt_poll_deliver(ltt_channel, ltt_buf,
				     rchan, rchan->buf[i]))
			wake_up_interruptible(&ltt_buf->read_wait);
	}
}

static void ltt_relay_finish_buffer(struct ltt_channel_struct *ltt_channel,
		unsigned int cpu)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;

	if (rchan->buf[cpu]) {
		struct ltt_channel_buf_struct *ltt_buf =
				rchan->buf[cpu]->chan_private;
		ltt_relay_buffer_flush(rchan->buf[cpu]);
		ltt_relay_wake_writers(ltt_buf);
	}
}


static void ltt_relay_finish_channel(struct ltt_channel_struct *ltt_channel)
{
	unsigned int i;

	for_each_possible_cpu(i)
		ltt_relay_finish_buffer(ltt_channel, i);
}

static void ltt_relay_remove_channel(struct ltt_channel_struct *channel)
{
	struct rchan *rchan = channel->trans_channel_data;

	ltt_relay_close(rchan);
}

/*
 * This is called with preemption disabled when user space has requested
 * blocking mode.  If one of the active traces has free space below a
 * specific threshold value, we reenable preemption and block.
 */
static int ltt_relay_user_blocking(struct ltt_trace_struct *trace,
		unsigned int chan_index, size_t data_size,
		struct user_dbg_data *dbg)
{
	struct rchan *rchan;
	struct ltt_channel_buf_struct *ltt_buf;
	struct ltt_channel_struct *channel;
	struct rchan_buf *relay_buf;
	int cpu;
	DECLARE_WAITQUEUE(wait, current);

	channel = &trace->channels[chan_index];
	rchan = channel->trans_channel_data;
	cpu = smp_processor_id();
	relay_buf = rchan->buf[cpu];
	ltt_buf = relay_buf->chan_private;

	/*
	 * Check if data is too big for the channel : do not
	 * block for it.
	 */
	if (LTT_RESERVE_CRITICAL + data_size > relay_buf->chan->subbuf_size)
		return 0;

	/*
	 * If free space too low, we block. We restart from the
	 * beginning after we resume (cpu id may have changed
	 * while preemption is active).
	 */
	spin_lock(&ltt_buf->full_lock);
	if (!channel->overwrite) {
		dbg->write = local_read(&ltt_buf->offset);
		dbg->read = atomic_long_read(&ltt_buf->consumed);
		dbg->avail_size = dbg->write + LTT_RESERVE_CRITICAL + data_size
				  - SUBBUF_TRUNC(dbg->read,
						 relay_buf->chan);
		if (dbg->avail_size > rchan->alloc_size) {
			__set_current_state(TASK_INTERRUPTIBLE);
			add_wait_queue(&ltt_buf->write_wait, &wait);
			spin_unlock(&ltt_buf->full_lock);
			preempt_enable();
			schedule();
			__set_current_state(TASK_RUNNING);
			remove_wait_queue(&ltt_buf->write_wait, &wait);
			if (signal_pending(current))
				return -ERESTARTSYS;
			preempt_disable();
			return 1;
		}
	}
	spin_unlock(&ltt_buf->full_lock);
	return 0;
}

static void ltt_relay_print_user_errors(struct ltt_trace_struct *trace,
		unsigned int chan_index, size_t data_size,
		struct user_dbg_data *dbg, int cpu)
{
	struct rchan *rchan;
	struct ltt_channel_buf_struct *ltt_buf;
	struct ltt_channel_struct *channel;
	struct rchan_buf *relay_buf;

	channel = &trace->channels[chan_index];
	rchan = channel->trans_channel_data;
	relay_buf = rchan->buf[cpu];
	ltt_buf = relay_buf->chan_private;

	printk(KERN_ERR "Error in LTT usertrace : "
	"buffer full : event lost in blocking "
	"mode. Increase LTT_RESERVE_CRITICAL.\n");
	printk(KERN_ERR "LTT nesting level is %u.\n",
		per_cpu(ltt_nesting, cpu));
	printk(KERN_ERR "LTT avail size %lu.\n",
		dbg->avail_size);
	printk(KERN_ERR "avai write : %lu, read : %lu\n",
			dbg->write, dbg->read);

	dbg->write = local_read(&ltt_buf->offset);
	dbg->read = atomic_long_read(&ltt_buf->consumed);

	printk(KERN_ERR "LTT cur size %lu.\n",
		dbg->write + LTT_RESERVE_CRITICAL + data_size
		- SUBBUF_TRUNC(dbg->read, relay_buf->chan));
	printk(KERN_ERR "cur write : %lu, read : %lu\n",
			dbg->write, dbg->read);
}

static void ltt_reserve_push_reader(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf,
		struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets)
{
	long consumed_old, consumed_new;

	do {
		consumed_old = atomic_long_read(&ltt_buf->consumed);
		/*
		 * If buffer is in overwrite mode, push the reader consumed
		 * count if the write position has reached it and we are not
		 * at the first iteration (don't push the reader farther than
		 * the writer). This operation can be done concurrently by many
		 * writers in the same buffer, the writer being at the farthest
		 * write position sub-buffer index in the buffer being the one
		 * which will win this loop.
		 * If the buffer is not in overwrite mode, pushing the reader
		 * only happens if a sub-buffer is corrupted.
		 */
		if (unlikely((SUBBUF_TRUNC(offsets->end-1, buf->chan)
		   - SUBBUF_TRUNC(consumed_old, buf->chan))
		   >= rchan->alloc_size))
			consumed_new = SUBBUF_ALIGN(consumed_old, buf->chan);
		else
			return;
	} while (unlikely(atomic_long_cmpxchg(&ltt_buf->consumed, consumed_old,
			consumed_new) != consumed_old));
}


/*
 * ltt_reserve_switch_old_subbuf: switch old subbuffer
 *
 * Concurrency safe because we are the last and only thread to alter this
 * sub-buffer. As long as it is not delivered and read, no other thread can
 * alter the offset, alter the reserve_count or call the
 * client_buffer_end_callback on this sub-buffer.
 *
 * The only remaining threads could be the ones with pending commits. They will
 * have to do the deliver themselves.  Not concurrency safe in overwrite mode.
 * We detect corrupted subbuffers with commit and reserve counts. We keep a
 * corrupted sub-buffers count and push the readers across these sub-buffers.
 *
 * Not concurrency safe if a writer is stalled in a subbuffer and another writer
 * switches in, finding out it's corrupted.  The result will be than the old
 * (uncommited) subbuffer will be declared corrupted, and that the new subbuffer
 * will be declared corrupted too because of the commit count adjustment.
 *
 * Note : offset_old should never be 0 here.
 */
static void ltt_reserve_switch_old_subbuf(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets, u64 *tsc)
{
	long oldidx = SUBBUF_INDEX(offsets->old - 1, rchan);
	long commit_count, padding_size;

	padding_size = rchan->subbuf_size
			- (SUBBUF_OFFSET(offsets->old - 1, rchan) + 1);
	ltt_buffer_end(buf, *tsc, offsets->old, oldidx);
	/*
	 * Must write slot data before incrementing commit count.
	 * This compiler barrier is upgraded into a smp_wmb() by the IPI
	 * sent by get_subbuf() when it does its smp_rmb().
	 */
	barrier();
	commit_count = local_read(&ltt_buf->commit_count[oldidx])
				  + padding_size;
	local_set(&ltt_buf->commit_count[oldidx], commit_count);
	ltt_check_deliver(ltt_channel, ltt_buf, rchan, buf,
		offsets->old - 1, commit_count, oldidx);
	ltt_write_commit_counter(buf, ltt_buf, oldidx,
		offsets->old, commit_count, padding_size);
}

/*
 * ltt_reserve_switch_new_subbuf: Populate new subbuffer.
 *
 * This code can be executed unordered : writers may already have written to the
 * sub-buffer before this code gets executed, caution.  The commit makes sure
 * that this code is executed before the deliver of this sub-buffer.
 */
static void ltt_reserve_switch_new_subbuf(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets, u64 *tsc)
{
	long beginidx = SUBBUF_INDEX(offsets->begin, rchan);
	long commit_count;

	ltt_buffer_begin(buf, *tsc, beginidx);
	/*
	 * Must write slot data before incrementing commit count.
	 * This compiler barrier is upgraded into a smp_wmb() by the IPI
	 * sent by get_subbuf() when it does its smp_rmb().
	 */
	barrier();
	commit_count = local_read(&ltt_buf->commit_count[beginidx])
				  + ltt_subbuffer_header_size();
	local_set(&ltt_buf->commit_count[beginidx], commit_count);
	ltt_check_deliver(ltt_channel, ltt_buf, rchan, buf,
		offsets->begin, commit_count, beginidx);
	ltt_write_commit_counter(buf, ltt_buf, beginidx,
		offsets->begin, commit_count, ltt_subbuffer_header_size());
}


/*
 * ltt_reserve_end_switch_current: finish switching current subbuffer
 *
 * Concurrency safe because we are the last and only thread to alter this
 * sub-buffer. As long as it is not delivered and read, no other thread can
 * alter the offset, alter the reserve_count or call the
 * client_buffer_end_callback on this sub-buffer.
 *
 * The only remaining threads could be the ones with pending commits. They will
 * have to do the deliver themselves.  Not concurrency safe in overwrite mode.
 * We detect corrupted subbuffers with commit and reserve counts. We keep a
 * corrupted sub-buffers count and push the readers across these sub-buffers.
 *
 * Not concurrency safe if a writer is stalled in a subbuffer and another writer
 * switches in, finding out it's corrupted.  The result will be than the old
 * (uncommited) subbuffer will be declared corrupted, and that the new subbuffer
 * will be declared corrupted too because of the commit count adjustment.
 */
static void ltt_reserve_end_switch_current(
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets, u64 *tsc)
{
	long endidx = SUBBUF_INDEX(offsets->end - 1, rchan);
	long commit_count, padding_size;

	padding_size = rchan->subbuf_size
			- (SUBBUF_OFFSET(offsets->end - 1, rchan) + 1);
	ltt_buffer_end(buf, *tsc, offsets->end, endidx);
	/*
	 * Must write slot data before incrementing commit count.
	 * This compiler barrier is upgraded into a smp_wmb() by the IPI
	 * sent by get_subbuf() when it does its smp_rmb().
	 */
	barrier();
	commit_count = local_read(&ltt_buf->commit_count[endidx])
				  + padding_size;
	local_set(&ltt_buf->commit_count[endidx], commit_count);
	ltt_check_deliver(ltt_channel, ltt_buf, rchan, buf,
		offsets->end - 1, commit_count, endidx);
	ltt_write_commit_counter(buf, ltt_buf, endidx,
		offsets->end, commit_count, padding_size);
}

/*
 * Returns :
 * 0 if ok
 * !0 if execution must be aborted.
 */
static int ltt_relay_try_switch_slow(
		enum force_switch_mode mode,
		struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets,
		u64 *tsc)
{
	long subbuf_index;
	long reserve_commit_diff;

	offsets->begin = local_read(&ltt_buf->offset);
	offsets->old = offsets->begin;
	offsets->begin_switch = 0;
	offsets->end_switch_old = 0;

	*tsc = trace_clock_read64();

	if (SUBBUF_OFFSET(offsets->begin, buf->chan) != 0) {
		offsets->begin = SUBBUF_ALIGN(offsets->begin, buf->chan);
		offsets->end_switch_old = 1;
	} else {
		/* we do not have to switch : buffer is empty */
		return -1;
	}
	if (mode == FORCE_ACTIVE)
		offsets->begin += ltt_subbuffer_header_size();
	/*
	 * Always begin_switch in FORCE_ACTIVE mode.
	 * Test new buffer integrity
	 */
	subbuf_index = SUBBUF_INDEX(offsets->begin, buf->chan);
	reserve_commit_diff =
		(BUFFER_TRUNC(offsets->begin, buf->chan)
		 >> ltt_channel->n_subbufs_order)
		- (local_read(&ltt_buf->commit_count[subbuf_index])
			& ltt_channel->commit_count_mask);
	if (reserve_commit_diff == 0) {
		/* Next buffer not corrupted. */
		if (mode == FORCE_ACTIVE
		    && !ltt_channel->overwrite
		    && offsets->begin - atomic_long_read(&ltt_buf->consumed)
		       >= rchan->alloc_size) {
			/*
			 * We do not overwrite non consumed buffers and we are
			 * full : ignore switch while tracing is active.
			 */
			return -1;
		}
	} else {
		/*
		 * Next subbuffer corrupted. Force pushing reader even in normal
		 * mode
		 */
	}
	offsets->end = offsets->begin;
	return 0;
}

/*
 * Force a sub-buffer switch for a per-cpu buffer. This operation is
 * completely reentrant : can be called while tracing is active with
 * absolutely no lock held.
 *
 * Note, however, that as we are disabling interrupts to make local operations
 * atomic, this function must be called from the CPU which owns the buffer for
 * an ACTIVE flush.
 */
void ltt_force_switch_irqoff_slow(struct rchan_buf *buf,
		enum force_switch_mode mode)
{
	struct ltt_channel_struct *ltt_channel =
			(struct ltt_channel_struct *)buf->chan->private_data;
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	struct rchan *rchan = ltt_channel->trans_channel_data;
	struct ltt_reserve_switch_offsets offsets;
	unsigned long flags;
	u64 tsc;

	raw_local_irq_save(flags);

	offsets.size = 0;

	if (ltt_relay_try_switch_slow(mode, ltt_channel, ltt_buf,
			rchan, buf, &offsets, &tsc)) {
		raw_local_irq_restore(flags);
		return;
	}

	local_set(&ltt_buf->offset, offsets.end);

	save_last_tsc(ltt_buf, tsc);

	/*
	 * Push the reader if necessary
	 */
	if (mode == FORCE_ACTIVE) {
		ltt_reserve_push_reader(ltt_channel, ltt_buf, rchan,
					buf, &offsets);
		ltt_clear_noref_flag(rchan, buf, SUBBUF_INDEX(offsets.end - 1,
							      rchan));
	}

	/*
	 * Switch old subbuffer if needed.
	 */
	if (offsets.end_switch_old) {
		ltt_clear_noref_flag(rchan, buf, SUBBUF_INDEX(offsets.old - 1,
							      rchan));
		ltt_reserve_switch_old_subbuf(ltt_channel, ltt_buf, rchan, buf,
			&offsets, &tsc);
	}

	/*
	 * Populate new subbuffer.
	 */
	if (mode == FORCE_ACTIVE)
		ltt_reserve_switch_new_subbuf(ltt_channel,
			ltt_buf, rchan, buf, &offsets, &tsc);

	raw_local_irq_restore(flags);
}
EXPORT_SYMBOL_GPL(ltt_force_switch_irqoff_slow);

/*
 * Returns :
 * 0 if ok
 * !0 if execution must be aborted.
 */
static int ltt_relay_try_reserve_slow(struct ltt_channel_struct *ltt_channel,
		struct ltt_channel_buf_struct *ltt_buf, struct rchan *rchan,
		struct rchan_buf *buf,
		struct ltt_reserve_switch_offsets *offsets, size_t data_size,
		u64 *tsc, unsigned int *rflags, int largest_align)
{
	long reserve_commit_diff;

	offsets->begin = local_read(&ltt_buf->offset);
	offsets->old = offsets->begin;
	offsets->begin_switch = 0;
	offsets->end_switch_current = 0;
	offsets->end_switch_old = 0;

	*tsc = trace_clock_read64();
	if (last_tsc_overflow(ltt_buf, *tsc))
		*rflags = LTT_RFLAG_ID_SIZE_TSC;

	if (unlikely(SUBBUF_OFFSET(offsets->begin, buf->chan) == 0)) {
		offsets->begin_switch = 1;		/* For offsets->begin */
	} else {
		offsets->size = ltt_get_header_size(ltt_channel,
					offsets->begin, data_size,
					&offsets->before_hdr_pad, *rflags);
		offsets->size += ltt_align(offsets->begin + offsets->size,
					   largest_align)
				 + data_size;
		if (unlikely((SUBBUF_OFFSET(offsets->begin, buf->chan) +
			     offsets->size) > buf->chan->subbuf_size)) {
			offsets->end_switch_old = 1;	/* For offsets->old */
			offsets->begin_switch = 1;	/* For offsets->begin */
		}
	}
	if (unlikely(offsets->begin_switch)) {
		long subbuf_index;

		/*
		 * We are typically not filling the previous buffer completely.
		 */
		if (likely(offsets->end_switch_old))
			offsets->begin = SUBBUF_ALIGN(offsets->begin,
						      buf->chan);
		offsets->begin = offsets->begin + ltt_subbuffer_header_size();
		/* Test new buffer integrity */
		subbuf_index = SUBBUF_INDEX(offsets->begin, buf->chan);
		reserve_commit_diff =
			(BUFFER_TRUNC(offsets->begin, buf->chan)
			 >> ltt_channel->n_subbufs_order)
			- (local_read(&ltt_buf->commit_count[subbuf_index])
				& ltt_channel->commit_count_mask);
		if (likely(reserve_commit_diff == 0)) {
			/* Next buffer not corrupted. */
			if (unlikely(!ltt_channel->overwrite &&
				(SUBBUF_TRUNC(offsets->begin, buf->chan)
				 - SUBBUF_TRUNC(atomic_long_read(
							&ltt_buf->consumed),
						buf->chan))
				>= rchan->alloc_size)) {
				/*
				 * We do not overwrite non consumed buffers
				 * and we are full : event is lost.
				 */
				local_inc(&ltt_buf->events_lost);
				return -1;
			} else {
				/*
				 * next buffer not corrupted, we are either in
				 * overwrite mode or the buffer is not full.
				 * It's safe to write in this new subbuffer.
				 */
			}
		} else {
			/*
			 * Next subbuffer corrupted. Drop event in normal and
			 * overwrite mode. Caused by either a writer OOPS or
			 * too many nested writes over a reserve/commit pair.
			 */
			local_inc(&ltt_buf->events_lost);
			return -1;
		}
		offsets->size = ltt_get_header_size(ltt_channel,
					offsets->begin, data_size,
					&offsets->before_hdr_pad, *rflags);
		offsets->size += ltt_align(offsets->begin + offsets->size,
					   largest_align)
				 + data_size;
		if (unlikely((SUBBUF_OFFSET(offsets->begin, buf->chan)
			     + offsets->size) > buf->chan->subbuf_size)) {
			/*
			 * Event too big for subbuffers, report error, don't
			 * complete the sub-buffer switch.
			 */
			local_inc(&ltt_buf->events_lost);
			return -1;
		} else {
			/*
			 * We just made a successful buffer switch and the event
			 * fits in the new subbuffer. Let's write.
			 */
		}
	} else {
		/*
		 * Event fits in the current buffer and we are not on a switch
		 * boundary. It's safe to write.
		 */
	}
	offsets->end = offsets->begin + offsets->size;

	if (unlikely((SUBBUF_OFFSET(offsets->end, buf->chan)) == 0)) {
		/*
		 * The offset_end will fall at the very beginning of the next
		 * subbuffer.
		 */
		offsets->end_switch_current = 1;	/* For offsets->begin */
	}
	return 0;
}

/**
 * ltt_relay_reserve_slot_irqoff_slow - Atomic slot reservation in a buffer.
 * @trace: the trace structure to log to.
 * @ltt_channel: channel structure
 * @transport_data: data structure specific to ltt relay
 * @data_size: size of the variable length data to log.
 * @slot_size: pointer to total size of the slot (out)
 * @buf_offset : pointer to reserved buffer offset (out)
 * @tsc: pointer to the tsc at the slot reservation (out)
 * @cpu: cpuid
 *
 * Return : -ENOSPC if not enough space, else returns 0.
 * It will take care of sub-buffer switching.
 */
int ltt_reserve_slot_irqoff_slow(struct ltt_trace_struct *trace,
		struct ltt_channel_struct *ltt_channel, void **transport_data,
		size_t data_size, size_t *slot_size, long *buf_offset, u64 *tsc,
		unsigned int *rflags, int largest_align, int cpu)
{
	struct rchan *rchan = ltt_channel->trans_channel_data;
	struct rchan_buf *buf = *transport_data = rchan->buf[cpu];
	struct ltt_channel_buf_struct *ltt_buf = buf->chan_private;
	struct ltt_reserve_switch_offsets offsets;

	offsets.size = 0;

	if (unlikely(ltt_relay_try_reserve_slow(ltt_channel, ltt_buf,
			rchan, buf, &offsets, data_size, tsc, rflags,
			largest_align))) {
		raw_local_irq_restore(ltt_buf->irqflags);
		return -ENOSPC;
	}

	local_set(&ltt_buf->offset, offsets.end);

	save_last_tsc(ltt_buf, *tsc);

	/*
	 * Push the reader if necessary
	 */
	ltt_reserve_push_reader(ltt_channel, ltt_buf, rchan, buf, &offsets);

	/*
	 * Clear noref flag for this subbuffer.
	 */
	ltt_clear_noref_flag(rchan, buf, SUBBUF_INDEX(offsets.end - 1, rchan));

	/*
	 * Switch old subbuffer if needed.
	 */
	if (unlikely(offsets.end_switch_old)) {
		ltt_clear_noref_flag(rchan, buf, SUBBUF_INDEX(offsets.old - 1,
							      rchan));
		ltt_reserve_switch_old_subbuf(ltt_channel, ltt_buf, rchan, buf,
			&offsets, tsc);
	}

	/*
	 * Populate new subbuffer.
	 */
	if (unlikely(offsets.begin_switch))
		ltt_reserve_switch_new_subbuf(ltt_channel, ltt_buf, rchan,
			buf, &offsets, tsc);

	if (unlikely(offsets.end_switch_current))
		ltt_reserve_end_switch_current(ltt_channel, ltt_buf, rchan,
			buf, &offsets, tsc);

	*slot_size = offsets.size;
	*buf_offset = offsets.begin + offsets.before_hdr_pad;
	return 0;
}
EXPORT_SYMBOL_GPL(ltt_reserve_slot_irqoff_slow);

static struct ltt_transport ltt_relay_transport = {
	.name = "relay",
	.owner = THIS_MODULE,
	.ops = {
		.create_dirs = ltt_relay_create_dirs,
		.remove_dirs = ltt_relay_remove_dirs,
		.create_channel = ltt_relay_create_channel,
		.finish_channel = ltt_relay_finish_channel,
		.remove_channel = ltt_relay_remove_channel,
		.wakeup_channel = ltt_relay_async_wakeup_chan,
		.user_blocking = ltt_relay_user_blocking,
		.user_errors = ltt_relay_print_user_errors,
	},
};

static const struct file_operations ltt_file_operations = {
	.open = ltt_open,
	.release = ltt_release,
	.poll = ltt_poll,
	.splice_read = ltt_relay_file_splice_read,
	.ioctl = ltt_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ltt_compat_ioctl,
#endif
};

static int __init ltt_relay_init(void)
{
	printk(KERN_INFO "LTT : ltt-relay init\n");

	ltt_transport_register(&ltt_relay_transport);

	return 0;
}

static void __exit ltt_relay_exit(void)
{
	printk(KERN_INFO "LTT : ltt-relay exit\n");

	ltt_transport_unregister(&ltt_relay_transport);
}

module_init(ltt_relay_init);
module_exit(ltt_relay_exit);

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("Linux Trace Toolkit Next Generation Irqoff Relay");
