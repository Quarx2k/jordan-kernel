/*
 * ltt/probes/fs-trace.c
 *
 * FS tracepoint probes.
 *
 * (C) Copyright 2009 - Mathieu Desnoyers <mathieu.desnoyers@polymtl.ca>
 * Dual LGPL v2.1/GPL v2 license.
 */

#include <linux/module.h>
#include <linux/buffer_head.h>
#include <linux/ltt-type-serializer.h>
#include <trace/fs.h>

void probe_fs_buffer_wait_start(struct buffer_head *bh)
{
	trace_mark_tp(fs, buffer_wait_start, fs_buffer_wait_start,
		probe_fs_buffer_wait_start, "bh %p", bh);
}

void probe_fs_buffer_wait_end(struct buffer_head *bh)
{
	trace_mark_tp(fs, buffer_wait_end, fs_buffer_wait_end,
		probe_fs_buffer_wait_end, "bh %p", bh);
}

void probe_fs_exec(char *filename)
{
	trace_mark_tp(fs, exec, fs_exec, probe_fs_exec, "filename %s",
		filename);
}

void probe_fs_ioctl(unsigned int fd, unsigned int cmd, unsigned long arg)
{
	trace_mark_tp(fs, ioctl, fs_ioctl, probe_fs_ioctl,
		"fd %u cmd %u arg %lu", fd, cmd, arg);
}

void probe_fs_open(int fd, char *filename)
{
	trace_mark_tp(fs, open, fs_open, probe_fs_open,
		"fd %d filename %s", fd, filename);
}

void probe_fs_close(unsigned int fd)
{
	trace_mark_tp(fs, close, fs_close, probe_fs_close, "fd %u", fd);
}

void probe_fs_lseek(unsigned int fd, long offset, unsigned int origin)
{
	trace_mark_tp(fs, lseek, fs_lseek, probe_fs_lseek,
		"fd %u offset %ld origin %u", fd, offset, origin);
}

void probe_fs_llseek(unsigned int fd, loff_t offset, unsigned int origin)
{
	trace_mark_tp(fs, llseek, fs_llseek, probe_fs_llseek,
		"fd %u offset %lld origin %u", fd,
		(long long)offset, origin);
}

void probe_fs_read(unsigned int fd, char __user *buf, size_t count,
		ssize_t ret);

DEFINE_MARKER_TP(fs, read, fs_read, probe_fs_read,
	"count %zu fd %u");

notrace void probe_fs_read(unsigned int fd, char __user *buf, size_t count,
		ssize_t ret)
{
	struct marker *marker;
	struct serialize_sizet_int data;

	data.f1 = count;
	data.f2 = fd;

	marker = &GET_MARKER(fs, read);
	ltt_specialized_trace(marker, marker->single.probe_private,
		&data, serialize_sizeof(data), sizeof(size_t));
}

void probe_fs_write(unsigned int fd, char __user *buf, size_t count,
		ssize_t ret);

DEFINE_MARKER_TP(fs, write, fs_write, probe_fs_write,
	"count %zu fd %u");

notrace void probe_fs_write(unsigned int fd, char __user *buf, size_t count,
		ssize_t ret)
{
	struct marker *marker;
	struct serialize_sizet_int data;

	data.f1 = count;
	data.f2 = fd;

	marker = &GET_MARKER(fs, write);
	ltt_specialized_trace(marker, marker->single.probe_private,
		&data, serialize_sizeof(data), sizeof(size_t));
}

void probe_fs_pread64(unsigned int fd, char __user *buf, size_t count,
		loff_t pos, ssize_t ret)
{
	trace_mark_tp(fs, pread64, fs_pread64, probe_fs_pread64,
		"fd %u count %zu pos %llu",
		fd, count, (unsigned long long)pos);
}

void probe_fs_pwrite64(unsigned int fd, const char __user *buf,
		size_t count, loff_t pos, ssize_t ret)
{
	trace_mark_tp(fs, pwrite64, fs_pwrite64, probe_fs_pwrite64,
		"fd %u count %zu pos %llu",
		fd, count, (unsigned long long)pos);
}

void probe_fs_readv(unsigned long fd, const struct iovec __user *vec,
		unsigned long vlen, ssize_t ret)
{
	trace_mark_tp(fs, readv, fs_readv, probe_fs_readv,
		"fd %lu vlen %lu", fd, vlen);
}

void probe_fs_writev(unsigned long fd, const struct iovec __user *vec,
		unsigned long vlen, ssize_t ret)
{
	trace_mark_tp(fs, writev, fs_writev, probe_fs_writev,
		"fd %lu vlen %lu", fd, vlen);
}

void probe_fs_select(int fd, struct timespec *end_time)
{
	struct timespec tmptime;

	if (end_time) {
		tmptime = *end_time;
	} else {
		tmptime.tv_sec = -1L;
		tmptime.tv_nsec = -1L;
	}

	trace_mark_tp(fs, select, fs_select, probe_fs_select,
		"fd %d end_time_sec %ld end_time_nsec %ld", fd,
			tmptime.tv_sec, tmptime.tv_nsec);
}

void probe_fs_poll(int fd)
{
	trace_mark_tp(fs, pollfd, fs_poll, probe_fs_poll,
		"fd %d", fd);
}

MODULE_LICENSE("GPL and additional rights");
MODULE_AUTHOR("Mathieu Desnoyers");
MODULE_DESCRIPTION("FS Tracepoint Probes");
