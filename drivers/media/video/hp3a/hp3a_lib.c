/*
 * drivers/media/video/hp3a/hp3a_task.c
 *
 * HP Imaging/3A Driver : library function implementations.
 *
 * Copyright (C) 2008-2009 Hewlett-Packard Co.
 *
 * Contributors:
 *		Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 */

#include <linux/sched.h>
#include "hp3a_common.h"
#include "hp3a_user.h"
#include "ispmmu.h"

/**
 * map_user_memory - Maps user space memory into kernel.
 * @addr: Starting address of the memory to be mapped.
 * @size: Size of the memory to be mapped.
 *
 * Returns the address of the array of struct page containing memory
 * mapped into kernel space.
 **/
struct page **map_user_memory(unsigned long addr, u32 size)
{
	struct page **ppages = NULL;
	int nr_pages;
	int ret;

	if (addr && size) {
		nr_pages =  NR_PAGES(addr, (unsigned long)size);
		ppages = kzalloc(sizeof(struct page *)*nr_pages, GFP_KERNEL);

		if (likely(ppages != NULL)) {
			/* initialize page array. */
			down_read(&current->mm->mmap_sem);
			ret = get_user_pages(current,
				current->mm,
				addr & PAGE_MASK,
				nr_pages,
				1, /* 1 = from device, 0 = to device. */
				1, /* force */
				ppages,
				NULL);
			up_read(&current->mm->mmap_sem);

			if (unlikely(ret != nr_pages)) {
				kfree(ppages);
				ppages = NULL;
				printk(KERN_ERR "hp3a: Mapping user pages"
						" to kernel failed!\n");
			}
		} else {
			printk(KERN_ERR "hp3a: Error allocating kernel memory!\n");
		}
	}

	return ppages;
}

/**
 * unmap_user_memory - Releases mapped user space memory from kernel space.
 * @pages: Pointer to an array of page structure containing mapped
 *		pages into kernel space
 * @nr_pages: Number of pages to release
 *
 * Return 0 on success, -1 otherwise.
 **/
void unmap_user_memory(struct page **pages, int nr_pages)
{
	u32 i;

	if (likely(pages != NULL)) {
		for (i = 0; i < nr_pages; ++i) {
			page_cache_release(pages[i]);
		}
	}
}

/**
 * mmap_user_to_isp - Maps user memory in to kernel space and then maps
 *			kernel memory to isp address space.
 * @src: Pointer to user buffer info.
 * @dest: Pointer to internal buffer to save map info.
 *
 * Return 0 on success, -1 otherwise.
 **/
int map_user_to_kernel(struct hp3a_buffer *src, struct hp3a_internal_buffer *dest)
{
	dest->pages =  map_user_memory((unsigned long)src->addr,
					src->buffer_size);
	if (likely(dest->pages != NULL)) {
		dest->buffer_size = src->buffer_size;
		dest->user_addr = src->addr;
		dest->isp_addr = 0;

		return 0;
	}

	return -1;
}

/**
 * unmap_buffers_from_kernel - Unmaps memory from isp and kernel address space.
 * @ibuffer: Pointer to internal buffer to be unmapped.
 *
 * No return value.
 **/
void unmap_buffer_from_kernel(struct hp3a_internal_buffer *ibuffer)
{
	if (ibuffer->isp_addr) {
#if defined(CONFIG_VIDEO_OLDOMAP3)
		if (ispmmu_unmap(ibuffer->isp_addr) != 0) {
			printk(KERN_ERR "Error unmapping from ispmmu (0x%x)!",
				(unsigned int)ibuffer->isp_addr);
		}
#else
		ispmmu_vunmap(ibuffer->isp_addr);
#endif
		ibuffer->isp_addr = 0;
	}

	if (ibuffer->pages != NULL && ibuffer->buffer_size > 0) {
		unmap_user_memory(ibuffer->pages, NR_PAGES(ibuffer->user_addr,
				ibuffer->buffer_size));
		kfree(ibuffer->pages);
		ibuffer->pages = NULL;
	}
}

/**
 * flush_dcache_ibuffer - Flushes dcache corresponding to a internal buffer.
 * @ibuffer: Pointer to internal buffer to be flushed from cache.
 *
 * No return value.
 **/
void flush_dcache_ibuffer(struct hp3a_internal_buffer  *ibuffer)
{
	int i;
	int nr_pages = NR_PAGES(ibuffer->user_addr, (unsigned long)ibuffer->buffer_size);

	for (i = 0; i < nr_pages; ++i) {
		flush_dcache_page(ibuffer->pages[i]);
	}
}

/**
 * hp3a_clear_regs - Clear an array of registers.
 * @regs: Pointer to a array to be cleared.
 *
 * No return value.
 **/
void hp3a_clear_regs(struct hp3a_reg *regs)
{
	struct hp3a_reg *iter = regs;

	for (; iter->len != HP3A_REG_TOK_TERM; ++iter) {
		iter->val = 0;
	}
}

/**
 * hp3a_read_ispregs - Read an array of ISP registers.
 * @regs: Pointer to a array of registers to read.
 *
 * No return value.
 **/
void hp3a_read_ispregs(struct hp3a_reg *regs)
{
	struct hp3a_reg *iter = regs;

	for (; iter->len != HP3A_REG_TOK_TERM; ++iter) {
		iter->val = omap_readl(iter->reg);
	}
}

/**
 * hp3a_write_ispregs - Write an array of ISP registers.
 * @regs: Pointer to a array of registers to writes.
 *
 * No return value.
 **/
void hp3a_write_ispregs(struct hp3a_reg *regs)
{
	struct hp3a_reg *iter = regs;

	for (; iter->len != HP3A_REG_TOK_TERM; ++iter) {
		if (iter->len == HP3A_REG_TOK_DELAY) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule_timeout(msecs_to_jiffies(iter->val));
			continue;
		}
		omap_writel(iter->val , iter->reg);
	}
}

/**
 * hp3a_read_ispregs_to_user - Read an array of ISP registers into user buffer.
 * @user_page: Pointer to structure containing register page to read.
 *
 * Return 0 on success, -1 otherwise.
 **/
int hp3a_read_ispregs_to_user(struct hp3a_reg_page *user_page)
{
	int ret = -1;
	struct hp3a_reg_page page;
	struct hp3a_reg *regs;

	if (copy_from_user(&page,
			(struct hp3a_reg_page *)user_page,
			sizeof(struct hp3a_reg_page) == 0)) {
		regs = kmalloc(sizeof(struct hp3a_reg) * page.len, GFP_KERNEL);
		if (page.regs != 0) {
			if (copy_from_user(regs, page.regs,
				sizeof(struct hp3a_reg) * page.len) == 0) {
				hp3a_read_ispregs(regs);

				if (copy_to_user(page.regs, regs,
					sizeof(struct hp3a_reg) * page.len) == 0) {
					ret = 0;
				}
			}
			kfree(regs);
		} else {
			printk(KERN_ERR "hp3a: Error allocating memory for register read!\n");
		}
	}

	return ret;
}

/**
 * hp3a_read_ispreg_to_user - Read an array of ISP registers into user buffer.
 * @user_page: Pointer to structure containing register page to read.
 *
 * Return 0 on success, -1 otherwise.
 **/
int hp3a_read_ispreg_to_user(struct hp3a_reg *user_reg)
{
	int ret = -1;
	struct hp3a_reg omapreg;

	if (copy_from_user(&omapreg,
			(struct hp3a_reg *)user_reg,
			sizeof(struct hp3a_reg)) == 0) {

		omapreg.val = omap_readl(omapreg.reg);

		if (copy_to_user(user_reg, &omapreg,
			sizeof(struct hp3a_reg)) == 0) {
			ret = 0;
		}
	}

	return ret;
}
