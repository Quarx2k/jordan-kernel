/*
 * isp_mem_process.c
 *
 * Driver Library for ISP Control module in TI's OMAP3 Camera ISP
 * RAW bayer processing from memory is defined here.
 *
 * Copyright (C) 2010 Hewlett-Packard Co.
 *
 * Contributors:
 * 	Tanvir Islam <tanvir.islam@hp.com>
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */
#include <linux/mutex.h>
#include <linux/module.h>
#include <asm/page.h>
#include <linux/pagemap.h>
#include <linux/delay.h>

#include "isp.h"
#include "ispreg.h"
#include "ispccdc.h"
#include "isppreview.h"
#include "ispresizer.h"
#include "ispmmu.h"
#include "isp_mem_process.h"

#if 0
	#define DPRINTK_ISPPROC(format, ...)		\
		printk(KERN_INFO "ISPPROC: " format, ## __VA_ARGS__)
#else
	#define DPRINTK_ISPPROC(format, ...)
#endif
struct completion isp_wfc;

/**
 * map_user_memory_to_kernel - Map user memory to kernel
 * @addr: address to be mapped
 * @size: Size of memory
 * @nr_pages_mapped: Variable to store number of pages mapped.
 *
 * Return a pointer to a page array.
 **/
struct page **map_user_memory_to_kernel(
			unsigned long addr,
			u32 size,
			u32 *nr_pages_mapped)
{
	struct page **ppages = NULL;
	int nr_pages;
	int ret;

	if (addr && size) {
		nr_pages =  NR_PAGES(addr, (unsigned long)size);

		ppages = kzalloc(sizeof(struct page *)*nr_pages, GFP_KERNEL);
		if (likely(ppages != NULL)) {
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
				printk(KERN_ERR "isp : Mapping user pages"
						" to kernel failed\n");
			}
			if (nr_pages_mapped)
				*nr_pages_mapped = nr_pages;
		} else
			printk(KERN_ERR "isp : Error allocating kernel memory\n");
	}

	return ppages;
}

/**
 * unmap_user_memory_from_kernel - Unmaps user memory from kernel
 * @pages: Array of pages to be unmapped.
 * @nr_pages: Number of pages in the page array.
 **/
void unmap_user_memory_from_kernel(struct page **pages, int nr_pages)
{
	u32 i;

	if (pages != NULL) {
		for (i = 0; i < nr_pages; ++i)
			page_cache_release(pages[i]);
	}
}

/**
 * rsz_isr - Interrupt Service Routine for Resizer wrapper
 * @status: ISP IRQ0STATUS register value
 * @arg1: Currently not used
 * @arg2: Currently not used
 *
 * Interrupt Service Routine for Resizer wrapper
 **/
static void rsz_isr(unsigned long status, isp_vbq_callback_ptr arg1, void *arg2)
{
	ispresizer_enable(0);
	complete(&isp_wfc);

	DPRINTK_ISPPROC("rsz_isr\n");
}

/**
 * preview_isr - Interrupt Service Routine for Preview wrapper
 * @status: ISP IRQ0STATUS register value
 * @arg1: Currently not used
 * @arg2: Currently not used
 *
 * Interrupt Service Routine for Preview wrapper
 **/
static void prv_isr(unsigned long status, isp_vbq_callback_ptr arg1, void *arg2)
{
	isppreview_enable(0);
	ispresizer_enable(1);

	DPRINTK_ISPPROC("prv_isr\n");
}

int isp_process_mem_data(struct isp_mem_data *data)
{
	int i;
	int ret = -1;
	struct isp_mem_data *ppreview_user = \
		(struct isp_mem_data *)data;
	struct isp_mem_data preview_param;
	u32 input_buffer_size, output_buffer_size;
	u32 input_nr_pages, output_nr_pages;
	struct page **input_pages = NULL;
	struct page **output_pages = NULL;
	unsigned long isp_addr_in = 0;
	unsigned long  isp_addr_out = 0;
	unsigned long  isp_addr_tmp = 0;
	unsigned long timeout;
	struct isp_mem_resize_data resizer_param;
	u16 cropadjust = 0;

	if (ppreview_user == NULL) {
		printk(KERN_ERR "ISP_PROC_ERR: Invalid user data!\n");
		return -EINVAL;
	}

	memcpy(&preview_param, ppreview_user, \
		sizeof(struct isp_mem_data));

	DPRINTK_ISPPROC("input(%d-%d) - output(%d-%d)\n",
		preview_param.input_width,
		preview_param.input_height,
		preview_param.output_width,
		preview_param.output_height);

	DPRINTK_ISPPROC("start(%d-%d) - end(%d-%d)\n",
		preview_param.left,
		preview_param.top,
		preview_param.crop_width,
		preview_param.crop_height);

	if (ppreview_user->datain == 0 || ppreview_user->dataout == 0)
		return -EINVAL;

	isppreview_enable(0);
	ispresizer_enable(0);
	timeout = jiffies + msecs_to_jiffies(200);
	while (isppreview_busy() ||
			ispresizer_busy()) {
		if (time_after(jiffies, timeout))
			return -EINVAL;
		msleep(1);
	}

	isppreview_save_context();
	ispresizer_save_context();
	isppreview_free();
	ispresizer_free();
	isppreview_request();
	ispresizer_request();

	/* set data path before configuring modules. */
	isppreview_update_datapath(PRV_RAW_MEM, PREVIEW_MEM);
	ispresizer_config_datapath(RSZ_MEM_YUV, 0);

	ret = isppreview_try_size(preview_param.input_width,
		preview_param.input_height,
		&preview_param.output_width,
		&preview_param.output_height);
	if (ret < 0)
		goto exit_cleanup;
	ret = isppreview_config_size(preview_param.input_width,
		preview_param.input_height,
		preview_param.output_width,
		preview_param.output_height);
	if (ret < 0)
		goto exit_cleanup;

	input_buffer_size = ALIGN_TO(ppreview_user->input_width* \
		ppreview_user->input_height*2 , 0x100);
	input_pages = map_user_memory_to_kernel(preview_param.datain,
		input_buffer_size, &input_nr_pages);
	if (input_pages == NULL) {
		ret = -EINVAL;
		printk(KERN_ERR "ISP_PROC_ERR: memory allocation failed\n");
		goto exit_cleanup;
	}

	output_buffer_size = ALIGN_TO(ppreview_user->output_width* \
		ppreview_user->output_height*2, 0x1000);
	output_pages = map_user_memory_to_kernel(preview_param.dataout,
		output_buffer_size, &output_nr_pages);
	if (output_pages == NULL) {
		ret = -EINVAL;
		printk(KERN_ERR "ISP_PROC_ERR: memory allocation failed\n");
		goto exit_cleanup;
	}
	for (i = 0; i < output_nr_pages; ++i)
		flush_dcache_page(output_pages[i]);

	isp_addr_in = ispmmu_vmap_pages(input_pages, input_nr_pages);
	if (IS_ERR((void *)isp_addr_in)) {
		isp_addr_in = 0;
		ret = -EINVAL;
		printk(KERN_ERR "ISP_PROC_ERR: isp mmu map failed\n");
		goto exit_cleanup;
	}
	isp_addr_out = ispmmu_vmap_pages(output_pages, output_nr_pages);
	if (IS_ERR((void *)isp_addr_out)) {
		isp_addr_out = 0;
		ret = -EINVAL;
		printk(KERN_ERR "ISP_PROC_ERR: isp mmu map failed\n");
		goto exit_cleanup;
	}

	/* This buffer must be allocated and mapped to
		the ISP MMU previously. */
	isp_addr_tmp = isp_tmp_buf_addr();
	if (isp_addr_tmp == 0) {
		printk(KERN_ERR "ISP_PROC_ERR: Invalid isp tmp buffer address!\n");
		goto exit_cleanup;
	}

	isppreview_config_inlineoffset(ppreview_user->input_width * 2);
	isppreview_set_inaddr(isp_addr_in);
	isppreview_set_outaddr(isp_addr_tmp);

	resizer_param.input_width = preview_param.output_width;
	resizer_param.input_height = preview_param.output_height;
	resizer_param.output_width = ppreview_user->output_width;
	resizer_param.output_height = ppreview_user->output_height;

	if ((preview_param.left == 0) && (preview_param.top == 0)) {
		ret = ispresizer_try_size(&resizer_param.input_width,
				&resizer_param.input_height,
				&resizer_param.output_width,
				&resizer_param.output_height);
		if (ret < 0)
			goto exit_cleanup;
		ret = ispresizer_config_size(resizer_param.input_width,
				resizer_param.input_height,
				resizer_param.output_width,
				resizer_param.output_height);
		if (ret < 0)
			goto exit_cleanup;
		ispresizer_set_inaddr(isp_addr_tmp);
	} else {
		ispresizer_trycrop(preview_param.left,
				preview_param.top,
				preview_param.crop_width,
				preview_param.crop_height,
				resizer_param.output_width,
				resizer_param.output_height);

		ispresizer_applycrop();

		/* account for pixel loss when using crop*/
		if ((preview_param.input_height > preview_param.output_height)
				&& (preview_param.top > 16))
			cropadjust = 8;
		else
			cropadjust = 0;

		/* pixel alignment in 32bit space, vertical must be 0 per TRM */
		isp_reg_writel(((preview_param.left%16) <<
					ISPRSZ_IN_START_HORZ_ST_SHIFT) |
					(0 <<
					ISPRSZ_IN_START_VERT_ST_SHIFT),
					OMAP3_ISP_IOMEM_RESZ,
					ISPRSZ_IN_START);

		/* Align input address for cropping, per TRM  */
		ispresizer_set_inaddr(isp_addr_tmp -
				(resizer_param.input_width*2*cropadjust) +
				(preview_param.top*resizer_param.input_width*2)
				+ ((preview_param.left/16)*32));
	}

	ispresizer_set_outaddr(isp_addr_out);
	ispresizer_config_inlineoffset(
		ALIGN_TO(resizer_param.input_width*2, 32));

	if (isp_set_callback(CBK_PREV_DONE, prv_isr,
			(void *) NULL, (void *)NULL) != 0) {
		printk(KERN_ERR "ISP_PROC_ERR: Error setting PRV callback.\n");
		goto exit_cleanup;
	}

	if (isp_set_callback(CBK_RESZ_DONE, rsz_isr,
			(void *) NULL, (void *)NULL) != 0) {
		printk(KERN_ERR "ISP_PROC_ERR: Error setting RSZ callback.\n");
		goto exit_cleanup;
	}

	isp_reg_writel(0xFFFFFFFF, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_wfc.done = 0;

	/* start preview engine. */
	isppreview_enable(1);

	ret = wait_for_completion_timeout(&isp_wfc, msecs_to_jiffies(1000));
	if (!ret) {
		isppreview_enable(0);
		ispresizer_enable(0);
	}

	timeout = jiffies + msecs_to_jiffies(50);
	while (ispresizer_busy()) {
		msleep(5);
		if (time_after(jiffies, timeout)) {
			printk(KERN_ERR "ISP_RESZ_ERR: Resizer still busy");
			break;
		}
	}

	isp_reg_writel(0xFFFFFFFF, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_unset_callback(CBK_PREV_DONE);
	isp_unset_callback(CBK_RESZ_DONE);

exit_cleanup:
	isppreview_restore_context();
	ispresizer_restore_context();

	if (isp_addr_in != 0)
		ispmmu_vunmap(isp_addr_in);
	if (isp_addr_out != 0)
		ispmmu_vunmap(isp_addr_out);
	if (input_pages != NULL) {
		unmap_user_memory_from_kernel(input_pages, input_nr_pages);
		kfree(input_pages);
	}
	if (output_pages != NULL) {
		unmap_user_memory_from_kernel(output_pages, output_nr_pages);
		kfree(output_pages);
	}

	DPRINTK_ISPPROC("exit.\n");

	return ret;
}

int isp_resize_mem_data(struct isp_mem_resize_data *data)
{
	int i;
	int ret = -1;
	struct isp_mem_resize_data *presizer_user = \
		(struct isp_mem_resize_data *)data;
	u32 input_buffer_size, output_buffer_size;
	u32 input_nr_pages, output_nr_pages;
	struct page **input_pages = NULL;
	struct page **output_pages = NULL;
	unsigned long isp_addr_in = 0;
	unsigned long  isp_addr_out = 0;
	struct isp_mem_resize_data resizer_param;
	unsigned long timeout;

	if (presizer_user == NULL) {
		printk(KERN_ERR "ISP_RESZ_ERR : Invalid user data\n");
		return -EINVAL;
	}

	memcpy(&resizer_param, presizer_user, \
		sizeof(struct isp_mem_resize_data));

	DPRINTK_ISPPROC("\nRSZ input(%d-%d) - output(%d-%d)\n",
		resizer_param.input_width,
		resizer_param.input_height,
		resizer_param.output_width,
		resizer_param.output_height);

	DPRINTK_ISPPROC("RSZ start(%d-%d) - end(%d-%d)\n",
		resizer_param.left,
		resizer_param.top,
		resizer_param.crop_width,
		resizer_param.crop_height);

	if (presizer_user->datain == 0 || presizer_user->dataout == 0)
		return -EINVAL;

	ispresizer_enable(0);
	timeout = jiffies + msecs_to_jiffies(200);
	while (ispresizer_busy()) {
		if (time_after(jiffies, timeout))
			return -EINVAL;
		msleep(1);
	}

	ispresizer_save_context();
	ispresizer_free();
	ispresizer_request();

	/* set data path before configuring modules. */
	ispresizer_config_datapath(RSZ_MEM_YUV, 0);

	input_buffer_size = ALIGN_TO(presizer_user->input_width* \
		presizer_user->input_height*2 , 0x100);
	input_pages = map_user_memory_to_kernel(presizer_user->datain,
		input_buffer_size, &input_nr_pages);
	if (input_pages == NULL) {
		ret = -EINVAL;
		printk(KERN_ERR "ISP_RESZ_ERR: memory allocation failed\n");
		goto exit_cleanup;
	}

	output_buffer_size = ALIGN_TO(presizer_user->output_width* \
		presizer_user->output_height*2, 0x1000);
	output_pages = map_user_memory_to_kernel(presizer_user->dataout,
		output_buffer_size, &output_nr_pages);
	if (output_pages == NULL) {
		ret = -EINVAL;
		printk(KERN_ERR "ISP_RESZ_ERR: memory allocation failed\n");
		goto exit_cleanup;
	}
	for (i = 0; i < output_nr_pages; ++i)
		flush_dcache_page(output_pages[i]);

	isp_addr_in = ispmmu_vmap_pages(input_pages, input_nr_pages);
	if (IS_ERR((void *)isp_addr_in)) {
		isp_addr_in = 0;
		ret = -EINVAL;
		printk(KERN_ERR "ISP_RESZ_ERR: isp mmu map failed\n");
		goto exit_cleanup;
	}
	isp_addr_out = ispmmu_vmap_pages(output_pages, output_nr_pages);
	if (IS_ERR((void *)isp_addr_out)) {
		isp_addr_out = 0;
		ret = -EINVAL;
		printk(KERN_ERR "ISP_RESZ_ERR:  isp mmu map failed\n");
		goto exit_cleanup;
	}

	if ((resizer_param.left == 0) && (resizer_param.top == 0)) {
		ret = ispresizer_try_size(&resizer_param.input_width,
					&resizer_param.input_height,
					&resizer_param.output_width,
					&resizer_param.output_height);

		ret = ispresizer_config_size(resizer_param.input_width,
					resizer_param.input_height,
					resizer_param.output_width,
					resizer_param.output_height);

		ispresizer_set_inaddr(isp_addr_in);
	} else {
		ispresizer_trycrop(resizer_param.left,
					resizer_param.top,
					resizer_param.crop_width,
					resizer_param.crop_height,
					resizer_param.output_width,
					resizer_param.output_height);

		ispresizer_applycrop();

		/*pixel alignment in 32bit space, vertical must be 0 per TRM */
		isp_reg_writel(((resizer_param.left%16) <<
				ISPRSZ_IN_START_HORZ_ST_SHIFT) |
				(0 <<
				ISPRSZ_IN_START_VERT_ST_SHIFT),
				OMAP3_ISP_IOMEM_RESZ,
				ISPRSZ_IN_START);

		/* Align input address for cropping, per TRM  */
		ispresizer_set_inaddr(isp_addr_in +
			(resizer_param.top*resizer_param.input_width*2)
			+ ((resizer_param.left/16)*32));
	}

	ispresizer_set_inaddr(isp_addr_in);
	ispresizer_set_outaddr(isp_addr_out);
	ispresizer_config_ycpos(0);
	ispresizer_config_inlineoffset(
		ALIGN_TO(presizer_user->input_width*2, 32));

	isp_set_callback(CBK_RESZ_DONE, rsz_isr, (void *) NULL, (void *)NULL);
	isp_reg_writel(0xFFFFFFFF, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_wfc.done = 0;
	/* start resizer engine. */
	ispresizer_enable(1);

	ret = wait_for_completion_timeout(&isp_wfc, msecs_to_jiffies(1000));
	if (!ret)
		ispresizer_enable(0);

	timeout = jiffies + msecs_to_jiffies(50);
	while (ispresizer_busy()) {
		msleep(5);
		if (time_after(jiffies, timeout)) {
			printk(KERN_ERR "ISP_RESZ_ERR: Resizer still busy");
			break;
		}
	}

	isp_reg_writel(0xFFFFFFFF, OMAP3_ISP_IOMEM_MAIN, ISP_IRQ0STATUS);
	isp_unset_callback(CBK_RESZ_DONE);
	ret = 0;

exit_cleanup:
	ispresizer_restore_context();

	if (isp_addr_in != 0)
		ispmmu_vunmap(isp_addr_in);
	if (isp_addr_out != 0)
		ispmmu_vunmap(isp_addr_out);
	if (input_pages != NULL) {
		unmap_user_memory_from_kernel(input_pages, input_nr_pages);
		kfree(input_pages);
	}
	if (output_pages != NULL) {
		unmap_user_memory_from_kernel(output_pages, output_nr_pages);
		kfree(output_pages);
	}

	DPRINTK_ISPPROC("resizer exit.\n");

	return ret;
}

void isp_mem_process_init(void)
{
	init_completion(&isp_wfc);
	isp_wfc.done = 0;
}
