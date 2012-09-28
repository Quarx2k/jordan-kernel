/*
 * drivers/media/video/isp/isp_af.c
 *
 * AF module for TI's OMAP3430 Camera ISP
 *
 * Copyright (C) 2008 Texas Instruments.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/* Linux specific include files */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/cacheflush.h>
#include <linux/uaccess.h>
#include <linux/io.h>

#include <linux/mman.h>
#include <linux/syscalls.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/dma-mapping.h>

#include <media/v4l2-int-device.h>
#include "isp.h"
#include "ispreg.h"
#include "isph3a.h"
#include "isp_af.h"
#include "ispmmu.h"

static DECLARE_MUTEX(ispaf_mutex);

/**
 * struct isp_af_buffer - AF frame stats buffer.
 * @virt_addr: Virtual address to mmap the buffer.
 * @phy_addr: Physical address of the buffer.
 * @addr_align: Virtual Address 32 bytes aligned.
 * @ispmmu_addr: Address of the buffer mapped by the ISPMMU.
 * @mmap_addr: Mapped memory area of buffer. For userspace access.
 * @locked: 1 - Buffer locked from write. 0 - Buffer can be overwritten.
 * @frame_num: Frame number from which the statistics are taken.
 * @lens_position: Lens position currently set in the DW9710 Coil motor driver.
 * @next: Pointer to link next buffer.
 */
struct isp_af_buffer {
	unsigned long virt_addr;
	unsigned long phy_addr;
	unsigned long addr_align;
	unsigned long ispmmu_addr;
	unsigned long mmap_addr;

	u8 locked;
	u16 frame_num;
	struct isp_af_xtrastats xtrastats;
	struct isp_af_buffer *next;
};

/**
 * struct isp_af_status - AF status.
 * @initialized: 1 - Buffers initialized.
 * @update: 1 - Update registers.
 * @stats_req: 1 - Future stats requested.
 * @stats_done: 1 - Stats ready for user.
 * @frame_req: Number of frame requested for statistics.
 * @af_buff: Array of statistics buffers to access.
 * @stats_buf_size: Statistics buffer size.
 * @curr_cfg_buf_size: Current user configured stats buff size.
 * @min_buf_size: Minimum statisitics buffer size.
 * @frame_count: Frame Count.
 * @stats_wait: Wait primitive for locking/unlocking the stats request.
 * @buffer_lock: Spinlock for statistics buffers access.
 */
static struct isp_af_status {
	u8 initialized;
	u8 update;
	u8 stats_req;
	u8 stats_done;
	u16 frame_req;

	struct isp_af_buffer af_buff[H3A_MAX_BUFF];
	unsigned int stats_buf_size;
	unsigned int min_buf_size;
	unsigned int curr_cfg_buf_size;

	u32 frame_count;
	wait_queue_head_t stats_wait;
	spinlock_t buffer_lock;		/* For stats buffers read/write sync */
} afstat;

struct af_device *af_dev_configptr;
static struct isp_af_buffer *active_buff;
static int af_major = -1;
static int camnotify;


/**
 * isp_af_setxtrastats - Receives extra statistics from prior frames.
 * @xtrastats: Pointer to structure containing extra statistics fields like
 *             field count and timestamp of frame.
 *
 * Called from update_vbq in camera driver
 **/
void isp_af_setxtrastats(struct isp_af_xtrastats *xtrastats, u8 updateflag)
{
	int i, past_i;

	if (active_buff == NULL)
		return;

	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if (afstat.af_buff[i].frame_num == active_buff->frame_num)
			break;
	}

	if (i == H3A_MAX_BUFF)
		return;

	down(&ispaf_mutex);
	if (i == 0) {
		if (afstat.af_buff[H3A_MAX_BUFF - 1].locked == 0)
			past_i = H3A_MAX_BUFF - 1;
		else
			past_i = H3A_MAX_BUFF - 2;
	} else if (i == 1) {
		if (afstat.af_buff[0].locked == 0)
			past_i = 0;
		else
			past_i = H3A_MAX_BUFF - 1;
	} else {
		if (afstat.af_buff[i - 1].locked == 0)
			past_i = i - 1;
		else
			past_i = i - 2;
	}

	if (updateflag & AF_UPDATEXS_TS)
		afstat.af_buff[past_i].xtrastats.ts = xtrastats->ts;

	if (updateflag & AF_UPDATEXS_FIELDCOUNT)
		afstat.af_buff[past_i].xtrastats.field_count =
							xtrastats->field_count;
	up(&ispaf_mutex);
}
EXPORT_SYMBOL(isp_af_setxtrastats);

/*
 * Helper function to update buffer cache pages
 */
static void isp_af_update_req_buffer(struct isp_af_buffer *buffer)
{
	int size = afstat.stats_buf_size;

	size = PAGE_ALIGN(size);
	/* Update the kernel pages of the requested buffer */
	dmac_inv_range((void *)buffer->addr_align, (void *)buffer->addr_align +
									size);
}

/* Function to check paxel parameters */
int isp_af_check_paxel(void)
{
	/* Check horizontal Count */
	if ((af_dev_configptr->config->paxel_config.hz_cnt
	     < AF_PAXEL_HORIZONTAL_COUNT_MIN)
	    || (af_dev_configptr->config->paxel_config.hz_cnt
		> AF_PAXEL_HORIZONTAL_COUNT_MAX)) {
		DPRINTK_ISPH3A("Error : Horizontal Count is incorrect");
		return -AF_ERR_HZ_COUNT;
	}

	/*Check Vertical Count */
	if ((af_dev_configptr->config->paxel_config.vt_cnt
	     < AF_PAXEL_VERTICAL_COUNT_MIN)
	    || (af_dev_configptr->config->paxel_config.vt_cnt
		> AF_PAXEL_VERTICAL_COUNT_MAX)) {
		DPRINTK_ISPH3A("Error : Vertical Count is incorrect");
		return -AF_ERR_VT_COUNT;
	}

	/*Check Height */
	if ((af_dev_configptr->config->paxel_config.height
	     < AF_PAXEL_HEIGHT_MIN)
	    || (af_dev_configptr->config->paxel_config.height
		> AF_PAXEL_HEIGHT_MAX)) {
		DPRINTK_ISPH3A("Error : Height is incorrect");
		return -AF_ERR_HEIGHT;
	}

	/*Check width */
	if ((af_dev_configptr->config->paxel_config.width < AF_PAXEL_WIDTH_MIN)
	    || (af_dev_configptr->config->paxel_config.width
		> AF_PAXEL_WIDTH_MAX)) {
		DPRINTK_ISPH3A("Error : Width is incorrect");
		return -AF_ERR_WIDTH;
	}

	/*Check Line Increment */
	if ((af_dev_configptr->config->paxel_config.line_incr
	     < AF_PAXEL_INCREMENT_MIN)
	    || (af_dev_configptr->config->paxel_config.line_incr
		> AF_PAXEL_INCREMENT_MAX)) {
		DPRINTK_ISPH3A("Error : Line Increment is incorrect");
		return -AF_ERR_INCR;
	}

	/*Check Horizontal Start */
	if ((af_dev_configptr->config->paxel_config.hz_start % 2 != 0)
	    || (af_dev_configptr->config->paxel_config.hz_start
		< (af_dev_configptr->config->iir_config.hz_start_pos + 2))
	    || (af_dev_configptr->config->paxel_config.hz_start
		> AF_PAXEL_HZSTART_MAX)
	    || (af_dev_configptr->config->paxel_config.hz_start
		< AF_PAXEL_HZSTART_MIN)) {
		DPRINTK_ISPH3A("Error : Horizontal Start is incorrect");
		return -AF_ERR_HZ_START;
	}

	/*Check Vertical Start */
	if ((af_dev_configptr->config->paxel_config.vt_start
	     < AF_PAXEL_VTSTART_MIN)
	    || (af_dev_configptr->config->paxel_config.vt_start
		> AF_PAXEL_VTSTART_MAX)) {
		DPRINTK_ISPH3A("Error : Vertical Start is incorrect");
		return -AF_ERR_VT_START;
	}
	return 0;		/*Success */
}

/**
 * isp_af_check_iir - Function to check IIR Coefficient.
 **/
int isp_af_check_iir(void)
{
	int index;

	for (index = 0; index < AF_NUMBER_OF_COEF; index++) {
		if ((af_dev_configptr->config->iir_config.coeff_set0[index])
		    > AF_COEF_MAX) {
			DPRINTK_ISPH3A(
				"Error : Coefficient for set 0 is incorrect");
			return -AF_ERR_IIR_COEF;
		}

		if ((af_dev_configptr->config->iir_config.coeff_set1[index])
		    > AF_COEF_MAX) {
			DPRINTK_ISPH3A(
				"Error : Coefficient for set 1 is incorrect");
			return -AF_ERR_IIR_COEF;
		}
	}

	if ((af_dev_configptr->config->iir_config.hz_start_pos < AF_IIRSH_MIN)
	    || (af_dev_configptr->config->iir_config.hz_start_pos >
		AF_IIRSH_MAX)) {
		DPRINTK_ISPH3A("Error : IIRSH is incorrect");
		return -AF_ERR_IIRSH;
	}

	return 0;
}
/**
 * isp_af_unlock_buffers - Helper function to unlock all buffers.
 **/
static void isp_af_unlock_buffers(void)
{
	int i;
	unsigned long irqflags;

	spin_lock_irqsave(&afstat.buffer_lock, irqflags);
	for (i = 0; i < H3A_MAX_BUFF; i++)
		afstat.af_buff[i].locked = 0;

	spin_unlock_irqrestore(&afstat.buffer_lock, irqflags);
}

/*
 * Helper function to link allocated buffers
 */
static void isp_af_link_buffers(void)
{
	int i;

	down(&ispaf_mutex);
	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if ((i + 1) < H3A_MAX_BUFF)
			afstat.af_buff[i].next = &afstat.af_buff[i + 1];
		else
			afstat.af_buff[i].next = &afstat.af_buff[0];
	}
	up(&ispaf_mutex);
}

/* Function to perform hardware set up */
int isp_af_configure(struct af_configuration *afconfig)
{
	int result;
	int buff_size, i;
	unsigned int busyaf;

	if (NULL == afconfig) {
		printk(KERN_ERR "Null argument in configuration. \n");
		return -EINVAL;
	}

	down(&ispaf_mutex);
	memcpy(af_dev_configptr->config, afconfig,
					sizeof(struct af_configuration));

	/* Get the value of PCR register */
	busyaf = omap_readl(ISPH3A_PCR);

	if ((busyaf & AF_BUSYAF) == AF_BUSYAF) {
		/* DPRINTK_ISPH3A("AF_register_setup_ERROR : Engine Busy"); */
		/* DPRINTK_ISPH3A("\n Configuration cannot be done "); */
		/* return -AF_ERR_ENGINE_BUSY; */
		isp_af_enable(0);
	}

	/*Check IIR Coefficient and start Values */
	result = isp_af_check_iir();
	if (result < 0) {
		up(&ispaf_mutex);
		return result;
	}

	/*Check Paxel Values */
	result = isp_af_check_paxel();
	if (result < 0) {
		up(&ispaf_mutex);
		return result;
	}

	/*Check HMF Threshold Values */
	if (af_dev_configptr->config->hmf_config.threshold > AF_THRESHOLD_MAX) {
		DPRINTK_ISPH3A("Error : HMF Threshold is incorrect");
		up(&ispaf_mutex);
		return -AF_ERR_THRESHOLD;
	}

	/* Compute buffer size */
	buff_size =
	    (af_dev_configptr->config->paxel_config.hz_cnt + 1) *
	    (af_dev_configptr->config->paxel_config.vt_cnt + 1) * AF_PAXEL_SIZE;

	afstat.curr_cfg_buf_size = buff_size;
	/*Deallocate the previous buffers */
	if (afstat.stats_buf_size && (buff_size	> afstat.stats_buf_size)) {
		isp_af_enable(0);
		for (i = 0; i < H3A_MAX_BUFF; i++) {
			ispmmu_unmap(afstat.af_buff[i].ispmmu_addr);
			dma_free_coherent(NULL,
				  afstat.min_buf_size,
				  (void *)afstat.af_buff[i].virt_addr,
				  (dma_addr_t)afstat.af_buff[i].phy_addr);
			afstat.af_buff[i].virt_addr = 0;
		}
		afstat.stats_buf_size = 0;
	}

	if (!afstat.af_buff[0].virt_addr) {
		afstat.stats_buf_size = buff_size;
		afstat.min_buf_size = PAGE_ALIGN(afstat.stats_buf_size);

		for (i = 0; i < H3A_MAX_BUFF; i++) {
			afstat.af_buff[i].virt_addr =
				(unsigned long)dma_alloc_coherent(NULL,
						afstat.min_buf_size,
						(dma_addr_t *)
						 &afstat.af_buff[i].phy_addr,
						GFP_KERNEL | GFP_DMA);
			if (afstat.af_buff[i].virt_addr == 0) {
				printk(KERN_ERR "Can't acquire memory for "
					"buffer[%d]\n", i);
				up(&ispaf_mutex);
				return -ENOMEM;
			}
			afstat.af_buff[i].addr_align =
					afstat.af_buff[i].virt_addr;
			while ((afstat.af_buff[i].addr_align & 0xFFFFFFC0) !=
				       afstat.af_buff[i].addr_align)
				afstat.af_buff[i].addr_align++;
			afstat.af_buff[i].ispmmu_addr =
				ispmmu_map(afstat.af_buff[i].phy_addr,
					   afstat.min_buf_size);
		}
		isp_af_unlock_buffers();
		isp_af_link_buffers();

		/* First active buffer */
		if (active_buff == NULL)
			active_buff = &afstat.af_buff[0];
		isp_af_set_address(active_buff->ispmmu_addr);
	}

	result = isp_af_register_setup(af_dev_configptr);
	if (result < 0) {
		up(&ispaf_mutex);
		return result;
	}
	af_dev_configptr->size_paxel = buff_size;
	afstat.initialized = 1;
	/*Set configuration flag to indicate HW setup done */
	if (af_dev_configptr->config->af_config)
		isp_af_enable(1);

	/*Success */
	up(&ispaf_mutex);
	return 0;
}
EXPORT_SYMBOL(isp_af_configure);

int isp_af_register_setup(struct af_device *af_dev)
{
	unsigned int pcr = 0, pax1 = 0, pax2 = 0, paxstart = 0;
	unsigned int coef = 0;
	unsigned int base_coef_set0 = 0;
	unsigned int base_coef_set1 = 0;
	int index;


	/* Configure Hardware Registers */
	/* Set PCR Register */
	pcr = omap_readl(ISPH3A_PCR);	/* Read PCR Register */

	/*Set Accumulator Mode */
	if (af_dev->config->mode == ACCUMULATOR_PEAK)
		pcr |= FVMODE;
	else
		pcr &= ~FVMODE;

	/*Set A-law */
	if (af_dev->config->alaw_enable == H3A_AF_ALAW_ENABLE)
		pcr |= AF_ALAW_EN;
	else
		pcr &= ~AF_ALAW_EN;

	/*Set RGB Position */
	pcr &= ~RGBPOS;
	pcr |= (af_dev->config->rgb_pos) << AF_RGBPOS_SHIFT;

	/*HMF Configurations */
	if (af_dev->config->hmf_config.enable == H3A_AF_HMF_ENABLE) {
		pcr &= ~AF_MED_EN;
		/* Enable HMF */
		pcr |= AF_MED_EN;

		/* Set Median Threshold */
		pcr &= ~MED_TH;
		pcr |=
		    (af_dev->config->hmf_config.threshold) << AF_MED_TH_SHIFT;
	} else
		pcr &= ~AF_MED_EN;

	down(&ispaf_mutex);
	omap_writel(pcr, ISPH3A_PCR);

	pax1 &= ~PAXW;
	pax1 |= (af_dev->config->paxel_config.width) << AF_PAXW_SHIFT;

	/* Set height in AFPAX1 */
	pax1 &= ~PAXH;
	pax1 |= af_dev->config->paxel_config.height;

	omap_writel(pax1, ISPH3A_AFPAX1);

	/* Configure AFPAX2 Register */
	/* Set Line Increment in AFPAX2 Register */
	pax2 &= ~AFINCV;
	pax2 |= (af_dev->config->paxel_config.line_incr) << AF_LINE_INCR_SHIFT;
	/* Set Vertical Count */
	pax2 &= ~PAXVC;
	pax2 |= (af_dev->config->paxel_config.vt_cnt) << AF_VT_COUNT_SHIFT;
	/* Set Horizontal Count */
	pax2 &= ~PAXHC;
	pax2 |= af_dev->config->paxel_config.hz_cnt;
	omap_writel(pax2, ISPH3A_AFPAX2);

	/* Configure PAXSTART Register */
	/*Configure Horizontal Start */
	paxstart &= ~PAXSH;
	paxstart |=
	    (af_dev->config->paxel_config.hz_start) << AF_HZ_START_SHIFT;
	/* Configure Vertical Start */
	paxstart &= ~PAXSV;
	paxstart |= af_dev->config->paxel_config.vt_start;
	omap_writel(paxstart, ISPH3A_AFPAXSTART);

	/*SetIIRSH Register */
	omap_writel(af_dev->config->iir_config.hz_start_pos, ISPH3A_AFIIRSH);

	/*Set IIR Filter0 Coefficients */
	base_coef_set0 = ISPH3A_AFCOEF010;
	for (index = 0; index <= 8; index += 2) {
		coef &= ~COEF_MASK0;
		coef |= af_dev->config->iir_config.coeff_set0[index];
		coef &= ~COEF_MASK1;
		coef |=
		    (af_dev->config->iir_config.
		     coeff_set0[index + 1]) << AF_COEF_SHIFT;
		omap_writel(coef, base_coef_set0);

		base_coef_set0 = base_coef_set0 + AFCOEF_OFFSET;
	}

	/* set AFCOEF0010 Register */
	omap_writel(af_dev->config->iir_config.coeff_set0[10],
							ISPH3A_AFCOEF010);

	/*Set IIR Filter1 Coefficients */

	base_coef_set1 = ISPH3A_AFCOEF110;
	for (index = 0; index <= 8; index += 2) {
		coef &= ~COEF_MASK0;
		coef |= af_dev->config->iir_config.coeff_set1[index];
		coef &= ~COEF_MASK1;
		coef |=
		    (af_dev->config->iir_config.
		     coeff_set1[index + 1]) << AF_COEF_SHIFT;
		omap_writel(coef, base_coef_set1);

		base_coef_set1 = base_coef_set1 + AFCOEF_OFFSET;
	}
	omap_writel(af_dev->config->iir_config.coeff_set1[10],
							ISPH3A_AFCOEF1010);

	up(&ispaf_mutex);
	return 0;
}

/* Function to set address */
void isp_af_set_address(unsigned long address)
{
	spin_lock(&afstat.buffer_lock);
	omap_writel(address, ISPH3A_AFBUFST);
	spin_unlock(&afstat.buffer_lock);
}

static int isp_af_stats_available(struct isp_af_data *afdata)
{
	int i, ret;
	unsigned long irqflags;

	spin_lock_irqsave(&afstat.buffer_lock, irqflags);
	for (i = 0; i < H3A_MAX_BUFF; i++) {
		if ((afdata->frame_number == afstat.af_buff[i].frame_num)
			&& (afstat.af_buff[i].frame_num !=
				active_buff->frame_num)) {
			afstat.af_buff[i].locked = 1;
			spin_unlock_irqrestore(&afstat.buffer_lock, irqflags);
			isp_af_update_req_buffer(&afstat.af_buff[i]);
			afstat.af_buff[i].frame_num = 0;
			ret = copy_to_user(
					(void *)afdata->af_statistics_buf,
					(void *)afstat.af_buff[i].virt_addr,
					afstat.curr_cfg_buf_size);
			if (ret) {
				printk(KERN_ERR
					"Failed copy_to_user for "
					"H3A stats buff, %d\n",
					ret);
			}
			afdata->xtrastats.ts = afstat.af_buff[i].xtrastats.ts;
			afdata->xtrastats.field_count =
				afstat.af_buff[i].xtrastats.field_count;
			return 0;
		}
	}
	spin_unlock_irqrestore(&afstat.buffer_lock, irqflags);
	/* Stats unavailable */

	return -1;
}

void isp_af_notify(int notify)
{
	down(&ispaf_mutex);
	camnotify = notify;
	up(&ispaf_mutex);
	if (camnotify && afstat.initialized) {
		printk(KERN_DEBUG "Warning Camera Off \n");
		down(&ispaf_mutex);
		afstat.stats_req = 0;
		afstat.stats_done = 1;
		up(&ispaf_mutex);
		wake_up_interruptible(&afstat.stats_wait);
	}
}
EXPORT_SYMBOL(isp_af_notify);
/*
 * This API allows the user to update White Balance gains, as well as
 * exposure time and analog gain. It is also used to request frame
 * statistics.
 */
int isp_af_request_statistics(struct isp_af_data *afdata)
{
	int ret = 0;
	u16 frame_diff = 0;
	u16 frame_cnt = afstat.frame_count;
	wait_queue_t wqt;

	if (!af_dev_configptr->config->af_config) {
		printk(KERN_ERR "AF engine not enabled\n");
		return -EINVAL;
	}

	if (afdata->update != 0) {
		if (afdata->update & REQUEST_STATISTICS) {
			isp_af_unlock_buffers();
				/* Stats available? */
			DPRINTK_ISPH3A("Stats available?\n");
			ret = isp_af_stats_available(afdata);
			if (!ret)
				goto out;

			/* Stats in near future? */
			DPRINTK_ISPH3A("Stats in near future?\n");
			if (afdata->frame_number > frame_cnt) {
				frame_diff = afdata->frame_number - frame_cnt;
			} else if (afdata->frame_number < frame_cnt) {
				if ((frame_cnt >
					(MAX_FRAME_COUNT - MAX_FUTURE_FRAMES))
					&& (afdata->frame_number
						< MAX_FRAME_COUNT))
					frame_diff = afdata->frame_number
						    + MAX_FRAME_COUNT
						    - frame_cnt;
				else {
					/* Frame unavailable */
					frame_diff = MAX_FUTURE_FRAMES + 1;
				}
			}

			if (frame_diff > MAX_FUTURE_FRAMES) {
				printk(KERN_ERR "Invalid frame requested"
					", returning current frame stats\n");
				afdata->frame_number = frame_cnt;
			}
			if (!camnotify) {
				/* Block until frame in near future completes */
				down(&ispaf_mutex);
				afstat.frame_req = afdata->frame_number;
				afstat.stats_req = 1;
				afstat.stats_done = 0;
				up(&ispaf_mutex);
				init_waitqueue_entry(&wqt, current);
				ret =
				   wait_event_interruptible(afstat.stats_wait,
						afstat.stats_done == 1);
				if (ret < 0) {
					afdata->af_statistics_buf = NULL;
					return ret;
				}
			DPRINTK_ISPH3A("ISP AF request status"
						" interrupt raised\n");

				/* Stats now available */
				ret = isp_af_stats_available(afdata);
				if (ret) {
					printk(KERN_ERR "After waiting for"
						" stats, stats not available!!"
						"\n");
					afdata->af_statistics_buf = NULL;
				}
			}
		} else {
			afdata->af_statistics_buf = NULL;
		}
	} else {
		afdata->af_statistics_buf = NULL;
	}

out:
	afdata->curr_frame = afstat.frame_count;

	return 0;
}
EXPORT_SYMBOL(isp_af_request_statistics);

/* This function will handle the H3A interrupt. */
static void isp_af_isr(unsigned long status, isp_vbq_callback_ptr arg1,
								void *arg2)
{
	u16 frame_align;

	if ((H3A_AF_DONE & status) != H3A_AF_DONE)
		return;

	/* timestamp stats buffer */
	do_gettimeofday(&active_buff->xtrastats.ts);

	/* Exchange buffers */
	active_buff = active_buff->next;
	if (active_buff->locked == 1)
		active_buff = active_buff->next;
	isp_af_set_address(active_buff->ispmmu_addr);

	/* Update frame counter */
	afstat.frame_count++;
	frame_align = afstat.frame_count;
	if (afstat.frame_count > MAX_FRAME_COUNT) {
		afstat.frame_count = 1;
		frame_align++;
	}
	active_buff->frame_num = afstat.frame_count;

	/* Future Stats requested? */
	if (afstat.stats_req) {
		/* Is the frame we want already done? */
		if (frame_align >= (afstat.frame_req + 1)) {
			afstat.stats_req = 0;
			afstat.stats_done = 1;
			wake_up_interruptible(&afstat.stats_wait);
		}
	}
}

/* Function to Enable/Disable AF Engine */
int isp_af_enable(int enable)
{
	/* Before enabling AF H3A we need to clear pending interrupts */
	omap_writel(IRQ0STATUS_H3A_AF_DONE_IRQ, ISP_IRQ0STATUS);

	/* Set AF_EN bit in PCR Register */
	if (enable) {
		if (isp_set_callback(CBK_H3A_AF_DONE, isp_af_isr,
						(void *)NULL, (void *)NULL)) {
			printk(KERN_ERR "No callback for AF\n");
			return -EINVAL;
		}

		omap_writel((omap_readl(ISPH3A_PCR) | AF_EN), ISPH3A_PCR);
	} else {
		u32 timeout = 20;
		isp_unset_callback(CBK_H3A_AF_DONE);
		omap_writel((omap_readl(ISPH3A_PCR) & ~AF_EN), ISPH3A_PCR);
		while ((omap_readl(ISPH3A_PCR) & AF_BUSYAF) && timeout) {
			mdelay(10);
			timeout--;
		}
		if (timeout == 0) {
			printk(KERN_DEBUG "%s - can't disable AF H3A\n",
								__func__);
		}
	}
	return 0;
}

/* Function to register the AF character device driver. */
int __init isp_af_init(void)
{
	/*allocate memory for device structure and initialize it with 0 */
	af_dev_configptr = kzalloc(sizeof(struct af_device), GFP_KERNEL);
	if (!af_dev_configptr)
		goto err_nomem1;

	active_buff = NULL;

	af_dev_configptr->config = (struct af_configuration *)
			kzalloc(sizeof(struct af_configuration), GFP_KERNEL);

	if (af_dev_configptr->config == NULL)
		goto err_nomem2;

	printk(KERN_DEBUG "isp_af_init\n");
	memset(&afstat, 0, sizeof(afstat));

	init_waitqueue_head(&afstat.stats_wait);
	spin_lock_init(&afstat.buffer_lock);

	return 0;

err_nomem2:
	kfree(af_dev_configptr);
err_nomem1:
	printk(KERN_ERR "Error: kmalloc fail");
	return -ENOMEM;
}

void __exit isp_af_exit(void)
{
	int i;

	if (afstat.af_buff) {
		/* Free buffers */
		for (i = 0; i < H3A_MAX_BUFF; i++) {
			ispmmu_unmap(afstat.af_buff[i].ispmmu_addr);
			dma_free_coherent(NULL,
				afstat.min_buf_size,
				(void *)afstat.af_buff[i].virt_addr,
				(dma_addr_t)afstat.af_buff[i].phy_addr);
		}
	}
	kfree(af_dev_configptr->config);
	kfree(af_dev_configptr);

	memset(&afstat, 0, sizeof(afstat));

	af_major = -1;
	isp_af_enable(0);
}
