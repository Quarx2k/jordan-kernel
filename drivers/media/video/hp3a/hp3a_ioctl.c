/*
 * drivers/media/video/hp3a/hp3a_ioctl.c
 *
 * HP Imaging/3A Driver : hp3a device ioctl handler implementation.
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

#include <linux/device.h>
#include <linux/fs.h>
#include <linux/platform_device.h>
#include <media/v4l2-dev.h>

#include "hp3a_common.h"
#include "hp3a_user.h"
#include "ispmmu.h"

 /**
 * hp3a_unlocked_ioctl - I/O control function for hp3a module
 * @inode: Inode structure associated with the hp3a Wrapper.
 * @file: File structure associated with the hp3a driver.
 * @cmd: Type of command to execute.
 * @arg: Argument to send to requested command.
 *
 * Returns 0 if successful, -1 if bad command passed or access is denied,
 * -EFAULT if copy_from_user() or copy_to_user()  fails,
 * -EINVAL if parameter validation fails or parameter structure is not present.
 **/
long hp3a_unlocked_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	int ret = -1;
	struct hp3a_fh *fh = file->private_data;
	struct hp3a_dev *device = fh->device;

	if (unlikely(_IOC_TYPE(cmd) != OMAP3_HP3A_MAGIC)) {
		dev_err(device->dev, "Bad command value (%d)\n", cmd);
		return  -ENOTTY;
	}

	switch (cmd) {
	/*
	 * Gets all statistics.
	 */
	case HP3A_G_STATISTICS: {
		struct hp3a_statistics statistics;

		ret = hp3a_collect_statistics(&statistics);
		if (SUCCEEDED(ret)) {
			if (copy_to_user((struct hp3a_statistics *)arg,
				&statistics,
				sizeof(struct hp3a_statistics)) != 0)
				ret = -EFAULT;
		}
		break;
	}
	/*
	 * Set sensor parameters.
	 */
	case HP3A_S_SENSOR_PARAM: {
		struct hp3a_sensor_param sensor_param;

		if (copy_from_user(&sensor_param,
				(struct hp3a_sensor_param *)arg,
				sizeof(struct hp3a_sensor_param)) == 0) {
			ret = hp3a_set_sensor_param(&sensor_param, fh);
		} else {
			ret = -EFAULT;
		}

		break;
	}
	/*
	 * Set ISP/Hardpipe parameters
	 */
	case HP3A_S_HARDPIPE_PARAM: {
		struct hp3a_hardpipe_param hpipe_param;

		if (copy_from_user(&hpipe_param,
				(struct hp3a_hardpipe_param *)arg,
				sizeof(struct hp3a_hardpipe_param)) == 0) {
			ret = hp3a_set_hardpipe_param(&hpipe_param, fh);
		} else {
			ret = -EFAULT;
		}

		break;
	}
	/*
	 * Queue histogram stat buffer.
	 */
	case HP3A_QBUF_HISTQ: {
		struct hp3a_internal_buffer *ibuffer;
		int index = (int)arg;

		if (index < fh->buffer_count && index > -1) {
			ibuffer = &(fh->buffers[index]);
			ret = hp3a_enqueue_irqsave(&g_tc.hist_stat_queue,
				&ibuffer);
		}

		break;
	}
	/*
	 * Queue AF stat buffer.
	 */
	case HP3A_QBUF_AFQ: {
		struct hp3a_internal_buffer *ibuffer;
		int index = (int)arg;

		if (index < fh->buffer_count && index > -1) {
			ibuffer = &(fh->buffers[index]);
			if (ibuffer->isp_addr == 0) {
		#if defined(CONFIG_VIDEO_OLDOMAP3)
				ibuffer->isp_addr =
					ispmmu_map_pages(ibuffer->pages,
					NR_PAGES((unsigned long)ibuffer->user_addr,
					ibuffer->buffer_size));
				if (ibuffer->isp_addr == 0) {
					dev_err(device->dev , \
						"isp mmu fail to map memory\n");
					return -EFAULT;
				}
		#else
				ibuffer->isp_addr =
					ispmmu_vmap_pages(ibuffer->pages,
					NR_PAGES((unsigned long)ibuffer->user_addr,
					ibuffer->buffer_size));
				if (IS_ERR((void *)ibuffer->isp_addr)) {
					ibuffer->isp_addr = 0;
					dev_err(device->dev , \
						"isp mmu fail to map memory\n");
					return -EFAULT;
				}
		#endif
			}
			flush_dcache_ibuffer(ibuffer);
			ret = hp3a_enqueue_irqsave(&g_tc.af_stat_queue,
				&ibuffer);
		}

		break;
	}
	/*
	 * Queue RAW frame buffer.
	 */
	case HP3A_QBUF_RAWQ: {
		struct hp3a_internal_buffer *ibuffer ;
		int index = (int)arg;

		if (index < fh->buffer_count && index > -1) {
			ibuffer = &(fh->buffers[index]);
			if (ibuffer->isp_addr == 0) {
		#if defined(CONFIG_VIDEO_OLDOMAP3)
				ibuffer->isp_addr =
					ispmmu_map_pages(ibuffer->pages,
					NR_PAGES((unsigned long)ibuffer->user_addr,
					ibuffer->buffer_size));
				if (ibuffer->isp_addr == 0) {
					dev_err(device->dev , \
						"isp mmu fail to map memory\n");
					return -EFAULT;
				}
		#else
				ibuffer->isp_addr =
					ispmmu_vmap_pages(ibuffer->pages,
					NR_PAGES((unsigned long)ibuffer->user_addr,
					ibuffer->buffer_size));
				if (IS_ERR((void *)ibuffer->isp_addr)) {
					ibuffer->isp_addr = 0;
					dev_err(device->dev , \
						"isp mmu fail to map memory\n");
					return -EFAULT;
				}
		#endif
			}
			ret = hp3a_enqueue_irqsave(&g_tc.raw_frame_queue,
				&ibuffer);
		}

		break;
	}
	/*
	 * Request for array of buffer placeholders.
	 */
	case HP3A_REQBUF: {
		struct hp3a_request_bufffers req_buf;
		int i;

		if (copy_from_user(&req_buf,
				(struct hp3a_request_bufffers *)arg,
				sizeof(struct hp3a_request_bufffers)) == 0) {
			ret = -1;
			if (req_buf.count > 0) {
				fh->buffers = kzalloc(req_buf.count * \
					sizeof(struct hp3a_internal_buffer),
					GFP_KERNEL);
				if (fh->buffers) {
					fh->buffer_count = req_buf.count;
					for (i = 0; i < fh->buffer_count; ++i)
						fh->buffers[i].index = i;
					ret = 0;
				}
			}
		} else {
			ret = -EFAULT;
		}

		break;
	}
	/*
	 * Install buffers into internal list and map to kernel space.
	 */
	case HP3A_INSTALL_BUF: {
		struct hp3a_buffer buffer;

		if (copy_from_user(&buffer,
				(struct hp3a_buffer *)arg,
				sizeof(struct hp3a_buffer)) == 0) {
			ret = -1;
			if (buffer.index >= 0 && buffer.index <
				fh->buffer_count) {
				if (fh->buffers[buffer.index].buffer_size
					==	0) {
					ret = map_user_to_kernel(&buffer,
						&(fh->buffers[buffer.index]));
				}
			}
		} else {
			ret = -EFAULT;
		}

		break;
	}
	/*
	 * Remove buffers from internal list and unmap from kernel space.
	 */
	case HP3A_UNINSTALL_BUF: {
		struct hp3a_buffer buffer;

		if (copy_from_user(&buffer,
				(struct hp3a_buffer *)arg,
				sizeof(struct hp3a_buffer)) == 0) {
			ret = -1;
			if (buffer.index >= 0 && buffer.index <
				fh->buffer_count) {
				if (fh->buffers[buffer.index].buffer_size &&
					fh->buffers[buffer.index].pages) {
					unmap_buffer_from_kernel(
						&(fh->buffers[buffer.index]));
					ret = 0;
				}
			}
		} else {
			ret = -EFAULT;
		}

		break;
	}
	/*
	 * Configure Histogram hardware.
	 */
	case HP3A_CONFIG_HIST: {
		struct hp3a_histogram_config config;

		if (copy_from_user(&config,
				(struct hp3a_histogram_config *)arg,
				sizeof(struct hp3a_histogram_config)) == 0) {
			ret = hp3a_config_histogram(&config, fh);
		} else {
			ret = -EFAULT;
		}

		break;
	}
	/*
	 * Configure Histogram hardware.
	 */
   case HP3A_CONFIG_AF: {
		struct hp3a_af_config config;

		if (copy_from_user(&config,
				(struct hp3a_af_config *)arg,
				sizeof(struct hp3a_af_config)) == 0) {
			ret = hp3a_config_af(&config, fh);
		} else {
			ret = -EFAULT;
		}

      break;
   }
	/*
	 * Configure raw hardware.
	 */
	case HP3A_CONFIG_RAW: {
		struct hp3a_raw_config config;

		if (copy_from_user(&config,
				(struct hp3a_raw_config *)arg,
				sizeof(struct hp3a_raw_config)) == 0) {

			ret = hp3a_configure_raw(&config);
			if (SUCCEEDED(ret)) {
				if (copy_to_user((struct hp3a_raw_config *)arg,
					&config,
					sizeof(struct hp3a_raw_config)) != 0)
					ret = -EFAULT;
			}
		} else {
			ret = -EFAULT;
		}

		break;
	}
	/*
	 * Flush histogram queue.
	 */
	case HP3A_FLUSH_HISTQ: {
		hp3a_flush_queue_irqsave(&g_tc.hist_stat_queue);
		ret = 0;
		break;
	}
	/*
	 * Flush AF queue.
	 */
	case HP3A_FLUSH_AFQ:	{
		hp3a_flush_queue_irqsave(&g_tc.af_stat_queue);
		ret = 0;
		break;
	}
	/*
	 * Flush RAW queue.
	 */
	case HP3A_FLUSH_RAWQ: {
		hp3a_flush_queue_irqsave(&g_tc.raw_frame_queue);
		ret = 0;
		break;
	}
	/*
	 * Set V4L2 device specific index.
	 */
	case HP3A_S_V4L2_DEV_INDEX: {
		fh->v4l2_dev = (int)arg;
		g_tc.default_v4l2_dev = fh->v4l2_dev;
		ret = 0;
		break;
	}
	/*
	 * Read ISP registers.
	 */
	case HP3A_READ_ISP_REGS: {
		ret = hp3a_read_ispregs_to_user((struct hp3a_reg_page *)arg);
		break;
	}
	case HP3A_READ_ISP_REG: {
		ret = hp3a_read_ispreg_to_user((struct hp3a_reg *)arg);
		break;
	}
	default:
		break;
	}

	return (long)ret;
}
