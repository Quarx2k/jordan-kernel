/*
 * Copyright (C) 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* Necessary includes for device drivers */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>	/* printk() */
#include <linux/slab.h>		/* kmalloc() */
#include <linux/fs.h>
#include <linux/errno.h>	/* error codes */
#include <linux/types.h>	/* size_t */
#include <linux/proc_fs.h>
#include <linux/fcntl.h>
#include <asm/system.h>		/* cli(), *_flags */
#include <linux/uaccess.h>	/* copy_from/to_user */
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/io.h>

#define SIZE_OF_MODEL_ID 4
#define SIZE_OF_SWRV     (20)
#define SIZE_OF_PROC_ID   16

#define LARGER_OF_TWO (n, m) (n > m ? n : m)
#define LARGER_OF_THREE (x, y, z) \
	(x > (LARGER_OF_TWO(y, z)) ? x : LARGER_OF_TWO(y, z))

#define API_HAL_KM_SOFTWAREREVISION_READ          33

#define FLAG_IRQFIQ_MASK            0x3
#define FLAG_START_HAL_CRITICAL     0x4
#define FLAG_IRQ_ENABLE             0x2
#define FLAG_FIQ_ENABLE             0x1

#define SMICODEPUB_IRQ_END   0xFE
#define SMICODEPUB_FIQ_END   0xFD
#define SMICODEPUB_RPC_END   0xFC

#define REGISTER_ADDRESS_DIE_ID  0x4830A218
#define REGISTER_ADDRESS_MSV 0x480023B4

#define SEC_IOCTL_MODEL _IOWR(0x99, 100, int*)
#define SEC_IOCTL_SWRV _IOWR(0x99, 101, int*)
#define SEC_IOCTL_PROC_ID _IOWR(0x99, 102, int*)
#define SEC_IOCTL_EFUSE_RAISE _IOWR(0x99, 103, int*)
#define SEC_IOCTL_EFUSE_LOWER _IOWR(0x99, 104, int*)
#define SEC_IOCTL_MODELID_PROV _IOWR(0x99, 105, int)
#define SEC_IOCTL_BS_DIS   _IOWR(0x99, 106, int)
#define SEC_IOCTL_PROD_PROV _IOWR(0x99, 107, int)

#define API_HAL_NB_MAX_SVC         39
#define API_HAL_MOT_EFUSE            (API_HAL_NB_MAX_SVC + 10)
#define API_HAL_MOT_EFUSE_READ       (API_HAL_NB_MAX_SVC + 15)

/*===========================*
   struct SEC_PA_PARAMS
*============================*/
typedef struct {
	unsigned int component;
	unsigned int efuse_value;
	unsigned int bch_value;
} SEC_PA_PARAMS;

/*============================
   enum SEC_SV_COMPONENT_T
*============================*/
typedef enum {
	/*Starting with random non zero value for component type */
	SEC_AP_PA_PPA = 0x00000065,
	SEC_BP_PPA,
	SEC_BP_PA,
	SEC_ML_PBRDL,
	SEC_MBM,
	SEC_RRDL_BRDL,
	SEC_BPL,
	SEC_AP_OS,
	SEC_BP_OS,
	SEC_BS_DIS,
	SEC_ENG,
	SEC_PROD,
	SEC_CUST_CODE,
	SEC_PKC,
	SEC_MODEL_ID,
	SEC_MAX
} SEC_SV_COMPONENT_T;


/* Declaration of sec.c functions */
int sec_open(struct inode *inode, struct file *filp);
int sec_release(struct inode *inode, struct file *filp);
ssize_t sec_read(struct file *filp, char *buf, size_t count,
		loff_t *f_pos);
ssize_t sec_write(struct file *filp, char *buf, size_t count,
		loff_t *f_pos);
int sec_ioctl(struct inode *inode, struct file *file,
	      unsigned int ioctl_num, unsigned long ioctl_param);



extern u32 pub2sec_bridge_entry(u32 appl_id, u32 proc_ID, u32 flag,
				char *ptr);
extern u32 rpc_handler(u32 p1, u32 p2, u32 p3, u32 p4);
extern u32 v7_flush_kern_cache_all(void);

void sec_exit(void);

int sec_init(void);

static void SecGetModelId(void *);

static void SecGetSWRV(void *);

static void SecGetProcID(void *);
