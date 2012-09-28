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

#include "sec_core.h"
#include "sec_ks_external.h"
#include <linux/kernel.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/cpu.h>


static void SecGetModelId(void *data);

static void SecGetSWRV(void *data);

static void SecGetProcID(void *data);


static void SecRaiseVfuse(void);

static void SecLowerVfuse(void);

static int SecProvisionModelID(unsigned int model_id);

static int SecProvisionProd(unsigned int prod);

static u32
SEC_ENTRY_pub2sec_dispatcher(u32 appl_id, u32 proc_ID, u32 flag, ...);

static int SecBSDis(void);

static int SecVfuseOff(void);
static int SecVfuseOn(void);
static int SecFuseInit(void);

static struct regulator *sec_efuse_regulator;


/* Structure that declares the usual file */
/* access functions */
const struct file_operations sec_fops = {
	.read = sec_read,
	.open = sec_open,
	.release = sec_release,
	.ioctl = sec_ioctl
};

/* Mapping of the module init and exit functions */
module_init(sec_init);
module_exit(sec_exit);

static struct miscdevice sec_dev = {
	MISC_DYNAMIC_MINOR,
	"sec",
	&sec_fops
};

/******************************************************************************/
/*   KERNEL DRIVER APIs, ONLY IOCTL RESPONDS BACK TO USER SPACE REQUESTS      */
/******************************************************************************/
int sec_init(void)
{
	int result;

	result = misc_register(&sec_dev);

	if (result) {
		printk(KERN_ERR "sec: cannot obtain major number \n");
		return result;
	}

	printk(KERN_INFO "Inserting sec module\n");
	return 0;
}

void sec_exit(void)
{
	/* Freeing the major number */
	misc_deregister(&sec_dev);
}

int sec_open(struct inode *inode, struct file *filp)
{
	/* Not supported, return Success */
	return 0;
}

int sec_release(struct inode *inode, struct file *filp)
{

	/* Not supported, return Success */
	return 0;
}

ssize_t sec_read(struct file *filp, char *buf,
		 size_t count, loff_t *f_pos)
{
	/* Not supported, return Success */
	return 0;
}

ssize_t sec_write(struct file *filp, char *buf,
		  size_t count, loff_t *f_pos)
{
	/* Not supported, return Success */
	return 0;
}

int sec_ioctl(struct inode *inode, struct file *file,
	      unsigned int ioctl_num, unsigned long ioctl_param)
{
	unsigned long count = 0xDEADBEEF;
	void *sec_buffer = NULL;
	int ret_val = 99;

	switch (ioctl_num) {
	case SEC_IOCTL_MODEL:
		sec_buffer = kmalloc(SIZE_OF_MODEL_ID, GFP_KERNEL);
		if (sec_buffer != NULL) {
			SecGetModelId(sec_buffer);
			count =
			    copy_to_user((void __user *) ioctl_param,
					 (const void *) sec_buffer,
					 SIZE_OF_MODEL_ID);
			kfree(sec_buffer);
			ret_val = 0;
		}

		break;

	case SEC_IOCTL_MODELID_PROV:
		ret_val = SecProvisionModelID(ioctl_param);

		break;

	case SEC_IOCTL_PROD_PROV:
		ret_val = SecProvisionProd(ioctl_param);

		break;


	case SEC_IOCTL_SWRV:

		sec_buffer = kmalloc(SIZE_OF_SWRV, GFP_KERNEL);

		if (sec_buffer != NULL) {
			SecGetSWRV(sec_buffer);
			count =
			    copy_to_user((void __user *) ioctl_param,
					 (const void *) sec_buffer,
					 SIZE_OF_SWRV);
			kfree(sec_buffer);
			ret_val = 0;
		}
		break;

	case SEC_IOCTL_PROC_ID:
		sec_buffer = kmalloc(SIZE_OF_PROC_ID, GFP_KERNEL);
		if (sec_buffer != NULL) {
			SecGetProcID(sec_buffer);
			count =
			    copy_to_user((void __user *) ioctl_param,
					 (const void *) sec_buffer,
					 SIZE_OF_PROC_ID);
			kfree(sec_buffer);
			ret_val = 0;
		}
		break;

	case SEC_IOCTL_EFUSE_RAISE:
		SecRaiseVfuse();
		ret_val = 0;
		break;

	case SEC_IOCTL_EFUSE_LOWER:
		SecLowerVfuse();
		ret_val = 0;
		break;

	case SEC_IOCTL_BS_DIS:
		ret_val = SecBSDis();
		break;

	default:
		printk("sec ioctl called with bad cmd : %d ", ioctl_num);
		break;
	}

	if (count != 0) {
		printk("sec ioctl operation : %d failed, \
			copy_to_user returned: 0x%lX", ioctl_num, count);
	}

	return ret_val;
}

static void SecGetModelId(void *data)
{
	volatile unsigned int fuse_value = 0;
	memset(data, 0xFF, SIZE_OF_MODEL_ID);
	fuse_value = omap_readl(REGISTER_ADDRESS_MSV);
	memcpy(data, (void *) &fuse_value, sizeof(unsigned int));

	return;
}

static void SecGetSWRV(void *data)
{
	int ret_val = 99;

	memset(data, 0xFF, SIZE_OF_SWRV);
	ret_val = SEC_ENTRY_pub2sec_dispatcher(API_HAL_MOT_EFUSE_READ,
					       0,
					       FLAG_IRQ_ENABLE |
					       FLAG_FIQ_ENABLE |
					       FLAG_START_HAL_CRITICAL, 1,
					       __pa(data));

	if (ret_val != 0) {
		memset(data, 0xFF, SIZE_OF_SWRV);
	}
	return;
}

static void SecGetProcID(void *data)
{
	int i = 0;
	volatile unsigned int *fuse_ptr = NULL;
	volatile unsigned int fuse_value = 0;
	memset(data, 0xFF, SIZE_OF_PROC_ID);
	fuse_ptr = (unsigned int *) REGISTER_ADDRESS_DIE_ID;
	for (i = 0; i < 4; i++) {
		fuse_value = omap_readl((u32)(fuse_ptr + i));
		memcpy(data + (4 * i), (void *) &fuse_value,
		       sizeof(unsigned int));
	}
	return;
}

static void SecRaiseVfuse(void)
{
	int ret_value = 99;

	if (sec_efuse_regulator == NULL) {
		ret_value = SecFuseInit();
		if (ret_value != 0) {
			printk
			    ("Registration regulator framework failed:%d",
			     ret_value);
		}
	}

	if (ret_value == 0 || ret_value == 99)
		ret_value = SecVfuseOn();

	mdelay(10);

	return;
}

static void SecLowerVfuse(void)
{
	int ret_value = 99;

	ret_value = SecVfuseOff();
	if (ret_value != 0)
		printk(" Efuse voltage lowering failed : %d", ret_value);
	return;
}

static int SecProvisionModelID(unsigned int model_id)
{
	unsigned int result = 99;
	int ret_val = 99;
	SEC_PA_PARAMS ns_efuse_params;

	ns_efuse_params.component = SEC_MODEL_ID;
	ns_efuse_params.efuse_value = model_id;

	ns_efuse_params.bch_value = 99;

	result = SEC_ENTRY_pub2sec_dispatcher(API_HAL_MOT_EFUSE,
					      0,
					      FLAG_IRQFIQ_MASK |
					      FLAG_START_HAL_CRITICAL, 1,
					      (void *)
					      __pa(&ns_efuse_params));
	if (result == 0)
		ret_val = 0;

	return ret_val;
}

static int SecProvisionProd(unsigned int prod)
{
	unsigned int result = 99;
	int ret_val = 99;
	SEC_PA_PARAMS ns_efuse_params;

	ns_efuse_params.component = SEC_PROD;
	ns_efuse_params.efuse_value = prod;

	ns_efuse_params.bch_value = 99;

	result = SEC_ENTRY_pub2sec_dispatcher(API_HAL_MOT_EFUSE,
					      0,
					      FLAG_IRQFIQ_MASK |
					      FLAG_START_HAL_CRITICAL, 1,
					      (void *)
					      __pa(&ns_efuse_params));
	if (result == 0)
		ret_val = 0;

	return ret_val;
}

static int SecBSDis()
{
	unsigned int result = 99;
	int ret_val = 99;
	SEC_PA_PARAMS ns_efuse_params;

	ns_efuse_params.component = SEC_BS_DIS;
	ns_efuse_params.efuse_value = 0x01;
	ns_efuse_params.bch_value = 99;
	result = SEC_ENTRY_pub2sec_dispatcher(API_HAL_MOT_EFUSE,
					      0,
					      FLAG_IRQFIQ_MASK |
					      FLAG_START_HAL_CRITICAL, 1,
					      (void *)
					      __pa(&ns_efuse_params));
	if (result == 0)
		ret_val = 0;

	return ret_val;

}

/*----------------------------------------------------------------------------
 * Function responsible for formatting the parameters to pass
 * from NS-World to S-World.
 *----------------------------------------------------------------------------*/
u32 SEC_ENTRY_pub2sec_dispatcher(u32 appl_id, u32 proc_ID, u32 flag, ...)
{
	u32 return_value = 0;
	u32 *pArgs = NULL;

	va_list args;
	int temp_counter;

	/*
	 * We need a physically contiguous buffer to pass parameters to the SE
	 */
	pArgs = (u32 *) kmalloc(sizeof(u32) * 5, GFP_KERNEL);
	if (pArgs == NULL)
		return -ENOMEM;

	va_start(args, flag);

	for (temp_counter = 0; temp_counter < 5; temp_counter++)
		pArgs[temp_counter] = va_arg(args, int);

	va_end(args);


	/*
	 * OMAP3430 Secure ROM Code Functional Specification:
	 *    L2 Cache is not used by SW which runs in Secure Mode.
	 *    Thus, the non-Secure World's software must ensure that any data
	 *    in L2 Cache are coherent with memory before feeding such data to the
	 *    Secure World for processing.
	 */
	v7_flush_kern_cache_all();

	return_value =
	    pub2sec_bridge_entry(appl_id, proc_ID, flag,
				 (char *) __pa(pArgs));

	kfree(pArgs);

	return return_value;
}


static int SecFuseInit(void)
{
	int ret_value = 99;
	struct regulator *reg;

	reg = regulator_get(NULL, "vfuse");
	if (reg)
		sec_efuse_regulator = reg;

	return ret_value;

}


static int SecVfuseOn(void)
{
	int ret_value = 99;

	if (sec_efuse_regulator == NULL)
		return 99;

	if (cpu_is_omap3630())
		regulator_set_voltage(sec_efuse_regulator, 1700000, 1700000);
	else
		regulator_set_voltage(sec_efuse_regulator, 1900000, 1900000);

	ret_value = regulator_enable(sec_efuse_regulator);
	return ret_value;

}

static int SecVfuseOff(void)
{
	int ret_value = 99;

	if (sec_efuse_regulator == NULL)
		return 99;
	ret_value = regulator_disable(sec_efuse_regulator);
	return ret_value;

}

/******************************************************************************/
/*Functions exported to Kernel for other kernel services                      */
/******************************************************************************/
SEC_STAT_T SecProcessorID(unsigned char *buffer, int length)
{
	int counter = 0;
	volatile unsigned int *fuse_ptr = NULL;
	volatile unsigned int fuse_value = 0;
	if (length < SIZE_OF_PROC_ID)
		return SEC_FAIL;

	memset(buffer, 0xFF, length);
	fuse_ptr = (unsigned int *) REGISTER_ADDRESS_DIE_ID;

	for (counter = 0; counter < 4; counter++) {
		fuse_value = omap_readl((u32)(fuse_ptr + counter));
		memcpy(buffer + (4 * counter), (void *) &fuse_value,
		       sizeof(unsigned int));
	}
	return SEC_SUCCESS;
}

EXPORT_SYMBOL(SecProcessorID);

SEC_STAT_T SecModelID(unsigned char *buffer, int length)
{
	volatile unsigned int fuse_value = 0;

	if (length < SIZE_OF_MODEL_ID)
		return SEC_FAIL;
	memset(buffer, 0xFF, length);
	fuse_value = omap_readl(REGISTER_ADDRESS_MSV);
	memcpy(buffer, (void *) &fuse_value, sizeof(unsigned int));
	return SEC_SUCCESS;

}

EXPORT_SYMBOL(SecModelID);

SEC_MODE_T SecProcessorType(void)
{
	SEC_MODE_T ret_val = SEC_PRODUCTION;
	/* UINT16 : unsigned short int */
	u32 *iterator = NULL;
	u32 *data = (u32 *) kmalloc(SIZE_OF_SWRV, GFP_KERNEL);

	memset(data, 0xFF, SIZE_OF_SWRV);
	SEC_ENTRY_pub2sec_dispatcher(API_HAL_MOT_EFUSE_READ, 0,
				     FLAG_IRQFIQ_MASK |
				     FLAG_START_HAL_CRITICAL, 1,
				     (void *) __pa(data));
	if (ret_val != 0)
		memset(data, 0xFF, SIZE_OF_SWRV);

	iterator = data + 4;

	/*HAB_ENG : 13, HAB_PROD : 14 */
	/*Engineering only if engineering blown and production not */
	if (((*iterator) & (0x3 << 13)) == (0x1 << 13))
		ret_val = SEC_ENGINEERING;

	kfree(data);

	return ret_val;

}

EXPORT_SYMBOL(SecProcessorType);



/******************************************************************************/
/*Kernel Module License Information                                           */
/******************************************************************************/
MODULE_LICENSE("GPL");
MODULE_AUTHOR("MOTOROLA");
