/*
 * drv_interface.c
 *
 * DSP-BIOS Bridge driver support functions for TI OMAP processors.
 *
 * Copyright (C) 2005-2006 Texas Instruments, Inc.
 *
 * This package is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THIS PACKAGE IS PROVIDED ``AS IS'' AND WITHOUT ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, WITHOUT LIMITATION, THE IMPLIED
 * WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 *  ======== linux_driver.c ========
 *  Description:
 *      DSP/BIOS Bridge driver interface.
 *
 *  Public Functions:
 *      driver_init
 *      driver_exit
 *      driver_open
 *      driver_release
 *      driver_ioctl
 *      driver_mmap
 *
 *! Revision History
 *! ================
 *! 21-Apr-2004 map   Deprecated use of MODULE_PARM for kernel versions
 *!		   greater than 2.5, use module_param.
 *! 08-Mar-2004 sb    Added the dsp_debug argument, which keeps the DSP in self
 *!		   loop after image load and waits in a loop for DSP to start
 *! 16-Feb-2004 vp    Deprecated the usage of MOD_INC_USE_COUNT and
 *! 						MOD_DEC_USE_COUNT
 *!		   for kernel versions greater than 2.5
 *! 20-May-2003 vp    Added unregister functions for the DPM.
 *! 24-Mar-2003 sb    Pass pid instead of driverContext to DSP_Close
 *! 24-Mar-2003 vp    Added Power Management support.
 *! 21-Mar-2003 sb    Configure SHM size using insmod argument shm_size
 *! 10-Feb-2003 vp    Updated based on code review comments
 *! 18-Oct-2002 sb    Created initial version
 */

/*  ----------------------------------- Host OS */

#include <dspbridge/host_os.h>
#include <linux/platform_device.h>
#include <linux/pm.h>

#ifdef MODULE
#include <linux/module.h>
#endif

#include <linux/device.h>
#include <linux/init.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/kobject.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>

/* XXX 
#include <mach/board-3430sdp.h>
*/

/*  ----------------------------------- DSP/BIOS Bridge */
#include <dspbridge/std.h>
#include <dspbridge/dbdefs.h>
#include <dspbridge/errbase.h>
#include <_tiomap.h>

/*  ----------------------------------- Trace & Debug */
#include <dspbridge/gt.h>
#include <dspbridge/dbc.h>

/*  ----------------------------------- OS Adaptation Layer */
#include <dspbridge/services.h>
#include <dspbridge/sync.h>
#include <dspbridge/reg.h>

/*  ----------------------------------- Platform Manager */
#include <dspbridge/wcdioctl.h>
#include <dspbridge/_dcd.h>
#include <dspbridge/dspdrv.h>

/*  ----------------------------------- Resource Manager */
#include <dspbridge/pwr.h>

/*  ----------------------------------- This */
#include <drv_interface.h>

#ifndef RES_CLEANUP_DISABLE
#include <dspbridge/cfg.h>
#include <dspbridge/resourcecleanup.h>
#include <dspbridge/chnl.h>
#include <dspbridge/proc.h>
#include <dspbridge/dev.h>
#include <dspbridge/drvdefs.h>
#include <dspbridge/drv.h>
#endif

#include <plat/omap-pm.h>
#include <mach-omap2/omap3-opp.h>

#define BRIDGE_NAME "C6410"
/*  ----------------------------------- Globals */
#define DRIVER_NAME  "DspBridge"
s32 dsp_debug;

struct platform_device *omap_dspbridge_dev;

struct bridge_dev {
	struct cdev cdev;
};

static struct bridge_dev *bridge_device;

static struct class *bridge_class;

static u32 driverContext;
#ifdef CONFIG_BRIDGE_DEBUG
static char *GT_str;
#endif /* CONFIG_BRIDGE_DEBUG */
static s32 driver_major;
static s32 driver_minor;
static char *base_img;
char *iva_img;
static s32 shm_size = 0x500000;	/* 5 MB */
static u32 phys_mempool_base;
static u32 phys_mempool_size;
static int tc_wordswapon;	/* Default value is always false */
#ifdef CONFIG_BRIDGE_RECOVERY
static DECLARE_COMPLETION_ONSTACK(bridge_open_comp);
static bool recover;
#endif

/* Minimum ACTIVE VDD1 OPP level for reliable DSP operation */
unsigned long min_active_opp_freq;

#ifdef CONFIG_PM
struct omap34xx_bridge_suspend_data {
	int suspended;
	wait_queue_head_t suspend_wq;
};

static struct omap34xx_bridge_suspend_data bridge_suspend_data;

static void bridge_create_sysfs(void);
static void bridge_destroy_sysfs(void);

static int omap34xxbridge_suspend_lockout(
		struct omap34xx_bridge_suspend_data *s, struct file *f)
{
	if ((s)->suspended) {
		if ((f)->f_flags & O_NONBLOCK)
			return DSP_EDPMSUSPEND;
		wait_event_interruptible((s)->suspend_wq, (s)->suspended == 0);
	}
	return 0;
}

#endif

#ifdef DEBUG
module_param(GT_str, charp, 0);
MODULE_PARM_DESC(GT_str, "GT string, default = NULL");

module_param(dsp_debug, int, 0);
MODULE_PARM_DESC(dsp_debug, "Wait after loading DSP image. default = false");
#endif

module_param(base_img, charp, 0);
MODULE_PARM_DESC(base_img, "DSP base image, default = NULL");

module_param(shm_size, int, 0);
MODULE_PARM_DESC(shm_size, "SHM size, default = 4 MB, minimum = 64 KB");

module_param(phys_mempool_base, uint, 0);
MODULE_PARM_DESC(phys_mempool_base,
		"Physical memory pool base passed to driver");

module_param(phys_mempool_size, uint, 0);
MODULE_PARM_DESC(phys_mempool_size,
		"Physical memory pool size passed to driver");
module_param(tc_wordswapon, int, 0);
MODULE_PARM_DESC(tc_wordswapon, "TC Word Swap Option. default = 0");

module_param(min_active_opp_freq, ulong, S_IRUSR | S_IWUSR);
MODULE_PARM_DESC(min_active_freq, "Min ACTIVE VDD1 OPPfreq, default = 260000");

MODULE_AUTHOR("Texas Instruments");
MODULE_LICENSE("GPL");

static char *driver_name = DRIVER_NAME;

#ifdef CONFIG_BRIDGE_DEBUG
static struct GT_Mask driverTrace;
#endif /* CONFIG_BRIDGE_DEBUG */

static struct file_operations bridge_fops = {
	.open		= bridge_open,
	.release	= bridge_release,
	.unlocked_ioctl	= bridge_ioctl,
	.mmap		= bridge_mmap,
};

#ifdef CONFIG_PM
static u32 timeOut = 1000;
#ifdef CONFIG_BRIDGE_DVFS
static struct clk *clk_handle;
#endif
#endif

struct dspbridge_platform_data *omap_dspbridge_pdata;

#ifdef CONFIG_BRIDGE_RECOVERY
static struct workqueue_struct *bridge_recovery_workq;
#endif

#ifdef CONFIG_BRIDGE_DVFS
static int dspbridge_post_scale(struct notifier_block *op, unsigned long val,
				void *ptr)
{
	struct dspbridge_platform_data *pdata =
		omap_dspbridge_dev->dev.platform_data;
	if (CPUFREQ_POSTCHANGE == val && pdata->dsp_get_opp)
		PWR_PM_PostScale(PRCM_VDD1, pdata->dsp_get_opp());
	return 0;
}

static struct notifier_block iva_clk_notifier = {
	.notifier_call = dspbridge_post_scale,
	NULL,
};
#endif

static int __devinit omap34xx_bridge_probe(struct platform_device *pdev)
{
	int status;
	u32 initStatus;
	u32 temp;
	dev_t   dev = 0 ;
	int     result;

	struct dspbridge_platform_data *pdata = pdev->dev.platform_data;

	omap_dspbridge_dev = pdev;

	/* use 2.6 device model */
	result = alloc_chrdev_region(&dev, driver_minor, 1, driver_name);

	if (result < 0) {
		GT_1trace(driverTrace, GT_7CLASS, "bridge_init: "
				"Can't get Major %d \n", driver_major);
		goto err1;
	}

	driver_major = MAJOR(dev);

	bridge_device = kzalloc(sizeof(struct bridge_dev), GFP_KERNEL);
	if (!bridge_device) {
		result = -ENOMEM;
		goto err2;
	}

	cdev_init(&bridge_device->cdev, &bridge_fops);
	bridge_device->cdev.owner = THIS_MODULE;

	status = cdev_add(&bridge_device->cdev, dev, 1);

	if (status) {
		GT_0trace(driverTrace, GT_7CLASS,
				"Failed to add the bridge device \n");
		goto err3;
	}

	/* udev support */
	bridge_class = class_create(THIS_MODULE, "ti_bridge");

	if (IS_ERR(bridge_class))
		GT_0trace(driverTrace, GT_7CLASS,
				"Error creating bridge class \n");

	device_create(bridge_class, NULL, MKDEV(driver_major, driver_minor),
			NULL, "DspBridge");

	bridge_create_sysfs();

	GT_init();
	GT_create(&driverTrace, "LD");

#ifdef DEBUG
	if (GT_str)
		GT_set(GT_str);
#elif defined(DDSP_DEBUG_PRODUCT) && GT_TRACE
	GT_set("**=67");
#endif

	GT_0trace(driverTrace, GT_ENTER, "-> driver_init\n");

#ifdef CONFIG_PM
	/* Initialize the wait queue */
	if (!status) {
		bridge_suspend_data.suspended = 0;
		init_waitqueue_head(&bridge_suspend_data.suspend_wq);
	}
#endif

	SERVICES_Init();

	/*  Autostart flag.  This should be set to true if the DSP image should
	 *  be loaded and run during bridge module initialization  */

	if (base_img) {
		temp = true;
		REG_SetValue(AUTOSTART, (u8 *)&temp, sizeof(temp));
		REG_SetValue(DEFEXEC, (u8 *)base_img, strlen(base_img) + 1);
	} else {
		temp = false;
		REG_SetValue(AUTOSTART, (u8 *)&temp, sizeof(temp));
		REG_SetValue(DEFEXEC, (u8 *) "\0", (u32)2);
	}

	if (shm_size >= 0x10000) {	/* 64 KB */
		initStatus = REG_SetValue(SHMSIZE, (u8 *)&shm_size,
				sizeof(shm_size));
	} else {
		initStatus = DSP_EINVALIDARG;
		status = -1;
		GT_0trace(driverTrace, GT_7CLASS,
			  "SHM size must be at least 64 KB\n");
	}
	GT_1trace(driverTrace, GT_7CLASS,
		 "requested shm_size = 0x%x\n", shm_size);

	if (pdata->phys_mempool_base && pdata->phys_mempool_size) {
		phys_mempool_base = pdata->phys_mempool_base;
		phys_mempool_size = pdata->phys_mempool_size;
	}

	GT_1trace(driverTrace, GT_7CLASS, "phys_mempool_base = 0x%x \n",
		 phys_mempool_base);

	GT_1trace(driverTrace, GT_7CLASS, "phys_mempool_size = 0x%x\n",
		 phys_mempool_base);

	if ((phys_mempool_base > 0x0) && (phys_mempool_size > 0x0))
		MEM_ExtPhysPoolInit(phys_mempool_base, phys_mempool_size);
	if (tc_wordswapon) {
		GT_0trace(driverTrace, GT_7CLASS, "TC Word Swap is enabled\n");
		REG_SetValue(TCWORDSWAP, (u8 *)&tc_wordswapon,
				sizeof(tc_wordswapon));
	} else {
		GT_0trace(driverTrace, GT_7CLASS, "TC Word Swap is disabled\n");
		REG_SetValue(TCWORDSWAP, (u8 *)&tc_wordswapon,
				sizeof(tc_wordswapon));
	}
	if (DSP_SUCCEEDED(initStatus)) {
#ifdef CONFIG_BRIDGE_DVFS
		if (pdata->mpu_get_rate_table)
			pdata->mpu_rate_table = (*pdata->mpu_get_rate_table)();
		else {
			GT_0trace(driverTrace, GT_7CLASS, "dspbridge failed to"
				"get mpu opp table\n");
			return -EFAULT;
			goto err3;
		}

		clk_handle = clk_get(NULL, "iva2_ck");
		if (!clk_handle) {
			GT_0trace(driverTrace, GT_7CLASS,
			"clk_get failed to get iva2_ck \n");
		} else {
			GT_0trace(driverTrace, GT_7CLASS,
			"clk_get PASS to get iva2_ck \n");
		}
		if (!cpufreq_register_notifier(&iva_clk_notifier,
					       CPUFREQ_TRANSITION_NOTIFIER)) {
			GT_0trace(driverTrace, GT_7CLASS,
			"clk_notifier_register PASS for iva2_ck \n");
		} else {
			GT_0trace(driverTrace, GT_7CLASS,
			"clk_notifier_register FAIL for iva2_ck \n");
		}
#endif
		driverContext = DSP_Init(&initStatus);
		if (DSP_FAILED(initStatus)) {
			status = -1;
			GT_0trace(driverTrace, GT_7CLASS,
				 "DSP/BIOS Bridge initialization Failed\n");
		} else {
			GT_0trace(driverTrace, GT_5CLASS,
					"DSP/BIOS Bridge driver loaded\n");
		}
	}

	DBC_Assert(status == 0);
	DBC_Assert(DSP_SUCCEEDED(initStatus));
	GT_0trace(driverTrace, GT_ENTER, " <- driver_init\n");

#ifdef CONFIG_BRIDGE_RECOVERY
	bridge_recovery_workq = create_singlethread_workqueue("bridge_recovery");
#endif

	return 0;

err3:
	kfree(bridge_device);

err2:
	unregister_chrdev_region(dev, 1);

err1:
	return result;
}

static int __devexit omap34xx_bridge_remove(struct platform_device *pdev)
{
	dev_t devno;
	bool ret;
	DSP_STATUS dsp_status = DSP_SOK;
	HANDLE	     hDrvObject = NULL;

	GT_0trace(driverTrace, GT_ENTER, "-> driver_exit\n");

	dsp_status = CFG_GetObject((u32 *)&hDrvObject, REG_DRV_OBJECT);
	if (DSP_FAILED(dsp_status))
		goto func_cont;

#ifdef CONFIG_BRIDGE_DVFS
	if (!cpufreq_unregister_notifier(&iva_clk_notifier,
					 CPUFREQ_TRANSITION_NOTIFIER)) {

		GT_0trace(driverTrace, GT_7CLASS,
		"clk_notifier_unregister PASS for iva2_ck \n");
	} else {
		GT_0trace(driverTrace, GT_7CLASS,
		"clk_notifier_unregister FAILED for iva2_ck \n");
	}
#endif /* #ifdef CONFIG_BRIDGE_DVFS */

	if (driverContext) {
		/* Put the DSP in reset state */
		ret = DSP_Deinit(driverContext);
		driverContext = 0;
		DBC_Assert(ret == true);
	}

#ifdef CONFIG_BRIDGE_DVFS
	clk_put(clk_handle);
	clk_handle = NULL;
#endif /* #ifdef CONFIG_BRIDGE_DVFS */

func_cont:
	MEM_ExtPhysPoolRelease();

	SERVICES_Exit();
	GT_exit();

	/* Remove driver sysfs entries */
	bridge_destroy_sysfs();

	devno = MKDEV(driver_major, driver_minor);
	if (bridge_device) {
		cdev_del(&bridge_device->cdev);
		kfree(bridge_device);
	}
	unregister_chrdev_region(devno, 1);
	if (bridge_class) {
		/* remove the device from sysfs */
		device_destroy(bridge_class, MKDEV(driver_major, driver_minor));
		class_destroy(bridge_class);

	}
	return 0;
}


#ifdef CONFIG_PM
static int bridge_suspend(struct platform_device *pdev, pm_message_t state)
{
	u32 status;
	u32 command = PWR_EMERGENCYDEEPSLEEP;

	status = PWR_SleepDSP(command, timeOut);
	if (DSP_FAILED(status))
		return -1;

	bridge_suspend_data.suspended = 1;
	return 0;
}

static int bridge_resume(struct platform_device *pdev)
{
	u32 status = 0;
	struct WMD_DEV_CONTEXT *dwContext;

	DEV_GetWMDContext(DEV_GetFirst(), &dwContext);
	if (!dwContext)
		return -EFAULT;

	/*
	 * only wake up the DSP if it was not in Hibernation before the
	 * suspend transition
	 */
	if (dwContext->dwBrdState != BRD_DSP_HIBERNATION)
		status = PWR_WakeDSP(timeOut);

	if (DSP_FAILED(status))
		return status;

	bridge_suspend_data.suspended = 0;
	wake_up(&bridge_suspend_data.suspend_wq);
	return 0;
}
#else
#define bridge_suspend NULL
#define bridge_resume NULL
#endif

static struct platform_driver bridge_driver = {
	.driver = {
		.name = BRIDGE_NAME,
	},
	.probe	 = omap34xx_bridge_probe,
	.remove	 = __devexit_p(omap34xx_bridge_remove),
	.suspend = bridge_suspend,
	.resume	 = bridge_resume,
};

static int __init bridge_init(void)
{
	return platform_driver_register(&bridge_driver);
}

static void __exit bridge_exit(void)
{
	platform_driver_unregister(&bridge_driver);
}

#ifdef CONFIG_BRIDGE_RECOVERY
static unsigned int event_mask = DSP_SYSERROR | DSP_MMUFAULT | DSP_PWRERROR;
module_param(event_mask, uint, 0644);
static char *firmware_file = "/system/lib/dsp/baseimage.dof";
module_param(firmware_file, charp, 0644);
int bridge_force_recovery(const char *val, struct kernel_param *kp);
module_param_call(recovery, bridge_force_recovery, NULL, NULL, 0600);
static LIST_HEAD(bridge_pctxt_list);
static DEFINE_MUTEX(bridge_pctxt_list_lock);

void bridge_prcm_registers_dump(void)
{

#define OMAP_GRP_SEL_BASE 0x48307000
#define OMAP_MBX_BASE 0x48094000

	u32 addr;
	u32 prm_base = 0;
	u32 cm_base = 0;
	u32 grp_base = 0;
	u32 mbx_base = 0;
	u32 regVal ;

	prm_base = OMAP_IVA2_PRM_BASE;
	cm_base = OMAP_IVA2_CM_BASE;
	grp_base = OMAP_GRP_SEL_BASE;
	mbx_base = OMAP_MBX_BASE;

	regVal = omap_readl(cm_base + 0x00);
	printk(KERN_INFO "CM_FCLKEN_IVA2(0x%x) = 0x%x \n",
				  (OMAP_IVA2_CM_BASE + 0x00), regVal);
	regVal = omap_readl(cm_base + 0x04);
	printk(KERN_INFO "CM_CLKEN_PLL_IVA2(0x%x) = 0x%x \n",
				 (OMAP_IVA2_CM_BASE + 0x04), regVal);
	regVal = omap_readl(cm_base + 0x10);
	printk(KERN_INFO "CM_ICLKEN1_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_CM_BASE + 0x10), regVal);
	regVal = omap_readl(cm_base + 0x20);
	printk(KERN_INFO "CM_IDLEST_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_CM_BASE + 0x20), regVal);
	regVal = omap_readl(cm_base + 0x24);
	printk(KERN_INFO "CM_IDLEST_PLL_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_CM_BASE + 0x24), regVal);
	regVal = omap_readl(cm_base + 0x34);
	printk(KERN_INFO "CM_AUTOIDLE_PLL_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_CM_BASE + 0x34), regVal);
	regVal = omap_readl(cm_base + 0x40);
	printk(KERN_INFO "CM_CLKSEL1_PLL_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_CM_BASE + 0x40), regVal);
	regVal = omap_readl(cm_base + 0x48);
	printk(KERN_INFO "CM_CLKSTCTRL_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_CM_BASE + 0x48), regVal);
	regVal = omap_readl(cm_base + 0x4c);
	printk(KERN_INFO "CM_CLKSTST_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_CM_BASE + 0x4c), regVal);
	regVal = omap_readl(prm_base + 0x50);
	printk(KERN_INFO "RM_RSTCTRL_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_PRM_BASE + 0x50), regVal);
	regVal = omap_readl(prm_base + 0x58);
	printk(KERN_INFO "RM_RSTST_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_PRM_BASE + 0x58), regVal);
	regVal = omap_readl(prm_base + 0xe0);
	printk(KERN_INFO "PM_PWSTCTRL_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_PRM_BASE + 0xe0), regVal);
	regVal = omap_readl(prm_base + 0xe4);
	printk(KERN_INFO "PM_PWSTST_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_PRM_BASE + 0xe4), regVal);
	regVal = omap_readl(cm_base + 0xA10);
	printk(KERN_INFO "CM_ICLKEN1_CORE(0x%x) = 0x%x \n",
				(OMAP_IVA2_PRM_BASE + 0xa10), regVal);
	regVal = omap_readl(grp_base + 0xA4);
	printk(KERN_INFO "PM_MPUGRPSEL_PER(0x%x) = 0x%x \n",
				(OMAP_GRP_SEL_BASE + 0xa4), regVal);
	regVal = omap_readl(grp_base + 0xA8);
	printk(KERN_INFO "PM_IVA2GRPSEL_PER(0x%x) = 0x%x \n",
				(OMAP_GRP_SEL_BASE + 0xa8), regVal);
	regVal = omap_readl(prm_base + 0xf8);
	printk(KERN_INFO "PRM_IRQSTATUS_IVA2(0x%x) = 0x%x \n",
				(OMAP_IVA2_PRM_BASE + 0xf8), regVal);
	printk(KERN_INFO "\n\n******* MBX DUMP ********************* \n");
	regVal = omap_readl(mbx_base + 0x10);
	printk(KERN_INFO "MAILBOX_SYSCONFIG(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0x10), regVal);
	regVal = omap_readl(mbx_base + 0x80);
	printk(KERN_INFO "MAILBOX_FIFOSTATUS_MPU(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0x80), regVal);
	regVal = omap_readl(mbx_base + 0x84);
	printk(KERN_INFO "MAILBOX_FIFOSTATUS_DSP(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0x84), regVal);
	regVal = omap_readl(mbx_base + 0xC0);
	printk(KERN_INFO "MAILBOX_MSGSTATUS_MPU(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0xC0), regVal);
	regVal = omap_readl(mbx_base + 0xC4);
	printk(KERN_INFO "MAILBOX_MSGSTATUS_DSP(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0xC4), regVal);
	regVal = omap_readl(mbx_base + 0x100);
	printk(KERN_INFO "MAILBOX_IRQSTATUS_MPU(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0x100), regVal);
	regVal = omap_readl(mbx_base + 0x108);
	printk(KERN_INFO "MAILBOX_IRQSTATUS_DSP(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0x108), regVal);
	regVal = omap_readl(mbx_base + 0x104);
	printk(KERN_INFO "MAILBOX_IRQENABLE_MPU(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0x104), regVal);
	regVal = omap_readl(mbx_base + 0x10C);
	printk(KERN_INFO "MAILBOX_IRQENABLE_DSP(0x%x) = 0x%x \n",
				  (OMAP_MBX_BASE + 0x10C), regVal);

}

static void bridge_load_firmware(void)
{
	DSP_HPROCESSOR hProcessor = NULL;
	struct PROCESS_CONTEXT pctxt = {0};
	DSP_STATUS status;
	const char* argv[2];
	argv[0] = firmware_file;
	argv[1] = NULL;

	printk(KERN_INFO "%s: loading bridge firmware from %s\n", __func__,
	       firmware_file);

	mutex_init(&pctxt.node_lock);
	mutex_init(&pctxt.dmm_lock);
	mutex_init(&pctxt.strm_lock);

	DRV_ProcUpdatestate(&pctxt, PROC_RES_ALLOCATED);

	status = PROC_Attach(0, NULL, &hProcessor, &pctxt);
	if (DSP_FAILED(status))
		printk(KERN_ERR "%s: error attaching to processor\n", __func__);

	status = PROC_Stop(hProcessor);
	if (DSP_FAILED(status))
		printk(KERN_ERR "%s: error stopping processor\n", __func__);

	status = PROC_Load(hProcessor, 1, argv, NULL);
	if (DSP_FAILED(status))
		printk(KERN_ERR "%s: error reloading firmware\n", __func__);

	status = PROC_Start(hProcessor);
	if (DSP_FAILED(status))
		printk(KERN_ERR "%s: error starting processor\n", __func__);

	DRV_RemoveAllResources(&pctxt);
	PROC_Detach(&pctxt);

	bridge_prcm_registers_dump() ;
}

static bool bridge_all_closed(void)
{
	int ret;

	mutex_lock(&bridge_pctxt_list_lock);
	ret = list_empty(&bridge_pctxt_list);
	mutex_unlock(&bridge_pctxt_list_lock);
	return ret;
}

static void bridge_kill_all_users(struct work_struct *work)
{
	struct PROCESS_CONTEXT *pctxt;

	printk(KERN_INFO "dspbridge: delivering sigkill to all bridge users\n");
	mutex_lock(&bridge_pctxt_list_lock);
	list_for_each_entry(pctxt, &bridge_pctxt_list, list)
		force_sig(SIGKILL, pctxt->task);
	mutex_unlock(&bridge_pctxt_list_lock);

	printk(KERN_INFO "dspbridge: waiting for bridge users to exit\n");
	while (!bridge_all_closed())
		cpu_relax();
	printk(KERN_INFO "dspbridge: all bridge users exited\n");

	/*clear_loadref();*/

	/*
	reset the mbbox
	*/
	{
		DSP_STATUS status = DSP_SOK;
		struct CFG_HOSTRES hostRes;

		status = CFG_GetHostResources(
		(struct CFG_DEVNODE *)DRV_GetFirstDevExtension() , &hostRes);
		if (DSP_SUCCEEDED(status)) {
			printk(KERN_INFO "reset mbox\n") ;
			HW_MBOX_initSettings(hostRes.dwMboxBase);
		}
	}

	bridge_load_firmware();

	recover = false;
	complete_all(&bridge_open_comp);
}

DECLARE_WORK(bridge_recovery_work, bridge_kill_all_users);

void bridge_recovery_notify(u32 event)
{
	if (!(event & event_mask))
		return;
	printk(KERN_INFO "dspbridge fatal error occured, attempting to "
	       "recover\n");

	INIT_COMPLETION(bridge_open_comp);
	recover = true;

	queue_work(bridge_recovery_workq, &bridge_recovery_work);
}

int bridge_force_recovery(const char *val, struct kernel_param *kp)
{
	printk(KERN_INFO "dspbridge forcing recovery from userspace.\n");
	bridge_recovery_notify(event_mask);
	return 0;
}

static void bridge_recovery_add(struct PROCESS_CONTEXT *pctxt)
{
	get_task_struct(current->group_leader);
	pctxt->task = current->group_leader;

	mutex_lock(&bridge_pctxt_list_lock);
	list_add(&pctxt->list, &bridge_pctxt_list);
	mutex_unlock(&bridge_pctxt_list_lock);
}

static void bridge_recovery_remove(struct PROCESS_CONTEXT *pctxt)
{
	mutex_lock(&bridge_pctxt_list_lock);
	list_del(&pctxt->list);
	mutex_unlock(&bridge_pctxt_list_lock);
	if (pctxt->task)
		put_task_struct(pctxt->task);
	pctxt->task = NULL;
}
#endif

/* This function is called when an application opens handle to the
 * bridge driver. */
static int bridge_open(struct inode *ip, struct file *filp)
{
	int status = 0;
#ifndef RES_CLEANUP_DISABLE
	struct PROCESS_CONTEXT *pPctxt = NULL;

	GT_0trace(driverTrace, GT_ENTER, "-> driver_open\n");

#ifdef CONFIG_BRIDGE_RECOVERY
	if (recover)
		wait_for_completion(&bridge_open_comp);
#endif

	pPctxt = MEM_Calloc(sizeof(struct PROCESS_CONTEXT), MEM_PAGED);

	if (pPctxt != NULL) {
		mutex_init(&pPctxt->node_lock);
		mutex_init(&pPctxt->dmm_lock);
		mutex_init(&pPctxt->strm_lock);
		DRV_ProcUpdatestate(pPctxt, PROC_RES_ALLOCATED);
		filp->private_data = pPctxt;
	}

#ifdef CONFIG_BRIDGE_RECOVERY
	bridge_recovery_add(pPctxt);
#endif
#endif

	GT_0trace(driverTrace, GT_ENTER, " <- driver_open\n");
	return status;
}

/* This function is called when an application closes handle to the bridge
 * driver. */
static int bridge_release(struct inode *ip, struct file *filp)
{
	int status;
	HANDLE hDrvObject = NULL;
	struct PROCESS_CONTEXT *pr_ctxt;

	GT_0trace(driverTrace, GT_ENTER, "-> driver_release\n");

	status = CFG_GetObject((u32 *)&hDrvObject, REG_DRV_OBJECT);

	/* Checking weather task structure for all process existing
	 * in the process context list If not removing those recovery*/
	if (DSP_FAILED(status))
		goto func_end;

	pr_ctxt = filp->private_data;

	if (pr_ctxt) {
		flush_signals(current);
		DRV_RemoveAllResources(pr_ctxt);
		if (pr_ctxt->hProcessor)
			PROC_Detach(pr_ctxt);
#ifdef CONFIG_BRIDGE_RECOVERY
		bridge_recovery_remove(pr_ctxt);
#endif
		MEM_Free(pr_ctxt);
		filp->private_data = NULL;
	}
func_end:
	(status == true) ? (status = 0) : (status = -1);

	GT_0trace(driverTrace, GT_ENTER, " <- driver_release\n");

	return status;
}

/* This function provides IO interface to the bridge driver. */
static long bridge_ioctl(struct file *filp, unsigned int code,
		unsigned long args)
{
	int status;
	u32 retval = DSP_SOK;
	union Trapped_Args pBufIn;

	DBC_Require(filp != NULL);
#ifdef CONFIG_BRIDGE_RECOVERY
	if (recover) {
		status = -EBUSY;
		goto err;
	}
#endif

#ifdef CONFIG_PM
	status = omap34xxbridge_suspend_lockout(&bridge_suspend_data, filp);
	if (status != 0)
		return status;
#endif

	GT_0trace(driverTrace, GT_ENTER, " -> driver_ioctl\n");

	/* Deduct one for the CMD_BASE. */
	code = (code - CMD_BASE);

	status = copy_from_user(&pBufIn, (union Trapped_Args *)args,
				sizeof(union Trapped_Args));


	if (status >= 0) {
		status = WCD_CallDevIOCtl(code, &pBufIn, &retval,
				filp->private_data);

		if (DSP_SUCCEEDED(status)) {
			status = retval;
		} else {
			GT_1trace(driverTrace, GT_7CLASS,
				 "IOCTL Failed, code : 0x%x\n", code);
			status = -1;
		}

	}

	GT_0trace(driverTrace, GT_ENTER, " <- driver_ioctl\n");

err:
	return status;
}
/*
#ifdef CONFIG_BRIDGE_DVFS
	int i = 0;
#endif
*/
/* This function maps kernel space memory to user space memory. */
static int bridge_mmap(struct file *filp, struct vm_area_struct *vma)
{
#if GT_TRACE
	u32 offset = vma->vm_pgoff << PAGE_SHIFT;
#endif
	u32 status;

	DBC_Assert(vma->vm_start < vma->vm_end);

	vma->vm_flags |= VM_RESERVED | VM_IO;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	GT_6trace(driverTrace, GT_3CLASS,
		 "vm filp %p offset %lx start %lx end %lx"
		 " page_prot %lx flags %lx\n", filp, offset, vma->vm_start,
		 vma->vm_end, vma->vm_page_prot, vma->vm_flags);

	status = remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
				vma->vm_end - vma->vm_start, vma->vm_page_prot);
	if (status != 0)
		status = -EAGAIN;

	return status;
}

#ifndef RES_CLEANUP_DISABLE
/* To remove all process resources before removing the process from the
 * process context list*/
DSP_STATUS DRV_RemoveAllResources(HANDLE hPCtxt)
{
	DSP_STATUS status = DSP_SOK;
	struct PROCESS_CONTEXT *pCtxt = (struct PROCESS_CONTEXT *)hPCtxt;
	if (pCtxt != NULL) {
		mutex_lock(&pCtxt->strm_lock);
		DRV_RemoveAllSTRMResElements(pCtxt);
		mutex_unlock(&pCtxt->strm_lock);

		mutex_lock(&pCtxt->node_lock);
		DRV_RemoveAllNodeResElements(pCtxt);
		mutex_unlock(&pCtxt->node_lock);

		mutex_lock(&pCtxt->dmm_lock);
		DRV_RemoveAllDMMResElements(pCtxt);
		mutex_unlock(&pCtxt->dmm_lock);

		DRV_ProcUpdatestate(pCtxt, PROC_RES_FREED);
	}
	return status;
}
#endif

/*
 * sysfs
 */
static ssize_t drv_state_show(struct kobject *kobj, struct kobj_attribute *attr,
                        char *buf)
{
	struct WMD_DEV_CONTEXT *dwContext;
	struct DEV_OBJECT *hDevObject = NULL;
	int drv_state = 0;

	for (hDevObject = (struct DEV_OBJECT *)DRV_GetFirstDevObject();
		hDevObject != NULL;
		hDevObject = (struct DEV_OBJECT *)DRV_GetNextDevObject
							((u32)hDevObject)) {
		if (DSP_FAILED(DEV_GetWMDContext(hDevObject,
		   (struct WMD_DEV_CONTEXT **)&dwContext))) {
			continue;
		}
		drv_state = dwContext->dwBrdState;
	}

        return sprintf(buf, "%d\n", drv_state);
}

static struct kobj_attribute drv_state_attr = __ATTR_RO(drv_state);

static struct attribute *attrs[] = {
        &drv_state_attr.attr,
        NULL,
};

static struct attribute_group attr_group = {
        .attrs = attrs,
};

static void bridge_create_sysfs(void)
{
	int error;

	error = sysfs_create_group(&omap_dspbridge_dev->dev.kobj, &attr_group);

	if (error)
		kobject_put(&omap_dspbridge_dev->dev.kobj);
}

static void bridge_destroy_sysfs(void)
{
	sysfs_remove_group(&omap_dspbridge_dev->dev.kobj, &attr_group);
}

/* Bridge driver initialization and de-initialization functions */
module_init(bridge_init);
module_exit(bridge_exit);

