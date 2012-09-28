/**********************************************************************
 *
 * Copyright(c) 2008 Imagination Technologies Ltd. All rights reserved.
 * 
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope it will be useful but, except 
 * as otherwise stated in writing, without any warranty; without even the 
 * implied warranty of merchantability or fitness for a particular purpose. 
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * 
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK 
 *
 ******************************************************************************/

#include <linux/io.h>
#include <linux/uaccess.h>

#include <linux/version.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/platform_device.h>

#include <plat/display.h>

#include "img_defs.h"
#include "servicesext.h"
#include "kerneldisplay.h"
#include "omaplfb.h"
#include "pvrmodule.h"

MODULE_SUPPORTED_DEVICE(DEVNAME);

#define unref__ __attribute__ ((unused))

void *OMAPLFBAllocKernelMem(unsigned long ulSize)
{
	return kmalloc(ulSize, GFP_KERNEL);
}

void OMAPLFBFreeKernelMem(void *pvMem)
{
	kfree(pvMem);
}


OMAP_ERROR OMAPLFBGetLibFuncAddr (char *szFunctionName, PFN_DC_GET_PVRJTABLE *ppfnFuncTable)
{
	if(strcmp("PVRGetDisplayClassJTable", szFunctionName) != 0)
	{
		return (OMAP_ERROR_INVALID_PARAMS);
	}

	
	*ppfnFuncTable = PVRGetDisplayClassJTable;

	return (OMAP_OK);
}

static struct omap_overlay_manager* lcd_mgr = 0;
static struct omap_overlay*         omap_gfxoverlay = 0;
static struct omap_overlay_info     gfxoverlayinfo;
struct fb_info *fb_info;

void OMAPLFBDisplayInit(void)
{
    struct omap_overlay*         overlayptr = 0;
    unsigned int                 i = 0;
    unsigned int                 numoverlays = 0;

    // there is tv and lcd managers... we only care about lcd at this time.
    lcd_mgr = omap_dss_get_overlay_manager(OMAP_DSS_OVL_MGR_LCD);
    if(!lcd_mgr)
    {
        DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBSetDisplayInfo couldn't find lcd overlay manager\n"));
        return;
    }

    numoverlays = omap_dss_get_num_overlays();
    if( numoverlays == 0)
    {
        DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBSetDisplayInfo couldn't find any overlays\n"));
        return;
    }

    for( i = 0; i < numoverlays; i++ )
    {
        overlayptr = omap_dss_get_overlay(i);
        if( strncmp( overlayptr->name, "gfx", 3 ) == 0)
        {
            DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBSetDisplayInfo found GFX overlay\n"));
            omap_gfxoverlay = overlayptr;
            break;
        }
    }
    if( omap_gfxoverlay == 0 )
    {
        DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBSetDisplayInfo GFX overlay no found\n"));
        return;
    }

    omap_gfxoverlay->get_overlay_info( omap_gfxoverlay, &gfxoverlayinfo );
    fb_info =  registered_fb[0];

}

/*#define SGX_FB_DEBUG_FLIP_TIMING 1 */
#ifdef SGX_FB_DEBUG_FLIP_TIMING
static ktime_t start;
static ktime_t end_getovl;
static ktime_t end_setovl;
static ktime_t end_apply;
static ktime_t end_sync;
static ktime_t end_update;
static ktime_t start_sync;
static ktime_t end_sync;

static long int getovltimediff;
static long int setovltimediff;
static long int applytimediff;
static long int synctimediff;
static long int updatetimediff;
static int count;
static int countSync;
#endif

void OMAPLFBFlip(OMAPLFB_SWAPCHAIN *psSwapChain, unsigned long paddr)
{
	u32 pixels;

	if (lcd_mgr && lcd_mgr->device && omap_gfxoverlay) {

#ifdef SGX_FB_DEBUG_FLIP_TIMING
		start = ktime_get();
#endif
		omap_gfxoverlay->get_overlay_info(omap_gfxoverlay,
						  &gfxoverlayinfo);

#ifdef SGX_FB_DEBUG_FLIP_TIMING
		end_getovl = ktime_get();
#endif
		gfxoverlayinfo.paddr = paddr;
		/* TODO: plumb vaddr in to this function */
		gfxoverlayinfo.vaddr = (void *)(paddr -
		0x81314000 + 0xd2800000);

		omap_gfxoverlay->set_overlay_info(omap_gfxoverlay,
						  &gfxoverlayinfo);

#ifdef SGX_FB_DEBUG_FLIP_TIMING
		end_setovl = ktime_get();
#endif
		lcd_mgr->apply(lcd_mgr);

#ifdef SGX_FB_DEBUG_FLIP_TIMING
		end_apply = ktime_get();
#endif
		/* TODO: make sync a function that always exists, then
		 * we can remove this specialty check in this code,
		 * (i.e. a stub function for dumb displays like HDMI) */
		if (lcd_mgr->device->sync)
			lcd_mgr->device->sync(lcd_mgr->device);

#ifdef SGX_FB_DEBUG_FLIP_TIMING
		end_sync = ktime_get();
#endif
		/* TODO: make update a function that always exists, then
		 * we can remove this specialty check in this code,
		 * (i.e. a stub function for dumb displays like HDMI) */
		if (lcd_mgr->device->update) {
			lcd_mgr->device->update(lcd_mgr->device, 0, 0,
						gfxoverlayinfo.width,
						gfxoverlayinfo.height);
		}

#ifdef SGX_FB_DEBUG_FLIP_TIMING
		end_update = ktime_get();
#endif
		pixels = (paddr - fb_info->fix.smem_start) /
			 (fb_info->var.bits_per_pixel / 8);
		fb_info->var.yoffset = pixels / fb_info->var.xres;
		fb_info->var.xoffset = pixels % fb_info->var.xres;
	}

#ifdef SGX_FB_DEBUG_FLIP_TIMING
	getovltimediff += ktime_to_ns(ktime_sub(end_getovl, start));
	setovltimediff += ktime_to_ns(ktime_sub(end_setovl, end_getovl));
	applytimediff += ktime_to_ns(ktime_sub(end_apply, end_setovl));
	synctimediff += ktime_to_ns(ktime_sub(end_sync, end_apply));
	updatetimediff += ktime_to_ns(ktime_sub(end_update, end_sync));

	if (++count >= 60) {
		printk(KERN_INFO DRIVER_PREFIX "--Avg times (millisecs)\n" \
		       "    get: %ld, set: %ld, apply: %ld, "\
		       "sync: %ld, update: %ld\n",
		       ((getovltimediff / 1000000) / count),
		       ((setovltimediff / 1000000) / count),
		       ((applytimediff / 1000000) / count),
		       ((synctimediff / 1000000) / count),
		       ((updatetimediff / 1000000) / count));
		count = 0;
		getovltimediff = 0;
		setovltimediff = 0;
		applytimediff = 0;
		synctimediff = 0;
		updatetimediff = 0;
	}

#endif

}

static OMAP_BOOL bDeviceSuspended;

static void OMAPLFBCommonSuspend(void)
{
	if (bDeviceSuspended)
	{
		return;
	}

	OMAPLFBDriverSuspend();

	bDeviceSuspended = OMAP_TRUE;
}

static int OMAPLFBDriverProbe_Entry(struct platform_device *pdev)
{
	dev_dbg(&pdev->dev, "probe\n");
	if(OMAPLFBInit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: OMAPLFBInit failed\n");
		return -ENODEV;
	}

	return 0;
}

static int OMAPLFBDriverSuspend_Entry(struct platform_device unref__ *pDevice, pm_message_t unref__ state)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverSuspend_Entry\n"));

	OMAPLFBCommonSuspend();

	return 0;
}

static int OMAPLFBDriverResume_Entry(struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverResume_Entry\n"));

	OMAPLFBDriverResume();

	bDeviceSuspended = OMAP_FALSE;

	return 0;
}

static IMG_VOID OMAPLFBDriverShutdown_Entry(struct platform_device unref__ *pDevice)
{
	DEBUG_PRINTK((KERN_INFO DRIVER_PREFIX ": OMAPLFBDriverShutdown_Entry\n"));

	OMAPLFBCommonSuspend();
}

static int __exit OMAPLFBDriverRemove_Entry(struct platform_device *pdev)
{
	if(OMAPLFBDeinit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: OMAPLFBDeinit failed\n");
	}
	return 0;
}

static struct platform_driver omaplfb_driver = {
	.driver = {
		.name		= DRVNAME,
	},
	.probe		= OMAPLFBDriverProbe_Entry,
 	.suspend	= OMAPLFBDriverSuspend_Entry,
	.resume		= OMAPLFBDriverResume_Entry,
	.shutdown	= OMAPLFBDriverShutdown_Entry,
	.remove		= __exit_p(OMAPLFBDriverRemove_Entry),

};

static int __init OMAPLFB_Init(void)
{
	int error;

	if ((error = platform_driver_register(&omaplfb_driver)) != 0)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Init: Unable to register platform driver (%d)\n", error);

		return -ENODEV;
	}

	return 0;
}

static IMG_VOID __exit OMAPLFB_Cleanup(IMG_VOID)
{    
	platform_driver_unregister(&omaplfb_driver);

	if(OMAPLFBDeinit() != OMAP_OK)
	{
		printk(KERN_WARNING DRIVER_PREFIX ": OMAPLFB_Cleanup: OMAPLFBDeinit failed\n");
	}
}

late_initcall(OMAPLFB_Init);
module_exit(OMAPLFB_Cleanup);
