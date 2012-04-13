#ifndef __DSSCOMP_ISPRESIZER_H
#define __DSSCOMP_ISPRESIZER_H

#include <linux/kernel.h>
#include <linux/sched.h>
#include <mach/isp_user.h>
#include <linux/ispdss.h>
#include "../../../media/video/isp/ispresizer.h"
#include <linux/omap_resizer.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>

#define MAX_VIDEO_BUFFERS       6
#define VID_MAX_WIDTH           1280    /* Largest width */
#define VID_MAX_HEIGHT          720     /* Largest height */

int ispresizer_init(struct dss2_ovl_info *oi);
int ispresizer_begin(struct dss2_ovl_info *oi);
int is_isprsz_enabled(void);
int create_isprsz_sysfs(struct platform_device *pdev);

#endif
