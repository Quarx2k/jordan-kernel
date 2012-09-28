
#include <plat/display.h>

#ifndef __PANEL_H__
#define __PANEL_H__

#define OMAP_PANEL_MAX_NAME_SIZE	(32)

struct omap_panel_device {
	char name[OMAP_PANEL_MAX_NAME_SIZE + 1];

	void (*fod_disable)(struct omap_dss_device *);

	struct omap_dss_device *dssdev;
};

extern void *omap_panel_register(struct omap_panel_device *dev);
extern void  omap_panel_unregister(void *handle);

/* For Freeze-On-Disable */
extern int   omap_panel_fod_dss_state(void *handle, int en);
extern int   omap_panel_fod_panel_state(void *handle, int en);
extern int   omap_panel_fod_enabled(void *handle);

#endif /* __PANEL_H__ */
