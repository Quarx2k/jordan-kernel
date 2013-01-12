#ifndef __ARCH_ARM_PLAT_OMAP_MAPPHONE_DSI_PANEL_H
#define __ARCH_ARM_PLAT_OMAP_MAPPHONE_DSI_PANEL_H

#include "omapdss.h"

#define MAPPHONE_DSI_MAX_NAME_SIZE	32
#define MAPPHONE_DSI_MAX_PWR_SUPPLY	5

struct mapphone_dsi_panel_pwr_supply {
	char name[MAPPHONE_DSI_MAX_NAME_SIZE];
	struct regulator *reg_handle;
	int en_gpio;
	u8 en_gpio_value;
};

struct mapphone_lvds_panel_timing {
	u16 hbpr;
	u16 hpw;
	u16 hfpr;
	u16 vbpr;
	u16 vspr;
	u16 vfpr;
};

struct mapphone_dsi_panel_data {
	int num_pwr_supply;

	struct mapphone_dsi_panel_pwr_supply
		disp_vol_supply[MAPPHONE_DSI_MAX_PWR_SUPPLY];
	const char *name;

	int reset_gpio;
	int rst_delay_after_pwr;
	int rst_delay_after_high;

	bool use_ext_te;
	int ext_te_gpio;

	bool use_esd_check;

	atomic_t state;
	void *panel_handle;

	bool te_support;
	bool manual_te_trigger;
	u32 te_scan_line;
	enum omap_dsi_te_type te_type;
	bool cmoste_wr;	/* CMOS external TE SW workaround */

	struct {
		bool som; /* Secret Off Mode */
	} ftr_support;

	struct mapphone_lvds_panel_timing lvds_panel_timing;

	int max_backlight_level;
	int (*set_backlight)(struct omap_dss_device *dssdev, int level);
	int (*get_backlight)(struct omap_dss_device *dssdev);
};

/* for panel detection */
enum mapphone_panel_init_state {
	MAPPHONE_PANEL_UNDETERMINE      = 0,
	MAPPHONE_PANEL_NOT_PRESENT,
	MAPPHONE_PANEL_PRESENT,
	MAPPHONE_PANEL_INIT_DONE,
	MAPPHONE_PANEL_INIT_FAILED,
};
enum mapphone_panel_init_state get_panel_state(void);

#endif /* __ARCH_ARM_PLAT_OMAP_MAPPHONE_DSI_PANEL_H */
