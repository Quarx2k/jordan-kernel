/*
 * linux/include/asm-arm/arch-omap/board-mapphone.h
 *
 * Hardware definitions for OMAP3430-based Motorola reference design.
 *
 * Copyright (C) 2009 Motorola, Inc.
 *
 * Derived from include/asm-arm/arch-omap/board-3430sdp.h
 * Initial creation by Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * THIS SOFTWARE IS PROVIDED ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#ifndef __ASM_ARCH_OMAP_MAPPHONE_H
#define __ASM_ARCH_OMAP_MAPPHONE_H

#include <linux/init.h>
#include "board-mapphone-padconf.h"

extern void __init mapphone_usb_init(void);
extern void __init mapphone_flash_init(void);
extern void __init mapphone_panel_init(void);
extern void __init mapphone_sensors_init(void);
extern void __init mapphone_spi_init(void);
extern void __init mapphone_flash_init(void);
extern void __init mapphone_padconf_init(void);
extern void __init mapphone_hsmmc_init(void);
extern void __init mapphone_gpio_mapping_init(void);
extern void __init mapphone_camera_init(void);
extern void __init mapphone_mmcprobe_init(void);
extern void __init mapphone_cpcap_client_init(void);
extern int __init mapphone_mdm_ctrl_init(void);
extern void __init mapphone_gadget_init(void);
extern void __init mapphone_ehci_init(void);
extern int mapphone_bp_get_type(void);

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
extern struct mt9p012_platform_data mapphone_mt9p012_platform_data;
#endif
#if defined(CONFIG_VIDEO_OV8810)
extern struct ov8810_platform_data mapphone_ov8810_platform_data;
#endif
#if defined(CONFIG_VIDEO_OV5650)
extern struct ov5650_platform_data mapphone_ov5650_platform_data;
#endif
#ifdef CONFIG_VIDEO_OMAP3_HPLENS
extern struct hplens_platform_data mapphone_hplens_platform_data;
#endif

#define GPIO_MT9P012_STANDBY		58
#define GPIO_MT9P012_RESET		98
#define GPIO_SLIDER			177
#if defined(CONFIG_VIDEO_OV8810)
#define GPIO_OV8810_RESET	98
#define GPIO_OV8810_STANDBY	64
#define GPIO_FLASH_READY	36
#endif
#if defined(CONFIG_VIDEO_OV5650)
#define GPIO_OV5650_RESET	98
#define GPIO_OV5650_STANDBY	64
#define GPIO_FLASH_READY	36
#endif

#define OMAP_MCAM_SRC_CLK		    864000000
#define OMAP_MCAM_SRC_DIV		    4
/* legacy MIPI SRC DIV, kept to avoid conflict with old code */
#define OMAP_MCAM_SRC_DIV_MIPI	            4

/* must match value defined in device tree schema */
#define MAPPHONE_BP_VIACBP71            0x00220000

#define is_cdma_phone() (!strcmp("CDMA", bp_model))
extern char *bp_model;

extern void change_vio_mode(int source, int value);
extern int is_cpcap_vio_supply_converter(void);
#endif /*  __ASM_ARCH_OMAP_MAPPHONE_H */
