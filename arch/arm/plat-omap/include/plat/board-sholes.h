/*
 * linux/include/asm-arm/arch-omap/board-sholes.h
 *
 * Hardware definitions for OMAP3430-based Motorola reference design.
 *
 * Copyright (C) 2007 Motorola, Inc.
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

#ifndef __ASM_ARCH_OMAP_SHOLES_H
#define __ASM_ARCH_OMAP_SHOLES_H

extern void sholes_usb_init(void);
extern void sholes_flash_init(void);
extern void sholes_panel_init(void);
extern void sholes_sensors_init(void);
extern void sholes_hsmmc_init(void);
extern void sholes_camera_init(void);
extern void sholes_mmcprobe_init(void);

#if defined(CONFIG_VIDEO_MT9P012) || defined(CONFIG_VIDEO_MT9P012_MODULE)
extern struct mt9p012_platform_data sholes_mt9p012_platform_data;
#endif
#ifdef CONFIG_VIDEO_OMAP3_HPLENS
extern struct hplens_platform_data sholes_hplens_platform_data;
#endif

#define GPIO_MT9P012_STANDBY		58
#define GPIO_MT9P012_RESET		98
#define GPIO_SILENCE_KEY		100
#define GPIO_SLIDER			177

/*
#define GPIO_SIGNAL_LCD_PANEL_RESET	92
#define GPIO_SIGNAL_LCD_PANEL_SD	93
#define GPIO_SIGNAL_USB_IPC_RESET_PHY	149 
#define GPIO_SIGNAL_ATMEGA_INT		180
#define GPIO_SIGNAL_ATMEGA_RESET	87
#define GPIO_SIGNAL_TCH_INT		109
#define GPIO_SIGNAL_TCH_RESET		164
#define GPIO_SIGNAL_MMC_DET		163
*/

#endif /*  __ASM_ARCH_OMAP_SHOLES_H */

