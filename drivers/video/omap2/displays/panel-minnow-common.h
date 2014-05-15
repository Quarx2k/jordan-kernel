/*
 * Minnow DSI command mode panel - common initialization data header
 *
 * Copyright (C) 2013-2014 Motorola Mobility LLC.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _MINNOW_PANEL_COMMON_HEADER_

/* This header file is used to sync Bootloader and Kernel Display Initialize
 * Structure/Data, please make sure sync it for both Bootloader/Kernel when
 * it changes some settings for Solomon/Orise
 */
#ifndef	u8
typedef	unsigned char		u8;
#endif

enum minnow_panel_type {
	PANEL_INIT = -1,	/* Not Initialize */
	PANEL_DUMMY,		/* None panel detected */
	OTM3201_1_0,		/* Initial Orise Panel 1.0, it's not stable */
	OTM3201_2_0,		/* Orise Panel 2.0, only for test purpose */
	OTM3201_2_1,		/* Orise Panel 2.1 */
};

/* Panel initialize command type description:
 * DCS_WRITE_SYNC, DCS_WRITE:
 *   standard DCS type command with/without BTA sync
 * GENERIC_WRITE_SYNC, GENERIC_WRITE:
 *   standard Generic type command with/without BTA sync
 * BTA_SYNC:
 *   standard BTA sync command
 * WAIT_MS:
 *   sleep for given milliseconds
 * SSD2848_CMD:
 *   special command for SSD2848 bridge register, it has fixed 6 bytes format,
 *   the fist 2 bytes is the register address, the last is 32 bits register
 * OTM3201_CMD:
 *   special command for OTM3201 register, it has flexible register length,
 *   the fist byte is the register address
 * SWITCH_TO_PANEL:
 *   turn on/off bridge retransmission mode by follow 1 byte
 *   this is a indicator that separate initialize sequence for bridge and panel.
 *
 * There's different requirement for bridge and panel initialization,
 *   for bridge, it needs initialize first and it should verify all registers
 *     only after write all the registers. when the verification is failed,
 *     it must reset hardware to retry initialize.
 *   for panel, it needs initialize after bridge, and it should verify each
 *     register after each write. when the verify is failed, it could retry to
 *     re-write the failed register.
 */
enum minnow_cmd_type {
	DCS_WRITE_SYNC,
	GENERIC_WRITE_SYNC,
	DCS_WRITE,
	GENERIC_WRITE,
	BTA_SYNC,
	WAIT_MS,
	SSD2848_CMD,
	OTM3201_CMD,
	SWITCH_TO_PANEL,
	CMD_TYPE_MAX
};
/* Special register id to indicate the verification is needed */
#define	CMD_VERIFY_REG			0xFF

/* Panel initialize command buffer description:
 * it uses compact buffer to store all initialize commands, the first
 * byte of each command is the command length in byte
 */
static u8 panel_init_ssd2848_320x320[] = {
/*n, type, data_0, data_1 ... data_n-1*/
1, SWITCH_TO_PANEL, 0,
6, SSD2848_CMD, 0x00, 0x08, 0x01, 0xF4, 0x05, 0x32,
6, SSD2848_CMD, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x0B,
6, SSD2848_CMD, 0x00, 0x14, 0x0C, 0x07, 0x80, 0x0F,
1, DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1, WAIT_MS, 1,
6, SSD2848_CMD, 0x10, 0x08, 0x01, 0x20, 0x01, 0x45,
6, SSD2848_CMD, 0x20, 0x0C, 0x00, 0x00, 0x00, 0x02,
6, SSD2848_CMD, 0x20, 0x10, 0x00, 0xF7, 0x00, 0x6C,
6, SSD2848_CMD, 0x20, 0x14, 0x01, 0x88, 0x00, 0x2A,
6, SSD2848_CMD, 0x20, 0x18, 0x01, 0x4E, 0x00, 0x0A,
6, SSD2848_CMD, 0x20, 0x20, 0x01, 0x40, 0x01, 0x40,
6, SSD2848_CMD, 0x20, 0x24, 0x01, 0x40, 0x01, 0x40,
6, SSD2848_CMD, 0x20, 0x30, 0x00, 0x00, 0x00, 0x15,
6, SSD2848_CMD, 0x20, 0x34, 0x00, 0x00, 0x00, 0x00,
6, SSD2848_CMD, 0x20, 0x38, 0x01, 0x3F, 0x01, 0x3F,
6, SSD2848_CMD, 0x20, 0x3C, 0x01, 0x40, 0x01, 0x40,
6, SSD2848_CMD, 0x20, 0xA0, 0x00, 0x00, 0x05, 0x00,
5, DCS_WRITE_SYNC, 0x2A, 0x00, 0x00, 0x01, 0x3F,
5, DCS_WRITE_SYNC, 0x2B, 0x00, 0x00, 0x01, 0x3F,
6, SSD2848_CMD, 0x60, 0x08, 0x00, 0x02, 0x00, 0x0A,
6, SSD2848_CMD, 0x60, 0x0C, 0x0A, 0x2A, 0x02, 0x0A,
6, SSD2848_CMD, 0x60, 0x10, 0x01, 0x40, 0x02, 0x14,
6, SSD2848_CMD, 0x60, 0x14, 0x01, 0x00, 0x01, 0x00,
6, SSD2848_CMD, 0x60, 0x40, 0x13, 0x01, 0x0A, 0x01,
6, SSD2848_CMD, 0x60, 0x44, 0x05, 0x05, 0x04, 0x0A,
6, SSD2848_CMD, 0x60, 0x84, 0x00, 0x00, 0x01, 0x40,
1, SSD2848_CMD, CMD_VERIFY_REG, /* command for verify ssd2848 registers */
1, SWITCH_TO_PANEL, 1,
3, OTM3201_CMD, 0xF0, 0x54, 0x47,
2, OTM3201_CMD, 0xA0, 0x00,
4, OTM3201_CMD, 0xBD, 0x00, 0x11, 0x31,
2, OTM3201_CMD, 0xE9, 0x46,
2, OTM3201_CMD, 0xB1, 0x12,
2, OTM3201_CMD, 0xE2, 0xF0,
5, OTM3201_CMD, 0xBA, 0x06, 0x15, 0x2B, 0x01,
6, OTM3201_CMD, 0xB3, 0x02, 0x0A, 0x14, 0x2A, 0x2A,
5, OTM3201_CMD, 0xB5, 0x78, 0x78, 0x76, 0xF6,
18, OTM3201_CMD, 0xC0, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC1, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC2, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC3, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC4, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
18, OTM3201_CMD, 0xC5, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
1, OTM3201_CMD, CMD_VERIFY_REG, /* command for verify otm3201 registers */
2, OTM3201_CMD, 0xA0, 0x80,
3, OTM3201_CMD, 0xF0, 0x00, 0x00,
1, DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1, WAIT_MS, 120,
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_ON,
1, SWITCH_TO_PANEL, 0,
0
};

/* Special settings for OTM3201 PANEL revision 1.0
 * Orise panel 1.0 has known issue that don't meet MIPI timing requirement
 * it needs the different timing setting that apply for panel 2.0 or above
 */
static u8 panel_init_ssd2848_320x320_1[] = {
6, SSD2848_CMD, 0x60, 0x44, 0x0A, 0x0A, 0x04, 0x16,
0
};

static u8 panel_off_ssd2848_320x320[] = {
/*n, type, data_0, data_1 ... data_n-1*/
1, SWITCH_TO_PANEL, 1,
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_OFF,
1, DCS_WRITE, MIPI_DCS_ENTER_SLEEP_MODE,
1, SWITCH_TO_PANEL, 0,
1, DCS_WRITE_SYNC, MIPI_DCS_SET_DISPLAY_OFF,
1, WAIT_MS, 50,
1, DCS_WRITE, MIPI_DCS_ENTER_SLEEP_MODE,
1, WAIT_MS, 20,
6, GENERIC_WRITE, 0x10, 0x28, 0x00, 0x00, 0x00, 0x01, /*power cut enabled*/
1, WAIT_MS, 5,
0
};

/* Special code to process Orise internal register */
static u8 otm3201_eng_mode[] = { 0xF0, 0x54, 0x47 };
static u8 otm3201_write_mode[] = { 0xA0, 0x00 };
static u8 otm3201_read_mode[] = { 0xA0, 0x80 };

#define _MINNOW_PANEL_COMMON_HEADER_
#endif
