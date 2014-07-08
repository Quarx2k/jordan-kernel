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

#define	INIT_DATA_VERSION	(0x070714) /*MM/DD/YY*/
/* This header file is used to sync Bootloader and Kernel Display Initialize
 * Structure/Data, please make sure sync it for both Bootloader/Kernel when
 * it changes some settings for Solomon/Orise. Bootloader should pass
 * INIT_DATA_VERSION to kernel that make sure settings are same on both side.
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
/* SCM PLL Register[0x0008]
 *   POSTDIV = 5, MULT = 50, PLLOUT = 26 x 50 / (3+1) = 325.0 MHz
 */
6, SSD2848_CMD, 0x00, 0x08, 0x01, 0xF4, 0x03, 0x32,
/* SCM Clock Control Register[0x000C]
 *   MTXDIV = 1, MIPITX speed = 325.0 / (1 + 1) = 162.5 Mbps
 *               MIPITX clock = 162.5 / 2 = 81.3 MHz
 *   SYSDIV - 11, system clock = 325.0 / 2 / (11 + 1) = 13.5 MHz
 */
6, SSD2848_CMD, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x1B,
/* SCM Miscellaneous Control Register[0x0014]
 *   MTXVPF = 3, MIPITX Video Pixel Format = 24bpp
 *   MRXLS = 0, MIPIRX Lane Select = 1 lane
 *   MRXECC = MRXCRC = MRXEOT = MRXEE = 1
 *     enable MIPIRX ECC, CRC, EOR, Error Check
 */
6, SSD2848_CMD, 0x00, 0x14, 0x0C, 0x07, 0x80, 0x0F,
/* Sleep out and Waiting for SSD2848 PLL locked */
1, DCS_WRITE_SYNC, MIPI_DCS_EXIT_SLEEP_MODE,
1, WAIT_MS, 1,
/* MIPIRX Delay Configuration Register[0x0008] */
6, SSD2848_CMD, 0x10, 0x08, 0x01, 0x20, 0x01, 0x45,
/* VTCM Configuration Register[0x000C]
 *   TE_SEL = 1, Tear signal from display interface unit
 *   TEP = 0, vtcm_rgb_te signal Active high
 */
6, SSD2848_CMD, 0x20, 0x0C, 0x00, 0x00, 0x00, 0x02,
/* VTCM Pixel Clock Frequency Ratio Register[0x0010]
 *   PCLKDEN = 23, PCLKNUM = 5,
 *     Pixel clock = 13.5 x 5 / 23 = 2.93 MHz
 * since SSD2848 uses 48 bits bus, the actual pixel clock is
 *   depend on current pixel format setting(24 bpp now)
 *   actual pix_clk = 2.93 * 48 / 24 = 5.86 MHz
 */
6, SSD2848_CMD, 0x20, 0x10, 0x00, 0x17, 0x00, 0x05,
/* VTCM Display Horizontal Configuration Register[0x0014]
 *   Horizontal Total = 392 = 20 + 42 + 320 + 10
 *   Horizontal Display Period Start = 42
 */
6, SSD2848_CMD, 0x20, 0x14, 0x01, 0x88, 0x00, 0x2A,
/* VTCM Vertical Display Configuration Register[0x0018]
 *   Vertical Total = 334 = 2 + 10 + 320 + 2
 *   Vertical Display Period Start = 10
 */
6, SSD2848_CMD, 0x20, 0x18, 0x01, 0x4E, 0x00, 0x0A,
/* VTCM Display Size Register[0x0020]
 *   Display Width = 320
 *   Display Height = 320
 */
6, SSD2848_CMD, 0x20, 0x20, 0x01, 0x40, 0x01, 0x40,
/* VTCM Panel Size Register[0x0024]
 *   Panel Width = 320
 *   Panel Height = 320
 */
6, SSD2848_CMD, 0x20, 0x24, 0x01, 0x40, 0x01, 0x40,
/* VTCM URAM Control Register[0x0030] */
6, SSD2848_CMD, 0x20, 0x30, 0x00, 0x00, 0x00, 0x15,
/* VTCM Panel Offset Start Register[0x0034] */
6, SSD2848_CMD, 0x20, 0x34, 0x00, 0x00, 0x00, 0x00,
/* VTCM Panel Offset End Register[0x0038] */
6, SSD2848_CMD, 0x20, 0x38, 0x01, 0x3F, 0x01, 0x3F,
/* VTCM Image Size Register[0x003C] */
6, SSD2848_CMD, 0x20, 0x3C, 0x01, 0x40, 0x01, 0x40,
/* VTCM Qualcomm FBC Control Register[0x00A0]
 *   DEC_MODE = COM_MODE = 0 Bypass
 */
6, SSD2848_CMD, 0x20, 0xA0, 0x00, 0x00, 0x05, 0x00,
5, DCS_WRITE_SYNC, 0x2A, 0x00, 0x00, 0x01, 0x3F,
5, DCS_WRITE_SYNC, 0x2B, 0x00, 0x00, 0x01, 0x3F,
/* DSITX Control Register[0x0008]
 *   LPD = 1, LP clock = 162.5 / 8 / (1 + 1) = 10.15 MHz
 *   EOT = 1, EOT Packet Enable
 */
6, SSD2848_CMD, 0x60, 0x08, 0x00, 0x01, 0x00, 0x0A,
/* DSITX Video Timing Control Register[0x000C]
 *   VBP = 10, HBP = 42, VSA = 2, HSA = 10
 */
6, SSD2848_CMD, 0x60, 0x0C, 0x0A, 0x2A, 0x02, 0x0A,
/* DSITX Video Timing Control 2 Register[0x0010]
 *   VACT = 320, VFP = 2, HFP = 20
 */
6, SSD2848_CMD, 0x60, 0x10, 0x01, 0x40, 0x02, 0x14,
/* DSITX Video Configuration Register[0x0014]
 *   VM = 00, Non burst mode with sync pulses
 *   VEC = 1, Command packet will be sent after video packet
       are sent during Vertical blanking period for Non burst
       video transfer or Vertical/Horizontal blanking period
       for Burst video transfer
 */
6, SSD2848_CMD, 0x60, 0x14, 0x01, 0x00, 0x01, 0x40,
/* DSITX Delay Adjustment 1 Register[0x0040]
 * HPD = 1, HZD = 10, CPD = 1, CZD = 19
 *   byte_clk = 1000 / (216.6 / 8) = 36.9 ns
 *   Ths-prepare-HPD = 36.9 * (1 + 3) = 147.7 ns
 *   Ths-zero-HZD = 36.9 * (10 + 1.25) = 415.5 ns
 *   Tclk-prepare-CPD = 36.9 * (1 + 3) = 147.7 ns
 *   Tclk-zero-CZD = 36.9 * (19 + 1.25) = 747.9 ns
 */
6, SSD2848_CMD, 0x60, 0x40, 0x13, 0x01, 0x0A, 0x01,
/* DSITX Delay Adjustment 2 Register[0x0044]
 * CPTD = 10, CPED = 4, HTD = 5, CTD = 5
 *   byte_clk = 1000 / (216.6 / 8) = 36.9 ns
 *   Tclk-post-CPTD = 36.9 * (10 + 2) = 443.2 ns
 *   Tclk-pre-CPED = 36.9 * 4 = 147.7 ns
 *   Ths-trail-HTD = 36.9 * (5 - 1.25) = 138.5 ns
 *   Tclk-trail-CTD = 36.9 * (5 + 1) - 4 = 217.6 ns
 */
6, SSD2848_CMD, 0x60, 0x44, 0x05, 0x05, 0x04, 0x0A,
/* DSITX DSIn Video Register[0x0080+(n*32) + 0x004]
 *   HACT = 320
 */
6, SSD2848_CMD, 0x60, 0x84, 0x00, 0x00, 0x01, 0x40,
1, SSD2848_CMD, CMD_VERIFY_REG, /* command for verify ssd2848 registers */
1, SWITCH_TO_PANEL, 1,
/* Orise Engineering Mode Enable (RF0h)
 *   Enable Engineering Mode
 */
3, OTM3201_CMD, 0xF0, 0x54, 0x47,
/* Register Read Mode Enable (RA0h)
 *   Enable to write
 */
2, OTM3201_CMD, 0xA0, 0x00,
/* Mux1 to 9 CKH timing structure register (RBDH) */
4, OTM3201_CMD, 0xBD, 0x00, 0x11, 0x31,
/* Landscape MIPI Video Mode One Line Clock Number (RE9h) */
2, OTM3201_CMD, 0xE9, 0x46,
/* Display Inversion Control (RB1h) */
2, OTM3201_CMD, 0xB1, 0x12,
/* ??? undefined */
2, OTM3201_CMD, 0xE2, 0xF0,
/* Display Waveform Cycle setting (RBAh)  */
5, OTM3201_CMD, 0xBA, 0x06, 0x15, 0x2B, 0x01,
/* RGB Interface Blanking Porch setting (RB3h)
 *   VFP = 2, VBP = 10, HFP = 20, HBP = 42, VSW = 2, HSW = 10
 */
6, OTM3201_CMD, 0xB3, 0x02, 0x0A, 0x14, 0x2A, 0x2A,
/* Gamma Voltage adjust Control (RB5h) */
5, OTM3201_CMD, 0xB5, 0x78, 0x78, 0x76, 0xF6,
/* Gamma (‘+’polarity) Correction Characteristics Setting R gamma (RC0h) */
18, OTM3201_CMD, 0xC0, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
/* Gamma (‘-’polarity) Correction Characteristics Setting R gamma (RC1h) */
18, OTM3201_CMD, 0xC1, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
/* Gamma (‘+’polarity) Correction Characteristics Setting G gamma (RC2h) */
18, OTM3201_CMD, 0xC2, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
/* Gamma (‘-’polarity) Correction Characteristics Setting G gamma (RC3h) */
18, OTM3201_CMD, 0xC3, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
/* Gamma (‘+’polarity) Correction Characteristics Setting B gamma (RC4h) */
18, OTM3201_CMD, 0xC4, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
/* Gamma (‘-’polarity) Correction Characteristics Setting B gamma (RC5h) */
18, OTM3201_CMD, 0xC5, 0x00, 0x06, 0x17, 0x11, 0x16, 0x25, 0x0E,
		0x0C, 0x0C, 0x0E, 0x0C, 0x2F, 0x07, 0x0A, 0x3F, 0x3F, 0x3F,
1, OTM3201_CMD, CMD_VERIFY_REG, /* command for verify otm3201 registers */
/* Register Read Mode Enable (RA0h)
 *   Enable to read, locked for write
 */
2, OTM3201_CMD, 0xA0, 0x80,
/* Orise Engineering Mode Enable (RF0h)
 *   Disable Engineering Mode, locked for second group register
 */
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
/* SCM PLL Register[0x0008]
 *   POSTDIV = 5, MULT = 50, PLLOUT = 26 x 50 / (5+1) = 216.6 MHz
 */
6, SSD2848_CMD, 0x00, 0x08, 0x01, 0xF4, 0x05, 0x32,
/* SCM Clock Control Register[0x000C]
 *   MTXDIV = 0, MIPITX speed = 216.6 / (0 + 1) = 216.6 Mbps
 *               MIPITX clock = 216.6 / 2 = 108.3 MHz
 *   SYSDIV - 11, system clock = 216.6 / 2 / (11 + 1) = 9.0 MHz
 */
6, SSD2848_CMD, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x0B,
/* VTCM Pixel Clock Frequency Ratio Register[0x0010]
 *   PCLKDEN = 247, PCLKNUM = 108,
 *     Pixel clock = 9.0 x 108 / 247 = 3.93 MHz
 * since SSD2848 uses 48 bits bus, the actual pixel clock is
 *   depend on current pixel format setting(24 bpp now)
 *   actual pix_clk = 3.93 * 48 / 24 = 7.87 MHz
 */
6, SSD2848_CMD, 0x20, 0x10, 0x00, 0xF7, 0x00, 0x6C,
/* DSITX Control Register[0x0008]
 *   LPD = 4, LP clock = 216.6 / 8 / (4 + 1) = 5.4 MHz
 *   EOT = 1, EOT Packet Enable
 */
6, SSD2848_CMD, 0x60, 0x08, 0x00, 0x04, 0x00, 0x0A,
/* DSITX Delay Adjustment 2 Register[0x0044]
 * CPTD = 22, CPED = 4, HTD = 10, CTD = 10
 *   byte_clk = 1000 / (216.6 / 8) = 36.9 ns
 *   Tclk-post-CPTD = 36.9 * (22 + 2) = 886.4 ns
 *   Tclk-pre-CPED = 36.9 * 4 = 147.7 ns
 *   Ths-trail-HTD = 36.9 * (10 - 1.25) = 323.1 ns
 *   Tclk-trail-CTD = 36.9 * (10 + 1) - 4 = 402.2 ns
 */
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
/* MIPIRX Power Cut Register[0x0028]
 *   PWC - This bit will enable power cut to the whole chip and only global
 *         reset can restore the power supply to the chip.
 */
6, GENERIC_WRITE, 0x10, 0x28, 0x00, 0x00, 0x00, 0x01,
1, WAIT_MS, 5,
0
};

/* Special code to process Orise internal register */
static u8 otm3201_eng_mode[] = { 0xF0, 0x54, 0x47 };
static u8 otm3201_write_mode[] = { 0xA0, 0x00 };
static u8 otm3201_read_mode[] = { 0xA0, 0x80 };

#define _MINNOW_PANEL_COMMON_HEADER_
#endif
