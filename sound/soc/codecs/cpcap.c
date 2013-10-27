/*
 * Copyright (C) 2007 - 2009 Motorola, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/cpcap_audio_platform_data.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/tlv.h>

#include "cpcap.h"

/* #define CPCAP_AUDIO_DEBUG */
#ifdef CPCAP_AUDIO_DEBUG
#define CPCAP_AUDIO_DEBUG_LOG(args...) printk(KERN_INFO "ALSA CPCAP:" args)
#else
#define CPCAP_AUDIO_DEBUG_LOG(args...)
#endif

/*constants for ST delay workaround*/
#define STM_STDAC_ACTIVATE_RAMP_TIME   1
#define STM_STDAC_EN_TEST_PRE          0x090C
#define STM_STDAC_EN_TEST_POST         0x0000
#define STM_STDAC_EN_ST_TEST1_PRE      0x2400
#define STM_STDAC_EN_ST_TEST1_POST     0x0400

/*power control flag for EMU anti-pop*/
static int emu_analog_antipop;
static struct cpcap_audio_state *cpcap_global_state_pointer;

static const unsigned short cpcap_audio_reg_mask[CPCAP_AUDIO_REG_NUM] = {
	0x0077, /* [512] CPCAP_REG_VAUDIOC */
	0xFFFF, /* [513] CPCAP_REG_CC */
	0xBFFF, /* [514] CPCAP_REG_CDI */
	0x0FFF, /* [515] CPCAP_REG_SDAC */
	0x3FFF, /* [516] CPCAP_REG_SDACDI */
	0x0FDF, /* [517] CPCAP_REG_TXI */
	0x0FFF, /* [518] CPCAP_REG_TXMP */
	0x01FF, /* [519] CPCAP_REG_RXOA */
	0xFF3C, /* [520] CPCAP_REG_RXVC */
	0x07FF, /* [521] CPCAP_REG_RXCOA */
	0x1FFF, /* [522] CPCAP_REG_RXSDOA */
	0x7FFF, /* [523] CPCAP_REG_RXEPOA */
	0xFFFF, /* [524] CPCAP_REG_RXLL */
	0x00FF, /* [525] CPCAP_REG_A2LA */
};

static int snd_soc_put_cpcap_switch(struct snd_kcontrol *,
				    struct snd_ctl_elem_value *);

static int snd_soc_put_cpcap_mixer(struct snd_kcontrol *,
				   struct snd_ctl_elem_value *);

static int cpcap_audio_power_event(struct snd_soc_dapm_widget *,
				   struct snd_kcontrol *, int);

static int snd_soc_get_cpcap_gpio(struct snd_kcontrol *,
				  struct snd_ctl_elem_value *);

static int snd_soc_put_cpcap_gpio(struct snd_kcontrol *,
				  struct snd_ctl_elem_value *);

static int snd_soc_get_cpcap_dai_mode(struct snd_kcontrol *,
				      struct snd_ctl_elem_value *);

static int snd_soc_put_cpcap_dai_mode(struct snd_kcontrol *,
				      struct snd_ctl_elem_value *);

static int snd_soc_get_emu_antipop(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);

static int snd_soc_set_emu_antipop(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol);
/*
 * Capture Gain Control:
 * from 0dB to 31dB in 1dB steps
 */
static DECLARE_TLV_DB_SCALE(mic_gain_tlv, 0, 100, 0);

/*
 * Playback Gain Control:
 * from -33dB to 12dB in 3dB steps
 */
static DECLARE_TLV_DB_SCALE(vol_tlv, -3300, 300, 0);

/* Right analog microphone selection
 * 0 - "Off"
 * 1 - "Mic1"
 * 2 - "Hs Mic"
 * 4 - "EMU Mic"
 * Other values are invalid
 */
static const char *cpcap_mic1_texts[] = {
	"Off", "Mic1", "HS Mic", "NULL", "EMU Mic"};

/* Left analog microphone selection
 * 0 - "Off"
 * 1 - "Mic2"
 */
static const char *cpcap_mic2_texts[] = {
	"Off", "Mic2"};

/* External RX PGA Right output
 * 0 - "Off"
 * 1 - "EXT Mic"
 */
static const char *cpcap_ext_micr_texts[] = {
	"Off", "EXT MicR"};

/* External RX PGA Left output
 * 0 - "Off"
 * 1 - "EXT Micl"
 */
static const char *cpcap_ext_micl_texts[] = {
	"Off", "EXT MicL"};

static const struct soc_enum cpcap_mic_enum[] = {
	SOC_ENUM_SINGLE(CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
		2, 5, cpcap_mic1_texts),
	SOC_ENUM_SINGLE(CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
		7, 2, cpcap_mic2_texts),
	SOC_ENUM_SINGLE(CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
		8, 2, cpcap_ext_micr_texts),
	SOC_ENUM_SINGLE(CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
		9, 2, cpcap_ext_micl_texts),
};

static const struct snd_kcontrol_new cpcap_micr_control =
	SOC_DAPM_ENUM("Route", cpcap_mic_enum[0]);

static const struct snd_kcontrol_new cpcap_micl_control =
	SOC_DAPM_ENUM("Route", cpcap_mic_enum[1]);

static const struct snd_kcontrol_new cpcap_ext_micr_control =
	SOC_DAPM_ENUM("Route", cpcap_mic_enum[2]);

static const struct snd_kcontrol_new cpcap_ext_micl_control =
	SOC_DAPM_ENUM("Route", cpcap_mic_enum[3]);

/* CPCAP Codec operation modes */
static const char *cpcap_codec_op_modes_texts[] = {
	"Audio",
	"Voice Call Handset",
	"Voice Call Headset",
	"Voice Call Headset Mic",
	"Voice Call BT"
};

static const struct soc_enum cpcap_codec_op_modes =
	SOC_ENUM_SINGLE_EXT(ARRAY_SIZE(cpcap_codec_op_modes_texts),
			    cpcap_codec_op_modes_texts);

/* CDC playback switches */
static const struct snd_kcontrol_new epcdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 0, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new ldsprcdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 1, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new ldsplcdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 2, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new linercdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 3, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new linelcdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 4, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new hsrcdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 5, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new hslcdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 6, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new usbdmcdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 7, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new usbdpcdc_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 8, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

/* STDAC playback switches */
static const struct snd_kcontrol_new epdac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 0, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new ldsprdac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 1, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new ldspldac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 2, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new linerdac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 3, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new lineldac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 4, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new hsrdac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 5, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new hsldac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 6, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new usbdmdac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 7, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new usbdpdac_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 8, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

/* ExternalPGA playback switches */
static const struct snd_kcontrol_new epext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 0, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new ldsprext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 1, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new ldsplext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 2, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new linerext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 3, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new linelext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 4, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new hsrext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 5, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new hslext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 6, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new usbdmext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 7, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new usbdpext_switch_controls =
	SOC_SINGLE_EXT("Switch",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 8, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_switch);

static const struct snd_kcontrol_new cpcap_mixer_controls[] = {
	SOC_SINGLE_EXT("Voice Codec",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC), 6, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_mixer),
	SOC_SINGLE_EXT("Stereo DAC",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_SDAC), 0, 1, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_mixer),
	SOC_SINGLE_EXT("External PGA",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 11, 3, 0,
		snd_soc_dapm_get_volsw, snd_soc_put_cpcap_mixer),
};

static const struct snd_kcontrol_new cpcap_snd_controls[] = {
	/* Capture Gain */
	SOC_SINGLE_TLV("MIC1 Gain",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXMP),
		0, 0x1F, 0, mic_gain_tlv),
	SOC_SINGLE_TLV("MIC2 Gain",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXMP),
		5, 0x1F, 0, mic_gain_tlv),
	/* External PGA Volume */
	SOC_SINGLE_TLV("Ext Volume",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXVC),
		12, 0xF, 0, vol_tlv),
	/* Stereo DAC Volume */
	SOC_SINGLE_TLV("STDac Volume",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXVC),
		8, 0xF, 0, vol_tlv),
	/* Codec Volume */
	SOC_SINGLE_TLV("Codec Volume",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXVC),
		2, 0xF, 0, vol_tlv),
	/* Enable/Disable Loopback Mode */
	SOC_SINGLE("DLM",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
		0, 1, 0),
	/* Select between secondary and tertiary mic */
	SOC_SINGLE_BOOL_EXT("SecTer Mic Select",
		CPCAP_REG_GPIO4, snd_soc_get_cpcap_gpio,
		snd_soc_put_cpcap_gpio),
	SOC_ENUM_EXT("DAI Mode", cpcap_codec_op_modes,
		snd_soc_get_cpcap_dai_mode, snd_soc_put_cpcap_dai_mode),
	/* Enable EMU analog anti-pop for car dock */
	SOC_SINGLE_BOOL_EXT("EMU AntiPop",
		0, snd_soc_get_emu_antipop,
		snd_soc_set_emu_antipop),
};

static const struct snd_soc_dapm_widget cpcap_dapm_widgets[] = {
	/* Inputs */
	SND_SOC_DAPM_INPUT("MIC1R"),
	SND_SOC_DAPM_INPUT("MICHS"),
	SND_SOC_DAPM_INPUT("MICEMU"),
	SND_SOC_DAPM_INPUT("MICEXTR"),
	SND_SOC_DAPM_INPUT("MIC2L"),
	SND_SOC_DAPM_INPUT("MICEXTL"),

	/* Outputs */
	SND_SOC_DAPM_OUTPUT("LDSPR"),
	SND_SOC_DAPM_OUTPUT("LDSPL"),
	SND_SOC_DAPM_OUTPUT("HSR"),
	SND_SOC_DAPM_OUTPUT("HSL"),
	SND_SOC_DAPM_OUTPUT("LINER"),
	SND_SOC_DAPM_OUTPUT("LINEL"),
	SND_SOC_DAPM_OUTPUT("EMUSPKR"),
	SND_SOC_DAPM_OUTPUT("EMUSPKL"),
	SND_SOC_DAPM_OUTPUT("EP"),

	/* Analog input muxes for the capture amplifiers */
	SND_SOC_DAPM_MUX("Analog Right Capture Route",
		SND_SOC_NOPM, 0, 0, &cpcap_micr_control),
	SND_SOC_DAPM_MUX("Analog Left Capture Route",
		SND_SOC_NOPM, 0, 0, &cpcap_micl_control),
	SND_SOC_DAPM_MUX("Analog External Right Capture Route",
		SND_SOC_NOPM, 0, 0, &cpcap_ext_micr_control),
	SND_SOC_DAPM_MUX("Analog External Left Capture Route",
		SND_SOC_NOPM, 0, 0, &cpcap_ext_micl_control),

	/* Analog capture PGAs */
	SND_SOC_DAPM_PGA("MIC1PGA",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
		1, 0, NULL, 0),
	SND_SOC_DAPM_PGA("MIC2PGA",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
		6, 0, NULL, 0),

	/* ADCs */
	SND_SOC_DAPM_ADC("ADC Right", "Right Front Capture",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC), 4, 0),
	SND_SOC_DAPM_ADC("ADC Left", "Left Front Capture",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC), 7, 0),

	SND_SOC_DAPM_AIF_IN("AIFIN Voice", "Playback", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIFIN Multimedia", "Playback", 0,
		SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_AIF_IN("AIFIN ExternalPGA", "Playback", 0,
		SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MIXER("CPCAP Mixer",
		SND_SOC_NOPM, 0, 0, cpcap_mixer_controls,
		ARRAY_SIZE(cpcap_mixer_controls)),

	SND_SOC_DAPM_DAC_E("CDC Playback", "Codec Output",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 10, 0,
		cpcap_audio_power_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("DAC Playback", "STDac Output",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA), 12, 0,
		cpcap_audio_power_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("EXT Playback Right", "ExternalPGA INR",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 13, 0,
		cpcap_audio_power_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
	SND_SOC_DAPM_DAC_E("EXT Playback Left", "ExternalPGA INL",
		CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA), 14, 0,
		cpcap_audio_power_event,
		SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),

	/* Analog playback switches */
	SND_SOC_DAPM_SWITCH("EPCDC",
		SND_SOC_NOPM, 0, 0, &epcdc_switch_controls),
	SND_SOC_DAPM_SWITCH("LDSPRCDC",
		SND_SOC_NOPM, 0, 0, &ldsprcdc_switch_controls),
	SND_SOC_DAPM_SWITCH("LDSPLCDC",
		SND_SOC_NOPM, 0, 0, &ldsplcdc_switch_controls),
	SND_SOC_DAPM_SWITCH("LINERCDC",
		SND_SOC_NOPM, 0, 0, &linercdc_switch_controls),
	SND_SOC_DAPM_SWITCH("LINELCDC",
		SND_SOC_NOPM, 0, 0, &linelcdc_switch_controls),
	SND_SOC_DAPM_SWITCH("HSRCDC",
		SND_SOC_NOPM, 0, 0, &hsrcdc_switch_controls),
	SND_SOC_DAPM_SWITCH("HSLCDC",
		SND_SOC_NOPM, 0, 0, &hslcdc_switch_controls),
	SND_SOC_DAPM_SWITCH("USBDMCDC",
		SND_SOC_NOPM, 0, 0, &usbdmcdc_switch_controls),
	SND_SOC_DAPM_SWITCH("USBDPCDC",
		SND_SOC_NOPM, 0, 0, &usbdpcdc_switch_controls),

	SND_SOC_DAPM_SWITCH("EPDAC",
		SND_SOC_NOPM, 0, 0, &epdac_switch_controls),
	SND_SOC_DAPM_SWITCH("LDSPRDAC",
		SND_SOC_NOPM, 0, 0, &ldsprdac_switch_controls),
	SND_SOC_DAPM_SWITCH("LDSPLDAC",
		SND_SOC_NOPM, 0, 0, &ldspldac_switch_controls),
	SND_SOC_DAPM_SWITCH("LINERDAC",
		SND_SOC_NOPM, 0, 0, &linerdac_switch_controls),
	SND_SOC_DAPM_SWITCH("LINELDAC",
		SND_SOC_NOPM, 0, 0, &lineldac_switch_controls),
	SND_SOC_DAPM_SWITCH("HSRDAC",
		SND_SOC_NOPM, 0, 0, &hsrdac_switch_controls),
	SND_SOC_DAPM_SWITCH("HSLDAC",
		SND_SOC_NOPM, 0, 0, &hsldac_switch_controls),
	SND_SOC_DAPM_SWITCH("USBDMDAC",
		SND_SOC_NOPM, 0, 0, &usbdmdac_switch_controls),
	SND_SOC_DAPM_SWITCH("USBDPDAC",
		SND_SOC_NOPM, 0, 0, &usbdpdac_switch_controls),

	SND_SOC_DAPM_SWITCH("EPEXT",
		SND_SOC_NOPM, 0, 0, &epext_switch_controls),
	SND_SOC_DAPM_SWITCH("LDSPREXT",
		SND_SOC_NOPM, 0, 0, &ldsprext_switch_controls),
	SND_SOC_DAPM_SWITCH("LDSPLEXT",
		SND_SOC_NOPM, 0, 0, &ldsplext_switch_controls),
	SND_SOC_DAPM_SWITCH("LINEREXT",
		SND_SOC_NOPM, 0, 0, &linerext_switch_controls),
	SND_SOC_DAPM_SWITCH("LINELEXT",
		SND_SOC_NOPM, 0, 0, &linelext_switch_controls),
	SND_SOC_DAPM_SWITCH("HSREXT",
		SND_SOC_NOPM, 0, 0, &hsrext_switch_controls),
	SND_SOC_DAPM_SWITCH("HSLEXT",
		SND_SOC_NOPM, 0, 0, &hslext_switch_controls),
	SND_SOC_DAPM_SWITCH("USBDMEXT",
		SND_SOC_NOPM, 0, 0, &usbdmext_switch_controls),
	SND_SOC_DAPM_SWITCH("USBDPEXT",
		SND_SOC_NOPM, 0, 0, &usbdpext_switch_controls),
};

static const struct snd_soc_dapm_route intercon[] = {
	/* Capture path */
	{"Analog Right Capture Route", "Mic1", "MIC1R"},
	{"Analog Right Capture Route", "HS Mic", "MICHS"},
	{"Analog Right Capture Route", "EMU Mic", "MICEMU"},
	{"Analog External Right Capture Route", "EXT MicR", "MICEXTR"},

	{"Analog Left Capture Route", "Mic2", "MIC2L"},
	{"Analog External Left Capture Route", "EXT MicL", "MICEXTL"},

	{"MIC1PGA", NULL, "Analog Right Capture Route"},
	{"MIC2PGA", NULL, "Analog Left Capture Route"},

	{"ADC Right", NULL, "MIC1PGA"},
	{"ADC Right", NULL, "Analog External Right Capture Route"},
	{"ADC Left", NULL, "MIC2PGA"},
	{"ADC Left", NULL, "Analog External Left Capture Route"},

	/* Playback Path */
	{"CPCAP Mixer", "Voice Codec", "AIFIN Voice"},
	{"CPCAP Mixer", "Stereo DAC", "AIFIN Multimedia"},
	{"CPCAP Mixer", "External PGA", "AIFIN ExternalPGA"},

	{"CDC Playback", NULL, "CPCAP Mixer"},
	{"DAC Playback", NULL, "CPCAP Mixer"},
	{"EXT Playback Right", NULL, "CPCAP Mixer"},
	{"EXT Playback Left", NULL, "CPCAP Mixer"},

	/* To Earpice */
	{"EPCDC", "Switch", "CDC Playback"},
	{"EPDAC", "Switch", "DAC Playback"},
	{"EPEXT", "Switch", "EXT Playback Right"},

	/* To LDSP */
	{"LDSPRCDC", "Switch", "CDC Playback"},
	{"LDSPLCDC", "Switch", "CDC Playback"},
	{"LDSPRDAC", "Switch", "DAC Playback"},
	{"LDSPLDAC", "Switch", "DAC Playback"},
	{"LDSPREXT", "Switch", "EXT Playback Right"},
	{"LDSPLEXT", "Switch", "EXT Playback Left"},

	/* To Line Out */
	{"LINERCDC", "Switch", "CDC Playback"},
	{"LINELCDC", "Switch", "CDC Playback"},
	{"LINERDAC", "Switch", "DAC Playback"},
	{"LINELDAC", "Switch", "DAC Playback"},
	{"LINEREXT", "Switch", "EXT Playback Right"},
	{"LINELEXT", "Switch", "EXT Playback Left"},

	/* To HS */
	{"HSRCDC", "Switch", "CDC Playback"},
	{"HSLCDC", "Switch", "CDC Playback"},
	{"HSRDAC", "Switch", "DAC Playback"},
	{"HSLDAC", "Switch", "DAC Playback"},
	{"HSREXT", "Switch", "EXT Playback Right"},
	{"HSLEXT", "Switch", "EXT Playback Left"},

	/* To EMUSPK */
	{"USBDPCDC", "Switch", "CDC Playback"},
	{"USBDMCDC", "Switch", "CDC Playback"},
	{"USBDPDAC", "Switch", "DAC Playback"},
	{"USBDMDAC", "Switch", "DAC Playback"},
	{"USBDPEXT", "Switch", "EXT Playback Right"},
	{"USBDMEXT", "Switch", "EXT Playback Left"},

	/* CDC Playback Path */
	{"EP", NULL, "EPCDC"},
	{"LDSPR", NULL, "LDSPRCDC"},
	{"LDSPL", NULL, "LDSPLCDC"},
	{"LINER", NULL, "LINERCDC"},
	{"LINEL", NULL, "LINELCDC"},
	{"HSR", NULL, "HSRCDC"},
	{"HSL", NULL, "HSLCDC"},
	{"EMUSPKR", NULL, "USBDPCDC"},
	{"EMUSPKL", NULL, "USBDMCDC"},

	/* STDAC Playback Path */
	{"EP", NULL, "EPDAC"},
	{"LDSPR", NULL, "LDSPRDAC"},
	{"LDSPL", NULL, "LDSPLDAC"},
	{"LINER", NULL, "LINERDAC"},
	{"LINEL", NULL, "LINELDAC"},
	{"HSR", NULL, "HSRDAC"},
	{"HSL", NULL, "HSLDAC"},
	{"EMUSPKR", NULL, "USBDPDAC"},
	{"EMUSPKL", NULL, "USBDMDAC"},

	/* ExternalPGA Playback Path */
	{"EP", NULL, "EPEXT"},
	{"LDSPR", NULL, "LDSPREXT"},
	{"LDSPL", NULL, "LDSPLEXT"},
	{"LINER", NULL, "LINEREXT"},
	{"LINEL", NULL, "LINELEXT"},
	{"HSR", NULL, "HSREXT"},
	{"HSL", NULL, "HSLEXT"},
	{"EMUSPKR", NULL, "USBDPEXT"},
	{"EMUSPKL", NULL, "USBDMEXT"},
};

/* Define regulator to turn on the audio portion of cpcap */
struct vaudio_data {
	struct regulator *regulator;
	unsigned char mode;
};
static struct vaudio_data vaudio;

static int vaudio_mode(unsigned char mode)
{
	if (IS_ERR(vaudio.regulator)) {
		printk(KERN_ERR "%s: Invalid vaudio\n", __func__);
		return -EINVAL;
	}
	if (mode != vaudio.mode) {
		regulator_set_mode(vaudio.regulator, mode);
		vaudio.mode = mode;
	}
	if (mode == REGULATOR_MODE_NORMAL)
		mdelay(SLEEP_ACTIVATE_POWER);
	return 0;
}

static int vaudio_get(void)
{
	vaudio.regulator = regulator_get(NULL, "vaudio");
	if (IS_ERR(vaudio.regulator)) {
		printk(KERN_ERR "%s: invalid vaudio\n", __func__);
		return -EINVAL;
	}
	return 0;
}

/*
 * read cpcap audio register through spi
 */
static unsigned int cpcap_audio_reg_read(struct snd_soc_codec *codec,
					 unsigned int reg)
{
	unsigned short value;
	struct cpcap_device *cpcap;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);
	unsigned short *cache = codec->reg_cache;

	if (reg >= CPCAP_AUDIO_REG_NUM) {
		printk(KERN_ERR "%s: invalid register %u\n", __func__, reg);
		return -EIO;
	}
	if (!state) {
		printk(KERN_ERR "%s: codec init might have failed\n", __func__);
		return -EIO;
	}
	cpcap = state->cpcap;
	if (!cpcap || !cache) {
		printk(KERN_ERR "%s: invalid pointers cpcap_device = %p"
				" cpcap_reg_cache = %p", __func__,
			cpcap, cache);
		return -EIO;
	}

	if (!cpcap_regacc_read(cpcap, CPCAP_AUDIO_INDEX_REG(reg), &value)) {
		cache[reg] = value;
		return value;
	} else {
		printk(KERN_ERR "%s: failed to read %u\n", __func__, reg);
		return -EIO;
	}
}

/*
 * write cpcap audio register through spi, update cache as well
 */
static int cpcap_audio_reg_write(struct snd_soc_codec *codec,
				 unsigned int reg,
				 unsigned int value)
{
	struct cpcap_device *cpcap;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);
	unsigned short *cache = codec->reg_cache;

	if (reg >= CPCAP_AUDIO_REG_NUM) {
		printk(KERN_ERR "%s: invalid register %u\n", __func__, reg);
		return -EIO;
	}
	if (!state) {
		printk(KERN_ERR "%s: codec init might have failed\n", __func__);
		return -EIO;
	}
	cpcap = state->cpcap;
	if (!cpcap || !cache) {
		printk(KERN_ERR "%s: invalid pointers cpcap_device = %p"
				" cpcap_reg_cache = %p", __func__,
			cpcap, cache);
		return -EIO;
	}

	if (!cpcap_regacc_write(cpcap, CPCAP_AUDIO_INDEX_REG(reg),
				value, cpcap_audio_reg_mask[reg])) {
		cache[reg] = value & cpcap_audio_reg_mask[reg];
		return 0;
	} else {
		printk(KERN_ERR "%s: failed to write %u to register %u\n",
			__func__, value, reg);
		return -EIO;
	}
}

static void cpcap_audio_register_dump(struct snd_soc_codec *codec)
{
	unsigned short *cache;
	int i = 0;

	if (!codec) {
		printk(KERN_ERR "%s: invalid pointer\n", __func__);
		return;
	} else
		cache = codec->reg_cache;

	for (i = 0; i < CPCAP_AUDIO_REG_NUM; i++)
		cpcap_audio_reg_read(codec, i);
	dev_dbg(codec->dev, "\t0x200 = %x\n\t0x201 = %x\n\t0x202 = %x\n"
			   "\t0x203 = %x\n\t0x204 = %x\n\t0x205 = %x\n"
			   "\t0x206 = %x\n\t0x207 = %x\n\t0x208 = %x\n"
			   "\t0x209 = %x\n\t0x20A = %x\n\t0x20B = %x\n"
			   "\t0x20C = %x\n\t0x20D = %x\n",
		cache[0], cache[1], cache[2], cache[3], cache[4],
		cache[5], cache[6], cache[7], cache[8], cache[9],
		cache[10], cache[11], cache[12], cache[13]);
}

static int snd_soc_put_cpcap_switch(struct snd_kcontrol *kcontrol,
				    struct snd_ctl_elem_value *ucontrol)
{
	int err;
	unsigned short val, value = 0, value_rxsdoa = 0, value_rxepoa = 0;
	unsigned int reg = CPCAP_AUDIO_REG_NUM;
	unsigned int reg_rxcoa = CPCAP_AUDIO_REG_NUM;
	unsigned int reg_rxsdoa = CPCAP_AUDIO_REG_NUM;
	unsigned int reg_rxepoa = CPCAP_AUDIO_REG_NUM;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int shift = mc->shift;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);
	struct cpcap_device *cpcap;
	unsigned short *cache;
	char *name = kcontrol->id.name;
	unsigned short need = 0;
	unsigned short value_rxcoa = 0;

	if (!state && cpcap_global_state_pointer) {
		state = cpcap_global_state_pointer;
		CPCAP_AUDIO_DEBUG_LOG("put_switch using global codec %p"
			" instead of %p\n", state->codec, codec);
		codec = state->codec;
	}

	cpcap = state->cpcap;
	cache = codec->reg_cache;

	CPCAP_AUDIO_DEBUG_LOG("%s: kcontrol named %s\n", __func__, name);

	val = (ucontrol->value.integer.value[0] << shift);

	reg_rxcoa = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA);
	value_rxcoa = cache[reg_rxcoa];

	reg_rxsdoa = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA);
	value_rxsdoa = cache[reg_rxsdoa];

	reg_rxepoa = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA);
	value_rxepoa = cache[reg_rxepoa];

	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXOA);
	value = cache[reg];

	if (strcmp(name, "EPCDC Switch") == 0 ||
	    strcmp(name, "EPDAC Switch") == 0 ||
	    strcmp(name, "EPEXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_A1_EAR_EN;
			value_rxsdoa |= CPCAP_BIT_MONO_DAC0 |
					CPCAP_BIT_MONO_DAC1;
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT0;
			value_rxepoa |= CPCAP_BIT_MONO_EXT1;
		} else {
			/* Turn off Earpiece only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa & CPCAP_BIT_A1_EAR_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa & CPCAP_BIT_A1_EAR_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa & CPCAP_BIT_A1_EAR_EXT_SW;
			if (!need)
				value = cache[reg] & ~CPCAP_BIT_A1_EAR_EN;
		}
	} else if (strcmp(name, "LDSPRCDC Switch") == 0 ||
		   strcmp(name, "LDSPRDAC Switch") == 0 ||
		   strcmp(name, "LDSPREXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_A2_LDSP_R_EN;
			value_rxsdoa |= CPCAP_BIT_MONO_DAC0 |
					CPCAP_BIT_MONO_DAC1;
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT0;
			value_rxepoa |= CPCAP_BIT_MONO_EXT1;
		} else {
			/* Turn off Right LDSP only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa &
					CPCAP_BIT_A2_LDSP_R_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa &
					CPCAP_BIT_A2_LDSP_R_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa &
					CPCAP_BIT_A2_LDSP_R_EXT_SW;
			if (!need)
				value = cache[reg] & ~CPCAP_BIT_A2_LDSP_R_EN;
		}
	} else if (strcmp(name, "LDSPLCDC Switch") == 0 ||
		   strcmp(name, "LDSPLDAC Switch") == 0 ||
		   strcmp(name, "LDSPLEXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_A2_LDSP_L_EN;
			value_rxsdoa |= CPCAP_BIT_MONO_DAC0 |
					CPCAP_BIT_MONO_DAC1;
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT0;
			value_rxepoa |= CPCAP_BIT_MONO_EXT1;
		} else {
			/* Turn off Left LDSP only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa &
					CPCAP_BIT_A2_LDSP_L_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa &
					CPCAP_BIT_A2_LDSP_L_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa &
					CPCAP_BIT_A2_LDSP_L_EXT_SW;
			if (!need)
				value = cache[reg] & ~CPCAP_BIT_A2_LDSP_L_EN;
		}
	} else if (strcmp(name, "LINERCDC Switch") == 0 ||
		   strcmp(name, "LINERDAC Switch") == 0 ||
		   strcmp(name, "LINEREXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_A4_LINEOUT_R_EN;
			/* Right and Left PGA separated */
			value_rxsdoa &= ~(CPCAP_BIT_MONO_DAC0 |
						CPCAP_BIT_MONO_DAC1);
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT1;
			value_rxepoa |= CPCAP_BIT_MONO_EXT0;
		} else {
			/* Turn off Right Line only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa &
					CPCAP_BIT_A4_LINEOUT_R_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa &
					CPCAP_BIT_A4_LINEOUT_R_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa &
					CPCAP_BIT_A4_LINEOUT_R_EXT_SW;
			if (!need)
				value = cache[reg] & ~CPCAP_BIT_A4_LINEOUT_R_EN;
		}
	} else if (strcmp(name, "LINELCDC Switch") == 0 ||
		   strcmp(name, "LINELDAC Switch") == 0 ||
		   strcmp(name, "LINELEXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_A4_LINEOUT_L_EN;
			/* Right and Left PGA separated */
			value_rxsdoa &= ~(CPCAP_BIT_MONO_DAC0 |
						CPCAP_BIT_MONO_DAC1);
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT1;
			value_rxepoa |= CPCAP_BIT_MONO_EXT0;
		} else {
			/* Turn off Left Line only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa &
					CPCAP_BIT_A4_LINEOUT_L_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa &
					CPCAP_BIT_A4_LINEOUT_L_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa &
					CPCAP_BIT_A4_LINEOUT_L_EXT_SW;
			if (!need)
				value = cache[reg] & ~CPCAP_BIT_A4_LINEOUT_L_EN;
		}
	} else if (strcmp(name, "HSRCDC Switch") == 0 ||
		   strcmp(name, "HSRDAC Switch") == 0 ||
		   strcmp(name, "HSREXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_HS_R_EN;
			/* Right and Left PGA separated */
			value_rxsdoa &= ~(CPCAP_BIT_MONO_DAC0 |
						CPCAP_BIT_MONO_DAC1);
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT1;
			value_rxepoa |= CPCAP_BIT_MONO_EXT0;
		} else {
			/* Turn off Right HS only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa &
					CPCAP_BIT_ARIGHT_HS_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa &
					CPCAP_BIT_ARIGHT_HS_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa &
					CPCAP_BIT_ARIGHT_HS_EXT_SW;
			if (!need)
				value = cache[reg] & ~CPCAP_BIT_HS_R_EN;
		}
	} else if (strcmp(name, "HSLCDC Switch") == 0 ||
		   strcmp(name, "HSLDAC Switch") == 0 ||
		   strcmp(name, "HSLEXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_HS_L_EN;
			/* Right and Left PGA separated */
			value_rxsdoa &= ~(CPCAP_BIT_MONO_DAC0 |
						CPCAP_BIT_MONO_DAC1);
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT1;
			value_rxepoa |= CPCAP_BIT_MONO_EXT0;
		} else {
			/* Turn off Left HS only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa &
					CPCAP_BIT_ALEFT_HS_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa &
					CPCAP_BIT_ALEFT_HS_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa &
					CPCAP_BIT_ALEFT_HS_EXT_SW;
			if (!need)
				value = cache[reg] & ~CPCAP_BIT_HS_L_EN;
		}
	} else if (strcmp(name, "USBDPCDC Switch") == 0 ||
		   strcmp(name, "USBDPDAC Switch") == 0 ||
		   strcmp(name, "USBDPEXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_EMU_SPKR_R_EN;
			/* Right and Left PGA separated */
			value_rxsdoa &= ~(CPCAP_BIT_MONO_DAC0 |
						CPCAP_BIT_MONO_DAC1);
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT1;
			value_rxepoa |= CPCAP_BIT_MONO_EXT0;
		} else {
			/* Turn off Right EMUSP only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa &
					CPCAP_BIT_PGA_OUTR_USBDP_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa &
					CPCAP_BIT_PGA_OUTR_USBDP_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa &
					CPCAP_BIT_PGA_OUTR_USBDP_EXT_SW;
			if (!need && !emu_analog_antipop)
				value = cache[reg] & ~CPCAP_BIT_EMU_SPKR_R_EN;
		}
	} else if (strcmp(name, "USBDMCDC Switch") == 0 ||
		   strcmp(name, "USBDMDAC Switch") == 0 ||
		   strcmp(name, "USBDMEXT Switch") == 0) {
		if (val) {
			value = cache[reg] | CPCAP_BIT_EMU_SPKR_L_EN;
			/* Right and Left PGA separated */
			value_rxsdoa &= ~(CPCAP_BIT_MONO_DAC0 |
						CPCAP_BIT_MONO_DAC1);
			value_rxepoa &= ~CPCAP_BIT_MONO_EXT1;
			value_rxepoa |= CPCAP_BIT_MONO_EXT0;
		} else {
			/* Turn off Left EMUSP only if no one else needs it */
			if (mc->reg != reg_rxcoa)
				need |= value_rxcoa &
					CPCAP_BIT_PGA_OUTL_USBDN_CDC_SW;
			if (mc->reg != reg_rxsdoa)
				need |= value_rxsdoa &
					CPCAP_BIT_PGA_OUTL_USBDN_DAC_SW;
			if (mc->reg != reg_rxepoa)
				need |= value_rxepoa &
					CPCAP_BIT_PGA_OUTL_USBDN_EXT_SW;
			if (!need && !emu_analog_antipop)
				value = cache[reg] & ~CPCAP_BIT_EMU_SPKR_L_EN;
		}
	}

	err = cpcap_audio_reg_write(codec, reg_rxsdoa, value_rxsdoa);
	if (err)
		return err;

	err = cpcap_audio_reg_write(codec, reg_rxepoa, value_rxepoa);
	if (err)
		return err;

	err = cpcap_audio_reg_write(codec, reg, value);
	if (err)
		return err;
	reg = mc->reg;
	if (val)
		value = cache[reg] | val;
	else
		value = cache[reg] & ~(1 << shift);

	err = cpcap_audio_reg_write(codec, reg, value);
	if (err)
		return err;

	return err;
}

static int snd_soc_get_emu_antipop(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = emu_analog_antipop;
	return 0;
}

static int snd_soc_set_emu_antipop(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);
	struct cpcap_device *cpcap = state->cpcap;
	unsigned short *cache = codec->reg_cache;
	unsigned int reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXOA);
	unsigned short value = cache[reg];
	unsigned short val = ucontrol->value.integer.value[0];

	CPCAP_AUDIO_DEBUG_LOG("%s: old value %u; new value %u\n",
			__func__, emu_analog_antipop, val);

	if (val > 1)
		return -EINVAL;
	else if (val == emu_analog_antipop)
		return 0;

	emu_analog_antipop = val;

	if (val == 1) {
		/*First enable vaudio if not already on*/
		if ((cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_SDAC)] &
				 CPCAP_BIT_ST_DAC_EN) == 0 &&
		    (cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC)] &
			(CPCAP_BIT_CDC_EN_RX | CPCAP_BIT_MIC1_CDC_EN |
					CPCAP_BIT_MIC2_CDC_EN)) == 0) {
			vaudio_mode(REGULATOR_MODE_NORMAL);
		}

		/*set EMU con mode*/
		if (cpcap_regacc_write(cpcap, CPCAP_REG_USBC2,
			CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE0,
			CPCAP_BIT_UARTMUX1 | CPCAP_BIT_UARTMUX0 |
			CPCAP_BIT_EMUMODE2 | CPCAP_BIT_EMUMODE1 |
			CPCAP_BIT_EMUMODE0)) {
			printk(KERN_INFO "%s: EMUMODE=101 failed\n", __func__);
			return -EIO;
		}

		/*last, enable EMU speaker outs (ie bias voltage)*/
		value |= (CPCAP_BIT_EMU_SPKR_R_EN | CPCAP_BIT_EMU_SPKR_L_EN);
		cpcap_audio_reg_write(codec, reg, value);
	} else {
		/*next, disable EMU speakers*/
		value &= ~(CPCAP_BIT_EMU_SPKR_R_EN | CPCAP_BIT_EMU_SPKR_L_EN);
		cpcap_audio_reg_write(codec, reg, value);

		/*finally, set audio low power if nothing else is going on*/
		if ((cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_SDAC)] &
				 CPCAP_BIT_ST_DAC_EN) == 0 &&
		    (cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC)] &
			(CPCAP_BIT_CDC_EN_RX | CPCAP_BIT_MIC1_CDC_EN |
					CPCAP_BIT_MIC2_CDC_EN)) == 0) {
			vaudio_mode(REGULATOR_MODE_STANDBY);
		}
	}
	return 0;
}

static int snd_soc_put_cpcap_mixer(struct snd_kcontrol *kcontrol,
				   struct snd_ctl_elem_value *ucontrol)
{
	int err;
	unsigned short val, value = 0;
	unsigned int reg = CPCAP_AUDIO_REG_NUM;
	struct soc_mixer_control *mc =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned int shift = mc->shift;
	unsigned short *cache = codec->reg_cache;
	char *name = kcontrol->id.name;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);
	int stdac_workaround_needed = 0;

	if (!state && cpcap_global_state_pointer) {
		state = cpcap_global_state_pointer;
		CPCAP_AUDIO_DEBUG_LOG("put_mixer using global codec %p"
			" instead of %p\n", state->codec, codec);
		codec = state->codec;
	}
	cache = codec->reg_cache;

	CPCAP_AUDIO_DEBUG_LOG("%s: kcontrol named %s\n", __func__, name);

	val = (ucontrol->value.integer.value[0] << shift);
	if (strcmp(name, "CPCAP Mixer Voice Codec") == 0) {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA);
		if (val)
			value = cache[reg] | CPCAP_BIT_PGA_CDC_EN;
		else
			value = cache[reg] & ~CPCAP_BIT_PGA_CDC_EN;
	} else if (strcmp(name, "CPCAP Mixer Stereo DAC") == 0) {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA);
		if (val) {
			if (!(cache[reg] & CPCAP_BIT_PGA_DAC_EN))
				stdac_workaround_needed = 1;
			else
				stdac_workaround_needed = 0;
			value = cache[reg] | CPCAP_BIT_PGA_DAC_EN;
		} else {
			value = cache[reg] & ~CPCAP_BIT_PGA_DAC_EN;
		}
	} else if (strcmp(name, "CPCAP Mixer External PGA") == 0) {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA);
		if (val) {
			value = cache[reg] |
				CPCAP_BIT_PGA_EXT_L_EN |
				CPCAP_BIT_PGA_EXT_R_EN;
		} else {
			value = cache[reg] & ~(CPCAP_BIT_PGA_EXT_L_EN |
					       CPCAP_BIT_PGA_EXT_R_EN);
		}
	}

	err = cpcap_audio_reg_write(codec, reg, value);
	if (err)
		return err;

	/* Workaround for ST TI DAC ramp time */
	if (stdac_workaround_needed &&
		(state->cpcap->vendor == CPCAP_VENDOR_ST)) {
		cpcap_regacc_write(state->cpcap, CPCAP_REG_TEST,
					STM_STDAC_EN_TEST_PRE, 0xFFFF);
		cpcap_regacc_write(state->cpcap, CPCAP_REG_ST_TEST1,
					STM_STDAC_EN_ST_TEST1_PRE, 0xFFFF);
	}

	reg = mc->reg;

	if (val)
		value = cache[reg] | val;
	else if (reg != CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA))
		value = cache[reg] & ~(1 << shift);
	else
		/* Disable left and right EXT PGA together */
		value = cache[reg] & ~(3 << shift);

	err |= cpcap_audio_reg_write(codec, reg, value);

	if (stdac_workaround_needed &&
		(state->cpcap->vendor == CPCAP_VENDOR_ST)) {
		msleep(STM_STDAC_ACTIVATE_RAMP_TIME);
		cpcap_regacc_write(state->cpcap, CPCAP_REG_ST_TEST1,
					STM_STDAC_EN_ST_TEST1_POST, 0xFFFF);
		cpcap_regacc_write(state->cpcap, CPCAP_REG_TEST,
					STM_STDAC_EN_TEST_POST, 0xFFFF);
	}

	return err;
}

static int cpcap_audio_power_event(struct snd_soc_dapm_widget *w,
				   struct snd_kcontrol *kcontrol, int event)
{
	unsigned short value = 0;
	unsigned int reg = CPCAP_AUDIO_REG_NUM;
	struct snd_soc_codec *codec = w->codec;
	unsigned short *cache = codec->reg_cache;

	CPCAP_AUDIO_DEBUG_LOG("%s: widget named %s\n", __func__, w->name);

	if (!strcmp(w->name, "CDC Playback")) {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA);
		if (SND_SOC_DAPM_EVENT_ON(event))
			value = cache[reg] | CPCAP_BIT_CDC_SW;
		else
			value = cache[reg] & ~CPCAP_BIT_CDC_SW;
	} else if (!strcmp(w->name, "DAC Playback")) {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA);
		if (SND_SOC_DAPM_EVENT_ON(event))
			value = cache[reg] | CPCAP_BIT_ST_DAC_SW;
		else
			value = cache[reg] & ~CPCAP_BIT_ST_DAC_SW;
	} else if (!strcmp(w->name, "EXT Playback Right")) {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA);
		if (SND_SOC_DAPM_EVENT_ON(event))
			value = cache[reg] | CPCAP_BIT_PGA_IN_R_SW;
		else
			value = cache[reg] & ~CPCAP_BIT_PGA_IN_R_SW;
	} else if (!strcmp(w->name, "EXT Playback Left")) {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXEPOA);
		if (SND_SOC_DAPM_EVENT_ON(event))
			value = cache[reg] | CPCAP_BIT_PGA_IN_L_SW;
		else
			value = cache[reg] & ~CPCAP_BIT_PGA_IN_L_SW;
	}

	return cpcap_audio_reg_write(codec, reg, value);
}

static int snd_soc_get_cpcap_gpio(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	unsigned short value, mask;
	unsigned int reg = (unsigned int)kcontrol->private_value;
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);
	struct cpcap_device *cpcap = state->cpcap;

	CPCAP_AUDIO_DEBUG_LOG("%s: %s read reg %u\n",
		__func__, kcontrol->id.name, reg);

	if (reg == CPCAP_REG_GPIO4)
		mask = CPCAP_BIT_GPIO4DRV;
	else {
		printk(KERN_INFO "Unsupported GPIO\n");
		return -EIO;
	}

	if (cpcap_regacc_read(cpcap, reg, &value)) {
		printk(KERN_INFO "%s: capcap read failed\n", __func__);
		return -EIO;
	}
	ucontrol->value.integer.value[0] = value & mask;

	return 0;
}

static int snd_soc_put_cpcap_gpio(struct snd_kcontrol *kcontrol,
				  struct snd_ctl_elem_value *ucontrol)
{
	int err;
	unsigned short value, mask, tmp;
	unsigned int reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI);
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);
	struct cpcap_device *cpcap = state->cpcap;
	unsigned short *cache = codec->reg_cache;

	CPCAP_AUDIO_DEBUG_LOG("%s: %s - %ld\n", __func__,
		kcontrol->id.name, ucontrol->value.integer.value[0]);

	value = cache[reg] | CPCAP_BIT_MB_ON1L;
	err = cpcap_audio_reg_write(codec, reg, value);
	if (err)
		return err;

	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC);
	value = cpcap_audio_reg_read(codec, reg) & (CPCAP_BIT_MIC2_CDC_EN |
						    CPCAP_BIT_MIC1_CDC_EN);
	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CDI);
	tmp = cache[reg] & ~(CPCAP_BIT_MIC1_RX_TIMESLOT2 |
				CPCAP_BIT_MIC1_RX_TIMESLOT1 |
				CPCAP_BIT_MIC1_RX_TIMESLOT0 |
				CPCAP_BIT_MIC2_TIMESLOT2 |
				CPCAP_BIT_MIC2_TIMESLOT1 |
				CPCAP_BIT_MIC2_TIMESLOT0);
	if (value == CPCAP_BIT_MIC2_CDC_EN) {
		/* If only MIC2 is enabled, then let it occupy slot 0.
		 * This is done by moving MIC1 to slot 1.
		 */
		value = tmp | CPCAP_BIT_MIC1_RX_TIMESLOT0;
	} else if (value == (CPCAP_BIT_MIC2_CDC_EN | CPCAP_BIT_MIC1_CDC_EN)) {
		/* Stereo capture. I2S for voice call so we ignore these bits */
		value = tmp | CPCAP_BIT_MIC1_RX_TIMESLOT0;
	} else {
		/* By default, MIC1 is on slot 0 and MIC2 is on slot 1 */
		value = tmp | CPCAP_BIT_MIC2_TIMESLOT0;
	}
	err = cpcap_audio_reg_write(codec, reg, value);
	if (err)
		return err;

	/* Reset clock tree */
	err = cpcap_audio_reg_write(codec, CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC),
		cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC)] |
			CPCAP_BIT_CDC_CLOCK_TREE_RESET |
			CPCAP_BIT_DF_RESET);
	if (err)
		return err;

	/* Wait for clock tree reset to complete */
	mdelay(CLOCK_TREE_RESET_TIME);
	if (cpcap_audio_reg_read(codec, CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC)) &
		(CPCAP_BIT_DF_RESET | CPCAP_BIT_CDC_CLOCK_TREE_RESET)) {
		printk(KERN_ERR "%s: CPCAP_REG_CC "
			"DF_RESET and CLOCK_TREE_RESET should have "
			"self-cleared\n", __func__);
	}

	reg = (unsigned int)kcontrol->private_value;
	if (reg == CPCAP_REG_GPIO4)
		mask = CPCAP_BIT_GPIO4DRV;
	else {
		printk(KERN_INFO "Unsupported GPIO\n");
		return -EIO;
	}

	if (ucontrol->value.integer.value[0] != 0)
		value = mask;
	else
		value = 0;
	if (cpcap_regacc_write(cpcap, reg, value, mask)) {
		printk(KERN_INFO "%s: cpcap write failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int snd_soc_get_cpcap_dai_mode(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int snd_soc_put_cpcap_dai_mode(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_value *ucontrol)
{
	int err;
	unsigned short v;
	unsigned short value = ucontrol->value.integer.value[0];
	unsigned int reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CDI);
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	unsigned short *cache = codec->reg_cache;

	CPCAP_AUDIO_DEBUG_LOG("%s: %s - %d\n",
			      __func__, kcontrol->id.name, value);

	switch (value) {
	case 1: /* cpcap_codec_op_modes_texts[1]: Voice Call Handset*/
		cpcap_audio_reg_write(codec,
			CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC),
			cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC)] | 0x0093);
		cpcap_audio_reg_write(codec,
			CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
			cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI)] | 0x0CC6);
		cpcap_audio_reg_write(codec,
			CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXMP),
			cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXMP)] | 0x0273);

	case 2: /* cpcap_codec_op_modes_texts[2]: Voice Call Headset*/
	case 3: /* cpcap_codec_op_modes_texts[3]: Voice Call Headset Mic*/
		v = cache[reg] | CPCAP_BIT_CLK_IN_SEL |
				 CPCAP_BIT_CDC_DIG_AUD_FS1 |
				 CPCAP_BIT_CDC_DIG_AUD_FS0 |
				 CPCAP_BIT_CLK_INV;
		err = cpcap_audio_reg_write(codec, reg, v);
		if (err)
			return err;
		break;
	case 4: /* cpcap_codec_op_modes_texts[4]: Voice Call BT*/
		v = cache[reg] | CPCAP_BIT_CLK_IN_SEL |
				 CPCAP_BIT_MIC2_TIMESLOT0;
		err = cpcap_audio_reg_write(codec, reg, v);
		if (err)
			return err;
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC);
		break;
	default:
		return -EINVAL; /* invalid mode */
	}

	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC);
	v = cache[reg] & ~(CPCAP_BIT_CDC_CLK2 |
			   CPCAP_BIT_CDC_CLK1 |
			   CPCAP_BIT_CDC_CLK0);
	v |= CPCAP_BIT_CDC_CLK1 | CPCAP_BIT_CDC_CLK0 |
	     CPCAP_BIT_CDC_CLOCK_TREE_RESET |
	     CPCAP_BIT_DF_RESET;
	err = cpcap_audio_reg_write(codec, reg, v);
	if (err)
		return err;
	/* Wait for clock tree reset to complete */
	mdelay(CLOCK_TREE_RESET_TIME);
	v = cpcap_audio_reg_read(codec, reg);
	if (v & (CPCAP_BIT_DF_RESET | CPCAP_BIT_CDC_CLOCK_TREE_RESET)) {
		printk(KERN_ERR "%s: CPCAP_REG_CC = %u! "
			"DF_RESET and CLOCK_TREE_RESET should have "
			"self-cleared\n", __func__, value);
	}

	return 0;
}

void cpcap_audio_init(struct snd_soc_codec *codec)
{
	int i;
	struct cpcap_device *cpcap;
	unsigned short *cache = codec->reg_cache;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s() called\n", __func__);

	emu_analog_antipop = 0;

	cpcap = state->cpcap;
	for (i = 0; i < CPCAP_AUDIO_REG_NUM; i++)
		cpcap_audio_reg_read(codec, i);
	cpcap_audio_reg_write(codec, 1, 0);
	cpcap_audio_reg_write(codec, 2, 0);
	cpcap_audio_reg_write(codec, 3, 0);
	cpcap_audio_reg_write(codec, 4, 4);
	cpcap_audio_reg_write(codec, 5, 0);
	cpcap_audio_reg_write(codec, 6, 0x0400);
	cpcap_audio_reg_write(codec, 7, 0);
	cpcap_audio_reg_write(codec, 9, 0);
	cpcap_audio_reg_write(codec, 10, 0);
	cpcap_audio_reg_write(codec, 11, 0);
	cpcap_audio_reg_write(codec, 13, cache[13] | CPCAP_BIT_A2_FREE_RUN);

	/* This is not an audio register, go through cpcap api directly */
	cpcap_regacc_write(cpcap, CPCAP_REG_GPIO4,
			   CPCAP_BIT_GPIO4DIR, CPCAP_BIT_GPIO4DIR);
	vaudio_get();
}

static void audio_callback(struct cpcap_device *cpcap, int status)
{
	unsigned short *cache;
	int i = 0;
	unsigned short HSstate = 0;
	struct cpcap_audio_state *curr_state = cpcap->h2w_new_state_data;
	struct snd_soc_codec *codec = curr_state->codec;

	CPCAP_AUDIO_DEBUG_LOG("%s: Entered\n", __func__);

	if (!codec) {
		printk(KERN_ERR "%s: invalid pointer\n", __func__);
		return;
	} else
		cache = codec->reg_cache;

	/* HS insertion status = 1, HS detach status = 0 */
	if (status == 0 || status == 1) {
		if (cache[5] & CPCAP_BIT_HS_MIC_MUX)
			HSstate |= CPCAP_BIT_HS_MIC_MUX;
		if (cache[7] & (CPCAP_BIT_HS_L_EN | CPCAP_BIT_HS_R_EN)) {
			HSstate |= cache[7] & (CPCAP_BIT_HS_L_EN |
					       CPCAP_BIT_HS_R_EN);
		}
	}

	for (i = 0; i < CPCAP_AUDIO_REG_NUM; i++)
		cpcap_audio_reg_read(codec, i);

	if (HSstate & CPCAP_BIT_HS_MIC_MUX) {
		cpcap_audio_reg_write(codec, 5,
				      cache[5] | CPCAP_BIT_HS_MIC_MUX);
		HSstate &= ~CPCAP_BIT_HS_MIC_MUX;
	}
	if (HSstate & (CPCAP_BIT_HS_L_EN | CPCAP_BIT_HS_R_EN))
		cpcap_audio_reg_write(codec, 7, cache[7] | HSstate);
}

static int cpcap_add_widgets(struct snd_soc_codec *codec)
{
	CPCAP_AUDIO_DEBUG_LOG("%s: Entered\n", __func__);

	snd_soc_dapm_new_controls(&codec->dapm, cpcap_dapm_widgets,
				  ARRAY_SIZE(cpcap_dapm_widgets));
	snd_soc_dapm_add_routes(&codec->dapm, intercon, ARRAY_SIZE(intercon));
	snd_soc_dapm_new_widgets(&codec->dapm);

	snd_soc_dapm_ignore_suspend(&codec->dapm, "MIC1R");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "MICHS");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "MICEMU");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "MICEXTR");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "MIC2L");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "MICEXTL");

	snd_soc_dapm_ignore_suspend(&codec->dapm, "LDSPR");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "LDSPL");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "HSR");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "HSL");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "LINER");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "LINEL");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "EMUSPKR");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "EMUSPKL");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "EP");

	snd_soc_dapm_ignore_suspend(&codec->dapm, "ADC Right");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "ADC Left");

	snd_soc_dapm_ignore_suspend(&codec->dapm, "AIFIN Voice");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "AIFIN Multimedia");
	snd_soc_dapm_ignore_suspend(&codec->dapm, "AIFIN ExternalPGA");

	return 0;
}

static int cpcap_mm_startup(struct snd_pcm_substream *substream,
			 struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s: Entered, %d stdac streams\n",
			      __func__, state->stdac_strm_cnt);

	if (state->stdac_strm_cnt == 0 &&
	    state->codec_strm_cnt == 0 &&
	    emu_analog_antipop == 0) {
		if (vaudio_mode(REGULATOR_MODE_NORMAL) != 0)
			return -EINVAL;
	}
	state->stdac_strm_cnt++;

	return 0;
}

static void cpcap_mm_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s: Entered, %d stdac streams\n",
			      __func__, state->stdac_strm_cnt);

	if (state->stdac_strm_cnt <= 0) {
		printk(KERN_INFO "%s: no stream to shutdown\n", __func__);
		return;
	}

	state->stdac_strm_cnt--;
	if (state->stdac_strm_cnt == 0) {
		cpcap_audio_reg_write(codec, 3, 0);
		cpcap_audio_reg_write(codec, 4, 4);
		cpcap_audio_reg_write(codec, 10, 0);
		if (state->codec_strm_cnt == 0) {
			if (emu_analog_antipop == 0) {
				cpcap_audio_reg_write(codec, 7, 0);
				if (vaudio_mode(REGULATOR_MODE_STANDBY) != 0)
					return;
			}
		}
	}
	cpcap_audio_register_dump(codec);
}

static int cpcap_mm_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params,
			      struct snd_soc_dai *dai)
{
	int ret, rate;
	unsigned int reg;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = dai->codec;

	CPCAP_AUDIO_DEBUG_LOG("%s entered\n", __func__);

	cache = (unsigned short *)codec->reg_cache;
	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_SDAC);
	value = cache[reg] &
		~(CPCAP_BIT_ST_SR3 | CPCAP_BIT_ST_SR2 |
		  CPCAP_BIT_ST_SR1 | CPCAP_BIT_ST_SR0);
	rate = params_rate(params);
	switch (rate) {
	case 48000:
		value |= CPCAP_BIT_ST_SR3;
		break;
	case 44100:
		value |= CPCAP_BIT_ST_SR2 |
			 CPCAP_BIT_ST_SR1 |
			 CPCAP_BIT_ST_SR0;
		break;
	case 32000:
		value |= CPCAP_BIT_ST_SR2 |
			 CPCAP_BIT_ST_SR1;
		break;
	case 24000:
		value |= CPCAP_BIT_ST_SR2 |
			 CPCAP_BIT_ST_SR0;
		break;
	case 22050:
		value |= CPCAP_BIT_ST_SR2;
		break;
	case 16000:
		value |= CPCAP_BIT_ST_SR1 |
			 CPCAP_BIT_ST_SR0;
		break;
	case 12000:
		value |= CPCAP_BIT_ST_SR1;
		break;
	case 11025:
		value |= CPCAP_BIT_ST_SR0;
		break;
	case 8000:
		break;
	default:
		printk(KERN_ERR "%s: invalid sample rate %d\n", __func__, rate);
		value |= CPCAP_BIT_ST_SR2 |
			 CPCAP_BIT_ST_SR1 |
			 CPCAP_BIT_ST_SR0;  /* default 44100Hz */
		break;
	}
	value |= CPCAP_BIT_DF_RESET_ST_DAC |
		 CPCAP_BIT_ST_CLOCK_TREE_RESET;
	ret = cpcap_audio_reg_write(codec, reg, value);
	if (ret)
		return ret;
	/* Wait for clock tree reset to complete */
	mdelay(CLOCK_TREE_RESET_TIME);
	value = cpcap_audio_reg_read(codec, reg);
	if (value &
	    (CPCAP_BIT_DF_RESET_ST_DAC | CPCAP_BIT_ST_CLOCK_TREE_RESET)) {
		printk(KERN_ERR "%s: CPCAP_REG_SDAC = %u! "
			"DF_RESET and CLOCK_TREE_RESET should have "
			"self-cleared\n", __func__, value);
	}

	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_SDACDI);
	value = cache[reg] | CPCAP_BIT_ST_CLK_EN;
	ret = cpcap_audio_reg_write(codec, reg, value);
	if (ret)
		return ret;

	cpcap_audio_register_dump(codec);
	return 0;
}

static int cpcap_mm_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	int ret;
	unsigned int reg;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = codec_dai->codec;

	CPCAP_AUDIO_DEBUG_LOG("%s(%d, %u, %d) entered\n",
			      __func__, clk_id, freq, dir);

	cache = (unsigned short *)codec->reg_cache;
	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_SDACDI);
	value = cache[reg];
	switch (clk_id) {
	case 0:
		value &= ~CPCAP_BIT_ST_DAC_CLK_IN_SEL;
		break;
	case 1:
		value |= CPCAP_BIT_ST_DAC_CLK_IN_SEL;
		break;
	default:
		printk(KERN_ERR "%s: wrong clk_id %d\n", __func__, clk_id);
		value &= ~CPCAP_BIT_ST_DAC_CLK_IN_SEL; /* default to CLK_IN0 */
		break;
	}
	ret = cpcap_audio_reg_write(codec, reg, value);
	if (ret)
		return ret;

	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_SDAC);
	value = cache[reg] & ~(CPCAP_BIT_ST_DAC_CLK2 |
			       CPCAP_BIT_ST_DAC_CLK1 |
			       CPCAP_BIT_ST_DAC_CLK0);
	switch (freq) {
	case 15360000:
		value |= CPCAP_BIT_ST_DAC_CLK0;
		break;
	case 16800000:
		value |= CPCAP_BIT_ST_DAC_CLK1;
		break;
	case 19200000:
		value |= CPCAP_BIT_ST_DAC_CLK1 |
			 CPCAP_BIT_ST_DAC_CLK0;
		break;
	case 26000000:
		value |= CPCAP_BIT_ST_DAC_CLK2;
		break;
	case 33600000:
		value |= CPCAP_BIT_ST_DAC_CLK2 |
			 CPCAP_BIT_ST_DAC_CLK0;
		break;
	case 38400000:
		value |= CPCAP_BIT_ST_DAC_CLK2 |
			 CPCAP_BIT_ST_DAC_CLK1;
		break;
	default:
		printk(KERN_ERR "%s: wrong freq %u\n", __func__, freq);
		value |= CPCAP_BIT_ST_DAC_CLK2; /* default to 26MHz */
		break;
	}

	return cpcap_audio_reg_write(codec, reg, value);
}

static int cpcap_mm_set_dai_fmt(struct snd_soc_dai *codec_dai,
			     unsigned int fmt)
{
	unsigned int reg;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = codec_dai->codec;

	CPCAP_AUDIO_DEBUG_LOG("%s(%u) entered\n", __func__, fmt);

	/*
	 * cpcap_dai[0] for "HiFi Playback" is always configured as
	 * SND_SOC_DAIFMT_CBM_CFM - codec clk & frm master
	 * SND_SOC_DAIFMT_I2S - I2S mode
	 * ignore the fmt passed in
	 */
	/*
	 * TODO: missing interface for controling
	 * CPCAP_BIT_ST_FS_INV, CPCAP_BIT_ST_CLK_INV
	 */
	cache = (unsigned short *)codec->reg_cache;
	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_SDACDI);
	value = cache[reg] & ~(CPCAP_BIT_ST_DIG_AUD_FS0 |
			       CPCAP_BIT_ST_DIG_AUD_FS1);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		value &= ~CPCAP_BIT_SMB_ST_DAC;
		break;
	default:
		printk(KERN_ERR "%s: CPCAP should always be the master\n",
			__func__);
		/* TODO: need to support slave mode? */
		value &= ~CPCAP_BIT_SMB_ST_DAC;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		value |= CPCAP_BIT_ST_CLK_INV |
			 CPCAP_BIT_ST_FS_INV;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		value |= CPCAP_BIT_ST_CLK_INV;
		value &= ~CPCAP_BIT_ST_FS_INV;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		value |= CPCAP_BIT_ST_FS_INV;
		value &= ~CPCAP_BIT_ST_CLK_INV;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		value &= ~(CPCAP_BIT_ST_CLK_INV |
			   CPCAP_BIT_ST_FS_INV);
		break;
	default:
		printk(KERN_ERR "%s: unsupported clock invert mode\n",
			__func__);
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		value |= CPCAP_BIT_ST_DIG_AUD_FS0 | /* 11 - true I2S mode */
			 CPCAP_BIT_ST_DIG_AUD_FS1;
		break;
	default:
		value |= CPCAP_BIT_ST_DIG_AUD_FS0 | /* 4 slots network mode */
			 CPCAP_BIT_ST_L_TIMESLOT0; /* L on slot 1 */
		break;
	}

	return cpcap_audio_reg_write(codec, reg, value);
}

static int cpcap_mm_mute(struct snd_soc_dai *dai, int mute)
{
	unsigned int reg;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = dai->codec;

	CPCAP_AUDIO_DEBUG_LOG("%s(%d) entered\n", __func__, mute);

	cache = (unsigned short *)codec->reg_cache;
	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXSDOA);
	if (mute)
		value = cache[reg] & ~CPCAP_BIT_ST_DAC_SW;
	else
		value = cache[reg] | CPCAP_BIT_ST_DAC_SW;

	return cpcap_audio_reg_write(codec, reg, value);
}

static int cpcap_voice_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s: Entered, %d codec streams\n",
			      __func__, state->codec_strm_cnt);

	if (state->codec_strm_cnt == 0 &&
	    state->stdac_strm_cnt == 0 &&
	    emu_analog_antipop == 0) {
		if (vaudio_mode(REGULATOR_MODE_NORMAL) != 0)
			return -EINVAL;
	}
	state->codec_strm_cnt++;

	return 0;
}

static void cpcap_voice_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);
	unsigned short *cache = (unsigned short *)codec->reg_cache;

	CPCAP_AUDIO_DEBUG_LOG("%s: Entered, %d codec streams\n",
			      __func__, state->codec_strm_cnt);

	if (state->codec_strm_cnt <= 0) {
		printk(KERN_INFO "%s: no stream to shutdown\n", __func__);
		return;
	}

	state->codec_strm_cnt--;
	if (state->codec_strm_cnt == 0) {
		cpcap_audio_reg_write(codec,
				CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC),
				cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC)] &
				(CPCAP_BIT_MIC1_CDC_EN |
				CPCAP_BIT_MIC2_CDC_EN));
		cpcap_audio_reg_write(codec,
				CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CDI), 0);
		cpcap_audio_reg_write(codec,
				CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI),
				cache[CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI)] &
				(CPCAP_BIT_MIC1_PGA_EN | CPCAP_BIT_MIC1_MUX |
				CPCAP_BIT_HS_MIC_MUX | CPCAP_BIT_EMU_MIC_MUX |
				CPCAP_BIT_MIC2_PGA_EN | CPCAP_BIT_MIC2_MUX));
		cpcap_audio_reg_write(codec,
				CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA), 0);
		if (state->stdac_strm_cnt == 0) {
			/* BT streams operate with CPCAP in standby mode, so
			 * don't put CPCAP in standby again when we close
			 * BT streams.
			 */
			if (!strstr(dai->name, "bt") &&
			    emu_analog_antipop == 0)
				if (vaudio_mode(REGULATOR_MODE_STANDBY) != 0)
					return;
		}
	}
	cpcap_audio_register_dump(codec);
}

static int cpcap_voice_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int ret, rate;
	unsigned int reg;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s entered, %d codec streams\n",
			      __func__, state->codec_strm_cnt);

	cache = (unsigned short *)codec->reg_cache;
	if (state->codec_strm_cnt > 1) /* skip common settings */
		goto capture_params;

	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC);
	value = cache[reg] &
		~(CPCAP_BIT_CDC_SR3 | CPCAP_BIT_CDC_SR2 |
		  CPCAP_BIT_CDC_SR1 | CPCAP_BIT_CDC_SR0);
	rate = params_rate(params);
	switch (rate) {
	case 48000:
		value |= CPCAP_BIT_CDC_SR3;
		break;
	case 44100:
		value |= CPCAP_BIT_CDC_SR2 |
			 CPCAP_BIT_CDC_SR1 |
			 CPCAP_BIT_CDC_SR0;
		break;
	case 32000:
		value |= CPCAP_BIT_CDC_SR2 |
			 CPCAP_BIT_CDC_SR1;
		break;
	case 24000:
		value |= CPCAP_BIT_CDC_SR2 |
			 CPCAP_BIT_CDC_SR0;
		break;
	case 22050:
		value |= CPCAP_BIT_CDC_SR2;
		break;
	case 16000:
		value |= CPCAP_BIT_CDC_SR1 |
			 CPCAP_BIT_CDC_SR0;
		break;
	case 12000:
		value |= CPCAP_BIT_CDC_SR1;
		break;
	case 11025:
		value |= CPCAP_BIT_CDC_SR0;
		break;
	case 8000:
		break;
	default:
		printk(KERN_ERR "%s: invalid sample rate %d,"
		       " set to 8KHz\n", __func__, rate);
		break;
	}
	value |= CPCAP_BIT_DF_RESET |
		 CPCAP_BIT_CDC_CLOCK_TREE_RESET;
	ret = cpcap_audio_reg_write(codec, reg, value);
	if (ret)
		return ret;
	/* Wait for clock tree reset to complete */
	mdelay(CLOCK_TREE_RESET_TIME);
	value = cpcap_audio_reg_read(codec, reg);
	if (value &
	    (CPCAP_BIT_DF_RESET | CPCAP_BIT_CDC_CLOCK_TREE_RESET)) {
		printk(KERN_ERR "%s: CPCAP_REG_CC = %u! "
			"DF_RESET and CLOCK_TREE_RESET should have "
			"self-cleared\n", __func__, value);
	}

capture_params:
	if (substream->stream) {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC);
		value = cache[reg] | CPCAP_BIT_AUDIHPF_0 |
				     CPCAP_BIT_AUDIHPF_1;
		ret = cpcap_audio_reg_write(codec, reg, value);
		if (ret)
			return ret;
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_TXI);
		value = cache[reg] | CPCAP_BIT_MB_ON1R;
		if (params_channels(params) >= 2)
			value |= CPCAP_BIT_MB_ON1L;
		ret = cpcap_audio_reg_write(codec, reg, value);
		if (ret)
			return ret;
	} else {
		reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC);
		value = cache[reg] | CPCAP_BIT_AUDOHPF_0 |
				     CPCAP_BIT_AUDOHPF_1;
		ret = cpcap_audio_reg_write(codec, reg, value);
		if (ret)
			return ret;
	}

	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CDI);
	value = cache[reg];
	if (substream->stream) {
		value &= ~(CPCAP_BIT_MIC1_RX_TIMESLOT2 |
			   CPCAP_BIT_MIC1_RX_TIMESLOT1 |
			   CPCAP_BIT_MIC1_RX_TIMESLOT0 |
			   CPCAP_BIT_MIC2_TIMESLOT2 |
			   CPCAP_BIT_MIC2_TIMESLOT1 |
			   CPCAP_BIT_MIC2_TIMESLOT0);
		if (params_channels(params) >= 2)
			value |= CPCAP_BIT_MIC1_RX_TIMESLOT0;
	}
	value |= CPCAP_BIT_CDC_CLK_EN;
	ret = cpcap_audio_reg_write(codec, reg, value);
	if (ret)
		return ret;

	cpcap_audio_register_dump(codec);
	return 0;
}

static int cpcap_voice_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	int ret;
	unsigned int reg;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s(%d, %u, %d) entered, %d codec streams\n",
			      __func__, clk_id, freq, dir,
			      state->codec_strm_cnt);

	if (state->codec_strm_cnt > 1) /* stay with current settings */
		return 0;

	cache = (unsigned short *)codec->reg_cache;
	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CDI);
	/* Codec uses independent PLL in normal operation */
	value = cache[reg] | CPCAP_BIT_CDC_PLL_SEL;
	switch (clk_id) {
	case 0:
		value &= ~CPCAP_BIT_CLK_IN_SEL;
		break;
	case 1:
		value |= CPCAP_BIT_CLK_IN_SEL;
		break;
	default:
		printk(KERN_ERR "%s: wrong clik_id %d\n", __func__, clk_id);
		value &= ~CPCAP_BIT_CLK_IN_SEL; /* default to CLK_IN0 */
		break;
	}
	ret = cpcap_audio_reg_write(codec, reg, value);
	if (ret)
		return ret;

	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CC);
	value = cache[reg] & ~(CPCAP_BIT_CDC_CLK2 |
			       CPCAP_BIT_CDC_CLK1 |
			       CPCAP_BIT_CDC_CLK0);
	switch (freq) {
	case 15360000:
		value |= CPCAP_BIT_CDC_CLK0;
		break;
	case 16800000:
		value |= CPCAP_BIT_CDC_CLK1;
		break;
	case 19200000:
		value |= CPCAP_BIT_CDC_CLK1 |
			 CPCAP_BIT_CDC_CLK0;
		break;
	case 26000000:
		value |= CPCAP_BIT_CDC_CLK2;
		break;
	case 33600000:
		value |= CPCAP_BIT_CDC_CLK2 |
			 CPCAP_BIT_CDC_CLK0;
		break;
	case 38400000:
		value |= CPCAP_BIT_CDC_CLK2 |
			 CPCAP_BIT_CDC_CLK1;
		break;
	default:
		printk(KERN_ERR "%s: wrong freq %u\n", __func__, freq);
		value |= CPCAP_BIT_CDC_CLK2; /* default to 26MHz */
		break;
	}

	return cpcap_audio_reg_write(codec, reg, value);
}

static int cpcap_voice_set_dai_fmt(struct snd_soc_dai *codec_dai,
				   unsigned int fmt)
{
	unsigned int reg;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s(%u) entered, %d codec streams\n",
			      __func__, fmt, state->codec_strm_cnt);

	if (state->codec_strm_cnt > 1) /* stay with current settings */
		return 0;

	/*
	 * cpcap_dai[1] for "Voice Playback" and "Capture"
	 * is always configured as
	 * SND_SOC_DAIFMT_CBM_CFM - codec clk & frm master
	 */
	cache = (unsigned short *)codec->reg_cache;
	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_CDI);
	value = cache[reg] & ~(CPCAP_BIT_DIG_AUD_IN | /* select DAI0 - McBSP3 */
			       CPCAP_BIT_CDC_DIG_AUD_FS0 |
			       CPCAP_BIT_CDC_DIG_AUD_FS1);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		value &= ~CPCAP_BIT_SMB_CDC;
		break;
	default:
		printk(KERN_ERR "%s: CPCAP should always be the master\n",
			__func__);
		/* TODO: need to support slave mode? */
		value &= ~CPCAP_BIT_SMB_CDC;
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		value |= CPCAP_BIT_CLK_INV |
			 CPCAP_BIT_FS_INV;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		value |= CPCAP_BIT_CLK_INV;
		value &= ~CPCAP_BIT_FS_INV;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		value |= CPCAP_BIT_FS_INV;
		value &= ~CPCAP_BIT_CLK_INV;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		value &= ~(CPCAP_BIT_CLK_INV |
			   CPCAP_BIT_FS_INV);
		break;
	default:
		printk(KERN_ERR "%s: unsupported clock invert mode\n",
			__func__);
		break;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		value |= CPCAP_BIT_CDC_DIG_AUD_FS0 |
			 CPCAP_BIT_CDC_DIG_AUD_FS1;
		break;
	default:
		/* 4 timeslots network mode */
		value |= CPCAP_BIT_CDC_DIG_AUD_FS0;
		break;
	}

	return cpcap_audio_reg_write(codec, reg, value);
}

static int cpcap_voice_mute(struct snd_soc_dai *dai, int mute)
{
	unsigned int reg;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = dai->codec;

	CPCAP_AUDIO_DEBUG_LOG("%s(%d) entered\n", __func__, mute);

	cache = (unsigned short *)codec->reg_cache;
	reg = CPCAP_AUDIO_REG_INDEX(CPCAP_REG_RXCOA);
	if (mute)
		value = cache[reg] & ~CPCAP_BIT_CDC_SW;
	else
		value = cache[reg] | CPCAP_BIT_CDC_SW;

	return cpcap_audio_reg_write(codec, reg, value);
}

static int cpcap_incall_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int ret;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s entered, %d codec streams\n",
			      __func__, state->codec_strm_cnt);

	cache = (unsigned short *)codec->reg_cache;
	if (state->codec_strm_cnt == 1) {
		struct platform_device *pdev = container_of(codec->dev,
				struct platform_device, dev);
		struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
		int rate = params_rate(params);

		if (pdata->voice_type == VOICE_TYPE_STE) {
			/* STE_M570 */
			ret = cpcap_audio_reg_write(codec, 2, 0xAE06);
			if (rate == 16000)
				ret |= cpcap_audio_reg_write(codec, 1, 0x8720);
			else
				ret |= cpcap_audio_reg_write(codec, 1, 0x8120);
		} else if (pdata->voice_type == VOICE_TYPE_QC) {
			/* MDM6600 */
			ret = cpcap_audio_reg_write(codec, 2, 0xAE02);
			if (rate == 16000) {
				ret |= cpcap_audio_reg_write(codec, 1, 0x6720);
			} else {
				ret |= cpcap_audio_reg_write(codec, 1, 0x6120);
			}
		} else {
			ret = -EIO;
			printk(KERN_ERR "%s: voice_type = %u, not valid modem",
				__func__, pdata->voice_type);
		}
		if (ret)
			return ret;
		/* Wait for clock tree reset to complete */
		mdelay(CLOCK_TREE_RESET_TIME);
		value = cpcap_audio_reg_read(codec, 1);
		if (value & (CPCAP_BIT_DF_RESET |
			     CPCAP_BIT_CDC_CLOCK_TREE_RESET)) {
			printk(KERN_ERR "%s: CPCAP_REG_CC = %u! "
				"DF_RESET and CLOCK_TREE_RESET should have "
				"self-cleared\n", __func__, value);
		}
	}
	if (substream->stream) { /* up link */
		ret = cpcap_audio_reg_write(codec, 1,
			cache[1] | CPCAP_BIT_AUDIHPF_1 |
				   CPCAP_BIT_AUDIHPF_0);
		ret |= cpcap_audio_reg_write(codec, 5,
			cache[5] | CPCAP_BIT_MB_ON1L |
				   CPCAP_BIT_MB_ON1R);
		if (ret)
			return ret;
	} else { /* down link */
		ret = cpcap_audio_reg_write(codec, 1,
			cache[1] | CPCAP_BIT_AUDOHPF_1 |
				   CPCAP_BIT_AUDOHPF_0);
		if (ret)
			return ret;
	}

	return 0;
}

static int cpcap_btcall_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int ret;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s entered, %d codec streams\n",
				__func__, state->codec_strm_cnt);

	cache = (unsigned short *)codec->reg_cache;
	if (state->codec_strm_cnt == 1) {
		struct platform_device *pdev = container_of(codec->dev,
				struct platform_device, dev);
		struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
		if (pdata->voice_type == VOICE_TYPE_STE) {
			/* STE_M570 */
			ret = cpcap_audio_reg_write(codec, 2, 0xAE06);
			ret |= cpcap_audio_reg_write(codec, 1, 0x8000);
		} else if (pdata->voice_type == VOICE_TYPE_QC) {
			/* MDM6600 */
			ret = cpcap_audio_reg_write(codec, 2, 0xAA40);
			ret |= cpcap_audio_reg_write(codec, 1, 0x6000);
		} else {
			ret = -EIO;
			printk(KERN_ERR "%s: voice_type = %u, not valid modem",
				__func__, pdata->voice_type);
		}
		if (ret)
			return ret;
		/* Wait for clock tree reset to complete */
		mdelay(CLOCK_TREE_RESET_TIME);
		value = cpcap_audio_reg_read(codec, 1);
		if (value & (CPCAP_BIT_DF_RESET |
			     CPCAP_BIT_CDC_CLOCK_TREE_RESET)) {
			printk(KERN_ERR "%s: CPCAP_REG_CC = %u! "
				"DF_RESET and CLOCK_TREE_RESET should have "
				"self-cleared\n", __func__, value);
		}
		/* Clocks can still be generated in low power mode */
		if (emu_analog_antipop == 0)
			vaudio_mode(REGULATOR_MODE_STANDBY);
	}

	return 0;
}

static int cpcap_btvoice_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	int ret;
	unsigned short value;
	unsigned short *cache;
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s entered, %d codec streams\n",
				__func__, state->codec_strm_cnt);

	cache = (unsigned short *)codec->reg_cache;
	if (state->codec_strm_cnt == 1) {
		struct platform_device *pdev = container_of(codec->dev,
				struct platform_device, dev);
		struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
		if (pdata->voice_type == VOICE_TYPE_STE) {
			/* STE_M570 */
			ret = cpcap_audio_reg_write(codec, 2, 0x8E06);
			ret |= cpcap_audio_reg_write(codec, 1, 0x8000);
		} else if (pdata->voice_type == VOICE_TYPE_QC) {
			/* MDM6600 */
			ret = cpcap_audio_reg_write(codec, 2, 0x8A40);
			ret |= cpcap_audio_reg_write(codec, 1, 0x8000);
		} else {
			ret = -EIO;
			printk(KERN_ERR "%s: voice_type = %u, not valid modem",
				__func__, pdata->voice_type);
		}
		if (ret)
			return ret;
		/* Wait for clock tree reset to complete */
		mdelay(CLOCK_TREE_RESET_TIME);
		value = cpcap_audio_reg_read(codec, 1);
		if (value & (CPCAP_BIT_DF_RESET |
			     CPCAP_BIT_CDC_CLOCK_TREE_RESET)) {
			printk(KERN_ERR "%s: CPCAP_REG_CC = %u! "
				"DF_RESET and CLOCK_TREE_RESET should have "
				"self-cleared\n", __func__, value);
		}
		/* Clocks can still be generated in low power mode */
		if (emu_analog_antipop == 0)
			vaudio_mode(REGULATOR_MODE_STANDBY);
	}

	return 0;
}

static int cpcap_fm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cpcap_audio_state *state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("%s entered, %d stdac streams\n",
			      __func__, state->stdac_strm_cnt);

	if (state->stdac_strm_cnt != 1) {
		printk(KERN_INFO "%s:WARNING: %d stdac stream\n",
			__func__, state->stdac_strm_cnt);
	}

	return 0;
}

static struct snd_soc_dai_ops cpcap_dai_mm_ops = {
	.startup        = cpcap_mm_startup,
	.shutdown       = cpcap_mm_shutdown,
	.hw_params      = cpcap_mm_hw_params,
	.set_sysclk     = cpcap_mm_set_dai_sysclk,
	.set_fmt        = cpcap_mm_set_dai_fmt,
	.digital_mute   = cpcap_mm_mute,
};

static struct snd_soc_dai_ops cpcap_dai_voice_ops = {
	.startup        = cpcap_voice_startup,
	.shutdown       = cpcap_voice_shutdown,
	.hw_params      = cpcap_voice_hw_params,
	.set_sysclk     = cpcap_voice_set_dai_sysclk,
	.set_fmt        = cpcap_voice_set_dai_fmt,
	.digital_mute   = cpcap_voice_mute,
};

static struct snd_soc_dai_ops cpcap_dai_incall_ops = {
	.startup        = cpcap_voice_startup,
	.shutdown       = cpcap_voice_shutdown,
	.hw_params      = cpcap_incall_hw_params,
	.digital_mute   = cpcap_voice_mute,
};

static struct snd_soc_dai_ops cpcap_dai_btcall_ops = {
	.startup        = cpcap_voice_startup,
	.shutdown       = cpcap_voice_shutdown,
	.hw_params      = cpcap_btcall_hw_params,
};

static struct snd_soc_dai_ops cpcap_dai_btvoice_ops = {
	.startup        = cpcap_voice_startup,
	.shutdown       = cpcap_voice_shutdown,
	.hw_params      = cpcap_btvoice_hw_params,
};

static struct snd_soc_dai_ops cpcap_dai_fm_ops = {
	.startup        = cpcap_mm_startup,
	.shutdown       = cpcap_mm_shutdown,
	.hw_params      = cpcap_fm_hw_params,
};

struct snd_soc_dai_driver cpcap_dai[] = {
{
	.name = "cpcap stdac",
	.playback = {
		.stream_name = "HiFi Playback",
		.channels_min = 2,
		.channels_max = 4,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FORMAT_S24_LE,},
	.ops = &cpcap_dai_mm_ops,
},
{
	.name = "cpcap codec",
	.playback = {
		.stream_name = "Voice Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &cpcap_dai_voice_ops,
},
{
	.name = "cpcap in-call",
	.playback = {
		.stream_name = "InCall DL",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &cpcap_dai_incall_ops,
},
{
	.name = "cpcap bt-call",
	.playback = {
		.stream_name = "BTCall DL",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "BTCall UL",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &cpcap_dai_btcall_ops,
},
{
	.name = "cpcap bt",
	.playback = {
		.stream_name = "BT Voice Playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "BT Capture",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &cpcap_dai_btvoice_ops,
},
{
	.name = "BPVoice",
	.playback = {
		.stream_name = "incall-playback",
		.channels_min = 1,
		.channels_max = 1,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.capture = {
		.stream_name = "incall-capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
},
{
	.name = "cpcap fm",
	.playback = {
		.stream_name = "FMAudio Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
	.ops = &cpcap_dai_fm_ops,
},
};

static int cpcap_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	CPCAP_AUDIO_DEBUG_LOG("%s: Entered\n", __func__);
	return 0;
}

static int cpcap_resume(struct snd_soc_codec *codec)
{
	CPCAP_AUDIO_DEBUG_LOG("%s: Entered\n", __func__);
	return 0;
}

static int cpcap_probe(struct snd_soc_codec *codec)
{
	struct platform_device *pdev;
	struct cpcap_audio_state *curr_state;

	curr_state = kzalloc(sizeof(struct cpcap_audio_state), GFP_KERNEL);
	if (!curr_state) {
		printk(KERN_ERR "Failed to allocate cpcap_audio_state\n");
		return -ENOMEM;
	}
	cpcap_global_state_pointer = curr_state;

	pdev = container_of(codec->dev, struct platform_device, dev);
	curr_state->cpcap = platform_get_drvdata(pdev);
	if (curr_state->cpcap) {
		curr_state->cpcap->h2w_new_state = &audio_callback;
		curr_state->cpcap->h2w_new_state_data = curr_state;
	} else {
		printk(KERN_ERR "invalid cpcap_audio platform device");
		kfree(curr_state);
		return -ENODEV;
	}
	curr_state->codec = codec;
	snd_soc_codec_set_drvdata(codec, curr_state);
	cpcap_audio_init(codec);
	snd_soc_add_controls(codec, cpcap_snd_controls,
			     ARRAY_SIZE(cpcap_snd_controls));
	cpcap_add_widgets(codec);

	return 0;
}

static int cpcap_remove(struct snd_soc_codec *codec)
{
	struct cpcap_audio_state *curr_state = snd_soc_codec_get_drvdata(codec);

	CPCAP_AUDIO_DEBUG_LOG("Removing CPCAP Audio Codec\n");

	curr_state->cpcap->h2w_new_state = NULL;
	curr_state->cpcap->h2w_new_state_data = NULL;
	curr_state->cpcap = NULL;
	kfree(curr_state);

	return 0;
}

struct snd_soc_codec_driver soc_codec_dev_cpcap = {
	.probe = cpcap_probe,
	.remove = cpcap_remove,
	.suspend = cpcap_suspend,
	.resume = cpcap_resume,
	.read = cpcap_audio_reg_read,
	.write = cpcap_audio_reg_write,
	.reg_cache_size = CPCAP_AUDIO_REG_NUM,
	.reg_word_size = sizeof(unsigned short),
};

static int __devinit cpcap_codec_probe(struct platform_device *pdev)
{
	int ret = 0;

	ret = snd_soc_register_codec(&pdev->dev,
			&soc_codec_dev_cpcap, cpcap_dai, ARRAY_SIZE(cpcap_dai));
	if (ret) {
		printk(KERN_ERR "Not able to register codec\n");
		return ret;
	}

	return 0;
}

static int __devexit cpcap_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

MODULE_ALIAS("platform:cpcap_audio");

static struct platform_driver cpcap_codec_driver = {
	.driver = {
		.name = "cpcap_audio",
		.owner = THIS_MODULE,
	},
	.probe = cpcap_codec_probe,
	.remove = __devexit_p(cpcap_codec_remove),
};

static int __init cpcap_codec_init(void)
{
	return platform_driver_register(&cpcap_codec_driver);
}
module_init(cpcap_codec_init);

static void __exit cpcap_codec_exit(void)
{
	platform_driver_unregister(&cpcap_codec_driver);
}
module_exit(cpcap_codec_exit);

MODULE_DESCRIPTION("ASoC CPCAP codec driver");
MODULE_AUTHOR("Motorola");
MODULE_LICENSE("GPL");
