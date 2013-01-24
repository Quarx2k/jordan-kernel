 /*
  * Copyright (C)2010 Motorola, Inc.
  *
  * This program is free software; you can redistribute it and/or modify
  * it under the terms of the GNU General Public License version 2 as
  * published by the Free Software Foundation.
  *
  * This program is distributed in the hope that it will be useful,
  * but WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  * GNU General Public License for more details.
  *
  * You should have received a copy of the GNU General Public License
  * along with this program; if not, write to the Free Software
  * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
  * 02111-1307, USA
  *
  */

#ifndef CPCAP_AUDIO_PLATFORM_DATA_H
#define CPCAP_AUDIO_PLATFORM_DATA_H

enum cpcap_audio_data_voice_type {
	VOICE_TYPE_NOT_SUPPORT = 0, /* Not supported voice type */
	VOICE_TYPE_QC_ANALOG = 1,
	VOICE_TYPE_QC = 2,
	VOICE_TYPE_STE = 3,
	VOICE_TYPE_MOT = 4,
};
struct cpcap_audio_pdata {
	int stereo_loudspeaker;
	int mic3;
	enum cpcap_audio_data_voice_type voice_type;
};

void cpcap_audio_set_platform_config(struct cpcap_audio_pdata *pdata);

/*is the Bluetooth audio path for voice calls completely separate
  from CPCAP phone codec, or must CPCAP provide bus clocking?*/
int cpcap_audio_has_independent_bt(void);

/*does this product have two independent loudspeakers?*/
int cpcap_audio_has_stereo_loudspeaker(void);

/*does this product have a third internal microphone?*/
int cpcap_audio_has_mic3(void);

/*does this product use 19.2Mhz in-call clock or 26Mhz? */
int cpcap_audio_has_19mhz_bp(void);

/*does this product use i2s for voice calls, or network mode?*/
int cpcap_audio_has_i2s_bp(void);

#endif
