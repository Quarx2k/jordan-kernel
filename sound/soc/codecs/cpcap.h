/*
 * Copyright (C)2007 - 2009 Motorola, Inc.
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

#ifndef __ABE_CPCAP_H__
#define __ABE_CPCAP_H__

#include <linux/soundcard.h>
#include <linux/spi/cpcap-regbits.h>
#include <linux/spi/cpcap.h>
#include <linux/regulator/consumer.h>

/* this is only true for audio registers, but those are the only ones we use */
#define CPCAP_REG_FOR_POWERIC_REG(a) ((a) + (0x200 - CPCAP_REG_VAUDIOC))

/* Generate index for accessing cpcap_audio_reg array */
#define CPCAP_AUDIO_REG_INDEX(a) (a - CPCAP_REG_VAUDIOC)

/* Generate cpcap_reg enum from index */
#define CPCAP_AUDIO_INDEX_REG(a) (a + CPCAP_REG_VAUDIOC)

/* Size of cpcap_audio_reg array - number of registers used in this driver */
#define CPCAP_AUDIO_REG_NUM (CPCAP_REG_A2LA - CPCAP_REG_VAUDIOC + 1)

#define SLEEP_ACTIVATE_POWER 2
#define CLOCK_TREE_RESET_TIME 1

struct cpcap_audio_state {
	struct cpcap_device *cpcap;
	struct snd_soc_codec *codec;
	int codec_strm_cnt;
	int stdac_strm_cnt;
};

#endif /* __ABE_CPCAP_H__ */
