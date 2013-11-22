/*
 * motsnd.c  --  SoC audio for Motorola Android Platform
 *
 * Author: Motorola
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/cpcap_audio_platform_data.h>
#include <linux/clk.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm_params.h>

#include <asm/mach-types.h>
#include <plat/hardware.h>
#include <plat/gpio.h>
#include <plat/mcbsp.h>
#include <plat/clock.h>

#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
#include "../../../arch/arm/mach-omap2/board-mapphone.h"
#endif

#define ABE_BYPASS

#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
#define MOTSND_CONFIG_ENABLE_ABE
#define MOTSND_CONFIG_ENABLE_SPDIF
#endif

#ifdef MOTSND_CONFIG_ENABLE_ABE
#include "omap-abe.h"
#include <sound/soc-dsp.h>
#include "omap-abe-dsp.h"
#endif
#include "omap-mcbsp.h"
#include "omap-pcm.h"
#include "../codecs/cpcap.h"

#include "../../../arch/arm/mach-omap2/clock.h"

#define MOTSND_DEBUG
#ifdef MOTSND_DEBUG
#define MOTSND_DEBUG_LOG(args...) printk(KERN_INFO "ALSA MOTSND:" args)
#else
#define MOTSND_DEBUG_LOG(args...)
#endif

static unsigned long dpll_abe_rate;

#define DPLL_ABE_RATE_SPDIF	90315789


static int motsnd_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;
	unsigned long rate;
	struct clk *dpll_abe_ck;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_IB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_sysclk(cpu_dai,
				     OMAP_MCBSP_SYSCLK_CLKX_EXT,
				     0, 0);
	if (ret < 0)
		printk(KERN_ERR "can't set cpu DAI system clock\n");

	return ret;
}

static struct snd_soc_ops motsnd_ops = {
	.hw_params = motsnd_hw_params,
};

static int motsnd_voice_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				  SND_SOC_DAIFMT_DSP_A |
				  SND_SOC_DAIFMT_NB_NF |
				  SND_SOC_DAIFMT_CBM_CFM);
	if (ret) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_DSP_A |
				SND_SOC_DAIFMT_IB_NF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu DAI configuration\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
				     SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_sysclk(cpu_dai,
				     OMAP_MCBSP_SYSCLK_CLKX_EXT,
				     0, 0);
	if (ret < 0)
		printk(KERN_ERR "can't set cpu DAI system clock\n");

	return ret;
}

static struct snd_soc_ops motsnd_voice_ops = {
	.hw_params = motsnd_voice_hw_params,
};

static int motsnd_incall_startup(struct snd_pcm_substream *substream)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
		mcbsp3_i2s1_pin_mux_switch(1);
	}
#endif

	return 0;
}

static void motsnd_incall_shutdown(struct snd_pcm_substream *substream)
{

	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
		mcbsp3_i2s1_pin_mux_switch(0);
	}
#endif
}
static int motsnd_incall_hw_params(struct snd_pcm_substream *substream,
				   struct snd_pcm_hw_params *params)
{
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	return 0;
}

static struct snd_soc_ops motsnd_incall_ops = {
	.startup = motsnd_incall_startup,
	.shutdown = motsnd_incall_shutdown,
	.hw_params = motsnd_incall_hw_params,
};

static int motsnd_fm_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	return 0;
}

static struct snd_soc_ops motsnd_fm_ops = {
	.hw_params = motsnd_fm_hw_params,
};

static int motsnd_btvoice_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
	int ret;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
		/* Set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SND_SOC_DAIFMT_IB_NF |
					  SND_SOC_DAIFMT_CBM_CFM);
		if (ret < 0) {
			printk(KERN_ERR "can't set cpu DAI configuration\n");
			return ret;
		}
	} else if (pdata->voice_type == VOICE_TYPE_QC) {
		/* MDM6600 */
		/* Set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SND_SOC_DAIFMT_IB_IF |
					  SND_SOC_DAIFMT_CBM_CFM);
		if (ret < 0) {
			printk(KERN_ERR "can't set cpu DAI configuration\n");
			return ret;
		}
	} else {
		ret = -EIO;
		printk(KERN_ERR "%s: voice_type = %u, not valid modem",
			__func__, pdata->voice_type);
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_sysclk(cpu_dai,
				     OMAP_MCBSP_SYSCLK_CLKX_EXT,
				     0, 0);
	if (ret < 0)
		printk(KERN_ERR "can't set cpu DAI system clock\n");

	return ret;
}

static struct snd_soc_ops motsnd_btvoice_ops = {
	.hw_params = motsnd_btvoice_hw_params,
};

#ifdef MOTSND_CONFIG_ENABLE_SPDIF
static int motsnd_spdif_startup(struct snd_pcm_substream *substream)
{
	unsigned long rate;
	struct clk *dpll_abe_ck;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	dpll_abe_ck = clk_get(NULL, "dpll_abe_ck");
	if (dpll_abe_ck == NULL) {
		printk(KERN_INFO "%s: "
			"SPDIF Error: Cannot open ABE DPLL!\n",
			__func__);
		return -EPERM ;
	}

	rate = DPLL_ABE_RATE_SPDIF;
	dpll_abe_rate = clk_get_rate(dpll_abe_ck);

	if (rate == 0) {
		printk(KERN_INFO "%s: "
			"SPDIF Error: ABE DPLL rate invalid!\n",
			__func__);
		return -EPERM;
	}

	if (omap3_noncore_dpll_set_rate(dpll_abe_ck, rate) < 0) {
		printk(KERN_INFO "%s: "
			"SPDIF Error: ABE DPLL programming failed!\n",
			__func__);
		return -EPERM;
	} else {
		propagate_rate(dpll_abe_ck);
	}

	return 0;
}

static void motsnd_spdif_shutdown(struct snd_pcm_substream *substream)
{
	unsigned long rate;
	struct clk *dpll_abe_ck;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	dpll_abe_ck = clk_get(NULL, "dpll_abe_ck");
	if (dpll_abe_ck == NULL) {
		printk(KERN_INFO "%s: "
			"SPDIF Error: Cannot open ABE DPLL!\n",
			__func__);
		return;
	}

	rate = dpll_abe_rate;

	if (rate == 0) {
		printk(KERN_INFO "%s: "
			"SPDIF Error: ABE DPLL rate invalid!\n",
			__func__);
		return;
	}

	if (omap3_noncore_dpll_set_rate(dpll_abe_ck, rate) < 0) {
		printk(KERN_INFO "%s: "
			"SPDIF Error: ABE DPLL programming failed!\n",
			__func__);
		return;
	} else {
		propagate_rate(dpll_abe_ck);
	}
}

static int motsnd_spdif_hw_params(struct snd_pcm_substream *substream,
				  struct snd_pcm_hw_params *params)
{
	MOTSND_DEBUG_LOG("%s: entered\n", __func__);
	return 0;
}

static struct snd_soc_ops motsnd_spdif_ops = {
	.startup = motsnd_spdif_startup,
	.shutdown = motsnd_spdif_shutdown,
	.hw_params = motsnd_spdif_hw_params,
};
#endif /*ENABLE_SPDIF*/

static int motsnd_bpvoice_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec_dai->codec;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct platform_device *pdev = container_of(codec->dev,
			struct platform_device, dev);
	struct cpcap_audio_pdata *pdata = pdev->dev.platform_data;
	int ret;

	MOTSND_DEBUG_LOG("%s: entered\n", __func__);

	if (pdata->voice_type == VOICE_TYPE_STE) {
		/* STE_M570 */
		/* Set cpu DAI configuration */
		ret = snd_soc_dai_set_fmt(cpu_dai,
					  SND_SOC_DAIFMT_I2S |
					  SND_SOC_DAIFMT_IB_NF |
					  SND_SOC_DAIFMT_CBM_CFM);

		if (ret < 0) {
			printk(KERN_ERR "can't set cpu DAI configuration\n");
			return ret;
		}
	} else {
		ret = -EIO;
		printk(KERN_ERR "%s: voice_type = %u, not valid modem",
			__func__, pdata->voice_type);
		return ret;
	}
	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_sysclk(cpu_dai,
				     OMAP_MCBSP_SYSCLK_CLKX_EXT,
				     0, 0);
	if (ret < 0)
		printk(KERN_ERR "can't set cpu DAI system clock\n");
	return ret;
}

static struct snd_soc_ops motsnd_bpvoice_ops = {
	.hw_params = motsnd_bpvoice_hw_params,
};

#ifdef MOTSND_CONFIG_ENABLE_ABE
static int mcbsp_be_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	unsigned int be_id = rtd->dai_link->be_id;
	unsigned int threshold;

	switch (be_id) {
	case OMAP_ABE_DAI_MM_FM:
		channels->min = 2;
		threshold = 2;
		break;
	case OMAP_ABE_DAI_BT_VX:
		channels->min = 1;
		threshold = 1;
		break;
	default:
		threshold = 1;
		break;
	}

	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
				    SNDRV_PCM_HW_PARAM_FIRST_MASK],
			SNDRV_PCM_FORMAT_S16_LE);

	omap_mcbsp_set_tx_threshold(cpu_dai->id, threshold);
	omap_mcbsp_set_rx_threshold(cpu_dai->id, threshold);

	return 0;
}
#endif /*ENABLE_ABE*/

static int motsnd_cpcap_init(struct snd_soc_pcm_runtime *rtd)
{
	MOTSND_DEBUG_LOG("%s: Entered\n", __func__);
	return 0;
}

static int motsnd_cpcap_voice_init(struct snd_soc_pcm_runtime *rtd)
{
	MOTSND_DEBUG_LOG("%s: Entered\n", __func__);
	return 0;
}

static struct snd_soc_dai_driver dai[] = {
{
	.name = "MODEM",
	.playback = {
		.stream_name = "VoiceCall-DL",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "VoiceCall-UL",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "FMDummy",
	.playback = {
		.stream_name = "FM-Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
}
};

#ifdef MOTSND_CONFIG_ENABLE_ABE
static const char *mm1_be[] = {
		OMAP_ABE_BE_MM_EXT0,

};

struct snd_soc_dsp_link fe_lp_media = {
	.playback	= true,
	.trigger = {
		SND_SOC_DSP_TRIGGER_BESPOKE, SND_SOC_DSP_TRIGGER_BESPOKE},
};
#endif /*ENABLE_ABE*/

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link motsnd_dai[] = {
#ifdef ABE_BYPASS
{
	.name = "Multimedia",
	.stream_name = "McBSP2-STDac",
	.cpu_dai_name = "omap-mcbsp-dai.1",
	.codec_dai_name = "cpcap stdac",
	.platform_name = "omap-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_init,
	.ops = &motsnd_ops,
	.ignore_suspend = 1,
},
#else
{
	.name = "Multimedia LP",
	.stream_name = "Multimedia",
	.cpu_dai_name = "MultiMedia1 LP",
	.platform_name = "aess",
	.dynamic = 1,
	.dsp_link = &fe_lp_media,
//	.supported_be = mm1_be,
//	.num_be = ARRAY_SIZE(mm1_be),
//	.fe_playback_channels = 2,
	.ignore_suspend = 1,
},
#endif
{
	.name = "Voice",
	.stream_name = "McBSP3-Codec",
	.cpu_dai_name = "omap-mcbsp-dai.2",
	.codec_dai_name = "cpcap codec",
	.platform_name = "omap-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_voice_ops,
	.ignore_suspend = 1,
},
{
	.name = "VoiceCall",
	.stream_name = "Modem-Codec",
	.cpu_dai_name = "MODEM",
	.codec_dai_name = "cpcap in-call",
	.platform_name = "omap-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_incall_ops,
	.ignore_suspend = 1,
},
{
	.name = "FMRadio",
	.stream_name = "FMAudio",
	.cpu_dai_name = "FMDummy",
	.codec_dai_name = "cpcap fm",
	.platform_name = "omap-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_fm_ops,
	.ignore_suspend = 1,
},
{
	.name = "BTCall",
	.stream_name = "Modem-BT",
	.cpu_dai_name = "MODEM",
	.codec_dai_name = "cpcap bt-call",
	.platform_name = "omap-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_incall_ops,
	.ignore_suspend = 1,
},
{
	.name = "BTVoice",
	.stream_name = "McBSP3-BT",
	.cpu_dai_name = "omap-mcbsp-dai.2",
	.codec_dai_name = "cpcap bt",
	.platform_name = "omap-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_btvoice_ops,
	.ignore_suspend = 1,
},
#ifdef MOTSND_CONFIG_ENABLE_SPDIF
{
	.name = "McASP",
	.stream_name = "SPDIF PCM Playback",
	.cpu_dai_name = "omap-mcasp-dai",
	.platform_name = "omap-pcm-audio",
	.codec_dai_name =  "null-codec-dai",
	.codec_name = "null-codec",
	.ops = &motsnd_spdif_ops,
	.ignore_suspend = 1,
},
#endif
{
	.name = "BPVoice",
	.stream_name = "McBSP3-BP",
	.cpu_dai_name = "omap-mcbsp-dai.2",
	.codec_dai_name = "BPVoice",
	.platform_name = "omap-pcm-audio",
	.codec_name = "cpcap_audio",
	.init = motsnd_cpcap_voice_init,
	.ops = &motsnd_bpvoice_ops,
	.ignore_suspend = 1,
},
#ifdef ABE_BYPASS
#ifndef CONFIG_MACH_OMAP_MAPPHONE_DEFY
{
	.name = "Multimedia LP",
	.stream_name = "Multimedia",
	.cpu_dai_name = "MultiMedia1 LP",
	.platform_name = "aess",
	.dynamic = 1,
	.dsp_link = &fe_lp_media,
//	.supported_be = mm1_be,
//	.num_be = ARRAY_SIZE(mm1_be),
//	.fe_playback_channels = 2,
	.ignore_suspend = 1,
},
#endif
#endif
#ifdef MOTSND_CONFIG_ENABLE_ABE
{
	.name = OMAP_ABE_BE_MM_EXT0,
	.stream_name = "FM Playback",

	/* ABE components - DL1 */
	.cpu_dai_name = "omap-mcbsp-dai.1",
	.platform_name = "aess",

	/* Phoenix - DL1 DAC */
	.codec_dai_name =  "cpcap stdac",
	.codec_name = "cpcap_audio",

	.no_pcm = 1, /* don't create ALSA pcm for this */
	.init = motsnd_cpcap_init,
	.be_hw_params_fixup = mcbsp_be_hw_params_fixup,
	.ops = &motsnd_ops,
	.be_id = OMAP_ABE_DAI_MM_FM,
	.ignore_suspend = 1,
},
#endif
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_mot = {
	.name = "motsnd",
	.long_name = "Motorola MAPPHONE",
	.dai_link = motsnd_dai,
	.num_links = ARRAY_SIZE(motsnd_dai),
};

static struct platform_device *mot_snd_device;

static int __init motsnd_soc_init(void)
{
	int ret;

	pr_info("ENTER: MOTSND SoC init\n");
	mot_snd_device = platform_device_alloc("soc-audio", -1);
	if (!mot_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}
	snd_soc_register_dais(&mot_snd_device->dev, dai, ARRAY_SIZE(dai));
	platform_set_drvdata(mot_snd_device, &snd_soc_mot);

	ret = platform_device_add(mot_snd_device);
	if (ret)
		goto err;

	return 0;

err:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(mot_snd_device);
	return ret;
}
module_init(motsnd_soc_init);

static void __exit motsnd_soc_exit(void)
{
	platform_device_unregister(mot_snd_device);
}
module_exit(motsnd_soc_exit);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("ALSA SoC MOTSND");
MODULE_LICENSE("GPL");
