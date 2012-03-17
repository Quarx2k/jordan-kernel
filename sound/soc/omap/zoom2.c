/*
 * zoom2.c  --  SoC audio for Zoom2
 *
 * Author: Misael Lopez Cruz <x0052729@ti.com>
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>

#include <asm/mach-types.h>
#include <mach/hardware.h>
#include <mach/gpio.h>
#include <mach/board-zoom.h>
#include <plat/mcbsp.h>

/* Register descriptions for twl4030 codec part */
#include <linux/mfd/twl4030-codec.h>
#include "../../../arch/arm/mach-omap2/mux.h"
#include "omap-mcbsp.h"
#include "omap-pcm.h"

#define ZOOM2_BT_MCBSP_GPIO		164
#define ZOOM2_HEADSET_MUX_GPIO		(OMAP_MAX_GPIO_LINES + 15)
/* McBSP mux config group */
enum {
	ZOOM2_ASOC_MUX_MCBSP2_SLAVE = 0,
	ZOOM2_ASOC_MUX_MCBSP2_MASTER,
	ZOOM2_ASOC_MUX_MCBSP3_SLAVE,
	ZOOM2_ASOC_MUX_MCBSP3_MASTER,
	ZOOM2_ASOC_MUX_MCBSP3_TRISTATE,
	ZOOM2_ASOC_MUX_MCBSP4_SLAVE,
	ZOOM2_ASOC_MUX_MCBSP4_MASTER,
};


static int zoom2_asoc_mux_config(int group)
{
	switch (group) {
	case ZOOM2_ASOC_MUX_MCBSP2_SLAVE:
		omap_mux_init_signal("mcbsp2_fsx.mcbsp2_fsx",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp2_clkx.mcbsp2_clkx",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp2_dr.mcbsp2_dr",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp2_dx.mcbsp2_dx",
						OMAP_PIN_OUTPUT);
		break;

	case ZOOM2_ASOC_MUX_MCBSP3_SLAVE:
		omap_mux_init_signal("mcbsp3_fsx.mcbsp3_fsx",
						OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mcbsp3_clkx.mcbsp3_clkx",
						OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mcbsp3_dr.mcbsp3_dr",
						OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mcbsp3_dx.mcbsp3_dx",
						OMAP_PIN_OUTPUT);
		break;

	case ZOOM2_ASOC_MUX_MCBSP3_MASTER:
		omap_mux_init_signal("mcbsp_clks.mcbsp_clks",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp3_fsx.mcbsp3_fsx",
						OMAP_PIN_OUTPUT);
		omap_mux_init_signal("mcbsp3_clkx.mcbsp3_clkx",
						OMAP_PIN_OUTPUT);
		omap_mux_init_signal("mcbsp3_dr.mcbsp3_dr",
						OMAP_PIN_INPUT_PULLDOWN);
		omap_mux_init_signal("mcbsp3_dx.mcbsp3_dx",
						OMAP_PIN_OUTPUT);
		break;

	case ZOOM2_ASOC_MUX_MCBSP3_TRISTATE:
		omap_mux_init_signal("mcbsp3_fsx.safe_mode",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp3_clkx.safe_mode",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp3_dr.safe_mode",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp3_dx.safe_mode",
						OMAP_PIN_INPUT);
		break;

	case ZOOM2_ASOC_MUX_MCBSP4_MASTER:
		omap_mux_init_signal("mcbsp_clks.mcbsp_clks",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("mcbsp4_fsx.mcbsp4_fsx",
						OMAP_PIN_OUTPUT);
		omap_mux_init_signal("gpmc_ncs4.mcbsp4_clkx",
						OMAP_PIN_OUTPUT);
		omap_mux_init_signal("gpmc_ncs5.mcbsp4_dr",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("gpmc_ncs6.mcbsp4_dx",
						OMAP_PIN_OUTPUT);
		break;

	case ZOOM2_ASOC_MUX_MCBSP4_SLAVE:
		omap_mux_init_signal("mcbsp4_fsx.mcbsp4_fsx",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("gpmc_ncs4.mcbsp4_clkx",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("gpmc_ncs5.mcbsp4_dr",
						OMAP_PIN_INPUT);
		omap_mux_init_signal("gpmc_ncs6.mcbsp4_dx",
						OMAP_PIN_OUTPUT);
		break;

	default:
		return -ENODEV;
	}
	return 0;
}


static int zoom2_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

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
				  SND_SOC_DAIFMT_NB_NF |
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

	/* Enable the 256 FS clock for HDMI */
	ret = twl4030_cpu_enable_ext_clock(codec_dai->codec, cpu_dai);
	if (ret < 0) {
		printk(KERN_ERR "can't set 256 FS clock\n");
		return ret;
	}

	/* Use external clock for McBSP2 */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_EXT,
					0, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu_dai system clock\n");
		return ret;
	}

	/*
	 * Set headset EXTMUTE signal to ON to make sure we
	 * get correct headset status
	 */
	gpio_direction_output(ZOOM2_HEADSET_EXTMUTE_GPIO, 1);

	return 0;
}

static void zoom2_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Use functional clock for McBSP2 */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
					0, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		printk(KERN_ERR "can't set cpu_dai system clock\n");
		/* Ignore error and follow through */
	}

	/* Disable the 256 FS clock used for HDMI */
	ret = twl4030_cpu_disable_ext_clock(codec_dai->codec, cpu_dai);
	if (ret < 0)
		printk(KERN_ERR "can't disable 256 FS clock\n");
}

static struct snd_soc_ops zoom2_ops = {
	.shutdown  = zoom2_shutdown,
	.hw_params = zoom2_hw_params,
};

static int zoom2_hw_voice_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				SND_SOC_DAIFMT_DSP_A |
				SND_SOC_DAIFMT_IB_NF |
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

	/* set codec Voice IF to application mode*/
	ret = snd_soc_dai_set_tristate(codec_dai, 0);
	if (ret) {
		printk(KERN_ERR "can't disable codec VIF tristate\n");
		return ret;
	}

	/* Set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, 26000000,
					SND_SOC_CLOCK_IN);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec system clock\n");
		return ret;
	}

	ret = twl4030_cpu_enable_ext_clock(codec_dai->codec, cpu_dai);
	if (ret < 0) {
		printk(KERN_ERR "can't set 256 FS clock\n");
		return ret;
	}

	return 0;
}

static void zoom2_voice_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	zoom2_asoc_mux_config(ZOOM2_ASOC_MUX_MCBSP3_TRISTATE);

	ret = snd_soc_dai_set_tristate(codec_dai, 1);
	if (ret) {
		printk(KERN_ERR "can't set codec VIF tristate\n");
		/* Ignore error and follow through */
	}

	ret = twl4030_cpu_disable_ext_clock(codec_dai->codec, cpu_dai);
	if (ret)
		printk(KERN_ERR "can't set 256 FS clock\n");
}

static struct snd_soc_ops zoom2_voice_ops = {
	.shutdown  = zoom2_voice_shutdown,
	.hw_params = zoom2_hw_voice_params,
};

static int zoom2_fm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	zoom2_asoc_mux_config(ZOOM2_ASOC_MUX_MCBSP4_SLAVE);

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				  SND_SOC_DAIFMT_I2S |
				  SND_SOC_DAIFMT_NB_NF |
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

	ret = twl4030_cpu_enable_ext_clock(codec_dai->codec, cpu_dai);
	if (ret < 0) {
		printk(KERN_ERR "can't set 256 FS clock\n");
		return ret;
	}

	return 0;
}

static void zoom2_fm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	ret = twl4030_cpu_disable_ext_clock(codec_dai->codec, cpu_dai);
	if (ret < 0)
		printk(KERN_ERR "can't disable 256 FS clock\n");
}

static struct snd_soc_ops zoom2_fm_ops = {
	.shutdown  = zoom2_fm_shutdown,
	.hw_params = zoom2_fm_hw_params,
};

static int zoom2_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	if (gpio_request(ZOOM2_BT_MCBSP_GPIO, "bt_mux") == 0) {
		gpio_direction_output(ZOOM2_BT_MCBSP_GPIO, 1);
		gpio_free(ZOOM2_BT_MCBSP_GPIO);
	}
	zoom2_asoc_mux_config(ZOOM2_ASOC_MUX_MCBSP3_SLAVE);

	/* Set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai,
				SND_SOC_DAIFMT_DSP_B |
				SND_SOC_DAIFMT_NB_IF |
				SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec DAI configuration\n");
		return ret;
	}

	/* Set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai,
				SND_SOC_DAIFMT_DSP_B |
				SND_SOC_DAIFMT_NB_IF |
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

	ret = snd_soc_dai_set_tristate(codec_dai, 1);
	if (ret < 0) {
		printk(KERN_ERR "can't set codec VIF tristate\n");
		return ret;
	}

	ret = twl4030_cpu_enable_ext_clock(codec_dai->codec, cpu_dai);
	if (ret < 0) {
		printk(KERN_ERR "can't set 256 FS clock\n");
		return ret;
	}

	return 0;
}


static void zoom2_pcm_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	zoom2_asoc_mux_config(ZOOM2_ASOC_MUX_MCBSP3_TRISTATE);

	ret = twl4030_cpu_disable_ext_clock(codec_dai->codec, cpu_dai);
	if (ret < 0)
		printk(KERN_ERR "can't disable 256 FS clock\n");
}

static struct snd_soc_ops zoom2_pcm_ops = {
	.shutdown  = zoom2_pcm_shutdown,
	.hw_params = zoom2_pcm_hw_params,
};

/* Zoom2 machine DAPM */
static const struct snd_soc_dapm_widget zoom2_twl4030_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Ext Mic", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
	SND_SOC_DAPM_HP("Headset Stereophone", NULL),
	SND_SOC_DAPM_LINE("Aux In", NULL),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* External Mics: MAINMIC, SUBMIC with bias*/
	{"MAINMIC", NULL, "Mic Bias 1"},
	{"SUBMIC", NULL, "Mic Bias 2"},
	{"Mic Bias 1", NULL, "Ext Mic"},
	{"Mic Bias 2", NULL, "Ext Mic"},

	/* External Speakers: HFL, HFR */
	{"Ext Spk", NULL, "HFL"},
	{"Ext Spk", NULL, "HFR"},

	/* Headset Stereophone:  HSOL, HSOR */
	{"Headset Stereophone", NULL, "HSOL"},
	{"Headset Stereophone", NULL, "HSOR"},

	/* Headset Mic: HSMIC with bias */
	{"HSMIC", NULL, "Headset Mic Bias"},
	{"Headset Mic Bias", NULL, "Headset Mic"},

	/* Aux In: AUXL, AUXR */
	{"Aux In", NULL, "AUXL"},
	{"Aux In", NULL, "AUXR"},
};

static int zoom2_twl4030_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	struct snd_soc_dapm_context *dapm = &codec->dapm;
	int ret;

	/* Add Zoom2 specific widgets */
	ret = snd_soc_dapm_new_controls(dapm, zoom2_twl4030_dapm_widgets,
				ARRAY_SIZE(zoom2_twl4030_dapm_widgets));
	if (ret)
		return ret;

	/* Set up Zoom2 specific audio path audio_map */
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	/* Zoom2 connected pins */
	snd_soc_dapm_enable_pin(dapm, "Ext Mic");
	snd_soc_dapm_enable_pin(dapm, "Ext Spk");
	snd_soc_dapm_enable_pin(dapm, "Headset Mic");
	snd_soc_dapm_enable_pin(dapm, "Headset Stereophone");
	snd_soc_dapm_enable_pin(dapm, "Aux In");

	/* TWL4030 not connected pins */
	snd_soc_dapm_nc_pin(dapm, "CARKITMIC");
	snd_soc_dapm_nc_pin(dapm, "DIGIMIC0");
	snd_soc_dapm_nc_pin(dapm, "DIGIMIC1");
	snd_soc_dapm_nc_pin(dapm, "OUTL");
	snd_soc_dapm_nc_pin(dapm, "OUTR");
	snd_soc_dapm_nc_pin(dapm, "EARPIECE");
	snd_soc_dapm_nc_pin(dapm, "PREDRIVEL");
	snd_soc_dapm_nc_pin(dapm, "PREDRIVER");
	snd_soc_dapm_nc_pin(dapm, "CARKITL");
	snd_soc_dapm_nc_pin(dapm, "CARKITR");

	ret = snd_soc_dapm_sync(dapm);

	return ret;
}

static int zoom2_twl4030_voice_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	unsigned short reg;

	/* Enable voice interface */
	reg = codec->driver->read(codec, TWL4030_REG_VOICE_IF);
	reg |= TWL4030_VIF_DIN_EN | TWL4030_VIF_DOUT_EN | TWL4030_VIF_EN;
	codec->driver->write(codec, TWL4030_REG_VOICE_IF, reg);

	return 0;
}

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link zoom2_dai[] = {
	{
		.name = "TWL4030 I2S",
		.stream_name = "TWL4030 Audio",
		.cpu_dai_name = "omap-mcbsp-dai.1",
		.codec_dai_name = "twl4030-hifi",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.init = zoom2_twl4030_init,
		.ops = &zoom2_ops,
	},
	{
		.name = "TWL4030 VOICE",
		.stream_name = "TWL4030 Voice",
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.codec_dai_name = "twl4030-voice",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.init = zoom2_twl4030_voice_init,
		.ops = &zoom2_voice_ops,
	},
	{	/* For bluetooth */
		.name = "TWL4030 PCM",
		.stream_name = "TWL4030 PCM",
		.cpu_dai_name = "omap-mcbsp-dai.2",
		.codec_dai_name = "twl4030-voice",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.init = zoom2_twl4030_voice_init,
		.ops = &zoom2_pcm_ops,
	},
	{
		.name = "TWL4030 FM",
		.stream_name = "TWL4030 FM",
		.cpu_dai_name = "omap-mcbsp-dai.3",
		.codec_dai_name = "twl4030-clock",
		.platform_name = "omap-pcm-audio",
		.codec_name = "twl4030-codec",
		.ops = &zoom2_fm_ops,
	},
};

/* Audio machine driver */
static struct snd_soc_card snd_soc_zoom2 = {
	.name = "Zoom2",
	.dai_link = zoom2_dai,
	.num_links = ARRAY_SIZE(zoom2_dai),
};

static struct platform_device *zoom2_snd_device;

static int __init zoom2_soc_init(void)
{
	int ret;

	if (!machine_is_omap_zoom2() && !machine_is_omap_zoom3())
		return -ENODEV;
	printk(KERN_INFO "Zoom2 SoC init\n");

	zoom2_snd_device = platform_device_alloc("soc-audio", -1);
	if (!zoom2_snd_device) {
		printk(KERN_ERR "Platform device allocation failed\n");
		return -ENOMEM;
	}

	platform_set_drvdata(zoom2_snd_device, &snd_soc_zoom2);
	ret = platform_device_add(zoom2_snd_device);
	if (ret)
		goto err1;

	BUG_ON(gpio_request(ZOOM2_HEADSET_MUX_GPIO, "hs_mux") < 0);
	gpio_direction_output(ZOOM2_HEADSET_MUX_GPIO, 0);

	BUG_ON(gpio_request(ZOOM2_HEADSET_EXTMUTE_GPIO, "ext_mute") < 0);
	gpio_direction_output(ZOOM2_HEADSET_EXTMUTE_GPIO, 1);
	return 0;

err1:
	printk(KERN_ERR "Unable to add platform device\n");
	platform_device_put(zoom2_snd_device);

	return ret;
}
module_init(zoom2_soc_init);

static void __exit zoom2_soc_exit(void)
{
	gpio_free(ZOOM2_HEADSET_MUX_GPIO);
	gpio_free(ZOOM2_HEADSET_EXTMUTE_GPIO);

	platform_device_unregister(zoom2_snd_device);
}
module_exit(zoom2_soc_exit);

MODULE_AUTHOR("Misael Lopez Cruz <x0052729@ti.com>");
MODULE_DESCRIPTION("ALSA SoC Zoom2");
MODULE_LICENSE("GPL");

