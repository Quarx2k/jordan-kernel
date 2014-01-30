/*
 * Copyright (C) 2014 Motorola, Inc.
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
 */

#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>

/* codec private data */
struct soc_c55_private {
	int mic_bias1_en_gpio;
	int mic_bias3_en_gpio;
};

static const struct snd_soc_dapm_widget soc_c55_dapm_widgets[] = {
	SND_SOC_DAPM_ADC("ADC", "Voice Capture", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_INPUT("MIC"),
};

static const struct snd_soc_dapm_route soc_c55_audio_map[] = {
	{"ADC", NULL, "MIC"},
};

static const struct of_device_id soc_c55_of_match[] = {
	{.compatible = "mot,soc-c55", },
	{ },
};
MODULE_DEVICE_TABLE(of, soc_c55_of_match);

static int soc_codec_c55_set_bias_level(struct snd_soc_codec *codec,
				  enum snd_soc_bias_level level)
{
	struct soc_c55_private *priv = snd_soc_codec_get_drvdata(codec);

	switch (level) {
	case SND_SOC_BIAS_ON:
		break;
	case SND_SOC_BIAS_PREPARE:
		break;
	case SND_SOC_BIAS_STANDBY:
		if (codec->dapm.bias_level == SND_SOC_BIAS_OFF) {
			/* Enable mic biases */
			gpio_set_value(priv->mic_bias1_en_gpio, 1);
			gpio_set_value(priv->mic_bias3_en_gpio, 1);
		}
		break;
	case SND_SOC_BIAS_OFF:
		/* Disable mic biases */
		gpio_set_value(priv->mic_bias1_en_gpio, 0);
		gpio_set_value(priv->mic_bias3_en_gpio, 0);
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

static struct snd_soc_dai_driver soc_codec_c55_dai = {
	.name = "c55-voice",
	.capture = {
		.stream_name = "Voice Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,},
};

static int soc_codec_c55_probe(struct snd_soc_codec *codec)
{
	struct soc_c55_private *c55;
	int gpio;
	int ret;

	c55 = devm_kzalloc(codec->dev, sizeof(struct soc_c55_private),
			       GFP_KERNEL);
	if (c55 == NULL) {
		dev_err(codec->dev, "Can not allocate memory\n");
		return -ENOMEM;
	}
	snd_soc_codec_set_drvdata(codec, c55);

#ifdef CONFIG_OF
	if (of_match_device(soc_c55_of_match, codec->dev)) {
		gpio = of_get_named_gpio(codec->dev->of_node,
			"mot,mic_bias1_en", 0);
		ret = (gpio < 0) ? -ENODEV : gpio_request(gpio, "mic_bias1_en");
		if (ret) {
			dev_err(codec->dev,
				"Failed acquiring Mic Bias 1 En GPIO-%d (%d)\n",
				gpio, ret);
			return ret;
		}
		gpio_direction_output(gpio, 0);
		c55->mic_bias1_en_gpio = gpio;

		gpio = of_get_named_gpio(codec->dev->of_node,
			"mot,mic_bias3_en", 0);
		ret = (gpio < 0) ? -ENODEV : gpio_request(gpio, "mic_bias3_en");
		if (ret) {
			dev_err(codec->dev,
				"Failed acquiring Mic Bias 3 En GPIO-%d (%d)\n",
				gpio, ret);
			return ret;
		}
		gpio_direction_output(gpio, 0);
		c55->mic_bias3_en_gpio = gpio;
	}
#endif

	return 0;
}

static int soc_codec_c55_remove(struct snd_soc_codec *codec)
{
	struct soc_c55_private *priv = snd_soc_codec_get_drvdata(codec);

	soc_codec_c55_set_bias_level(codec, SND_SOC_BIAS_OFF);

	if (priv) {
		gpio_free(priv->mic_bias1_en_gpio);
		gpio_free(priv->mic_bias3_en_gpio);
	}

	return 0;
}

static struct snd_soc_codec_driver soc_codec_c55 = {
	.probe = soc_codec_c55_probe,
	.remove = soc_codec_c55_remove,
	.set_bias_level = soc_codec_c55_set_bias_level,
	.idle_bias_off = true,
	.dapm_widgets = soc_c55_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(soc_c55_dapm_widgets),
	.dapm_routes = soc_c55_audio_map,
	.num_dapm_routes = ARRAY_SIZE(soc_c55_audio_map),
};

static int soc_c55_probe(struct platform_device *pdev)
{
	return snd_soc_register_codec(&pdev->dev, &soc_codec_c55,
		&soc_codec_c55_dai, 1);
}

static int soc_c55_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);
	return 0;
}

static struct platform_driver soc_c55_driver = {
	.probe		= soc_c55_probe,
	.remove		= soc_c55_remove,
	.driver		= {
		.name	= "soc-c55",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(soc_c55_of_match),
	},
};

module_platform_driver(soc_c55_driver);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("ALSA SoC c55 audio codec driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:soc-c55");
