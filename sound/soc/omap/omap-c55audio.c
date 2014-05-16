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

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/of.h>
#include <sound/soc.h>

#include "omap-mcbsp.h"

/* We desire a bit clock of 512 kHz for 16 bit 2 channel 16 kHz audio. */
#define BIT_CLOCK (16000*16*2)
static struct clk *per_96m_fck;
static unsigned long rate;
static int clkdiv;

static int omap_soc_c55_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int ret;

	/* Set McBSP4 clock to use PER_96M_FCLK */
	ret = snd_soc_dai_set_sysclk(cpu_dai, OMAP_MCBSP_SYSCLK_CLKS_FCLK,
		rate, SND_SOC_CLOCK_OUT);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set cpu system clock\n");
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(cpu_dai, OMAP_MCBSP_CLKGDV, clkdiv);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set SRG clock divider\n");
		return ret;
	}

	return 0;
}

static int omap_soc_c55_startup(struct snd_pcm_substream *substream)
{
	return clk_enable(per_96m_fck);
}

static void omap_soc_c55_shutdown(struct snd_pcm_substream *substream)
{
	clk_disable(per_96m_fck);
}

static struct snd_soc_ops omap_soc_c55_ops = {
	.startup = omap_soc_c55_startup,
	.hw_params = omap_soc_c55_hw_params,
	.shutdown = omap_soc_c55_shutdown,
};

/* Digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link omap_soc_c55_dai_links[] = {
	{
		.name = "OMAP SOC C55 Audio",
		.stream_name = "OMAP SOC C55",
		.cpu_dai_name = "omap-mcbsp.4",
		.codec_dai_name = "c55-voice",
		.platform_name = "omap-pcm-audio",
		.codec_name = "soc.1",
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBS_CFS,
		.ops = &omap_soc_c55_ops,
	},
};

/* Audio machine driver */
static struct snd_soc_card omap_soc_c55_card = {
	.owner = THIS_MODULE,
	.dai_link = omap_soc_c55_dai_links,
	.num_links = ARRAY_SIZE(omap_soc_c55_dai_links),
};

static int omap_soc_c55_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct snd_soc_card *card = &omap_soc_c55_card;
	int ret = 0;

	card->dev = &pdev->dev;

#ifdef CONFIG_OF
	if (node) {
		struct device_node *dai_node;

		if (snd_soc_of_parse_card_name(card, "mot,model")) {
			dev_err(&pdev->dev, "Card name is not provided\n");
			return -ENODEV;
		}

		dai_node = of_parse_phandle(node, "mot,mcbsp", 0);
		if (!dai_node) {
			dev_err(&pdev->dev, "McBSP node is not provided\n");
			return -EINVAL;
		}
		omap_soc_c55_dai_links[0].cpu_dai_name = NULL;
		omap_soc_c55_dai_links[0].cpu_of_node = dai_node;

	} else {
		dev_err(&pdev->dev, "Missing node\n");
		return -ENODEV;
	}
#endif

	snd_soc_card_set_drvdata(card, NULL);
	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
			ret);
		return ret;
	}

	per_96m_fck = clk_get(&pdev->dev, "per_96m_fck");
	if (IS_ERR(per_96m_fck)) {
		dev_err(&pdev->dev, "could not get per_96m_fck clock\n");
		return PTR_ERR(per_96m_fck);
	}

	rate = clk_get_rate(per_96m_fck);
	clkdiv = rate / BIT_CLOCK;

	return 0;
}

static int omap_soc_c55_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	clk_put(per_96m_fck);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id omap_soc_c55_of_match[] = {
	{.compatible = "mot,omap-soc-c55", },
	{ },
};
MODULE_DEVICE_TABLE(of, omap_soc_c55_of_match);

static struct platform_driver omap_soc_c55_driver = {
	.driver = {
		.name = "omap-soc-c55",
		.owner = THIS_MODULE,
		.pm = &snd_soc_pm_ops,
		.of_match_table = of_match_ptr(omap_soc_c55_of_match),
	},
	.probe = omap_soc_c55_probe,
	.remove = omap_soc_c55_remove,
};

module_platform_driver(omap_soc_c55_driver);

MODULE_AUTHOR("Motorola");
MODULE_DESCRIPTION("ALSA SoC for c55 audio");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:omap-soc-c55");
