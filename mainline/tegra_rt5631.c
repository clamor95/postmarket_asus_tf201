/*
 * tegra_rt5631.c - Tegra machine ASoC driver for boards using RT5631 codec.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "tegra_asoc_utils.h"

#include "../codecs/rt5631.h"

#define DRV_NAME "tegra-snd-rt5631"

struct tegra_rt5631 {
	struct tegra_asoc_utils_data util_data;
	int gpio_hp_det;
	enum of_gpio_flags gpio_hp_det_flags;
};

static int tegra_rt5631_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = asoc_rtd_to_codec(rtd, 0);
	struct snd_soc_card *card = rtd->card;
	struct tegra_rt5631 *machine = snd_soc_card_get_drvdata(card);

	int srate, mclk;
	int err;

	srate = params_rate(params);
	switch (srate) {
	case 64000:
	case 88200:
	case 96000:
		mclk = 128 * srate;
		break;
	default:
		mclk = 384 * srate;
		break;
	}
	/* FIXME: Codec only requires >= 3MHz if OSR==0 */
	while (mclk < 6000000)
		mclk *= 2;

	err = tegra_asoc_utils_set_rate(&machine->util_data, srate, mclk);
	if (err < 0) {
		dev_err(card->dev, "Can't configure clocks\n");
		return err;
	}

	err = snd_soc_dai_set_sysclk(codec_dai, 0, mclk,
					SND_SOC_CLOCK_IN);
	if (err < 0) {
		dev_err(card->dev, "codec_dai clock not set\n");
		return err;
	}

	return 0;
}

static struct snd_soc_ops tegra_rt5631_ops = {
	.hw_params = tegra_rt5631_hw_params,
};

static struct snd_soc_jack tegra_rt5631_hp_jack;

static struct snd_soc_jack_pin tegra_rt5631_hp_jack_pins[] = {
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static struct snd_soc_jack_gpio tegra_rt5631_hp_jack_gpio = {
	.name = "Headphone detection",
	.report = SND_JACK_HEADPHONE,
	.debounce_time = 150,
	.invert = 1,
};

static const struct snd_soc_dapm_widget tegra_rt5631_dapm_widgets[] = {
	SND_SOC_DAPM_SPK("Int Spk", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("Int Mic", NULL),
};

static const struct snd_kcontrol_new tegra_rt5631_controls[] = {
	SOC_DAPM_PIN_SWITCH("Int Spk"),
	SOC_DAPM_PIN_SWITCH("Int Mic"),
};

static int tegra_rt5631_init(struct snd_soc_pcm_runtime *rtd)
{
	struct tegra_rt5631 *machine = snd_soc_card_get_drvdata(card);

	snd_soc_card_jack_new(rtd->card, "Headphone Jack", SND_JACK_HEADPHONE,
			      &tegra_rt5631_hp_jack, tegra_rt5631_hp_jack_pins,
			      ARRAY_SIZE(tegra_rt5631_hp_jack_pins));

	if (gpio_is_valid(machine->gpio_hp_det)) {
		tegra_rt5631_hp_jack_gpio.gpio = machine->gpio_hp_det;
		tegra_rt5631_hp_jack_gpio.invert =
			!!(machine->gpio_hp_det_flags & OF_GPIO_ACTIVE_LOW);
		snd_soc_jack_add_gpios(&tegra_rt5631_hp_jack, 1,
					&tegra_rt5631_hp_jack_gpio);
	}

	return 0;
}

SND_SOC_DAILINK_DEFS(hifi,
	DAILINK_COMP_ARRAY(COMP_EMPTY()),
	DAILINK_COMP_ARRAY(COMP_CODEC(NULL, "rt5631-hifi")),
	DAILINK_COMP_ARRAY(COMP_EMPTY()));

static struct snd_soc_dai_link tegra_rt5631_dai = {
	.name = "RT5631",
	.stream_name = "RT5631 PCM",
	.init = tegra_rt5631_init,
	.ops = &tegra_rt5631_ops,
	.dai_fmt = SND_SOC_DAIFMT_I2S |
		   SND_SOC_DAIFMT_NB_NF |
		   SND_SOC_DAIFMT_CBS_CFS,
	SND_SOC_DAILINK_REG(hifi),
};

static struct snd_soc_card snd_soc_tegra_rt5631 = {
	.name = "tegra-rt5631",
	.owner = THIS_MODULE,
	.dai_link = tegra_rt5631_dai,
	.num_links = 1,
	.controls = tegra_rt5631_controls,
	.num_controls = ARRAY_SIZE(tegra_rt5631_controls),
	.dapm_widgets = tegra_rt5631_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(tegra_rt5631_dapm_widgets),
	.fully_routed = true,
};

static int tegra_rt5631_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct snd_soc_card *card = &snd_soc_tegra_rt5631;
	struct tegra_rt5631 *machine;

	int ret;

	machine = devm_kzalloc(&pdev->dev,
			sizeof(struct tegra_rt5631), GFP_KERNEL);
	if (!machine)
		return -ENOMEM;

	card->dev = &pdev->dev;
	snd_soc_card_set_drvdata(card, machine);

	machine->gpio_hp_det = of_get_named_gpio_flags(
		np, "nvidia,hp-det-gpios", 0, &machine->gpio_hp_det_flags);
	if (machine->gpio_hp_det == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	ret = snd_soc_of_parse_card_name(card, "nvidia,model");
	if (ret)
		return ret;

	ret = snd_soc_of_parse_audio_routing(card, "nvidia,audio-routing");
	if (ret)
		return ret;

	tegra_rt5631_dai.codecs->of_node = of_parse_phandle(np,
			"nvidia,audio-codec", 0);
	if (!tegra_rt5631_dai.codecs->of_node) {
		dev_err(&pdev->dev,
			"Property 'nvidia,audio-codec' missing or invalid\n");
		return -EINVAL;
	}

	tegra_rt5631_dai.platforms->of_node = tegra_rt5631_dai.cpus->of_node;

	ret = tegra_asoc_utils_init(&machine->util_data, &pdev->dev);
	if (ret)
		return ret;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card failed (%d)\n",
			ret);
		return ret;
	}

	return 0;
}

static int tegra_rt5631_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id tegra_rt5631_of_match[] = {
	{ .compatible = "nvidia,tegra-audio-rt5631", },
	{ .compatible = "nvidia,tegra-audio-alc5631", },
	{},
};

static struct platform_driver tegra_rt5631_driver = {
	.driver = {
		.name = DRV_NAME,
		.pm = &snd_soc_pm_ops,
		.of_match_table = tegra_rt5631_of_match,
	},
	.probe = tegra_rt5631_probe,
	.remove = tegra_rt5631_remove,
};
module_platform_driver(tegra_rt5631_driver);

MODULE_AUTHOR("Stephen Warren <swarren@nvidia.com>");
MODULE_DESCRIPTION("Tegra+RT5631 machine ASoC driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRV_NAME);
MODULE_DEVICE_TABLE(of, tegra_rt5631_of_match);
