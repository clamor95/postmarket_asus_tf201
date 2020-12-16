/*
 * rt5631-eq.c  --  ASUS tweaks for RT5631 ALSA Soc Audio driver
 *
 * Copyright (c) 2012, ASUSTek Corporation
 * Copyright (c) 2020, Svyatoslav Ryhel
 * Copyright (c) 2020, Narkolai
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/export.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <sound/core.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <mach/board-transformer-misc.h>

#include <../gpio-names.h>
#include <../board-transformer.h>

#include "rt5631.h"
#include "rt5631-eq.h"

struct rt5631_eq {
	int eq_mode;
	int pll_used_flag;
};

struct rt5631_init_reg {
	u8 reg;
	u16 val;
};

struct snd_soc_codec *rt5631_audio_codec = NULL;
EXPORT_SYMBOL(rt5631_audio_codec);

/**
 * rt5631_write_index - write index register of 2nd layer
 */
static void rt5631_write_index(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	snd_soc_write(codec, RT5631_INDEX_ADD, reg);
	snd_soc_write(codec, RT5631_INDEX_DATA, value);
}

/**
 * rt5631_read_index - read index register of 2nd layer
 */
static unsigned int rt5631_read_index(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int value;

	snd_soc_write(codec, RT5631_INDEX_ADD, reg);
	value = snd_soc_read(codec, RT5631_INDEX_DATA);

	return value;
}

/*
 * speaker channel volume select SPKMIXER, 0DB by default
 * Headphone channel volume select OUTMIXER,0DB by default
 * AXO1/AXO2 channel volume select OUTMIXER,0DB by default
 * Record Mixer source from Mic1/Mic2 by default
 * Mic1/Mic2 boost 44dB by default
 * DAC_L-->OutMixer_L by default
 * DAC_R-->OutMixer_R by default
 * DAC-->SpeakerMixer
 * Speaker volume-->SPOMixer(L-->L,R-->R)
 * Speaker AMP ratio gain is 1.99X (5.99dB)
 * HP from OutMixer,speaker out from SpeakerOut Mixer
 * enable HP zero cross
 * change Mic1 & mic2 to differential mode
 */
static struct rt5631_init_reg init_list[] = {
	{RT5631_ADC_CTRL_1				, 0x8080},
	{RT5631_SPK_OUT_VOL				, 0xc7c7},
	{RT5631_HP_OUT_VOL				, 0xc5c5},
	{RT5631_MONO_AXO_1_2_VOL		, 0xe040},
	{RT5631_ADC_REC_MIXER			, 0xb0f0},
	{RT5631_MIC_CTRL_2				, 0x6600},
	{RT5631_OUTMIXER_L_CTRL			, 0xdfC0},
	{RT5631_OUTMIXER_R_CTRL			, 0xdfC0},
	{RT5631_SPK_MIXER_CTRL			, 0xd8d8},
	{RT5631_SPK_MONO_OUT_CTRL		, 0x6c00},
	{RT5631_GEN_PUR_CTRL_REG		, 0x7e00}, //Speaker AMP ratio gain is 1.99X (5.99dB)
	{RT5631_SPK_MONO_HP_OUT_CTRL	, 0x0000},
	{RT5631_MIC_CTRL_1				, 0x8000}, //change Mic1 to differential mode,mic2 to single end mode
	{RT5631_INT_ST_IRQ_CTRL_2		, 0x0f18},
	{RT5631_ALC_CTRL_1				, 0x0B00}, //ALC Attack time  = 170.667ms, Recovery time = 83.333us
	{RT5631_ALC_CTRL_3				, 0x2410}, //Enable for DAC path, Limit level = -6dBFS
	{RT5631_AXO2MIXER_CTRL			, 0x8860},
};

#define RT5631_INIT_REG_LEN ARRAY_SIZE(init_list)

int rt5631_get_gain(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

int rt5631_set_gain(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	int ret = 0;

	mutex_lock(&codec->mutex);

	if (ucontrol->value.enumerated.item[0]) {
		/* set heaset mic gain */
		pr_info("%s():set headset gain\n", __func__);
		if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
			snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x001f, 0x0005);
		else
			snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x001f, 0x0000);
	} else {
		/* set dmic gain */
		pr_info("%s(): use codec for capture gain\n", __func__);
		if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
			snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x00ff, 0x0013);
		else
			snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x001f, 0x000f);    //boost 22.5dB
	}
	mutex_unlock(&codec->mutex);

	return ret;
}

/**
 * config_common_power - control all common power of codec system
 * @pmu: power up or not
 */
static int config_common_power(struct snd_soc_codec *codec, bool pmu)
{
	struct rt5631_eq *rt5631 = snd_soc_codec_get_drvdata(codec);
	unsigned int mux_val;
	static int ref_count = 0;

	if (pmu) {
		ref_count++;
		snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1,
			RT5631_PWR_MAIN_I2S_EN | RT5631_PWR_DAC_REF,
			RT5631_PWR_MAIN_I2S_EN | RT5631_PWR_DAC_REF);
		mux_val = snd_soc_read(codec, RT5631_SPK_MONO_HP_OUT_CTRL);
		if (!(mux_val & RT5631_HP_L_MUX_SEL_DAC_L))
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1,
				RT5631_PWR_DAC_L_TO_MIXER, RT5631_PWR_DAC_L_TO_MIXER);
		if (!(mux_val & RT5631_HP_R_MUX_SEL_DAC_R))
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1,
				RT5631_PWR_DAC_R_TO_MIXER, RT5631_PWR_DAC_R_TO_MIXER);
		if (rt5631->pll_used_flag)
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD2,
						RT5631_PWR_PLL1, RT5631_PWR_PLL1);
	} else {
		ref_count--;
		if (ref_count == 0) {
			pr_info("%s: Real powr down, ref_count = 0\n", __func__);
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1,
				RT5631_PWR_MAIN_I2S_EN | RT5631_PWR_DAC_REF |
				RT5631_PWR_DAC_L_TO_MIXER | RT5631_PWR_DAC_R_TO_MIXER, 0);
		}
		if (rt5631->pll_used_flag)
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD2,
						RT5631_PWR_PLL1, 0);
	}

	return 0;
}

int spk_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	static int spkl_out_enable, spkr_out_enable;
	unsigned int rt531_dac_pwr = 0;
	unsigned int tf700t_pcb_id = 0;
	unsigned int reg_val;

	tf700t_pcb_id = tegra3_query_pcba_revision_pcbid();
	rt531_dac_pwr = (snd_soc_read(codec, RT5631_PWR_MANAG_ADD1) & 0x0300) >> 8;

	switch (event) {
		case SND_SOC_DAPM_POST_PMU:
		if (!spkl_out_enable && !strcmp(w->name, "SPKL Amp")) {
			if (tegra3_get_project_id() == TEGRA3_PROJECT_TF201) {
				snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL, RT5631_L_VOL, 0x0700);
			} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG) {
				snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL, RT5631_L_VOL, 0x0700);
			} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
				snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL, RT5631_L_VOL, 0x0600);
			}

			if ((tf700t_pcb_id == TF700T_PCB_ER1) &&
				(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)) {
				snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL, RT5631_L_VOL, 0x0d00);
				pr_info("%s: TF700T ER1 spk L ch vol = -7.5dB\n", __func__);
			}

			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD4,
					RT5631_PWR_SPK_L_VOL, RT5631_PWR_SPK_L_VOL);
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1,
					RT5631_PWR_CLASS_D, RT5631_PWR_CLASS_D);
			snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL,
					RT5631_L_MUTE, 0);
			spkl_out_enable = 1;
		}
		if (!spkr_out_enable && !strcmp(w->name, "SPKR Amp")) {
			if (tegra3_get_project_id() == TEGRA3_PROJECT_TF201) {
				snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL, RT5631_R_VOL, 0x0007);
			} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG) {
				snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL, RT5631_R_VOL, 0x0007);
			} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
				snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL, RT5631_R_VOL, 0x0006);
			}

			if ((tf700t_pcb_id == TF700T_PCB_ER1) &&
			(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)) {
				snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL, RT5631_R_VOL, 0x000d);
				pr_info("%s: TF700T ER1 spk R ch vol = -7.5dB\n", __func__);
			}

			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD4,
					RT5631_PWR_SPK_R_VOL, RT5631_PWR_SPK_R_VOL);
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1,
					RT5631_PWR_CLASS_D, RT5631_PWR_CLASS_D);
			snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL,
					RT5631_R_MUTE, 0);
			spkr_out_enable = 1;
		}
		break;

	case SND_SOC_DAPM_POST_PMD:
		if (spkl_out_enable && !strcmp(w->name, "SPKL Amp")) {
			snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL,
					RT5631_L_MUTE, RT5631_L_MUTE);
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD4,
					RT5631_PWR_SPK_L_VOL, 0);
			spkl_out_enable = 0;
		}

		if (spkr_out_enable && !strcmp(w->name, "SPKR Amp")) {
			snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL,
					RT5631_R_MUTE, RT5631_R_MUTE);
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD4,
					RT5631_PWR_SPK_R_VOL, 0);
			spkr_out_enable = 0;
		}

		if (0 == spkl_out_enable && 0 == spkr_out_enable)
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1,
					RT5631_PWR_CLASS_D, 0);
		break;

	case SND_SOC_DAPM_PRE_PMD:
	default:
		return 0;
	}

	if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T ||
	   tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG ||
	   tegra3_get_project_id() == TEGRA3_PROJECT_TF300TL) {
		rt5631_write_index(codec, 0x48, 0xF73C);
		reg_val = rt5631_read_index(codec, 0x48);
		pr_info("%s -codec index 0x48=0x%04X\n", __func__, reg_val);
	}

	return 0;
}

int adc_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	static bool pmu;

	switch (event) {
	case SND_SOC_DAPM_POST_PMD:
		if (pmu) {
			config_common_power(codec, false);
			pmu = false;
		}
		break;

	case SND_SOC_DAPM_PRE_PMU:
		if (!pmu) {
			config_common_power(codec, true);
			pmu = true;
		}
		break;

	case SND_SOC_DAPM_PRE_PMD:
		snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x8080, 0x8080);
		break;

	case SND_SOC_DAPM_POST_PMU:
	default:
		break;
	}

	return 0;
}

int dac_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	static bool pmu;

	switch (event) {
	case SND_SOC_DAPM_POST_PMD:
		if (pmu) {
			config_common_power(codec, false);
			pmu = false;
		}
		break;

	case SND_SOC_DAPM_PRE_PMU:
		if (!pmu) {
			config_common_power(codec, true);
			pmu = true;
		}
		break;

	case SND_SOC_DAPM_PRE_PMD:
		break;

	default:
		break;
	}

	return 0;
}

int codec_3v3_power_switch_init(void)
{
	int ret = 0;

	if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
		ret = gpio_request_one(TEGRA_GPIO_PP0, GPIOF_INIT_HIGH,
				"rt5631_3v3_power_control");

	return ret;
}

int rt5631_reg_init(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < RT5631_INIT_REG_LEN; i++)
		snd_soc_write(codec, init_list[i].reg, init_list[i].val);

	rt5631_audio_codec = codec;

	return 0;
}
