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

#if defined(CONFIG_SND_HWDEP) || defined(CONFIG_SND_HWDEP_MODULE)
#include "rt56xx_ioctl.h"
#endif

struct rt5631_eq {
	int eq_mode;
	int pll_used_flag;
};

struct rt5631_init_reg {
	u8 reg;
	u16 val;
};

static int pw_ladc = 0;

#if ENABLE_ALC
static bool spk_out_flag = false;
static bool ADC_flag = false;
static bool DMIC_flag = true;   // heaset = false;
#endif

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

static void rt5631_write_index_mask(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value, unsigned int mask)
{
	unsigned int reg_val;

	if (!mask)
		return;

	if (mask != 0xffff) {
		reg_val = rt5631_read_index(codec, reg);
		reg_val &= ~mask;
		reg_val |= (value & mask);
		rt5631_write_index(codec, reg, reg_val);
	} else
		rt5631_write_index(codec, reg, value);

	return;
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
	{RT5631_ADC_CTRL_1		, 0x8080},
	{RT5631_SPK_OUT_VOL		, 0xc7c7},
	{RT5631_HP_OUT_VOL		, 0xc5c5},
	{RT5631_MONO_AXO_1_2_VOL	, 0xe040},
	{RT5631_ADC_REC_MIXER		, 0xb0f0},
	{RT5631_MIC_CTRL_2		, 0x6600},
	{RT5631_OUTMIXER_L_CTRL		, 0xdfC0},
	{RT5631_OUTMIXER_R_CTRL		, 0xdfC0},
	{RT5631_SPK_MIXER_CTRL		, 0xd8d8},
	{RT5631_SPK_MONO_OUT_CTRL	, 0x6c00},
	{RT5631_GEN_PUR_CTRL_REG	, 0x7e00}, //Speaker AMP ratio gain is 1.99X (5.99dB)
	{RT5631_SPK_MONO_HP_OUT_CTRL	, 0x0000},
	{RT5631_MIC_CTRL_1		, 0x8000}, //change Mic1 to differential mode,mic2 to single end mode
	{RT5631_INT_ST_IRQ_CTRL_2	, 0x0f18},
	{RT5631_ALC_CTRL_1		, 0x0B00}, //ALC Attack time  = 170.667ms, Recovery time = 83.333us
	{RT5631_ALC_CTRL_3		, 0x2410}, //Enable for DAC path, Limit level = -6dBFS
	{RT5631_AXO2MIXER_CTRL		, 0x8860},
};

#define RT5631_INIT_REG_LEN ARRAY_SIZE(init_list)

/*
 * EQ parameter
 */
enum {
	NORMAL,
	CLUB,
	DANCE,
	LIVE,
	POP,
	ROCK,
	OPPO,
	TREBLE,
	BASS,
	TF201,
	TF300TG,
	TF700T,
	TF300TL,
};

typedef struct _HW_EQ_PRESET {
	u16 HwEqType;
	u16 EqValue[22];
	u16 HwEQCtrl;
	u16 EqInVol;
	u16 EqOutVol;
}HW_EQ_PRESET;

static HW_EQ_PRESET HwEq_Preset[] = {
/* EQ param reg :  0x0,    0x1,    0x2,    0x3,    0x4,    0x5,    0x6,    0x7,    0x8,    0x9,    0xA,    0xB,    0xC,    0xD,    0xE,    0xF      EQ control reg: 0x6e  */
	{NORMAL		, {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}, 0x0000, 0x8000, 0x0003},
	{CLUB		, {0x1C10, 0x0000, 0xC1CC, 0x1E5D, 0x0699, 0xCD48, 0x188D, 0x0699, 0xC3B6, 0x1CD0, 0x0699, 0x0436, 0x0000, 0x0000, 0x0000, 0x0000}, 0x000E, 0x8000, 0x0003},
	{DANCE		, {0x1F2C, 0x095B, 0xC071, 0x1F95, 0x0616, 0xC96E, 0x1B11, 0xFC91, 0xDCF2, 0x1194, 0xFAF2, 0x0436, 0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{LIVE		, {0x1EB5, 0xFCB6, 0xC24A, 0x1DF8, 0x0E7C, 0xC883, 0x1C10, 0x0699, 0xDA41, 0x1561, 0x0295, 0x0436, 0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{POP		, {0x1EB5, 0xFCB6, 0xC1D4, 0x1E5D, 0x0E23, 0xD92E, 0x16E6, 0xFCB6, 0x0000, 0x0969, 0xF988, 0x0436, 0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{ROCK		, {0x1EB5, 0xFCB6, 0xC071, 0x1F95, 0x0424, 0xC30A, 0x1D27, 0xF900, 0x0C5D, 0x0FC7, 0x0E23, 0x0436, 0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{OPPO		, {0x0000, 0x0000, 0xCA4A, 0x17F8, 0x0FEC, 0xCA4A, 0x17F8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{TREBLE		, {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x188D, 0x1699, 0x0000, 0x0000, 0x0000}, 0x0010, 0x8000, 0x0003},
	{BASS		, {0x1A43, 0x0C00, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000}, 0x0001, 0x8000, 0x0003},
	{TF201		, {0x0264, 0xFE43, 0xC111, 0x1EF8, 0x1D18, 0xC1EC, 0x1E61, 0xFA19, 0xC6B1, 0x1B54, 0x0FEC, 0x0D41, 0x095B, 0xC0B6, 0x1F4C, 0x1FA5}, 0x403F, 0x8003, 0x0004},
	{TF300TG	, {0x1CD0, 0x1D18, 0xC21C, 0x1E30, 0xF900, 0xC2C8, 0x1EC4, 0x095B, 0xCA22, 0x1C10, 0x1830, 0xF76D, 0x0FEC, 0xC130, 0x1ED6, 0x1F69}, 0x403F, 0x8004, 0x0005},
	{TF700T 	, {0x0264, 0xFE43, 0xC0E5, 0x1F2C, 0x0C73, 0xC19B, 0x1EB2, 0xFA19, 0xC5FC, 0x1C10, 0x095B, 0x1561, 0x0699, 0xC18B, 0x1E7F, 0x1F3D}, 0x402A, 0x8003, 0x0005},
	{TF300TL	, {0x1CD0, 0x1D18, 0xC21C, 0x1E30, 0xF900, 0xC2C8, 0x1EC4, 0x095B, 0xCA22, 0x1C10, 0x1830, 0xF76D, 0x0FEC, 0xC130, 0x1ED6, 0x1F69}, 0x403F, 0x8004, 0x0005},
};

static void rt5631_update_eqmode(struct snd_soc_codec *codec, int mode)
{
	u16 HwEqIndex;

	if (NORMAL == mode) {
		/* In Normal mode, the EQ parameter is cleared,
		 * and hardware LP, BP1, BP2, BP3, HP1, HP2
		 * block control and EQ block are disabled.
		 */
		for (HwEqIndex = RT5631_EQ_BW_LOP; HwEqIndex <= RT5631_EQ_HPF_GAIN; HwEqIndex++)
			rt5631_write_index(codec, HwEqIndex,
				HwEq_Preset[mode].EqValue[HwEqIndex]);
		snd_soc_update_bits(codec, RT5631_EQ_CTRL, 0x003f, 0x0000);
		rt5631_write_index_mask(codec, RT5631_EQ_PRE_VOL_CTRL, 
				0x0000, 0x8000);
	} else {
		/* 
		 * Fill and update EQ parameter,
		 * and EQ block are enabled.
		 */
		snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1, 
			RT5631_PWR_ADC_L_CLK, RT5631_PWR_ADC_L_CLK);
		rt5631_write_index_mask(codec, RT5631_EQ_PRE_VOL_CTRL,
					0x8000, 0x8000);
		snd_soc_write(codec, RT5631_EQ_CTRL, 0x8000);
		for (HwEqIndex = RT5631_EQ_BW_LOP; HwEqIndex <= RT5631_EQ_HPF_GAIN; HwEqIndex++)
			rt5631_write_index(codec, HwEqIndex,
				HwEq_Preset[mode].EqValue[HwEqIndex]);
		rt5631_write_index(codec, RT5631_EQ_PRE_VOL_CTRL, HwEq_Preset[mode].EqInVol); //set EQ input volume
		rt5631_write_index(codec, RT5631_EQ_POST_VOL_CTRL, HwEq_Preset[mode].EqOutVol); //set EQ output volume
		snd_soc_write(codec, RT5631_EQ_CTRL, HwEq_Preset[mode].HwEQCtrl | 0xc000);
		if ((HwEq_Preset[mode].HwEQCtrl & 0x8000))
			snd_soc_update_bits(codec, RT5631_EQ_CTRL, 0xc000, 0x8000);
		else
			snd_soc_update_bits(codec, RT5631_EQ_CTRL, 0xc000, 0x0000);
		if (!pw_ladc)
			snd_soc_update_bits(codec, RT5631_PWR_MANAG_ADD1, RT5631_PWR_ADC_L_CLK, 0);
	}

	return;
}

int rt5631_eq_sel_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rt5631_eq *rt5631 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = rt5631->eq_mode;

	return 0;
}

int rt5631_eq_sel_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rt5631_eq *rt5631 = snd_soc_codec_get_drvdata(codec);

	if (rt5631->eq_mode == ucontrol->value.integer.value[0])
		return 0;

	rt5631_update_eqmode(codec, ucontrol->value.enumerated.item[0]);
	rt5631->eq_mode = ucontrol->value.integer.value[0];

	return 0;
}

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
#if ENABLE_ALC
		pr_info("%s(): set ALC AMIC parameter\n", __func__);
		DMIC_flag = false;
		if (!spk_out_flag) {
			if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x000a);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe090);
			} else {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0004);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe084);
			}
		}
#endif
		/* set heaset mic gain */
		pr_info("%s():set headset gain\n", __func__);
		if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
			snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x001f, 0x0005);
		else
			snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x001f, 0x0000);
	} else {
#if ENABLE_ALC
		pr_info("%s(): set ALC DMIC parameter\n", __func__);
		DMIC_flag = true;
		if (!spk_out_flag) {
			if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x000e);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe099);
			} else {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0006);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe09a);
			}
		}
#endif
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
#if ENABLE_ALC
			pr_info("spk_event --ALC_SND_SOC_DAPM_POST_PMU\n");
			spk_out_flag = true;
			/* Enable ALC */
			switch (tegra3_get_project_id()) {
			case TEGRA3_PROJECT_TF201:
				snd_soc_write(codec,
					RT5631_GEN_PUR_CTRL_REG, 0x6e00);
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0B00);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0000);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0x6410);
				break;
			case TEGRA3_PROJECT_TF300TG:
			case TEGRA3_PROJECT_TF300TL:
				snd_soc_write(codec,
					RT5631_GEN_PUR_CTRL_REG, 0x6e00);
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0B00);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0000);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0x6510);
				break;
			case TEGRA3_PROJECT_TF700T:
				snd_soc_write(codec,
					RT5631_GEN_PUR_CTRL_REG, 0x7e00);
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0307);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0000);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0x6510);
				break;
			}
#endif
#if ENABLE_EQ
			if (rt531_dac_pwr == 0x3) {
				snd_soc_update_bits(codec,RT5631_PWR_MANAG_ADD1, 0x8000, 0x8000); //enable IIS interface power
				if (tegra3_get_project_id() == TEGRA3_PROJECT_TF201) {
					rt5631_update_eqmode(codec, TF201);
				} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG) {
					rt5631_update_eqmode(codec, TF300TG);
				} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
					rt5631_update_eqmode(codec, TF700T);
				} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300TL) {
					rt5631_update_eqmode(codec, TF300TL);
				} else {
					rt5631_update_eqmode(codec, TF201);
				} //enable EQ after power on DAC power
			}
#endif
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
#if ENABLE_ALC
		pr_info("spk_event --ALC_SND_SOC_DAPM_PRE_PMD\n");
		spk_out_flag = false;
		if (!spk_out_flag && !ADC_flag) {
			//Disable ALC
			snd_soc_update_bits(codec, RT5631_ALC_CTRL_3, 0xf000, 0x2000);
		} else if (!spk_out_flag && DMIC_flag ) {
			if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x000e);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe099);
			} else {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0006);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe09a);
			}
		} else if (!spk_out_flag && !DMIC_flag ) {
			if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x000a);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe090);
			} else {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0004);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe084);
			}
		}
#endif
#if ENABLE_EQ
		pr_info("spk_event --EQ_SND_SOC_DAPM_PRE_PMD\n");
		if (rt531_dac_pwr == 0x3)
			rt5631_update_eqmode(codec, NORMAL);    //disable EQ before powerdown speaker power
#endif
		break;

	default:
		return 0;
	}

	if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T ||
	   tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG ||
	   tegra3_get_project_id() == TEGRA3_PROJECT_TF300TL) {
		rt5631_write_index(codec, 0x48, 0xF73C);
		reg_val = rt5631_read_index(codec, 0x48);
		pr_info("%s -codec index 0x48=0x%04X\n", __FUNCTION__, reg_val);
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
		pw_ladc = 0;
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

	case SND_SOC_DAPM_POST_PMU:
		pw_ladc = 1;

#if ENABLE_ALC
		pr_info("adc_event --ALC_SND_SOC_DAPM_POST_PMU\n");
		ADC_flag = true;
		if (!spk_out_flag && DMIC_flag){
			if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x000e);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe099);
			} else {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0006);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe09a);
			}
		} else if (!spk_out_flag && !DMIC_flag) {
			if (tegra3_get_project_id() == TEGRA3_PROJECT_TF700T) {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x000a);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe090);
				snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x001f, 0x0005);
			} else {
				snd_soc_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				snd_soc_write(codec, RT5631_ALC_CTRL_2, 0x0004);
				snd_soc_write(codec, RT5631_ALC_CTRL_3, 0xe084);
			}
		}
		msleep(1);
		snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x8080, 0x0000);
#endif

		break;

	case SND_SOC_DAPM_PRE_PMD:
#if ENABLE_ALC
		pr_info("adc_event --ALC_SND_SOC_DAPM_PRE_PMD\n");
		ADC_flag = false;
		if (!spk_out_flag )
			snd_soc_update_bits(codec, RT5631_ALC_CTRL_3, 0xf000, 0x2000);     //Disable ALC
#endif
		snd_soc_update_bits(codec, RT5631_ADC_CTRL_1, 0x8080, 0x8080);

		break;

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
#if ENABLE_ALC
		pr_info("dac_event --ALC_SND_SOC_DAPM_PRE_PMD\n");
		if (!spk_out_flag && !ADC_flag ) {
			//Disable ALC
			snd_soc_update_bits(codec, RT5631_ALC_CTRL_3, 0xf000, 0x2000);
		}
#endif
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

#if defined(CONFIG_SND_HWDEP) || defined(CONFIG_SND_HWDEP_MODULE)
    realtek_ce_init_hwdep(codec);
#endif

	return 0;
}
