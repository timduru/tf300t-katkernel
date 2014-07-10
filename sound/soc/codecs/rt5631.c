/*
 * rt5631.c  --  RT5631 ALSA Soc Audio driver
 *
 * Copyright 2011 Realtek Microelectronics
 *
 * Author: flove <flove@realtek.com>
 *
 * Based on WM8753.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/spi/spi.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <linux/gpio.h>
#include <../gpio-names.h>
#include <mach/board-cardhu-misc.h>

#include "rt5631.h"

#include <../board-cardhu.h>

#define RTK_IOCTL
#if defined(RTK_IOCTL)
#include "rt56xx_ioctl.h"
#endif //RTK_IOCTL
#define VIRTUAL_REG_FOR_MISC_FUNC 0x90
#define RT5631_PWR_ADC_L_CLK (1 << 11)

#define AUDIO_IOC_MAGIC	0xf7
#define AUDIO_IOC_MAXNR	6
#define AUDIO_STRESS_TEST	_IOW(AUDIO_IOC_MAGIC, 1,int)
#define AUDIO_DUMP	_IOW(AUDIO_IOC_MAGIC, 2,int)
#define OUTPUT_POWER_CONTROL	_IOW(AUDIO_IOC_MAGIC, 3,int)
#define AUDIO_I2C_READ	_IOW(AUDIO_IOC_MAGIC, 4,int)
#define AUDIO_I2C_WRITE	_IOW(AUDIO_IOC_MAGIC, 5,int)
#define AUDIO_CAPTURE_MODE _IOW(AUDIO_IOC_MAGIC, 6,int)
#define AUDIO_IOCTL_START_HEAVY (2)
#define AUDIO_IOCTL_START_NORMAL (1)
#define AUDIO_IOCTL_STOP (0)
#define START_NORMAL (HZ/2)
#define START_HEAVY (HZ/20)

#define ENABLE_EQ (1)
#define ENABLE_ALC (1)

#define INPUT_SOURCE_NORMAL 100
#define INPUT_SOURCE_VR 101
#define OUTPUT_SOURCE_NORMAL	 200
#define OUTPUT_SOURCE_VOICE 201
#define INPUT_SOURCE_NO_AGC 300
#define INPUT_SOURCE_AGC 301

#define RT5631_3V3_POWER_EN TEGRA_GPIO_PP0

#define RT5631_VERSION "0.01 alsa 1.0.24"
#define RETRY_MAX (5)
#define TF700T_PCB_ER1 (0x3)
#define DEPOP_DELAY (1)

#define CODEC_SPKVDD_POWER_5V0_EN_GPIO TPS6591X_GPIO_8

struct rt5631_priv {
	int codec_version;
	int master;
	int sysclk;
	int dmic_used_flag;
	int eq_mode;
	int pll_used_flag;
};

static int pw_ladc=0;
static struct snd_soc_codec *rt5631_codec;
static const u16 rt5631_reg[0x80];
static int timesofbclk = 64;
static unsigned int reg90;
static int poll_rate = 0;
struct delayed_work poll_audio_work;
int count_base = 1;
int count_100 = 0;
static int input_source=INPUT_SOURCE_NORMAL;
static int output_source=OUTPUT_SOURCE_NORMAL;
static int input_agc = INPUT_SOURCE_NO_AGC;
static int audio_codec_status = 0;
static int project_id = 0;
#if ENABLE_ALC
static bool spk_out_flag = false;
static bool ADC_flag = false;
static bool DMIC_flag= true;   //heaset = false;
#endif
struct snd_soc_codec *rt5631_audio_codec = NULL;
EXPORT_SYMBOL(rt5631_audio_codec) ;
extern bool headset_alive;

extern int asusAudiodec_i2c_write_data(char *data, int length);
extern int asusAudiodec_i2c_read_data(char *data, int length);
extern int asus_dock_in_state(void);

module_param(timesofbclk, int, S_IRUSR | S_IWUSR | S_IRGRP | S_IROTH);
MODULE_PARM_DESC(timeofbclk, "relationship between bclk and fs");

/*
 * read rt5631 register cache
 */
static inline unsigned int rt5631_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg < 1 || reg > (ARRAY_SIZE(rt5631_reg) + 1))
		return -1;
	return cache[reg];
}


/*
 * write rt5631 register cache
 */

static inline void rt5631_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg < 0 || reg > 0x7e)
		return;
	cache[reg] = value;
}


static inline int rt5631_write(struct snd_soc_codec *codec,
			unsigned int reg, unsigned int val)
{
	int ret = 0;
	int retry = 0;

	if (reg > 0x7e) {
		if (reg == 0x90)
			reg90 = val;
		return 0;
	}

	ret = snd_soc_write(codec, reg, val);
	while((ret != 0) && (retry < RETRY_MAX)){
		msleep(1);
		ret = snd_soc_write(codec, reg, val);
		retry++;
		printk("%s: retry times = %d\n", __func__, retry);
	}

	if(ret == 0){
		rt5631_write_reg_cache(codec, reg, val);
                return 0;
	}else{
                printk(KERN_ERR "%s failed\n", __func__);
                return -EIO;
        }

}

static inline unsigned int rt5631_read(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int value = 0x0;

	if (reg > 0x7e) {
		if (reg == 0x90)
		     return reg90;
	}

	value = snd_soc_read(codec, reg);
	if (value >= 0) {
		return value;
	} else {
		printk(KERN_ERR "%s failed\n", __func__);
              value = rt5631_read_reg_cache(codec, reg);
              printk(KERN_ERR "%s failed, fetch from cache[0x%02X]=0x%02X\n", __func__, reg, value);
              return value;
	}

}

static int rt5631_write_mask(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value, unsigned int mask)
{
	unsigned int reg_val;
	int ret = 0;

	if (!mask)
		return 0;

	if (mask != 0xffff) {
		reg_val = rt5631_read(codec, reg);
		reg_val &= ~mask;
		reg_val |= (value & mask);
		ret = rt5631_write(codec, reg, reg_val);
	} else {
		ret = rt5631_write(codec, reg, value);
	}

	return ret;
}

static void rt5631_write_index(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	rt5631_write(codec, RT5631_INDEX_ADD, reg);
	rt5631_write(codec, RT5631_INDEX_DATA, value);
	return;
}

static unsigned int rt5631_read_index(struct snd_soc_codec *codec,
				unsigned int reg)
{
	unsigned int value;

	rt5631_write(codec, RT5631_INDEX_ADD, reg);
	value = rt5631_read(codec, RT5631_INDEX_DATA);

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
	} else {
		rt5631_write_index(codec, reg, value);
	}

	return;
}

static inline int rt5631_reset(struct snd_soc_codec *codec)
{
	return snd_soc_write(codec, RT5631_RESET, 0);
}

struct rt5631_init_reg {
	u8 reg;
	u16 val;
};

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
	{RT5631_MIC_CTRL_1        	, 0x8000}, //change Mic1 to differential mode,mic2 to single end mode
	{RT5631_INT_ST_IRQ_CTRL_2	, 0x0f18},
	{RT5631_ALC_CTRL_1	, 0x0B00}, //ALC Attack time  = 170.667ms, Recovery time = 83.333us
	{RT5631_ALC_CTRL_3  , 0x2410}, //Enable for DAC path, Limit level = -6dBFS
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
	TF201_PAD,
	TF300TG,
	TF700T,
	TF300TL,
};

struct hw_eq_preset {
	u16 type;
	u16 value[22];
	u16 ctrl;
	u16  EqInVol;
	u16  EqOutVol;
};

/*
 * EQ param reg : 0x0, 0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7,
 *		0x8, 0x9, 0xa, 0xb, 0xc, 0xd, 0xe, 0xf
 * EQ control reg : 0x6e
 */
struct hw_eq_preset hweq_preset[] = {
	{NORMAL	, {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000}, 0x0000, 0x8000, 0x0003},
	{CLUB	, {0x1C10, 0x0000, 0xC1CC, 0x1E5D, 0x0699, 0xCD48,
		0x188D, 0x0699, 0xC3B6, 0x1CD0, 0x0699, 0x0436,
		0x0000, 0x0000, 0x0000, 0x0000}, 0x000E, 0x8000, 0x0003},
	{DANCE	, {0x1F2C, 0x095B, 0xC071, 0x1F95, 0x0616, 0xC96E,
		0x1B11, 0xFC91, 0xDCF2, 0x1194, 0xFAF2, 0x0436,
		0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{LIVE	, {0x1EB5, 0xFCB6, 0xC24A, 0x1DF8, 0x0E7C, 0xC883,
		0x1C10, 0x0699, 0xDA41, 0x1561, 0x0295, 0x0436,
		0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{POP	, {0x1EB5, 0xFCB6, 0xC1D4, 0x1E5D, 0x0E23, 0xD92E,
		0x16E6, 0xFCB6, 0x0000, 0x0969, 0xF988, 0x0436,
		0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{ROCK	, {0x1EB5, 0xFCB6, 0xC071, 0x1F95, 0x0424, 0xC30A,
		0x1D27, 0xF900, 0x0C5D, 0x0FC7, 0x0E23, 0x0436,
		0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{OPPO	, {0x0000, 0x0000, 0xCA4A, 0x17F8, 0x0FEC, 0xCA4A,
		0x17F8, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000}, 0x000F, 0x8000, 0x0003},
	{TREBLE	, {0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x188D,
		0x1699, 0x0000, 0x0000, 0x0000}, 0x0010, 0x8000, 0x0003},
	{BASS	, {0x1A43, 0x0C00, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
		0x0000, 0x0000, 0x0000, 0x0000}, 0x0001, 0x8000, 0x0003},
	{TF201_PAD ,{0x0264, 0xFE43, 0xC111, 0x1EF8, 0x1D18,	0xC1EC,
		0x1E61, 0xFA19, 0xC6B1, 0x1B54, 0x0FEC, 0x0D41,
		0x095B,0xC0B6,0x1F4C,0x1FA5},0x403F, 0x8003, 0x0004},
	{TF300TG ,{0x1CD0,0x1D18,0xC21C,0x1E30,0xF900,0xC2C8,0x1EC4,
		0x095B,0xCA22,0x1C10,0x1830,0xF76D,0x0FEC,0xC130,
		0x1ED6,0x1F69},0x403F, 0x8004, 0x0005},
	{TF700T ,{0x0264, 0xFE43, 0xC0E5, 0x1F2C, 0x0C73,0xC19B,
		0x1EB2, 0xFA19, 0xC5FC, 0x1C10, 0x095B, 0x1561,
		0x0699,0xC18B,0x1E7F,0x1F3D},0x402A, 0x8003, 0x0005},
	{TF300TL ,{0x1CD0,0x1D18,0xC21C,0x1E30,0xF900,0xC2C8,0x1EC4,
                0x095B,0xCA22,0x1C10,0x1830,0xF76D,0x0FEC,0xC130,
                0x1ED6,0x1F69},0x403F, 0x8004, 0x0005},

};

static int rt5631_reg_init(struct snd_soc_codec *codec)
{
	int i;

	for (i = 0; i < RT5631_INIT_REG_LEN; i++)
		rt5631_write(codec, init_list[i].reg, init_list[i].val);

	return 0;
}

static const char *rt5631_spol_source_sel[] = {
	"SPOLMIX", "MONOIN_RX", "VDAC", "DACL"};
static const char *rt5631_spor_source_sel[] = {
	"SPORMIX", "MONOIN_RX", "VDAC", "DACR"};
static const char *rt5631_mono_source_sel[] = {"MONOMIX", "MONOIN_RX", "VDAC"};
static const char *rt5631_input_mode_source_sel[] = {
	"Single-end", "Differential"};
static const char *rt5631_mic_boost[] = {"Bypass", "+20db", "+24db", "+30db",
			"+35db", "+40db", "+44db", "+50db", "+52db"};
static const char *rt5631_hpl_source_sel[] = {"LEFT HPVOL", "LEFT DAC"};
static const char *rt5631_hpr_source_sel[] = {"RIGHT HPVOL", "RIGHT DAC"};
static const char *rt5631_eq_sel[] = {"NORMAL", "CLUB", "DANCE", "LIVE", "POP",
				"ROCK", "OPPO", "TREBLE", "BASS"};


static const struct soc_enum rt5631_enum[] = {
SOC_ENUM_SINGLE(RT5631_SPK_MONO_HP_OUT_CTRL, 14, 4, rt5631_spol_source_sel),
SOC_ENUM_SINGLE(RT5631_SPK_MONO_HP_OUT_CTRL, 10, 4, rt5631_spor_source_sel),
SOC_ENUM_SINGLE(RT5631_SPK_MONO_HP_OUT_CTRL, 6, 3, rt5631_mono_source_sel),
SOC_ENUM_SINGLE(RT5631_MIC_CTRL_1, 15, 2,  rt5631_input_mode_source_sel),
SOC_ENUM_SINGLE(RT5631_MIC_CTRL_1, 7, 2,  rt5631_input_mode_source_sel),
SOC_ENUM_SINGLE(RT5631_MONO_INPUT_VOL, 15, 2, rt5631_input_mode_source_sel),
SOC_ENUM_SINGLE(RT5631_MIC_CTRL_2, 12, 9, rt5631_mic_boost),
SOC_ENUM_SINGLE(RT5631_MIC_CTRL_2, 8, 9, rt5631_mic_boost),
SOC_ENUM_SINGLE(RT5631_SPK_MONO_HP_OUT_CTRL, 3, 2, rt5631_hpl_source_sel),
SOC_ENUM_SINGLE(RT5631_SPK_MONO_HP_OUT_CTRL, 2, 2, rt5631_hpr_source_sel),
SOC_ENUM_SINGLE(0, 4, 9, rt5631_eq_sel),
};

static int rt5631_dmic_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = rt5631->dmic_used_flag;

	return 0;
}

static void rt5631_enable_dmic(struct snd_soc_codec *codec)
{
	rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL, DMIC_ENA, DMIC_ENA_MASK);
	rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL,
		DMIC_L_CH_UNMUTE | DMIC_R_CH_UNMUTE,
		DMIC_L_CH_MUTE_MASK | DMIC_R_CH_MUTE_MASK);
}

static void rt5631_close_dmic(struct snd_soc_codec *codec)
{
	rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL,
		DMIC_L_CH_MUTE | DMIC_R_CH_MUTE,
		DMIC_L_CH_MUTE_MASK | DMIC_R_CH_MUTE_MASK);
	rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL,
		DMIC_DIS, DMIC_ENA_MASK);

	rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x0000, 0x001f);	//boost 0dB
	return;
}

static int rt5631_dmic_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);

	if (rt5631->dmic_used_flag == ucontrol->value.integer.value[0])
		return 0;

	if (ucontrol->value.integer.value[0]) {
		rt5631_enable_dmic(codec);
		rt5631->dmic_used_flag = 1;
	} else {
		rt5631_close_dmic(codec);
		rt5631->dmic_used_flag = 0;
	}

	return 0;
}

static int rt5631_eq_sel_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);

	ucontrol->value.integer.value[0] = rt5631->eq_mode;

	return 0;
}

static void rt5631_update_eqmode(struct snd_soc_codec *codec, int mode)
{
	int i;

	if (NORMAL == mode) {
		/* In Normal mode, the EQ parameter is cleared,
		 * and hardware LP, BP1, BP2, BP3, HP1, HP2
		 * block control and EQ block are disabled.
		 */
		for (i = RT5631_EQ_BW_LOP; i <= RT5631_EQ_HPF_GAIN; i++)
			rt5631_write_index(codec, i,
				hweq_preset[mode].value[i]);
		rt5631_write_mask(codec, RT5631_EQ_CTRL, 0x0000, 0x003f);
		rt5631_write_index_mask(codec, RT5631_EQ_PRE_VOL_CTRL
						, 0x0000, 0x8000);
	} else {
		/* Fill and update EQ parameter,
		 * and EQ block are enabled.
		 */
		rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1,RT5631_PWR_ADC_L_CLK ,
			RT5631_PWR_ADC_L_CLK );
		rt5631_write_index_mask(codec, RT5631_EQ_PRE_VOL_CTRL
						, 0x8000, 0x8000);
		snd_soc_write(codec, RT5631_EQ_CTRL,0x8000);
		for (i = RT5631_EQ_BW_LOP; i <= RT5631_EQ_HPF_GAIN; i++)
			rt5631_write_index(codec, i,
				hweq_preset[mode].value[i]);
		rt5631_write_index(codec, RT5631_EQ_PRE_VOL_CTRL, hweq_preset[mode].EqInVol); //set EQ input volume
		rt5631_write_index(codec, RT5631_EQ_POST_VOL_CTRL, hweq_preset[mode].EqOutVol); //set EQ output volume
		snd_soc_write(codec, RT5631_EQ_CTRL, hweq_preset[mode].ctrl | 0xc000);
		if((hweq_preset[mode].ctrl & 0x8000))
			rt5631_write_mask(codec, RT5631_EQ_CTRL, 0x8000, 0xc000);
		else
			rt5631_write_mask(codec, RT5631_EQ_CTRL, 0x0000, 0xc000);
		if(!pw_ladc)
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1, 0, RT5631_PWR_ADC_L_CLK );
	}

	return;
}

static int rt5631_eq_sel_put(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);

	if (rt5631->eq_mode == ucontrol->value.integer.value[0])
		return 0;

	rt5631_update_eqmode(codec, ucontrol->value.enumerated.item[0]);
	rt5631->eq_mode = ucontrol->value.integer.value[0];

	return 0;
}

static int rt5631_get_gain(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	return 0;
}

static int rt5631_set_gain(struct snd_kcontrol *kcontrol,
			     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);

	int ret = 0;

	mutex_lock(&codec->mutex);

	if(ucontrol->value.enumerated.item[0]){
		#if ENABLE_ALC
		printk("%s(): set ALC AMIC parameter\n", __func__);
		DMIC_flag = false;
		if(!spk_out_flag){
			if(project_id == TEGRA3_PROJECT_TF700T){
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x000a);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe090);
			}else{
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0004);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe084);
			}
		}
		#endif
		/* set heaset mic gain */
		printk("%s():set headset gain\n", __func__);
		if(project_id == TEGRA3_PROJECT_TF700T)
			rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x0005, 0x001f);
		else
			rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x0000, 0x001f);
	}else{
		#if ENABLE_ALC
		printk("%s(): set ALC DMIC parameter\n", __func__);
		DMIC_flag = true;
		if(!spk_out_flag){
			if(project_id == TEGRA3_PROJECT_TF700T){
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x000e);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe099);
			}else{
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0006);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe09a);
			}
		}
		#endif
		/* set dmic gain */
		if(output_source==OUTPUT_SOURCE_VOICE || input_source==INPUT_SOURCE_VR || input_agc==INPUT_SOURCE_AGC){
			printk("%s(): use dsp for capture gain\n", __func__);
			rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x0000, 0x001f);	//boost 0dB
		}else{
			printk("%s(): use codec for capture gain\n", __func__);
			if(project_id == TEGRA3_PROJECT_TF700T)
			rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x0013, 0x00ff);
			else
			rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x000f, 0x001f);    //boost 22.5dB
		}
	}
	mutex_unlock(&codec->mutex);

	return ret;
}


static const struct snd_kcontrol_new rt5631_snd_controls[] = {
SOC_ENUM("MIC1 Mode Control",  rt5631_enum[3]),
SOC_ENUM("MIC1 Boost", rt5631_enum[6]),
SOC_ENUM("MIC2 Mode Control", rt5631_enum[4]),
SOC_ENUM("MIC2 Boost", rt5631_enum[7]),
SOC_ENUM("MONOIN Mode Control", rt5631_enum[5]),
//SOC_DOUBLE("PCM Playback Volume", RT5631_STEREO_DAC_VOL_2, 8, 0, 255, 1),
SOC_DOUBLE("PCM Playback Switch", RT5631_STEREO_DAC_VOL_1, 15, 7, 1, 1),
SOC_DOUBLE("MONOIN_RX Capture Volume", RT5631_MONO_INPUT_VOL, 8, 0, 31, 1),
SOC_DOUBLE("AXI Capture Volume", RT5631_AUX_IN_VOL, 8, 0, 31, 1),
SOC_SINGLE("AXO1 Playback Switch", RT5631_MONO_AXO_1_2_VOL, 15, 1, 1),
SOC_SINGLE("AXO2 Playback Switch", RT5631_MONO_AXO_1_2_VOL, 7, 1, 1),
//SOC_DOUBLE("OUTVOL Playback Volume", RT5631_MONO_AXO_1_2_VOL, 8, 0, 31, 1),
//SOC_DOUBLE("Speaker Playback Switch", RT5631_SPK_OUT_VOL, 15, 7, 1, 1),
//SOC_DOUBLE("Speaker Playback Volume", RT5631_SPK_OUT_VOL, 8, 0, 63, 1),
SOC_SINGLE("MONO Playback Switch", RT5631_MONO_AXO_1_2_VOL, 13, 1, 1),
//SOC_DOUBLE("HP Playback Switch", RT5631_HP_OUT_VOL, 15, 7, 1, 1),
//SOC_DOUBLE("HP Playback Volume", RT5631_HP_OUT_VOL, 8, 0, 63, 1),
SOC_SINGLE_EXT("DMIC Capture Switch", 0, 2, 1, 0,
	rt5631_dmic_get, rt5631_dmic_put),
SOC_ENUM_EXT("EQ Mode", rt5631_enum[10], rt5631_eq_sel_get, rt5631_eq_sel_put),
SOC_SINGLE("MIC1 Mute", RT5631_ADC_REC_MIXER, 14, 1, 0),
SOC_SINGLE("DMIC Mute Left", RT5631_DIG_MIC_CTRL, 13, 1, 0),
SOC_SINGLE("DMIC Mute Right", RT5631_DIG_MIC_CTRL, 12, 1, 0),

/* Set recording gain */
SOC_SINGLE_BOOL_EXT("Recording Gain", 0,
	rt5631_get_gain, rt5631_set_gain),
};

static const struct snd_kcontrol_new rt5631_recmixl_mixer_controls[] = {
SOC_DAPM_SINGLE("OUTMIXL Capture Switch", RT5631_ADC_REC_MIXER, 15, 1, 1),
SOC_DAPM_SINGLE("MIC1_BST1 Capture Switch", RT5631_ADC_REC_MIXER, 14, 1, 1),
SOC_DAPM_SINGLE("AXILVOL Capture Switch", RT5631_ADC_REC_MIXER, 13, 1, 1),
SOC_DAPM_SINGLE("MONOIN_RX Capture Switch", RT5631_ADC_REC_MIXER, 12, 1, 1),
};

static const struct snd_kcontrol_new rt5631_recmixr_mixer_controls[] = {
SOC_DAPM_SINGLE("MONOIN_RX Capture Switch", RT5631_ADC_REC_MIXER, 4, 1, 1),
SOC_DAPM_SINGLE("AXIRVOL Capture Switch", RT5631_ADC_REC_MIXER, 5, 1, 1),
SOC_DAPM_SINGLE("MIC2_BST2 Capture Switch", RT5631_ADC_REC_MIXER, 6, 1, 1),
SOC_DAPM_SINGLE("OUTMIXR Capture Switch", RT5631_ADC_REC_MIXER, 7, 1, 1),
};

static const struct snd_kcontrol_new rt5631_spkmixl_mixer_controls[] = {
SOC_DAPM_SINGLE("RECMIXL Playback Switch", RT5631_SPK_MIXER_CTRL, 15, 1, 1),
SOC_DAPM_SINGLE("MIC1_P Playback Switch", RT5631_SPK_MIXER_CTRL, 14, 1, 1),
SOC_DAPM_SINGLE("DACL Playback Switch", RT5631_SPK_MIXER_CTRL, 13, 1, 1),
SOC_DAPM_SINGLE("OUTMIXL Playback Switch", RT5631_SPK_MIXER_CTRL, 12, 1, 1),
};

static const struct snd_kcontrol_new rt5631_spkmixr_mixer_controls[] = {
SOC_DAPM_SINGLE("OUTMIXR Playback Switch", RT5631_SPK_MIXER_CTRL, 4, 1, 1),
SOC_DAPM_SINGLE("DACR Playback Switch", RT5631_SPK_MIXER_CTRL, 5, 1, 1),
SOC_DAPM_SINGLE("MIC2_P Playback Switch", RT5631_SPK_MIXER_CTRL, 6, 1, 1),
SOC_DAPM_SINGLE("RECMIXR Playback Switch", RT5631_SPK_MIXER_CTRL, 7, 1, 1),
};

static const struct snd_kcontrol_new rt5631_outmixl_mixer_controls[] = {
SOC_DAPM_SINGLE("RECMIXL Playback Switch", RT5631_OUTMIXER_L_CTRL, 15, 1, 1),
SOC_DAPM_SINGLE("RECMIXR Playback Switch", RT5631_OUTMIXER_L_CTRL, 14, 1, 1),
SOC_DAPM_SINGLE("DACL Playback Switch", RT5631_OUTMIXER_L_CTRL, 13, 1, 1),
SOC_DAPM_SINGLE("MIC1_BST1 Playback Switch", RT5631_OUTMIXER_L_CTRL, 12, 1, 1),
SOC_DAPM_SINGLE("MIC2_BST2 Playback Switch", RT5631_OUTMIXER_L_CTRL, 11, 1, 1),
SOC_DAPM_SINGLE("MONOIN_RXP Playback Switch", RT5631_OUTMIXER_L_CTRL, 10, 1, 1),
SOC_DAPM_SINGLE("AXILVOL Playback Switch", RT5631_OUTMIXER_L_CTRL, 9, 1, 1),
SOC_DAPM_SINGLE("AXIRVOL Playback Switch", RT5631_OUTMIXER_L_CTRL, 8, 1, 1),
SOC_DAPM_SINGLE("VDAC Playback Switch", RT5631_OUTMIXER_L_CTRL, 7, 1, 1),
};

static const struct snd_kcontrol_new rt5631_outmixr_mixer_controls[] = {
SOC_DAPM_SINGLE("VDAC Playback Switch", RT5631_OUTMIXER_R_CTRL, 7, 1, 1),
SOC_DAPM_SINGLE("AXIRVOL Playback Switch", RT5631_OUTMIXER_R_CTRL, 8, 1, 1),
SOC_DAPM_SINGLE("AXILVOL Playback Switch", RT5631_OUTMIXER_R_CTRL, 9, 1, 1),
SOC_DAPM_SINGLE("MONOIN_RXN Playback Switch", RT5631_OUTMIXER_R_CTRL, 10, 1, 1),
SOC_DAPM_SINGLE("MIC2_BST2 Playback Switch", RT5631_OUTMIXER_R_CTRL, 11, 1, 1),
SOC_DAPM_SINGLE("MIC1_BST1 Playback Switch", RT5631_OUTMIXER_R_CTRL, 12, 1, 1),
SOC_DAPM_SINGLE("DACR Playback Switch", RT5631_OUTMIXER_R_CTRL, 13, 1, 1),
SOC_DAPM_SINGLE("RECMIXR Playback Switch", RT5631_OUTMIXER_R_CTRL, 14, 1, 1),
SOC_DAPM_SINGLE("RECMIXL Playback Switch", RT5631_OUTMIXER_R_CTRL, 15, 1, 1),
};

static const struct snd_kcontrol_new rt5631_AXO1MIX_mixer_controls[] = {
SOC_DAPM_SINGLE("MIC1_BST1 Playback Switch", RT5631_AXO1MIXER_CTRL, 15 , 1, 1),
SOC_DAPM_SINGLE("MIC2_BST2 Playback Switch", RT5631_AXO1MIXER_CTRL, 11, 1, 1),
SOC_DAPM_SINGLE("OUTVOLL Playback Switch", RT5631_AXO1MIXER_CTRL, 7 , 1 , 1),
SOC_DAPM_SINGLE("OUTVOLR Playback Switch", RT5631_AXO1MIXER_CTRL, 6, 1, 1),
};

static const struct snd_kcontrol_new rt5631_AXO2MIX_mixer_controls[] = {
SOC_DAPM_SINGLE("MIC1_BST1 Playback Switch", RT5631_AXO2MIXER_CTRL, 15, 1, 1),
SOC_DAPM_SINGLE("MIC2_BST2 Playback Switch", RT5631_AXO2MIXER_CTRL, 11, 1, 1),
SOC_DAPM_SINGLE("OUTVOLL Playback Switch", RT5631_AXO2MIXER_CTRL, 7, 1, 1),
SOC_DAPM_SINGLE("OUTVOLR Playback Switch", RT5631_AXO2MIXER_CTRL, 6, 1 , 1),
};

static const struct snd_kcontrol_new rt5631_spolmix_mixer_controls[] = {
SOC_DAPM_SINGLE("SPKVOLL Playback Switch", RT5631_SPK_MONO_OUT_CTRL, 15, 1, 1),
SOC_DAPM_SINGLE("SPKVOLR Playback Switch", RT5631_SPK_MONO_OUT_CTRL, 14, 1, 1),
};

static const struct snd_kcontrol_new rt5631_spormix_mixer_controls[] = {
SOC_DAPM_SINGLE("SPKVOLL Playback Switch", RT5631_SPK_MONO_OUT_CTRL, 13, 1, 1),
SOC_DAPM_SINGLE("SPKVOLR Playback Switch", RT5631_SPK_MONO_OUT_CTRL, 12, 1, 1),
};

static const struct snd_kcontrol_new rt5631_monomix_mixer_controls[] = {
SOC_DAPM_SINGLE("OUTVOLL Playback Switch", RT5631_SPK_MONO_OUT_CTRL, 11, 1, 1),
SOC_DAPM_SINGLE("OUTVOLR Playback Switch", RT5631_SPK_MONO_OUT_CTRL, 10, 1, 1),
};

static const struct snd_kcontrol_new rt5631_spol_mux_control =
SOC_DAPM_ENUM("Route", rt5631_enum[0]);
static const struct snd_kcontrol_new rt5631_spor_mux_control =
SOC_DAPM_ENUM("Route", rt5631_enum[1]);
static const struct snd_kcontrol_new rt5631_mono_mux_control =
SOC_DAPM_ENUM("Route", rt5631_enum[2]);

static const struct snd_kcontrol_new rt5631_hpl_mux_control =
SOC_DAPM_ENUM("Route", rt5631_enum[8]);
static const struct snd_kcontrol_new rt5631_hpr_mux_control =
SOC_DAPM_ENUM("Route", rt5631_enum[9]);

static int spk_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	static int spkl_out_enable, spkr_out_enable;
	unsigned int rt531_dac_pwr = 0;
	unsigned int tf700t_pcb_id = 0;
	unsigned int reg_val;

	tf700t_pcb_id = tegra3_query_pcba_revision_pcbid();
	rt531_dac_pwr = (rt5631_read(codec, RT5631_PWR_MANAG_ADD1) & 0x0300)>>8;

	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		#if ENABLE_ALC
			printk("spk_event --ALC_SND_SOC_DAPM_POST_PMU\n");
			spk_out_flag = true;
			/* Enable ALC */
			switch (project_id) {
			case TEGRA3_PROJECT_TF201:
				rt5631_write(codec,
					RT5631_GEN_PUR_CTRL_REG, 0x6e00);
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0B00);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0000);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0x6410);
				break;
			case TEGRA3_PROJECT_TF300TG:
			case TEGRA3_PROJECT_TF300TL:
				rt5631_write(codec,
					RT5631_GEN_PUR_CTRL_REG, 0x6e00);
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0B00);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0000);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0x6510);
				break;
			case TEGRA3_PROJECT_TF700T:
				rt5631_write(codec,
					RT5631_GEN_PUR_CTRL_REG, 0x7e00);
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0307);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0000);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0x6510);
				break;
			}
		#endif
		#if ENABLE_EQ
			if(rt531_dac_pwr==0x3 ){
				  rt5631_write_mask(codec,RT5631_PWR_MANAG_ADD1, 0x8000, 0x8000); //enable IIS interface power
				  if(project_id == TEGRA3_PROJECT_TF201){
						rt5631_update_eqmode(codec,TF201_PAD);
				  }else if(project_id == TEGRA3_PROJECT_TF300TG){
						rt5631_update_eqmode(codec,TF300TG);
				  }else if(project_id == TEGRA3_PROJECT_TF700T){
						rt5631_update_eqmode(codec,TF700T);
				  }else if(project_id == TEGRA3_PROJECT_TF300TL){
						rt5631_update_eqmode(codec,TF300TL);
				  }else{
						rt5631_update_eqmode(codec,TF201_PAD);
				  }//enable EQ after power on DAC power
			}
		#endif
		if (!spkl_out_enable && !strcmp(w->name, "SPKL Amp")) {

			if(project_id == TEGRA3_PROJECT_TF201){
				rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,0x0700, RT5631_L_VOL_MASK);
			}else if(project_id == TEGRA3_PROJECT_TF300TG){
				rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,0x0700, RT5631_L_VOL_MASK);
			}else if(project_id == TEGRA3_PROJECT_TF700T){
				rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,0x0600, RT5631_L_VOL_MASK);
			}
			if((tf700t_pcb_id == TF700T_PCB_ER1) &&
				(project_id == TEGRA3_PROJECT_TF700T)){
			   rt5631_write_mask(codec,
                               RT5631_SPK_OUT_VOL,0x0d00, RT5631_L_VOL_MASK);
			   printk("%s: %s\n",
                              __func__, "TF700T ER1 spk L ch vol = -7.5dB");
			}

			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD4,
					PWR_SPK_L_VOL, PWR_SPK_L_VOL);
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1,
					PWR_CLASS_D, PWR_CLASS_D);
			rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,
					0, RT_L_MUTE);
			spkl_out_enable = 1;
		}
		if (!spkr_out_enable && !strcmp(w->name, "SPKR Amp")) {

			if(project_id == TEGRA3_PROJECT_TF201){
				rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,0x0007, RT5631_R_VOL_MASK);
			}else if(project_id == TEGRA3_PROJECT_TF300TG){
				rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,0x0007, RT5631_R_VOL_MASK);
			}else if(project_id == TEGRA3_PROJECT_TF700T){
				rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,0x0006, RT5631_R_VOL_MASK);
			}
			if((tf700t_pcb_id == TF700T_PCB_ER1) &&
				(project_id  == TEGRA3_PROJECT_TF700T)){
                            rt5631_write_mask(codec,
                                RT5631_SPK_OUT_VOL,0x000d, RT5631_R_VOL_MASK);
                            printk("%s: %s\n",
                               __func__, "TF700T ER1 spk R ch vol = -7.5dB");
			}

			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD4,
					PWR_SPK_R_VOL, PWR_SPK_R_VOL);
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1,
					PWR_CLASS_D, PWR_CLASS_D);
			rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,
					0, RT_R_MUTE);
			spkr_out_enable = 1;
		}
		break;

	case SND_SOC_DAPM_POST_PMD:
		if (spkl_out_enable && !strcmp(w->name, "SPKL Amp")) {
			rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,
					RT_L_MUTE, RT_L_MUTE);
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD4,
					0, PWR_SPK_L_VOL);
			spkl_out_enable = 0;
		}
		if (spkr_out_enable && !strcmp(w->name, "SPKR Amp")) {
			rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,
					RT_R_MUTE, RT_R_MUTE);
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD4,
					0, PWR_SPK_R_VOL);
			spkr_out_enable = 0;
		}
		if (0 == spkl_out_enable && 0 == spkr_out_enable)
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1,
					0, PWR_CLASS_D);
		break;

	case SND_SOC_DAPM_PRE_PMD:
		#if ENABLE_ALC
		printk("spk_event --ALC_SND_SOC_DAPM_PRE_PMD\n");
		spk_out_flag = false;
		if(!spk_out_flag && !ADC_flag){
			//Disable ALC
			rt5631_write_mask(codec, RT5631_ALC_CTRL_3, 0x2000,0xf000);
		}else if(!spk_out_flag && DMIC_flag ){
			if(project_id == TEGRA3_PROJECT_TF700T){
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x000e);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe099);
			}else{
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0006);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe09a);
			}
		}else if(!spk_out_flag && !DMIC_flag ){
			if(project_id == TEGRA3_PROJECT_TF700T){
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x000a);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe090);
			}else{
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0004);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe084);
			}
		}
		#endif
		#if ENABLE_EQ
			printk("spk_event --EQ_SND_SOC_DAPM_PRE_PMD\n");
		if (rt531_dac_pwr==0x3){
			rt5631_update_eqmode(codec,NORMAL);    //disable EQ before powerdown speaker power
		}
		#endif
		break;

	default:
		return 0;
	}

	if(project_id == TEGRA3_PROJECT_TF700T ||
		project_id == TEGRA3_PROJECT_TF300TG ||
		project_id == TEGRA3_PROJECT_TF300TL){
		rt5631_write_index(codec, 0x48, 0xF73C);
		reg_val = rt5631_read_index(codec, 0x48);
		printk("%s -codec index 0x48=0x%04X\n", __FUNCTION__, reg_val);
	}

	return 0;
}


static void hp_depop_mode2_onebit(struct snd_soc_codec *codec, int enable)
{
	unsigned int soft_vol, hp_zc;

	rt5631_write_mask(codec, RT5631_DEPOP_FUN_CTRL_2, 0, EN_ONE_BIT_DEPOP);

	soft_vol = rt5631_read(codec, RT5631_SOFT_VOL_CTRL);
	rt5631_write(codec, RT5631_SOFT_VOL_CTRL, 0);
	hp_zc = rt5631_read(codec, RT5631_INT_ST_IRQ_CTRL_2);
	rt5631_write(codec, RT5631_INT_ST_IRQ_CTRL_2, hp_zc & 0xf7ff);
	if (enable) {
		rt5631_write_index(codec, RT5631_TEST_MODE_CTRL, 0x84c0);
		rt5631_write_index(codec, RT5631_SPK_INTL_CTRL, 0x309f);
		rt5631_write_index(codec, RT5631_CP_INTL_REG2, 0x6530);
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_2,
				EN_CAP_FREE_DEPOP);
	} else {
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_2, 0);
		schedule_timeout_uninterruptible(msecs_to_jiffies(100));
	}

	rt5631_write(codec, RT5631_SOFT_VOL_CTRL, soft_vol);
	rt5631_write(codec, RT5631_INT_ST_IRQ_CTRL_2, hp_zc);

	return;
}

static void hp_mute_unmute_depop_onebit(struct snd_soc_codec *codec, int enable)
{
	unsigned int soft_vol, hp_zc;

	rt5631_write_mask(codec, RT5631_DEPOP_FUN_CTRL_2, 0, EN_ONE_BIT_DEPOP);
	soft_vol = rt5631_read(codec, RT5631_SOFT_VOL_CTRL);
	rt5631_write(codec, RT5631_SOFT_VOL_CTRL, 0);
	hp_zc = rt5631_read(codec, RT5631_INT_ST_IRQ_CTRL_2);
	rt5631_write(codec, RT5631_INT_ST_IRQ_CTRL_2, hp_zc & 0xf7ff);
	if (enable) {
		schedule_timeout_uninterruptible(msecs_to_jiffies(10));
		rt5631_write_index(codec, RT5631_SPK_INTL_CTRL, 0x307f);
		rt5631_write_mask(codec, RT5631_HP_OUT_VOL, 0,
				RT_L_MUTE | RT_R_MUTE);
		schedule_timeout_uninterruptible(msecs_to_jiffies(300));

	} else {
		rt5631_write_mask(codec, RT5631_HP_OUT_VOL,
			RT_L_MUTE | RT_R_MUTE, RT_L_MUTE | RT_R_MUTE);
		schedule_timeout_uninterruptible(msecs_to_jiffies(100));
	}
	rt5631_write(codec, RT5631_SOFT_VOL_CTRL, soft_vol);
	rt5631_write(codec, RT5631_INT_ST_IRQ_CTRL_2, hp_zc);

	return;
}

static void hp_depop2(struct snd_soc_codec *codec, int enable)
{
	unsigned int soft_vol, hp_zc;

	rt5631_write_mask(codec, RT5631_DEPOP_FUN_CTRL_2,
		EN_ONE_BIT_DEPOP, EN_ONE_BIT_DEPOP);
	soft_vol = rt5631_read(codec, RT5631_SOFT_VOL_CTRL);
	rt5631_write(codec, RT5631_SOFT_VOL_CTRL, 0);
	hp_zc = rt5631_read(codec, RT5631_INT_ST_IRQ_CTRL_2);
	rt5631_write(codec, RT5631_INT_ST_IRQ_CTRL_2, hp_zc & 0xf7ff);
	if (enable) {
		rt5631_write_index(codec, RT5631_SPK_INTL_CTRL, 0x303e);
		rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3,
			PWR_CHARGE_PUMP | PWR_HP_L_AMP | PWR_HP_R_AMP,
			PWR_CHARGE_PUMP | PWR_HP_L_AMP | PWR_HP_R_AMP);
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_1,
			POW_ON_SOFT_GEN | EN_DEPOP2_FOR_HP);
		schedule_timeout_uninterruptible(msecs_to_jiffies(100));
		rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3,
			PWR_HP_DEPOP_DIS, PWR_HP_DEPOP_DIS);
	} else {
		rt5631_write_index(codec, RT5631_SPK_INTL_CTRL, 0x303F);
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_1,
			POW_ON_SOFT_GEN | EN_MUTE_UNMUTE_DEPOP |
			PD_HPAMP_L_ST_UP | PD_HPAMP_R_ST_UP);
		schedule_timeout_uninterruptible(msecs_to_jiffies(75));
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_1,
			POW_ON_SOFT_GEN | PD_HPAMP_L_ST_UP | PD_HPAMP_R_ST_UP);
		rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3, 0,
					PWR_HP_DEPOP_DIS);
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_1,
			POW_ON_SOFT_GEN | EN_DEPOP2_FOR_HP |
			PD_HPAMP_L_ST_UP | PD_HPAMP_R_ST_UP);
		schedule_timeout_uninterruptible(msecs_to_jiffies(80));
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_1, POW_ON_SOFT_GEN);
		rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3, 0,
			PWR_CHARGE_PUMP | PWR_HP_L_AMP | PWR_HP_R_AMP);
	}

	rt5631_write(codec, RT5631_SOFT_VOL_CTRL, soft_vol);
	rt5631_write(codec, RT5631_INT_ST_IRQ_CTRL_2, hp_zc);

	return;
}

static void hp_mute_unmute_depop(struct snd_soc_codec *codec, int enable)
{
	unsigned int soft_vol, hp_zc;

	rt5631_write_mask(codec, RT5631_DEPOP_FUN_CTRL_2,
		EN_ONE_BIT_DEPOP, EN_ONE_BIT_DEPOP);
	soft_vol = rt5631_read(codec, RT5631_SOFT_VOL_CTRL);
	rt5631_write(codec, RT5631_SOFT_VOL_CTRL, 0);
	hp_zc = rt5631_read(codec, RT5631_INT_ST_IRQ_CTRL_2);
	rt5631_write(codec, RT5631_INT_ST_IRQ_CTRL_2, hp_zc & 0xf7ff);
	if (enable) {
		schedule_timeout_uninterruptible(msecs_to_jiffies(10));
		rt5631_write_index(codec, RT5631_SPK_INTL_CTRL, 0x302f);
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_1,
			POW_ON_SOFT_GEN | EN_MUTE_UNMUTE_DEPOP |
			EN_HP_R_M_UN_MUTE_DEPOP | EN_HP_L_M_UN_MUTE_DEPOP);
		rt5631_write_mask(codec, RT5631_HP_OUT_VOL, 0,
				RT_L_MUTE | RT_R_MUTE);
		schedule_timeout_uninterruptible(msecs_to_jiffies(160));
	} else {
		rt5631_write_index(codec, RT5631_SPK_INTL_CTRL, 0x302f);
		rt5631_write(codec, RT5631_DEPOP_FUN_CTRL_1,
			POW_ON_SOFT_GEN | EN_MUTE_UNMUTE_DEPOP |
			EN_HP_R_M_UN_MUTE_DEPOP | EN_HP_L_M_UN_MUTE_DEPOP);
		rt5631_write_mask(codec, RT5631_HP_OUT_VOL,
			RT_L_MUTE | RT_R_MUTE, RT_L_MUTE | RT_R_MUTE);
		schedule_timeout_uninterruptible(msecs_to_jiffies(150));
	}

	rt5631_write(codec, RT5631_SOFT_VOL_CTRL, soft_vol);
	rt5631_write(codec, RT5631_INT_ST_IRQ_CTRL_2, hp_zc);

	return;
}

static int hp_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);
	static bool hp_en;
	int pu_l, pu_r;

	pu_l = rt5631_read(codec, RT5631_PWR_MANAG_ADD4) & PWR_HP_L_OUT_VOL;
	pu_r = rt5631_read(codec, RT5631_PWR_MANAG_ADD4) & PWR_HP_R_OUT_VOL;
	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		if ((pu_l && pu_r) && hp_en) {
			if (rt5631->codec_version) {
				hp_mute_unmute_depop_onebit(codec, 0);
				hp_depop_mode2_onebit(codec, 0);
			} else {
				hp_mute_unmute_depop(codec, 0);
				hp_depop2(codec, 0);
			}
			hp_en = false;
		}
		break;

	case SND_SOC_DAPM_POST_PMU:
		if ((pu_l && pu_r) && !hp_en) {
			if (rt5631->codec_version) {
				hp_depop_mode2_onebit(codec, 1);
				hp_mute_unmute_depop_onebit(codec, 1);
			} else {
				hp_depop2(codec, 1);
				hp_mute_unmute_depop(codec, 1);
			}
			hp_en = true;
		}
		break;

	default:
		break;
	}

	return 0;
}

static int dac_to_hp_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);
	static bool hp_en;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:

		if (hp_en) {
			if (rt5631->codec_version) {
				hp_mute_unmute_depop_onebit(codec, 0);
				hp_depop_mode2_onebit(codec, 0);
			} else {
				hp_mute_unmute_depop(codec, 0);
				hp_depop2(codec, 0);
			}
			hp_en = false;
		}
		break;

	case SND_SOC_DAPM_POST_PMU:
		if (!hp_en) {
			if (rt5631->codec_version) {
				hp_depop_mode2_onebit(codec, 1);
				hp_mute_unmute_depop_onebit(codec, 1);
			} else {
				hp_depop2(codec, 1);
				hp_mute_unmute_depop(codec, 1);
			}
			hp_en = true;
		}
		break;

	default:
		break;
	}

	return 0;
}

static int mic_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	int val_mic1, val_mic2;

	val_mic1 = rt5631_read(codec, RT5631_PWR_MANAG_ADD2) &
				PWR_MIC1_BOOT_GAIN;
	val_mic2 = rt5631_read(codec, RT5631_PWR_MANAG_ADD2) &
				PWR_MIC2_BOOT_GAIN;
	switch (event) {
	case SND_SOC_DAPM_POST_PMU:
		/*
		 * If microphone is stereo, need not copy ADC channel
		 * If mic1 is used, copy ADC left to right
		 * If mic2 is used, copy ADC right to left
		 */
		if (val_mic1 && val_mic2)
			rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2,
							0x0000, 0xc000);
		else if (val_mic1)
			rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2,
							0x4000, 0xc000);
		else if (val_mic2)
			rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2,
							0x8000, 0xc000);
		else
			rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2,
							0x0000, 0xc000);
		break;

	default:
		break;
	}

	return 0;
}

static int auxo1_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	static bool aux1_en;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		if (aux1_en) {
			rt5631_write_mask(codec, RT5631_MONO_AXO_1_2_VOL,
						RT_L_MUTE, RT_L_MUTE);
			aux1_en = false;
		}
		break;

	case SND_SOC_DAPM_POST_PMU:
		if (!aux1_en) {
			rt5631_write_mask(codec, RT5631_MONO_AXO_1_2_VOL,
						0, RT_L_MUTE);
			aux1_en = true;
		}
		break;

	default:
		break;
	}

	return 0;
}

static int auxo2_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	static bool aux2_en;
	char mute_all_audioDock[2] = {0xFF, 0x01};
	char unmute_all_audioDock[2] = {0x00, 0x01};

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		asusAudiodec_i2c_write_data(mute_all_audioDock, 2);
		if (aux2_en) {
			rt5631_write_mask(codec, RT5631_MONO_AXO_1_2_VOL,
						RT_R_MUTE, RT_R_MUTE);
			aux2_en = false;
		}
		break;

	case SND_SOC_DAPM_POST_PMU:
		if (!aux2_en) {
			rt5631_write_mask(codec, RT5631_MONO_AXO_1_2_VOL,
						0, RT_R_MUTE);
			aux2_en = true;
		}
		asusAudiodec_i2c_write_data(unmute_all_audioDock, 2);
		break;

	default:
		break;
	}

	return 0;
}

static int mono_event(struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	static bool mono_en;

	switch (event) {
	case SND_SOC_DAPM_PRE_PMD:
		if (mono_en) {
			rt5631_write_mask(codec, RT5631_MONO_AXO_1_2_VOL,
						MUTE_MONO, MUTE_MONO);
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3,
						0, PWR_MONO_DEPOP_DIS);
			mono_en = false;
		}
		break;

	case SND_SOC_DAPM_POST_PMU:
		if (!mono_en) {
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3,
				PWR_MONO_DEPOP_DIS, PWR_MONO_DEPOP_DIS);
			rt5631_write_mask(codec, RT5631_MONO_AXO_1_2_VOL,
						0, MUTE_MONO);
			mono_en = true;
		}
		break;

	default:
		break;
	}

	return 0;
}

/**
 * config_common_power - control all common power of codec system
 * @pmu: power up or not
 */
static int config_common_power(struct snd_soc_codec *codec, bool pmu)
{
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);
	unsigned int mux_val;
	static int ref_count = 0;

	if (pmu) {
		ref_count++;
		rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1,
			PWR_MAIN_I2S_EN | PWR_DAC_REF,
			PWR_MAIN_I2S_EN | PWR_DAC_REF);
		mux_val = rt5631_read(codec, RT5631_SPK_MONO_HP_OUT_CTRL);
		if (!(mux_val & HP_L_MUX_SEL_DAC_L))
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1,
				PWR_DAC_L_TO_MIXER, PWR_DAC_L_TO_MIXER);
		if (!(mux_val & HP_R_MUX_SEL_DAC_R))
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1,
				PWR_DAC_R_TO_MIXER, PWR_DAC_R_TO_MIXER);
		if (rt5631->pll_used_flag)
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD2,
						PWR_PLL, PWR_PLL);
	} else{
		ref_count--;
		if(ref_count == 0){
			printk("%s: Real powr down, ref_count = 0\n", __func__);
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD1, 0,
				PWR_MAIN_I2S_EN | PWR_DAC_REF |
				PWR_DAC_L_TO_MIXER | PWR_DAC_R_TO_MIXER);
		}
		if (rt5631->pll_used_flag)
			rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD2,
						0, PWR_PLL);
	}

	return 0;
}

static int adc_event(struct snd_soc_dapm_widget *w,
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
		printk("adc_event --ALC_SND_SOC_DAPM_POST_PMU\n");
		ADC_flag = true;
		if(!spk_out_flag && DMIC_flag ){
			if(project_id == TEGRA3_PROJECT_TF700T){
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x000e);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe099);
			}else{
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0006);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe09a);
			}
		}else if(!spk_out_flag && !DMIC_flag ){
			if(project_id == TEGRA3_PROJECT_TF700T){
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x000a);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe090);
				rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x0005, 0x001f);
			}else{
				rt5631_write(codec, RT5631_ALC_CTRL_1, 0x0207);
				rt5631_write(codec, RT5631_ALC_CTRL_2, 0x0004);
				rt5631_write(codec, RT5631_ALC_CTRL_3, 0xe084);
			}
		}
		msleep(DEPOP_DELAY);
		rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x0000, 0x8080);

		#endif
		break;

	case SND_SOC_DAPM_PRE_PMD:
		#if ENABLE_ALC
		printk("adc_event --ALC_SND_SOC_DAPM_PRE_PMD\n");
		ADC_flag = false;
		if(!spk_out_flag ){
			//Disable ALC
			rt5631_write_mask(codec, RT5631_ALC_CTRL_3, 0x2000,0xf000);
			}
		#endif
		rt5631_write_mask(codec, RT5631_ADC_CTRL_1, 0x8080, 0x8080);

		break;

	default:
		break;
	}

	return 0;
}

static int dac_event(struct snd_soc_dapm_widget *w,
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
		printk("dac_event --ALC_SND_SOC_DAPM_PRE_PMD\n");
		if(!spk_out_flag && !ADC_flag ){
			//Disable ALC
			rt5631_write_mask(codec, RT5631_ALC_CTRL_3, 0x2000,0xf000);
		}
		#endif
	break;

	default:
		break;
	}

	return 0;
}

static const struct snd_soc_dapm_widget rt5631_dapm_widgets[] = {
SND_SOC_DAPM_INPUT("DMIC"),
SND_SOC_DAPM_INPUT("MIC1"),
SND_SOC_DAPM_INPUT("MIC2"),
SND_SOC_DAPM_INPUT("AXIL"),
SND_SOC_DAPM_INPUT("AXIR"),
SND_SOC_DAPM_INPUT("MONOIN_RXN"),
SND_SOC_DAPM_INPUT("MONOIN_RXP"),

SND_SOC_DAPM_MICBIAS("Mic Bias1", RT5631_PWR_MANAG_ADD2, 3, 0),
SND_SOC_DAPM_MICBIAS("Mic Bias2", RT5631_PWR_MANAG_ADD2, 2, 0),

SND_SOC_DAPM_PGA_E("Mic1 Boost", RT5631_PWR_MANAG_ADD2, 5, 0, NULL, 0,
		mic_event, SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("Mic2 Boost", RT5631_PWR_MANAG_ADD2, 4, 0, NULL, 0,
		mic_event, SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA("MONOIN_RXP Boost", RT5631_PWR_MANAG_ADD4, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("MONOIN_RXN Boost", RT5631_PWR_MANAG_ADD4, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("AXIL Boost", RT5631_PWR_MANAG_ADD4, 9, 0, NULL, 0),
SND_SOC_DAPM_PGA("AXIR Boost", RT5631_PWR_MANAG_ADD4, 8, 0, NULL, 0),
SND_SOC_DAPM_MIXER("MONO_IN", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_MIXER("RECMIXL Mixer", RT5631_PWR_MANAG_ADD2, 11, 0,
		&rt5631_recmixl_mixer_controls[0],
		ARRAY_SIZE(rt5631_recmixl_mixer_controls)),
SND_SOC_DAPM_MIXER("RECMIXR Mixer", RT5631_PWR_MANAG_ADD2, 10, 0,
		&rt5631_recmixr_mixer_controls[0],
		ARRAY_SIZE(rt5631_recmixr_mixer_controls)),
SND_SOC_DAPM_MIXER("ADC Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_ADC_E("Left ADC", "Left ADC HIFI Capture",
		RT5631_PWR_MANAG_ADD1, 11, 0,
		adc_event, SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_ADC_E("Right ADC", "Right ADC HIFI Capture",
		RT5631_PWR_MANAG_ADD1, 10, 0,
		adc_event, SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_DAC_E("Left DAC", "Left DAC HIFI Playback",
		RT5631_PWR_MANAG_ADD1, 9, 0,
		dac_event, SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_DAC_E("Right DAC", "Right DAC HIFI Playback",
		RT5631_PWR_MANAG_ADD1, 8, 0,
		dac_event, SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_DAC("Voice DAC", "Voice DAC Mono Playback", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_PGA("Voice DAC Boost", SND_SOC_NOPM, 0, 0, NULL, 0),

SND_SOC_DAPM_MIXER("SPKMIXL Mixer", RT5631_PWR_MANAG_ADD2, 13, 0,
		&rt5631_spkmixl_mixer_controls[0],
		ARRAY_SIZE(rt5631_spkmixl_mixer_controls)),
SND_SOC_DAPM_MIXER("OUTMIXL Mixer", RT5631_PWR_MANAG_ADD2, 15, 0,
		&rt5631_outmixl_mixer_controls[0],
		ARRAY_SIZE(rt5631_outmixl_mixer_controls)),
SND_SOC_DAPM_MIXER("OUTMIXR Mixer", RT5631_PWR_MANAG_ADD2, 14, 0,
		&rt5631_outmixr_mixer_controls[0],
		ARRAY_SIZE(rt5631_outmixr_mixer_controls)),
SND_SOC_DAPM_MIXER("SPKMIXR Mixer", RT5631_PWR_MANAG_ADD2, 12, 0,
		&rt5631_spkmixr_mixer_controls[0],
		ARRAY_SIZE(rt5631_spkmixr_mixer_controls)),

SND_SOC_DAPM_PGA("Left SPK Vol", RT5631_PWR_MANAG_ADD4, 15, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right SPK Vol", RT5631_PWR_MANAG_ADD4, 14, 0, NULL, 0),
SND_SOC_DAPM_PGA_E("Left HP Vol", RT5631_PWR_MANAG_ADD4, 11, 0, NULL, 0,
		hp_event, SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("Right HP Vol", RT5631_PWR_MANAG_ADD4, 10, 0, NULL, 0,
		hp_event, SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_PGA_E("Left DAC_HP", SND_SOC_NOPM, 0, 0, NULL, 0,
	dac_to_hp_event, SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("Right DAC_HP", SND_SOC_NOPM, 0, 0, NULL, 0,
	dac_to_hp_event, SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_PGA("Left Out Vol", RT5631_PWR_MANAG_ADD4, 13, 0, NULL, 0),
SND_SOC_DAPM_PGA("Right Out Vol", RT5631_PWR_MANAG_ADD4, 12, 0, NULL, 0),

SND_SOC_DAPM_MIXER_E("AXO1MIX Mixer", RT5631_PWR_MANAG_ADD3, 11, 0,
		&rt5631_AXO1MIX_mixer_controls[0],
		ARRAY_SIZE(rt5631_AXO1MIX_mixer_controls),
		auxo1_event, SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_MIXER("SPOLMIX Mixer", SND_SOC_NOPM, 0, 0,
		&rt5631_spolmix_mixer_controls[0],
		ARRAY_SIZE(rt5631_spolmix_mixer_controls)),
SND_SOC_DAPM_MIXER("MONOMIX Mixer", RT5631_PWR_MANAG_ADD3, 9, 0,
		&rt5631_monomix_mixer_controls[0],
		ARRAY_SIZE(rt5631_monomix_mixer_controls)),
SND_SOC_DAPM_MIXER("SPORMIX Mixer", SND_SOC_NOPM, 0, 0,
		&rt5631_spormix_mixer_controls[0],
		ARRAY_SIZE(rt5631_spormix_mixer_controls)),
SND_SOC_DAPM_MIXER_E("AXO2MIX Mixer", RT5631_PWR_MANAG_ADD3, 10, 0,
		&rt5631_AXO2MIX_mixer_controls[0],
		ARRAY_SIZE(rt5631_AXO2MIX_mixer_controls),
		auxo2_event, SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),

SND_SOC_DAPM_MUX("SPOL Mux", SND_SOC_NOPM, 0, 0, &rt5631_spol_mux_control),
SND_SOC_DAPM_MUX("SPOR Mux", SND_SOC_NOPM, 0, 0, &rt5631_spor_mux_control),
SND_SOC_DAPM_MUX("Mono Mux", SND_SOC_NOPM, 0, 0, &rt5631_mono_mux_control),
SND_SOC_DAPM_MUX("HPL Mux", SND_SOC_NOPM, 0, 0, &rt5631_hpl_mux_control),
SND_SOC_DAPM_MUX("HPR Mux", SND_SOC_NOPM, 0, 0, &rt5631_hpr_mux_control),

SND_SOC_DAPM_PGA_E("Mono Amp", RT5631_PWR_MANAG_ADD3, 7, 0, NULL, 0,
		mono_event, SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMU),
SND_SOC_DAPM_PGA_E("SPKL Amp", SND_SOC_NOPM, 0, 0, NULL, 0,
		spk_event, SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU |SND_SOC_DAPM_PRE_PMD),
SND_SOC_DAPM_PGA_E("SPKR Amp", SND_SOC_NOPM, 1, 0, NULL, 0,
		spk_event, SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD),

SND_SOC_DAPM_OUTPUT("AUXO1"),
SND_SOC_DAPM_OUTPUT("AUXO2"),
SND_SOC_DAPM_OUTPUT("SPOL"),
SND_SOC_DAPM_OUTPUT("SPOR"),
SND_SOC_DAPM_OUTPUT("HPOL"),
SND_SOC_DAPM_OUTPUT("HPOR"),
SND_SOC_DAPM_OUTPUT("MONO"),
};


static const struct snd_soc_dapm_route audio_map[] = {
	{"Mic1 Boost", NULL, "MIC1"},
	{"Mic2 Boost", NULL, "MIC2"},
	{"MONOIN_RXP Boost", NULL, "MONOIN_RXP"},
	{"MONOIN_RXN Boost", NULL, "MONOIN_RXN"},
	{"AXIL Boost", NULL, "AXIL"},
	{"AXIR Boost", NULL, "AXIR"},

	{"MONO_IN", NULL, "MONOIN_RXP Boost"},
	{"MONO_IN", NULL, "MONOIN_RXN Boost"},

	{"RECMIXL Mixer", "OUTMIXL Capture Switch", "OUTMIXL Mixer"},
	{"RECMIXL Mixer", "MIC1_BST1 Capture Switch", "Mic1 Boost"},
	{"RECMIXL Mixer", "AXILVOL Capture Switch", "AXIL Boost"},
	{"RECMIXL Mixer", "MONOIN_RX Capture Switch", "MONO_IN"},

	{"RECMIXR Mixer", "OUTMIXR Capture Switch", "OUTMIXR Mixer"},
	{"RECMIXR Mixer", "MIC2_BST2 Capture Switch", "Mic2 Boost"},
	{"RECMIXR Mixer", "AXIRVOL Capture Switch", "AXIR Boost"},
	{"RECMIXR Mixer", "MONOIN_RX Capture Switch", "MONO_IN"},

	{"ADC Mixer", NULL, "DMIC"},
	{"ADC Mixer", NULL, "RECMIXL Mixer"},
	{"ADC Mixer", NULL, "RECMIXR Mixer"},
	{"Left ADC", NULL, "ADC Mixer"},
	{"Right ADC", NULL, "ADC Mixer"},

	{"Voice DAC Boost", NULL, "Voice DAC"},

	{"SPKMIXL Mixer", "RECMIXL Playback Switch", "RECMIXL Mixer"},
	{"SPKMIXL Mixer", "MIC1_P Playback Switch", "MIC1"},
	{"SPKMIXL Mixer", "DACL Playback Switch", "Left DAC"},
	{"SPKMIXL Mixer", "OUTMIXL Playback Switch", "OUTMIXL Mixer"},

	{"SPKMIXR Mixer", "OUTMIXR Playback Switch", "OUTMIXR Mixer"},
	{"SPKMIXR Mixer", "DACR Playback Switch", "Right DAC"},
	{"SPKMIXR Mixer", "MIC2_P Playback Switch", "MIC2"},
	{"SPKMIXR Mixer", "RECMIXR Playback Switch", "RECMIXR Mixer"},

	{"OUTMIXL Mixer", "RECMIXL Playback Switch", "RECMIXL Mixer"},
	{"OUTMIXL Mixer", "RECMIXR Playback Switch", "RECMIXR Mixer"},
	{"OUTMIXL Mixer", "DACL Playback Switch", "Left DAC"},
	{"OUTMIXL Mixer", "MIC1_BST1 Playback Switch", "Mic1 Boost"},
	{"OUTMIXL Mixer", "MIC2_BST2 Playback Switch", "Mic2 Boost"},
	{"OUTMIXL Mixer", "MONOIN_RXP Playback Switch", "MONOIN_RXP Boost"},
	{"OUTMIXL Mixer", "AXILVOL Playback Switch", "AXIL Boost"},
	{"OUTMIXL Mixer", "AXIRVOL Playback Switch", "AXIR Boost"},
	{"OUTMIXL Mixer", "VDAC Playback Switch", "Voice DAC Boost"},

	{"OUTMIXR Mixer", "RECMIXL Playback Switch", "RECMIXL Mixer"},
	{"OUTMIXR Mixer", "RECMIXR Playback Switch", "RECMIXR Mixer"},
	{"OUTMIXR Mixer", "DACR Playback Switch", "Right DAC"},
	{"OUTMIXR Mixer", "MIC1_BST1 Playback Switch", "Mic1 Boost"},
	{"OUTMIXR Mixer", "MIC2_BST2 Playback Switch", "Mic2 Boost"},
	{"OUTMIXR Mixer", "MONOIN_RXN Playback Switch", "MONOIN_RXN Boost"},
	{"OUTMIXR Mixer", "AXILVOL Playback Switch", "AXIL Boost"},
	{"OUTMIXR Mixer", "AXIRVOL Playback Switch", "AXIR Boost"},
	{"OUTMIXR Mixer", "VDAC Playback Switch", "Voice DAC Boost"},

	{"Left SPK Vol",  NULL, "SPKMIXL Mixer"},
	{"Right SPK Vol",  NULL, "SPKMIXR Mixer"},
	{"Left HP Vol",  NULL, "OUTMIXL Mixer"},
	{"Left Out Vol",  NULL, "OUTMIXL Mixer"},
	{"Right Out Vol",  NULL, "OUTMIXR Mixer"},
	{"Right HP Vol",  NULL, "OUTMIXR Mixer"},

	{"AXO1MIX Mixer", "MIC1_BST1 Playback Switch", "Mic1 Boost"},
	{"AXO1MIX Mixer", "OUTVOLL Playback Switch", "Left Out Vol"},
	{"AXO1MIX Mixer", "OUTVOLR Playback Switch", "Right Out Vol"},
	{"AXO1MIX Mixer", "MIC2_BST2 Playback Switch", "Mic2 Boost"},

	{"AXO2MIX Mixer", "MIC1_BST1 Playback Switch", "Mic1 Boost"},
	{"AXO2MIX Mixer", "OUTVOLL Playback Switch", "Left Out Vol"},
	{"AXO2MIX Mixer", "OUTVOLL Playback Switch", "Right Out Vol"},
	{"AXO2MIX Mixer", "OUTVOLR Playback Switch", "Right Out Vol"},
	{"AXO2MIX Mixer", "MIC2_BST2 Playback Switch", "Mic2 Boost"},

	{"SPOLMIX Mixer", "SPKVOLL Playback Switch", "Left SPK Vol"},
	{"SPOLMIX Mixer", "SPKVOLR Playback Switch", "Right SPK Vol"},

	{"SPORMIX Mixer", "SPKVOLL Playback Switch", "Left SPK Vol"},
	{"SPORMIX Mixer", "SPKVOLR Playback Switch", "Right SPK Vol"},

	{"MONOMIX Mixer", "OUTVOLL Playback Switch", "Left Out Vol"},
	{"MONOMIX Mixer", "OUTVOLR Playback Switch", "Right Out Vol"},

	{"SPOL Mux", "SPOLMIX", "SPOLMIX Mixer"},
	{"SPOL Mux", "MONOIN_RX", "MONO_IN"},
	{"SPOL Mux", "VDAC", "Voice DAC Boost"},
	{"SPOL Mux", "DACL", "Left DAC"},

	{"SPOR Mux", "SPORMIX", "SPORMIX Mixer"},
	{"SPOR Mux", "MONOIN_RX", "MONO_IN"},
	{"SPOR Mux", "VDAC", "Voice DAC Boost"},
	{"SPOR Mux", "DACR", "Right DAC"},

	{"Mono Mux", "MONOMIX", "MONOMIX Mixer"},
	{"Mono Mux", "MONOIN_RX", "MONO_IN"},
	{"Mono Mux", "VDAC", "Voice DAC Boost"},

	{"Right DAC_HP", "NULL", "Right DAC"},
	{"Left DAC_HP", "NULL", "Left DAC"},

	{"HPL Mux", "LEFT HPVOL", "Left HP Vol"},
	{"HPL Mux", "LEFT DAC", "Left DAC_HP"},
	{"HPR Mux", "RIGHT HPVOL", "Right HP Vol"},
	{"HPR Mux", "RIGHT DAC", "Right DAC_HP"},

	{"SPKL Amp", NULL, "SPOL Mux"},
	{"SPKR Amp", NULL, "SPOR Mux"},
	{"Mono Amp", NULL, "Mono Mux"},

	{"AUXO1", NULL, "AXO1MIX Mixer"},
	{"AUXO2", NULL, "AXO2MIX Mixer"},
	{"SPOL", NULL, "SPKL Amp"},
	{"SPOR", NULL, "SPKR Amp"},

	{"HPOL", NULL, "HPL Mux"},
	{"HPOR", NULL, "HPR Mux"},

	{"MONO", NULL, "Mono Amp"}
};

static int rt5631_add_widgets(struct snd_soc_codec *codec)
{
	struct snd_soc_dapm_context *dapm = &codec->dapm;

	snd_soc_dapm_new_controls(dapm, rt5631_dapm_widgets,
			ARRAY_SIZE(rt5631_dapm_widgets));
	snd_soc_dapm_add_routes(dapm, audio_map, ARRAY_SIZE(audio_map));

	return 0;
}

struct coeff_clk_div {
	u32 mclk;
	u32 bclk;
	u32 rate;
	u16 reg_val;
};

/* PLL divisors yes*/
struct pll_div {
	u32 pll_in;
	u32 pll_out;
	u16 reg_val;
};

static const struct pll_div codec_master_pll_div[] = {
	{2048000,  8192000,  0x0ea0},
	{3686400,  8192000,  0x4e27},
	{12000000,  8192000,  0x456b},
	{13000000,  8192000,  0x495f},
	{13100000,  8192000,  0x0320},
	{2048000,  11289600,  0xf637},
	{3686400,  11289600,  0x2f22},
	{12000000,  11289600,  0x3e2f},
	{13000000,  11289600,  0x4d5b},
	{13100000,  11289600,  0x363b},
	{2048000,  16384000,  0x1ea0},
	{3686400,  16384000,  0x9e27},
	{12000000,  16384000,  0x452b},
	{13000000,  16384000,  0x542f},
	{13100000,  16384000,  0x03a0},
	{2048000,  16934400,  0xe625},
	{3686400,  16934400,  0x9126},
	{12000000,  16934400,  0x4d2c},
	{13000000,  16934400,  0x742f},
	{13100000,  16934400,  0x3c27},
	{2048000,  22579200,  0x2aa0},
	{3686400,  22579200,  0x2f20},
	{12000000,  22579200,  0x7e2f},
	{13000000,  22579200,  0x742f},
	{13100000,  22579200,  0x3c27},
	{2048000,  24576000,  0x2ea0},
	{3686400,  24576000,  0xee27},
	{12000000,  24576000,  0x2915},
	{13000000,  24576000,  0x772e},
	{13100000,  24576000,  0x0d20},
	{26000000,  24576000,  0x2027},
	{26000000,  22579200,  0x392f},
	{24576000,  22579200,  0x0921},
	{24576000,  24576000,  0x02a0},
};

static const struct pll_div codec_slave_pll_div[] = {
	{256000,  2048000,  0x46f0},
	{256000,  4096000,  0x3ea0},
	{352800,	 5644800,  0x3ea0},
	{512000,	 8192000,  0x3ea0},
	{1024000,  8192000,  0x46f0},
	{705600,  11289600,  0x3ea0},
	{1024000,  16384000,  0x3ea0},
	{1411200,  22579200,  0x3ea0},
	{1536000,  24576000,  0x3ea0},
	{2048000,  16384000,  0x1ea0},
	{2822400,  22579200,  0x1ea0},
	{2822400,  45158400,  0x5ec0},
	{5644800,  45158400,  0x46f0},
	{3072000,  24576000,  0x1ea0},
	{3072000,  49152000,  0x5ec0},
	{6144000,  49152000,  0x46f0},
	{705600,  11289600,  0x3ea0},
	{705600,  8467200,  0x3ab0},
	{24576000,  24576000,  0x02a0},
	{1411200,  11289600,  0x1690},
	{2822400,  11289600,  0x0a90},
	{1536000,  12288000,  0x1690},
	{3072000,  12288000,  0x0a90},
};

struct coeff_clk_div coeff_div[] = {
	/* sysclk is 256fs */
	{2048000,  8000 * 32,  8000, 0x1000},
	{2048000,  8000 * 64,  8000, 0x0000},
	{2822400,  11025 * 32,  11025,  0x1000},
	{2822400,  11025 * 64,  11025,  0x0000},
	{4096000,  16000 * 32,  16000,  0x1000},
	{4096000,  16000 * 64,  16000,  0x0000},
	{5644800,  22050 * 32,  22050,  0x1000},
	{5644800,  22050 * 64,  22050,  0x0000},
	{8192000,  32000 * 32,  32000,  0x1000},
	{8192000,  32000 * 64,  32000,  0x0000},
	{11289600,  44100 * 32,  44100,  0x1000},
	{11289600,  44100 * 64,  44100,  0x0000},
	{12288000,  48000 * 32,  48000,  0x1000},
	{12288000,  48000 * 64,  48000,  0x0000},
	{22579200,  88200 * 32,  88200,  0x1000},
	{22579200,  88200 * 64,  88200,  0x0000},
	{24576000,  96000 * 32,  96000,  0x1000},
	{24576000,  96000 * 64,  96000,  0x0000},
	/* sysclk is 384fs */
	{18432000, 48000 * 64, 48000, 0x0080},
	/* sysclk is 512fs */
	{4096000,  8000 * 32,  8000, 0x3000},
	{4096000,  8000 * 64,  8000, 0x2000},
	{5644800,  11025 * 32,  11025, 0x3000},
	{5644800,  11025 * 64,  11025, 0x2000},
	{8192000,  16000 * 32,  16000, 0x3000},
	{8192000,  16000 * 64,  16000, 0x2000},
	{11289600,  22050 * 32,  22050, 0x3000},
	{11289600,  22050 * 64,  22050, 0x2000},
	{16384000,  32000 * 32,  32000, 0x3000},
	{16384000,  32000 * 64,  32000, 0x2000},
	{22579200,  44100 * 32,  44100, 0x3000},
	{22579200,  44100 * 64,  44100, 0x2000},
	{24576000,  48000 * 32,  48000, 0x3000},
	{24576000,  48000 * 64,  48000, 0x2000},
	{45158400,  88200 * 32,  88200, 0x3000},
	{45158400,  88200 * 64,  88200, 0x2000},
	{49152000,  96000 * 32,  96000, 0x3000},
	{49152000,  96000 * 64,  96000, 0x2000},
	/* sysclk is 24.576Mhz or 22.5792Mhz */
	{24576000,  8000 * 32,  8000,  0x7080},
	{24576000,  8000 * 64,  8000,  0x6080},
	{24576000,  16000 * 32,  16000,  0x5080},
	{24576000,  16000 * 64,  16000,  0x4080},
	{24576000,  24000 * 32,  24000,  0x5000},
	{24576000,  24000 * 64,  24000,  0x4000},
	{24576000,  32000 * 32,  32000,  0x3080},
	{24576000,  32000 * 64,  32000,  0x2080},
	{22579200,  11025 * 32,  11025,  0x7000},
	{22579200,  11025 * 64,  11025,  0x6000},
	{22579200,  22050 * 32,  22050,  0x5000},
	{22579200,  22050 * 64,  22050,  0x4000},
};

static int get_coeff(int mclk, int rate, int timesofbclk)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].mclk == mclk && coeff_div[i].rate == rate &&
			(coeff_div[i].bclk / coeff_div[i].rate) == timesofbclk)
			return i;
	}
	return -EINVAL;
}

static int get_coeff_in_slave_mode(int mclk, int rate)
{
	return get_coeff(mclk, rate, timesofbclk);
}

static int get_coeff_in_master_mode(int mclk, int rate, int bclk)
{
	return get_coeff(mclk, rate, (bclk / rate));
}

static void rt5631_set_dmic_params(struct snd_soc_codec *codec,
	struct snd_pcm_hw_params *params)
{
	int rate;

	rt5631_write_mask(codec, RT5631_GPIO_CTRL,
		GPIO_PIN_FUN_SEL_GPIO_DIMC | GPIO_DMIC_FUN_SEL_DIMC,
		GPIO_PIN_FUN_SEL_MASK | GPIO_DMIC_FUN_SEL_MASK);
	rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL, DMIC_ENA, DMIC_ENA_MASK);
	rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL,
		DMIC_L_CH_LATCH_FALLING | DMIC_R_CH_LATCH_RISING,
		DMIC_L_CH_LATCH_MASK|DMIC_R_CH_LATCH_MASK);

	rate = params_rate(params);
	switch (rate) {
	case 44100:
	case 48000:
		rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL,
			DMIC_CLK_CTRL_TO_32FS, DMIC_CLK_CTRL_MASK);
		break;

	case 32000:
	case 22050:
		rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL,
			DMIC_CLK_CTRL_TO_64FS, DMIC_CLK_CTRL_MASK);
		break;

	case 16000:
	case 11025:
	case 8000:
		rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL,
			DMIC_CLK_CTRL_TO_128FS, DMIC_CLK_CTRL_MASK);
		break;

	default:
		break;
	}

	rt5631_write_mask(codec, RT5631_DIG_MIC_CTRL,
		DMIC_L_CH_UNMUTE | DMIC_R_CH_UNMUTE,
		DMIC_L_CH_MUTE_MASK | DMIC_R_CH_MUTE_MASK);

	return;
}

static int rt5631_hifi_pcm_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_codec *codec = rtd->codec;
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);
	int stream = substream->stream, rate = params_rate(params), coeff;
	unsigned int iface = 0;

	pr_debug("enter %s\n", __func__);

	if (!rt5631->master)
		coeff = get_coeff_in_slave_mode(rt5631->sysclk, rate);
	else
		coeff = get_coeff_in_master_mode(rt5631->sysclk, rate,
					rate * timesofbclk);
	if (coeff < 0)
		pr_err("%s: get coeff err!\n", __func__);

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= SDP_I2S_DL_20;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= SDP_I2S_DL_24;
		break;
	case SNDRV_PCM_FORMAT_S8:
		iface |= SDP_I2S_DL_8;
		break;
	default:
		return -EINVAL;
	}

	if (SNDRV_PCM_STREAM_CAPTURE == stream) {
		if (rt5631->dmic_used_flag)
			rt5631_set_dmic_params(codec, params);
		if(headset_alive && !rt5631->dmic_used_flag)
			rt5631_close_dmic(codec);
	}

	rt5631_write_mask(codec, RT5631_SDP_CTRL, iface, SDP_I2S_DL_MASK);

	if (coeff >= 0)
		rt5631_write(codec, RT5631_STEREO_AD_DA_CLK_CTRL,
					coeff_div[coeff].reg_val);

	return 0;
}

static int rt5631_hifi_codec_set_dai_fmt(struct snd_soc_dai *codec_dai,
						unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);
	unsigned int iface = 0;

	pr_debug("enter %s\n", __func__);

	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		rt5631->master = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		iface |= SDP_MODE_SEL_SLAVE;
		rt5631->master = 0;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= SDP_I2S_DF_LEFT;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= SDP_I2S_DF_PCM_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface  |= SDP_I2S_DF_PCM_B;
		break;
	default:
		return -EINVAL;
	}

	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= SDP_I2S_BCLK_POL_CTRL;
		break;
	default:
		return -EINVAL;
	}

	rt5631_write(codec, RT5631_SDP_CTRL, iface);

	return 0;
}

static int rt5631_hifi_codec_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);

	pr_info("enter %s, syclk=%d\n", __func__, freq);
	if ((freq >= (256 * 8000)) && (freq <= (512 * 96000))) {
		rt5631->sysclk = freq;
		return 0;
	}

	pr_info("unsupported sysclk freq %u for audio i2s\n", freq);
	pr_info("set sysclk to 24.576Mhz by default\n");

	rt5631->sysclk = 24576000;
	return 0;
}

static int rt5631_codec_set_dai_pll(struct snd_soc_dai *codec_dai, int pll_id,
		int source, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);
	int i, ret = -EINVAL;

	printk(KERN_DEBUG "enter %s\n", __func__);

	if (!freq_in || !freq_out)
		return 0;

	if (rt5631->master) {
		for (i = 0; i < ARRAY_SIZE(codec_master_pll_div); i++)
			if (freq_in == codec_master_pll_div[i].pll_in &&
			freq_out == codec_master_pll_div[i].pll_out) {
				rt5631_write(codec, RT5631_PLL_CTRL,
					codec_master_pll_div[i].reg_val);
				schedule_timeout_uninterruptible(
					msecs_to_jiffies(20));
				rt5631_write(codec, RT5631_GLOBAL_CLK_CTRL,
					SYSCLK_SOUR_SEL_PLL);
				rt5631->pll_used_flag = 1;
				ret = 0;
				break;
			}
	} else {
		for (i = 0; i < ARRAY_SIZE(codec_slave_pll_div); i++)
			if (freq_in == codec_slave_pll_div[i].pll_in &&
			freq_out == codec_slave_pll_div[i].pll_out) {
				rt5631_write(codec, RT5631_PLL_CTRL,
					codec_slave_pll_div[i].reg_val);
				schedule_timeout_uninterruptible(
					msecs_to_jiffies(20));
				rt5631_write(codec, RT5631_GLOBAL_CLK_CTRL,
					SYSCLK_SOUR_SEL_PLL |
					PLLCLK_SOUR_SEL_BITCLK);
				rt5631->pll_used_flag = 1;
				ret = 0;
				break;
			}
	}

	return ret;
}

static void DumpRT5631Q(struct snd_soc_codec *codec)
{
	int i = 0;
	u16 reg;

	for (i=0; i<0x7F; i++)
	{
		reg = snd_soc_read(codec, i);
		printk("Read 0X%02x = 0X%04x\n", i, reg);
	}
}

static int audio_codec_stress()
{
	u16 temp;

	temp = snd_soc_read(rt5631_codec, RT5631_VENDOR_ID1); /* Read codec ID */

	count_base = count_base+1;

	if (count_base == 100){
		count_base = 0;
		count_100 = count_100 + 1;
		printk("AUDIO_CODEC: count = %d (* 100), the register 0x7Ch is %x\n",count_100,temp);
	}

	schedule_delayed_work(&poll_audio_work, poll_rate);

	return 0;
}

int audio_codec_open(struct inode *inode, struct file *filp)
{
	return 0;
}

long audio_codec_ioctl(struct file *filp,
                 unsigned int cmd, unsigned long arg)
{
	char tmp[3];
	u8 address;
	u16 buf[1];
	int err = 0;
	int retval = 0;

	if (_IOC_TYPE(cmd) != AUDIO_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > AUDIO_IOC_MAXNR) return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 * access_ok: 1 (successful, accessable)
	 */
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;


       /* cmd: the ioctl commend user-space asked */
	switch(cmd){
		case AUDIO_STRESS_TEST:
			printk("AUDIO_CODEC: AUDIO_STRESS_TEST: %lu (1: Start, 0: Stop)\n",arg);
			if(arg==AUDIO_IOCTL_START_HEAVY){
				poll_rate = START_HEAVY;
				schedule_delayed_work(&poll_audio_work, poll_rate);
			}else if(arg==AUDIO_IOCTL_START_NORMAL){
				poll_rate = START_NORMAL;
				schedule_delayed_work(&poll_audio_work,poll_rate);
			}else if(arg==AUDIO_IOCTL_STOP){
				cancel_delayed_work_sync(&poll_audio_work);
			}
			break;
		case AUDIO_DUMP:
			printk("AUDIO_CODEC: AUDIO_DUMP\n");
			DumpRT5631Q(rt5631_codec);
			break;

		case AUDIO_I2C_READ:
			if(copy_from_user(tmp, (void __user*)arg, sizeof(tmp))){
				printk("AUDIO_CODEC : read data from user space fail\n");
			}
			address = (u8)tmp[0];
			buf[0] = snd_soc_read(rt5631_codec, address);
			tmp[1] = buf[0];
			tmp[2] = buf[0]>>8;
			printk("AUDIO_CODEC: Read 0X%02x = 0X%04x\n", tmp[0], (tmp[2] <<8 |tmp[1]));
			if(copy_to_user((void __user*)arg, tmp, sizeof(tmp))){
				printk("AUDIO_CODEC: AUDIO_I2C_READ error\n");
			}
			break;

		case AUDIO_I2C_WRITE:
			if(copy_from_user(tmp,(void __user *)arg,sizeof(tmp)))
				return -EFAULT;
			printk("AUDIO_CODEC: Write 0X%02x = 0X%04x\n",tmp[0],(tmp[2] <<8 |tmp[1]));

			snd_soc_write(rt5631_codec, tmp[0], (tmp[2] <<8 |tmp[1]));
			if(copy_to_user((void __user*)arg, tmp, sizeof(tmp)))
				return -EFAULT;

			break;
		case AUDIO_CAPTURE_MODE:
			switch(arg){
				case INPUT_SOURCE_NORMAL:
				case INPUT_SOURCE_VR:
					printk("AUDIO_CODEC: Capture mode [%s]\n",	 arg == INPUT_SOURCE_NORMAL ? "NORMAL" : "VR");
					input_source=arg;
					break;
				case INPUT_SOURCE_AGC:
				case INPUT_SOURCE_NO_AGC:
					printk("AUDIO_CODEC: Capture mode [%s]\n",	 arg == INPUT_SOURCE_AGC ? "AGC" : "NON-AGC");
					input_agc = arg;
					break;
			       case OUTPUT_SOURCE_NORMAL:
				case OUTPUT_SOURCE_VOICE:
                                        printk("AUDIO_CODEC: Capture mode [%s]\n",
                                                 arg == OUTPUT_SOURCE_NORMAL ? "NORMAL" : "VOICE");
					output_source=arg;
					break;
				default:
					break;
			}
			break;

	  default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
	return retval;
}

struct file_operations audio_codec_fops = {
	.owner =    THIS_MODULE,
	.open =     audio_codec_open,
	.unlocked_ioctl =    audio_codec_ioctl,
};

static struct miscdevice i2c_audio_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rt5631",
	.fops = &audio_codec_fops,
};

static ssize_t read_audio_dock_status(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        char data1[2] = {0x05, 0x01};
	char data3[8] = {0};
	int i = 0;
        asusAudiodec_i2c_write_data(data1, 2);
        asusAudiodec_i2c_read_data(data3, 8);
        for( i = 0; i < 8; i++){
             printk("index: %d data = %d\n", i, data3[i]);
        }
        return sprintf(buf, "use command: dmesg -c \n");
}
DEVICE_ATTR(read_audio_dock, S_IRUGO, read_audio_dock_status, NULL);


static ssize_t unmute1_audio_dock_status(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        char data1[2] = {0x01, 0x01};

        asusAudiodec_i2c_write_data(data1, 2);

        return sprintf(buf, "unmute\n");
}
DEVICE_ATTR(unmute1_audio_dock, S_IRUGO, unmute1_audio_dock_status, NULL);

static ssize_t unmute2_audio_dock_status(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        char data1[2] = {0x02, 0x01};

        asusAudiodec_i2c_write_data(data1, 2);

        return sprintf(buf, "unmute\n");
}
DEVICE_ATTR(unmute2_audio_dock, S_IRUGO, unmute2_audio_dock_status, NULL);


static ssize_t unmute3_audio_dock_status(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        char data1[2] = {0x03, 0x01};

        asusAudiodec_i2c_write_data(data1, 2);

        return sprintf(buf, "unmute\n");
}
DEVICE_ATTR(unmute3_audio_dock, S_IRUGO, unmute3_audio_dock_status, NULL);


static ssize_t unmute4_audio_dock_status(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        char data1[2] = {0x04, 0x01};

        asusAudiodec_i2c_write_data(data1, 2);

        return sprintf(buf, "unmute\n");
}
DEVICE_ATTR(unmute4_audio_dock, S_IRUGO, unmute4_audio_dock_status, NULL);

static ssize_t unmute_audio_dock_status(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        char data1[2] = {0x00, 0x01};

        asusAudiodec_i2c_write_data(data1, 2);

        return sprintf(buf, "unmute\n");
}
DEVICE_ATTR(unmute_audio_dock, S_IRUGO, unmute_audio_dock_status, NULL);

static ssize_t mute_audio_dock_status(struct device *dev,
        struct device_attribute *attr, char *buf)
{
        char data1[2] = {0xff, 0x01};

        asusAudiodec_i2c_write_data(data1, 2);

        return sprintf(buf, "mute\n");
}
DEVICE_ATTR(mute_audio_dock, S_IRUGO, mute_audio_dock_status, NULL);

static ssize_t read_audio_codec_status(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", audio_codec_status);
}
DEVICE_ATTR(audio_codec_status, S_IRUGO, read_audio_codec_status, NULL);

static ssize_t rt5631_index_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	#define IDX_REG_FMT "%02x: %04x\n"
	#define IDX_REG_LEN 9
	unsigned int val;
	int cnt = 0, i;

	cnt += sprintf(buf, "RT5631 index register\n");
	for (i = 0; i < 0x55; i++) {
		if (cnt + IDX_REG_LEN >= PAGE_SIZE - 1)
			break;
		val = rt5631_read_index(rt5631_codec, i);
		if (!val)
			continue;
		cnt += sprintf(buf + cnt, IDX_REG_FMT, i, val);
	}

	if (cnt >= PAGE_SIZE)
		cnt = PAGE_SIZE - 1;

	return cnt;
}
static DEVICE_ATTR(index_reg, 0444, rt5631_index_reg_show, NULL);

static u32 StrtoInt(const char *str)
{
	u32 i,CodecValue=0;

	printk("strtoin=%s \n",str);
	if(!str)
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(i=0; *str; str++)
	{
		if((*str>='A' && *str<='F')||(*str>='a' && *str<='f')||(*str>='0' && *str<='9'))
		{
			 CodecValue*=0x10;
			if(*str>='A' && *str<='F')
				CodecValue += *str-'A'+10;
			else if(*str>='a' && *str<='f')
				CodecValue += *str-'a'+10;
			else if(*str>='0' && *str<='9')
				CodecValue += *str-'0';
		}
		else
				return CodecValue;
	}
	return CodecValue;
}


enum
{
	WRITE_REG=0,
	READ_REG,
	WRITE_INDEX,
	READ_INDEX,
	BAD_FORMAT,
};

static u32 last_command=0;
static u32 read_codec_reg=0;
static u32 read_codec_value=0;


//static ssize_t rt5631_codec_reg_show(struct class *cls, char *_buf)
static ssize_t rt5631_codec_reg_show(struct device *dev,struct device_attribute *attr ,char *buf)
{
	if(last_command==READ_REG)
	{
		return sprintf(buf,"%04x\n",read_codec_value);
	}
	if(last_command==READ_INDEX)
	{
		return sprintf(buf,"%04x\n",read_codec_value);
	}

	return sprintf(buf,"read fail\n");
}

//static ssize_t rt5631_codec_reg_store(struct class *cls, const char *_buf, size_t _count)
static ssize_t rt5631_codec_reg_store(struct device *dev,struct device_attribute *attr, const char *buf,
 size_t count)
{
	const char * p=buf;
	u32 reg=0, val=0;
	printk("store buf=%s \n",buf);

	if(!strncmp(buf, "readr", strlen("readr")))
	{
		p+=strlen("readr");
		read_codec_reg=(u32)StrtoInt(p);
		read_codec_value=rt5631_read(rt5631_codec, read_codec_reg);
		last_command=READ_REG;
		printk("%s(): get 0x%04x=0x%04x\n", __FUNCTION__, read_codec_reg, val);
	}
	else if(!strncmp(buf, "writer", strlen("writer")))
	{
		p+=strlen("writer");
		reg=(u32)StrtoInt(p);
		p=strchr(buf, '=');
		if(p)
		{
			++ p;
			val=(u32)StrtoInt(p);
			rt5631_write(rt5631_codec, reg, val);
			last_command=WRITE_REG;
			printk("%s(): set 0x%04x=0x%04x\n", __FUNCTION__, reg, val);
		}
		else
		{
			last_command=BAD_FORMAT;
			printk("%s(): Bad string format input!\n", __FUNCTION__);
		}
	}
	else if(!strncmp(buf, "writei", strlen("writei")))
	{
		p+=strlen("writei");
		reg=(u32)StrtoInt(p);
		p=strchr(buf, '=');
		if(p)
		{
			++ p;
			val=(u32)StrtoInt(p);
			rt5631_write(rt5631_codec, 0x6a, reg);
			rt5631_write(rt5631_codec, 0x6c, val);
			last_command=WRITE_INDEX;
			printk("%s(): set 0x%04x=0x%04x\n", __FUNCTION__, reg, val);
		}
		else
		{
			last_command=BAD_FORMAT;
			printk("%s(): Bad string format input!\n", __FUNCTION__);
		}
	}
	else if(!strncmp(buf, "readi", strlen("readi")))
	{
		p+=strlen("readi");
		read_codec_reg=(u32)StrtoInt(p);
		rt5631_write(rt5631_codec, 0x6a, read_codec_reg);
		read_codec_value=rt5631_read(rt5631_codec, 0x6c);
		last_command=READ_INDEX;
		printk("%s(): get 0x%04x=0x%04x\n", __FUNCTION__, read_codec_reg, val);
	}
	else
	{
		last_command=BAD_FORMAT;
		printk("%s(): Bad string format input!\n", __FUNCTION__);
	}

	return count;
}

static DEVICE_ATTR(rt_codec_reg, S_IRUGO, rt5631_codec_reg_show, rt5631_codec_reg_store);

#define RT5631_STEREO_RATES SNDRV_PCM_RATE_8000_96000
#define RT5631_FORMAT	(SNDRV_PCM_FMTBIT_S16_LE | \
			SNDRV_PCM_FMTBIT_S20_3LE | \
			SNDRV_PCM_FMTBIT_S24_LE | \
			SNDRV_PCM_FMTBIT_S8)


struct snd_soc_dai_ops rt5631_ops = {
	.hw_params = rt5631_hifi_pcm_params,
	.set_fmt = rt5631_hifi_codec_set_dai_fmt,
	.set_sysclk = rt5631_hifi_codec_set_dai_sysclk,
	.set_pll = rt5631_codec_set_dai_pll,
};

struct snd_soc_dai_driver rt5631_dai[] = {
	{
		.name = "rt5631-hifi",
		.id = 1,
		.playback = {
			.stream_name = "HIFI Playback",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5631_STEREO_RATES,
			.formats = RT5631_FORMAT,
		},
		.capture = {
			.stream_name = "HIFI Capture",
			.channels_min = 1,
			.channels_max = 2,
			.rates = RT5631_STEREO_RATES,
			.formats = RT5631_FORMAT,
		},
		.ops = &rt5631_ops,
	},
};
EXPORT_SYMBOL_GPL(rt5631_dai);

static int rt5631_set_bias_level(struct snd_soc_codec *codec,
			enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON:
		break;

	case SND_SOC_BIAS_PREPARE:
		rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3,
			PWR_VREF | PWR_MAIN_BIAS, PWR_VREF | PWR_MAIN_BIAS);
		break;

	case SND_SOC_BIAS_STANDBY:
		break;

	case SND_SOC_BIAS_OFF:
		rt5631_write_mask(codec, RT5631_SPK_OUT_VOL,
			RT_L_MUTE | RT_R_MUTE, RT_L_MUTE | RT_R_MUTE);
		rt5631_write_mask(codec, RT5631_HP_OUT_VOL,
			RT_L_MUTE | RT_R_MUTE, RT_L_MUTE | RT_R_MUTE);
		rt5631_write(codec, RT5631_PWR_MANAG_ADD1, 0x0000);
		rt5631_write(codec, RT5631_PWR_MANAG_ADD2, 0x0000);
		rt5631_write(codec, RT5631_PWR_MANAG_ADD3, 0x0000);
		rt5631_write(codec, RT5631_PWR_MANAG_ADD4, 0x0000);
		break;

	default:
		break;
	}
	codec->dapm.bias_level = level;

	return 0;
}

#if defined(RTK_IOCTL)
#if defined(CONFIG_SND_HWDEP)
#define RT_CE_CODEC_HWDEP_NAME "rt56xx hwdep "

static int rt56xx_hwdep_open(struct snd_hwdep *hw, struct file *file)
{
	printk("enter %s\n", __func__);
	return 0;
}

static int rt56xx_hwdep_release(struct snd_hwdep *hw, struct file *file)
{
	printk("enter %s\n", __func__);
	return 0;
}


static int rt56xx_hwdep_ioctl_common(struct snd_hwdep *hw, struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rt56xx_cmd rt56xx;
	int *buf;
	int *p;
	struct rt56xx_cmd __user *_rt56xx =(struct rt56xx_cmd *)arg;
	struct snd_soc_codec *codec = hw->private_data;
	u16 virtual_reg;
	int rt5631_eq_mode;

	if (copy_from_user(&rt56xx, _rt56xx, sizeof(rt56xx))) {
		printk("copy_from_user faild\n");
		return -EFAULT;
	}

	buf = kmalloc(sizeof(*buf) * rt56xx.number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;
	if (copy_from_user(buf, rt56xx.buf, sizeof(*buf) * rt56xx.number)) {
		goto err;
	}

	switch (cmd) {
		case RT_READ_CODEC_REG_IOCTL:
			for (p = buf; p < buf + rt56xx.number/2; p++)
			{
				*(p+rt56xx.number/2) = snd_soc_read(codec, *p);
			}
			if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
				goto err;
			break;
		case RT_WRITE_CODEC_REG_IOCTL:
			for (p = buf; p < buf + rt56xx.number/2; p++)
				codec->write(codec, *p, *(p+rt56xx.number/2));
			break;
		case RT_READ_CODEC_INDEX_IOCTL:
			for (p = buf; p < buf + rt56xx.number/2; p++)
			{
				*(p+rt56xx.number/2) = rt5631_read_index(codec, *p);
			}
			if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
				goto err;
			break;
		case RT_WRITE_CODEC_INDEX_IOCTL:
			for (p = buf; p < buf + rt56xx.number/2; p++)
				rt5631_write_index(codec, *p, *(p+rt56xx.number/2));
			break;
		case RT_SET_CODEC_HWEQ_IOCTL:
			virtual_reg = rt5631_read(codec, VIRTUAL_REG_FOR_MISC_FUNC);
			rt5631_eq_mode=(virtual_reg&0x00f0)>>4;

			if ( rt5631_eq_mode == *buf)
				break;

			rt5631_update_eqmode(codec, *buf);

			virtual_reg &= 0xff0f;
			virtual_reg |= (*buf<<4);
			rt5631_write(codec, VIRTUAL_REG_FOR_MISC_FUNC, virtual_reg);

			break;
		case RT_SET_CODEC_SPK_VOL_IOCTL:
			snd_soc_update_bits(codec, RT5631_SPK_OUT_VOL,
				RT5631_L_VOL_MASK | RT5631_R_VOL_MASK,
				*(buf)<<RT5631_L_VOL_SFT | *(buf)<<RT5631_R_VOL_SFT);
			break;
		case RT_SET_CODEC_MIC_GAIN_IOCTL:
			snd_soc_update_bits(codec, RT5631_MIC_CTRL_2,
				RT5631_L_VOL_MASK | RT5631_R_VOL_MASK,
				*(buf)<<12 | *(buf)<<8);
			break;
		case RT_GET_CODEC_ID:
			*buf = rt5631_read(codec, RT5631_VENDOR_ID2);

			if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * rt56xx.number))
				goto err;
			break;
		default:
			printk("unsupported io cmd\n");
			break;
	}

	kfree(buf);
	return 0;

err:
	kfree(buf);
	return -EFAULT;

}

static int rt56xx_codec_dump_reg(struct snd_hwdep *hw, struct file *file, unsigned long arg)
{
	struct rt56xx_cmd rt56xx;
	struct rt56xx_cmd __user *_rt56xx =(struct rt56xx_cmd *)arg;
	int *buf;
	struct snd_soc_codec *codec = hw->private_data;
	int number = codec->reg_size;
	int i;

	printk(KERN_DEBUG "enter %s, number = %d\n", __func__, number);
	if (copy_from_user(&rt56xx, _rt56xx, sizeof(rt56xx)))
		return -EFAULT;

	buf = kmalloc(sizeof(*buf) * number, GFP_KERNEL);
	if (buf == NULL)
		return -ENOMEM;

	for (i = 0; i < number/2; i++)
	{
		buf[i] = i << 1;
		buf[i+number/2] = codec->read(codec, buf[i]);
	}
	if (copy_to_user(rt56xx.buf, buf, sizeof(*buf) * i))
		goto err;
	rt56xx.number = number;
	if (copy_to_user(_rt56xx, &rt56xx, sizeof(rt56xx)))
		goto err;
	kfree(buf);
	return 0;
err:
	kfree(buf);
	return -EFAULT;

}

static int rt56xx_hwdep_ioctl(struct snd_hwdep *hw, struct file *file, unsigned int cmd, unsigned long arg)
{

	printk("***********************rt56xx_hwdep_ioctl cmd=%x,arg=%lx**********************\n",cmd,arg);

	if (cmd == RT_READ_ALL_CODEC_REG_IOCTL)
	{
		return rt56xx_codec_dump_reg(hw, file, arg);
	}
	else
	{
		return rt56xx_hwdep_ioctl_common(hw, file, cmd, arg);
	}
}

static int realtek_ce_init_hwdep(struct snd_soc_codec *codec)
{

	struct snd_hwdep *hw;
	struct snd_card *card = codec->card->snd_card;
	int err;

	if ((err = snd_hwdep_new(card, RT_CE_CODEC_HWDEP_NAME, 0, &hw)) < 0)
		return err;

	strcpy(hw->name, RT_CE_CODEC_HWDEP_NAME);
	hw->private_data = codec;
	hw->ops.open = rt56xx_hwdep_open;
	hw->ops.release = rt56xx_hwdep_release;
	hw->ops.ioctl = rt56xx_hwdep_ioctl;
	return 0;
}
#endif
#endif //RTK_IOCTL

static int rt5631_probe(struct snd_soc_codec *codec)
{
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);
	unsigned int val;
	int ret;

	ret = gpio_request(CODEC_SPKVDD_POWER_5V0_EN_GPIO, "RT5631_5V");
	if (ret) {
		printk("gpio_request failed for input %d\n", CODEC_SPKVDD_POWER_5V0_EN_GPIO);
	}
	ret = gpio_direction_output(CODEC_SPKVDD_POWER_5V0_EN_GPIO, 1) ;
	if (ret) {
		printk("gpio_direction_output failed for input %d\n", CODEC_SPKVDD_POWER_5V0_EN_GPIO);
	}
	printk("GPIO = %d , state = %d\n", CODEC_SPKVDD_POWER_5V0_EN_GPIO,
			gpio_get_value(CODEC_SPKVDD_POWER_5V0_EN_GPIO));
	gpio_set_value(CODEC_SPKVDD_POWER_5V0_EN_GPIO, 1);

	project_id = tegra3_get_project_id();
	ret = snd_soc_codec_set_cache_io(codec, 8, 16, SND_SOC_I2C);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to set cache I/O: %d\n", ret);
		return ret;
	}
	codec->cache_bypass = 1;

	val = rt5631_read_index(codec, RT5631_ADDA_MIXER_INTL_REG3);
	if (val & 0x0002)
		rt5631->codec_version = 1;
	else
		rt5631->codec_version = 0;

	rt5631_reset(codec);
	rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3,
		PWR_VREF | PWR_MAIN_BIAS, PWR_VREF | PWR_MAIN_BIAS);
	schedule_timeout_uninterruptible(msecs_to_jiffies(80));
	rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3, PWR_FAST_VREF_CTRL,
					PWR_FAST_VREF_CTRL);
	rt5631_reg_init(codec);

	/* power off ClassD auto Recovery */
	if (rt5631->codec_version)
		rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2,
					0x2000, 0x2000);
	else
		rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2,
					0, 0x2000);

	codec->dapm.bias_level = SND_SOC_BIAS_STANDBY;
	rt5631_codec = codec;
	rt5631_audio_codec = codec;
	snd_soc_add_controls(codec, rt5631_snd_controls,
		ARRAY_SIZE(rt5631_snd_controls));
	rt5631_add_widgets(codec);

	ret = device_create_file(codec->dev, &dev_attr_index_reg);
	if (ret != 0) {
		dev_err(codec->dev,
			"Failed to create index_reg sysfs files: %d\n", ret);
		return ret;
	}
	ret = device_create_file(codec->dev, &dev_attr_rt_codec_reg);
	if (ret != 0) {
		dev_err(codec->dev,
			"Failed to create codec_reg sysfs files: %d\n", ret);
		return ret;
	}

	ret = device_create_file(codec->dev, &dev_attr_audio_codec_status);
	if (ret != 0) {
		dev_err(codec->dev,
			"Failed to create audio_codec_status sysfs files: %d\n", ret);
		return ret;
	}
        ret = device_create_file(codec->dev, &dev_attr_mute_audio_dock);
        if (ret != 0) {
                dev_err(codec->dev,
                        "Failed to create audio_codec_status sysfs files: %d\n", ret);
                return ret;
        }
        ret = device_create_file(codec->dev, &dev_attr_unmute_audio_dock);
        if (ret != 0) {
                dev_err(codec->dev,
                        "Failed to create audio_codec_status sysfs files: %d\n", ret);
                return ret;
        }
        ret = device_create_file(codec->dev, &dev_attr_read_audio_dock);
        if (ret != 0) {
                dev_err(codec->dev,
                        "Failed to create audio_codec_status sysfs files: %d\n", ret);
                return ret;
        }

        ret = device_create_file(codec->dev, &dev_attr_unmute1_audio_dock);
        if (ret != 0) {
                dev_err(codec->dev,
                        "Failed to create audio_codec_status sysfs files: %d\n", ret);
                return ret;
        }
        ret = device_create_file(codec->dev, &dev_attr_unmute2_audio_dock);
        if (ret != 0) {
                dev_err(codec->dev,
                        "Failed to create audio_codec_status sysfs files: %d\n", ret);
                return ret;
        }
        ret = device_create_file(codec->dev, &dev_attr_unmute3_audio_dock);
        if (ret != 0) {
                dev_err(codec->dev,
                        "Failed to create audio_codec_status sysfs files: %d\n", ret);
                return ret;
        }
        ret = device_create_file(codec->dev, &dev_attr_unmute4_audio_dock);
        if (ret != 0) {
                dev_err(codec->dev,
                        "Failed to create audio_codec_status sysfs files: %d\n", ret);
                return ret;
        }

	if(rt5631_read(rt5631_codec, RT5631_VENDOR_ID1) == 0x10EC)
		audio_codec_status = 1;
	else
		printk("%s: incorrect audio codec rt5631 vendor ID\n", __func__);

	pr_info("RT5631 initial ok!\n");

	#if defined(RTK_IOCTL)
       #if defined(CONFIG_SND_HWDEP)
       printk("************************realtek_ce_init_hwdep*************************************\n");
       realtek_ce_init_hwdep(rt5631_codec);
       #endif
       #endif //RTK_IOCTL

	return 0;
}

static int rt5631_remove(struct snd_soc_codec *codec)
{
	rt5631_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int rt5631_suspend(struct snd_soc_codec *codec, pm_message_t state)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	rt5631_set_bias_level(codec, SND_SOC_BIAS_OFF);
	printk(KERN_INFO "%s- #####\n", __func__);
	return 0;
}

static int rt5631_resume(struct snd_soc_codec *codec)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	struct rt5631_priv *rt5631 = snd_soc_codec_get_drvdata(codec);

	rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3,
		PWR_VREF | PWR_MAIN_BIAS, PWR_VREF | PWR_MAIN_BIAS);
	schedule_timeout_uninterruptible(msecs_to_jiffies(110));
	rt5631_write_mask(codec, RT5631_PWR_MANAG_ADD3,
		PWR_FAST_VREF_CTRL, PWR_FAST_VREF_CTRL);
	rt5631_reg_init(codec);

	/* power off ClassD auto Recovery */
	if (rt5631->codec_version)
		rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2,
					0x2000, 0x2000);
	else
		rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2,
					0, 0x2000);
	printk(KERN_INFO "%s- #####\n", __func__);

	return 0;
}

/*
 * detect short current for mic1
 */
int rt5631_ext_mic_detect(void)
{
	struct snd_soc_codec *codec = rt5631_codec;
	int det;

	rt5631_write_mask(codec, RT5631_MIC_CTRL_2, MICBIAS1_S_C_DET_ENA,
				MICBIAS1_S_C_DET_MASK);
	det = rt5631_read(codec, RT5631_INT_ST_IRQ_CTRL_2) & 0x0001;
	rt5631_write_mask(codec, RT5631_INT_ST_IRQ_CTRL_2, 0x0001, 0x00001);

	return det;
}
EXPORT_SYMBOL_GPL(rt5631_ext_mic_detect);

static struct snd_soc_codec_driver soc_codec_dev_rt5631 = {
	.probe = rt5631_probe,
	.remove = rt5631_remove,
	.suspend = rt5631_suspend,
	.resume = rt5631_resume,
	.set_bias_level = rt5631_set_bias_level,
	.reg_cache_size = ARRAY_SIZE(rt5631_reg),
	.reg_word_size = sizeof(u16),
	.reg_cache_default = rt5631_reg,
	.reg_cache_step = 1,
};

static const struct i2c_device_id rt5631_i2c_id[] = {
	{ "rt5631", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, rt5631_i2c_id);

static int rt5631_i2c_probe(struct i2c_client *i2c,
		    const struct i2c_device_id *id)
{
	struct rt5631_priv *rt5631;
	int ret;
	printk("%s+\n", __func__);
	pr_info("RT5631 Audio Codec %s\n", RT5631_VERSION);
	audio_codec_status = 0;

	rt5631 = kzalloc(sizeof(struct rt5631_priv), GFP_KERNEL);
	if (NULL == rt5631)
		return -ENOMEM;

	i2c_set_clientdata(i2c, rt5631);

	ret = misc_register(&i2c_audio_device);
	if (ret < 0) {
		dev_err(&i2c->adapter->dev,
			"ERROR: misc_register returned %d\n", ret);
	}

	ret = snd_soc_register_codec(&i2c->dev, &soc_codec_dev_rt5631,
			rt5631_dai, ARRAY_SIZE(rt5631_dai));
	if (ret < 0)
		kfree(rt5631);
	INIT_DELAYED_WORK(&poll_audio_work, audio_codec_stress);
	printk("%s-\n", __func__);
	return ret;
}

static __devexit int rt5631_i2c_remove(struct i2c_client *client)
{
	snd_soc_unregister_codec(&client->dev);
	kfree(i2c_get_clientdata(client));
	return 0;
}

static int rt5631_i2c_shutdown(struct i2c_client *client)
{

	printk(KERN_INFO "%s+ #####\n", __func__);
	rt5631_set_bias_level(rt5631_codec, SND_SOC_BIAS_OFF);
	printk(KERN_INFO "%s- #####\n", __func__);
	return 0;
}

struct i2c_driver rt5631_i2c_driver = {
	.driver = {
		.name = "rt5631",
		.owner = THIS_MODULE,
	},
	.probe = rt5631_i2c_probe,
	.remove   = __devexit_p(rt5631_i2c_remove),
	.shutdown = rt5631_i2c_shutdown,
	.id_table = rt5631_i2c_id,
};

static int codec_3v3_power_switch_init(void)
{
	printk("%s\n", __func__);

	u32 project_info = tegra3_get_project_id();
	int ret = 0;

	if(project_info == TEGRA3_PROJECT_TF700T){
		ret = gpio_request(RT5631_3V3_POWER_EN, "rt5631_3v3_power_control");
		if(ret < 0){
			printk("rt5631_3v3_power_control: request rt5631_3v3_power_control fail! : %d\n", ret);
			return ret;
		}
		ret = gpio_direction_output(RT5631_3V3_POWER_EN, 1);
		if(ret < 0){
			printk("rt5631_3v3_power_control: set rt5631_3v3_power_control as output fail! : %d\n", ret);
		}
		gpio_free(RT5631_3V3_POWER_EN);
	}
	return ret;
}

static int __init rt5631_modinit(void)
{
	int ret = 0;
	printk(KERN_INFO "%s+ #####\n", __func__);

	codec_3v3_power_switch_init();

	ret = i2c_add_driver(&rt5631_i2c_driver);

	printk(KERN_INFO "%s- #####\n", __func__);
	return ret;

}
module_init(rt5631_modinit);

static void __exit rt5631_modexit(void)
{
	i2c_del_driver(&rt5631_i2c_driver);
}
module_exit(rt5631_modexit);

MODULE_DESCRIPTION("ASoC RT5631 driver");
MODULE_AUTHOR("flove <flove@realtek.com>");
MODULE_LICENSE("GPL");
