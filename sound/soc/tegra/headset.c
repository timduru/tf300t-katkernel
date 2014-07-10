
/*
 * Headset device detection driver.
 *
 * Copyright (C) 2011 ASUSTek Corporation.
 *
 * Authors:
 * Jason Cheng <jason4_cheng@asus.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 */
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/input.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include <linux/timer.h>
#include <linux/switch.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <asm/string.h>
#include <sound/soc.h>
#include <linux/wakelock.h>
#include "../gpio-names.h"
#include <mach/board-cardhu-misc.h>
#include "../codecs/wm8903.h"
#include "../codecs/rt5640.h"
#include "../../../arch/arm/mach-tegra/board.h"
#include <mach/pinmux.h>
#include <mach/pinmux-t3.h>

MODULE_DESCRIPTION("Headset detection driver");
MODULE_LICENSE("GPL");

/*----------------------------------------------------------------------------
 ** FUNCTION DECLARATION
 **----------------------------------------------------------------------------*/
static int __init 	headset_init(void);
static void __exit headset_exit(void);
static irqreturn_t 	detect_irq_handler(int irq, void *dev_id);
static irqreturn_t hook_irq_handler(int irq, void *dev_id);
static void 		detection_work(struct work_struct *work);
static int 	jack_config_gpio(void);
static irqreturn_t 	lineout_irq_handler(int irq, void *dev_id);
static void 		lineout_work_queue(struct work_struct *work);
static void		hook_work_queue(struct work_struct *work);
static int 	lineout_config_gpio(void);
static void 		detection_work(struct work_struct *work);
static int 	btn_config_gpio(void);
int 			hs_micbias_power(int on);
/*----------------------------------------------------------------------------
 ** GLOBAL VARIABLES
 **----------------------------------------------------------------------------*/
#define JACK_GPIO		(TEGRA_GPIO_PW2)
#define LINEOUT_GPIO	(TEGRA_GPIO_PX3)
#define LINEOUT_ME301T	(TEGRA_GPIO_PW3)
#define HOOK_GPIO		(TEGRA_GPIO_PX2)
#define UART_HEADPHONE_SWITCH (TEGRA_GPIO_PS2)
#define ON	1
#define OFF	0
#define MICDET_ENA		(1 << 1)
#define MICBIAS_ENA		(1 << 0)
#define FORCE_HEADPHONE (1)
#define NO_FORCE_HEADPHONE (0)

enum{
	NO_DEVICE = 0,
	HEADSET_WITH_MIC = 1,
	HEADSET_WITHOUT_MIC = 2,
};

enum{
	NO_LINEOUT = 0,
	LINEOUT_IN = 1,
};

struct headset_data {
	struct switch_dev sdev;
	struct switch_dev ldev;
	struct input_dev *input;
	unsigned int irq;
	unsigned int lineout_gpio;
	unsigned long irq_flags;
	struct hrtimer timer;
	ktime_t debouncing_time;
};

u32 project_info = TEGRA3_PROJECT_INVALID;
static struct headset_data *hs_data;
bool headset_alive = false;
EXPORT_SYMBOL(headset_alive);
bool lineout_alive;
EXPORT_SYMBOL(lineout_alive);
bool debugboard_alive = false;
EXPORT_SYMBOL(debugboard_alive);
bool isHpDetecting = false;
EXPORT_SYMBOL(isHpDetecting);

static struct workqueue_struct *g_detection_work_queue;
static DECLARE_WORK(g_detection_work, detection_work);

struct wake_lock hp_detect_wakelock;
static bool console_disable;
extern struct snd_soc_codec *rt5631_audio_codec;
extern struct snd_soc_codec *wm8903_codec;
extern struct snd_soc_codec *rt5640_audio_codec;
struct work_struct headset_work;
struct work_struct lineout_work;
struct work_struct hook_work;
struct snd_soc_codec *global_codec;
bool need_spk;
extern int PRJ_ID;
//extern unsigned int factory_mode;
extern int force_headphone;
//extern int audio_dock_in_out(u8 status);
extern bool isAudioStandIn(void);
extern int audio_stand_route(bool);

void set_lineout_state(bool status)
{
	if(status)
		switch_set_state(&hs_data->ldev, LINEOUT_IN);
	else
		switch_set_state(&hs_data->ldev, NO_DEVICE);
}
EXPORT_SYMBOL(set_lineout_state);

static bool is_support_dock(void)
{
	unsigned project_info = -1;
	unsigned tf300tg_hw = -1;
	bool support_dock = false;

	project_info = tegra3_get_project_id();
	tf300tg_hw = tegra3_query_audio_codec_pcbid();

	switch (project_info){
		case TEGRA3_PROJECT_TF201:
		case TEGRA3_PROJECT_TF300T:
		case TEGRA3_PROJECT_TF300TL:
		case TEGRA3_PROJECT_TF500T:
		case TEGRA3_PROJECT_TF700T:
			support_dock = true;
			break;
		case TEGRA3_PROJECT_TF300TG:
			if(tf300tg_hw != TEGRA3_DEVKIT_MISC_HW_0_ACODEC_3)
				support_dock = false;
			else
				support_dock = true;
			break;
		default:
			support_dock = false;
	}

	return support_dock;
}

static ssize_t lineout_name_show(struct switch_dev *ldev, char *buf)
{
	switch (switch_get_state(&hs_data->ldev)){
		case NO_LINEOUT:
			{
				return sprintf(buf, "%s\n", "No Device");
			}
		case LINEOUT_IN:
			{
				return sprintf(buf, "%s\n", "LINEOUT_IN");
			}
	}
	return -EINVAL;
}

static ssize_t lineout_state_show(struct switch_dev *ldev, char *buf)
{
	switch (switch_get_state(&hs_data->ldev)){
		case NO_LINEOUT:
			return sprintf(buf, "%d\n", 0);
		case LINEOUT_IN:
			return sprintf(buf, "%d\n", 1);
	}
	return -EINVAL;
}


static ssize_t headset_name_show(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs_data->sdev)){
		case NO_DEVICE:
			{
				return sprintf(buf, "%s\n", "No Device");
			}
		case HEADSET_WITH_MIC:
			{
				return sprintf(buf, "%s\n", "HEADSET");
			}
		case HEADSET_WITHOUT_MIC:
			{
				return sprintf(buf, "%s\n", "HEADPHONE");
			}
	}
	return -EINVAL;
}

static ssize_t headset_state_show(struct switch_dev *sdev, char *buf)
{
	switch (switch_get_state(&hs_data->sdev)){
		case NO_DEVICE:
			return sprintf(buf, "%d\n", 0);
		case HEADSET_WITH_MIC:
			return sprintf(buf, "%d\n", 1);
		case HEADSET_WITHOUT_MIC:
			return sprintf(buf, "%d\n", 2);
	}
	return -EINVAL;
}

static void insert_headset(void)
{
	unsigned int hook_irq = -1;

	if(gpio_get_value(hs_data->lineout_gpio) == 0 &&
			(project_info == TEGRA3_PROJECT_ME301T ||
			 project_info == TEGRA3_PROJECT_ME301TL) ){
		printk("%s: debug board\n", __func__);
		debugboard_alive = true;
		tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA0,
				TEGRA_TRI_NORMAL);
		gpio_direction_output(UART_HEADPHONE_SWITCH, 0);
		switch_set_state(&hs_data->sdev, NO_DEVICE);
		/* disable irq and irq_wake to avoid system can't suspend */
		disable_irq(hs_data->irq);
		disable_irq_wake(hs_data->irq);
		hook_irq = TEGRA_GPIO_TO_IRQ(HOOK_GPIO);
		enable_irq(hook_irq);
		hs_micbias_power(ON);
	}else if(gpio_get_value(HOOK_GPIO)
			/*|| (factory_mode && (force_headphone == FORCE_HEADPHONE))*/ ){
		printk("%s: headphone\n", __func__);
		switch_set_state(&hs_data->sdev, HEADSET_WITHOUT_MIC);
		hs_micbias_power(OFF);

		if(project_info == TEGRA3_PROJECT_ME570T){
			if(console_disable)
				gpio_direction_output(UART_HEADPHONE_SWITCH, 1);
			else
				gpio_direction_output(UART_HEADPHONE_SWITCH, 0);
		}
		if(project_info == TEGRA3_PROJECT_ME301T ||
				project_info == TEGRA3_PROJECT_ME301TL) {
			gpio_direction_output(UART_HEADPHONE_SWITCH, 1);
			tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA0,
					TEGRA_TRI_TRISTATE);
		}

		headset_alive = false;
	}else{
		printk("%s: headset\n", __func__);
		switch_set_state(&hs_data->sdev, HEADSET_WITH_MIC);
		hs_micbias_power(ON);

		if(project_info == TEGRA3_PROJECT_ME570T){
			if(console_disable)
				gpio_direction_output(UART_HEADPHONE_SWITCH, 1);
			else
				gpio_direction_output(UART_HEADPHONE_SWITCH, 0);
		}
		if(project_info == TEGRA3_PROJECT_ME301T ||
				project_info == TEGRA3_PROJECT_ME301TL) {
			gpio_direction_output(UART_HEADPHONE_SWITCH, 1);
			tegra_pinmux_set_tristate(TEGRA_PINGROUP_ULPI_DATA0,
					TEGRA_TRI_TRISTATE);
		}
		headset_alive = true;
	}
	hs_data->debouncing_time = ktime_set(0, 20000000); /* 20 ms */
}
EXPORT_SYMBOL(insert_headset);

static void remove_headset(void)
{
	printk("%s:++++++++++\n", __func__);
	switch_set_state(&hs_data->sdev, NO_DEVICE);
	hs_data->debouncing_time = ktime_set(0, 100000000); /* 100 ms */
	headset_alive = false;

	if(project_info == TEGRA3_PROJECT_ME301T ||
			project_info == TEGRA3_PROJECT_ME301TL ||
			project_info == TEGRA3_PROJECT_ME570T)
		gpio_direction_output(UART_HEADPHONE_SWITCH, 1);

	return;
}

static void detection_work(struct work_struct *work)
{
	unsigned long irq_flags;
	int cable_in1;
	int mic_in = 0;

	wake_lock(&hp_detect_wakelock);
	hs_micbias_power(ON);
	isHpDetecting = true;

	/* Disable headset interrupt while detecting.*/
	local_irq_save(irq_flags);
	disable_irq(hs_data->irq);
	local_irq_restore(irq_flags);

	/* Delay 1000ms for pin stable. */
	msleep(1000);

	/* Restore IRQs */
	local_irq_save(irq_flags);
	enable_irq(hs_data->irq);
	local_irq_restore(irq_flags);

	if (gpio_get_value(JACK_GPIO) != 0) {
		/* Headset not plugged in */
		if (switch_get_state(&hs_data->sdev) == HEADSET_WITH_MIC || switch_get_state(&hs_data->sdev) == HEADSET_WITHOUT_MIC)
			remove_headset();
		goto closed_micbias;
	}

	cable_in1 = gpio_get_value(JACK_GPIO);
	mic_in = gpio_get_value(HOOK_GPIO);
	if (cable_in1 == 0) {
		printk("HOOK_GPIO value: %d\n", mic_in);
		if(switch_get_state(&hs_data->sdev) == NO_DEVICE)
			insert_headset();
		else if ( mic_in == 1)
			goto closed_micbias;
	} else{
		printk("HEADSET: Jack-in GPIO is low, but not a headset \n");
		goto closed_micbias;
	}
	isHpDetecting = false;
	wake_unlock(&hp_detect_wakelock);
	return;

closed_micbias:
	isHpDetecting = false;
	hs_micbias_power(OFF);
	wake_unlock(&hp_detect_wakelock);
	return;
}

static enum hrtimer_restart detect_event_timer_func(struct hrtimer *data)
{
	queue_work(g_detection_work_queue, &g_detection_work);
	return HRTIMER_NORESTART;
}

/**********************************************************
 ** Function: Jack detection-in gpio configuration function
 ** Parameter: none
 ** Return value: if sucess, then returns 0
 **
 ************************************************************/
static int jack_config_gpio()
{
	int ret;

	printk("HEADSET: Config Jack-in detection gpio\n");
	hs_micbias_power(ON);
	ret = gpio_request(JACK_GPIO, "h2w_detect");
	ret = gpio_direction_input(JACK_GPIO);

	hs_data->irq = gpio_to_irq(JACK_GPIO);
	ret = request_irq(hs_data->irq, detect_irq_handler,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING, "h2w_detect", NULL);

	ret = irq_set_irq_wake(hs_data->irq, 1);

	switch (project_info) {
		case TEGRA3_PROJECT_TF201:
		case TEGRA3_PROJECT_TF300T:
		case TEGRA3_PROJECT_TF300TG:
		case TEGRA3_PROJECT_TF300TL:
		case TEGRA3_PROJECT_TF500T:
		case TEGRA3_PROJECT_TF700T:
		case TEGRA3_PROJECT_ME301T:
		case TEGRA3_PROJECT_ME301TL:
			/* delay 100ms to wait hook_det stable */
			msleep(100);
			break;
		case TEGRA3_PROJECT_P1801:
			/* delay 230ms to wait hook_det stable */
			msleep(230);
			break;
		default:
			/* delay 100ms to wait hook_det stable */
			msleep(100);
			break;
	}

	if (gpio_get_value(JACK_GPIO) == 0){
		insert_headset();
	}else {
		hs_micbias_power(OFF);
		headset_alive = false;
		switch_set_state(&hs_data->sdev, NO_DEVICE);
		remove_headset();
	}

	return 0;
}

/**********************************************************
 ** Function: Headset Hook Key Detection interrupt handler
 ** Parameter: irq
 ** Return value: IRQ_HANDLED
 ** High: Hook button pressed
 ************************************************************/
static int btn_config_gpio()
{
	int ret;
	unsigned long hook_irq = -1;

	printk("HEADSET: Config Headset Button detection gpio\n");

	ret = gpio_request(HOOK_GPIO, "btn_INT");
	ret = gpio_direction_input(HOOK_GPIO);

	if(project_info == TEGRA3_PROJECT_ME301T ||
			project_info == TEGRA3_PROJECT_ME301TL)
	{
		hook_irq = TEGRA_GPIO_TO_IRQ(HOOK_GPIO);
		ret = request_irq(hook_irq, hook_irq_handler,
				IRQF_TRIGGER_FALLING, "hook_detect", NULL);
		disable_irq(hook_irq);
	}

	return 0;
}

static void hook_work_queue(struct work_struct *work)
{
	unsigned int hook_irq = -1;

	hook_irq = TEGRA_GPIO_TO_IRQ(HOOK_GPIO);

	disable_irq(hook_irq);
}

static void lineout_work_queue(struct work_struct *work)
{
	msleep(300);

	if(!is_support_dock()){
		printk("%s: Not support dock\n",__func__);
		lineout_alive = false;
		switch_set_state(&hs_data->ldev, NO_DEVICE);
		return;
	}

	/* check if audio stand is inserted */
	if(!isAudioStandIn()){
		printk("LINEOUT: No Audio Stand in\n");
		return;
	}

	if (gpio_get_value(LINEOUT_GPIO) == 0){
		printk("LINEOUT: LineOut inserted\n");
		lineout_alive = true;
		audio_stand_route(true);
		switch_set_state(&hs_data->ldev, LINEOUT_IN);
	}else if(gpio_get_value(LINEOUT_GPIO)){
		printk("LINEOUT: LineOut removed\n");
		lineout_alive = false;
		audio_stand_route(false);
		switch_set_state(&hs_data->ldev, NO_DEVICE);
	}

}

/**********************************************************
 ** Function: LineOut Detection configuration function
 ** Parameter: none
 ** Return value: IRQ_HANDLED
 **
 ************************************************************/
static int lineout_config_gpio()
{
	int ret;

	printk("HEADSET: Config LineOut detection gpio\n");

	ret = gpio_request(hs_data->lineout_gpio, "lineout_int");
	ret = gpio_direction_input(hs_data->lineout_gpio);

	ret = request_irq(gpio_to_irq(hs_data->lineout_gpio),
			&lineout_irq_handler,
			IRQF_TRIGGER_FALLING|IRQF_TRIGGER_RISING,
			"lineout_int", 0);

	if(!is_support_dock()){
		printk("%s: Not support dock\n", __func__);
		lineout_alive = false;
		switch_set_state(&hs_data->ldev, NO_DEVICE);
	}else if (gpio_get_value(LINEOUT_GPIO) == 0){
		lineout_alive = true;
		audio_stand_route(true);
		switch_set_state(&hs_data->ldev, LINEOUT_IN);
	}else{
		lineout_alive = false;
		audio_stand_route(false);
		switch_set_state(&hs_data->ldev, NO_DEVICE);
	}

	return 0;
}


static int lineout_config_no_dock()
{
	int ret;

	printk("HEADSET: Config LineOut detection gpio without dock\n");

	ret = gpio_request(hs_data->lineout_gpio, "lineout_int");
	ret = gpio_direction_input(hs_data->lineout_gpio);
	switch_set_state(&hs_data->ldev, NO_DEVICE);

	return 0;
}

static int switch_config_gpio()
{
	int ret;

	printk("HEADSET: Config uart<->headphone gpio\n");

	ret = gpio_request(UART_HEADPHONE_SWITCH, "uart_headphone_switch");
	gpio_direction_output(UART_HEADPHONE_SWITCH, 1);

	return 0;
}

static irqreturn_t hook_irq_handler(int irq, void *dev_id)
{
	if(debugboard_alive){
		gpio_direction_output(UART_HEADPHONE_SWITCH, 1);
		/* Enable irq and irq_wake for Headset/Headphone detection */
		enable_irq(hs_data->irq);
		enable_irq_wake(hs_data->irq);
		schedule_work(&hook_work);
		debugboard_alive = false;
	}

	return IRQ_HANDLED;
}
/**********************************************************
 ** Function: LineOut detection interrupt handler
 ** Parameter: dedicated irq
 ** Return value: if sucess, then returns IRQ_HANDLED
 **
 ************************************************************/
static irqreturn_t lineout_irq_handler(int irq, void *dev_id)
{
	schedule_work(&lineout_work);
	return IRQ_HANDLED;
}

/**********************************************************
 ** Function: Headset jack-in detection interrupt handler
 ** Parameter: dedicated irq
 ** Return value: if sucess, then returns IRQ_HANDLED
 **
 ************************************************************/
static irqreturn_t detect_irq_handler(int irq, void *dev_id)
{
	int value1, value2;
	int retry_limit = 10;

	do {
		value1 = gpio_get_value(JACK_GPIO);
		irq_set_irq_type(hs_data->irq, value1 ?
				IRQF_TRIGGER_FALLING : IRQF_TRIGGER_RISING);
		value2 = gpio_get_value(JACK_GPIO);
	}while (value1 != value2 && retry_limit-- > 0);

	if ((switch_get_state(&hs_data->sdev) == NO_DEVICE) ^ value2){
		hrtimer_start(&hs_data->timer, hs_data->debouncing_time, HRTIMER_MODE_REL);
	}

	return IRQ_HANDLED;
}

static int codec_micbias_power(int on)
{
	unsigned int CtrlReg = 0;

	if(on){
		if(project_info == TEGRA3_PROJECT_TF201 || project_info == TEGRA3_PROJECT_TF300TG ||
				project_info == TEGRA3_PROJECT_TF700T || project_info == TEGRA3_PROJECT_TF300TL)
		{
			if(rt5631_audio_codec == NULL){
				printk("%s: No rt5631 rt5631_audio_codec - set micbias on fail\n", __func__);
				return 0;
			}
			/* Mic Bias enable */
			CtrlReg = snd_soc_read(rt5631_audio_codec, 0x3B);
			CtrlReg = CtrlReg | (1 << 3);
			snd_soc_write(rt5631_audio_codec, 0x3B, CtrlReg);
		}else if(project_info == TEGRA3_PROJECT_TF300T){
			if(wm8903_codec == NULL){
				printk("%s: No wm8903_codec - set micbias on fail\n", __func__);
				return 0;
			}
			CtrlReg = MICBIAS_ENA | MICDET_ENA;
			snd_soc_write(wm8903_codec, WM8903_MIC_BIAS_CONTROL_0, CtrlReg);
		}else if(project_info == TEGRA3_PROJECT_TF500T || project_info == TEGRA3_PROJECT_P1801 ||
				project_info == TEGRA3_PROJECT_ME301T ||
				project_info == TEGRA3_PROJECT_ME301TL ||
				project_info == TEGRA3_PROJECT_ME570T){
			if(rt5640_audio_codec == NULL){
				printk("%s: No RT5642_codec - set micbias on fail\n", __func__);
				return 0;
			}
			 /* Ensure the power is strong enough to drive MicBias2 */
			snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG1,
					     RT5640_PWR_MB | RT5640_PWR_VREF2,
					     RT5640_PWR_MB | RT5640_PWR_VREF2);
			snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG1, RT5640_PWR_LDO2, RT5640_PWR_LDO2); /* Enable LDO2 */
			snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG2, RT5640_PWR_MB1, RT5640_PWR_MB1); /*Enable MicBias1 */
		}
	}else{
		if(project_info == TEGRA3_PROJECT_TF201 || project_info == TEGRA3_PROJECT_TF300TG ||
				project_info == TEGRA3_PROJECT_TF700T || project_info == TEGRA3_PROJECT_TF300TL)
		{
			if(rt5631_audio_codec == NULL){
				printk("%s: No rt5631 rt5631_audio_codec - set micbias off fail\n", __func__);
				return 0;
			}
			/* Mic Bias diable*/
			CtrlReg = snd_soc_read(rt5631_audio_codec, 0x3B);
			CtrlReg = CtrlReg & (0xFFFFFFF7);
			snd_soc_write(rt5631_audio_codec, 0x3B, CtrlReg);
		}else if(project_info == TEGRA3_PROJECT_TF300T){
			if(wm8903_codec == NULL){
				printk("%s: No wm8903_codec - set micbias off fail\n", __func__);
				return 0;
			}
			CtrlReg = 0;
			snd_soc_write(wm8903_codec, WM8903_MIC_BIAS_CONTROL_0, CtrlReg);
		}else if(project_info == TEGRA3_PROJECT_TF500T || project_info == TEGRA3_PROJECT_P1801 ||
				project_info == TEGRA3_PROJECT_ME301T ||
				project_info == TEGRA3_PROJECT_ME301TL ||
				project_info == TEGRA3_PROJECT_ME570T){
			if(rt5640_audio_codec == NULL){
				printk("%s: No RT5642_codec - set micbias on fail\n", __func__);
				return 0;
			}
			snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG2, RT5640_PWR_MB1, 0); /* Disable MicBias1 */
			snd_soc_update_bits(rt5640_audio_codec, RT5640_PWR_ANLG1, RT5640_PWR_LDO2, 0); /* Disable LDO2 */
		}
	}
	return 0;
}


int hs_micbias_power(int on)
{
	static int nLastVregStatus = -1;

	if(on && nLastVregStatus!=ON){
		printk("HEADSET: Turn on micbias power\n");
		nLastVregStatus = ON;
		codec_micbias_power(ON);
	}
	else if(!on && nLastVregStatus!=OFF){
		printk("HEADSET: Turn off micbias power\n");
		nLastVregStatus = OFF;
		codec_micbias_power(OFF);
	}
	return 0;
}
EXPORT_SYMBOL(hs_micbias_power);


/**********************************************************
 ** Function: Headset driver init function
 ** Parameter: none
 ** Return value: none
 **
 ************************************************************/
static int __init headset_init(void)
{
	int ret;
	printk(KERN_INFO "%s+ #####\n", __func__);

	printk("HEADSET: Headset detection init\n");

	console_disable = is_tegra_debug_uartport_hs();
	project_info = tegra3_get_project_id();

	hs_data = kzalloc(sizeof(struct headset_data), GFP_KERNEL);
	if (!hs_data)
		return -ENOMEM;

	hs_data->debouncing_time = ktime_set(0, 100000000); /* 100 ms */
	hs_data->sdev.name = "h2w";
	hs_data->sdev.print_name = headset_name_show;
	hs_data->sdev.print_state = headset_state_show;

	hs_data->ldev.name = "lineout";
	hs_data->ldev.print_name = lineout_name_show;
	hs_data->ldev.print_state = lineout_state_show;

	ret = switch_dev_register(&hs_data->sdev);
	if (ret < 0)
		goto err_switch_dev_register;

	ret = switch_dev_register(&hs_data->ldev);
	if (ret < 0)
		goto err_switch_dev_register;

	g_detection_work_queue = create_workqueue("detection");

	hrtimer_init(&hs_data->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	hs_data->timer.function = detect_event_timer_func;

	printk("HEADSET: Headset detection mode\n");
	btn_config_gpio();/*Config hook detection GPIO*/

	if(project_info == TEGRA3_PROJECT_ME301T ||
			project_info == TEGRA3_PROJECT_ME301TL ||
			project_info == TEGRA3_PROJECT_ME570T)
		switch_config_gpio(); /*Config uart and headphone switch*/
	wake_lock_init(&hp_detect_wakelock, WAKE_LOCK_SUSPEND,
		"headset detection");
	INIT_WORK(&lineout_work, lineout_work_queue);
	if (project_info ==  TEGRA3_PROJECT_ME301T ||
		project_info == TEGRA3_PROJECT_ME301TL)
		INIT_WORK(&hook_work, hook_work_queue);

	switch (project_info) {
		case TEGRA3_PROJECT_TF201:
		case TEGRA3_PROJECT_TF300T:
		case TEGRA3_PROJECT_TF300TG:
		case TEGRA3_PROJECT_TF300TL:
		case TEGRA3_PROJECT_TF500T:
		case TEGRA3_PROJECT_TF700T:
			hs_data->lineout_gpio = LINEOUT_GPIO;
			lineout_config_gpio();
			break;
		case TEGRA3_PROJECT_ME301T:
			hs_data->lineout_gpio = LINEOUT_ME301T;
			lineout_config_no_dock();
			break;
		default:
			hs_data->lineout_gpio = LINEOUT_GPIO;
			lineout_config_gpio();
			break;
	}

	jack_config_gpio();/*Config jack detection GPIO*/

	printk(KERN_INFO "%s- #####\n", __func__);
	return 0;

err_switch_dev_register:
	printk(KERN_ERR "Headset: Failed to register driver\n");

	return ret;
}

/**********************************************************
 ** Function: Headset driver exit function
 ** Parameter: none
 ** Return value: none
 **
 ************************************************************/
static void __exit headset_exit(void)
{
	printk("HEADSET: Headset exit\n");
	if (switch_get_state(&hs_data->sdev))
		remove_headset();
	gpio_free(JACK_GPIO);
	gpio_free(HOOK_GPIO);
	gpio_free(LINEOUT_GPIO);

	wake_lock_destroy(&hp_detect_wakelock);
	free_irq(hs_data->irq, 0);
	destroy_workqueue(g_detection_work_queue);
	switch_dev_unregister(&hs_data->sdev);
}

module_init(headset_init);
module_exit(headset_exit);
