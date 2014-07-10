#ifndef _aw8ec_H
#define _aw8ec_H

#include <linux/switch.h>
#include <linux/wakelock.h>

/*
 * compiler option
 */
#define aw8ec_DEBUG			0
#define TOUCHPAD_MODE			1	// 0: relative mode, 1: absolute mode
#define TOUCHPAD_ELAN			1	// 0: not elan, 1:elantech
#define DOCK_SPEAKER			0	// 0: not ready, 1: ready
#define DOCK_USB			1	// 0: not ready, 1: ready
#define BATTERY_DRIVER			1	// 0: not ready, 1: ready
/*
 * Debug Utility
 */
#if aw8ec_DEBUG
#define aw8ec_INFO(format, arg...)	\
	printk(KERN_INFO "aw8ec: [%s] " format , __FUNCTION__ , ## arg)
#define aw8ec_I2C_DATA(array, i)	\
					do {		\
						for (i = 0; i < array[0]+1; i++) \
							aw8ec_INFO("ec_data[%d] = 0x%x\n", i, array[i]);	\
					} while(0)
#else
#define aw8ec_INFO(format, arg...)
#define aw8ec_I2C_DATA(array, i)
#endif

#define aw8ec_NOTICE(format, arg...)	\
	printk(KERN_NOTICE "aw8ec: [%s] " format , __FUNCTION__ , ## arg)

#define aw8ec_ERR(format, arg...)	\
	printk(KERN_ERR "aw8ec: [%s] " format , __FUNCTION__ , ## arg)

//----------------------------------------

#define DRIVER_DESC     		"ASUS Dock EC Driver"
#define DOCK_SDEV_NAME			"dock"
#define W8_SDEV_NAME			"aio"
#define SCALAR_SDEV_NAME		"scalar_status"
#define CONVERSION_TIME_MS		50

#define aw8ec_I2C_ERR_TOLERANCE	8
#define aw8ec_RETRY_COUNT		3
#define aw8ec_POLLING_RATE		80

#define aw8ec_OBF_MASK		0x1
#define aw8ec_KEY_MASK		0x4
#define aw8ec_KBC_MASK		0x8
#define aw8ec_AUX_MASK		0x20
#define aw8ec_SCI_MASK		0x40
#define aw8ec_SMI_MASK		0x80


#define aw8ec_RELATIVE_MODE		0
#define aw8ec_ABSOLUTE_MODE		1

#define NUM_RELATIVE_MODE		3	// The number of bytes per packet in relative mode
#define NUM_ABSOLUTE_MODE		6	// The number of bytes per packet in absolute mode

/* relative mode packet formate */
#define Y_OVERFLOW_MASK			0x80
#define X_OVERFLOW_MASK			0x40
#define Y_SIGN_MASK			0x20
#define X_SIGN_MASK			0x10
#define RIGHT_BTN_MASK			0x2
#define LEFT_BTN_MASK			0x1

/* absolute mode packet formate */
#define TOUCHPAD_SAMPLE_RATE		20
#define ABSOLUTE_BIT_MASK		0x80
#define Z_SNESITIVITY			30
#define X_LIMIT				5600
#define Y_LIMIT				1300
#define X_MAX				5855
#define X_MIN				1198
#define Y_MAX				4942
#define Y_MIN				946

#define aw8ec_PS2_ACK			0xFA

//-----------------------------------------
#define aw8ec_KEY_TOUCHPAD	KEY_F2
#define aw8ec_KEY_AUTOBRIGHT	KEY_F3
#define aw8ec_KEY_SETTING		KEY_F4

/*************scan 2 make mapping***************/
#define aw8ec_KEYPAD_ESC		0x76
#define aw8ec_KEYPAD_KEY_WAVE		0x0E
#define aw8ec_KEYPAD_KEY_1		0x16
#define aw8ec_KEYPAD_KEY_2		0X1E
#define aw8ec_KEYPAD_KEY_3		0x26
#define aw8ec_KEYPAD_KEY_4		0x25
#define aw8ec_KEYPAD_KEY_5		0x2E
#define aw8ec_KEYPAD_KEY_6        	0x36
#define aw8ec_KEYPAD_KEY_7        	0x3D
#define aw8ec_KEYPAD_KEY_8        	0x3E
#define aw8ec_KEYPAD_KEY_9        	0x46
#define aw8ec_KEYPAD_KEY_0        	0x45
#define aw8ec_KEYPAD_KEY_MINUS    	0x4E
#define aw8ec_KEYPAD_KEY_EQUAL		0x55
#define aw8ec_KEYPAD_KEY_BACKSPACE	0x66
#define aw8ec_KEYPAD_KEY_TAB      	0x0D
#define aw8ec_KEYPAD_KEY_Q        	0x15
#define aw8ec_KEYPAD_KEY_W        	0x1D
#define aw8ec_KEYPAD_KEY_E        	0x24
#define aw8ec_KEYPAD_KEY_R        	0x2D
#define aw8ec_KEYPAD_KEY_T        	0x2C
#define aw8ec_KEYPAD_KEY_Y        	0x35
#define aw8ec_KEYPAD_KEY_U        	0x3C
#define aw8ec_KEYPAD_KEY_I        	0x43
#define aw8ec_KEYPAD_KEY_O        	0x44
#define aw8ec_KEYPAD_KEY_P        	0x4D
#define aw8ec_KEYPAD_KEY_LEFTBRACE	0x54
#define aw8ec_KEYPAD_KEY_RIGHTBRACE 	0x5B
#define aw8ec_KEYPAD_KEY_BACKSLASH	0x5D
#define aw8ec_KEYPAD_KEY_CAPSLOCK 	0x58
#define aw8ec_KEYPAD_KEY_A        	0x1C
#define aw8ec_KEYPAD_KEY_S        	0x1B
#define aw8ec_KEYPAD_KEY_D        	0x23
#define aw8ec_KEYPAD_KEY_F        	0x2B
#define aw8ec_KEYPAD_KEY_G        	0x34
#define aw8ec_KEYPAD_KEY_H        	0x33
#define aw8ec_KEYPAD_KEY_J        	0x3B
#define aw8ec_KEYPAD_KEY_K        	0x42
#define aw8ec_KEYPAD_KEY_L        	0x4B
#define aw8ec_KEYPAD_KEY_SEMICOLON	0x4C
#define aw8ec_KEYPAD_KEY_APOSTROPHE	0x52
#define aw8ec_KEYPAD_KEY_ENTER    	0x5A
#define aw8ec_KEYPAD_KEY_LEFTSHIFT 	0x12
#define aw8ec_KEYPAD_KEY_Z        	0x1A
#define aw8ec_KEYPAD_KEY_X        	0x22
#define aw8ec_KEYPAD_KEY_C        	0x21
#define aw8ec_KEYPAD_KEY_V        	0x2A
#define aw8ec_KEYPAD_KEY_B        	0x32
#define aw8ec_KEYPAD_KEY_N        	0x31
#define aw8ec_KEYPAD_KEY_M        	0x3A
#define aw8ec_KEYPAD_KEY_COMMA    	0x41
#define aw8ec_KEYPAD_KEY_DOT   	0x49
#define aw8ec_KEYPAD_KEY_SLASH    	0x4A
#define aw8ec_KEYPAD_KEY_RIGHTSHIFT   	0x59

#define aw8ec_KEYPAD_KEY_LEFT   	0xE06B
#define aw8ec_KEYPAD_KEY_RIGHT   	0xE074
#define aw8ec_KEYPAD_KEY_UP		0xE075
#define aw8ec_KEYPAD_KEY_DOWN		0xE072

#define aw8ec_KEYPAD_RIGHTWIN		0xE027
#define aw8ec_KEYPAD_LEFTCTRL		0x14
#define aw8ec_KEYPAD_LEFTWIN		0xE01F
#define aw8ec_KEYPAD_LEFTALT		0x11
#define aw8ec_KEYPAD_KEY_SPACE		0x29
#define aw8ec_KEYPAD_RIGHTALT		0xE011
#define aw8ec_KEYPAD_WINAPP		0xE02F
#define aw8ec_KEYPAD_RIGHTCTRL		0xE014
#define aw8ec_KEYPAD_HOME		0xE06C
#define aw8ec_KEYPAD_PAGEUP		0xE07D
#define aw8ec_KEYPAD_PAGEDOWN		0xE07A
#define aw8ec_KEYPAD_END		0xE069
/************  JP keys *************/
#define aw8ec_HANKAKU_ZENKAKU		0x5F
#define aw8ec_YEN			0x6A
#define aw8ec_MUHENKAN			0x67
#define aw8ec_HENKAN			0x64
#define aw8ec_HIRAGANA_KATAKANA		0x13
#define aw8ec_RO			0x51
/********************************/
/************  UK keys *************/
#define aw8ec_EUROPE_2			0x61
/********************************/


#define aw8ec_KEYPAD_LOCK		0xE071

#define aw8ec_KEYPAD_KEY_BREAK   	0xF0
#define aw8ec_KEYPAD_KEY_EXTEND   	0xE0

/*************scan 2 make code mapping***************/

/************* SMI event ********************/
#define aw8ec_SMI_HANDSHAKING		0x50
#define aw8ec_SMI_WAKE			0x53
#define aw8ec_SMI_RESET			0x5F
#define aw8ec_SMI_ADAPTER_EVENT		0x60
#define aw8ec_SMI_BACKLIGHT_ON		0x63
/*************IO control setting***************/
#define aw8ec_IOCTL_HEAVY	2
#define aw8ec_IOCTL_NORMAL	1
#define aw8ec_IOCTL_END	0
#define aw8ec_CPAS_LED_ON	1
#define aw8ec_CPAS_LED_OFF	0
#define aw8ec_TP_ON	1
#define aw8ec_TP_OFF	0
#define aw8ec_EC_ON	1
#define aw8ec_EC_OFF	0
#define aw8ec_SPLASHTOP_ON	1  //new
#define aw8ec_SPLASHTOP_OFF	0  //new
#define aw8ec_IOC_MAGIC	0xf4
#define aw8ec_IOC_MAXNR	8
#define aw8ec_POLLING_DATA	_IOR(aw8ec_IOC_MAGIC,	1,	int)
#define aw8ec_FW_UPDATE 		_IOR(aw8ec_IOC_MAGIC,	2,	int)
#define aw8ec_CPASLOCK_LED	_IOR(aw8ec_IOC_MAGIC,	3,	int)
#define aw8ec_INIT			_IOR(aw8ec_IOC_MAGIC,	4,	int)
#define aw8ec_TP_CONTROL		_IOR(aw8ec_IOC_MAGIC,	5,	int)
#define aw8ec_EC_WAKEUP		_IOR(aw8ec_IOC_MAGIC,	6,	int)
#define aw8ec_FW_DUMMY		_IOR(aw8ec_IOC_MAGIC, 7,	int)
#define aw8ec_DETECT_SPLASHTOP  _IOR(aw8ec_IOC_MAGIC, 8,	int)
/*************IO control setting***************/

/************* EC FW update ***********/
#define EC_BUFF_LEN  256
/********************** ***********/

/*
 * The x/y limits are taken from the Synaptics TouchPad interfacing Guide,
 * section 2.3.2, which says that they should be valid regardless of the
 * actual size of the sensor.
 */
#define XMIN_NOMINAL 0
#define XMAX_NOMINAL 1279
#define YMIN_NOMINAL 0
#define YMAX_NOMINAL 799

/*
 * data struct
 */

struct aw8ec_keypad{
	int value;
	int input_keycode;
	int extend;
};

struct aw8ec_touchpad_relative{
	int y_overflow;
	int x_overflow;
	int y_sign;
	int x_sign;
	int left_btn;
	int right_btn;
	int delta_x;
	int delta_y;
};

struct aw8ec_touchpad_absolute{
	int w_val;
	int x_pos;
	int y_pos;
	int z_val;
	int left;
	int right;
	int x_prev;
	int y_prev;
	int z_prev;
	int x2_pos;
	int y2_pos;
	int z2_val;
};

struct aw8ec_chip {
	struct input_dev	*indev;
	struct input_dev	*lid_indev;
	struct switch_dev 	dock_sdev;
	struct switch_dev  w8_sdev;  //new
	struct switch_dev  scalar_status_sdev;
	struct i2c_client	*client;
	struct mutex		lock;
	struct mutex		kbc_lock;
	struct mutex		input_lock;
	struct mutex		dock_init_lock;
	struct wake_lock 	wake_lock;
	struct wake_lock 	wake_lock_init;
	struct wake_lock	wake_lock_w8;
	struct delayed_work aw8ec_work;
	struct delayed_work aw8ec_dock_init_work;
	struct delayed_work aw8ec_fw_update_work;
	struct delayed_work aw8ec_led_on_work;
	struct delayed_work aw8ec_led_off_work;
	struct delayed_work aw8ec_hall_sensor_work;
	struct delayed_work aw8ec_w8_work;
	struct delayed_work aw8ec_scalar_status_work;
#if DOCK_SPEAKER
	struct delayed_work aw8ec_audio_report_work;
#endif
	struct delayed_work aw8ec_pad_battery_report_work;
	struct aw8ec_keypad keypad_data;
	struct elantech_data *private;
	struct timer_list aw8ec_timer;
#if TOUCHPAD_MODE
	struct aw8ec_touchpad_absolute t_abs;
#else
	struct aw8ec_touchpad_relative touchpad_data;
#endif
	int ret_val;
	u8 ec_data[32];
	u8 i2c_data[32];
	u8 i2c_dm_data[32];
	int bc;			// byte counter
	int index;		// for message
	int status;
	int touchpad_member;
	char ec_model_name[32];
	char ec_version[32];
	char dock_pid[32];
	int polling_rate;
	int dock_in;	// 0: without dock, 1: with dock
	//new
	int w8_on;    // 0:w8 off, 1:w8 on
	int scalar_status; //0:dispaly pad, 1: dispaly win8
	int op_mode;	// 0: normal mode, 1: fw update mode
	int kbc_value;	// capslock_led 0: led off, 1: led on
	int dock_det;	// dock-in interrupt count
	int dock_init;	// 0: dock not init, 1: dock init successfully
	int d_index;	// touchpad byte counter
	int suspend_state; // 0: normal, 1: suspend
	int init_success; // 0: ps/2 not ready. 1: init OK, -1: tp not ready
	int wakeup_lcd;		// 0 : keep lcd state 1: make lcd on
	int tp_wait_ack;	// 0 : normal mode, 1: waiting for an ACK
	int tp_enable;		// 0 : touchpad has not enabled, 1: touchpad has enabled
	int re_init;		// 0 : first time init, not re-init, 1: in re-init procedure
	int ec_wakeup;		// 0 : ec shutdown when PAD in LP0, 1 : keep ec active when PAD in LP0,
	int ap_wake_wakeup;	// 0 : no ap_wake wakeup signal, 1: get ap_wake wakeup signal
	int tf_dock;		// 0 : not tf dock, 1: tf dock
	int dock_behavior;	// 0: susb_on follows wakeup event, 1: susb_on follows ec_req
	int ec_in_s3;		// 0: normal mode, 1: ec in deep sleep mode
	int susb_on;	// 0: susb off, 1: susb on
};

#endif

