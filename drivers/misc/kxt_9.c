/*
 * Copyright (C) 2009 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
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

#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/kxt_9.h>

#define NAME			"KXT_9"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define INT_SRC_REG1		0x15
#define INT_STATUS_REG		0x16
#define TILT_POS_CUR		0x10
#define INT_REL			0x1A
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define DATA_CTRL		0x21
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define CTRL_REG3		0x1D
#define TILT_TIMER		0x28
#define WUF_TIMER		0x29
#define WUF_THRESH		0x5A
#define TDT_TIMER		0x2B
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x00
#define PC1_ON			0x80
/* INTERRUPT SOURCE 2 BITS */
#define TPS			0x01
#define TDTS0			0x04
#define TDTS1			0x08
/* INPUT_ABS CONSTANTS */
#define FUZZ			32
#define FLAT			32
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RES_TILT_TIMER		3
#define RES_CTRL_REG3		4
#define RES_WUF_TIMER		5
#define RES_WUF_THRESH		6
#define RES_TDT_TIMER		7
#define RES_TDT_H_THRESH	8
#define RES_TDT_L_THRESH	9
#define RES_TAP_TIMER		10
#define RES_TOTAL_TIMER		11
#define RES_LAT_TIMER		12
#define RES_WIN_TIMER		13
#define RESUME_ENTRIES		14

#define KXTF9_CALIBRATION_PATH "/data/sensors/Accel_Config.ini"
#define REG_NUM 	96	/* Number of registers about KXTF9*/
/*
 * The following table lists the maximum appropriate poll interval for each
 * available output data rate.
 */
struct {
	unsigned int cutoff;
	u8 mask;
} KXT_9_odr_table[] = {
	{
	3,	ODR800F}, {
	5,	ODR400F}, {
	10,	ODR200F}, {
	20,	ODR100F}, {
	40,	ODR50F}, {
	80,	ODR25F}, {
	0,	ODR12_5F},
};

struct KXT_9_data {
	struct i2c_client *client;
	struct KXT_9_platform_data *pdata;
	struct mutex lock;
	struct delayed_work input_work;
	struct input_dev *input_dev;
	struct work_struct irq_work;

	int hw_initialized;
	atomic_t enabled;
	u8 resume[RESUME_ENTRIES];
	int res_interval;
	int irq;

	/* for set accel bias */
	int raw_xyz[3];
	int offset[3];
	int sensitivity[3];
	bool fLoadConfig;
};

static int access_calibration_file(int *offset, int *sensitivity)
{
	char buf[256];
	int ii, ret = 0;
	struct file *fp = NULL;
	mm_segment_t oldfs;
	int max[3] = {0};
	int min[3] = {0};

	oldfs=get_fs();
	set_fs(get_ds());
	memset(buf, 0, sizeof(u8)*256);

	fp=filp_open(KXTF9_CALIBRATION_PATH, O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		printk("kxtf9 open config file success\n");
		ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);

		printk("kxtf9 config content is :%s\n", buf);
		sscanf(buf,"%6d %6d %6d %6d %6d %6d\n",
			&max[0], &min[0], &max[1], &min[1], &max[2], &min[2]);

		for(ii = 0; ii < 3; ii++) {
			sensitivity[ii] = (max[ii] -min[ii]) >> 1;
			offset[ii] = min[ii] + sensitivity[ii];
		}
		printk("kxtf9: Offset: %d, %d, %d\nkxtf9: Sensitivity: %d, %d, %d\n",
			offset[0], offset[1], offset[2],
			sensitivity[0], sensitivity[1], sensitivity[2]);

		filp_close(fp, NULL);
		set_fs(oldfs);
		return 0;
	}
	else {

		for(ii = 0; ii < 3; ii++) {
			offset[ii] = 0;
			sensitivity[ii] = 1024;
		}

		printk("No kxtf9 calibration file\n");

		set_fs(oldfs);
		return -1;
	}

}

static int KXT_9_i2c_read(struct KXT_9_data *tf9, u8 addr, u8 *data, int len)
{
	int err;

	struct i2c_msg msgs[] = {
		{
		 .addr = tf9->client->addr,
		 .flags = tf9->client->flags & I2C_M_TEN,
		 .len = 1,
		 .buf = &addr,
		 },
		{
		 .addr = tf9->client->addr,
		 .flags = (tf9->client->flags & I2C_M_TEN) | I2C_M_RD,
		 .len = len,
		 .buf = data,
		 },
	};
	err = i2c_transfer(tf9->client->adapter, msgs, 2);

	if (err != 2)
		dev_err(&tf9->client->dev, "read transfer error\n");
	else
		err = 0;

	return err;
}

static int KXT_9_i2c_write(struct KXT_9_data *tf9, u8 addr, u8 *data, int len)
{
	int err;
	int i;
	u8 buf[len + 1];

	struct i2c_msg msgs[] = {
		{
		 .addr = tf9->client->addr,
		 .flags = tf9->client->flags & I2C_M_TEN,
		 .len = len + 1,
		 .buf = buf,
		 },
	};

	buf[0] = addr;
	for (i = 0; i < len; i++)
		buf[i + 1] = data[i];

	err = i2c_transfer(tf9->client->adapter, msgs, 1);

	if (err != 1)
		dev_err(&tf9->client->dev, "write transfer error\n");
	else
		err = 0;

	return err;
}

static int KXT_9_verify(struct KXT_9_data *tf9)
{
	int err;
	u8 buf;

	err = KXT_9_i2c_read(tf9, WHO_AM_I, &buf, 1);
	/*** DEBUG OUTPUT - REMOVE ***/
	dev_info(&tf9->client->dev, "WHO_AM_I = 0x%02x\n", buf);
	/*** <end> DEBUG OUTPUT - REMOVE ***/
	if (err < 0)
		dev_err(&tf9->client->dev, "read err int source\n");
	//if (buf != Chip_ID)
		//err = -1;
	return err;
}

static int KXT_9_hw_init(struct KXT_9_data *tf9)
{
	int err = -1;
	u8 buf[7];

	buf[0] = PC1_OFF;
	err = KXT_9_i2c_write(tf9, CTRL_REG1, buf, 1);
	if (err < 0)
		return err;
	err = KXT_9_i2c_write(tf9, DATA_CTRL, &tf9->resume[RES_DATA_CTRL], 1);
	if (err < 0)
		return err;
	err = KXT_9_i2c_write(tf9, CTRL_REG3, &tf9->resume[RES_CTRL_REG3], 1);
	if (err < 0)
		return err;
	err = KXT_9_i2c_write(tf9, TILT_TIMER, &tf9->resume[RES_TILT_TIMER], 1);
	if (err < 0)
		return err;
	err = KXT_9_i2c_write(tf9, WUF_TIMER, &tf9->resume[RES_WUF_TIMER], 1);
	if (err < 0)
		return err;
	err = KXT_9_i2c_write(tf9, WUF_THRESH, &tf9->resume[RES_WUF_THRESH], 1);
	if (err < 0)
		return err;
	buf[0] = tf9->resume[RES_TDT_TIMER];
	buf[1] = tf9->resume[RES_TDT_H_THRESH];
	buf[2] = tf9->resume[RES_TDT_L_THRESH];
	buf[3] = tf9->resume[RES_TAP_TIMER];
	buf[4] = tf9->resume[RES_TOTAL_TIMER];
	buf[5] = tf9->resume[RES_LAT_TIMER];
	buf[6] = tf9->resume[RES_WIN_TIMER];
	err = KXT_9_i2c_write(tf9, TDT_TIMER, buf, 7);
	if (err < 0)
		return err;
	err = KXT_9_i2c_write(tf9, INT_CTRL1, &tf9->resume[RES_INT_CTRL1], 1);
	if (err < 0)
		return err;
	buf[0] = (tf9->resume[RES_CTRL_REG1] | PC1_ON);
	err = KXT_9_i2c_write(tf9, CTRL_REG1, buf, 1);
	if (err < 0)
		return err;
	tf9->resume[RES_CTRL_REG1] = buf[0];
	tf9->hw_initialized = 1;

	return 0;
}

static void KXT_9_device_power_off(struct KXT_9_data *tf9)
{
	int err;
	u8 buf = PC1_OFF;

	err = KXT_9_i2c_write(tf9, CTRL_REG1, &buf, 1);
	if (err < 0)
		dev_err(&tf9->client->dev, "soft power off failed\n");
	disable_irq(tf9->irq);
	if (tf9->pdata->power_off)
		tf9->pdata->power_off();
	tf9->hw_initialized = 0;
}

static int KXT_9_device_power_on(struct KXT_9_data *tf9)
{
	int err;
	if (tf9->pdata->power_on) {
		err = tf9->pdata->power_on();
		if (err < 0)
			return err;
	}
	enable_irq(tf9->irq);
	if (!tf9->hw_initialized) {
		mdelay(100);
		err = KXT_9_hw_init(tf9);
		if (err < 0) {
			KXT_9_device_power_off(tf9);
			return err;
		}
	}
	return 0;
}

static irqreturn_t KXT_9_isr(int irq, void *dev)
{
	struct KXT_9_data *tf9 = dev;

	disable_irq_nosync(irq);
	schedule_work(&tf9->irq_work);

	return IRQ_HANDLED;
}

static u8 KXT_9_resolve_dir(struct KXT_9_data *tf9, u8 dir)
{
	switch (dir) {
	case 0x20:	/* -X */
		if (tf9->pdata->negate_x)
			dir = 0x10;
		if (tf9->pdata->axis_map_y == 0)
			dir >>= 2;
		if (tf9->pdata->axis_map_z == 0)
			dir >>= 4;
		break;
	case 0x10:	/* +X */
		if (tf9->pdata->negate_x)
			dir = 0x20;
		if (tf9->pdata->axis_map_y == 0)
			dir >>= 2;
		if (tf9->pdata->axis_map_z == 0)
			dir >>= 4;
		break;
	case 0x08:	/* -Y */
		if (tf9->pdata->negate_y)
			dir = 0x04;
		if (tf9->pdata->axis_map_x == 1)
			dir <<= 2;
		if (tf9->pdata->axis_map_z == 1)
			dir >>= 2;
		break;
	case 0x04:	/* +Y */
		if (tf9->pdata->negate_y)
			dir = 0x08;
		if (tf9->pdata->axis_map_x == 1)
			dir <<= 2;
		if (tf9->pdata->axis_map_z == 1)
			dir >>= 2;
		break;
	case 0x02:	/* -Z */
		if (tf9->pdata->negate_z)
			dir = 0x01;
		if (tf9->pdata->axis_map_x == 2)
			dir <<= 4;
		if (tf9->pdata->axis_map_y == 2)
			dir <<= 2;
		break;
	case 0x01:	/* +Z */
		if (tf9->pdata->negate_z)
			dir = 0x02;
		if (tf9->pdata->axis_map_x == 2)
			dir <<= 4;
		if (tf9->pdata->axis_map_y == 2)
			dir <<= 2;
		break;
	default:
		return -EINVAL;
	}

	return dir;
}

static void KXT_9_irq_work_func(struct work_struct *work)
{
/*
 *	int_status output:
 *	[INT_SRC_REG2][INT_SRC_REG1][TILT_POS_PRE][TILT_POS_CUR]
 *	INT_SRC_REG1, TILT_POS_PRE, and TILT_POS_CUR directions are translated
 *	based on platform data variables.
 */

	int err;
	int int_status = 0;
	u8 status;
	u8 buf[2];

	struct KXT_9_data *tf9
			= container_of(work, struct KXT_9_data, irq_work);

	err = KXT_9_i2c_read(tf9, INT_STATUS_REG, &status, 1);
	if (err < 0)
		dev_err(&tf9->client->dev, "read err int source\n");
	int_status = status << 24;

	if ((status & TPS) > 0) {
		err = KXT_9_i2c_read(tf9, TILT_POS_CUR, buf, 2);
		if (err < 0)
			dev_err(&tf9->client->dev, "read err tilt dir\n");
		int_status |= KXT_9_resolve_dir(tf9, buf[0]);
		int_status |= KXT_9_resolve_dir(tf9, buf[1]) << 8;
		/*** DEBUG OUTPUT - REMOVE ***/
		dev_info(&tf9->client->dev, "IRQ TILT [%x]\n",
						KXT_9_resolve_dir(tf9, buf[0]));
		/*** <end> DEBUG OUTPUT - REMOVE ***/
	}
	if (((status & TDTS0) | (status & TDTS1)) > 0) {
		err = KXT_9_i2c_read(tf9, INT_SRC_REG1, buf, 1);
		if (err < 0)
			dev_err(&tf9->client->dev, "read err tap dir\n");
		int_status |= (KXT_9_resolve_dir(tf9, buf[0])) << 16;
		/*** DEBUG OUTPUT - REMOVE ***/
		dev_info(&tf9->client->dev, "IRQ TAP%d [%x]\n",
		((status & TDTS1) ? (2) : (1)), KXT_9_resolve_dir(tf9, buf[0]));
		/*** <end> DEBUG OUTPUT - REMOVE ***/
	}
	/*** DEBUG OUTPUT - REMOVE ***/
	if ((status & 0x02) > 0) {
		if (((status & TDTS0) | (status & TDTS1)) > 0)
			dev_info(&tf9->client->dev, "IRQ WUF + TAP\n");
		else
			dev_info(&tf9->client->dev, "IRQ WUF\n");
	}
	/*** <end> DEBUG OUTPUT - REMOVE ***/
	if (int_status & 0x2FFF) {
		input_report_abs(tf9->input_dev, ABS_MISC, int_status);
		input_sync(tf9->input_dev);
	}
	err = KXT_9_i2c_read(tf9, INT_REL, buf, 1);
	if (err < 0)
		dev_err(&tf9->client->dev,
				"error clearing interrupt status: %d\n", err);

	enable_irq(tf9->irq);
}

int KXT_9_update_g_range(struct KXT_9_data *tf9, u8 new_g_range)
{
	int err;
	u8 shift;
	u8 buf;

	switch (new_g_range) {
	case KXT_9_G_2G:
		shift = SHIFT_ADJ_2G;
		break;
	case KXT_9_G_4G:
		shift = SHIFT_ADJ_4G;
		break;
	case KXT_9_G_8G:
		shift = SHIFT_ADJ_8G;
		break;
	default:
		dev_err(&tf9->client->dev, "invalid g range request\n");
		return -EINVAL;
	}
	if (shift != tf9->pdata->shift_adj) {
		if (tf9->pdata->shift_adj > shift)
			tf9->resume[RES_WUF_THRESH] >>=
						(tf9->pdata->shift_adj - shift);
		if (tf9->pdata->shift_adj < shift)
			tf9->resume[RES_WUF_THRESH] <<=
						(shift - tf9->pdata->shift_adj);

		if (atomic_read(&tf9->enabled)) {
			buf = PC1_OFF;
			err = KXT_9_i2c_write(tf9, CTRL_REG1, &buf, 1);
			if (err < 0)
				return err;
			buf = tf9->resume[RES_WUF_THRESH];
			err = KXT_9_i2c_write(tf9, WUF_THRESH, &buf, 1);
			if (err < 0)
				return err;
			buf = (tf9->resume[RES_CTRL_REG1] & 0xE7) | new_g_range;
			err = KXT_9_i2c_write(tf9, CTRL_REG1, &buf, 1);
			if (err < 0)
				return err;
		}
	}else
		buf = tf9->resume[RES_CTRL_REG1];

	tf9->resume[RES_CTRL_REG1] = buf;
	tf9->pdata->shift_adj = shift;

	return 0;
}

int KXT_9_update_odr(struct KXT_9_data *tf9, int poll_interval)
{
	int err = -1;
	int i;
	u8 config;

	/* Convert the poll interval into an output data rate configuration
	 *  that is as low as possible.  The ordering of these checks must be
	 *  maintained due to the cascading cut off values - poll intervals are
	 *  checked from shortest to longest.  At each check, if the next slower
	 *  ODR cannot support the current poll interval, we stop searching */
	for (i = 0; i < ARRAY_SIZE(KXT_9_odr_table); i++) {
		config = KXT_9_odr_table[i].mask;
		if (poll_interval < KXT_9_odr_table[i].cutoff)
			break;
	}

	if (atomic_read(&tf9->enabled)) {
		tf9->resume[RES_CTRL_REG1] &= (~PC1_ON);
		KXT_9_i2c_write(tf9, CTRL_REG1, &tf9->resume[RES_CTRL_REG1], 1);
		err = KXT_9_i2c_write(tf9, DATA_CTRL, &config, 1);
		tf9->resume[RES_CTRL_REG1] |= PC1_ON;
		KXT_9_i2c_write(tf9, CTRL_REG1, &tf9->resume[RES_CTRL_REG1], 1);
		if (err < 0)
			return err;
		/*
		 *  Latch on input_dev - indicates that KXT_9_input_init passed
		 *  and this workqueue is available
		 */
		if (tf9->input_dev) {
			cancel_delayed_work_sync(&tf9->input_work);
			schedule_delayed_work(&tf9->input_work,
				      msecs_to_jiffies(poll_interval));
		}
	}
	tf9->resume[RES_DATA_CTRL] = config;

	return 0;
}

static int KXT_9_get_acceleration_data(struct KXT_9_data *tf9, int *xyz)
{
	int err;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6];
	/* x,y,z hardware values */
	int hw_d[3];
	int i =0;
	long tmp_value = 0;

	if (!tf9->fLoadConfig)
	{
		printk("Read g sensor calibration file\n");
		err = access_calibration_file(tf9->offset, tf9->sensitivity);
		if (err < 0)
			printk("in %s, read g sensor calibration file FAIL\n", __func__);

		printk("offset: (%d, %d, %d), sensitivity: (%d, %d, %d)\n",
			tf9->offset[0], tf9->offset[1], tf9->offset[2],
			tf9->sensitivity[0], tf9->sensitivity[1], tf9->sensitivity[2]);

		tf9->fLoadConfig = true;
	}

	err = KXT_9_i2c_read(tf9, XOUT_L, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (int) (((acc_data[1]) << 8) | acc_data[0]);
	hw_d[1] = (int) (((acc_data[3]) << 8) | acc_data[2]);
	hw_d[2] = (int) (((acc_data[5]) << 8) | acc_data[4]);

	hw_d[0] = (hw_d[0] & 0x8000) ? (hw_d[0] | 0xFFFF0000) : (hw_d[0]);
	hw_d[1] = (hw_d[1] & 0x8000) ? (hw_d[1] | 0xFFFF0000) : (hw_d[1]);
	hw_d[2] = (hw_d[2] & 0x8000) ? (hw_d[2] | 0xFFFF0000) : (hw_d[2]);

	hw_d[0] >>= tf9->pdata->shift_adj;
	hw_d[1] >>= tf9->pdata->shift_adj;
	hw_d[2] >>= tf9->pdata->shift_adj;

	xyz[0] = ((tf9->pdata->negate_x) ? (-hw_d[tf9->pdata->axis_map_x])
		  : (hw_d[tf9->pdata->axis_map_x]));
	xyz[1] = ((tf9->pdata->negate_y) ? (-hw_d[tf9->pdata->axis_map_y])
		  : (hw_d[tf9->pdata->axis_map_y]));
	xyz[2] = ((tf9->pdata->negate_z) ? (-hw_d[tf9->pdata->axis_map_z])
		  : (hw_d[tf9->pdata->axis_map_z]));

	//dev_info(&tf9->client->dev, "before add bias, x:%d y:%d z:%d\n", xyz[0], xyz[1], xyz[2]);

	for (i = 0; i < 3; i++)
	{
		tf9->raw_xyz[i] = xyz[i];

		tmp_value = xyz[i] - tf9->offset[i];

		if(tmp_value < 0) {
			tmp_value = -tmp_value;
			tmp_value = (long)(tmp_value << 10);

			if(tf9->sensitivity[i] != 0)
				xyz[i] = (int)(tmp_value / tf9->sensitivity[i]);

			xyz[i] = -xyz[i];
		} else {
			tmp_value = (long)(tmp_value << 10);

			if(tf9->sensitivity[i] != 0)
				xyz[i] = (int)(tmp_value / tf9->sensitivity[i]);
		}
		/* if sensitivity = 0, xyz[] do not feed bias */
	}

	//dev_info(&tf9->client->dev, "after add bias, x:%d y:%d z:%d\n", xyz[0], xyz[1], xyz[2]);

	/*** DEBUG OUTPUT - REMOVE ***/
	//dev_info(&tf9->client->dev, "x:%d y:%d z:%d\n", xyz[0], xyz[1], xyz[2]);
	/*** <end> DEBUG OUTPUT - REMOVE ***/

	return err;
}

static void KXT_9_report_values(struct KXT_9_data *tf9, int *xyz)
{
	input_report_abs(tf9->input_dev, ABS_X, xyz[0]);
	input_report_abs(tf9->input_dev, ABS_Y, xyz[1]);
	input_report_abs(tf9->input_dev, ABS_Z, xyz[2]);
	input_sync(tf9->input_dev);
}

static int KXT_9_enable(struct KXT_9_data *tf9)
{
	int err;
	int int_status = 0;
	u8 buf;

	if (!atomic_cmpxchg(&tf9->enabled, 0, 1)) {
		err = KXT_9_device_power_on(tf9);
		err = KXT_9_i2c_read(tf9, INT_REL, &buf, 1);
		if (err < 0) {
			dev_err(&tf9->client->dev,
					"error clearing interrupt: %d\n", err);
			atomic_set(&tf9->enabled, 0);
			return err;
		}
		if ((tf9->resume[RES_CTRL_REG1] & TPE) > 0) {
			err = KXT_9_i2c_read(tf9, TILT_POS_CUR, &buf, 1);
			if (err < 0) {
				dev_err(&tf9->client->dev,
					"read err current tilt\n");
			int_status |= KXT_9_resolve_dir(tf9, buf);
			input_report_abs(tf9->input_dev, ABS_MISC, int_status);
			input_sync(tf9->input_dev);
			}
		}
		schedule_delayed_work(&tf9->input_work,
			msecs_to_jiffies(tf9->res_interval));
	}

	return 0;
}

static int KXT_9_disable(struct KXT_9_data *tf9)
{
	if (atomic_cmpxchg(&tf9->enabled, 1, 0)) {
		cancel_delayed_work_sync(&tf9->input_work);
		KXT_9_device_power_off(tf9);
	}

	return 0;
}

static void KXT_9_input_work_func(struct work_struct *work)
{
	struct KXT_9_data *tf9 = container_of((struct delayed_work *)work,
						struct KXT_9_data, input_work);
	int xyz[3] = { 0 };

	mutex_lock(&tf9->lock);

	if (KXT_9_get_acceleration_data(tf9, xyz) == 0)
		KXT_9_report_values(tf9, xyz);

	schedule_delayed_work(&tf9->input_work,
			msecs_to_jiffies(tf9->res_interval));
	mutex_unlock(&tf9->lock);
}

int KXT_9_input_open(struct input_dev *input)
{
	struct KXT_9_data *tf9 = input_get_drvdata(input);

	return KXT_9_enable(tf9);
}

void KXT_9_input_close(struct input_dev *dev)
{
	struct KXT_9_data *tf9 = input_get_drvdata(dev);

	KXT_9_disable(tf9);
}

static int KXT_9_input_init(struct KXT_9_data *tf9)
{
	int err;

	INIT_DELAYED_WORK(&tf9->input_work, KXT_9_input_work_func);
	tf9->input_dev = input_allocate_device();
	if (!tf9->input_dev) {
		err = -ENOMEM;
		dev_err(&tf9->client->dev, "input device allocate failed\n");
		goto err0;
	}
	tf9->input_dev->open = KXT_9_input_open;
	tf9->input_dev->close = KXT_9_input_close;

	input_set_drvdata(tf9->input_dev, tf9);

	set_bit(EV_ABS, tf9->input_dev->evbit);
	set_bit(ABS_MISC, tf9->input_dev->absbit);

	input_set_abs_params(tf9->input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(tf9->input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	tf9->input_dev->name = "KXT_9_accel";

	err = input_register_device(tf9->input_dev);
	if (err) {
		dev_err(&tf9->client->dev,
			"unable to register input polled device %s: %d\n",
			tf9->input_dev->name, err);
		goto err1;
	}

	return 0;
err1:
	input_free_device(tf9->input_dev);
err0:
	return err;
}

static void KXT_9_input_cleanup(struct KXT_9_data *tf9)
{
	input_unregister_device(tf9->input_dev);
}

/* sysfs */
static ssize_t KXT_9_delay_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", tf9->res_interval);
}

static ssize_t KXT_9_delay_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);

	tf9->res_interval = max(val, tf9->pdata->min_interval);
	KXT_9_update_odr(tf9, tf9->res_interval);

	return count;
}

static ssize_t KXT_9_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	return sprintf(buf, "%d\n", atomic_read(&tf9->enabled));
}

static ssize_t KXT_9_enable_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		KXT_9_enable(tf9);
	else
		KXT_9_disable(tf9);
	return count;
}

static ssize_t KXT_9_tilt_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	u8 tilt;

	if (tf9->resume[RES_CTRL_REG1] & TPE) {
		KXT_9_i2c_read(tf9, TILT_POS_CUR, &tilt, 1);
		return sprintf(buf, "%d\n", KXT_9_resolve_dir(tf9, tilt));
	} else {
		return sprintf(buf, "%d\n", 0);
	}
}

static ssize_t KXT_9_tilt_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		tf9->resume[RES_CTRL_REG1] |= TPE;
	else
		tf9->resume[RES_CTRL_REG1] &= (~TPE);
	KXT_9_i2c_write(tf9, CTRL_REG1, &tf9->resume[RES_CTRL_REG1], 1);
	return count;
}

static ssize_t KXT_9_wake_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	u8 val = tf9->resume[RES_CTRL_REG1] & WUFE;
	if (val)
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);
}

static ssize_t KXT_9_wake_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		tf9->resume[RES_CTRL_REG1] |= WUFE;
	else
		tf9->resume[RES_CTRL_REG1] &= (~WUFE);
	KXT_9_i2c_write(tf9, CTRL_REG1, &tf9->resume[RES_CTRL_REG1], 1);
	return count;
}

static ssize_t KXT_9_tap_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	u8 val = tf9->resume[RES_CTRL_REG1] & TDTE;
	if (val)
		return sprintf(buf, "%d\n", 1);
	else
		return sprintf(buf, "%d\n", 0);
}

static ssize_t KXT_9_tap_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	if (val)
		tf9->resume[RES_CTRL_REG1] |= TDTE;
	else
		tf9->resume[RES_CTRL_REG1] &= (~TDTE);
	KXT_9_i2c_write(tf9, CTRL_REG1, &tf9->resume[RES_CTRL_REG1], 1);
	return count;
}

static ssize_t KXT_9_selftest_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	u8 ctrl = 0x00;
	if (val)
		ctrl = 0xCA;
	KXT_9_i2c_write(tf9, 0x3A, &ctrl, 1);
	return count;
}

static ssize_t KXT_9_dump_reg(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);
	int ii, err = -1;
	ssize_t bytes_printed = 0;
	unsigned char data;

	for (ii = 0; ii < REG_NUM; ii++) {
		err = KXT_9_i2c_read(tf9, ii, &data, 1);
		bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n", ii, data);
	}

	return bytes_printed;
}

static DEVICE_ATTR(delay, S_IRUGO|S_IWUSR, KXT_9_delay_show, KXT_9_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, KXT_9_enable_show,
						KXT_9_enable_store);
static DEVICE_ATTR(tilt, S_IRUGO|S_IWUSR, KXT_9_tilt_show, KXT_9_tilt_store);
static DEVICE_ATTR(wake, S_IRUGO|S_IWUSR, KXT_9_wake_show, KXT_9_wake_store);
static DEVICE_ATTR(tap, S_IRUGO|S_IWUSR, KXT_9_tap_show, KXT_9_tap_store);
static DEVICE_ATTR(selftest, S_IWUSR, NULL, KXT_9_selftest_store);
static DEVICE_ATTR(dump_reg, S_IRUGO, KXT_9_dump_reg, NULL);

#ifdef CONFIG_DEBUG_ASUS
static struct attribute *KXT_9_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_tilt.attr,
	&dev_attr_wake.attr,
	&dev_attr_tap.attr,
	&dev_attr_selftest.attr,
	&dev_attr_dump_reg.attr,
	NULL
};

#else
static struct attribute *KXT_9_attributes[] = {
	&dev_attr_delay.attr,
	&dev_attr_enable.attr,
	&dev_attr_tilt.attr,
	&dev_attr_wake.attr,
	&dev_attr_tap.attr,
	NULL
};

#endif

static struct attribute_group KXT_9_attribute_group = {
	.attrs = KXT_9_attributes
};
/* /sysfs */

static int __devinit KXT_9_probe(struct i2c_client *client,
						const struct i2c_device_id *id)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	int ii, err = -1;
	struct KXT_9_data *tf9 = kzalloc(sizeof(*tf9), GFP_KERNEL);

	if (tf9 == NULL) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		err = -ENOMEM;
		goto err0;
	}
	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		err = -ENODEV;
		goto err0;
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto err0;
	}
	mutex_init(&tf9->lock);
	mutex_lock(&tf9->lock);
	tf9->client = client;

	/* init variables for bias setting */
	tf9->fLoadConfig = false;
	for(ii = 0; ii < 3; ii++) {
		tf9->raw_xyz[ii] = 0;
		tf9->offset[ii] = 0;
		tf9->sensitivity[ii] = 0;
	}

	i2c_set_clientdata(client, tf9);

	INIT_WORK(&tf9->irq_work, KXT_9_irq_work_func);
	tf9->pdata = kmalloc(sizeof(*tf9->pdata), GFP_KERNEL);
	if (tf9->pdata == NULL)
		goto err1;

	err = sysfs_create_group(&client->dev.kobj, &KXT_9_attribute_group);
	if (err)
		goto err1;

	memcpy(tf9->pdata, client->dev.platform_data, sizeof(*tf9->pdata));
	if (tf9->pdata->init) {
		err = tf9->pdata->init();
		if (err < 0)
			goto err2;
	}

	//tf9->irq = gpio_to_irq(tf9->pdata->gpio);

	memset(tf9->resume, 0, ARRAY_SIZE(tf9->resume));
	tf9->resume[RES_DATA_CTRL] = tf9->pdata->data_odr_init;
	tf9->resume[RES_CTRL_REG1] = tf9->pdata->ctrl_reg1_init;
	tf9->resume[RES_INT_CTRL1] = tf9->pdata->int_ctrl_init;
	tf9->resume[RES_TILT_TIMER] = tf9->pdata->tilt_timer_init;
	tf9->resume[RES_CTRL_REG3] = tf9->pdata->engine_odr_init;
	tf9->resume[RES_WUF_TIMER] = tf9->pdata->wuf_timer_init;
	tf9->resume[RES_WUF_THRESH] = tf9->pdata->wuf_thresh_init;
	tf9->resume[RES_TDT_TIMER] = tf9->pdata->tdt_timer_init;
	tf9->resume[RES_TDT_H_THRESH] = tf9->pdata->tdt_h_thresh_init;
	tf9->resume[RES_TDT_L_THRESH] = tf9->pdata->tdt_l_thresh_init;
	tf9->resume[RES_TAP_TIMER] = tf9->pdata->tdt_tap_timer_init;
	tf9->resume[RES_TOTAL_TIMER] = tf9->pdata->tdt_total_timer_init;
	tf9->resume[RES_LAT_TIMER] = tf9->pdata->tdt_latency_timer_init;
	tf9->resume[RES_WIN_TIMER]    = tf9->pdata->tdt_window_timer_init;
	tf9->res_interval = tf9->pdata->poll_interval;

	err = KXT_9_device_power_on(tf9);
	if (err < 0)
		goto err3;
	atomic_set(&tf9->enabled, 1);

	err = KXT_9_verify(tf9);
	if (err < 0) {
		dev_err(&client->dev, "unresolved i2c client\n");
		goto err4;
	}

	err = KXT_9_update_g_range(tf9, tf9->pdata->g_range);
	if (err < 0)
		goto err4;

	err = KXT_9_update_odr(tf9, tf9->res_interval);
	if (err < 0)
		goto err4;

	err = KXT_9_input_init(tf9);
	if (err < 0)
		goto err4;

	KXT_9_device_power_off(tf9);
	atomic_set(&tf9->enabled, 0);
	/*err = request_irq(tf9->irq, KXT_9_isr,
			IRQF_TRIGGER_RISING | IRQF_DISABLED, "KXT_9-irq", tf9);
	if (err < 0) {
		pr_err("%s: request irq failed: %d\n", __func__, err);
		goto err5;
	}
	disable_irq_nosync(tf9->irq);*/

	mutex_unlock(&tf9->lock);
	printk(KERN_INFO "%s- #####\n", __func__);

	return 0;

err5:
	KXT_9_input_cleanup(tf9);
err4:
	KXT_9_device_power_off(tf9);
err3:
	if (tf9->pdata->exit)
		tf9->pdata->exit();
err2:
	kfree(tf9->pdata);
	sysfs_remove_group(&client->dev.kobj, &KXT_9_attribute_group);
err1:
	mutex_unlock(&tf9->lock);
	kfree(tf9);
err0:
	return err;
}

static int __devexit KXT_9_remove(struct i2c_client *client)
{
	struct KXT_9_data *tf9 = i2c_get_clientdata(client);

	free_irq(tf9->irq, tf9);
	gpio_free(tf9->pdata->gpio);
	KXT_9_input_cleanup(tf9);
	KXT_9_device_power_off(tf9);
	if (tf9->pdata->exit)
		tf9->pdata->exit();
	kfree(tf9->pdata);
	sysfs_remove_group(&client->dev.kobj, &KXT_9_attribute_group);
	kfree(tf9);

	return 0;
}

#ifdef CONFIG_PM
static int KXT_9_resume(struct device *dev)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	struct i2c_client *client = i2c_verify_client(dev);
	struct KXT_9_data *tf9;
	int res;

	tf9 = i2c_get_clientdata(client);
	res = KXT_9_enable(tf9);

	printk(KERN_INFO "%s- #####\n", __func__);
	return res;
}

static int KXT_9_suspend(struct device *dev, pm_message_t mesg)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	struct i2c_client *client = i2c_verify_client(dev);
	struct KXT_9_data *tf9;
	int res;

	tf9 = i2c_get_clientdata(client);
	res = KXT_9_disable(tf9);

	printk(KERN_INFO "%s- #####\n", __func__);
	return res;
}
#endif

static const struct i2c_device_id KXT_9_id[] = {
	{NAME, 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, KXT_9_id);

static const struct dev_pm_ops kxt_9_pm_ops={
	.suspend = KXT_9_suspend,
	.resume = KXT_9_resume,
};

static struct i2c_driver KXT_9_driver = {
	.driver = {
		   .name = NAME,
		   .pm = &kxt_9_pm_ops,
		   },
	.probe = KXT_9_probe,
	.remove = __devexit_p(KXT_9_remove),
	.id_table = KXT_9_id,
};

static int __init KXT_9_init(void)
{
	int res;

	printk(KERN_INFO "%s+ #####\n", __func__);
	res = i2c_add_driver(&KXT_9_driver);
	printk(KERN_INFO "%s- #####\n", __func__);

	return res;
}

static void __exit KXT_9_exit(void)
{
	i2c_del_driver(&KXT_9_driver);
}

module_init(KXT_9_init);
module_exit(KXT_9_exit);

MODULE_DESCRIPTION("KXT_9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
