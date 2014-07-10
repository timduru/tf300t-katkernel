/*
 * kernel/drivers/media/video/tegra
 *
 * iCatch SPCA7002A ISP driver
 *
 * Copyright (C) 2012 ASUS Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <media/yuv_sensor.h>

#undef _CAM_SENSOR_DETECT_
#ifdef _CAM_SENSOR_DETECT_
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <mach/clk.h>
#endif
#include <linux/switch.h>
#include <mach/board-cardhu-misc.h>

#define I7002A_SDEV_NAME "camera"

#define SPI_CMD_BYTE_READ 	0x03
#define SPI_CMD_RD_ID 		0x9F
#define SPI_CMD_WRT_EN		0x06
#define SPI_CMD_BYTE_PROG 	0x02
#define SPI_CMD_RD_STS		0x05
#define SPI_CMD_BYTE_PROG_AAI	0xAD
#define SPI_CMD_WRT_STS_EN	0x50
#define SPI_CMD_WRT_STS 	0x01
#define SPI_CMD_WRT_DIS 	0x04
#define SPI_CMD_ERASE_ALL	0xC7

#define	SPI_CMD_SECTOR_ERASE		0x20
#define	SPI_CMD_32KB_BLOCK_ERASE	0x52
#define	SPI_CMD_64KB_BLOCK_ERASE	0xD8

#define SENSOR_ID_MI1040	0x2481
#define SENSOR_ID_OV2720	0x2720
#define SENSOR_ID_IMX175	0x175
#define SENSOR_ID_OV5650	0x5650
#define SENSOR_ID_OV5651	0x5651

//extern unsigned int factory_mode;
static unsigned int factory_mode=0;

struct switch_dev i7002a_sdev;
static unsigned int version_num_in_isp = 0xffffff;
static unsigned int fw_front_type_in_isp = 0x0;
static unsigned int front_chip_id = 0xABCD;

static u16 coloreffect;
/* mi1040 format:
 * 0: YUV
 * 1: RGB
 * 2: Bayer
 */
static unsigned int mi1040_output_format = 0xFF;

static int dbg_i7002a_page_index = 2047;
/* The unit of force firmware size is KB */
static int dbg_i7002a_force_fw_size = 0;

/* iCatch Camera Firmware Header
 * It locates on the end of the bin file.
 * Total: 32 bytes.
 * byte[0] ~ byte[7]: 0xFF's
 * byte[8] ~ byte[11]: Compensation for Overall Checksum
 * byte[12] ~ byte[15]: Overall Checksum

 * byte[16] ~ byte[20]: 0xFF's
 * byte[21]: Front Format
 * byte[22]: Rear Lane#
 * byte[23]: Front Lane#
 * byte[24] ~ byte[25]: Rear Sensor ID
 * byte[26] ~ byte[27]: Front sensor ID
 * byte[28] ~ byte[31]: FW Version
 */
#define BIN_FILE_HEADER_SIZE 32

#define ICATCH7002A_DELAY_TEST
#ifdef ICATCH7002A_DELAY_TEST
static u32 iCatch7002a_init_delay= 5;
static u32 iCatch7002a_init_long_delay= 10;
static u32 iCatch7002a_preview_delay=100;
static u32 touch_focus_enable=0;
#endif
#define _ENABLE_WRITE_TABLE_2_GROUP_LATCH_
#define _AVOID_GROUP_LATCH_AFTER_SET_MODE_
//#undef _AVOID_GROUP_LATCH_AFTER_SET_MODE_
#ifdef _AVOID_GROUP_LATCH_AFTER_SET_MODE_
#define _AVOID_GROUP_LATCH_TIME_MS_ 200
#endif
#define SENSOR_WIDTH_REG 0x2703
#define SENSOR_640_WIDTH_VAL 0x0280
#define ICATCH7002A_SENSOR_NAME "i7002a"
#define CAM_TWO_MODE
#ifdef CAM_TWO_MODE
static unsigned int g_div = 100;
static int g_initialized_1280_960=0;
static int g_initialized_1080p=0;
#endif
static bool sensor_opened = false;
static bool first_open = true;
static bool af_start = false;
static bool capture_mode = false;
/* Used for calculating the iCatch fw update progress */
static int page_count = -1;
static int total_page_count = -1;
static u32 is_calibration=0;
static u32 calibrating = 0;

enum iCatch_fw_update_status{
	ICATCH_FW_NO_CMD,
	ICATCH_FW_IS_BURNING,
	ICATCH_FW_UPDATE_SUCCESS,
	ICATCH_FW_UPDATE_FAILED,
};
static enum iCatch_fw_update_status fw_update_status = ICATCH_FW_NO_CMD;

enum iCatch_flash_type{
	ICATCH_FLASH_TYPE_ST,
	ICATCH_FLASH_TYPE_SST,
};
static enum iCatch_flash_type flash_type = ICATCH_FLASH_TYPE_ST;

struct sensor_reg {
	u16 addr;
	u16 val;
};

struct sensor_reg_2 {
	u16 cmd;
	u16 addr;
	u16 val;
	u16 val2;
};

struct sensor_info {
	int mode;
	struct i2c_client *i2c_client;
	struct yuv_sensor_platform_data *pdata;
#ifdef _CAM_SENSOR_DETECT_
	struct device dev;
#endif
};

static struct sensor_info *info;
static int touch_mode = TOUCH_STATUS_OFF;
static bool caf_mode = false;
static int focus_control = 0;

static struct sensor_reg Autofocus_Trigger[] = {
{0x7140, 0x00}, //ROI Size_H
{0x7141, 0x50}, //ROI Size_H
{0x7142, 0x02}, //ROI X_H
{0x7143, 0x58}, //ROI X_L
{0x7144, 0x02}, //ROI Y_H
{0x7145, 0x58}, //ROI Y_L
{SENSOR_TABLE_END, 0x0000}
};

enum {
	SENSOR_MODE_3264x2448,
	SENSOR_MODE_2592x1944,
	SENSOR_MODE_1920x1080,
	SENSOR_MODE_1280x960,
	SENSOR_MODE_1280x720,
	SENSOR_MODE_640x480,
};

int tegra_camera_mclk_on_off(int on);

bool IsTF300(void){
	if( tegra3_get_project_id() == TEGRA3_PROJECT_TF300T  ||
		tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG ||
		tegra3_get_project_id() == TEGRA3_PROJECT_TF300TL)
		return 1;
	else
		return 0;
}

void i7002a_isp_on(int power_on)
{
	printk("%s(%d)++\n", __FUNCTION__, power_on);
	if (power_on == 0) {
		if (sensor_opened == true) {
			if (info->pdata && info->pdata->power_off) {
				tegra_camera_mclk_on_off(0);
				info->pdata->power_off();
				sensor_opened = false;
			} else {
				printk("%s: iCatch7002a info isn't enough for power_off.\n", __FUNCTION__);
			}
		} else
			printk("%s: No action. sensor is already OFF\n", __FUNCTION__);
	} else {
		/* Power on ISP */
		if (sensor_opened == false) {
			if (info->pdata && info->pdata->power_on) {
				info->pdata->power_on();
				tegra_camera_mclk_on_off(1);
				msleep(100);
				sensor_opened = true;
			} else {
				printk("%s: iCatch7002a info isn't enough for power_on.\n", __FUNCTION__);
			}
		} else {
			printk("%s: sensor is already ON?\n", __FUNCTION__);
			if (info->pdata && info->pdata->power_off) {
				printk("%s: Turn off first.\n", __FUNCTION__);
				info->pdata->power_off();
				sensor_opened = false;
			} else {
				printk("%s: iCatch7002a info isn't enough for power_off.\n", __FUNCTION__);
			}
			msleep(100);
			printk("%s: Re-power_on.\n", __FUNCTION__);
			i7002a_isp_on(1);
		}
	}
	printk("%s(%d)--\n", __FUNCTION__, power_on);
}

static int sensor_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[4];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;

	msg[1].len = 1;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 1);
	*val=*val&0xff;

	return 0;
}

static int sensor_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
	int err;
	struct i2c_msg msg;
	unsigned char data[4];
	int retry = 0;
	if (!client->adapter)
		return -ENODEV;

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);
	data[2] = (u8) (val & 0xff);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 3;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		pr_err("yuv_sensor : i2c transfer failed, retrying 0x%x 0x%x\n",
			addr, val);
		pr_err("%s(%d) : i2c transfer failed, count 0x%x, err= 0x%x\n",
			__FUNCTION__, __LINE__, msg.addr, err);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	if(err == 0) {
		printk("%s(%d): i2c_transfer error, but return 0!?\n", __FUNCTION__, __LINE__);
		err = 0xAAAA;
	}

	return err;
}

static int I2C_SPIFlashPortWrite(u16 val)
{
	return sensor_write_reg(info->i2c_client, 0x40e3, val);
}

static int I2CDataWrite(u16 addr, u16 val)
{
	return sensor_write_reg(info->i2c_client, addr, val);
}

static int seqI2CDataWrite(struct i2c_client *client, u16 addr, u8 *buf, int bytenum)
{
	int i, err;
	struct i2c_msg msg;
	unsigned char *data;
	int retry = 0;

	if (!client->adapter)
		return -ENODEV;

	if (bytenum <= 0) {
		printk("%s(%d): Invalid bytenum\n", __FUNCTION__, __LINE__);
		return -ENODEV;
	}

	data = kmalloc(2 + bytenum, GFP_KERNEL);

	data[0] = (u8) (addr >> 8);
	data[1] = (u8) (addr & 0xff);

	for (i = 0; i < bytenum; i++) {
		data[i + 2] = (u8) ((*(buf + i)) & 0xff);
	}

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = 2 + bytenum;
	msg.buf = data;

	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1) {
			kfree(data);
			return 0;
		}
		retry++;
		pr_err("%s(%d): i2c transfer failed, slave 0x%x, reg 0x%x, data:\n",
		       __FUNCTION__, __LINE__, msg.addr, addr);
		for(i = 0; i < bytenum; i++)
			pr_err(" 0x%x", data[2+i]);
		pr_err("\n");
	} while (retry <= SENSOR_MAX_RETRIES);

	kfree(data);
	return err;
}

static int sensor_sequential_write_reg(struct i2c_client *client, unsigned char *data, u16 datasize)
{
	int err;
	struct i2c_msg msg;
	int retry = 0;

	return 0;
	if (datasize==0)
		return 0;
	if (!client->adapter)
		return -ENODEV;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = datasize;
	msg.buf = data;
	do {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			return 0;
		retry++;
		//pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n",
		// addr, val);
		pr_err("yuv_sensor : i2c transfer failed, count %x \n",
			msg.addr);
//		msleep(3);
	} while (retry <= SENSOR_MAX_RETRIES);

	return err;
}

static int build_sequential_buffer(unsigned char *pBuf, u16 width, u16 value) {
	u32 count = 0;

	switch (width)
	{
		case 0:
		// possibly no address, some focusers use this
		break;

		// cascading switch
		case 32:
			pBuf[count++] = (u8)((value>>24) & 0xFF);
		case 24:
			pBuf[count++] = (u8)((value>>16) & 0xFF);
		case 16:
			pBuf[count++] = (u8)((value>>8) & 0xFF);
		case 8:
			pBuf[count++] = (u8)(value & 0xFF);
		break;

		default:
		printk("Unsupported Bit Width %d\n", width);
		break;
	}
	return count;
}

static int sensor_write_table(struct i2c_client *client,
	const struct sensor_reg table[])
{
	int err;
	const struct sensor_reg *next;
	u16 val;
	unsigned char data[10];
	u16 datasize = 0;

	//for (next = table; next->addr != SENSOR_TABLE_END; next++) {
	next = table;
	while (next->addr != SENSOR_TABLE_END) {
		if (next->addr == SENSOR_WAIT_MS) {
			msleep(next->val);
			next +=1;
			continue;
		}
		if (next->addr == SEQ_WRITE_START) {
			next += 1;
			while (next->addr !=SEQ_WRITE_END) {
				if (datasize==0) {//
					datasize += build_sequential_buffer(&data[datasize], 16, next->addr);
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				}
				else
					datasize += build_sequential_buffer(&data[datasize], 8, next->val);
				if (datasize==10) {
					sensor_sequential_write_reg(client, data, datasize);
					datasize = 0;
				}
				next += 1;
			}
			sensor_sequential_write_reg(client, data, datasize); //flush out the remaining buffer.
			datasize = 0;
		}
		else {
			val = next->val;

			err = sensor_write_reg(client, next->addr, val);
			if (err) {
				printk("%s(%d): isensor_write_reg ret= 0x%x\n", __FUNCTION__, __LINE__, err);
				return err;
			}
		}
		next += 1;
	}
	return 0;
}

struct sensor_reg query_mi1040_id_msb_seq[] = {
	/*Start - Power on sensor & enable clock*/
	{0x0084, 0x14},		/* To sensor clock divider */
	{0x0034, 0xFF},		/* Turn on all clock */
	{0x9030, 0x3f},
	{0x9031, 0x04},
	{0x9034, 0xf2},
	{0x9035, 0x04},
	{0x9033, 0x04},
	{0x9032, 0x3c},
	{SENSOR_WAIT_MS, 10},	/* 10ms */
	{0x9033, 0x00},
	{SENSOR_WAIT_MS, 10},	/* 10ms */
	{0x9033, 0x04},
	{0x9032, 0x3e},
	{SENSOR_WAIT_MS, 10},	/* 10ms */
	{0x9032, 0x3c},
	/*End - Power on sensor & enable clock */

	/*Start - I2C Read ID*/
	{0x9138, 0x30}, /* Sub address enable */
	{0x9140, 0x90}, /* Slave address      */
	{0x9100, 0x03}, /* Read mode          */
	{0x9110, 0x00}, /* Register addr MSB  */
	{0x9112, 0x00}, /* Register addr LSB  */
	{0x9104, 0x01}, /* Trigger I2C read   */
	{SENSOR_WAIT_MS, 1},	/* 1ms */
	{SENSOR_TABLE_END, 0x0000}
};

struct sensor_reg query_mi1040_id_lsb_seq[] = {
	{0x9110, 0x00}, /* Register addr MSB  */
	{0x9112, 0x01}, /* Register addr LSB  */
	{0x9104, 0x01}, /* Trigger I2C read   */
	{SENSOR_WAIT_MS, 1},	/* 1ms */
	{SENSOR_TABLE_END, 0x0000}
};

struct sensor_reg query_mi1040_output_format_seq[] = {
	/*Start - I2C Read YUV/RGB mode*/
	{0x9138, 0x30}, /* Sub address enable */
	{0x9140, 0x90}, /* Slave address      */
	{0x9100, 0x03}, /* Read mode          */
	{0x9110, 0xC8}, /* Register addr MSB  */
	{0x9112, 0x6C}, /* Register addr LSB  */
	{0x9104, 0x01}, /* Trigger I2C read   */
	{SENSOR_WAIT_MS, 1},	/* 1ms */
	{SENSOR_TABLE_END, 0x0000}
};
/*
facing: 0 for back; 1 for front
*/
static unsigned int i7002a_get_sensor_id(int facing)
{
	u16 tmp;
	int chip_id = 0;

	if (facing == 0) {
		/* back camera: SONY IMX175, chip_id=0x175; */
		sensor_write_reg(info->i2c_client, 0x0084, 0x14); /* To sensor clock divider */
		sensor_write_reg(info->i2c_client, 0x0034, 0xFF); /* Turn on all clock */
		sensor_write_reg(info->i2c_client, 0x9030, 0x3f);
		sensor_write_reg(info->i2c_client, 0x9031, 0x04);
		sensor_write_reg(info->i2c_client, 0x9034, 0xf2);
		sensor_write_reg(info->i2c_client, 0x9035, 0x04);
		sensor_write_reg(info->i2c_client, 0x9032, 0x00);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9032, 0x20);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9032, 0x30);
		msleep(10);
		/*End - Power on sensor & enable clock */
		sensor_write_reg(info->i2c_client, 0x9008, 0x00); /* Need to check with vincent */
		sensor_write_reg(info->i2c_client, 0x9009, 0x00);
		sensor_write_reg(info->i2c_client, 0x900A, 0x00);
		sensor_write_reg(info->i2c_client, 0x900B, 0x00);

		/*Start - I2C Read*/
		sensor_write_reg(info->i2c_client, 0x9238, 0x30); /* Sub address enable */
		sensor_write_reg(info->i2c_client, 0x9240, 0x20); /* Slave address      */
		sensor_write_reg(info->i2c_client, 0x9200, 0x03); /* Read mode          */
		sensor_write_reg(info->i2c_client, 0x9210, 0x00); /* Register addr MSB  */
		sensor_write_reg(info->i2c_client, 0x9212, 0x00); /* Register addr LSB  */
		sensor_write_reg(info->i2c_client, 0x9204, 0x01); /* Trigger I2C read   */

		msleep(10);
		sensor_read_reg(info->i2c_client, 0x9211, &tmp);
		// printk("0x%x\n", tmp);
		chip_id = (tmp << 8) & 0xFF00;

		sensor_write_reg(info->i2c_client, 0x9210, 0x00); /* Register addr MSB  */
		sensor_write_reg(info->i2c_client, 0x9212, 0x01); /* Register addr LSB  */
		sensor_write_reg(info->i2c_client, 0x9204, 0x01); /* Trigger I2C read   */

		msleep(10);
		sensor_read_reg(info->i2c_client, 0x9211, &tmp);
		// printk("0x%x\n", tmp);
		chip_id = chip_id | (tmp & 0xFF);
	} else if (facing == 1){
		/* Start - Power on sensor & enable clock - Front I2C (OV2720);
		 * ov2720: chip_id= 0x2720;
		 */
		sensor_write_reg(info->i2c_client, 0x0084, 0x14); /* To sensor clock divider */
		sensor_write_reg(info->i2c_client, 0x0034, 0xFF); /* Turn on all clock */
		sensor_write_reg(info->i2c_client, 0x9030, 0x3f);
		sensor_write_reg(info->i2c_client, 0x9031, 0x04);
		sensor_write_reg(info->i2c_client, 0x9034, 0xf3);
		sensor_write_reg(info->i2c_client, 0x9035, 0x04);

		sensor_write_reg(info->i2c_client, 0x9032, 0x02);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9032, 0x00);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9033, 0x00);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9033, 0x04);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9034, 0xf2);
		/*End - Power on sensor & enable clock */

		sensor_write_reg(info->i2c_client, 0x9008, 0x00); /* Need to check with vincent */
		sensor_write_reg(info->i2c_client, 0x9009, 0x00);
		sensor_write_reg(info->i2c_client, 0x900A, 0x00);
		sensor_write_reg(info->i2c_client, 0x900B, 0x00);

		/*Start - I2C Read*/
		sensor_write_reg(info->i2c_client, 0x9138, 0x30); /* Sub address enable */
		sensor_write_reg(info->i2c_client, 0x9140, 0x6C); /* Slave address      */
		sensor_write_reg(info->i2c_client, 0x9100, 0x03); /* Read mode          */
		sensor_write_reg(info->i2c_client, 0x9110, 0x30); /* Register addr MSB  */
		sensor_write_reg(info->i2c_client, 0x9112, 0x0a); /* Register addr LSB  */
		sensor_write_reg(info->i2c_client, 0x9104, 0x01); /* Trigger I2C read   */

		msleep(10);
		sensor_read_reg(info->i2c_client, 0x9111, &tmp);

		//printk("0x%x\n", tmp);
		chip_id = (tmp << 8) & 0xFF00;

		sensor_write_reg(info->i2c_client, 0x9110, 0x30); /* Register addr MSB  */
		sensor_write_reg(info->i2c_client, 0x9112, 0x0b); /* Register addr LSB  */
		sensor_write_reg(info->i2c_client, 0x9104, 0x01); /* Trigger I2C read   */

		msleep(10);
		sensor_read_reg(info->i2c_client, 0x9111, &tmp);
		//printk("0x%x\n", tmp);
		chip_id = chip_id | (tmp & 0xFF);

		if (chip_id != SENSOR_ID_OV2720) {
			/* Check if mi1040 is available. */
			sensor_write_table(info->i2c_client, query_mi1040_id_msb_seq);
			sensor_read_reg(info->i2c_client, 0x9111, &tmp);

			chip_id = (tmp << 8) & 0xFF00;

			sensor_write_table(info->i2c_client, query_mi1040_id_lsb_seq);
			sensor_read_reg(info->i2c_client, 0x9111, &tmp);
			chip_id = chip_id | (tmp & 0xFF);
		}
	} else {
		/* Unknown */
		chip_id = 0xabcdef;
	}

	return chip_id;
}

int I2C_SPIInit(void)
{
	int ret = 0;
	// I2CDataWrite(0x0026,0xc0);
	// I2CDataWrite(0x4051,0x01); /* spien */
	// I2CDataWrite(0x40e1,0x00); /* spi mode */
	// I2CDataWrite(0x40e0,0x11); /* spi freq */
	struct sensor_reg SPI_init_seq[] = {
		{0x0026, 0xc0},
		{0x4051, 0x01},
		{0x40e1, 0x00},
		{0x40e0, 0x11},
		{SENSOR_TABLE_END, 0x0000}
	};

	ret = sensor_write_table(info->i2c_client, SPI_init_seq);
	if(ret) {
		printk("%s: init fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

u32 I2C_SPIFlashPortRead(void)
{
	u16 ret;

	// ret = hsI2CDataRead(0x40e4);
	sensor_read_reg(info->i2c_client, 0x40e4, &ret);
	/* polling SPI state machine ready */
#if 0
	if (I2C_SPIFlashPortWait() != SUCCESS) {
		return 0;
	}
#endif
	//ret = hsI2CDataRead(0x40e5);
	sensor_read_reg(info->i2c_client, 0x40e5, &ret);

	return (u32)ret;
}

u32 I2C_SPIFlashRead(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 err = 0;
	u32 i, size=0;
	u32 pageSize = 0x100;

	addr = addr * pageSize;
	size = pages*pageSize;

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_BYTE_READ);	/* Write one byte command*/
	I2C_SPIFlashPortWrite((u8)(addr >> 16));	/* Send 3 bytes address*/
	I2C_SPIFlashPortWrite((u8)(addr >> 8));
	I2C_SPIFlashPortWrite((u8)(addr));

	for (i = 0; i < size ; i++) {
		*pbuf = I2C_SPIFlashPortRead();
		if((i%256)==0)
			printk("%s: page count: 0x%x\n", __FUNCTION__, (i/256));
		pbuf ++;
	}

	I2CDataWrite(0x40e7, 0x01);

	return err;
}

u32 I2C_SPIFlashReadId(void)
{
	u8 id[3];
	u32 ID;

	id[0] = 0;
	id[1] = 0;
	id[2] = 0;

	I2CDataWrite(0x40e7, 0x00);

	I2C_SPIFlashPortWrite(SPI_CMD_RD_ID);	/*read ID command*/

#if 0
	if (err != SUCCESS) {
		printf("Get serial flash ID failed\n");
		return 0;
	}
#endif

	id[0] = I2C_SPIFlashPortRead();	/* Manufacturer's ID */
	id[1] = I2C_SPIFlashPortRead();	/* Device ID          */
	id[2] = I2C_SPIFlashPortRead();	/* Manufacturer's ID */

	I2CDataWrite(0x40e7, 0x01);

	printk("ID %2x %2x %2x\n", id[0], id[1], id[2]);

	ID = ((u32)id[0] << 16) | ((u32)id[1] << 8) | \
		((u32)id[2] << 0);

	return ID;
}

static const u32 stSpiIdInfo[7][3] =
{
	/*Winbond*/
	{0x00EF3017,4096, 2048},
	{0x00EF3016,4096, 1024},
	{0x00EF3015,4096, 512},
	{0x00EF3014,4096, 256},
	{0x00EF5014,4096, 256},
	{0x00EF3013,4096, 128},
	{0x00EF5013,4096, 128},
	/*Fail*/
	{0x00000000,0,0},
};

static const u32 sstSpiIdInfo[6][3] =
{
	/*ESMT*/
	{0x008C4016,4096,512},
	/*SST*/
	{0x00BF254A,4096,1024},
	{0x00BF2541,4096,512},
	{0x00BF258E,4096,256},
	{0x00BF258D,4096,128},
	/*Fail*/
	{0x00000000,0,0},
};

u32
BB_SerialFlashTypeCheck(
	u32 id,
	u32 *spiSize
)
{
	u32 i=0;
	u32 fullID = 1;
	u32 shift = 0, tblId, type = 0;
	/*printf("id:0x%x spiSize:0x%x\n",id,spiSize);*/
	/* check whether SST type serial flash */
	while( 1 ){
		tblId = sstSpiIdInfo[i][0] >> shift;
		if( id == tblId ) {
			printk("SST type serial flash:%x %x %x\n",i,id,sstSpiIdInfo[i][0]);
			type = 2;
			*spiSize = sstSpiIdInfo[i][1]*sstSpiIdInfo[i][2];
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( sstSpiIdInfo[i][0] == 0x00000000 ) {
			#if 0
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			#endif
			type = 3;
			break;
		}
		i ++;
	}
	if( type == 2 )
		return type;

	i = 0;
	/* check whether ST type serial flash */
	while( 1 ){
		tblId = stSpiIdInfo[i][0] >> shift;
		if( id == tblId ) {
			printk("ST Type serial flash:%x %x %x\n",i,id,stSpiIdInfo[i][0]);
			type = 1;
			*spiSize = stSpiIdInfo[i][1]*stSpiIdInfo[i][2];
			/*printf("spiSize:0x%x\n",*spiSize);*/
			break;
		}
		if( id == 0x00FFFFFF || id == 0x00000000) {
			return 0;
		}
		if( stSpiIdInfo[i][0] == 0x00000000 ) {
			if( fullID ){
				fullID = 0;/* sarch partial ID */
				i = 0;
				shift = 16;
				id = id >> shift;
				continue;
			}
			type = 3;
			break;
		}
		i ++;
	}

	return type;
}

int I2C_SPIFlashWrEnable(void)
{
	int ret = 0;
	//hsI2CDataWrite(0x40e7,0x00);
	//I2C_SPIFlashPortWrite(SPI_CMD_WRT_EN);
	//hsI2CDataWrite(0x40e7,0x01);
	struct sensor_reg I2C_SPIFlashWrEnable_seq[] = {
		{0x40e7, 0x00},
		{0x40e3, SPI_CMD_WRT_EN},
		{0x40e7, 0x01},
		{SENSOR_TABLE_END, 0x0000}
	};

	ret = sensor_write_table(info->i2c_client, I2C_SPIFlashWrEnable_seq);

	if(ret) {
		printk("%s: fail. ret= 0x%x\n", __FUNCTION__, ret);
	}
	return ret;
}

u32 I2C_SPIStsRegRead(void)
{
	u32 ret;

	I2CDataWrite(0x40e7, 0x00);

	I2C_SPIFlashPortWrite(SPI_CMD_RD_STS);
	ret = I2C_SPIFlashPortRead();

	I2CDataWrite(0x40e7, 0x01);

	return ret;
}

void I2C_SPITimeOutWait(u32 poll, u32 *ptimeOut)
{
	/* MAX_TIME for SECTOR/BLOCK ERASE is 25ms */
	u32 sts;
	u32 time = 0;
	while (1) {
		sts = I2C_SPIStsRegRead();
		if (!(sts & poll))	/* sfStatusRead() > 4.8us */ {
			break;
		}
		time ++;
		if( *ptimeOut < time ) {
			printk("iCatch: TimeOut %d, sts=0x%x, poll=0x%x\n",time,sts,poll);
			break;
		}
	}
}

int I2C_SPIStChipErase(
	void
)
{
	u32 timeout;
	int ret = 0;
	printk("iCatch: ST Chip Erasing...\n");

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	I2C_SPIFlashPortWrite(0x02);
	I2CDataWrite(0x40e7, 0x01);

	ret = I2C_SPIFlashWrEnable();
	if (ret) {
		printk("iCatch: ST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_ERASE_ALL);
	I2CDataWrite(0x40e7, 0x01);

	timeout = 0xffffffff;
	I2C_SPITimeOutWait(0x01, &timeout);
#if 0
	ros_thread_sleep(1);
#endif
	I2CDataWrite(0x40e7, 0x01);

	printk("iCatch: ST Chip Erased\n");
	return 0;
}

int I2C_SPISstChipErase(void)
{
	u32 timeout;
	int ret = 0;
	printk("iCatch: SST Chip Erasing...\n");

	ret = I2C_SPIFlashWrEnable();
	if (ret) {
		printk("iCatch: SST Chip Erase fail, ret= 0x%x\n", ret);
		return ret;
	}

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);	/*Write Status register command*/
	I2CDataWrite(0x40e7, 0x01);

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	I2C_SPIFlashPortWrite(0x02);
	I2CDataWrite(0x40e7, 0x01);

	I2C_SPIFlashWrEnable();
	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_ERASE_ALL);
	I2CDataWrite(0x40e7, 0x01);

	timeout = 0xffffffff;
	I2C_SPITimeOutWait(0x01, &timeout);

	printk("iCatch: SST Chip Erased\n");
	return 0;
}

void writeUpdateProgresstoFile(int page_left, int total_page_num)
{
	struct file *fp_progress = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;
	char str_progress[4];
	int percentage = 0;

	percentage = 100 * (total_page_num - page_left + 1)/total_page_num;

	if(page_left % 64 == 1){
		printk("%s: page:0x%x; percentage= %d;\n", __FUNCTION__, page_left, percentage);
		fp_progress = filp_open("/data/isp_fw_update_progress", O_RDWR | O_CREAT | O_TRUNC, S_IRUGO | S_IWUGO);
		if ( IS_ERR_OR_NULL(fp_progress) ){
			filp_close(fp_progress, NULL);
			printk("%s: open %s fail\n", __FUNCTION__, "/data/isp_fw_update_progress");
		}
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		offset = 0;
		if (fp_progress->f_op != NULL && fp_progress->f_op->write != NULL){
			sprintf(str_progress, "%d\n", percentage);
			fp_progress->f_op->write(fp_progress,
				str_progress,
				strlen(str_progress),
				&offset);
		}else
			pr_err("%s: f_op might be null\n", __FUNCTION__);
		set_fs(old_fs);
		filp_close(fp_progress, NULL);
	}
}

u32 I2C_SPIFlashWrite(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100;
	u32 rsvSec1, rsvSec2;

	rsvSec1 = pages*pageSize - 0x5000;
	rsvSec2 = pages*pageSize - 0x1000;
	addr = addr * pageSize;

	printk("iCatch: ST type writing...\n");
	total_page_count = (int)pages;

	while(pages) {
		page_count = (int)pages;
		writeUpdateProgresstoFile(page_count, total_page_count);
		/* reserve the last 2 ~ 5 sectors for calibration data */
		if((addr >= rsvSec1) && (addr < rsvSec2))
		{
			addr += 0x1000;
			pbuf += 0x1000;
			pages -= 0x10;
			continue;
		}
		I2C_SPIFlashWrEnable();
		I2CDataWrite(0x40e7, 0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG);	/* Write one byte command*/
		I2C_SPIFlashPortWrite((u8)(addr >> 16));	/* Send 3 bytes address*/
		I2C_SPIFlashPortWrite((u8)(addr >> 8));
		I2C_SPIFlashPortWrite((u8)(addr));

		for (i = 0; i < pageSize ; i++) {
			// How about "Early return" here?
			I2C_SPIFlashPortWrite(*pbuf);
			pbuf++;
		}
		I2CDataWrite(0x40e7, 0x01);

		addr += pageSize;
		pages --;
		// tmrUsWait(2000);
		udelay(2000);
	}
	printk("iCatch: ST type writing Done\n");
	return err;
}

void I2C_SPISstStatusWrite(u8 dat)
{
	u32 timeout, poll;

	I2C_SPIFlashWrEnable();

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);
	I2CDataWrite(0x40e7, 0x01);

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);
	I2C_SPIFlashPortWrite(dat);
	I2CDataWrite(0x40e7,0x01);

	poll = 0x01;
#if 0
	if( spiDev.bus != SPI_1BIT_MODE ) {/* 1 bit mode */
		poll = 0x80;
	} else {
		poll = 0x01;
	}
#endif
	timeout = 100000;
	I2C_SPITimeOutWait(poll, &timeout);
	//msleep(500);
	return;
}

u32 I2C_SPI64KBBlockErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	printk("%s(%d): addr:0x%x\n", __FUNCTION__, __LINE__, address);
	if(!stFlag) {
		I2C_SPIFlashWrEnable();

		I2CDataWrite(0x40e7, 0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);	/*Write Status register command*/
		I2CDataWrite(0x40e7, 0x01);
	}

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);	/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	I2CDataWrite(0x40e7, 0x01);

	I2C_SPIFlashWrEnable();

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_64KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);	/* A15~A08 */
	I2C_SPIFlashPortWrite(address);		/* A07~A00 */
	I2CDataWrite(0x40e7, 0x01);

	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);

	return 0;
}

u32 I2C_SPISectorErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	printk("%s(%d): addr:0x%x\n", __FUNCTION__, __LINE__, address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable();

		I2CDataWrite(0x40e7, 0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);	/*Write Status register command*/
		I2CDataWrite(0x40e7, 0x01);
	}

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);	/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	I2CDataWrite(0x40e7, 0x01);

	I2C_SPIFlashWrEnable();

	I2CDataWrite(0x40e7, 0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_SECTOR_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);	/* A15~A08 */
	I2C_SPIFlashPortWrite(address);		/* A07~A00 */
	I2CDataWrite(0x40e7, 0x01);

	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);

	return 0;
}

u32 I2C_SPI32KBBlockErase(
	u32 address,
	u32 stFlag
)
{
	u32 timeout;
	printk("%s(%d): addr:0x%x\n", __FUNCTION__, __LINE__, address);
	if(!stFlag)
	{
		I2C_SPIFlashWrEnable();

		I2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS_EN);	/*Write Status register command*/
		I2CDataWrite(0x40e7,0x01);
	}

	I2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_WRT_STS);	/*Write Status register command*/
	I2C_SPIFlashPortWrite(0x02);
	I2CDataWrite(0x40e7,0x01);

	I2C_SPIFlashWrEnable();

	I2CDataWrite(0x40e7,0x00);
	I2C_SPIFlashPortWrite(SPI_CMD_32KB_BLOCK_ERASE);
	I2C_SPIFlashPortWrite(address >> 16);	/* A23~A16 */
	I2C_SPIFlashPortWrite(address >> 8);	/* A15~A08 */
	I2C_SPIFlashPortWrite(address);		/* A07~A00 */
	I2CDataWrite(0x40e7,0x01);

	timeout = 5000000;
	I2C_SPITimeOutWait(0x01, &timeout);

	return 0;
}

void
BB_EraseSPIFlash(
	u32 type,
	u32 spiSize
)
{
	u8 typeFlag;
	u32 i, temp1;
	if( type == 2 )/* SST */
	{
		typeFlag = 0;
	}
	else if( type == 1 || type == 3 )/* ST */
	{
		typeFlag = 1;
	}
	/*printf("spiSize:0x%x\n",spiSize);*/
	if(spiSize == (512*1024)) {
		/* skip 0x7B000 ~ 0x7EFF, to keep calibration data */
		temp1 = (spiSize / 0x10000)-1;
		for(i=0;i<temp1;i++) {
			I2C_SPI64KBBlockErase(i*0x10000,typeFlag);
		}
		I2C_SPI32KBBlockErase(temp1*0x10000,typeFlag);
		temp1 = temp1*0x10000 + 0x8000;
		for(i=temp1;i<spiSize-0x5000;i+=0x1000) {
			I2C_SPISectorErase(i,typeFlag);
		}
		I2C_SPISectorErase(spiSize-0x1000,typeFlag);
	} else if(spiSize == (1024*1024)) {
		/* only erase 256*3KB */
		temp1 = ((spiSize*3/4) / 0x10000)-1;
		for(i=0;i<temp1;i++) {
			I2C_SPI64KBBlockErase(i*0x10000,typeFlag);
		}
		I2C_SPISectorErase(spiSize-0x1000,typeFlag);
	}
}

u32 I2C_SPISstFlashWrite(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100;
	u32 timeout = 100000;

	addr = addr * pageSize;

	printk("iCatch: SST type writing...\n");
	I2C_SPISstStatusWrite(0x40);

	total_page_count = (int)pages;

	while( pages ) {
		page_count = (int)pages;
		writeUpdateProgresstoFile(page_count, total_page_count);
		if((addr>=0x7C000) && (addr <0x7F000))
		{
			addr += 0x1000;
			pages -= 0x10;
			continue;
		}
		I2C_SPIFlashWrEnable();
		I2CDataWrite(0x40e7, 0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI);	/* Write one byte command*/
		I2C_SPIFlashPortWrite((u8)(addr >> 16));	/* Send 3 bytes address*/
		I2C_SPIFlashPortWrite((u8)(addr >> 8));
		I2C_SPIFlashPortWrite((u8)(addr));
		I2C_SPIFlashPortWrite(*pbuf);
		pbuf++;
		I2C_SPIFlashPortWrite(*pbuf);
		pbuf++;
		I2CDataWrite(0x40e7, 0x01);
		timeout = 100000;
		I2C_SPITimeOutWait(0x01, &timeout);

		for (i = 2; i < pageSize ; i = i+2) {
			I2CDataWrite(0x40e7, 0x00);
			I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG_AAI);
			I2C_SPIFlashPortWrite(*pbuf);
			pbuf++;
			I2C_SPIFlashPortWrite(*pbuf);
			pbuf++;
			I2CDataWrite(0x40e7, 0x01);
			timeout = 100000;
			I2C_SPITimeOutWait(0x01, &timeout);
		}

		I2CDataWrite(0x40e7, 0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		I2CDataWrite(0x40e7, 0x01);

		addr += pageSize;
		pages --;

		I2CDataWrite(0x40e7, 0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_WRT_DIS);
		I2CDataWrite(0x40e7,0x01);
	}
	printk("iCatch: SST type writing Done.\n");
	return err;
}

static int seqI2CDataRead(struct i2c_client *client, u16 addr, u8 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];
	if (!client->adapter)
		return -ENODEV;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8) (addr >> 8);;
	data[1] = (u8) (addr & 0xff);

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;

	msg[1].len = 4;
	msg[1].buf = data + 2;

	err = i2c_transfer(client->adapter, msg, 2);

	if (err != 2)
		return -EINVAL;

	memcpy(val, data+2, 4);

	return 0;
}

u32 I2C_7002DmemRd(
	u32 bankNum,
	u32 byteNum,
	u8* pbuf
)
{
	u32 i, bank;

	bank = 0x40+bankNum;
	I2CDataWrite(0x10A6,bank);

	for(i=0;i<byteNum;i+=4) {
		seqI2CDataRead(info->i2c_client, (0x1800+i), (pbuf+i));
	}

	bank = 0x40 + ((bankNum+1)%2);
	I2CDataWrite(0x10A6,bank);
}

u32 I2C_SPIFlashRead_DMA(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u8* pbufR;
	u32 ch, err = 0, dmemBank;
	u32 i, ret, count=0, size=0, bytes, offset;
	u32 pageSize = 0x100;

	addr = addr * pageSize;
	size = pages*pageSize;

	/* Set DMA bytecnt as 256-1 */
	I2CDataWrite(0x4170,0xff);
	I2CDataWrite(0x4171,0x00);
	I2CDataWrite(0x4172,0x00);

	/* Set DMA bank & DMA start address */
	I2CDataWrite(0x1084,0x01);
	I2CDataWrite(0x1080,0x00);
	I2CDataWrite(0x1081,0x00);
	I2CDataWrite(0x1082,0x00);

	/* enable DMA checksum and reset checksum */
	I2CDataWrite(0x4280,0x01);
	I2CDataWrite(0x4284,0x00);
	I2CDataWrite(0x4285,0x00);
	I2CDataWrite(0x4164,0x01);

	while(pages) {
		I2CDataWrite(0x40e7,0x00);
		I2C_SPIFlashPortWrite(SPI_CMD_BYTE_READ);	/* Write one byte command*/
		I2C_SPIFlashPortWrite((u8)(addr >> 16));	/* Send 3 bytes address*/
		I2C_SPIFlashPortWrite((u8)(addr >> 8));
		I2C_SPIFlashPortWrite((u8)(addr));

		if((pages%0x40) == 0x00) {
			printk("RE:0x%x addr:0x%x\n",pages, addr);
		}
		dmemBank = pages % 2;
		I2CDataWrite(0x1081,dmemBank*0x20);
		I2CDataWrite(0x1084,(1<<dmemBank));
		I2CDataWrite(0x4160,0x01);
		udelay(100);
		I2CDataWrite(0x40e7,0x01);
		I2C_7002DmemRd(dmemBank,pageSize,pbuf);

		pbuf += pageSize;
		pages--;
		addr += pageSize;
	}

	return err;
}

/* get_one_page_from_i7002a():
 *   Dump the ISP page whose index is "which_page" to "pagebuf".
 *   mclk, power & rst are requisite for getting correct page data.
 */
void get_one_page_from_i7002a(int which_page, u8* pagebuf)
{
	int i = 0;
	int ret = 0;
	//I2CDataWrite(0x70c4,0x00);
	//I2CDataWrite(0x70c5,0x00);
	sensor_write_reg(info->i2c_client, 0x70c4,0x00);
	sensor_write_reg(info->i2c_client, 0x70c5,0x00);

	ret = I2C_SPIInit();
	if (ret) {
		printk("%s: get nothing. ret= %d", __FUNCTION__, ret);
		return;
	}

	I2C_SPIFlashReadId();

	I2C_SPIFlashRead_DMA(which_page, 1, pagebuf);

#if 1 // dump to kmsg ?
	printk("page#%d:\n", which_page);
	for(i=0; i < 0x100; i++) {
		if(i%16 == 0)
			printk("[%03x]", i);
		printk("%02X ", pagebuf[i]);
		if(i%16 == 15)
			printk("\n");
	}
#endif
}

unsigned int get_fw_version_in_isp(void)
{
	u8 tmp_page[0x100];
	unsigned int vn = 0xABCDEF;
	int i = 0;
	int retry = 3;
	bool b_ok;

	for (i = 0; i < retry; i++) {
		int j =0;
		b_ok = true;

		if(dbg_i7002a_force_fw_size == 512) {
			get_one_page_from_i7002a(2047, tmp_page);
		} else if (dbg_i7002a_force_fw_size == 1024) {
			get_one_page_from_i7002a(4095, tmp_page);
		} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF500T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL) {
			/* The fw veriosn is in the page with the index, 4095.*/
			get_one_page_from_i7002a(4095, tmp_page);
		}
		else {
			/* The fw veriosn is in the page with the index, 2047.*/
			get_one_page_from_i7002a(2047, tmp_page);
		}

		/* The header format looks like:
		 * FF FF FF FF FF FF FF FF XX XX XX XX XX XX XX
		 * FF FF FF FF FF XX XX XX XX XX XX XX XX XX XX
		 */
		for (j = 0; j < 8; j++) {
			if (tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j] != 0xFF) {
				printk("%s: tmp_page[0x%02X]= %02X\n", __FUNCTION__,
					0x100 - BIN_FILE_HEADER_SIZE + j,
					tmp_page[0x100 - BIN_FILE_HEADER_SIZE + j]);
				b_ok = false;
				break;
			}
		}
		if (b_ok == true)
			break;
		else {
			printk("%s: wrong page data? Try again (%d).\n", __FUNCTION__, i);
			msleep(10);
		}
	}

	if (b_ok == true) {
		vn = (tmp_page[0xFF - 1] <<16) | (tmp_page[0xFF - 2] << 8) | tmp_page[0xFF -3];
		printk("%s: vn=0x%X\n", __FUNCTION__, vn);

		if ((tmp_page[0xFF - 4] == 0x27) && (tmp_page[0xFF - 5] == 0x20))
			fw_front_type_in_isp = 1;
		else if ((tmp_page[0xFF - 4] == 0x10) && (tmp_page[0xFF - 5] == 0x40))
			fw_front_type_in_isp = 2;
		else
			fw_front_type_in_isp = 0;
	} else
		fw_front_type_in_isp = 0;

	return vn;
}

void I2C_7002DmemWr(
	u32 bankNum,
	u32 byteNum,
	u8* pbuf
)
{
	u32 i, bank;
	u32 seqWriteByteNum = 64;

	bank = 0x40 + bankNum;
	I2CDataWrite(0x10A6, bank);

	for(i = 0; i < byteNum; i+=seqWriteByteNum) {
		/* sequentially write DMEM */
		seqI2CDataWrite(info->i2c_client, (0x1800+i), (pbuf+i), seqWriteByteNum);
	}

	bank = 0x40 + ((bankNum+1)%2);

	I2CDataWrite(0x10A6, bank);
}

u32 I2C_SPIFlashWrite_DMA(
	u32 addr,
	u32 pages,
	u8 *pbuf
)
{
	u32 i, err = 0;
	u32 pageSize = 0x100, size;
	u32 rsvSec1, rsvSec2;
	u32 dmemBank = 0;
	u32 chk1=0;
	u16 chk2, temp = 0;

	rsvSec1 = pages*pageSize - 0x7000;
	rsvSec2 = pages*pageSize - 0x1000;
	addr = addr * pageSize;

	/* Set DMA bytecnt as 256-1 */
	I2CDataWrite(0x4170, 0xff);
	I2CDataWrite(0x4171, 0x00);
	I2CDataWrite(0x4172, 0x00);

	/* Set DMA bank & DMA start address */
	I2CDataWrite(0x1084, 0x01);
	I2CDataWrite(0x1080, 0x00);
	I2CDataWrite(0x1081, 0x00);
	I2CDataWrite(0x1082, 0x00);

	/* enable DMA checksum and reset checksum */
	I2CDataWrite(0x4280, 0x01);
	I2CDataWrite(0x4284, 0x00);
	I2CDataWrite(0x4285, 0x00);
	I2CDataWrite(0x4164, 0x00);

	size = pages * pageSize;
	for(i = 0; i < size; i++) {
		if((i >= rsvSec2) || (i < rsvSec1)) {
			chk1 += *(pbuf+i);
		}
		if(chk1 >= 0x10000) {
			chk1 -= 0x10000;
		}
	}

	total_page_count = (int)pages;

	while(pages) {
		page_count = (int)pages;
		writeUpdateProgresstoFile(page_count, total_page_count);

		if((addr>=rsvSec1) && (addr <rsvSec2)) {
			addr += 0x1000;
			pbuf += 0x1000;
			pages -= 0x10;
			continue;
		}
		if((pages == 1)) {
			for (i = 0; i < pageSize ; i++) {
				printk("%2x ",*(pbuf+i));
				if((i % 0x10) == 0x0f)
					printk("\n");
			}
		}

		dmemBank = pages % 2;
		I2CDataWrite(0x1081, dmemBank * 0x20);
		I2CDataWrite(0x1084, (1 << dmemBank));

		I2C_7002DmemWr(dmemBank, pageSize, pbuf);

		I2C_SPIFlashWrEnable();
		I2CDataWrite(0x40e7, 0x00);

		I2C_SPIFlashPortWrite(SPI_CMD_BYTE_PROG);	/* Write one byte command*/
		I2C_SPIFlashPortWrite((u8)(addr >> 16));	/* Send 3 bytes address*/
		I2C_SPIFlashPortWrite((u8)(addr >> 8));
		I2C_SPIFlashPortWrite((u8)(addr));

		I2CDataWrite(0x4160, 0x01);
		//tmrUsWait(100);/* wait for DMA done */
		udelay(100);
		I2CDataWrite(0x40e7, 0x01);

		pbuf += pageSize;
		addr += pageSize;
		pages --;
	}

	//tmrUsWait(500);	/* wait for DMA done */
	udelay(500);

	//temp = hsI2CDataRead(0x4285);
	//chk2 = hsI2CDataRead(0x4284);
	sensor_read_reg(info->i2c_client, 0x4285, &temp);
	sensor_read_reg(info->i2c_client, 0x4284, &chk2);
	chk2 = chk2 | (temp<<8);
	printk("checksum: 0x%x 0x%x\n",chk1,chk2);

	return err;
}

void
BB_WrSPIFlash(char* binfile_path)
{
	u32 id, type;
	u32 pages, spiSize;

	u8 *pbootBuf;
	u8 bin_file_header[BIN_FILE_HEADER_SIZE];
	u8 checksum1_in_bin[2], checksum2_in_bin[2];
	u8 checksum1_in_isp[2], checksum2_in_isp[2];
	unsigned int version_num_in_bin = 0xFFFFFF;
	int firmware2_offset;
	u8 tmp_page[0x100];

	struct file *fp = NULL;
	mm_segment_t old_fs;
	struct inode *inode;
	int bootbin_size = 0;
	int i, ret = 0;

	fw_update_status = ICATCH_FW_IS_BURNING;

	/* Calculate BOOT.BIN file size. */
	fp = filp_open(binfile_path, O_RDONLY, 0);

	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		printk("%s: fp->f_dentry->d_inode->i_size=%d\n", __FUNCTION__, bootbin_size);
		pbootBuf = kmalloc(bootbin_size, GFP_KERNEL);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			int byte_count= 0;
			printk("Start to read %s\n", binfile_path);

			byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);

			if (byte_count <= 0) {
				printk("iCatch: EOF or error. last byte_count= %d;\n", byte_count);
				kfree(pbootBuf);
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				return;
			} else
				printk("iCatch: BIN file size= %d bytes\n", bootbin_size);

#if 0
			for(i=0; i < bootbin_size; i++) {
				printk("%c", pbootBuf[i]);
			}
			printk("\n");
#endif
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err("iCatch \"%s\" not found error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		return;
	} else{
		pr_err("iCatch \"%s\" open error\n", binfile_path);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		return;
	}

	printk("%s: bin_file_header=\n", __FUNCTION__);
	for (i=0; i < BIN_FILE_HEADER_SIZE; i++)
	{
		bin_file_header[i] = pbootBuf[bootbin_size - BIN_FILE_HEADER_SIZE + i];
		printk(" %02X", bin_file_header[i]);
		if ((i % 16) == 15)
			printk("\n");
	}
	version_num_in_bin = (bin_file_header[30] << 16) | (bin_file_header[29] << 8) | bin_file_header[28];

	/* Get the checksum in bin file.
	 *   firmware2_offset
	 *     = fw1 header size
	 *     + fw1 DMEM FICDMEM size
	 *     + fw1 IMEM size
	 */
	memcpy(checksum1_in_bin, pbootBuf + 10, 2);

	firmware2_offset = 16 +
		((pbootBuf[3] << 24) | (pbootBuf[2] << 16) | (pbootBuf[1] << 8) | pbootBuf[0]) +
		((pbootBuf[7] << 24) | (pbootBuf[6] << 16) | (pbootBuf[5] << 8) | pbootBuf[4]);
	memcpy(checksum2_in_bin, pbootBuf + firmware2_offset + 10, 2);

	printk("%s: checksum in bin:%02X %02X; %02X %02X\n", __FUNCTION__,
		checksum1_in_bin[0],checksum1_in_bin[1],checksum2_in_bin[0], checksum2_in_bin[1]);

	/* 20120828 updated */
	I2CDataWrite(0x1011,0x01); /* CPU SW RESET */
	I2CDataWrite(0x001C,0x08); /* reset FM register */
	I2CDataWrite(0x001C,0x00);
	I2CDataWrite(0x108C,0x00); /* DMA select*/

	ret = I2C_SPIInit();
	if (ret) {
		printk("%s: SPI init fail. ret= 0x%x", __FUNCTION__, ret);
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		return;
	}

	id = I2C_SPIFlashReadId();

	if(id==0) {
		printk("read id failed\n");
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		return;
	}

	type = BB_SerialFlashTypeCheck(id, &spiSize);
	if(type == 0) {
		printk("BB_SerialFlashTypeCheck(%d) failed\n", id);
		kfree(pbootBuf);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		return;
	}

	pages = bootbin_size/0x100;

	printk("%s: pages:0x%x\n", __FUNCTION__, pages);

	BB_EraseSPIFlash(type,spiSize);//for 8Mb spi flash test

	/* Writing Flash here */
	if( type == 2 ) {
		flash_type = ICATCH_FLASH_TYPE_SST;
		printk("SST operation\n");
		//ret = I2C_SPISstChipErase();
		if(ret) {
			printk("%s: SST erase fail.\n", __FUNCTION__);
			kfree(pbootBuf);
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			return;
		}
		I2C_SPISstFlashWrite(0, pages, pbootBuf);
	} else if( type == 1 || type == 3 ) {
		flash_type = ICATCH_FLASH_TYPE_ST;
		printk("ST operation: DMA\n");
		//ret = I2C_SPIStChipErase();
		if(ret) {
			printk("%s: ST erase fail.\n", __FUNCTION__);
			kfree(pbootBuf);
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			return;
		}
		I2C_SPIFlashWrite_DMA(0, pages, pbootBuf);
	} else {
		printk("type unknown: %d; Won't update iCatch FW.\n", type);
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		kfree(pbootBuf);
		return;
	}
	kfree(pbootBuf);

	/* Check the update reult. */
	/* Compare Check sum here */
	get_one_page_from_i7002a(0, tmp_page);
	memcpy(checksum1_in_isp, tmp_page + 10, 2);

	if (memcmp(checksum1_in_isp, checksum1_in_bin, 2) == 0) {
		/* checksum1 PASS */
		firmware2_offset = 16 +
			((tmp_page[3] << 24) | (tmp_page[2] << 16) | (tmp_page[1] << 8) | tmp_page[0]) +
			((tmp_page[7] << 24) | (tmp_page[6] << 16) | (tmp_page[5] << 8) | tmp_page[4]);

		get_one_page_from_i7002a(firmware2_offset >> 8, tmp_page);
		memcpy(checksum2_in_isp, tmp_page + 10, 2);

		if (memcmp(checksum2_in_isp, checksum2_in_bin, 2) == 0) {
			/* checksum2 PASS */
			version_num_in_isp = get_fw_version_in_isp();
			if (version_num_in_isp == version_num_in_bin) {
				/* version number PASS */
				fw_update_status = ICATCH_FW_UPDATE_SUCCESS;
				printk("%s: ICATCH FW UPDATE SUCCESS.\n", __FUNCTION__);
			} else {
				/* version number FAIL */
				fw_update_status = ICATCH_FW_UPDATE_FAILED;
				printk("%s: check version FAIL: ISP(0x%06X) != BIN(0x%06X)\n", __FUNCTION__, version_num_in_isp, version_num_in_bin);
				version_num_in_isp = 0xABCDEF;
				fw_front_type_in_isp = 0;
			}
		} else {
			/* checksum2 FAIL */
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			printk("%s: checksum2 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
				__FUNCTION__, checksum2_in_isp[0], checksum2_in_isp[1],
				checksum2_in_bin[0], checksum2_in_bin[1]);
			version_num_in_isp = 0xABCDEF;
			fw_front_type_in_isp = 0;
		}
	} else {
		/* checksum1 FAIL */
		fw_update_status = ICATCH_FW_UPDATE_FAILED;
		printk("%s: checksum1 FAIL: ISP(%02X %02X) != BIN(%02X %02X)\n",
			__FUNCTION__, checksum1_in_isp[0], checksum1_in_isp[1],
			checksum1_in_bin[0], checksum1_in_bin[1]);
		version_num_in_isp = 0xABCDEF;
		fw_front_type_in_isp = 0;
	}
}

static int sensor_set_mode(struct sensor_info *info, struct sensor_mode *mode)
{

	int sensor_table;
	u16 testval, i;

	pr_info("%s: xres %u yres %u\n",__func__, mode->xres, mode->yres);

	if (mode->xres == 3264 && mode->yres == 2448) {
		sensor_table = SENSOR_MODE_3264x2448;
		//sensor_write_reg(info->i2c_client, 0x7120, 0x00);//preview mode
		//sensor_write_reg(info->i2c_client, 0x7106, 0x01);
		if (factory_mode==2) {
			sensor_write_reg(info->i2c_client, 0x7106, 0x01);//preview mode
			sensor_write_reg(info->i2c_client, 0x7120, 0x00);
		}
		else {
			sensor_write_reg(info->i2c_client, 0x71EB, 0x01);//AE/AWB lock
			sensor_write_reg(info->i2c_client, 0x710f, 0x00);//capture mode
			sensor_write_reg(info->i2c_client, 0x7120, 0x01);
		}
		printk("%s: resolution supplied to set mode %d %d\n",
			__func__, mode->xres, mode->yres);
		for (i=0;i<200;i++)
		{
			sensor_read_reg(info->i2c_client, 0x72f8, &testval);
			printk("testval=0x%X, i=%d",testval,i);
			if (testval & 0x04) {
				sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
				sensor_read_reg(info->i2c_client, 0x72f8, &testval);
				printk("Clear testval=0x%X, i=%d\n",testval,i);
				break;
			}
			printk("testval=0x%X, i=%d",testval,i);
			msleep(iCatch7002a_init_delay);
		}
	}
	else if (mode->xres == 2592 && mode->yres == 1944) {
		capture_mode = true;
		sensor_table = SENSOR_MODE_2592x1944;
		printk("%s: resolution supplied to set mode %d %d\n",
			 __func__, mode->xres, mode->yres);
		if(factory_mode == 2){
			sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
			sensor_write_reg(info->i2c_client, 0x7106, 0x0A);//2592x1944
			sensor_write_reg(info->i2c_client, 0x7120, 0x00);//preview mode
		}
		else{
			sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
			sensor_write_reg(info->i2c_client, 0x710F, 0x03);//Burst mode
			sensor_write_reg(info->i2c_client, 0x7120, 0x03);//Non-ZSL capture mode
		}
		for (i=0;i<200;i++)
		{
			sensor_read_reg(info->i2c_client, 0x72f8, &testval);
			printk("testval=0x%X, i=%d",testval,i);
			if (testval & 0x04) {
				sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
				sensor_read_reg(info->i2c_client, 0x72f8, &testval);
				printk("Clear testval=0x%X, i=%d\n",testval,i);
				break;
			}
			printk("testval=0x%X, i=%d",testval,i);
			msleep(iCatch7002a_init_delay);
		}
	}
	else if (mode->xres == 1920 && mode->yres == 1080) {
		//Stop Burst mode
		if(capture_mode && (tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
		    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
		    capture_mode = false;
		    sensor_write_reg(info->i2c_client, 0x7122, 0x01);
		}
		sensor_table = SENSOR_MODE_1920x1080;
		if(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
		    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)
			sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
		sensor_write_reg(info->i2c_client, 0x7106, 0x02);//1920x1080
		sensor_write_reg(info->i2c_client, 0x7120, 0x00);//preview mode
		printk("%s: resolution supplied to set mode %d %d\n",
			__func__, mode->xres, mode->yres);
		if(first_open && (tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
			for (i=0;i<200;i++) {
				sensor_read_reg(info->i2c_client, 0x72c3, &testval);
				printk("testval=0x%X, i=%d",testval,i);
				if (testval & 0x01)
					break;
				msleep(iCatch7002a_init_long_delay);
			}
			sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
			sensor_read_reg(info->i2c_client, 0x72f8, &testval);
			printk("Clear testval=0x%X, i=%d\n",testval,i);
		}
		else{
			for (i=0;i<200;i++) {
				sensor_read_reg(info->i2c_client, 0x72f8, &testval);
				printk("testval=0x%X, i=%d",testval,i);
				if (testval & 0x04) {
					sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
					sensor_read_reg(info->i2c_client, 0x72f8, &testval);
					printk("Clear testval=0x%X, i=%d\n",testval,i);
					break;
				}
				printk("testval=0x%X, i=%d",testval,i);
				msleep(iCatch7002a_init_delay);
			}
		}
	        if(first_open)
			first_open=false;
		msleep(iCatch7002a_preview_delay);
	}
	else if (mode->xres == 1280 && mode->yres == 960) {
		//Stop Burst mode
		if(capture_mode && (tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
		    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
		    capture_mode = false;
		    sensor_write_reg(info->i2c_client, 0x7122, 0x01);
		}
		printk("%s: 1.2M Preview Mode\n", __func__);
		sensor_table = SENSOR_MODE_1280x960;
		//sensor_write_reg(info->i2c_client, 0x7120, 0x00);//preview mode
		//sensor_write_reg(info->i2c_client, 0x7106, 0x00);
		if(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
		    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)
			sensor_write_reg(info->i2c_client, 0x72f8, 0x04);		
		sensor_write_reg(info->i2c_client, 0x7106, 0x00);//preview mode
		sensor_write_reg(info->i2c_client, 0x7120, 0x00);
		//sensor_write_reg(info->i2c_client, 0x7106, 0x00);//workaround for ov2720 output size, not affect IMX175
		printk("%s: resolution supplied to set mode %d %d\n",
		__func__, mode->xres, mode->yres);
		if (!first_open || (tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)) {
			for (i=0;i<200;i++) {
				sensor_read_reg(info->i2c_client, 0x72f8, &testval);
				printk("testval=0x%X, i=%d",testval,i);
				if (testval & 0x04) {
					sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
					sensor_read_reg(info->i2c_client, 0x72f8, &testval);
					printk("Clear testval=0x%X, i=%d\n",testval,i);
					break;
				}
				printk("testval=0x%X, i=%d",testval,i);
				msleep(iCatch7002a_init_delay);
			}
		}
		else{
			first_open=false;
			if((tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
				tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
				for (i=0;i<200;i++) {
					sensor_read_reg(info->i2c_client, 0x72c3, &testval);
					printk("testval=0x%X, i=%d",testval,i);
					if (testval & 0x01)
						break;
					msleep(iCatch7002a_init_long_delay);
				}
				sensor_write_reg(info->i2c_client, 0x72f8, 0x04);
				sensor_read_reg(info->i2c_client, 0x72f8, &testval);
				printk("Clear testval=0x%X, i=%d\n",testval,i);
			}
		}
		msleep(iCatch7002a_preview_delay);
	}
	else if (mode->xres == 1280 && mode->yres == 720) {
		sensor_table = SENSOR_MODE_1280x720;
		sensor_write_reg(info->i2c_client, 0x7120, 0x00);//preview mode
		sensor_write_reg(info->i2c_client, 0x7106, 0x02);//1280x720
		printk("%s: resolution supplied to set mode %d %d\n",
			__func__, mode->xres, mode->yres);
	}
	else {
		pr_err("%s: invalid resolution supplied to set mode %d %d\n",
			__func__, mode->xres, mode->yres);
		return -EINVAL;
	}
	info->mode = sensor_table;
	return 0;
}

static long sensor_ioctl(struct file *file,
			 unsigned int cmd, unsigned long arg)
{
	struct sensor_info *info = file->private_data;
	int err=0;

	switch (cmd) {
	case SENSOR_IOCTL_SET_MODE:
	{
		struct sensor_mode mode;
		if (copy_from_user(&mode,(const void __user *)arg,
			sizeof(struct sensor_mode))) {
			return -EFAULT;
		}
		return sensor_set_mode(info, &mode);
	}
	case SENSOR_IOCTL_GET_STATUS:
	{
		return 0;
	}
	case SENSOR_IOCTL_SET_CAMERA:
	{
		u8 is_front_camera = 0;
		//u16 testval;
		if (copy_from_user(&is_front_camera,(const void __user *)arg,
			sizeof(is_front_camera))) {
			return -EFAULT;
		}
		printk("SET_CAMERA as 0x%X\n", is_front_camera);
		if(IsTF300())
			msleep(100);
		else
			msleep(10);
		//sensor_read_reg(info->i2c_client, 0x002c, &testval);
		//printk("%s: test val is %d\n", __func__, testval);
		if (is_calibration) {
			sensor_write_reg(info->i2c_client, 0x1011, 0x01);//cpu reset
			if(IsTF300()){
				sensor_write_reg(info->i2c_client, 0x941C, 0x04);
				sensor_write_reg(info->i2c_client, 0x9010, 0x01);
				sensor_write_reg(info->i2c_client, 0x9010, 0x00);
			}
			else{
				sensor_write_reg(info->i2c_client, 0x001C, 0x08);
				sensor_write_reg(info->i2c_client, 0x001C, 0x00);
				sensor_write_reg(info->i2c_client, 0x1010, 0x02);
				sensor_write_reg(info->i2c_client, 0x1010, 0x00);
			}
			sensor_write_reg(info->i2c_client, 0x1306, 0x02);//calibration
			sensor_write_reg(info->i2c_client, 0x1011, 0x00);
			if(IsTF300())
				msleep(100);
			else
				msleep(10);
			//sensor_write_reg(info->i2c_client, 0x7188, 0x01);//let AF windows work
			break;
		}
		sensor_write_reg(info->i2c_client, 0x1011, 0x01);//cpu reset
		if(IsTF300()){
			sensor_write_reg(info->i2c_client, 0x941C, 0x04);
			sensor_write_reg(info->i2c_client, 0x9010, 0x01);
			sensor_write_reg(info->i2c_client, 0x9010, 0x00);
		}
		else{
			sensor_write_reg(info->i2c_client, 0x001C, 0x08);
			sensor_write_reg(info->i2c_client, 0x001C, 0x00);
			sensor_write_reg(info->i2c_client, 0x1010, 0x02);
			sensor_write_reg(info->i2c_client, 0x1010, 0x00);
		}
		if (is_front_camera)
			sensor_write_reg(info->i2c_client, 0x1306, 0x01);//front camera
		else
			sensor_write_reg(info->i2c_client, 0x1306, 0x00);//rear camera
		sensor_write_reg(info->i2c_client, 0x1011, 0x00);
		msleep(100);
		sensor_write_reg(info->i2c_client, 0x7188, 0x01);//let AF windows work
		break;
	}
	case SENSOR_IOCTL_SET_COLOR_EFFECT:
	{
		if( tegra3_get_project_id() == TEGRA3_PROJECT_TF500T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL) {
			if (copy_from_user(&coloreffect,(const void __user *)arg,
				sizeof(coloreffect))) {
				return -EFAULT;
			}
			printk("SET_COLOR_EFFECT as 0x%X\n", coloreffect);
			switch(coloreffect) {
			case YUV_ColorEffect_None:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x00);//none
				break;
			case YUV_ColorEffect_Aqua:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x01);//aqua
				break;
			case YUV_ColorEffect_Negative:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x02);//negative
				break;
			case YUV_ColorEffect_Sepia:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x03);//sepia
				break;
			case YUV_ColorEffect_Mono:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x04);//grayscale
				break;
			case YUV_ColorEffect_Vivid:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x05);//vivid
				break;
			case YUV_ColorEffect_Aura:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x06);//aura
				break;
			case YUV_ColorEffect_Vintage:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x07);//vintage
				break;
			case YUV_ColorEffect_Vintage2:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x08);//vintage2
				break;
			case YUV_ColorEffect_Lomo:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x09);//lomo
				break;
			case YUV_ColorEffect_Red:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x0a);//red
				break;
			case YUV_ColorEffect_Blue:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x0b);//blue
				break;
			case YUV_ColorEffect_Yellow:
				err = sensor_write_reg(info->i2c_client, 0x7102, 0x0c);//yellow
				break;
			default:
				printk("Unknown color effect: 0x%X\n", coloreffect);
				break;
			}

			if (err) {
				printk("SET_COLOR_EFFECT error: 0x%x\n", err);
				return err;
			}
		} else
			printk("COLOR EFFECT is unsupported.\n");

		return 0;
	}
	case SENSOR_IOCTL_SET_WHITE_BALANCE:
	{
		u8 whitebalance;

		if (copy_from_user(&whitebalance,(const void __user *)arg,
			sizeof(whitebalance))) {
			return -EFAULT;
		}
		printk("SET_WHITE_BALANCE as %d\n", whitebalance);
		switch(whitebalance) {
		case YUV_Whitebalance_Auto:
			err = sensor_write_reg(info->i2c_client, 0x710A, 0x00);//auto
			break;
		case YUV_Whitebalance_Incandescent:
			err = sensor_write_reg(info->i2c_client, 0x710A, 0x06);//Incandescent
			break;
		case YUV_Whitebalance_Daylight:
			err = sensor_write_reg(info->i2c_client, 0x710A, 0x01);//Daylight
			break;
		case YUV_Whitebalance_Fluorescent:
			err = sensor_write_reg(info->i2c_client, 0x710A, 0x05);//Fluorescent_H
			break;
		case YUV_Whitebalance_CloudyDaylight:
			err = sensor_write_reg(info->i2c_client, 0x710A, 0x02);//Cloudy
			break;
		case YUV_Whitebalance_WarmFluorescent:
			err = sensor_write_reg(info->i2c_client, 0x710A, 0x04);//Fluorescent_L
			break;
		case YUV_Whitebalance_Shade:
			err = sensor_write_reg(info->i2c_client, 0x710A, 0x03);//Shade
			break;
		default:
			break;
		}
		if (err)
			return err;
		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_SET_FLASH_STATUS:
	{
		u8 flash_led_type;
		if (copy_from_user(&flash_led_type,(const void __user *)arg,
			sizeof(flash_led_type))) {
			return -EFAULT;
		}
		printk("SET_FLASH_LED as %d\n", flash_led_type);
		switch(flash_led_type) {
		case YUV_FlashControlOn:
			err = sensor_write_reg(info->i2c_client, 0x7104, 0x02);//on
			break;
		case YUV_FlashControlOff:
			err = sensor_write_reg(info->i2c_client, 0x7104, 0x01);//off
			break;
		case YUV_FlashControlAuto:
			err = sensor_write_reg(info->i2c_client, 0x7104, 0x00);//auto
			break;
		case YUV_FlashControlTorch:
			err = sensor_write_reg(info->i2c_client, 0x7104, 0x04);//torch
			break;
		default:
			break;
		}
		break;
	}
	case SENSOR_CUSTOM_IOCTL_GET_FLASH_STATUS:
	{
		int flash_status;
		u16 reg_status=0;

		if (copy_from_user(&flash_status,(const void __user *)arg,
			sizeof(flash_status))) {
			return -EFAULT;
		}
		sensor_read_reg(info->i2c_client, 0x72B9, &reg_status);
		if (reg_status == 0x00) //off
			flash_status = 0;
		else if (reg_status == 0x01) //on
			flash_status = 2;
		printk("GET_FLASH_STATUS as value:%d\n", flash_status);
		if (copy_to_user((const void __user *)arg, &flash_status, sizeof(flash_status)))
			return -EFAULT;

		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_SET_SCENEMODE:
	{
		u8 scene_mode;
		if (copy_from_user(&scene_mode,(const void __user *)arg,
			sizeof(scene_mode))) {
			return -EFAULT;
		}
		printk("SET_SCENEMODE as %d\n", scene_mode);
		switch(scene_mode) {
		case YUV_SceneMode_Invalid:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x00);//need to confirm
			break;
		case YUV_SceneMode_Auto:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x00);
			break;
		case YUV_SceneMode_Portrait:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x0A);
			break;
		case YUV_SceneMode_Landscape:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x06);//The same with vivid
			break;
		case YUV_SceneMode_Sports:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x0C);
			break;
		case YUV_SceneMode_Night:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x07);
			break;
		case YUV_SceneMode_Sunset:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x0E);
			break;
		case YUV_SceneMode_Snow:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x0B);
			break;
		case YUV_SceneMode_Party:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x09);
			break;
		case YUV_SceneMode_BackLight:
			err = sensor_write_reg(info->i2c_client, 0x7109, 0x16);
			break;
		default:
			break;
		}
		break;
	}
	case SENSOR_CUSTOM_IOCTL_SET_AF_MODE:
	{
		custom_af_cmd_package AF_cmd;
		//return 0;// disable AF temprarily
		if (copy_from_user(&AF_cmd,(const void __user *)arg,
			sizeof(AF_cmd))) {
			return -EFAULT;
		}
		switch(AF_cmd.cmd) {
			case AF_CMD_START:
			{
				u16 FW_Status = 1;
				af_start = true;
				pr_info("AF cmd start!\n");
				/*
				err = sensor_read_reg(info->i2c_client, 0x3029, &FW_Status);
				if (err)
					return err;
				while (FW_Status != 0 && FW_Status != 0x10 && FW_Status !=0x70) {
					if (i < 5)
						i++;
					else
						break;
					msleep(10);
					err = sensor_read_reg(info->i2c_client, 0x3029, &FW_Status);
					if (err)
						return err;
					pr_info("FW_Status is %x\n", FW_Status);
				}
				pr_info("FW_Status is %x\n", FW_Status);
				*/
				//err = sensor_write_table(info->i2c_client, Autofocus_Trigger);
				if (touch_mode==TOUCH_STATUS_OFF) {
					err = sensor_write_reg(info->i2c_client, 0x7188, 0x01);//ROI on
					err = sensor_write_reg(info->i2c_client, 0x7140, 0x00);
					err = sensor_write_reg(info->i2c_client, 0x7141, 0xC0);//0x50
					err = sensor_write_reg(info->i2c_client, 0x7142, 0x01);
					err = sensor_write_reg(info->i2c_client, 0x7143, 0xA0);//0xD8
					err = sensor_write_reg(info->i2c_client, 0x7144, 0x01);
					err = sensor_write_reg(info->i2c_client, 0x7145, 0xA0);//0xD8
					err = sensor_write_reg(info->i2c_client, 0x7146, 0x01);
				} else
					err = sensor_write_reg(info->i2c_client, 0x7146, 0x01);
				if (err)
					return err;
				break;
			}
			case AF_CMD_ABORT:
			{
				/*
				u16 FW_Status = 1;
				u16 MAIN_ACK = 1;
				u16 i = 0;

				err = sensor_write_table(info->i2c_client, Autofocus_Release);
				if (err)
					return err;
				//pr_info("fail to write i2c seq!");        //return NV_FALSE;
				for (i=0; i<10; i++) { //wait for ACK = 0
					sensor_read_reg(info->i2c_client, 0x3023, &MAIN_ACK);
					if (!MAIN_ACK) {
						//NvOdmImagerI2cRead(&pContext->I2c, 0x3022, &MAIN);
						//NvOsDebugPrintf("MAIN is %d\n", MAIN);
						//pr_info("ACK is %d\n", MAIN_ACK);
						sensor_read_reg(info->i2c_client, 0x3029, &FW_Status);
						pr_info("FW_Status is %x\n", FW_Status);
						break;
					}
					msleep(10);
				}
				*/
				if(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
					tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)
					err = sensor_write_reg(info->i2c_client, 0x714F, 0x01);//af abort
				else
					err = sensor_write_reg(info->i2c_client, 0x714F, 0x00);//release focus
				//err = sensor_write_reg(info->i2c_client, 0x710E, 0x00);//Seems default AE window size
				touch_mode = TOUCH_STATUS_OFF;
				if (err)
					return err;
				break;
			}
			case AF_CMD_SET_POSITION:
			case AF_CMD_SET_WINDOW_POSITION:
			case AF_CMD_SET_WINDOW_SIZE:
			case AF_CMD_SET_AFMODE:
			case AF_CMD_SET_CAF:
			default:
				pr_info("AF cmd %d not implemented yet\n",AF_cmd.cmd);
			return -1;
		}
		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_GET_AF_MODE:
	{
		custom_af_cmd_package AF_cmd;
		if (copy_from_user(&AF_cmd,(const void __user *)arg,
			sizeof(AF_cmd))) {
			return -EFAULT;
		}
#if 0 //disable AF temporarily.
		AF_cmd.data = 1; //Locked
		copy_to_user((const void __user *)arg, &AF_cmd, sizeof(AF_cmd));
		return 0;
#endif
		switch(AF_cmd.cmd) {
			case AF_CMD_GET_AF_STATUS:
			{
				u16 AF_status;
				u16 AF_result;
				if(af_start && (tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
						tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)){
					msleep(100);
					af_start = false;
				}
				sensor_read_reg(info->i2c_client, 0x72A0, &AF_status);
				if (AF_status) {
					pr_info("AF searching... %d\n", AF_status);
					AF_cmd.data = 0; //busy
					copy_to_user((const void __user *)arg, &AF_cmd, sizeof(AF_cmd));
					break;
				}
				sensor_read_reg(info->i2c_client, 0x72A1, &AF_result);
				pr_info("AF result... %d\n", AF_result);
				if (AF_result==0)
					AF_cmd.data = 1; //Locked
				else
					AF_cmd.data = 2; //failed to find
				//err = sensor_write_reg(info->i2c_client, 0x71EB, 0x01);//release AWB/AE lock
				copy_to_user((const void __user *)arg, &AF_cmd, sizeof(AF_cmd));
				pr_info("AF done and release AWB/AE lock... %d\n", AF_result);
				break;
			}
			default:
				pr_info("AF cmd %d not implemented yet\n",AF_cmd.cmd);
				return -1;
		}
		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_SET_EV:
	{
		short ev;

		if (copy_from_user(&ev,(const void __user *)arg, sizeof(short)))
			return -EFAULT;

		printk("SET_EV as %d\n",ev);
		switch(ev) {
		case -6:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x0C);
			break;
		case -5:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x0B);
			break;
		case -4:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x0A);
			break;
		case -3:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x09);
			break;
		case -2:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x08);
			break;
		case -1:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x07);
			break;
		case 0:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x06);
			break;
		case 1:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x05);
			break;
		case 2:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x04);
			break;
		case 3:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x03);
			break;
		case 4:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x02);
			break;
		case 5:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x01);
			break;
		case 6:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x00);
			break;
		default:
			err = sensor_write_reg(info->i2c_client, 0x7103, 0x06);
			break;
		}
		if (err)
			return err;
		return 0;
	}

	case SENSOR_CUSTOM_IOCTL_GET_EV:
	{
		int EV;
		u16 tmp;

		/* [0x72b4] will return a number from 12 to 0.
		 * It stands for "EV-2 ~ EV+2".
		 */

		sensor_read_reg(info->i2c_client, 0x72b4, &tmp);

		EV = 6 - tmp;
		printk("GET_EV: [0x72b4]:0x%X;\n", EV);

		if (copy_to_user((const void __user *)arg, &EV, sizeof(EV)))
			return -EFAULT;

		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_GET_ISO:
	{
		short iso;
		u16 ISO_lowbyte = 0;
		u16 ISO_highbyte = 0;
		sensor_read_reg(info->i2c_client, 0x72b7, &ISO_lowbyte);
		sensor_read_reg(info->i2c_client, 0x72b8, &ISO_highbyte);

		iso = (ISO_highbyte << 8) | ISO_lowbyte;

		printk("GET_ISO as value:%d\n", iso);
		if (copy_to_user((const void __user *)arg, &iso, sizeof(short)))
			return -EFAULT;

		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_FETCH_EXIF:
	{
		printk("iCatch: FETCH EXIF.\n");
		sensor_write_reg(info->i2c_client, 0x71EB, 0x01); // Get EXIF
		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_GET_ET:
	{
		custom_et_value_package ET;
		u16 et_numerator = 0;
		u16 et_denominator_byte1 = 1;
		u16 et_denominator_byte2 = 0;
		u16 et_denominator_byte3 = 0;

		sensor_read_reg(info->i2c_client, 0x72b0, &et_numerator);
		sensor_read_reg(info->i2c_client, 0x72b1, &et_denominator_byte1);
		sensor_read_reg(info->i2c_client, 0x72b2, &et_denominator_byte2);
		sensor_read_reg(info->i2c_client, 0x72b3, &et_denominator_byte3);

		printk("GET_ET: [0x72b0]:0x%X; [0x72b1]:0x%X\n", et_numerator, et_denominator_byte1);
		printk("GET_ET: [0x72b2]:0x%X; [0x72b3]:0x%X\n", et_denominator_byte2, et_denominator_byte3);

		ET.exposure = et_numerator;
		ET.vts = (et_denominator_byte3 << 16)|(et_denominator_byte2 << 8)|et_denominator_byte1;

		if (copy_to_user((const void __user *)arg, &ET, sizeof(ET)))
		{
			return -EFAULT;
		}

		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF:
	{
		custom_touch_af_cmd_package touch_af;
		u32 af_w, af_h, af_x, af_y;
		if (copy_from_user(&touch_af,(const void __user *)arg, sizeof(custom_touch_af_cmd_package)))
			return -EFAULT;
/*
		if (!touch_focus_enable) {
			printk("%s: SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF blocked\n", __func__);
			break;
		}
*/
		if(touch_af.zoom){
			touch_mode = TOUCH_STATUS_ON;
			af_w = touch_af.win_w;
			af_h = touch_af.win_h;
			//printk("SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF: af_w:0x%x af_h:0x%x af_x:0x%x af_y:0x%x\n",
			//	touch_af.win_w, touch_af.win_h, touch_af.win_x, touch_af.win_y);
			af_x = touch_af.win_x;
			af_y = touch_af.win_y;
			printk("SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF: af_w:0x%x af_h:0x%x af_x:0x%x af_y:0x%x\n", af_w, af_h, af_x, af_y);
			//AE window
			//err = sensor_write_reg(info->i2c_client, 0x7188, 0x01);//ROI on
			//err = sensor_write_reg(info->i2c_client, 0x7148, af_w>>8);
			//err = sensor_write_reg(info->i2c_client, 0x7149, af_w&0xff);
			//err = sensor_write_reg(info->i2c_client, 0x714A, af_x>>8);
			//err = sensor_write_reg(info->i2c_client, 0x714B, af_x&0xff);
			//err = sensor_write_reg(info->i2c_client, 0x714C, af_y>>8);
			//err = sensor_write_reg(info->i2c_client, 0x714D, af_y&0xff);
			//AF window
			err = sensor_write_reg(info->i2c_client, 0x7188, 0x01);//ROI on
			err = sensor_write_reg(info->i2c_client, 0x7140, af_w>>8);
			err = sensor_write_reg(info->i2c_client, 0x7141, af_w&0xff);
			err = sensor_write_reg(info->i2c_client, 0x7142, af_x>>8);
			err = sensor_write_reg(info->i2c_client, 0x7143, af_x&0xff);
			err = sensor_write_reg(info->i2c_client, 0x7144, af_y>>8);
			err = sensor_write_reg(info->i2c_client, 0x7145, af_y&0xff);
			//touch_focus_enable=0;
		} else {
			if(touch_mode != TOUCH_STATUS_OFF){
				touch_mode = TOUCH_STATUS_OFF;
				printk("SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF: Cancel touch af\n");
				//err = sensor_write_reg(info->i2c_client, 0x710E, 0x00);//Seems default AE window size
				err = sensor_write_reg(info->i2c_client, 0x714F, 0x00);//release focus
			}
		}
		break;
	}
	case SENSOR_CUSTOM_IOCTL_SET_ISO:
	{
		u8 iso;
		if (copy_from_user(&iso,(const void __user *)arg,
			sizeof(iso))) {
			return -EFAULT;
		}
		printk("SET_ISO as %d\n", iso);
		switch(iso) {
		case YUV_ISO_AUTO:
			err = sensor_write_reg(info->i2c_client, 0x7110, 0x00);
			break;
		case YUV_ISO_50:
			err = sensor_write_reg(info->i2c_client, 0x7110, 0x01);
			break;
		case YUV_ISO_100:
			err = sensor_write_reg(info->i2c_client, 0x7110, 0x02);
			break;
		case YUV_ISO_200:
			err = sensor_write_reg(info->i2c_client, 0x7110, 0x03);
			break;
		case YUV_ISO_400:
			err = sensor_write_reg(info->i2c_client, 0x7110, 0x04);
			break;
		case YUV_ISO_800:
			err = sensor_write_reg(info->i2c_client, 0x7110, 0x05);
			break;
		case YUV_ISO_1600:
			err = sensor_write_reg(info->i2c_client, 0x7110, 0x06);
			break;
		default:
			break;
		}
		break;
	}
	case SENSOR_CUSTOM_IOCTL_SET_FLICKERING:
	{
		u8 flickering;
		if (copy_from_user(&flickering,(const void __user *)arg,
			sizeof(flickering))) {
			return -EFAULT;
		}
		printk("SET_FLICKERING as %d\n", flickering);
		switch(flickering) {
		case YUV_ANTIBANGING_50HZ:
			err = sensor_write_reg(info->i2c_client, 0x7101, 0x01);
			break;
		case YUV_ANTIBANGING_60HZ:
			err = sensor_write_reg(info->i2c_client, 0x7101, 0x02);
			break;
		default:
			break;
		}
		break;
	}
	case SENSOR_CUSTOM_IOCTL_GET_CAF_STATE:
	{
		u16 caf_state;

		sensor_read_reg(info->i2c_client, 0x72a0, &caf_state);
		// printk("GET_CAF_STATE: [0x72a0]:0x%X;\n", caf_state);

		if (copy_to_user((const void __user *)arg, &caf_state, sizeof(caf_state)))
			return -EFAULT;

		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_SET_CONTINUOUS_AF:
	{
		u8 continuous_af;
		if (copy_from_user(&continuous_af,(const void __user *)arg,
			sizeof(continuous_af))) {
			return -EFAULT;
		}
		printk("SET_CONTINUOUS_AF as %d\n", continuous_af);
		if(continuous_af==1){
			caf_mode = true;
			err = sensor_write_reg(info->i2c_client, 0x7105, 0x03);//CAF
		} else if (continuous_af==0 && focus_control==2) {
			caf_mode = false;
			err = sensor_write_reg(info->i2c_client, 0x7105, 0x00);//auto
			if(err)
				pr_err("CAF stop error\n");
		} else if (continuous_af==0 && focus_control==0) {
			caf_mode = false;
			err = sensor_write_reg(info->i2c_client, 0x7105, 0x02);//infinity
			if(err)
				pr_err("Infinity focus error\n");
		}
		if (err)
			return err;
		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_SET_ICATCH_AE_WINDOW:
	{
		custom_ae_win_cmd_package ae_win;
		if (copy_from_user(&ae_win,(const void __user *)arg,
			sizeof(custom_ae_win_cmd_package))) {
			return -EFAULT;
		}
		printk("SET_AE_WINDOW as start at 0x%X, width is 0x%X\n", ae_win.win_x, ae_win.win_w);
		if (ae_win.zoom==0) //default AE window
			err = sensor_write_reg(info->i2c_client, 0x7188, 0x00);//ROI off
		else {
			//AE window
			err = sensor_write_reg(info->i2c_client, 0x7188, 0x01);//ROI on
//			err = sensor_write_reg(info->i2c_client, 0x714E, 0x02);//the same with af window
			err = sensor_write_reg(info->i2c_client, 0x7148, (ae_win.win_w)>>8);
			err = sensor_write_reg(info->i2c_client, 0x7149, (ae_win.win_w)&0xff);
			err = sensor_write_reg(info->i2c_client, 0x714A, (ae_win.win_x)>>8);
			err = sensor_write_reg(info->i2c_client, 0x714B, (ae_win.win_x)&0xff);
			err = sensor_write_reg(info->i2c_client, 0x714C, (ae_win.win_y)>>8);
			err = sensor_write_reg(info->i2c_client, 0x714D, (ae_win.win_y)&0xff);

			// TAE on (Use TAE ROI)
			err = sensor_write_reg(info->i2c_client, 0x714E, 0x01);
		}
		if (err)
			return err;
		break;
	}
	case SENSOR_CUSTOM_IOCTL_SET_WDR:
	{
		if (tegra3_get_project_id() == TEGRA3_PROJECT_TF500T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL) {
			u16 wdr_on;
			u16 dwdr_control;
			if (copy_from_user(&wdr_on,(const void __user *)arg,
				sizeof(wdr_on))) {
				return -EFAULT;
			}
			wdr_on = wdr_on & 0x1;
			printk("SET_WDR as 0x%X\n", wdr_on);
			sensor_read_reg(info->i2c_client, 0x729B, &dwdr_control); //DWDR read

			if (wdr_on) {
				printk("SET 0x%x to [0x711B]\n", dwdr_control | 0x0001);
				err = sensor_write_reg(info->i2c_client, 0x711B, dwdr_control | 0x0001);
			} else {
				printk("SET 0x%x to [0x711B]\n", dwdr_control & 0xFFFE);
				err = sensor_write_reg(info->i2c_client, 0x711B, dwdr_control & 0xFFFE);
			}
		} else
			printk("WDR is unsupported.\n");

		break;
	}
	case SENSOR_CUSTOM_IOCTL_SET_AURA:
	{
		if (tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
		    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL) {
			if (coloreffect == YUV_ColorEffect_Aura) {
				u16 aura;
				if (copy_from_user(&aura, (const void __user *)arg,
					sizeof(aura))) {
					return -EFAULT;
				}

				printk("SET_AURA as 0x%X\n", aura);
					err = sensor_write_reg(info->i2c_client, 0x7119, aura);
			} else
				printk("SET_AURA: Coloereffect is not aura. No action.\n");
		} else
			printk("AURA is unsupported.\n");

		break;
	}
	case SENSOR_CUSTOM_IOCTL_SET_AE_LOCK:
	{
		u32 ae_lock;
		if (copy_from_user(&ae_lock,(const void __user *)arg,
			sizeof(ae_lock))) {
			return -EFAULT;
		}
		//ae_mode = ae_lock;
		printk("SET_AE_LOCK as 0x%x\n", ae_lock);
		if (ae_lock==1) {
			//sensor_write_reg(info->i2c_client, 0x71E4, 0x04);//AE command
			//sensor_write_reg(info->i2c_client, 0x71E5, 0x01);//AE lock
			//sensor_write_reg(info->i2c_client, 0x71E8, 0x01);
			sensor_write_reg(info->i2c_client, 0x71EB, 0x03);
		} else if (ae_lock==0) {
			//sensor_write_reg(info->i2c_client, 0x71E4, 0x04);//AE command
			//sensor_write_reg(info->i2c_client, 0x71E5, 0x02);//AE unlock
			//sensor_write_reg(info->i2c_client, 0x71E8, 0x01);
			sensor_write_reg(info->i2c_client, 0x71EB, 0x05);
		}
		break;
	}
	case SENSOR_CUSTOM_IOCTL_SET_AWB_LOCK:
	{
		u32 awb_lock;
		if (copy_from_user(&awb_lock,(const void __user *)arg,
			sizeof(awb_lock))) {
			return -EFAULT;
		}
		//awb_mode = awb_lock;
		printk("SET_AWB_LOCK as 0x%x\n", awb_lock);
		if (awb_lock==1) {
			//sensor_write_reg(info->i2c_client, 0x71E4, 0x05);//AWB command
			//sensor_write_reg(info->i2c_client, 0x71E5, 0x01);//AWB lock
			//sensor_write_reg(info->i2c_client, 0x71E8, 0x01);
			sensor_write_reg(info->i2c_client, 0x71EB, 0x04);
		} else if (awb_lock==0) {
			//sensor_write_reg(info->i2c_client, 0x71E4, 0x05);//AWB command
			//sensor_write_reg(info->i2c_client, 0x71E5, 0x02);//AWB unlock
			//sensor_write_reg(info->i2c_client, 0x71E8, 0x01);
			sensor_write_reg(info->i2c_client, 0x71EB, 0x06);
		}
		break;
	}
	case SENSOR_CUSTOM_IOCTL_SET_AF_CONTROL:
	{
		//int focus_mode;
		if (copy_from_user(&focus_control,(const void __user *)arg,
			sizeof(focus_control))) {
			return -EFAULT;
		}
		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_FW_UPDATE_PROGRAM:
	{
		custom_fw_update_rom_package rom_cmd;
		printk("%s(FW_UPDATE_PROGRAM)++\n", __FUNCTION__);
		if (copy_from_user(&rom_cmd,(const void __user *)arg, sizeof(rom_cmd))) {
			fw_update_status = ICATCH_FW_UPDATE_FAILED;
			return -EFAULT;
		}
		printk("binfile_path=%s; cmd=%d; flash_rom_start_address=%d; program_size= %d\n",
			rom_cmd.binfile_path, rom_cmd.cmd, rom_cmd.flash_rom_start_address, rom_cmd.program_size);

		i7002a_isp_on(1);

		printk("%s: BB_WrSPIFlash()++\n", __FUNCTION__);
		BB_WrSPIFlash(rom_cmd.binfile_path);
		printk("%s: BB_WrSPIFlash()--\n", __FUNCTION__);

		i7002a_isp_on(0);

		if(fw_update_status != ICATCH_FW_UPDATE_SUCCESS){
			printk("i7002a Update FAIL: %d\n", -fw_update_status);
			return -fw_update_status;
		}

		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_REG_SET:
	{
		register_setting reg;
		if (copy_from_user(&reg, (const void __user *)arg, sizeof(register_setting)))
			return -EFAULT;
		printk("SENSOR_CUSTOM_IOCTL_REG_SET");
		err = sensor_write_reg(info->i2c_client, reg.addr, reg.val);
		if (err)
			return err;
		return 0;
	}
	case SENSOR_CUSTOM_IOCTL_REG_GET:
	{
		register_setting reg;
		u16 val;
		if (copy_from_user(&reg, (const void __user *)arg, sizeof(register_setting)))
			return -EFAULT;
		printk("SENSOR_CUSTOM_IOCTL_REG_GET");
		err = sensor_read_reg(info->i2c_client, reg.addr, &val);
		if (err)
			return err;
		reg.val = val;
		if (copy_to_user((const void __user *)arg, &reg, sizeof(register_setting)))
			return -EFAULT;
		return 0;
	}

	default:
		return -EINVAL;
	}
	return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
	int ret;

	pr_info("yuv %s and sensor_opened is %d, calibrating=%d\n",__func__, sensor_opened, calibrating);
	file->private_data = info;
	if (info->pdata && info->pdata->power_on && !sensor_opened) {
		ret = info->pdata->power_on();
		if (ret == 0) {
			sensor_opened = true;
			first_open = true;
		} else
			sensor_opened = false;
		msleep(20);
	}
	return 0;
}

int iCatch7002a_sensor_release(struct inode *inode, struct file *file)
{
	printk("%s()++\n", __FUNCTION__);
	if (sensor_opened == true) {
		if (info->pdata && info->pdata->power_off && calibrating==0) {
			info->pdata->power_off();
			sensor_opened = false;
			capture_mode = false;
		}
	} else
		printk("%s No action. Power is already off.\n", __FUNCTION__);
	printk("%s and calibrating is %d\n", __FUNCTION__, calibrating);
	file->private_data = NULL;
#ifdef CAM_TWO_MODE
	g_initialized_1280_960=0;
	g_initialized_1080p=0;
#endif
	msleep(300);
	printk("%s()--\n", __FUNCTION__);
	return 0;
}


static const struct file_operations sensor_fileops = {
	.owner = THIS_MODULE,
	.open = sensor_open,
	.unlocked_ioctl = sensor_ioctl,
	.release = iCatch7002a_sensor_release,
};

static struct miscdevice sensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = ICATCH7002A_SENSOR_NAME,
	.fops = &sensor_fileops,
};

static ssize_t i7002a_switch_name(struct switch_dev *sdev, char *buf)
{
	printk("%s: version_num_in_isp=0x%X\n", __FUNCTION__, version_num_in_isp);

	if(tegra3_get_project_id() == TEGRA3_PROJECT_TF300T)
		return sprintf(buf, "TF300T-%06X\n", version_num_in_isp);
	else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG)
		return sprintf(buf, "TF300TG-%06X\n", version_num_in_isp);
	else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF300TL)
		return sprintf(buf, "TF300TL-%06X\n", version_num_in_isp);
	else if (tegra3_get_project_id() == TEGRA3_PROJECT_ME301T)
		return sprintf(buf, "ME301T-%06X\n", version_num_in_isp);
	else if (tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)
		return sprintf(buf, "ME301TL-%06X\n", version_num_in_isp);
	else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF500T)
		return sprintf(buf, "TF500T-%06X\n", version_num_in_isp);
	else
		return sprintf(buf, "Unknown-%06X\n", version_num_in_isp);
}

static ssize_t i7002a_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", fw_update_status);
}

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err=0;

	pr_info("yuv %s, compiled at %s %s\n",__func__,__DATE__,__TIME__);

	info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);

	if (!info) {
		pr_err("yuv_sensor : Unable to allocate memory!\n");
		return -ENOMEM;
	}

#ifndef _CAM_SENSOR_DETECT_
	err = misc_register(&sensor_device);
	if (err) {
		pr_err("yuv_sensor : Unable to register misc device!\n");
		kfree(info);
		return err;
	}
#endif
	info->pdata = client->dev.platform_data;
	info->i2c_client = client;
#ifdef _CAM_SENSOR_DETECT_
	info->dev = client->dev;
#endif

	i2c_set_clientdata(client, info);

	/* Query fw version number in ISP. */
	i7002a_isp_on(1);
	front_chip_id = i7002a_get_sensor_id(1);
	if (front_chip_id == SENSOR_ID_MI1040) {
		u16 tmp;
		sensor_write_table(info->i2c_client, query_mi1040_output_format_seq);
		sensor_read_reg(info->i2c_client, 0x9111, &tmp);
		mi1040_output_format = tmp;
		printk("mi1040 output format= %d\n", mi1040_output_format);
	}

	version_num_in_isp = get_fw_version_in_isp();

	i7002a_sdev.name = I7002A_SDEV_NAME;
	i7002a_sdev.print_name = i7002a_switch_name;
	i7002a_sdev.print_state = i7002a_switch_state;
	if(switch_dev_register(&i7002a_sdev) < 0){
		pr_err("switch_dev_register for camera failed!\n");
	}
	switch_set_state(&i7002a_sdev, 0);

	pr_info("i7002a front_chip_id: 0x%X\n", front_chip_id);
	pr_info("i7002a check version number: 0x%06X\n", version_num_in_isp);
	pr_info("i7002a fw_front_type_in_isp: 0x%02X\n", fw_front_type_in_isp);

	i7002a_isp_on(0);

	return 0;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_info *info;

	pr_info("yuv %s\n",__func__);
	info = i2c_get_clientdata(client);
	misc_deregister(&sensor_device);
	kfree(info);
	return 0;
}

static const struct i2c_device_id sensor_id[] = {
	{ ICATCH7002A_SENSOR_NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
	.driver = {
		.name = ICATCH7002A_SENSOR_NAME,
		.owner = THIS_MODULE,
	},
	.probe = sensor_probe,
	.remove = sensor_remove,
	.id_table = sensor_id,
};

#ifdef _CAM_SENSOR_DETECT_
int tegra_camera_set_caminfo(int num, int on);

int __init iCatch7002a_late_init(void)
{
	int ret=-EINVAL;
	u16 temp;
	struct clk *csi_clk=NULL;
	struct clk *csus_clk=NULL;
	struct clk *sensor_clk=NULL;
	struct regulator *Tegra_camera_regulator_csi=NULL;

	pr_err("%s: entry point\n", __func__);

	if (!info || !info->pdata || !info->pdata->power_on)
		goto fail;

	info->pdata->power_on();

	csi_clk = clk_get(NULL, "csi");
	if (IS_ERR_OR_NULL(csi_clk)) {
		pr_err("%s: Couldn't get csi clock\n", __func__);
		csi_clk=NULL;
		goto fail;
	}

	csus_clk = clk_get(NULL, "csus");
	if (IS_ERR_OR_NULL(csus_clk)) {
		pr_err("Couldn't get csus clock\n");
		csus_clk=NULL;
		goto fail;
	}
	sensor_clk = clk_get(NULL, "vi_sensor");
	if (IS_ERR_OR_NULL(sensor_clk)) {
		pr_err("Couldn't get csus clock\n");
		sensor_clk=NULL;
		goto fail;
	}

	msleep(10);
	clk_enable(csus_clk);
	clk_enable(sensor_clk);

	ret = sensor_read_reg(info->i2c_client, 0x300A, &temp);
	if (ret)
		ret = sensor_read_reg(info->i2c_client, 0x300A, &temp);

	if (ret)
		printk("failed to detect iCatch7002a ISP!\n");
	else {
		printk("read ID as 0x%x",temp);
		misc_register(&sensor_device);
		tegra_camera_set_caminfo(0,1);
	}
	clk_disable(csi_clk);
	clk_disable(csus_clk);
	clk_disable(sensor_clk);

	ret = 0;
fail:
	if(csus_clk)
		clk_put(csus_clk);
	if(csi_clk)
		clk_put(csi_clk);
	if(sensor_clk)
		clk_put(sensor_clk);
	if(Tegra_camera_regulator_csi)
		regulator_put(Tegra_camera_regulator_csi);

	info->pdata->power_off();

	return ret;
}

late_initcall(iCatch7002a_late_init);
#endif
static int __init sensor_init(void)
{
	if ((tegra3_get_project_id() == TEGRA3_PROJECT_TF300T) ||
			(tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG) ||
			(tegra3_get_project_id() == TEGRA3_PROJECT_TF300TL) ||
   			(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T)  ||
			(tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL) ||
			(tegra3_get_project_id() == TEGRA3_PROJECT_TF500T)) {
		pr_info("i7002a %s\n",__func__);
		return i2c_add_driver(&sensor_i2c_driver);
	}
	return 0;
}

static void __exit sensor_exit(void)
{
	if ((tegra3_get_project_id() == TEGRA3_PROJECT_TF300T) ||
			(tegra3_get_project_id() == TEGRA3_PROJECT_TF300TG) ||
			(tegra3_get_project_id() == TEGRA3_PROJECT_TF300TL) ||
   			(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T)  ||
			(tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL) ||
			(tegra3_get_project_id() == TEGRA3_PROJECT_TF500T)) {
		pr_info("i7002a %s\n",__func__);
		i2c_del_driver(&sensor_i2c_driver);
	}
}

module_init(sensor_init);
module_exit(sensor_exit);



#define CONFIG_I2C_READ_WRITE
#ifdef CONFIG_I2C_READ_WRITE
#include <linux/debugfs.h>
#include <linux/uaccess.h>
//#include <stdio.h>
#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];
//static u32 i2c_set_value;
static u32 i2c_get_value;

static ssize_t i2c_set_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t i2c_get_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_chip_power_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_SPI_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#if 1
static ssize_t dbg_i7002a_fw_in_isp_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_fw_in_isp_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	/* [Project id]-[FrontSensor]-[FW Version]*/
	if (front_chip_id == SENSOR_ID_OV2720) {
		len = snprintf(bp, dlen, "%02X-%02X-%06X-%02X\n", tegra3_get_project_id(), 1, version_num_in_isp, fw_front_type_in_isp);
		tot += len; bp += len; dlen -= len;
	} else if (front_chip_id == SENSOR_ID_MI1040){
		/* mi1040 chip_id= 0x2481 */
		len = snprintf(bp, dlen, "%02X-%02X-%06X-%02X\n", tegra3_get_project_id(), 2, version_num_in_isp, fw_front_type_in_isp);
		tot += len; bp += len; dlen -= len;
	} else {
		len = snprintf(bp, dlen, "%02X-%02X-%06X-%02X\n", tegra3_get_project_id(), 0, version_num_in_isp, fw_front_type_in_isp);
		tot += len; bp += len; dlen -= len;
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}
#endif

static ssize_t dbg_i7002a_page_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_page_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i =0;
	u8 mypage[0x100];

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	i7002a_isp_on(1);

	len = snprintf(bp, dlen, "page_index=%d (0x%X)\n", dbg_i7002a_page_index, dbg_i7002a_page_index);
	tot += len; bp += len; dlen -= len;

	get_one_page_from_i7002a(dbg_i7002a_page_index, mypage);
	for(i=0; i < 0x100; i++) {
		if(i%16 == 0) {
			len = snprintf(bp, dlen, "[%03X] ", i);
			tot += len; bp += len; dlen -= len;
		}
		len = snprintf(bp, dlen, "%02X ", mypage[i]);
		tot += len; bp += len; dlen -= len;

		if(i%16 == 15) {
			len = snprintf(bp, dlen, "\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	i7002a_isp_on(0);

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_i7002a_bin_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_i7002a_bin_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len, tot = 0;
	char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int ret = 0;
	char* mybin;
	struct file *fp_bin_dump = NULL;
	mm_segment_t old_fs;
	loff_t offset = 0;

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	i7002a_isp_on(1);

	//I2CDataWrite(0x70c4,0x00);
	//I2CDataWrite(0x70c5,0x00);
	sensor_write_reg(info->i2c_client, 0x70c4,0x00);
	sensor_write_reg(info->i2c_client, 0x70c5,0x00);

	ret = I2C_SPIInit();
	if (ret) {
		printk("%s: get nothing. ret= %d", __FUNCTION__, ret);
		return -EFAULT;;
	}

	I2C_SPIFlashReadId();

	if(dbg_i7002a_force_fw_size == 512) {
		mybin = kmalloc(512*1024, GFP_KERNEL);
		I2C_SPIFlashRead_DMA(0, 2048, mybin);
	} else if(dbg_i7002a_force_fw_size == 1024) {
		mybin = kmalloc(1024*1024, GFP_KERNEL);
		I2C_SPIFlashRead_DMA(0, 4096, mybin);
	} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF500T ||
		tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
		tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL) {
		mybin = kmalloc(1024*1024, GFP_KERNEL);
		I2C_SPIFlashRead_DMA(0, 4096, mybin);
	}
	else {
		mybin = kmalloc(512*1024, GFP_KERNEL);
		I2C_SPIFlashRead_DMA(0, 2048, mybin);
	}

	i7002a_isp_on(0);

	/* Dump to /data/bin_dump.bin */
	fp_bin_dump = filp_open("/data/bin_dump.bin", O_RDWR | O_CREAT | O_TRUNC, S_IRUGO | S_IWUGO);
	if ( IS_ERR_OR_NULL(fp_bin_dump) ){
		filp_close(fp_bin_dump, NULL);
		len = snprintf(bp, dlen, "%s: open %s fail\n", __FUNCTION__, "/data/bin_dump.bin");
		tot += len; bp += len; dlen -= len;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	offset = 0;

	if (fp_bin_dump->f_op != NULL && fp_bin_dump->f_op->write != NULL){
		if(dbg_i7002a_force_fw_size == 512) {
			fp_bin_dump->f_op->write(fp_bin_dump, mybin, 512*1024, &offset);
		} else if(dbg_i7002a_force_fw_size == 1024) {
			fp_bin_dump->f_op->write(fp_bin_dump, mybin, 1024*1024, &offset);
		} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF500T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
			tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL)
			fp_bin_dump->f_op->write(fp_bin_dump, mybin, 1024*1024, &offset);
		else
			fp_bin_dump->f_op->write(fp_bin_dump, mybin, 512*1024, &offset);

	}
	else {
		len = snprintf(bp, dlen, "%s: f_op might be null\n", __FUNCTION__);
		tot += len; bp += len; dlen -= len;
	}
	set_fs(old_fs);
	filp_close(fp_bin_dump, NULL);
	kfree(mybin);

	len = snprintf(bp, dlen, "%s: Dump Complete.\n", __FUNCTION__);
	tot += len; bp += len; dlen -= len;

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_i7002a_fw_header_dump_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

#define DUMP_HEADER(mypage) do {	\
		for(i = 0; i < 0x100; i++) {	\
			if(i%16 == 0) {	\
				len = snprintf(bp, dlen, "[%02X] ", i);	\
				tot += len; bp += len; dlen -= len;	\
			}	\
			len = snprintf(bp, dlen, "%02X ", mypage[i]);	\
			tot += len; bp += len; dlen -= len;	\
			if(i%16 == 15) {	\
				len = snprintf(bp, dlen, "\n");	\
				tot += len; bp += len; dlen -= len;	\
			}	\
		}	\
	} while (0)


static ssize_t dbg_i7002a_fw_header_dump_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[3072];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;
	int i =0;
	u8 fw1page[0x100];
	u8 fw2page[0x100];
	u8 overallpage[0x100];
	int fw2_header_page_index, fw2_offset = 0;

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	i7002a_isp_on(1);

	/* dump fw1 header */
	get_one_page_from_i7002a(0, fw1page);
	len = snprintf(bp, dlen, "fw1: page[%d]:\n", 0);
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(fw1page);

	msleep(40);

	/* dump fw2 header */
	fw2_offset = 16 +
		((fw1page[3] << 24) | (fw1page[2] << 16) | (fw1page[1] << 8) | fw1page[0]) +
		((fw1page[7] << 24) | (fw1page[6] << 16) | (fw1page[5] << 8) | fw1page[4]);
	fw2_header_page_index = fw2_offset >> 8;
	get_one_page_from_i7002a(fw2_header_page_index, fw2page);
	len = snprintf(bp, dlen, "fw2: page[%d]:\n", fw2_header_page_index);
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(fw2page);

	msleep(40);

	/* dump overall header */
	if(dbg_i7002a_force_fw_size == 512) {
		get_one_page_from_i7002a(2047, overallpage);
		len = snprintf(bp, dlen, "Overall: page[%d]:\n", 2047);
	} else if(dbg_i7002a_force_fw_size == 1024) {
		get_one_page_from_i7002a(4095, overallpage);
		len = snprintf(bp, dlen, "Overall: page[%d]:\n", 4095);
	} else if (tegra3_get_project_id() == TEGRA3_PROJECT_TF500T ||
		tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
		tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL) {
		get_one_page_from_i7002a(4095, overallpage);
		len = snprintf(bp, dlen, "Overall: page[%d]:\n", 4095);
	}
	else {
		get_one_page_from_i7002a(2047, overallpage);
		len = snprintf(bp, dlen, "Overall: page[%d]:\n", 2047);
	}
	tot += len; bp += len; dlen -= len;
	DUMP_HEADER(overallpage);

	i7002a_isp_on(0);

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_fw_update_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_fw_update_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[512];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	switch(fw_update_status) {
	case ICATCH_FW_NO_CMD:
		len = snprintf(bp, dlen, "Never issue fw update cmd yet.\n");
		tot += len; bp += len; dlen -= len;
		break;

	case ICATCH_FW_IS_BURNING:
		if ((page_count >= 0) && (page_count <= total_page_count)) {
			int time_left = 0;
			if (flash_type == ICATCH_FLASH_TYPE_ST)
				time_left = page_count * 8 / 100;
			else
				time_left = page_count / 4;

			len = snprintf(bp, dlen, "FW update progress: %d/%d; Timeleft= %d secs\n", total_page_count - page_count + 1, total_page_count, time_left);
			tot += len; bp += len; dlen -= len;
		} else {
			len = snprintf(bp, dlen, "page_count=%d; total=%d\n", page_count, total_page_count);
			tot += len; bp += len; dlen -= len;
		}
		break;

	case ICATCH_FW_UPDATE_SUCCESS:
		len = snprintf(bp, dlen, "FW Update Complete!\n");
		tot += len; bp += len; dlen -= len;
		break;

	case ICATCH_FW_UPDATE_FAILED:
		len = snprintf(bp, dlen, "FW Update FAIL!\n");
		tot += len; bp += len; dlen -= len;
		break;

	default:
		len = snprintf(bp, dlen, "FW Update Status Unknown: %d\n", fw_update_status);
		tot += len; bp += len; dlen -= len;
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static int dbg_fw_update_write(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	char debug_buf[256];
	int cnt;
	char bin_path[80];

	printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
	if (count > sizeof(debug_buf))
		return -EFAULT;
	if (copy_from_user(debug_buf, buf, count))
		return -EFAULT;

	debug_buf[count] = '\0';	/* end of string */
	cnt = sscanf(debug_buf, "%s", bin_path);

	/* Turn on the power & clock. */
	i7002a_isp_on(1);

	/* burning */
	printk("%s: BB_WrSPIFlash()++\n", __FUNCTION__);
	BB_WrSPIFlash(bin_path);
	printk("%s: BB_WrSPIFlash()--\n", __FUNCTION__);

	/* Turn off the clock & power. */
	i7002a_isp_on(0);

	return count;
}

static int i2c_set_write(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len;
	int arg[2];
	//int gpio, set;

	//char gpioname[8];

	// printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
	arg[0]=0;

	if (*ppos)
		return 0;	/* the end */

	//+ parsing......
	len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf,buf,len))
		return -EFAULT;

	debugTxtBuf[len]=0; //add string end

	sscanf(debugTxtBuf, "%x %x", &arg[0], &arg[1]);
	printk("argument is arg1=0x%x arg2=0x%x\n",arg[0], arg[1]);


	*ppos=len;
	sensor_write_reg(info->i2c_client, arg[0], arg[1]);

	return len;	/* the end */
}

static int i2c_get_write(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len;
	int arg = 0;
	//int gpio, set;
	//char gpioname[8];
	// printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);

	if (*ppos)
		return 0;	/* the end */

	//+ parsing......
	len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf,buf,len))
		return -EFAULT;

	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%x", &arg);
	printk("argument is arg=0x%x\n",arg);

	*ppos=len;
	sensor_read_reg(info->i2c_client, arg, &i2c_get_value);

	return len;	/* the end */
}

static ssize_t i2c_get_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	char *bp = debugTxtBuf;

	if (*ppos)
		return 0;	/* the end */
	len = snprintf(bp, DBG_TXT_BUF_SIZE, "the value is 0x%x\n", i2c_get_value);

	if (copy_to_user(buf, debugTxtBuf, len))
		return -EFAULT;
	*ppos += len;
	return len;
}

static ssize_t dbg_iCatch7002a_vga_status_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_vga_status_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[1024];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u16 chip_id, tmp = 0x0;

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	if (sensor_opened == false) {
		if (info->pdata && info->pdata->power_on) {
			info->pdata->power_on();
			tegra_camera_mclk_on_off(1);
			msleep(100);
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_on.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	/*Start - Power on sensor & enable clock - Front I2C (OV2720)*/
	sensor_write_reg(info->i2c_client, 0x0084, 0x14); /* To sensor clock divider */
	sensor_write_reg(info->i2c_client, 0x0034, 0xFF); /* Turn on all clock */
	sensor_write_reg(info->i2c_client, 0x9030, 0x3f);
	sensor_write_reg(info->i2c_client, 0x9031, 0x04);
	sensor_write_reg(info->i2c_client, 0x9034, 0xf3);
	sensor_write_reg(info->i2c_client, 0x9035, 0x04);

	sensor_write_reg(info->i2c_client, 0x9032, 0x02);
	msleep(10);
	sensor_write_reg(info->i2c_client, 0x9032, 0x00);
	msleep(10);
	sensor_write_reg(info->i2c_client, 0x9033, 0x00);
	msleep(10);
	sensor_write_reg(info->i2c_client, 0x9033, 0x04);
	msleep(10);
	sensor_write_reg(info->i2c_client, 0x9034, 0xf2);
	/*End - Power on sensor & enable clock */

	sensor_write_reg(info->i2c_client, 0x9008, 0x00); /* Need to check with vincent */
	sensor_write_reg(info->i2c_client, 0x9009, 0x00);
	sensor_write_reg(info->i2c_client, 0x900A, 0x00);
	sensor_write_reg(info->i2c_client, 0x900B, 0x00);

	/*Start - I2C Read*/
	sensor_write_reg(info->i2c_client, 0x9138, 0x30); /* Sub address enable */
	sensor_write_reg(info->i2c_client, 0x9140, 0x6C); /* Slave address      */
	sensor_write_reg(info->i2c_client, 0x9100, 0x03); /* Read mode          */
	sensor_write_reg(info->i2c_client, 0x9110, 0x30); /* Register addr MSB  */
	sensor_write_reg(info->i2c_client, 0x9112, 0x0a); /* Register addr LSB  */
	sensor_write_reg(info->i2c_client, 0x9104, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(info->i2c_client, 0x9111, &tmp);

	//printk("0x%x\n", tmp);
	chip_id = (tmp << 8) & 0xFF00;

	sensor_write_reg(info->i2c_client, 0x9110, 0x30); /* Register addr MSB  */
	sensor_write_reg(info->i2c_client, 0x9112, 0x0b); /* Register addr LSB  */
	sensor_write_reg(info->i2c_client, 0x9104, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(info->i2c_client, 0x9111, &tmp);
	//printk("0x%x\n", tmp);
	chip_id = chip_id | (tmp & 0xFF);

	if (chip_id == SENSOR_ID_OV2720) {
		len = snprintf(bp, dlen, "1\n");
		tot += len; bp += len; dlen -= len;
	} else {
#if 0
		len = snprintf(bp, dlen, "back chip_id= 0x%x\n", chip_id);
		tot += len; bp += len; dlen -= len;
#endif
		/* Check if mi1040 is available. */
		sensor_write_table(info->i2c_client, query_mi1040_id_msb_seq);
		sensor_read_reg(info->i2c_client, 0x9111, &tmp);

		chip_id = (tmp << 8) & 0xFF00;

		sensor_write_table(info->i2c_client, query_mi1040_id_lsb_seq);
		sensor_read_reg(info->i2c_client, 0x9111, &tmp);
		chip_id = chip_id | (tmp & 0xFF);

		printk("0x%x\n", chip_id);

		if (chip_id == SENSOR_ID_MI1040) {
			/* mi1040 chip_id= 0x2481 */
			len = snprintf(bp, dlen, "1\n");
			tot += len; bp += len; dlen -= len;
		} else {
			len = snprintf(bp, dlen, "0\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (sensor_opened == false) {
		if (info->pdata && info->pdata->power_off) {
			tegra_camera_mclk_on_off(0);
			info->pdata->power_off();
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_off.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static ssize_t dbg_iCatch7002a_camera_status_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

static ssize_t dbg_iCatch7002a_camera_status_read(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len = 0;
	int tot = 0;
	char debug_buf[256];
	int dlen = sizeof(debug_buf);
	char *bp = debug_buf;

	u16 chip_id, tmp = 0x0;

	printk("%s: buf=%p, count=%d, ppos=%p; *ppos= %d\n", __FUNCTION__, buf, count, ppos, *ppos);

	if (*ppos)
		return 0;	/* the end */

	if (sensor_opened == false) {
		if (info->pdata && info->pdata->power_on) {
			info->pdata->power_on();
			tegra_camera_mclk_on_off(1);
			msleep(100);
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_on.\n");
			tot += len; bp += len; dlen -= len;
		}
	}
	/* SONY IMX175 */
	sensor_write_reg(info->i2c_client, 0x0084, 0x14); /* To sensor clock divider */
	sensor_write_reg(info->i2c_client, 0x0034, 0xFF); /* Turn on all clock */
	sensor_write_reg(info->i2c_client, 0x9030, 0x3f);
	sensor_write_reg(info->i2c_client, 0x9031, 0x04);
	sensor_write_reg(info->i2c_client, 0x9034, 0xf2);
	sensor_write_reg(info->i2c_client, 0x9035, 0x04);
	if( tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
	    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL ){
		sensor_write_reg(info->i2c_client, 0x9032, 0x10);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9032, 0x00);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9032, 0x20);
		msleep(10);
	}
	else{
		sensor_write_reg(info->i2c_client, 0x9032, 0x00);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9032, 0x20);
		msleep(10);
		sensor_write_reg(info->i2c_client, 0x9032, 0x30);
		msleep(10);
    }
	/*End - Power on sensor & enable clock */
	sensor_write_reg(info->i2c_client, 0x9008, 0x00); /* Need to check with vincent */
	sensor_write_reg(info->i2c_client, 0x9009, 0x00);
	sensor_write_reg(info->i2c_client, 0x900A, 0x00);
	sensor_write_reg(info->i2c_client, 0x900B, 0x00);

	/*Start - I2C Read*/
	sensor_write_reg(info->i2c_client, 0x9238, 0x30); /* Sub address enable */
	if( tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
	    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL )
		sensor_write_reg(info->i2c_client, 0x9240, 0x6C); /* Slave address      */
	else
		sensor_write_reg(info->i2c_client, 0x9240, 0x20); /* Slave address      */
	sensor_write_reg(info->i2c_client, 0x9200, 0x03); /* Read mode          */
	if( tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
	    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL ){
		sensor_write_reg(info->i2c_client, 0x9210, 0x30); /* Register addr MSB  */
		sensor_write_reg(info->i2c_client, 0x9212, 0x0A); /* Register addr LSB  */
	}
	else{
		sensor_write_reg(info->i2c_client, 0x9210, 0x00); /* Register addr MSB  */
		sensor_write_reg(info->i2c_client, 0x9212, 0x00); /* Register addr LSB  */
	}
	sensor_write_reg(info->i2c_client, 0x9204, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(info->i2c_client, 0x9211, &tmp);
	// printk("0x%x\n", tmp);
	chip_id = (tmp << 8) & 0xFF00;

	if( tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
	    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL ){
		sensor_write_reg(info->i2c_client, 0x9210, 0x30); /* Register addr MSB  */
		sensor_write_reg(info->i2c_client, 0x9212, 0x0B); /* Register addr LSB  */
	}
    else{
		sensor_write_reg(info->i2c_client, 0x9210, 0x00); /* Register addr MSB  */
		sensor_write_reg(info->i2c_client, 0x9212, 0x01); /* Register addr LSB  */
    }
	sensor_write_reg(info->i2c_client, 0x9204, 0x01); /* Trigger I2C read   */

	msleep(10);
	sensor_read_reg(info->i2c_client, 0x9211, &tmp);
	// printk("0x%x\n", tmp);
	chip_id = chip_id | (tmp & 0xFF);

	if( tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
	    tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL ){
		if (chip_id == SENSOR_ID_OV5650 || chip_id == SENSOR_ID_OV5651) {
			len = snprintf(bp, dlen, "1\n");
			tot += len; bp += len; dlen -= len;
		} else {
#if 0
			len = snprintf(bp, dlen, "back chip_id= 0x%x\n", chip_id);
			tot += len; bp += len; dlen -= len;
#endif
			len = snprintf(bp, dlen, "0\n");
			tot += len; bp += len; dlen -= len;
		}
	}
	else{
		if (chip_id == SENSOR_ID_IMX175) {
			len = snprintf(bp, dlen, "1\n");
			tot += len; bp += len; dlen -= len;
		} else {
#if 0
			len = snprintf(bp, dlen, "back chip_id= 0x%x\n", chip_id);
			tot += len; bp += len; dlen -= len;
#endif
			len = snprintf(bp, dlen, "0\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (sensor_opened == false) {
		if (info->pdata && info->pdata->power_off) {
			tegra_camera_mclk_on_off(0);
			info->pdata->power_off();
		} else {
			len = snprintf(bp, dlen, "iCatch7002a info isn't enough for power_off.\n");
			tot += len; bp += len; dlen -= len;
		}
	}

	if (copy_to_user(buf, debug_buf, tot))
		return -EFAULT;
	if (tot < 0)
		return 0;
	*ppos += tot;	/* increase offset */
	return tot;
}

static int dbg_iCatch7002a_chip_power_write(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
	int len;
	int arg;
	//int gpio, set;
	//char gpioname[8];
	// printk("%s: buf=%p, count=%d, ppos=%p\n", __FUNCTION__, buf, count, ppos);
	arg=0;

	if (*ppos)
		return 0;	/* the end */

	//+ parsing......
	len=(count > DBG_TXT_BUF_SIZE-1)?(DBG_TXT_BUF_SIZE-1):(count);
	if (copy_from_user(debugTxtBuf,buf,len))
		return -EFAULT;

	debugTxtBuf[len]=0; //add string end
	sscanf(debugTxtBuf, "%x", &arg);
	printk("argument is arg=0x%x\n",arg);

	*ppos=len;
	//sensor_write_reg(info->i2c_client, arg[0], arg[1]);
	if (arg==0) {
		//power off
		if (sensor_opened==true) {
			if (info->pdata && info->pdata->power_off) {
				tegra_camera_mclk_on_off(0);
				info->pdata->power_off();
				sensor_opened=false;
				printk("%s:power off\n", __func__);
			}
		}
	}
	if (arg==1) {
		//power on
		if (sensor_opened==false) {
			if (info->pdata && info->pdata->power_on) {
				tegra_camera_mclk_on_off(1);
				info->pdata->power_on();
				sensor_opened=true;
				msleep(100);
				printk("%s:power on\n", __func__);
			}
		}
		//msleep(10);
		//tegra_camera_mclk_on_off(1);
	}

	return len;	/* the end */
}

static int dbg_iCatch7002a_read_SPI_for_cal(u8 index)
{
    u8 *pTempBuf;
    u32 spiId, spiType;
    u32 spiSize, sectorSize = 0x1000, pageSize = 0x100;
    u32 resStartAddr, resEndAddr;
    u32 resCALIB_3ACALI_SIZE = 520, resCALIB_LSC_SIZE, resCALIB_LSCDQ_SIZE;
    u32 resOffset = 0x5000, resSize;
    u32 startPage, finalPage, pageCount;
    int i;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    loff_t offset = 0;

    printk("read_SPI");

    sensor_write_reg(info->i2c_client, 0x1011,0x01); /* CPU SW RESET */
    sensor_write_reg(info->i2c_client, 0x001C,0x08); /* reset FM register */
    sensor_write_reg(info->i2c_client, 0x001C,0x00);
    sensor_write_reg(info->i2c_client, 0x108C,0x00); /* DMA select*/
    sensor_write_reg(info->i2c_client, 0x009A,0x00); /* make sure CPU normal operation*/

    I2C_SPIInit();

    spiId = I2C_SPIFlashReadId();
    if( spiId == 0 )
    {
        printk("read id failed\n");
        return -ENOMEM;;
    }
    /*printf("spiSize:0x%x\n",&spiSize);*/
    spiType = BB_SerialFlashTypeCheck(spiId, &spiSize);

    if(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
       tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL ){
        resCALIB_LSC_SIZE = 1520;
        resCALIB_LSCDQ_SIZE = 5626;
    }
    else{
        resCALIB_LSC_SIZE = 2292;
        resCALIB_LSCDQ_SIZE = 8521;
    }

    if (index ==1){
        resStartAddr = spiSize - resOffset;
        resEndAddr = resStartAddr + resCALIB_3ACALI_SIZE;
        resSize = resCALIB_3ACALI_SIZE;
    }
    else if (index==2){
        resCALIB_3ACALI_SIZE += 8;
        resStartAddr = spiSize - resOffset + resCALIB_3ACALI_SIZE;
        resEndAddr = resStartAddr + resCALIB_LSC_SIZE;
        resSize = resCALIB_LSC_SIZE;
    }
    else if (index==3){
        resCALIB_3ACALI_SIZE += 8;
        resCALIB_LSC_SIZE = ((resCALIB_LSC_SIZE+15)>>4)<<4;
        resStartAddr = spiSize -resOffset + resCALIB_3ACALI_SIZE + resCALIB_LSC_SIZE;
        resEndAddr = resStartAddr + resCALIB_LSCDQ_SIZE;
        resSize = resCALIB_LSCDQ_SIZE;;
    }

    printk("read_SPI resStartAddr:0x%x resEndAddr:0x%x resSize:0x%x\n", resStartAddr, resEndAddr, resSize);

    startPage = resStartAddr/pageSize;
    finalPage = resEndAddr/pageSize+1;
    pageCount = finalPage - startPage;

    pTempBuf = kmalloc(pageCount * pageSize , GFP_KERNEL);
    I2C_SPIFlashRead_DMA(startPage, pageCount, pTempBuf);

    /* Dump to /data/SPI_XXX.txt */
    if (index ==1)
        fp = filp_open("/data/SPI_3ACALI.txt", O_RDWR | O_CREAT | O_TRUNC, S_IRUGO | S_IWUGO);
    else if (index==2)
        fp = filp_open("/data/SPI_LSC.txt", O_RDWR | O_CREAT | O_TRUNC, S_IRUGO | S_IWUGO);
    else if (index==3)
        fp = filp_open("/data/SPI_LSC_DQ.txt", O_RDWR | O_CREAT | O_TRUNC, S_IRUGO | S_IWUGO);

    if ( IS_ERR_OR_NULL(fp) )
        filp_close(fp, NULL);

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    offset = 0;

    if (fp->f_op != NULL && fp->f_op->write != NULL)
        fp->f_op->write(fp, pTempBuf + (resStartAddr % pageSize), resSize, &offset);
    else
        printk("%s: data/SPI_XXX.txt: f_op might be null\n", __FUNCTION__);

    set_fs(old_fs);
    filp_close(fp, NULL);
    kfree(pTempBuf);

    printk("%s: Dump Complete.\n", __FUNCTION__);

    return 0;
}

static int dbg_iCatch7002a_write_SPI_for_cal(u8 index)
{
    u8 *pTempBuf, *pBuf;
    u32 file_size, residue;
    u32 resStartAddr, resEndAddr;
    u32 resCALIB_3ACALI_SIZE = 520, resCALIB_LSC_SIZE, resCALIB_LSCDQ_SIZE;
    u32 resOffset = 0x5000;
    u32 spiId, spiType;
    u32 spiSize, sectorSize = 0x1000, pageSize = 0x100;
    u32 startSector, finalSector, sectorCount;
    mm_segment_t old_fs;
    struct file *fp = NULL;
    struct inode *inode;
    int i;

    pr_info("write_SPI");

    sensor_write_reg(info->i2c_client, 0x1011,0x01); /* CPU SW RESET */
    sensor_write_reg(info->i2c_client, 0x001C,0x08); /* reset FM register */
    sensor_write_reg(info->i2c_client, 0x001C,0x00);
    sensor_write_reg(info->i2c_client, 0x108C,0x00); /* DMA select*/
    sensor_write_reg(info->i2c_client, 0x009A,0x00); /* make sure CPU normal operation*/

    I2C_SPIInit();

    spiId = I2C_SPIFlashReadId();
    if( spiId == 0 )
    {
        printk("read id failed\n");
        return -ENOMEM;;
    }
    /*printf("spiSize:0x%x\n",&spiSize);*/
    spiType = BB_SerialFlashTypeCheck(spiId, &spiSize);

    if(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T ||
       tegra3_get_project_id() == TEGRA3_PROJECT_ME301TL ){
        resCALIB_LSC_SIZE = 1520;
        resCALIB_LSCDQ_SIZE = 5626;
    }
    else{
        resCALIB_LSC_SIZE = 2292;
        resCALIB_LSCDQ_SIZE = 8521;
    }

    if (index==1) {
        fp = filp_open("/data/3ACALI.BIN", O_RDONLY, 0);
        resStartAddr = spiSize - resOffset;
        resEndAddr = resStartAddr + resCALIB_3ACALI_SIZE;
    }
    if (index==2) {
        fp = filp_open("/data/LSC.BIN", O_RDONLY, 0);
        resCALIB_3ACALI_SIZE += 8;
        resStartAddr = spiSize - resOffset + resCALIB_3ACALI_SIZE;
        resEndAddr = resStartAddr + resCALIB_LSC_SIZE;
    }
    if (index==3) {
        fp = filp_open("/data/LSC_DQ.BIN", O_RDONLY, 0);
        resCALIB_3ACALI_SIZE += 8;
        resCALIB_LSC_SIZE = ((resCALIB_LSC_SIZE+15)>>4)<<4;
        resStartAddr = spiSize -resOffset + resCALIB_3ACALI_SIZE + resCALIB_LSC_SIZE;
        resEndAddr = resStartAddr + resCALIB_LSCDQ_SIZE;
    }

    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        inode = fp->f_dentry->d_inode;
        file_size = inode->i_size;
        pTempBuf = kmalloc(file_size, GFP_KERNEL);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            int byte_count= 0;
            byte_count = fp->f_op->read(fp, pTempBuf, file_size, &fp->f_pos);

            if (byte_count <= 0) {
                printk("iCatch: EOF or error. last byte_count= %d;\n", byte_count);
                kfree(pTempBuf);
                return;
            } else
                printk("iCatch: BIN file size= %d bytes\n", file_size);
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
    }
    else if(PTR_ERR(fp) == -ENOENT){
        pr_err("file not found error\n");
        return -ENOMEM;
    }
    else{
        pr_err("file open error\n");
        return -ENOMEM;
    }

    startSector = (resStartAddr/sectorSize)*sectorSize;
    finalSector = (resEndAddr/sectorSize)*sectorSize;
    sectorCount = ((finalSector - startSector)/sectorSize) + 1;
    residue = resStartAddr - startSector;

    printk("start:0x%x final:0x%x Count:0x%x residue:0x%x file_size0X%x\n", startSector, finalSector, sectorCount, residue, file_size);

    pBuf = kmalloc(sectorCount * sectorSize, GFP_KERNEL);

    I2C_SPIFlashRead_DMA(startSector/pageSize, (sectorCount*sectorSize)/pageSize, pBuf);

    memcpy((pBuf+residue), pTempBuf, file_size);

    if(spiType == 2)
    {
        for(i = 0; i < sectorCount; i++)
        {
            I2C_SPISectorErase((startSector+(i*sectorSize)),0);
        }
    }
    else if(spiType == 1 || spiType == 3)
    {
        for(i = 0; i < sectorCount; i++)
        {
            I2C_SPISectorErase((startSector+(i*sectorSize)),1);
        }
    }
    msleep(100);

    I2C_SPIFlashWrite_DMA(startSector/pageSize, (sectorCount*sectorSize)/pageSize, pBuf);
    kfree(pTempBuf);
    kfree(pBuf);

    return 0;
}
static int dbg_iCatch7002a_read_write_SPI_for_cal(struct file *file, char __user *buf, size_t count,
				loff_t *ppos)
{
    char *bp = debugTxtBuf;
    int len, rw, index;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%d %d", &rw, &index);

    pr_info("command is rw=%d cmd=%d\n", rw, index);

    *ppos = len;

    if(rw){
        dbg_iCatch7002a_write_SPI_for_cal(index);
    }
    else{
        dbg_iCatch7002a_read_SPI_for_cal(index);
    }
    return len;	/* the end */
}

static const struct file_operations dbg_i7002a_fw_in_isp_fops = {
	.open		= dbg_i7002a_fw_in_isp_open,
	.read		= dbg_i7002a_fw_in_isp_read,
};

static const struct file_operations dbg_i7002a_page_dump_fops = {
	.open		= dbg_i7002a_page_dump_open,
	.read		= dbg_i7002a_page_dump_read,
};

static const struct file_operations dbg_i7002a_bin_dump_fops = {
	.open		= dbg_i7002a_bin_dump_open,
	.read		= dbg_i7002a_bin_dump_read,
};

static const struct file_operations dbg_fw_update_fops = {
	.open		= dbg_fw_update_open,
	.read		= dbg_fw_update_read,
	.write = dbg_fw_update_write,
};

static const struct file_operations i2c_set_fops = {
	.open		= i2c_set_open,
	//.read		= i2c_config_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = i2c_set_write,
};

static const struct file_operations i2c_get_fops = {
	.open		= i2c_get_open,
	.read		= i2c_get_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = i2c_get_write,
};

static const struct file_operations dbg_iCatch7002a_vga_status_fops = {
	.open		= dbg_iCatch7002a_vga_status_open,
	.read		= dbg_iCatch7002a_vga_status_read,
};

static const struct file_operations dbg_iCatch7002a_camera_status_fops = {
	.open		= dbg_iCatch7002a_camera_status_open,
	.read		= dbg_iCatch7002a_camera_status_read,
};

static const struct file_operations dbg_i7002a_fw_header_dump_fops = {
	.open		= dbg_i7002a_fw_header_dump_open,
	.read		= dbg_i7002a_fw_header_dump_read,
};

static const struct file_operations iCatch7002a_power_fops = {
	.open		= dbg_iCatch7002a_chip_power_open,
	//.read		= i2c_get_read,
	//.llseek		= seq_lseek,
	//.release	= single_release,
	.write = dbg_iCatch7002a_chip_power_write,
};

static const struct file_operations iCatch7002a_read_write_spi_for_cal_fops = {
	.open		= dbg_iCatch7002a_SPI_open,
	.write = dbg_iCatch7002a_read_write_SPI_for_cal,
};

static int __init tegra_i2c_debuginit(void)
{
	struct dentry *dent = debugfs_create_dir("i7002a", NULL);

	(void) debugfs_create_file("fw_in_isp", S_IRUGO | S_IWUSR,
					dent, NULL, &dbg_i7002a_fw_in_isp_fops);

	(void) debugfs_create_file("page_dump", S_IRUGO | S_IWUSR,
					dent, NULL, &dbg_i7002a_page_dump_fops);

	(void) debugfs_create_file("bin_dump", S_IRUGO | S_IWUSR,
					dent, NULL, &dbg_i7002a_bin_dump_fops);

	(void) debugfs_create_file("fw_header_dump", S_IRUGO | S_IWUSR,
					dent, NULL, &dbg_i7002a_fw_header_dump_fops);

	(void) debugfs_create_file("fw_update", S_IRUGO | S_IWUSR,
					dent, NULL, &dbg_fw_update_fops);

	(void) debugfs_create_file("i2c_set", S_IRUGO | S_IWUSR,
					dent, NULL, &i2c_set_fops);

	(void) debugfs_create_file("i2c_get", S_IRUGO | S_IWUSR,
					dent, NULL, &i2c_get_fops);
	(void) debugfs_create_file("camera_status", S_IRUGO, dent, NULL, &dbg_iCatch7002a_camera_status_fops);
	(void) debugfs_create_file("vga_status", S_IRUGO, dent, NULL, &dbg_iCatch7002a_vga_status_fops);
	(void) debugfs_create_file("iCatch_chip_power", S_IRUGO | S_IWUSR, dent, NULL, &iCatch7002a_power_fops);
	(void) debugfs_create_file("iCatch_rw_spi_for_cal", S_IRUGO | S_IWUSR, dent, NULL, &iCatch7002a_read_write_spi_for_cal_fops);

#ifdef ICATCH7002A_DELAY_TEST
	if (debugfs_create_u32("iCatch7002a_delay", S_IRUGO | S_IWUSR, dent, &iCatch7002a_init_delay)
		== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}
	if (debugfs_create_u32("iCatch7002a_preview_delay", S_IRUGO | S_IWUSR, dent, &iCatch7002a_preview_delay)
		== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}
	if (debugfs_create_u32("touch_focus_enable", S_IRUGO | S_IWUSR, dent, &touch_focus_enable)
		== NULL) {
		printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
			__FILE__, __LINE__);
		return -1;
	}
	if (factory_mode==2) {
		if (debugfs_create_u32("is_calibration", 0777, dent, &is_calibration)
			== NULL) {
			printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
				__FILE__, __LINE__);
			return -1;
		}
		if (debugfs_create_u32("calibrating", 0777, dent, &calibrating)
			== NULL) {
			printk(KERN_ERR "%s(%d): debugfs_create_u32: debug fail\n",
				__FILE__, __LINE__);
			return -1;
		}
	}
#endif
	debugfs_create_u32("page_index",S_IRUGO | S_IWUSR, dent, &dbg_i7002a_page_index);
	debugfs_create_u32("force_fw_size",S_IRUGO | S_IWUSR, dent, &dbg_i7002a_force_fw_size);
#ifdef CAM_TWO_MODE
	debugfs_create_u32("div",S_IRUGO | S_IWUSR, dent, &g_div);
	return 0;
#endif
}

late_initcall(tegra_i2c_debuginit);
#endif


