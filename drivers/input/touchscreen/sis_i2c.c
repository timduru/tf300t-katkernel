/* drivers/input/touchscreen/sis_i2c.c - I2C Touch panel driver for SiS 9200 family
 *
 * Copyright (C) 2011 SiS, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Date: 2012/05/30
 * Version:	Android_v2.00.01-A639-0530
 */

#include <linux/module.h>
#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include "sis_i2c.h"
#include <linux/linkage.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <asm/uaccess.h>
#include <linux/irq.h>
#include <../arch/arm/mach-tegra/gpio-names.h>
#include <linux/switch.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#define GPIO_IRQ TEGRA_GPIO_PH4

#ifdef _STD_RW_IO
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <../arch/arm/mach-tegra/board-cardhu.h>
#include <mach/board-cardhu-misc.h>

#define DEVICE_NAME "sis_aegis_touch_device"
static int sis_char_devs_count = 1;        /* device count */
static int sis_char_major = 0;
static struct cdev sis_char_cdev;
static struct class *sis_char_class = NULL;
#endif

/* Addresses to scan */
static const unsigned short normal_i2c[] = { SIS_SLAVE_ADDR, I2C_CLIENT_END };
static struct workqueue_struct *sis_wq;
struct sis_ts_data *ts_bak = 0;
struct sisTP_driver_data *TPInfo = NULL;
static struct switch_dev sis_touch_sdev;
static int sis_i2c_status = 0;
static atomic_t touch_char_available = ATOMIC_INIT(1);
static unsigned int project_id=0;


#define TOUCH_PMIC_5V_POWER TPS6591X_GPIO_8
#define TOUCH_STRESS_TEST 1
#ifdef TOUCH_STRESS_TEST
#define STRESS_IOC_MAGIC 0xF3
#define STRESS_IOC_MAXNR 4
#define STRESS_POLL_DATA _IOR(STRESS_IOC_MAGIC,2,int )
#define STRESS_IOCTL_START_HEAVY 2
#define STRESS_IOCTL_START_NORMAL 1
#define STRESS_IOCTL_END 0
#define START_NORMAL	(HZ/5)
#define START_HEAVY	(HZ/200)
static int poll_mode=0;
static struct delayed_work sis_poll_data_work;
static struct workqueue_struct *stress_work_queue;
static atomic_t touch_stress_available = ATOMIC_INIT(1);
struct miscdevice  stress_misc_dev;
#endif

#define _ENABLE_DBG_LEVEL    
#ifdef _ENABLE_DBG_LEVEL
#define PROC_FS_NAME	"sis_dbg"
#define PROC_FS_MAX_LEN	8
static struct proc_dir_entry *dbgProcFile;
#endif
static unsigned int gPrint_point = 0;

static void sis_tpinfo_clear(struct sisTP_driver_data *TPInfo, int max);

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sis_ts_early_suspend(struct early_suspend *h);
static void sis_ts_late_resume(struct early_suspend *h);
#endif

#ifdef CONFIG_X86
//static const struct i2c_client_address_data addr_data;
/* Insmod parameters */
static int sis_ts_detect(struct i2c_client *client, struct i2c_board_info *info);
#endif


#ifdef TOUCH_STRESS_TEST
int sis_command_for_read(struct i2c_client *client, int rlength, unsigned char *rdata);

static void  stress_poll_data(struct work_struct * work)
{
	bool status;
	u8 count[7];
	status =  sis_command_for_read(ts_bak->client, 7, count);
	if(status < 0)
		printk("Read touch sensor data fail\n");

	if(poll_mode ==0)
		msleep(5);

	queue_delayed_work(stress_work_queue, &sis_poll_data_work, poll_mode);
}

static int sis_stress_open(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	if( !atomic_dec_and_test(&touch_stress_available)){
		atomic_inc(&touch_stress_available);
		return -EBUSY; /* already open */
	}
	return 0;          /* success */
}


static int sis_stress_release(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	atomic_inc(&touch_stress_available); /* release the device */
	return 0;          /* success */
}

static int sis_stress_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;

	printk("%s\n", __func__, cmd);
	if (_IOC_TYPE(cmd) != STRESS_IOC_MAGIC)
	return -ENOTTY;
	if (_IOC_NR(cmd) > STRESS_IOC_MAXNR)
	return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
	err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
	err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;
	switch (cmd) {
		case STRESS_POLL_DATA:
			if (arg == STRESS_IOCTL_START_HEAVY){
				printk("touch sensor heavey\n");
			poll_mode = START_HEAVY;
			queue_delayed_work(stress_work_queue, &sis_poll_data_work, poll_mode);
			}
			else if (arg == STRESS_IOCTL_START_NORMAL){
				printk("touch sensor normal\n");
				poll_mode = START_NORMAL;
				queue_delayed_work(stress_work_queue, &sis_poll_data_work, poll_mode);
			}
			else if  (arg == STRESS_IOCTL_END){
			printk("touch sensor end\n");
			cancel_delayed_work_sync(&sis_poll_data_work);
			}
			else
				return -ENOTTY;
			break;
		break;
	default: /* redundant, as cmd was checked against MAXNR */
	    return -ENOTTY;
	}
	return 0;
}

static struct file_operations stress_fops = {
		.owner =    THIS_MODULE,
		.unlocked_ioctl =	sis_stress_ioctl,
		.open =		sis_stress_open,
		.release =	sis_stress_release,
		};
#endif

void PrintBuffer(int start, int length, char* buf)
{
	int i;
	for ( i = start; i < length; i++ )
	{
		printk("%02x ", buf[i]);
	}
	printk("\n");	
}

int sis_command_for_write(struct i2c_client *client, int wlength, unsigned char *wdata)
{
    int ret = -1;
    struct i2c_msg msg[1];

    //printk(KERN_INFO "write\n");
    msg[0].addr = client->addr;
    msg[0].flags = 0; //Write
    msg[0].len = wlength;
    msg[0].buf = (unsigned char *)wdata;

    ret = i2c_transfer(client->adapter, msg, 1);
    //printk(KERN_INFO "write ret=%d\n", ret);

    return ret;
}

int sis_command_for_read(struct i2c_client *client, int rlength, unsigned char *rdata)
{
    int ret = -1;
    struct i2c_msg msg[1];

    //printk(KERN_INFO "read\n");
    msg[0].addr = client->addr;
    msg[0].flags = I2C_M_RD; //Read
    msg[0].len = rlength;
    msg[0].buf = rdata;

    ret = i2c_transfer(client->adapter, msg, 1);

    //printk(KERN_INFO "read ret =%d\n", ret);

    return ret;
}

int sis_sent_command_to_fw(struct i2c_client *client, int wlength, unsigned char *wdata, int rlength, unsigned char *rdata,\
							const unsigned char* func_name)
{
    int ret = -1;  

	ret = sis_command_for_write(client, wlength, wdata);
    if (ret < 0)
	{
		if (wdata[0] == 0x90)
		{
			printk(KERN_ERR "%s: CMD- 90 +%2x i2c_transfer write error - %d\n", func_name, wdata[2] ,ret);
		}
		else
		{
			printk(KERN_ERR "%s: CMD-%2x i2c_transfer write error - %d\n", func_name, wdata[0] ,ret);
		}
	}
	else 
	{
		msleep(3000);
		ret = sis_command_for_read(client, rlength, rdata);
		if (ret < 0)
		{
			printk(KERN_ERR "%s: CMD-%2x i2c_transfer write error - %d\n", func_name, wdata[0] ,ret);
		}
	} 	    
		
    return ret;
}

int sis_cul_unit(uint8_t report_id)
{
	int basic = 6;
	int area = 2;
	int pressure = 1;
	int ret = basic;
	
	if (BIT_AREA(report_id) && BIT_TOUCH(report_id))
	{
		ret += area;
	}
	if (BIT_PRESSURE(report_id))
	{
		ret += pressure;
	}
	
	return ret;	
}
#ifdef OLD_FORMAT_AEGIS
int sis_ReadPacket(struct i2c_client *client, uint8_t cmd, uint8_t* buf)
{
	uint8_t tmpbuf[MAX_BYTE] = {0};
    int ret = -1;
    int bytecount = 0;
    int touchnum = 0;
//	uint8_t offset = 0;
//	bool ReadNext = false;
//	uint8_t ByteCount = 0;
//	uint8_t fingers = 0;

/* 
#ifndef _SMBUS_INTERFACE
    struct i2c_msg msg[2];
 
	msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = (char *)(&cmd);

    msg[0].addr = client->addr;
    msg[0].flags = I2C_M_RD;
    msg[0].len = MAX_BYTE;
    msg[0].buf = tmpbuf;
#endif
	ret = i2c_transfer(client->adapter, msg, 1);
*/ 

/*
		New i2c format 
	* buf[0] = Low 8 bits of byte count value
	* buf[1] = High 8 bits of byte counte value
	* buf[2] = Report ID
	* buf[touch num * 6 + 2 ] = Touch informations; 1 touch point has 6 bytes, it could be none if no touch 
	* buf[touch num * 6 + 3] = Touch numbers
	* 
	* One touch point information include 6 bytes, the order is 
	* 
	* 1. status = touch down or touch up
	* 2. id = finger id 
	* 3. x axis low 8 bits
	* 4. x axis high 8 bits
	* 5. y axis low 8 bits
	* 6. y axis high 8 bits
	* 
*/
	ret = sis_command_for_read(client, MAX_BYTE, tmpbuf);
#if 0
	printk(KERN_INFO "chaoban test: Buf_Data [0~63] \n");
	PrintBuffer(0, 64, tmpbuf);	
#endif			
	if(ret < 0 )
	{
		printk(KERN_ERR "sis_ReadPacket: i2c transfer error\n");
	}
	memcpy(&buf[0], &tmpbuf[0], 58);
	bytecount = buf[BYTE_COUNT] & 0xff;
	touchnum = buf[TOUCH_NUM] & 0xff;
	if (bytecount == ((touchnum * 8) + 2 ))
	{
		if(touchnum > 7)
		{
			ret = sis_command_for_read(client, MAX_BYTE, tmpbuf);
#if 0
			printk(KERN_INFO "chaoban test: Buf_Data [64-125] \n");
			PrintBuffer(0, 64, tmpbuf);
#endif	
			if(ret < 0 )
			{
				printk(KERN_ERR "sis_ReadPacket: i2c transfer error\n");
				return ret;
			}
			memcpy(&buf[58], &tmpbuf[0], 64);
			ret = touchnum;
			return ret;			
		}
		else
		{
			ret = touchnum;
			return ret;
		}
	} 
	else
	{
		ret = -1;
		return ret;
	}
		
}	
#else
int sis_ReadPacket(struct i2c_client *client, uint8_t cmd, uint8_t* buf)
{
	uint8_t tmpbuf[MAX_BYTE] = {0};
#ifdef _CHECK_CRC
	uint16_t buf_crc = 0;
	uint16_t package_crc = 0;
#endif
    int ret = -1;
    int touchnum = 0;
    int L_COUNT_OTHER = 0;
    int bytecount = 0;
/*
		New i2c format 
	* buf[0] = Low 8 bits of byte count value
	* buf[1] = High 8 bits of byte counte value
	* buf[2] = Report ID
	* buf[touch num * 6 + 2 ] = Touch informations; 1 touch point has 6 bytes, it could be none if no touch 
	* buf[touch num * 6 + 3] = Touch numbers
	* 
	* One touch point information include 6 bytes, the order is 
	* 
	* 1. status = touch down or touch up
	* 2. id = finger id 
	* 3. x axis low 8 bits
	* 4. x axis high 8 bits
	* 5. y axis low 8 bits
	* 6. y axis high 8 bits
	* 
*/
	ret = sis_command_for_read(client, MAX_BYTE, tmpbuf);
#if 0
	printk(KERN_INFO "chaoban test: Buf_Data [0~63] \n");
	PrintBuffer(0, 64, tmpbuf);	
#endif			
	if(ret < 0 )
	{
		printk(KERN_ERR "sis_ReadPacket: i2c transfer error\n");
		return ret;
	}
	memcpy(&buf[0], &tmpbuf[2], 62);	//skip bytecount

	if (buf[L_REPORT_ID] == 0x10)		//One packet
	{	
		//L_COUNT_OTHER = sis_cul_unit(buf[L_REPORT_ID]) * 1 + 1;  // ***** Modify 
		//touchnum = buf[L_COUNT_OTHER] & 0xff;		
		
		bytecount = tmpbuf[0];
		bytecount = bytecount - 2 - 1;  // - byte_bytecout -byte_count
		
		touchnum = bytecount / sis_cul_unit(buf[L_REPORT_ID]);
		L_COUNT_OTHER = sis_cul_unit(buf[L_REPORT_ID]) * touchnum + 1;
		
		touchnum = buf[L_COUNT_OTHER] & 0xff;
		//touchnum = buf[L_COUNT_TOUCH] & 0xff;
		

#ifdef _CHECK_CRC
		buf_crc = cal_crc(buf, 0, L_COUNT_TOUCH -1);
		package_crc = ((buf[L_COUNT_TOUCH + 1] & 0xff) | ((buf[L_COUNT_TOUCH + 2] & 0xff)<< 8));
		if (buf_crc != package_crc)
		{
			printk(KERN_INFO "sis_ReadPacket: CRC Error\n");
			return -1;
		}
#endif

	}
	/*else if (buf[L_REPORT_ID] == 0x4) 	//BUTTON
	{
		return 1;
	}*/
	else 								//TOUCH_SERIAL
	{
		bytecount = tmpbuf[0];
		bytecount = bytecount - 2 - 1 - 1 - 2;  // - byte_bytecout -ReportID -byte_count -CRC
		
		touchnum = bytecount / sis_cul_unit(buf[L_REPORT_ID]);
		L_COUNT_OTHER = sis_cul_unit(buf[L_REPORT_ID]) * touchnum + 1; // + ReportID
		
		touchnum = buf[L_COUNT_OTHER] & 0xff;		
		
		//L_COUNT_OTHER = sis_cul_unit(buf[L_REPORT_ID]) * 5 + 1;
		//touchnum = buf[L_COUNT_OTHER] & 0xff;

#ifdef _CHECK_CRC
		buf_crc = cal_crc(buf, 0, L_COUNT_OTHER -1);
		if (BIT_SCANTIME(buf[L_REPORT_ID]))
		{
			package_crc = ((buf[L_COUNT_OTHER + 3] & 0xff) | ((buf[L_COUNT_OTHER + 4] & 0xff)<< 8));
		}
		else
		{
			package_crc = ((buf[L_COUNT_OTHER + 1] & 0xff) | ((buf[L_COUNT_OTHER + 2] & 0xff)<< 8));
		}
		if (buf_crc != package_crc)
		{
			printk(KERN_INFO "sis_ReadPacket: CRC Error\n");
			return -1;
		}
#endif		

		if(touchnum > 5)
		{
			ret = sis_command_for_read(client, MAX_BYTE, tmpbuf);
#if 0
			printk(KERN_INFO "chaoban test: Buf_Data [64-125] \n");
			PrintBuffer(0, 64, tmpbuf);
#endif
			if(ret < 0)
			{
				printk(KERN_ERR "sis_ReadPacket: i2c transfer error\n");
				return ret;
			}
			
			if ((tmpbuf[L_COUNT_OTHER + 2] & 0xff) != 0)
			{
				printk(KERN_ERR "sis_ReadPacket: get error package\n");
				return -1;
			}
			memcpy(&buf[64], &tmpbuf[2], 62);	//skip bytecount

#ifdef _CHECK_CRC
			buf_crc = cal_crc(buf, 0, L_COUNT_OTHER -1);
			if (BIT_SCANTIME(buf[L_REPORT_ID]))
			{
				package_crc = ((buf[L_COUNT_OTHER + 3] & 0xff) | ((buf[L_COUNT_OTHER + 4] & 0xff)<< 8));
			}
			else
			{
				package_crc = ((buf[L_COUNT_OTHER + 1] & 0xff) | ((buf[L_COUNT_OTHER + 2] & 0xff)<< 8));
			}
			if (buf_crc != package_crc)
			{
				printk(KERN_INFO "sis_ReadPacket: CRC Error\n");
				return -1;
			}
#endif				
		}		
	}
	return touchnum;
}	
#endif

int check_gpio_interrupt(void)
{
    int ret = 0;
    //TODO
    //CHECK GPIO INTERRUPT STATUS BY YOUR PLATFORM SETTING.
    ret = gpio_get_value(GPIO_IRQ);
    return ret;
}

void ts_report_key(struct i2c_client *client, uint8_t keybit_state)
{
	int i = 0;
	uint8_t diff_keybit_state= 0x0; //check keybit_state is difference with pre_keybit_state
	uint8_t key_value = 0x0; //button location for binary
	uint8_t  key_pressed = 0x0; //button is up or down
	struct sis_ts_data *ts = i2c_get_clientdata(client);

	if (!ts)
	{
		printk("%s error: Missing Platform Data!\n", __func__);
		return;
	}

	diff_keybit_state = TPInfo->pre_keybit_state ^ keybit_state;

	if (diff_keybit_state)
	{
		for (i = 0; i < BUTTON_KEY_COUNT; i++)
		{
		    if ((diff_keybit_state >> i) & 0x01)
			{
				key_value = diff_keybit_state & (0x01 << i);
				key_pressed = (keybit_state >> i) & 0x01;
				switch (key_value)
				{
					case MSK_COMP:
						input_report_key(ts->input_dev, KEY_COMPOSE, key_pressed);
						break;
					case MSK_BACK:
						input_report_key(ts->input_dev, KEY_BACK, key_pressed);
						break;
					case MSK_MENU:
						input_report_key(ts->input_dev, KEY_MENU, key_pressed);
						break;
					case MSK_HOME:
						input_report_key(ts->input_dev, KEY_HOME, key_pressed);
						break;
					case MSK_NOBTN:
						//Release the button if it touched.
					default:
						break;
				}
			}
		}
		TPInfo->pre_keybit_state = keybit_state;
	}
}

void sis_sticky(uint16_t* point_info, const unsigned int max, int* point_info_temp, int diff)
{
	if ((*point_info == max) || (*point_info == 1))  //get the MAX value or MIN value, report the value directly. 
	{
		point_info_temp[0] = *point_info;
	}
	else if (((point_info_temp[1]) > (point_info_temp[0]) + diff ) ||((point_info_temp[1]) < (point_info_temp[0]) - diff ))
	{												//the differnce of two point's value is beyond a standard range.
		*point_info =((point_info_temp[0]) + (point_info_temp[1])) >> 1;
		point_info_temp[1] = *point_info;
	}
	else               								//the differnce of two point's value is below a standard range.
	{
		*point_info = point_info_temp[1];
	}
}
#ifdef OLD_FORMAT_AEGIS
static void sis_ts_work_func(struct work_struct *work)
{
	struct sis_ts_data *ts = container_of(work, struct sis_ts_data, work);
    int ret = -1;
	uint8_t buf[PACKET_BUFFER_SIZE] = {0};
	uint8_t i = 0, fingers = 0;
	uint8_t px = 0, py = 0, pstatus = 0;
#ifdef _ANDROID_4
	bool all_touch_up = true;
#endif

    /* I2C or SMBUS block data read */
    ret = sis_ReadPacket(ts->client, SIS_CMD_NORMAL, buf);
#if 0
	printk(KERN_INFO "chaoban test: Buf_Data [0~63] \n");
	PrintBuffer(0, 64, buf);			
	if (ret > 7 )
	{
		printk(KERN_INFO "chaoban test: Buf_Data [64~125] \n");
		PrintBuffer(0, 64, buf);	
	}
#endif
	if (ret < 0) //Error fingers' number or Unknow bytecount
	{
	    printk(KERN_INFO "chaoban test: ret = -1\n");
		goto err_free_allocate;
	}
	else if (ret == 0)
	{
		for( i = 0; i < 10; i++) //when no touch, clean information temp buffer
		{
			ts->area_tmp[i][0]=0;
			ts->pressure_tmp[i][0]=0;
		}
		i=0;
		//printk(KERN_INFO "no touch!\n");
		TPInfo->pt[i].bPressure = 0;
		TPInfo->pt[i].bWidth = 0; 
#ifdef _ANDROID_4
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bWidth);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TPInfo->pt[i].bPressure);
#else
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TPInfo->pt[i].bWidth);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bPressure);		
#endif
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);
		//printk(KERN_INFO "ID = %d , pressure = %d , width = %d", TPInfo->pt[i].id, TPInfo->pt[i].bPressure, TPInfo->pt[i].bWidth);
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);
		goto err_free_allocate;
	}
/*	
	else if ((ret == 2) && (TPInfo->id == buf[0])) // Redundant package
	{
		goto label_send_report;
	}
*/
	sis_tpinfo_clear(TPInfo, MAX_FINGERS);

	/* Parser and Get the sis9200 data */
	fingers = (buf[TOUCH_NUM]);
	TPInfo->fingers = fingers = (fingers > MAX_FINGERS ? 0 : fingers);
//	TPInfo->id = buf[ADDR_REPORT_ID];
/*
	if ((buf[FORMAT_MODE] & MSK_BUTTON_POINT) == BUTTON_TOUCH_SERIAL)
	{
		int temp_fingers = 0;
		if (fingers > 1)
		{
			 temp_fingers = 2; // when fingers is >= 2, BS is placed at the same position
		}
		else
		{
			 temp_fingers = fingers;
		}
		ts_report_key(ts->client, buf[BUTTON_STATE + temp_fingers * 5]);
										//buf[BUTTON_STATE + temp_fingers * 5]: BS location
	}
	else
	{
		if (TPInfo->pre_keybit_state)
	  	{
			ts_report_key(ts->client, 0x0);//clear for polling
	  	}
	}
*/

	for (i = 0; i < fingers; i++)
	{
        pstatus = 2 + (i * 8);    // Calc point status

/*
		if (((buf[FORMAT_MODE] & MSK_BUTTON_POINT) == BUTTON_TOUCH_SERIAL) && i > 1)
		{
			pstatus += 1; 					// for button event and above 3 points
		}
*/
	    px = pstatus + 2;                   // Calc point x_coord
	    py = px + 2;                        // Calc point y_coord

		if ((buf[pstatus]) == TOUCHUP)
		{
			TPInfo->pt[i].bPressure = 0;
		}
		else if ((buf[pstatus]) == TOUCHDOWN)
		{
			TPInfo->pt[i].bPressure = (buf[pstatus + 7 ]);
		}
		else
		{
			goto err_free_allocate;
		}
//		TPInfo->pt[i].touch = (buf[pstatus + 6 ]);
		TPInfo->pt[i].bWidth = (buf[pstatus + 6 ]);
		TPInfo->pt[i].id = (buf[pstatus + 1 ]);
		TPInfo->pt[i].x = ((buf[px] & 0xff) | ((buf[px + 1] & 0xff)<< 8));
        TPInfo->pt[i].y = ((buf[py] & 0xff) | ((buf[py + 1] & 0xff)<< 8));
        
	}
	for (i = 0; i < TPInfo->fingers; i++)
	{
		ts->area_tmp[i][1]=TPInfo->pt[i].bWidth;
		ts->pressure_tmp[i][1]=TPInfo->pt[i].bPressure;
		//process the touch area and pressure sticky
		sis_sticky(&TPInfo->pt[i].bWidth, AREA_LENGTH_LONGER, ts->area_tmp[i], 2);
		sis_sticky(&TPInfo->pt[i].bPressure, PRESSURE_MAX, ts->pressure_tmp[i], 10);
	}
#if 0
	for (i = 0; i < TPInfo->fingers; i++)
	{
		printk(KERN_INFO "chaoban test: x[%d] = %d, y[%d] = %d, pressure[%d] = %d\n", i, TPInfo->pt[i].x, i, TPInfo->pt[i].y, i, TPInfo->pt[i].bPressure);
	}
#endif

//label_send_report:
    /* Report co-ordinates to the multi-touch stack */
#ifdef _ANDROID_4	
		for(i = 0; ((i < TPInfo->fingers) && (i < MAX_FINGERS)); i++)
		{
			if(TPInfo->pt[i].bPressure)
			{
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bWidth);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TPInfo->pt[i].bPressure);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);     //Android 2.3
				input_mt_sync(ts->input_dev);
				all_touch_up = false;
			}
			
			if (i == (TPInfo->fingers -1) && all_touch_up == true)
			{
				input_mt_sync(ts->input_dev);
			}
		}

		if(TPInfo->fingers == 0)
		{
			input_mt_sync(ts->input_dev);
		}
#else
	i = 0;
		do
		{
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bPressure);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TPInfo->pt[i].bWidth);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);		//Android 2.3
			//printk(KERN_INFO "poinert ID = %d\n", TPInfo->pt[i].id);
			input_mt_sync(ts->input_dev);
			i++;
		}
		while ((i < TPInfo->fingers) && (i < MAX_FINGERS));
#endif
	input_sync(ts->input_dev);

err_free_allocate:

    if (ts->use_irq)
    {
#ifdef _INT_MODE_1 //case 1 mode
	    //TODO: After interrupt status low, read i2c bus data by polling, until interrupt status is high
	    ret = check_gpio_interrupt();	//interrupt pin is still LOW, read data until interrupt pin is released.
	    if (!ret)
	    {
	        hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
	    }
	    else
	    {
			if (TPInfo->pre_keybit_state)
			{
				ts_report_key(ts->client, 0x0);//clear for interrupt
			}
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
        	if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
        	if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
        	{
				enable_irq(ts->client->irq);
			}
	    }
#else // case 2 mode

#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
		{
			enable_irq(ts->client->irq);
		}
#endif
	}

    return;
}
#else
static void sis_ts_work_func(struct work_struct *work)
{
	struct sis_ts_data *ts = container_of(work, struct sis_ts_data, work);
    int ret = -1;
    int point_unit;
    //int L_COUNT_OTHER;
    //int bytecount = 0;
    //int button_key;
	uint8_t buf[PACKET_BUFFER_SIZE] = {0};
	uint8_t i = 0, fingers = 0;
	uint8_t px = 0, py = 0, pstatus = 0;
#ifdef _ANDROID_4
	bool all_touch_up = true;
#endif

    /* I2C or SMBUS block data read */
    ret = sis_ReadPacket(ts->client, SIS_CMD_NORMAL, buf);
    
#if 0
	printk(KERN_INFO "ret = %d\n", ret);
	printk(KERN_INFO "chaoban test: Buf_Data [0~63] \n");
	PrintBuffer(0, 64, buf);			
	if ((buf[L_REPORT_ID] != 0x10) && (ret > 5))
	{
		printk(KERN_INFO "chaoban test: Buf_Data [64~125] \n");
		PrintBuffer(64, 128, buf);	
	}
#endif

	if (ret < 0) //Error fingers' number or Unknow bytecount
	{
	    printk(KERN_INFO "chaoban test: ret = -1\n");
		goto err_free_allocate;
	}
	else if (ret == 0)
	{
		for( i = 0; i < 10; i++) //when no touch, clean information temp buffer
		{
			ts->area_tmp[i][0]=0;
			ts->pressure_tmp[i][0]=0;
		}
		i=0;
		//printk(KERN_INFO "no touch!\n");
		TPInfo->pt[i].bPressure = 0;
		TPInfo->pt[i].bWidth = 0; 
#ifdef _ANDROID_4
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bWidth);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TPInfo->pt[i].bPressure);
#else
		input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TPInfo->pt[i].bWidth);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bPressure);		
#endif
		input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
		input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);
		//printk(KERN_INFO "ID = %d , pressure = %d , width = %d", TPInfo->pt[i].id, TPInfo->pt[i].bPressure, TPInfo->pt[i].bWidth);
		input_mt_sync(ts->input_dev);
		input_sync(ts->input_dev);
		goto err_free_allocate;
	}
/*	
	else if ((ret == 2) && (TPInfo->id == buf[0])) // Redundant package
	{
		goto label_send_report;
	}
*/
	sis_tpinfo_clear(TPInfo, MAX_FINGERS);

	/* Parser and Get the sis9200 data */
	point_unit = sis_cul_unit(buf[L_REPORT_ID]);
	if (buf[L_REPORT_ID] == 0x10)
	{
		//L_COUNT_OTHER = sis_cul_unit(buf[L_REPORT_ID]) * 1 + 1;  // ***** Modify
		//fingers = buf[L_COUNT_OTHER] & 0xff;	
		
		fingers = ret;
		
		//fingers = buf[L_COUNT_TOUCH] & 0xff;	
	}
	else 
	{
		fingers = ret;
		//L_COUNT_OTHER = point_unit * 5 + 1;
		//fingers = buf[L_COUNT_OTHER] & 0xff;
	}
	
	/*
	if (buf[L_REPORT_ID] == 0x4)
	{
		fingers = 0;
		button_key = ((buf[BUTTON_STATE] & 0xff) | ((buf[BUTTON_STATE + 1] & 0xff)<< 8));
		ts_report_key(ts->client, button_key);		
	}
	else
	{
		if (TPInfo->pre_keybit_state)
	  	{
			ts_report_key(ts->client, 0x0);//clear for polling
	  	}
	}
	*/

	TPInfo->fingers = fingers = (fingers > MAX_FINGERS ? 0 : fingers);
	
	for (i = 0; i < fingers; i++)
	{
        if ((buf[L_REPORT_ID] != 0x10) && (i >= 5))
        {
			pstatus = 1 + ((i - 5) * point_unit);    // Calc point status
			pstatus += 64;
		}
		else 
		{
			pstatus = 1 + (i * point_unit);          // Calc point status
		}

	    px = pstatus + 2;                   // Calc point x_coord
	    py = px + 2;                        // Calc point y_coord

		if ((buf[pstatus]) == TOUCHUP)
		{
			TPInfo->pt[i].bWidth = 0;
			TPInfo->pt[i].bPressure = 0;
		}
		else if ((buf[pstatus]) == TOUCHDOWN)
		{	
			if (buf[L_REPORT_ID] == 0x10)
			{
					TPInfo->pt[i].bWidth = 1;
					TPInfo->pt[i].bPressure = 1;
			}
			else
			{
				if (BIT_PRESSURE(buf[L_REPORT_ID]))
				{
					if (BIT_AREA(buf[L_REPORT_ID]))
					{
						//TPInfo->pt[i].bWidth = ((buf[pstatus + 6] & 0xff) | ((buf[pstatus + 7] & 0xff)<< 8));
						TPInfo->pt[i].bWidth = buf[pstatus + 6] & 0xff;
						TPInfo->pt[i].bPressure = (buf[pstatus + 8]);
					}
					else
					{
						TPInfo->pt[i].bWidth = 1;
						TPInfo->pt[i].bPressure = (buf[pstatus + 8]);
					}
				}
				else
				{
					if (BIT_AREA(buf[L_REPORT_ID]))
					{				
						TPInfo->pt[i].bWidth = ((buf[pstatus + 6] & 0xff) | ((buf[pstatus + 7] & 0xff)<< 8));
						TPInfo->pt[i].bPressure = 1;
					}
					else
					{
						TPInfo->pt[i].bWidth = 1;
						TPInfo->pt[i].bPressure = 1;
					}
				}
			}			
		}
		else
		{
			goto err_free_allocate;
		}
		
		TPInfo->pt[i].id = (buf[pstatus + 1]);
		TPInfo->pt[i].x = ((buf[px] & 0xff) | ((buf[px + 1] & 0xff)<< 8));
        TPInfo->pt[i].y = ((buf[py] & 0xff) | ((buf[py + 1] & 0xff)<< 8));      
	}
/*	
	for (i = 0; i < TPInfo->fingers; i++)
	{
		ts->area_tmp[i][1]=TPInfo->pt[i].bWidth;
		ts->pressure_tmp[i][1]=TPInfo->pt[i].bPressure;
		
		//process the touch area and pressure sticky
		sis_sticky(&TPInfo->pt[i].bWidth, AREA_LENGTH_LONGER, ts->area_tmp[i], 2);
		sis_sticky(&TPInfo->pt[i].bPressure, PRESSURE_MAX, ts->pressure_tmp[i], 10);
	}
*/

#if 0
	for (i = 0; i < TPInfo->fingers; i++)
	{
		 printk(KERN_INFO "chaoban test: i = %d, x= %d, y = %d, pstatus=%d, area = %d, pressure = %d, \n", i, TPInfo->pt[i].x, TPInfo->pt[i].y, buf[pstatus], TPInfo->pt[i].bWidth, TPInfo->pt[i].bPressure);
	}
#endif

//label_send_report:
    /* Report co-ordinates to the multi-touch stack */
#ifdef _ANDROID_4	
		for(i = 0; ((i < TPInfo->fingers) && (i < MAX_FINGERS)); i++)
		{
			if(TPInfo->pt[i].bPressure)
			{
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bWidth);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TPInfo->pt[i].bPressure);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
				input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);     //Android 2.3
				input_mt_sync(ts->input_dev);
				if(unlikely(gPrint_point))
					printk("ID = %d , x=%d, y=%d, pressure = %d , width = %d \n", TPInfo->pt[i].id, 
					TPInfo->pt[i].x, TPInfo->pt[i].y, TPInfo->pt[i].bPressure, TPInfo->pt[i].bWidth);
				all_touch_up = false;
			}
			
			if (i == (TPInfo->fingers -1) && all_touch_up == true)
			{
				input_mt_sync(ts->input_dev);
			}
		}

		if(TPInfo->fingers == 0)
		{
			input_mt_sync(ts->input_dev);
		}
#else
	i = 0;
		do
		{
			input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bPressure);
			input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, TPInfo->pt[i].bWidth);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_X, TPInfo->pt[i].x);
			input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, TPInfo->pt[i].y);
			input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, TPInfo->pt[i].id);		//Android 2.3
			//printk(KERN_INFO "poinert ID = %d\n", TPInfo->pt[i].id);
			input_mt_sync(ts->input_dev);
			i++;
		}
		while ((i < TPInfo->fingers) && (i < MAX_FINGERS));
#endif
	input_sync(ts->input_dev);

err_free_allocate:

    if (ts->use_irq)
    {
#ifdef _INT_MODE_1 //case 1 mode
	    //TODO: After interrupt status low, read i2c bus data by polling, until interrupt status is high
	    ret = check_gpio_interrupt();	//interrupt pin is still LOW, read data until interrupt pin is released.
	    if (!ret)
	    {
	        hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
	    }
	    else
	    {
			if (TPInfo->pre_keybit_state)
			{
				ts_report_key(ts->client, 0x0);//clear for interrupt
			}
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
        	if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
        	if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
        	{
				enable_irq(ts->client->irq);
			}
	    }
#else // case 2 mode

#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
		{
			enable_irq(ts->client->irq);
		}
#endif
	}

    return;
}
#endif

static void sis_tpinfo_clear(struct sisTP_driver_data *TPInfo, int max)
{
	int i = 0;
	for(i = 0; i < max; i++)
	{
		TPInfo->pt[i].id = -1;
		TPInfo->pt[i].touch = -1;
		TPInfo->pt[i].x = 0;
		TPInfo->pt[i].y = 0;
		TPInfo->pt[i].bPressure = 0;
		TPInfo->pt[i].bWidth = 0;
	}
	TPInfo->CRC = 0x0;
	TPInfo->id = 0x0;
	TPInfo->fingers = 0;
}

static enum hrtimer_restart sis_ts_timer_func(struct hrtimer *timer)
{
	struct sis_ts_data *ts = container_of(timer, struct sis_ts_data, timer);
	queue_work(sis_wq, &ts->work);
	if (!ts->use_irq)
	{	// For Polling mode
	    hrtimer_start(&ts->timer, ktime_set(0, TIMER_NS), HRTIMER_MODE_REL);
	}
	return HRTIMER_NORESTART;
}

static irqreturn_t sis_ts_irq_handler(int irq, void *dev_id)
{
	struct sis_ts_data *ts = dev_id;
	
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
	if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#else
	if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#endif
	{
		disable_irq_nosync(ts->client->irq);
	}
	queue_work(sis_wq, &ts->work);
	return IRQ_HANDLED;
}

static int initial_irq(void)
{
	int ret = 0;
#ifdef _I2C_INT_ENABLE
	/* initialize gpio and interrupt pins */
	/* TODO */
	//ret = gpio_request(GPIO_IRQ, "GPIO_133");	// ex. GPIO_133 for interrupt mode
	if (ret < 0)
	{
		// Set Active Low. Please reference the file include/linux/interrupt.h
		printk(KERN_ERR "sis_ts_probe: Failed to gpio_request\n");
		printk(KERN_ERR "sis_ts_probe: Fail : gpio_request was called before this driver call\n");
	}	
	/* setting gpio direction here OR boardinfo file*/
	/* TODO */
#else
	ret = -1;
#endif
	return ret;
}

uint16_t cal_crc (char* cmd, int start, int end)
{
	int i = 0;
	uint16_t crc = 0;
	for (i = start; i <= end ; i++)
	{
		crc = (crc<<8) ^ crc16tab[((crc>>8) ^ cmd[i] )&0x00FF];
	}
	return crc;
}

void write_crc (unsigned char *buf, int start, int end)
{
	uint16_t crc = 0;
	crc = cal_crc (buf, start , end);
	buf[end+1] = (crc >> 8)& 0xff;
	buf[end+2] = crc & 0xff;
}

#ifdef CONFIG_FW_SUPPORT_POWERMODE
bool sis_check_fw_ready(struct i2c_client *client)
{
  	bool retry = true;
  	int ret = 0;
  	int check_num = 10;
	unsigned char rdata[MAX_BYTE] = {0};
	unsigned char CheckI2C_Address[MAX_BYTE] = {0x88, 0x0e, 0x70, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6f, 0x93};

	do
	{
		ret = sis_sent_command_to_fw(client, SIXTEEN_BYTE, CheckI2C_Address, MAX_BYTE, rdata, __func__);

		if (ret > -1)	
		{
#if 0
			PrintBuffer(0, 16, rdata);
#endif
			ret = rdata[0];
			if (ret == 6 && rdata[4]== 0x01)
			{
				retry = false;
			}
		}
		if (retry == true) 
		{
			if (check_num != 0 )
			{
				printk(KERN_ERR "sis_check_ready: retry_count- %d\n", check_num);
				check_num--;
			}
			else
			{
				printk(KERN_ERR "sis_check_ready: I2C not ready\n");
			}
			msleep(50);
		}
	}while(retry);

	if (retry == false) 
	{
		return true;
	}
	return false;
}

uint8_t sis_check_fw_mode(struct i2c_client *client, uint8_t mode)
{
  	int ret;
	uint8_t tmpbuf[MAX_BYTE] = {0};
	uint8_t cmd[5] = {SIS_CMD_POWERMODE, 0x03, mode, 0, 0};
				   // command, bytecount, power mode, CRC,CRC
	write_crc(cmd, 2, 2);

#if 0
	printk(KERN_INFO "command:%02x\n", mode);
	PrintBuffer(0, 16, cmd);
#endif

	ret = sis_sent_command_to_fw(client, FIVE_BYTE, cmd, MAX_BYTE, tmpbuf, __func__);

	if (ret > -1)
	{
#if 0
		printk(KERN_INFO "get:\n");
		PrintBuffer(0, 16, tmpbuf);
#endif
		if (tmpbuf[0] == 0x03)
		{
			return tmpbuf[1];
		}
	}
	return -1;
}

void sis_fw_softreset(struct i2c_client *client)
{
	//softreset to get ITO base
	uint8_t tmpbuf[MAX_BYTE] = {0};
	uint8_t wcmd = SIS_CMD_SOFTRESET;
	int ret = 0;

	ret = sis_sent_command_to_fw(client, ONE_BYTE, &wcmd, MAX_BYTE, tmpbuf, __func__);

	if (ret > -1)
	{
#if 0
		printk(KERN_INFO "SOFTRESET: ");
		PrintBuffer(0, 16, tmpbuf);
#endif
		if(tmpbuf[0] == 0x04 && tmpbuf[1] == 0x0 && tmpbuf[2] == 0x80)
		{
			sis_check_fw_ready(client);
		}
		else
		{
			printk(KERN_ERR "sis_fw_softreset: SOFTRESET NACK %d\n", ret);
		}
	}
}
#endif //CONFIG_FW_SUPPORT_POWERMODE

void sis_sent_zero_command(struct i2c_client *client)
{
	/* skip the waiting time of recieve update FW command in bootloader */
	int ret = 0;
	int retry = 5;
	unsigned char read_cmd = SIS_CMD_NORMAL;
	unsigned char rdata[MAX_BYTE] = {0};
	do
	{	
		ret = sis_sent_command_to_fw(client, ONE_BYTE, &read_cmd, MAX_BYTE, rdata, __func__);
		if (ret < 0)
		{
			printk(KERN_INFO "sis_sent_zero_command: retry - %d\n", retry);
			retry--;
		}
	}
	while(ret < 0 && retry > 0);
}

#ifdef _STD_RW_IO
#define BUFFER_SIZE MAX_BYTE
ssize_t sis_cdev_write( struct file *file, const char __user *buf, size_t count, loff_t *f_pos )
{
	 int ret = 0;
	 char *kdata;
	 char cmd;
	 //printk(KERN_INFO "sis_cdev_write.\n");
	 
	 if (ts_bak == 0)
    	return -13;
    	
    ret = access_ok(VERIFY_WRITE, buf, BUFFER_SIZE);
    if (!ret) {
        printk(KERN_INFO "cannot access user space memory\n");
        return -11;
    }

	 kdata = kmalloc(BUFFER_SIZE, GFP_KERNEL);
     if (kdata == 0)
    	return -12;
    	
     ret = copy_from_user(kdata, buf, BUFFER_SIZE);
     if (ret) {
        printk(KERN_INFO "copy_from_user fail\n");
        kfree(kdata);
        return -14;
     } 
#if 0
	PrintBuffer(0, count, kdata);
#endif

	cmd = kdata[6];

    //printk(KERN_INFO "io cmd=%02x\n", cmd);

//Write & Read
    ret = sis_command_for_write(ts_bak->client, count, kdata);
    if (ret < 0) {
        printk(KERN_INFO "i2c_transfer write error %d\n", ret);
		kfree(kdata);
		return -21;
	}

    if ( copy_to_user((char*) buf, kdata, BUFFER_SIZE ) )
    {
        printk(KERN_INFO "copy_to_user fail\n" );
        ret = -19;
    }

    kfree( kdata );

	return ret;
}

//for get system time
ssize_t sis_cdev_read( struct file *file, char __user *buf, size_t count, loff_t *f_pos )
{
	 int ret = 0;
	 char *kdata;
	 char cmd;
	 int i;
	 //printk(KERN_INFO "sis_cdev_read.\n");
	 
	 if (ts_bak == 0)
    	return -13;
    	
    ret = access_ok(VERIFY_WRITE, buf, BUFFER_SIZE);
    if (!ret) {
        printk(KERN_INFO "cannot access user space memory\n");
        return -11;
    }

	 kdata = kmalloc(BUFFER_SIZE, GFP_KERNEL);
     if (kdata == 0)
    	return -12;
    	
     ret = copy_from_user(kdata, buf, BUFFER_SIZE);
     if (ret) {
        printk(KERN_INFO "copy_from_user fail\n");
        kfree(kdata);
        return -14;
     }    
#if 0
    PrintBuffer(0, count, kdata);
#endif
	 cmd = kdata[6];
	 //for making sure AP communicates with SiS driver
    if(cmd == 0xa2)
    {
		kdata[0] = 5;
		kdata[1] = 0;
		kdata[3] = 'S';
		kdata[4] = 'i';
		kdata[5] = 'S';
		if ( copy_to_user((char*) buf, kdata, BUFFER_SIZE ) )
		{
			printk(KERN_INFO "copy_to_user fail\n" );
			kfree( kdata );
			return -19;
		}

		kfree( kdata );
		return 3;	
	}
//Write & Read
    ret = sis_command_for_read(ts_bak->client, MAX_BYTE, kdata);
    if (ret < 0) {
        printk(KERN_INFO "i2c_transfer read error %d\n", ret);
		kfree(kdata);
		return -21;
	}

	ret = kdata[0] | (kdata[1] << 8);

/*
    for ( i = 0; i < BUFFER_SIZE - 1; i++ ) {
	    kdata[i] = kdata[i+1];
    }
*/
/*
    printk(KERN_INFO "%d\n", ret);

    for ( i = 0; i < ret && i < BUFFER_SIZE; i++ )
    {
        printk("%02x ", kdata[i]);
    }

    printk( "\n" );
*/
    if ( copy_to_user((char*) buf, kdata, BUFFER_SIZE ) )
    {
        printk(KERN_INFO "copy_to_user fail\n" );
        ret = -19;
    }

    kfree( kdata );

	return ret;
}

#undef BUFFER_SIZE

int sis_cdev_open(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "sis_cdev_open.\n");
	if ( ts_bak == 0 )
    	return -13;

	if( !atomic_dec_and_test(&touch_char_available)){
		atomic_inc(&touch_char_available);
		return -EBUSY; /* already open */
	}
	
	if (ts_bak->use_irq)
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
        if ((ts_bak->desc->status & IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#else
		if ((ts_bak->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#endif	
		{
			disable_irq(ts_bak->client->irq);
			msleep(200);
		}
		else
		{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
			printk(KERN_INFO "sis_cdev_open: IRQ_STATUS: %x\n",(ts_bak->desc->status & IRQ_DISABLED));
#else
			printk(KERN_INFO "sis_cdev_open: IRQ_STATUS: %x\n",(ts_bak->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED));
#endif	
		}
	}
	hrtimer_cancel(&ts_bak->timer);

    flush_workqueue(sis_wq);

    msleep(200);

	return 0; /* success */
}

int sis_cdev_release(struct inode *inode, struct file *filp)
{
	printk(KERN_INFO "sis_cdev_release.\n");
	 msleep(200);
    if (ts_bak == 0)
    	return -13;

	if (ts_bak->use_irq)
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
        if ((ts_bak->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts_bak->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif	
		{
			enable_irq(ts_bak->client->irq);
		}
	}
	else
		hrtimer_start(&ts_bak->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	
       atomic_inc(&touch_char_available); /* release the device */
    return 0;
}

#define SIS_IOCTLID	 0xB0
#define IOCTL_TP_VENDOR    _IOR(SIS_IOCTLID,  1, int)
#define IOCTL_FW_VERSION  _IOR(SIS_IOCTLID, 2, int)
#define IOCTL_MODE_LOCK  _IOR(SIS_IOCTLID, 3, int)
#define IOCTL_MODE_UNLOCK  _IOR(SIS_IOCTLID, 4, int)

static long sis_cdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg){
    struct sis_ts_data *ts = ts_bak;
    unsigned char fw_info_cmd[14] = {0x04, 0x00, 0x0C, 0x00, 0x09, 0x00, 0x86, 0x08, 0x04, 0xC0, 0x00, 0xA0, 0x34, 0x00};
    unsigned char buf[PACKET_BUFFER_SIZE];
    int ret = 0;
    dev_info(&ts->client->dev, "ioctl command:%X\n", cmd);	
    switch(cmd){
    case IOCTL_TP_VENDOR:
        return gpio_get_value(TEGRA_GPIO_PBB7) | (gpio_get_value(TEGRA_GPIO_PO7) << 1);
    case IOCTL_FW_VERSION:
        ret = sis_command_for_write(ts->client, 14, fw_info_cmd);
        msleep(2);
        ret = sis_command_for_read(ts->client, 64, buf);
        memcpy(ts->fw_info, buf + 8, 12);
        return  (ts->fw_info[10] << 8) | ts->fw_info[11];
    case IOCTL_MODE_LOCK:
    case IOCTL_MODE_UNLOCK:
    default:
        break;
    }
    return 0;
}

static const struct file_operations sis_cdev_fops = {
	.owner	= THIS_MODULE,
	.read	= sis_cdev_read,
	.write	= sis_cdev_write,
	.open	= sis_cdev_open,
	.release= sis_cdev_release,
	.unlocked_ioctl = sis_cdev_ioctl,
};

static int sis_setup_chardev(struct sis_ts_data *ts)
{
	
	dev_t dev = MKDEV(sis_char_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;
	int input_err = 0;
	struct device *class_dev = NULL;
	void *ptr_err;
	
	printk("sis_setup_chardev.\n");
	
	if (ts == NULL) 
	{
	  input_err = -ENOMEM;
	  goto error;
	} 
	 // dynamic allocate driver handle
	alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, DEVICE_NAME);
	if (alloc_ret)
		goto error;
		
	sis_char_major = MAJOR(dev);
	cdev_init(&sis_char_cdev, &sis_cdev_fops);
	sis_char_cdev.owner = THIS_MODULE;
	cdev_err = cdev_add(&sis_char_cdev, MKDEV(sis_char_major, 0), sis_char_devs_count);
	
	if (cdev_err) 
		goto error;
	
	printk(KERN_INFO "%s driver(major %d) installed.\n", DEVICE_NAME, sis_char_major);
	
	// register class
	sis_char_class = class_create(THIS_MODULE, DEVICE_NAME);
	if(IS_ERR(ptr_err = sis_char_class)) 
	{
		goto err2;
	}
	
	class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, DEVICE_NAME);
	
	if(IS_ERR(ptr_err = class_dev)) 
	{
		goto err;
	}
	
	return 0;
error:
	if (cdev_err == 0)
		cdev_del(&sis_char_cdev);
	if (alloc_ret == 0)
		unregister_chrdev_region(MKDEV(sis_char_major, 0), sis_char_devs_count);
	if(input_err != 0)
	{
		printk("sis_ts_bak error!\n");
	}
err:
	device_destroy(sis_char_class, MKDEV(sis_char_major, 0));
err2:
	class_destroy(sis_char_class);
	return -1;
}
#endif

static int sis_read_firmware_info(struct sis_ts_data *ts){
     int ret, count, retry_count;
     unsigned char buf_data[PACKET_BUFFER_SIZE];
     const unsigned char fw_info_cmd[14] = {0x04, 0x00, 0x0C, 0x00, 0x09, 0x00, 0x86, 
     0x08, 0x04, 0xC0, 0x00, 0xA0, 0x34, 0x00};

     for(count = 0, retry_count == 0; count < 50; count++){
         if(retry_count > 3) return -1;
         memset(ts->fw_info, 0, sizeof(ts->fw_info));
         ret = sis_command_for_write(ts->client, 14, fw_info_cmd);
	  printk("%s write count=%d ret=%d\n", __func__, count, ret);
	   if(ret < 0){
	        msleep(20);
		  /* If the remote device is not connected*/
	        if(ret == -ETIMEDOUT) retry_count++;
	        continue;
	    }          
	
          msleep(2);
          ret = sis_command_for_read(ts->client, 64, buf_data);
	    printk("%s read count=%d ret=%d\n", __func__, count, ret);
	    if(ret < 0 || buf_data[4] != 0xef || buf_data[5] != 0xbe){
	        msleep(5);
	        continue;
	    }
          
          memcpy(ts->fw_info, buf_data + 8, 12);
          return 0;
    } 

    return -1;
}

static ssize_t sis_touch_switch_name(struct switch_dev *sdev, char *buf)
{ 
       int fw_id, fw_version;

       sis_read_firmware_info(ts_bak);
       fw_id = (ts_bak->fw_info[8] << 8) | ts_bak->fw_info[9];
       fw_version =  (ts_bak->fw_info[10] << 8) | ts_bak->fw_info[11];
       return sprintf(buf,  "SIS-%c%c%c-%c%c%c%c-%d.%d\n", ts_bak->fw_info[1], ts_bak->fw_info[2], ts_bak->fw_info[3],
	   	        ts_bak->fw_info[4], ts_bak->fw_info[5], ts_bak->fw_info[6], ts_bak->fw_info[7], fw_id, fw_version);
}

static ssize_t sis_touch_switch_state(struct switch_dev *sdev, char *buf)
{ 
	if(project_id==TEGRA3_PROJECT_ME301T){       
		return sprintf(buf, "%d", gpio_get_value(TEGRA_GPIO_PH5));
	}
	else{
		return sprintf(buf, "%d", gpio_get_value(TEGRA_GPIO_PR3));	
	}
}

static ssize_t sis_show_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
      // 0: cannot connect to SIS touch chip,  1: I2C is work
	return sprintf(buf, "%d\n", sis_i2c_status);
}

DEVICE_ATTR(sis_touchpanel_status, S_IRUGO, sis_show_status, NULL);

static ssize_t sis_show_tp_vendor(struct device *dev, struct device_attribute *devattr, char *buf)
{
     unsigned int vendor_id = project_id == TEGRA3_PROJECT_TF500T ? 
	 	(gpio_get_value(TEGRA_GPIO_PBB7) | (gpio_get_value(TEGRA_GPIO_PO7) << 1)) :
	 	(gpio_get_value(TEGRA_GPIO_PI2) | (gpio_get_value(TEGRA_GPIO_PI4) << 1)); 
	return sprintf(buf, "%02d", vendor_id);
}

DEVICE_ATTR(tp_vendor, S_IRUGO, sis_show_tp_vendor, NULL);

static ssize_t sis_irq_gpio(struct device *dev, struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(GPIO_IRQ));
}

DEVICE_ATTR(gpio_irq, S_IRUGO, sis_irq_gpio, NULL);


static struct attribute *sis_attr[] = {
	&dev_attr_sis_touchpanel_status.attr,
	&dev_attr_tp_vendor.attr,
	&dev_attr_gpio_irq.attr,
	NULL
};


#ifdef _ENABLE_DBG_LEVEL
static int sis_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data )
{
	int ret;
	
	printk("call proc_read\n");
	
	if(offset > 0)  /* we have finished to read, return 0 */
		ret  = 0;
	else 
		ret = sprintf(buffer, "Debug Level: Release Date: %s\n","2011/10/05");

	return ret;
}

static int sis_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char procfs_buffer_size = 0; 
	int i, ret = 0;
	unsigned char procfs_buf[PROC_FS_MAX_LEN+1] = {0};
	unsigned int command;
	procfs_buffer_size = count;
	if(procfs_buffer_size > PROC_FS_MAX_LEN ) 
		procfs_buffer_size = PROC_FS_MAX_LEN+1;
	
	if( copy_from_user(procfs_buf, buffer, procfs_buffer_size) ) 
	{
		printk(" proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	command = 0;
	for(i=0; i<procfs_buffer_size-1; i++)
	{
		if( procfs_buf[i]>='0' && procfs_buf[i]<='9' )
			command |= (procfs_buf[i]-'0');
		else if( procfs_buf[i]>='A' && procfs_buf[i]<='F' )
			command |= (procfs_buf[i]-'A'+10);
		else if( procfs_buf[i]>='a' && procfs_buf[i]<='f' )
			command |= (procfs_buf[i]-'a'+10);
		
		if(i!=procfs_buffer_size-2)
			command <<= 4;
	}

	command = command&0xFFFFFFFF;
      switch(command){
      case 0xF1: 
	  	 gPrint_point = 1; 
	  	 break;
      case 0xF2: 
	  	 gPrint_point = 0; 
	  	 break;
	}
	printk("Run command: 0x%08X  result:%d\n", command, ret);

	return count; // procfs_buffer_size;
}
#endif // #ifdef _ENABLE_DBG_LEV

static int sis_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0;
	struct sis_ts_data *ts = NULL;
	struct sis_i2c_rmi_platform_data *pdata = NULL;
	unsigned char buf[PACKET_BUFFER_SIZE];
	unsigned char fw_info_cmd[14] = {0x04, 0x00, 0x0C, 0x00, 0x09, 0x00, 0x86, 0x08, 0x04, 0xC0, 0x00, 0xA0, 0x34, 0x00}; 

    printk(KERN_INFO "sis_ts_probe\n");

    TPInfo = kzalloc(sizeof(struct sisTP_driver_data), GFP_KERNEL);
    if (TPInfo == NULL) 
    {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) 
	{
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts_bak = ts;

	//1. Init Work queue and necessary buffers
	INIT_WORK(&ts->work, sis_ts_work_func);
	ts->client = client;
	i2c_set_clientdata(client, ts);
    pdata = client->dev.platform_data;

	if (pdata)
		ts->power = pdata->power;
	if (ts->power) 
	{
		ret = ts->power(1);
		if (ret < 0) 
		{
			printk(KERN_ERR "sis_ts_probe power on failed\n");
			goto err_power_failed;
		}
	}
	
        project_id = tegra3_get_project_id();
	printk("##### %s: start opend the PMIC GPIO 5V power.", __func__);
        gpio_direction_output(TOUCH_PMIC_5V_POWER, 1);
	printk("##### %s: end opend the PMIC GPIO 5V power.", __func__);
        msleep(20);
        dev_info(&client->dev, "SIS hardware reset\n");
        gpio_direction_output(TEGRA_GPIO_PH6, 0);
        msleep(5);
        gpio_set_value(TEGRA_GPIO_PH6, 1);
        msleep(1250);
	// check sis IC
        /*
	ret = sis_command_for_read(client, PACKET_BUFFER_SIZE, buf);
	if (ret < 0){
	    goto err_input_dev_alloc_failed;
	}
        */
      // Get Firmware info
      /*
      ret = sis_command_for_write(client, 14, fw_info_cmd);
      msleep(2);
      ret = sis_command_for_read(client, 64, buf);
      memcpy(ts->fw_info, buf + 8, 12);
      */
      ret = sis_read_firmware_info(ts);
	sis_i2c_status = ret < 0 ? 0 : 1; 
	//2. Allocate input device
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) 
	{
		ret = -ENOMEM;
		printk(KERN_ERR "sis_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}

	ts->input_dev->name = "sis_touch";//"SiS9200-i2c-touchscreen";

#ifdef _SKIP_FW_WAITING_TIME
		sis_sent_zero_command(client);	// skip the waiting time of recieve update FW command in bootloader
#endif

	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
    set_bit(ABS_MT_POSITION_X, ts->input_dev->absbit);
    set_bit(ABS_MT_POSITION_Y, ts->input_dev->absbit);
    set_bit(ABS_MT_TRACKING_ID, ts->input_dev->absbit);

#ifdef _ANDROID_4
    set_bit(ABS_MT_PRESSURE, ts->input_dev->absbit);
    set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
    input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE, 0, PRESSURE_MAX, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, AREA_LENGTH_LONGER, 0, 0);
#else
    set_bit(ABS_MT_TOUCH_MAJOR, ts->input_dev->absbit);
    set_bit(ABS_MT_WIDTH_MAJOR, ts->input_dev->absbit);
    input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESSURE_MAX, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, AREA_LENGTH_LONGER, 0, 0);
#endif

    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, SIS_MAX_X, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, SIS_MAX_Y, 0, 0);
    input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID, 0, 15, 0, 0);
	
    /* add for touch keys */
	set_bit(KEY_COMPOSE, ts->input_dev->keybit);
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);

	//3. Register input device to core
	ret = input_register_device(ts->input_dev);

	if (ret) 
	{
		printk(KERN_ERR "sis_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}
	
	//4. irq or timer setup
	ret = initial_irq();
	//ret = 1;
	if (ret < 0) 
	{

	}
	else
	{
		client->irq = gpio_to_irq(GPIO_IRQ);
		ret = request_irq(client->irq, sis_ts_irq_handler, IRQF_TRIGGER_FALLING, client->name, ts);
		if (ret == 0) 
		{
		   ts->use_irq = 1;
		}
		else 
		{
			dev_err(&client->dev, "request_irq failed\n");
		}
	}

	ts->desc = irq_to_desc(ts_bak->client->irq);
	
	hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ts->timer.function = sis_ts_timer_func;

	if (!ts->use_irq) 
	{
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ts->early_suspend.suspend = sis_ts_early_suspend;
	ts->early_suspend.resume = sis_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif
	printk(KERN_INFO "sis_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");
	
	if (ts->use_irq)
	{
#ifdef _INT_MODE_1
			printk(KERN_INFO "sis_ts_probe: interrupt case 1 mode\n");
#else
			printk(KERN_INFO "sis_ts_probe: interrupt case 2 mode\n");
#endif
	}
	
#ifdef _STD_RW_IO
	ret = sis_setup_chardev(ts);
	if(ret){
            printk( KERN_INFO"sis_setup_chardev fail\n");
	}
	
#endif
      ts->attrs.attrs = sis_attr;
      ret = sysfs_create_group(&client->dev.kobj, &ts->attrs);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
	}
      sis_touch_sdev.name = "touch";
      sis_touch_sdev.print_name = sis_touch_switch_name;
      sis_touch_sdev.print_state = sis_touch_switch_state;
      if(switch_dev_register(&sis_touch_sdev) < 0){
          dev_info(&client->dev, "switch_dev_register for dock failed!\n");
      }
      switch_set_state(&sis_touch_sdev, 0);
#ifdef TOUCH_STRESS_TEST
      stress_work_queue = create_singlethread_workqueue("i2c_touchsensor_wq");
	if(!stress_work_queue){
		dev_err(&client->dev, "Unable to create stress_work_queue workqueue\n");
	}
      INIT_DELAYED_WORK(&sis_poll_data_work, stress_poll_data);
	stress_misc_dev.minor  = MISC_DYNAMIC_MINOR;
	stress_misc_dev.name = "touchpanel";
	stress_misc_dev.fops = &stress_fops;
	ret = misc_register(&stress_misc_dev);
	if (ret) {
	    dev_err(&client->dev, "SIS stress test: Unable to register %s \\misc device\n", stress_misc_dev.name);
	}
#endif	 

#ifdef _ENABLE_DBG_LEVEL
	dbgProcFile = create_proc_entry(PROC_FS_NAME, 0666, NULL);
	if (dbgProcFile == NULL) 
	{
		remove_proc_entry(PROC_FS_NAME, NULL);
		printk(" Could not initialize /proc/%s\n", PROC_FS_NAME);
	}
	else
	{
		dbgProcFile->read_proc = sis_proc_read;
		dbgProcFile->write_proc = sis_proc_write;
		printk(" /proc/%s created\n", PROC_FS_NAME);
	}
#endif // #ifdef _ENABLE_DBG_LEVEL

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
	return ret;
}

static int sis_ts_remove(struct i2c_client *client)
{
	struct sis_ts_data *ts = i2c_get_clientdata(client);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ts->early_suspend);
#endif
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);
#ifdef TOUCH_STRESS_TEST
	misc_deregister(&stress_misc_dev);
#endif
#ifdef _ENABLE_DBG_LEVEL
	remove_proc_entry(PROC_FS_NAME, NULL);
#endif
	kfree(ts);
	return 0;
}

static int sis_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	struct sis_ts_data *ts = i2c_get_clientdata(client);
#ifdef _SMBUS_INTERFACE
	uint8_t tmpbuf[MAX_BYTE] = {0};
#endif

#ifdef CONFIG_FW_SUPPORT_POWERMODE
	int retry = 5;
	uint8_t status = -1;
#endif
if(project_id==TEGRA3_PROJECT_ME301T){
      gpio_direction_output(TEGRA_GPIO_PH5, 1);
}
else{
      gpio_direction_output(TEGRA_GPIO_PR3, 1);
}
	TPInfo->pre_keybit_state = 0x0;

	if (ts->use_irq)
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_ENABLED)
#endif
		{
			disable_irq(client->irq);
		}
	}
	else
		hrtimer_cancel(&ts->timer);
//	ret = cancel_work_sync(&ts->work); // only cancel one work(sis_ts_work_func), 
									   // but there maybe are others in workqueue.
//	flush_scheduled_work(); 		   // flush all of workqueue in kernel
	flush_workqueue(sis_wq); 		   //only flush sis_wq
    
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
		{
			enable_irq(client->irq);
		}
	}

#ifdef CONFIG_FW_SUPPORT_POWERMODE
#ifdef _SMBUS_INTERFACE
		uint8_t cmd[5] = {SIS_CMD_POWERMODE, 0x03, WRITE_DEEPSLEEP_MODE, 0, 0};
						// command, bytecount, power mode, CRC,CRC
		write_crc(cmd, 2, 2);
#else
		sis_check_fw_mode(client, WRITE_DEEPSLEEP_MODE); // Change to Deepsleep Mode
		
		while (retry > 0 && status != DEEPSLEEP_MODE)
		{
			//msleep(50);
			status = sis_check_fw_mode(client, READ_POWERMODE) & MSK_POWERMODE;
			printk(KERN_INFO "sis_power_saving_status: %x\n", status);	
			
			if (status != DEEPSLEEP_MODE)
			{
				if (retry == 1)
					printk(KERN_INFO "sis_ts_suspend: change mode failed\n");
				else
					printk(KERN_INFO "sis_ts_suspend: change mode retry - %d\n", retry);
				retry--;
			}
		}
#endif
#endif

#if 0
	/* Turn off SiS Chip*/
	/* TODO */
	gpio_direction_output(TOUCH_RESET_PIN, 0);
	printk("[MSI TOUCH] SiS Touch Reset Low\n");
//	msleep(5);
	gpio_direction_output(TOUCH_POWER_PIN, 0);
	printk("[MSI TOUCH] SiS Touch Power off\n");
#endif

	if (ts->power) {
		ret = ts->power(0);
		if (ret < 0)
			printk(KERN_ERR "sis_ts_suspend power off failed\n");
	}
	
	return 0;
}

static void force_release_pos(struct sis_ts_data *ts)
{
	int i;
	
	dev_info(&ts->client->dev, "Touch: force release position\n");
	for (i=0; i < 10; i++){
		if (TPInfo->pt[i].bWidth == 0 && TPInfo->pt[i].bPressure == 0) 
			continue;
             TPInfo->pt[i].bWidth = 0;
		TPInfo->pt[i].bPressure = 0;
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, i);
		input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, TPInfo->pt[i].bWidth);
		input_report_abs(ts->input_dev, ABS_MT_PRESSURE, TPInfo->pt[i].bPressure);
		input_mt_sync(ts->input_dev);
	}

	input_sync(ts->input_dev);
}

static int sis_ts_resume(struct i2c_client *client)
{
	int ret = 0;
	struct sis_ts_data *ts = i2c_get_clientdata(client);
#ifdef _SMBUS_INTERFACE
	uint8_t tmpbuf[MAX_BYTE] = {0};
	uint8_t cmd[5] = {0};
	uint16_t crc = 0;
#endif

#ifdef CONFIG_FW_SUPPORT_POWERMODE
	int retry = 5;
	uint8_t status = -1;
#endif
    force_release_pos(ts);
    if(project_id==TEGRA3_PROJECT_ME301T){
      gpio_direction_output(TEGRA_GPIO_PH5, 0); // turn on touch power 
      msleep(20);
      gpio_direction_output(TEGRA_GPIO_PH6, 0); // touch reset
	msleep(5);
      gpio_direction_output(TEGRA_GPIO_PH6, 1);
    }
    else{
      gpio_direction_output(TEGRA_GPIO_PR3, 0);		
    }
	if (ts->power)
	{
		ret = ts->power(1);
		if (ret < 0)
			printk(KERN_ERR "sis_ts_resume power on failed\n");
	}

#if 0
	/* Turn on SiS Chip*/
	/* TODO */
	gpio_direction_output(TOUCH_POWER_PIN, 1);
	printk("[MSI TOUCH] SiS Touch Power on\n");
	msleep(5);
	gpio_direction_output(TOUCH_RESET_PIN, 1);
	printk("[MSI TOUCH] SiS Touch Reset HI\n");
	msleep(5);
	gpio_direction_output(TOUCH_RESET_PIN, 0);
	printk("[MSI TOUCH] SiS Touch Reset Low\n");
	msleep(5);
	gpio_direction_output(TOUCH_RESET_PIN, 1);
	printk("[MSI TOUCH] SiS Touch Reset HI\n");
#endif

#ifdef CONFIG_FW_SUPPORT_POWERMODE
	status = sis_check_fw_mode(client, READ_POWERMODE) & MSK_POWERMODE;
	if( status != ACTIVE_MODE )
	{
#ifdef _SMBUS_INTERFACE
		uint8_t cmd[5] = {SIS_CMD_POWERMODE, 0x03, WRITE_ACTIVE_MODE, 0, 0}
						// command, bytecount, power mode, CRC,CRC
		write_crc(cmd, 2, 2);
		ret = i2c_smbus_read_block_data(client, cmd, tmpbuf);
#else
		sis_check_fw_mode(client, WRITE_ACTIVE_MODE); //Change to Active Mode

		while (retry > 0 && status != ACTIVE_MODE)
		{
			//msleep(50);
			status = sis_check_fw_mode(client, READ_POWERMODE) & MSK_POWERMODE;			
			printk(KERN_INFO "sis_power_saving_status: %x\n", status);	
					
			if (status != ACTIVE_MODE)
			{
				if (retry == 1)
					printk(KERN_INFO "sis_ts_suspend: change mode failed\n");
				else
					printk(KERN_INFO "sis_ts_suspend: change mode retry - %d\n", retry);
				retry--;
			}
		}
#endif
		sis_fw_softreset(client);
	}
	else
	{
		printk(KERN_ERR "sis_ts_resume: Active mode\n");
	}
#endif
	
#ifdef _SKIP_FW_WAITING_TIME	
	sis_sent_zero_command(client);	// skip the waiting time of recieve update FW command in bootloader
#endif

	if (ts->use_irq)
	{
#if ( LINUX_VERSION_CODE < KERNEL_VERSION (2, 6, 39) )
		if ((ts->desc->status & IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#else
		if ((ts->desc->irq_data.state_use_accessors & IRQD_IRQ_DISABLED) == IRQ_STATUS_DISABLED)
#endif
		{
			msleep(50);
			enable_irq(client->irq);
		}
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void sis_ts_early_suspend(struct early_suspend *h)
{
	struct sis_ts_data *ts;
	TPInfo->pre_keybit_state = 0x0;
	ts = container_of(h, struct sis_ts_data, early_suspend);
	sis_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void sis_ts_late_resume(struct early_suspend *h)
{
	struct sis_ts_data *ts;
	ts = container_of(h, struct sis_ts_data, early_suspend);
	sis_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id sis_ts_id[] = {
	{ SIS_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, sis_ts_id);

static struct i2c_driver sis_ts_driver = {
	.probe		= sis_ts_probe,
	.remove		= sis_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= sis_ts_suspend,
	.resume		= sis_ts_resume,
#endif
#ifdef CONFIG_X86
    .class      = I2C_CLASS_HWMON,
    .detect		= sis_ts_detect,
	.address_list	= normal_i2c,
#endif
	.id_table	= sis_ts_id,
	.driver = {
		.name	= SIS_I2C_NAME,
	},
};

static int __devinit sis_ts_init(void)
{
	printk( KERN_INFO "sis_ts_init\n" );
	sis_wq = create_singlethread_workqueue("sis_wq");

	if (!sis_wq)
		return -ENOMEM;
	return i2c_add_driver(&sis_ts_driver);
}

#ifdef CONFIG_X86
/* Return 0 if detection is successful, -ENODEV otherwise */
static int sis_ts_detect(struct i2c_client *client,
		       struct i2c_board_info *info)
{
	const char *type_name;
    printk(KERN_INFO "sis_ts_detect\n");
	type_name = "sis_i2c_ts";
	strlcpy(info->type, type_name, I2C_NAME_SIZE);
	return 0;
}
#endif

static void __exit sis_ts_exit(void)
{
#ifdef _STD_RW_IO
	dev_t dev;
#endif

	printk(KERN_INFO "sis_ts_exit\n");
	i2c_del_driver(&sis_ts_driver);
	if (sis_wq)
		destroy_workqueue(sis_wq);

#ifdef _STD_RW_IO
	dev = MKDEV(sis_char_major, 0);
	cdev_del(&sis_char_cdev);
	unregister_chrdev_region(dev, sis_char_devs_count);
	device_destroy(sis_char_class, MKDEV(sis_char_major, 0));
	class_destroy(sis_char_class);
#endif
}

module_init(sis_ts_init);
module_exit(sis_ts_exit);
MODULE_DESCRIPTION("SiS 9200 Family Touchscreen Driver");
MODULE_LICENSE("GPL");
