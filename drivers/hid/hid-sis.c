/*
 *  HID driver for sis9237 test touchscreens
 *
 *  Copyright (c) 2008 Rafi Rubin
 *  Copyright (c) 2009 Stephane Chatty
 *
 */

/*
 * This program is free software;  can  itmand/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/hid-debug.h>
#include "hid-ids.h"
//for i2c-bridge
#include <linux/usb.h>
#include "usbhid/usbhid.h"
#include <asm/uaccess.h>
#include <linux/types.h>

//for ioctl
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <../arch/arm/mach-tegra/gpio-names.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#define TOUCH_POWER_CONTROL 1

#define INTERNAL_DEVICE_NAME "sis_zeus_hid_touch_device"
#define BRIDGE_DEVICE_NAME "sis_zeus_hid_bridge_touch_device"
#define SIS817_DEVICE_NAME "sis_aegis_hid_touch_device"
#define SISF817_DEVICE_NAME "sis_aegis_hid_bridge_touch_device"
static int sis_char_devs_count = 1;        /* device count */
static int sis_char_major = 0;
static struct cdev sis_char_cdev;
static struct class *sis_char_class = NULL;
//struct sis_data *nd;

#define SIS_MAX_X		4095
#define SIS_MAX_Y		4095
#define MAX_POINT		15
#define HID_DG_SCANTIME		0x000d0056	//new usage not defined in hid.h
#define REPORTID_01		0x01
#define REPORTID_10		0x10
#define REPORTID_TYPE1		0x30


#define CTRL 0
#define ENDP_01 1
#define ENDP_02 2
#define DIR_IN 0x1

#define P1801_DOCK_DETECT_PIN TEGRA_GPIO_PI6
#define P1801_DOCK_ON 0
#define P1801_DOCK_OFF 1

//20110111 Tammy system call for tool
static struct hid_device *hid_dev_backup = NULL;  //backup address
static struct urb *backup_urb = NULL;

#define PACKET_BUFFER_SIZE				128
static unsigned char mfw_info[12] = {0}; 
static struct switch_dev sis_touch_sdev;

struct Point {
	u16 x, y, id, pressure, width, height;
};

struct sis_data {
	int id, total, ReportID, scantime;
	struct Point pt[MAX_POINT];
};


static int pkg_num=0;
static int idx=-1;

#define _ENABLE_DBG_LEVEL    
#define SIS_TOUCH_TP_VENDOR_PIN TEGRA_GPIO_PR6
#ifdef _ENABLE_DBG_LEVEL
#include <linux/proc_fs.h>
#define PROC_FS_NAME	"sis_hid_dbg"
#define PROC_FS_NAME_TP	"sis_tp_vendor"
#define PROC_FS_MAX_LEN	8
static struct proc_dir_entry *dbgProcFile = NULL;
static struct proc_dir_entry *dbgProcFile_tp = NULL;
#endif
static bool read_fw_info_flag = false;

/*
 * this driver is aimed at two firmware versions in circulation:
 *  - dual pen/fingedrivers/hid/hid-sis.c:83:r single touch
 *  - finger multitouch, pen not working
 */

static int sis_input_mapping(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	//hi->input->name = "SiS817-HID-touchscreen";	//yuger0524

	// No special mappings needed for the pen and single touch 
	if (field->physical == HID_GD_POINTER)
		return -1;
	
	else if (field->physical && (field->physical != HID_GD_POINTER))
		return 0;

	//printk (KERN_INFO "sis_input_mapping : usage->hid = %x\n", usage->hid);

	switch (usage->hid & HID_USAGE_PAGE) {
	case HID_UP_GENDESK:
		switch (usage->hid) {
		case HID_GD_X:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_POSITION_X);
                        /*input_set_abs_params(hi->input, ABS_MT_POSITION_X,
				field->logical_minimum, field->logical_maximum, 0, 0);
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_X);*/
			input_set_abs_params(hi->input, ABS_X, 
				field->logical_minimum, field->logical_maximum, 0, 0);
			return 1;

		case HID_GD_Y:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_POSITION_Y);
                        /*input_set_abs_params(hi->input, ABS_MT_POSITION_Y,
				field->logical_minimum, field->logical_maximum, 0, 0);
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_Y);*/
			input_set_abs_params(hi->input, ABS_Y,
				field->logical_minimum, field->logical_maximum, 0, 0);
			return 1;
		}
		return 0;

	case HID_UP_DIGITIZER:
		switch (usage->hid) {
		/* we do not want to map these for now */
		case HID_DG_CONFIDENCE:
		case HID_DG_INPUTMODE:
		case HID_DG_DEVICEINDEX:
		case HID_DG_CONTACTCOUNT:
		case HID_DG_CONTACTMAX:
		case HID_DG_INRANGE:

		//new usage for SiS817 Device(for later use)
		case HID_DG_SCANTIME:
		case HID_DG_WIDTH:
		case HID_DG_HEIGHT:		
		case HID_DG_TIPPRESSURE:
			//hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_PRESSURE);
			//input_set_abs_params(hi->input, ABS_MT_PRESSURE, 0, 2047, 0, 0);
			return -1;

		case HID_DG_TIPSWITCH:
//mark BTN	    hid_map_usage(hi, usage, bit, max, EV_KEY, BTN_TOUCH);
//mark BTN      input_set_capability(hi->input, EV_KEY, BTN_TOUCH);
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_PRESSURE);
			input_set_abs_params(hi->input, ABS_MT_PRESSURE, 0, 1, 0, 0);
			return 1;
#if 1
		case HID_DG_CONTACTID:
			hid_map_usage(hi, usage, bit, max, EV_ABS, ABS_MT_TRACKING_ID);
			input_set_abs_params(hi->input, ABS_MT_TRACKING_ID, 0, 127, 0, 0);
			return 1;
#endif
		}
		return 0;

	/*case HID_UP_BUTTON:
		return 0;*/

	case 0xff000000:
		/* ignore HID features */
		return -1;

	}
	/* ignore buttons */
	return 0;
}

static int sis_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{
	if (usage->type == EV_KEY || usage->type == EV_ABS)
		clear_bit(usage->code, *bit);

	return 0;
}

static void sis_event_emission(struct sis_data *nd, struct input_dev *input)
{
	int i;
	bool all_touch_up = true;
	for(i=0; i< nd->total; i++)
	{
		//printk(KERN_INFO "MT_event: finger(s)=%d, id=%d, x=%d, y=%d\n", nd->total, nd->pt[i].id, nd->pt[i].x, nd->pt[i].y);
		//printk(KERN_INFO "MT_event: pressure=%d, width=%d, height=%d, scantime=%d\n", nd->pt[i].pressure, nd->pt[i].width, nd->pt[i].height, nd->pt[i].scantime);
		if(nd->pt[i].pressure)
		{
			input_report_abs(input, ABS_MT_PRESSURE, nd->pt[i].pressure);
			input_report_abs(input, ABS_MT_POSITION_X, nd->pt[i].x);
			input_report_abs(input, ABS_MT_POSITION_Y, nd->pt[i].y);
			input_report_abs(input, ABS_MT_TRACKING_ID, nd->pt[i].id);
			input_mt_sync(input);
			all_touch_up = false;
		}

		if(i == (nd->total - 1) && all_touch_up == true)
			input_mt_sync(input);
	}
	//input_sync(input);
}

static void sis_event_clear(struct sis_data *nd, int max)
{
	int i;
	for(i=0; i<max; i++)
	{
		nd->pt[i].id = 0;
		nd->pt[i].x = 0;
		nd->pt[i].y = 0;
		nd->pt[i].pressure = 0;
		nd->pt[i].width = 0;
		nd->pt[i].height = 0;
	}
	nd->scantime = 0;
	idx = -1;
	pkg_num = 0;
}

static int sis_raw_event (struct hid_device *hid, struct hid_report *report,
		                        u8 *raw_data, int size)
{
	struct sis_data *nd = hid_get_drvdata(hid);
	nd->ReportID = raw_data[0];
	//printk(KERN_INFO "raw_event : ReportID = %d\n", nd->ReportID);
	hid_set_drvdata(hid, nd);
	return 0;
}

static void sis_event_lastdata(struct hid_device *hid, struct sis_data *nd, struct input_dev *input)
{
	int pkg_n=0;
	if ( (hid->product == USB_PRODUCT_ID_SIS817_TOUCH || hid->product == USB_PRODUCT_ID_SIS1012_TOUCH || hid->product == USB_PRODUCT_ID_SISF817_TOUCH) && (nd->ReportID == REPORTID_01 || nd->ReportID == REPORTID_10))	//817 method : original
	{
		sis_event_emission(nd, input);
		sis_event_clear(nd, MAX_POINT);
	}
	else if ( (hid->product == USB_PRODUCT_ID_SIS817_TOUCH || hid->product == USB_PRODUCT_ID_SIS1012_TOUCH || hid->product == USB_PRODUCT_ID_SISF817_TOUCH) && nd->ReportID != REPORTID_01 && nd->ReportID != REPORTID_10)	//817 method : Extend Class Format
	{
		if(nd->total >= 6)
		{				
			idx = 4;
			pkg_num = nd->total;	
		}
		else if(nd->total >= 1)
		{
			sis_event_emission(nd, input);
			sis_event_clear(nd, MAX_POINT);
		}
		else
		{
			if(pkg_num >0)
			{
				nd->total = pkg_num;
				sis_event_emission(nd, input);
				pkg_n = 0;
				sis_event_clear(nd, MAX_POINT);
			}
			else
			{
				sis_event_clear(nd, MAX_POINT);
			}
		}
	}
	else	//816 method
	{
		if(nd->total >= 3)
		{				
			idx = 1;
			pkg_num = nd->total;	
		}
		else if(nd->total >= 1)
		{
			sis_event_emission(nd, input);
			sis_event_clear(nd, MAX_POINT);
		}
		else
		{
			if(pkg_num >0)
			{
				if((pkg_num%2)>0)	
					pkg_n = pkg_num+1;
				else
					pkg_n = pkg_num;
			
				if(pkg_n == (idx + 1) )
				{
					nd->total = pkg_num;
					sis_event_emission(nd, input);
					pkg_n = 0;
					sis_event_clear(nd, MAX_POINT);
				}
			}
			else
			{
				sis_event_clear(nd, MAX_POINT);
			}
		}
	}
}
/*
 * this function is called upon all reports
 * so that we can filter contact point information,
 * decide whether we are in multi or single touch mode
 * and call input_mt_sync after each point if necessary
 */
static int sis_event (struct hid_device *hid, struct hid_field *field,
		                        struct hid_usage *usage, __s32 value)
{
	struct sis_data *nd = hid_get_drvdata(hid);
	//printk (KERN_INFO "sis_event");

        if (hid->claimed & HID_CLAIMED_INPUT) {
		//printk (KERN_INFO "sis_event : usage->hid = %x\n", usage->hid);
		struct input_dev *input = field->hidinput->input;
		switch (usage->hid) {
		case HID_DG_INRANGE:			
			break;

		case HID_DG_TIPSWITCH:
			idx++;
			if (value > 1 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : TipSwitch value error : %d, idx = %d", value, idx);
				value = 1;
			}
			nd->pt[idx].pressure = !!value;
			break;

		case HID_DG_CONTACTID:
			if (value > 30 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : Contact ID value error : %d, idx = %d", value, idx);
				value = 30;
			}
			nd->pt[idx].id = value;
			break;

		case HID_GD_X:
			if (value > 4095 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : X-axis value error : %d, idx = %d", value, idx);
				value = 4095;
			}
			nd->pt[idx].x = value;
			break;

		case HID_GD_Y:
			if (value > 4095 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : Y-axis value error : %d, idx = %d", value, idx);
				value = 4095;
			}
			nd->pt[idx].y = value;
			break;

		//new usage for SiS817 Extend Class Device
		case HID_DG_SCANTIME:
			if (value > 65535 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : scantime value error : %d, idx = %d", value, idx);
				value = 65535;
			}
			nd->scantime = value;
			if ( (nd->ReportID & 0xf0) > REPORTID_TYPE1 )
				sis_event_lastdata(hid, nd, input);
			break;

/*		case HID_DG_WIDTH:
			nd->pt[idx].width = value;
			break;

		case HID_DG_HEIGHT:
			nd->pt[idx].height = value;
			break;
		
		case HID_DG_TIPPRESSURE:
			nd->pt[idx].pressure = value;
			break;
*/		//end of new usage for SiS817 Extend Class Device

		case HID_DG_CONTACTCOUNT:
			if (value > 10 || value < 0)
			{
				//printk (KERN_INFO "!!!!!!!	sis_event : ContactCount value error : %d, idx = %d", value, idx);
			}
			nd->total = value;
			if ( (nd->ReportID & 0xf0) <= REPORTID_TYPE1 )
				sis_event_lastdata(hid, nd, input);
			break;			
		default:
			//fallback to the generic hidinput handling
			return 0;
		}
	}

	/* we have handled the hidinput part, now remains hiddev */
	if (hid->claimed & HID_CLAIMED_HIDDEV && hid->hiddev_hid_event)
	{
		//printk (KERN_INFO "!!!!!!!	sis_event : hid->hiddev_hid_event : idx = %d", idx);
		hid->hiddev_hid_event(hid, field, usage, value);
	}

	return 1;
}

static int sis_cdev_open(struct inode *inode, struct file *filp)	//20120306 Yuger ioctl for tool
{
	//20110511, Yuger, kill current urb by method of usbhid_stop
	if ( !hid_dev_backup )
	{
		printk( KERN_INFO "(stop)hid_dev_backup is not initialized yet" );
		return -1;
	}

	struct usbhid_device *usbhid = hid_dev_backup->driver_data;

	printk( KERN_INFO "sys_sis_HID_stop\n" );

	//printk( KERN_INFO "hid_dev_backup->vendor, hid_dev_backup->product = %x %x\n", hid_dev_backup->vendor, hid_dev_backup->product );

	//20110602, Yuger, fix bug: not contact usb cause kernel panic
	if( !usbhid )
	{
		printk( KERN_INFO "(stop)usbhid is not initialized yet" );
		return -1;
	}
	else if ( !usbhid->urbin )
	{
		printk( KERN_INFO "(stop)usbhid->urbin is not initialized yet" );
		return -1;
	}
	else if (hid_dev_backup->vendor == USB_VENDOR_ID_SIS2_TOUCH)
	{
		usb_fill_int_urb(backup_urb, usbhid->urbin->dev, usbhid->urbin->pipe,
			usbhid->urbin->transfer_buffer, usbhid->urbin->transfer_buffer_length,
			usbhid->urbin->complete, usbhid->urbin->context, usbhid->urbin->interval);

                clear_bit( HID_STARTED, &usbhid->iofl );
                set_bit( HID_DISCONNECTED, &usbhid->iofl );

                usb_kill_urb( usbhid->urbin );
                usb_free_urb( usbhid->urbin );
                usbhid->urbin = NULL;
		return 0;
	}
        else	
	{
		printk (KERN_INFO "This is not a SiS device");
		return -801;
	}
}

static int sis_cdev_release(struct inode *inode, struct file *filp)
{
	//20110505, Yuger, rebuild the urb which is at the same urb address, then re-submit it
	
	if ( !hid_dev_backup )
	{
		printk( KERN_INFO "(stop)hid_dev_backup is not initialized yet" );
		return -1;
	}

	int ret;

	struct usbhid_device *usbhid = hid_dev_backup->driver_data;
	unsigned long flags;

	printk( KERN_INFO "sys_sis_HID_start" );

	if( !usbhid )
	{
		printk( KERN_INFO "(start)usbhid is not initialized yet" );
		return -1;
	}

	if( !backup_urb )
	{
		printk( KERN_INFO "(start)backup_urb is not initialized yet" );
		return -1;
	}

	clear_bit( HID_DISCONNECTED, &usbhid->iofl );
	usbhid->urbin = usb_alloc_urb( 0, GFP_KERNEL );

	if( !backup_urb->interval )
	{
		printk( KERN_INFO "(start)backup_urb->interval does not exist" );
		return -1;
	}

	usb_fill_int_urb(usbhid->urbin, backup_urb->dev, backup_urb->pipe, 
		backup_urb->transfer_buffer, backup_urb->transfer_buffer_length, 
		backup_urb->complete, backup_urb->context, backup_urb->interval);
	usbhid->urbin->transfer_dma = usbhid->inbuf_dma;
	usbhid->urbin->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	set_bit( HID_STARTED, &usbhid->iofl );

	//method at hid_start_in
	spin_lock_irqsave( &usbhid->lock, flags );		
	ret = usb_submit_urb( usbhid->urbin, GFP_ATOMIC );
	spin_unlock_irqrestore( &usbhid->lock, flags );
	//yy

//	printk( KERN_INFO "ret = %d", ret );

	return ret;
}

//SiS 817 only
static ssize_t sis_cdev_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	int actual_length = 0, timeout = 0;
	u8 *rep_data = NULL;
	u16 size = 0;
	long rep_ret;

      if(hid_dev_backup == NULL) 
	  	return 0;
	
	struct usb_interface *intf = to_usb_interface(hid_dev_backup->dev.parent);
	struct usb_device *dev = interface_to_usbdev(intf);

	size = (((u16)(buf[64] & 0xff)) << 24) + (((u16)(buf[65] & 0xff)) << 16) + 
		(((u16)(buf[66] & 0xff)) << 8) + (u16)(buf[67] & 0xff);
	timeout = (((int)(buf[68] & 0xff)) << 24) + (((int)(buf[69] & 0xff)) << 16) + 
		(((int)(buf[70] & 0xff)) << 8) + (int)(buf[71] & 0xff);

	rep_data = kzalloc(size, GFP_KERNEL);
	if (!rep_data)
		return -12;

	if ( copy_from_user( rep_data, (void*)buf, size) ) 
	{
		printk( KERN_INFO "copy_to_user fail\n" );
		//free allocated data
		kfree( rep_data );
		rep_data = NULL;
		return -19;
	}

	if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS817_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_SIS1012_TOUCH)	//817 internal : endpoint 2 is int_in
		rep_ret = usb_interrupt_msg(dev, usb_rcvintpipe(dev, ENDP_02),
			rep_data, size, &actual_length, timeout);
	else								//817 bridge : endpoint 1 is int_in
		rep_ret = usb_interrupt_msg(dev, usb_rcvintpipe(dev, ENDP_01),
			rep_data, size, &actual_length, timeout);	

      if(read_fw_info_flag){
          memcpy(mfw_info, rep_data + 8, 12); 
          read_fw_info_flag = false;
      }

	if( rep_ret == 0 )
	{
		rep_ret = actual_length;
		if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
		{
			printk( KERN_INFO "copy_to_user fail\n" );
			//free allocated data
			kfree( rep_data );
			rep_data = NULL;
			return -19;
		}
	}

	//free allocated data
	kfree( rep_data );
	rep_data = NULL;
	//printk( KERN_INFO "sis_cdev_read : rep_ret = %d\n", rep_ret );
	return rep_ret;
}

static ssize_t sis_cdev_write( struct file *file, const char __user *buf, size_t count, loff_t *f_pos )
{
	int i, actual_length = 0;
	u8 *tmp_data = NULL;	//include report id
	u8 *rep_data = NULL;
	long rep_ret;

      if(hid_dev_backup == NULL) 
	  	return 0;
	
	struct usb_interface *intf = to_usb_interface( hid_dev_backup->dev.parent );
	struct usb_device *dev = interface_to_usbdev( intf );
	unsigned const char fw_info_cmd[10] = {0x09, 0x09, 0x86, 0x08, 0x04, 0xC0, 0x00, 0xA0, 0x34, 0x00};

      read_fw_info_flag = memcmp(buf, fw_info_cmd, sizeof(fw_info_cmd)) == 0;
	if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS817_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_SIS1012_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_SISF817_TOUCH )	//817 method
	{
		u16 size = (((u16)(buf[64] & 0xff)) << 24) + (((u16)(buf[65] & 0xff)) << 16) + 
			(((u16)(buf[66] & 0xff)) << 8) + (u16)(buf[67] & 0xff);
		int timeout = (((int)(buf[68] & 0xff)) << 24) + (((int)(buf[69] & 0xff)) << 16) + 
			(((int)(buf[70] & 0xff)) << 8) + (int)(buf[71] & 0xff);


		//printk( KERN_INFO "sys_sis_HID_IO %02X", request );
	
		//printk (KERN_INFO "timeout = %d, size %d\n", timeout, size);

		rep_data = kzalloc(size, GFP_KERNEL);
		if (!rep_data)
			return -12;

		if ( copy_from_user( rep_data, (void*)buf, size) ) 
		{
			printk( KERN_INFO "copy_to_user fail\n" );
			//free allocated data
			kfree( rep_data );
			rep_data = NULL;
			return -19;
		}

		if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS817_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_SIS1012_TOUCH)	//817 internal : endpoint 1 is int_out
			rep_ret = usb_interrupt_msg( dev, usb_sndintpipe( dev, ENDP_01 ),
				rep_data, size, &actual_length, timeout );
		else								//817 bridge : endpoint 2 is int_out
			rep_ret = usb_interrupt_msg( dev, usb_sndintpipe( dev, ENDP_02 ),
				rep_data, size, &actual_length, timeout );

		if( rep_ret == 0 )
		{
			rep_ret = actual_length;
			if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
			{
				printk( KERN_INFO "copy_to_user fail\n" );
				//free allocated data
				kfree( rep_data );
				rep_data = NULL;
				return -19;
			}
		}
		//printk( KERN_INFO "sis_cdev_write : rep_ret = %d\n", rep_ret );
	}
	else	//816 method
	{

		u8 request = buf[0];
		u8 dir = buf[1];
		u16 value = (((u16)(buf[2] & 0xff)) << 24) + (((u16)(buf[3] & 0xff)) << 16) + (((u16)(buf[4] & 0xff)) << 8) + (u16)(buf[5] & 0xff);
		u16 index = (((u16)(buf[6] & 0xff)) << 24) + (((u16)(buf[7] & 0xff)) << 16) + (((u16)(buf[8] & 0xff)) << 8) + (u16)(buf[9] & 0xff);


		u16 size = (((u16)(buf[29] & 0xff)) << 24) + (((u16)(buf[30] & 0xff)) << 16) + (((u16)(buf[31] & 0xff)) << 8) + (u16)(buf[32] & 0xff);
		int timeout = (((int)(buf[33] & 0xff)) << 24) + (((int)(buf[34] & 0xff)) << 16) + (((int)(buf[35] & 0xff)) << 8) + (int)(buf[36] & 0xff);

		//printk( KERN_INFO "sys_sis_HID_IO %02X", request );
	
		//printk (KERN_INFO "dir = %d, value = %d, index = %d, timeout = %d, buf[33] = %x, buf[34] = %x, buf[35] = %x, buf[36] = %x\n", dir, value, index, timeout, buf[33], buf[34], buf[35], buf[36]);

		rep_data = kzalloc( size , GFP_KERNEL );
		if ( rep_data == 0 ) 
		{
			return -12;
		}

		for( i = 0; i < size; i++)
		{
			rep_data[i] = buf[10+i];
		}

		tmp_data = kzalloc( size + 1, GFP_KERNEL );	//include report id, so size +1

		for( i = 0; i < size; i++ )
		{
			tmp_data[i+1] = rep_data[i];
		}

		buf += 10;

		if( dir & DIR_IN )
		{
			if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS2_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_NEW_SIS2_TOUCH )
			{
				//20110510, Yuger, for correcting intr data send into interrupt msg(receive, in, endp=2)
				tmp_data[0] = 0x0A;//in
				//printk(KERN_INFO "(INT_IN)tmp_data = %X %X %X, size = %u, actual_length = %d", tmp_data[1], tmp_data[2], tmp_data[3], size, actual_length);

				//816 internal : endpoint 2 is int_in
				rep_ret = usb_interrupt_msg( dev, usb_rcvintpipe( dev, ENDP_02 ), tmp_data, size+1, &actual_length, timeout );
				//yy
				//printk(KERN_INFO "rep_ret = %ld, actual_length = %d", rep_ret, actual_length);
				if( rep_ret == 0 )
				{
					rep_ret = actual_length;
				}

				//20110510, Yuger, for recovering rep_data
				for( i = 0; i < size; i++ )
				{
					rep_data[i] = tmp_data[i+1];
				}
				//yy

				//printk(KERN_INFO "(INT_IN)size = %u, dir = %u, rep_ret = %ld, rep_data = %X %X %X", size, dir, rep_ret, rep_data[0], rep_data[1], rep_data[2]);

				if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
				{
					printk( KERN_INFO "copy_to_user fail\n" );
					//free allocated data
					kfree( rep_data );
					kfree( tmp_data );
					rep_data = NULL;
					tmp_data = NULL;
					return -19;
				}
			}
			else
			{
				//control message
				rep_ret = usb_control_msg( dev, usb_rcvctrlpipe( dev, CTRL ), 
				request, (USB_DIR_IN|USB_TYPE_VENDOR|USB_RECIP_DEVICE), 
				value, index, rep_data, size, timeout );
#if 0
				printk ("(CTRL) size = %d, dir = %d, rep_ret = %ld, rep_data = ", size, dir, rep_ret);
				for (i=0; i<size-1; i++)  
				{
					printk ("%02X ", rep_data[i]);
				}
				if (i == size-1)
				printk ("%02X\n", rep_data[i]);
#endif
				if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
				{
					printk( KERN_INFO "copy_to_user fail\n" );
					//free allocated data
					kfree( rep_data );
					rep_data = NULL;
					return -19;
				}
			}
		}
		else
		{
			if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS2_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_NEW_SIS2_TOUCH )
			{
				//20110510, Yuger, for correcting intr data send into interrupt msg(send, out, endp=1)
				tmp_data[0] = 0x09;//out

				//816 internal : endpoint 1 is int_out
				rep_ret = usb_interrupt_msg( dev, usb_sndintpipe( dev, ENDP_01 ), tmp_data, size + 1, &actual_length, timeout );

				//just return success or not(no need to return actual_length if succeed)

				//20110510, Yuger, for recovering rep_data
				for( i = 0; i < size; i++ )
				{
					rep_data[i] = tmp_data[i+1];
				}
				//yy

				//printk(KERN_INFO "(INT_OUT)size = %u, actual_length = %d, rep_ret = %ld, rep_data = %x %x %x", size, actual_length, rep_ret, rep_data[0], rep_data[1], rep_data[2]);

				if ( copy_to_user( (void*)buf, rep_data, actual_length-1 ) ) 
				{
					printk( KERN_INFO "copy_to_user fail\n" );
					//free allocated data
					kfree( rep_data );
					kfree( tmp_data );
					rep_data = NULL;
					tmp_data = NULL;
					return -19;
				}
			}
			else
			{
				//control message
				rep_ret = usb_control_msg( dev, usb_sndctrlpipe( dev, CTRL ), 
				request, (USB_DIR_OUT|USB_TYPE_VENDOR|USB_RECIP_DEVICE), 
				value, index, rep_data, 16, timeout );
#if 0
				printk ("(CTRL) size = %d, dir = %d, rep_ret = %ld, rep_data = ", size, dir, rep_ret);
				for (i=0; i<size-1; i++)  
				{
					printk ("%02X ", rep_data[i]);
				}
				if (i == size-1)
					printk ("%02X\n", rep_data[i]);
#endif

				if ( copy_to_user( (void*)buf, rep_data, rep_ret ) ) 
				{
					printk( KERN_INFO "copy_to_user fail\n" );
					//free allocated data
					kfree( rep_data );
					rep_data = NULL;
					return -19;
				}
			}
		}
		//free allocated data
		kfree( tmp_data );
		tmp_data = NULL;
	}
	//free allocated data
	kfree( rep_data );
	rep_data = NULL;

	//printk( KERN_INFO "End of sys_sis_HID_IO\n" );
	return rep_ret;
}

//~TT

//for ioctl
static const struct file_operations sis_cdev_fops = {
	.owner	= THIS_MODULE,
	.read	= sis_cdev_read,
	.write	= sis_cdev_write,
	.open	= sis_cdev_open,
	.release= sis_cdev_release,
};

//for ioctl
static int sis_setup_chardev(struct hid_device *hdev, struct sis_data *nd)
{
	
	dev_t dev = MKDEV(sis_char_major, 0);
	int alloc_ret = 0;
	int cdev_err = 0;
	int input_err = 0;
	struct device *class_dev = NULL;
	void *ptr_err;
	
	printk("sis_setup_chardev.\n");
	
	if (nd == NULL) 
	{
          input_err = -ENOMEM;
          goto error;
	} 

	// dynamic allocate driver handle
	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
		alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, BRIDGE_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SIS817_TOUCH || hdev->product == USB_PRODUCT_ID_SIS1012_TOUCH)
		alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, SIS817_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SISF817_TOUCH)
		alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, SISF817_DEVICE_NAME);
	else
		alloc_ret = alloc_chrdev_region(&dev, 0, sis_char_devs_count, INTERNAL_DEVICE_NAME);

	if (alloc_ret)
		goto error;
		
	sis_char_major  = MAJOR(dev);
	cdev_init(&sis_char_cdev, &sis_cdev_fops);
	sis_char_cdev.owner = THIS_MODULE;
	cdev_err = cdev_add(&sis_char_cdev, MKDEV(sis_char_major, 0), sis_char_devs_count);
	if (cdev_err) 
		goto error;

	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
		printk(KERN_INFO "%s driver(major %d) installed.\n", BRIDGE_DEVICE_NAME, sis_char_major);
	else if (hdev->product == USB_PRODUCT_ID_SIS817_TOUCH || hdev->product == USB_PRODUCT_ID_SIS1012_TOUCH)
		printk(KERN_INFO "%s driver(major %d) installed.\n", SIS817_DEVICE_NAME, sis_char_major);
	else if (hdev->product == USB_PRODUCT_ID_SISF817_TOUCH)
		printk(KERN_INFO "%s driver(major %d) installed.\n", SISF817_DEVICE_NAME, sis_char_major);
	else
		printk(KERN_INFO "%s driver(major %d) installed.\n", INTERNAL_DEVICE_NAME, sis_char_major);

	// register class
	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
		sis_char_class = class_create(THIS_MODULE, BRIDGE_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SIS817_TOUCH || hdev->product == USB_PRODUCT_ID_SIS1012_TOUCH)
		sis_char_class = class_create(THIS_MODULE, SIS817_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SISF817_TOUCH)
		sis_char_class = class_create(THIS_MODULE, SISF817_DEVICE_NAME);
	else
		sis_char_class = class_create(THIS_MODULE, INTERNAL_DEVICE_NAME);

	if(IS_ERR(ptr_err = sis_char_class)) 
	{
		goto err2;
	}

	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
		class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, BRIDGE_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SIS817_TOUCH || hdev->product == USB_PRODUCT_ID_SIS1012_TOUCH)
		class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, SIS817_DEVICE_NAME);
	else if (hdev->product == USB_PRODUCT_ID_SISF817_TOUCH)
		class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, SISF817_DEVICE_NAME);
	else
		class_dev = device_create(sis_char_class, NULL, MKDEV(sis_char_major, 0), NULL, INTERNAL_DEVICE_NAME);

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

#ifdef _ENABLE_DBG_LEVEL
static int sis_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data ){
	int ret;
	
	if(offset > 0)  /* we have finished to read, return 0 */
		ret  = 0;
	else 
		ret = sprintf(buffer, "1\n");

	return ret;
}

static int sis_proc_write(struct file *file, const char *buffer, unsigned long count, void *data){	
	return 0; // procfs_buffer_size;
}

static int sis_proc_read_tp(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data ){
	int ret;
	
	if(offset > 0)  /* we have finished to read, return 0 */
		ret  = 0;
	else 
		ret = sprintf(buffer, "%d\n", gpio_get_value(SIS_TOUCH_TP_VENDOR_PIN));

	return ret;
}

static int sis_proc_write_tp(struct file *file, const char *buffer, unsigned long count, void *data){	
	return 0; // procfs_buffer_size;
}

#endif // #ifdef _ENABLE_DBG_LEV

static ssize_t sis_touch_switch_name(struct switch_dev *sdev, char *buf){ 
       int fw_id = (mfw_info[8] << 8) | mfw_info[9];
       int fw_version =  (mfw_info[10] << 8) | mfw_info[11];  
       return sprintf(buf,  "SIS-%c%c%c-%c%c%c%c-%d.%d\n", mfw_info[1], mfw_info[2], mfw_info[3],
	   	        mfw_info[4], mfw_info[5], mfw_info[6], mfw_info[7], fw_id, fw_version);
}

static ssize_t sis_touch_switch_state(struct switch_dev *sdev, char *buf){ 
    return sprintf(buf, "1");	
}

#if (CONFIG_HAS_EARLYSUSPEND && TOUCH_POWER_CONTROL) 
static struct early_suspend mEarly_suspend;
static bool mEarly_suspend_registered = false;

static void sis_ts_early_suspend(struct early_suspend *h){
    printk("%s\n", __func__);
    gpio_direction_output(TEGRA_GPIO_PS0, 0);	
}

static void sis_ts_late_resume(struct early_suspend *h){
    printk("%s\n", __func__);
    gpio_direction_output(TEGRA_GPIO_PS0, 1);
}
#endif


static void read_firmware_version(){
    const unsigned char fw_info_cmd[10] = {0x09, 0x09, 0x86, 0x08, 0x04, 0xC0, 0x00, 0xA0, 0x34, 0x00};
    unsigned char buf[64] = {0};
    int actual_length = 0, rep_ret;
    const unsigned int timeout = 1000;
    struct usb_interface *intf = to_usb_interface( hid_dev_backup->dev.parent );
    struct usb_device *dev = interface_to_usbdev( intf );

    if ( hid_dev_backup->product == USB_PRODUCT_ID_SIS817_TOUCH || hid_dev_backup->product == USB_PRODUCT_ID_SIS1012_TOUCH){	//817 internal : endpoint 1 is int_out
        rep_ret = usb_interrupt_msg( dev, usb_sndintpipe( dev, ENDP_01 ),
			  fw_info_cmd, sizeof(fw_info_cmd), &actual_length, timeout );
	 rep_ret = usb_interrupt_msg(dev, usb_rcvintpipe(dev, ENDP_02),
		        buf, sizeof(buf), &actual_length, timeout);
    }else{								//817 bridge : endpoint 2 is int_out
        rep_ret = usb_interrupt_msg( dev, usb_sndintpipe( dev, ENDP_02 ),
			  fw_info_cmd, sizeof(fw_info_cmd), &actual_length, timeout );
        rep_ret = usb_interrupt_msg(dev, usb_rcvintpipe(dev, ENDP_01),
			 buf, sizeof(buf), &actual_length, timeout);

    }
    
    memcpy(mfw_info, buf + 8, 12); 
      
}

static int sis_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
    	struct usb_interface *intf = to_usb_interface(hdev->dev.parent);
    	struct usb_device *dev = interface_to_usbdev(intf);
	struct sis_data *nd;
   	u8 *rep_data = NULL;
	hid_dev_backup = hdev;

	printk(KERN_INFO "sis_probe\n");
	
	backup_urb = usb_alloc_urb(0, GFP_KERNEL); //0721test
	if (!backup_urb) {
		dev_err(&hdev->dev, "cannot allocate backup_urb\n");
		return -ENOMEM;
	}

      if(!mfw_info[0]){
          printk("##########SIS: read firmeare ID and versino.\n");
          read_firmware_version();
	}

//	command Set_Feature for changing device from mouse to touch device
	rep_data = kmalloc(3,GFP_KERNEL);	//return value will be 0xabcd
	if (hdev->product == USB_PRODUCT_ID_SIS9200_TOUCH)
	{
		if(!rep_data)
			return -ENOMEM;
		rep_data[0] = 0x07;
		rep_data[1] = 0x02;
		rep_data[2] = 0xA9;
	
		ret = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), 
			USB_REQ_SET_CONFIGURATION, (USB_DIR_OUT | USB_TYPE_CLASS | 
			USB_RECIP_INTERFACE), 0x0307, 0, rep_data, 3, 1000);
	}

//	allocate memory for sis_data struct
	nd = kzalloc(sizeof(struct sis_data), GFP_KERNEL);
	if (!nd) {
		dev_err(&hdev->dev, "cannot allocate SiS 9200 data\n");
		kfree(rep_data);
		rep_data = NULL;
		return -ENOMEM;
	}
	
	hid_set_drvdata(hdev, nd);

	//for ioctl
	if(sis_char_class == NULL){
	    ret = sis_setup_chardev(hdev, nd);
	    if(ret){
		printk( KERN_INFO "sis_setup_chardev fail\n");
	    }
        sis_touch_sdev.name = "touch";
        sis_touch_sdev.print_name = sis_touch_switch_name;
        sis_touch_sdev.print_state = sis_touch_switch_state;
        if(switch_dev_register(&sis_touch_sdev) < 0){
            printk(KERN_ERR "switch_dev_register for dock failed!\n");
        }
        switch_set_state(&sis_touch_sdev, 0);
#ifdef _ENABLE_DBG_LEVEL
        if(dbgProcFile == NULL){
            dbgProcFile = create_proc_entry(PROC_FS_NAME, 0666, NULL);
	        if (dbgProcFile == NULL) {
		        remove_proc_entry(PROC_FS_NAME, NULL);
		        printk(" Could not initialize /proc/%s\n", PROC_FS_NAME);
	        }else{
		        dbgProcFile->read_proc = sis_proc_read;
		        dbgProcFile->write_proc = sis_proc_write;
		        printk(" /proc/%s created\n", PROC_FS_NAME);
               }
			
	      dbgProcFile_tp= create_proc_entry(PROC_FS_NAME_TP, 0666, NULL);
	        if (dbgProcFile_tp== NULL) {
		        remove_proc_entry(PROC_FS_NAME_TP, NULL);
		        printk(" Could not initialize /proc/%s\n", PROC_FS_NAME_TP);
	        }else{
		        dbgProcFile_tp->read_proc = sis_proc_read_tp;
		        dbgProcFile_tp->write_proc = sis_proc_write_tp;
		        printk(" /proc/%s created\n", PROC_FS_NAME_TP);
               }
        }
#endif // #ifdef _ENABLE_DBG_LEVEL
    }

#if(CONFIG_HAS_EARLYSUSPEND && TOUCH_POWER_CONTROL)
    if(!mEarly_suspend_registered){
        mEarly_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
        mEarly_suspend.suspend = sis_ts_early_suspend;
        mEarly_suspend.resume = sis_ts_late_resume;
        register_early_suspend(&mEarly_suspend);
        mEarly_suspend_registered = true;
    }
#endif


	ret = hid_parse(hdev);
	if (ret) {
		dev_err(&hdev->dev, "parse failed\n");
		goto err_free;
	}

	//set noget for not init reports
	if ( hdev->product != USB_PRODUCT_ID_SIS817_TOUCH || hdev->product != USB_PRODUCT_ID_SIS1012_TOUCH)
	{
		hdev->quirks |= HID_QUIRK_NOGET;
		printk(KERN_INFO "sis:sis-probe: quirk = %d", hdev->quirks);
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		dev_err(&hdev->dev, "hw start failed\n");
		goto err_free;
	}

err_free:
	kfree(rep_data);
	rep_data = NULL;	
	return ret;
}

static void sis_remove(struct hid_device *hdev)
{
	dev_t dev;

	printk(KERN_INFO "sis_remove\n");
	//for ioctl
	if(sis_char_class != NULL){
	    dev = MKDEV(sis_char_major, 0);
	    cdev_del(&sis_char_cdev);
	    unregister_chrdev_region(dev, sis_char_devs_count);
	    device_destroy(sis_char_class, MKDEV(sis_char_major, 0));
	    class_destroy(sis_char_class);
	    switch_dev_unregister(&sis_touch_sdev);
#ifdef _ENABLE_DBG_LEVEL
	    remove_proc_entry(PROC_FS_NAME, NULL);
	    remove_proc_entry(PROC_FS_NAME_TP, NULL);
          dbgProcFile = NULL;
          dbgProcFile_tp = NULL;
#endif
          sis_char_class = NULL;
	}
	
	if(hid_dev_backup){
	    usb_kill_urb( backup_urb );
	    usb_free_urb( backup_urb );
	    backup_urb = NULL;
	    hid_hw_stop(hdev);
          kfree(hid_get_drvdata(hdev));
          hid_set_drvdata(hdev, NULL);
          hid_dev_backup = NULL;
      }
}

static const struct hid_device_id sis_devices[] = {
	//{ HID_USB_DEVICE(USB_VENDOR_ID_SIS9200, USB_DEVICE_ID_SIS9200_TOUCHSCREEN) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS2_TOUCH) },	//0x0457, 0x0151
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS_TOUCH) },		//0x0457, 0x0810
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_NEW_SIS2_TOUCH) },	//0x0457, 0x0816
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS9200_TOUCH) },	//0x0457, 0x9200
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS817_TOUCH) },	//0x0457, 0x0817
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SISF817_TOUCH) },	//0x0457, 0xF817
	{ HID_USB_DEVICE(USB_VENDOR_ID_SIS2_TOUCH, USB_PRODUCT_ID_SIS1012_TOUCH)},     // 0x0457, 0x1012
	{ }
};
MODULE_DEVICE_TABLE(hid, sis_devices);



static struct hid_driver sis_driver = {
	.name = "sis",
	.id_table = sis_devices,
	.probe = sis_probe,
	.remove = sis_remove,
	.raw_event = sis_raw_event,
	.input_mapped = sis_input_mapped,
	.input_mapping = sis_input_mapping,
	.event = sis_event,
};

static int __init sis_init(void)
{
	int ret;
	printk(KERN_INFO "sis_init\n");

	ret = hid_register_driver(&sis_driver);

	if (ret)
		printk(KERN_INFO "sis_init : ret != 0\n");
	else
		printk(KERN_INFO "sis_init : ret = 0\n");

	return ret;
}

static void __exit sis_exit(void)
{
	printk(KERN_INFO "sis_exit\n");
	hid_unregister_driver(&sis_driver);
}

module_init(sis_init);
module_exit(sis_exit);
MODULE_DESCRIPTION("SiS 817 Touchscreen Driver");
MODULE_LICENSE("GPL");
