/*
 *  Atmel maXTouch Touchscreen Controller
 *
 *
 *  Copyright (C) 2010 Atmel Corporation
 *  Copyright (C) 2009 Raphael Derosso Pereira <raphaelpereira@gmail.com>
 *  Copyright (C) 2011 NVIDIA Corporation
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>

#include <linux/uaccess.h>
#include <linux/i2c/atmel_maxtouch.h>
#include <linux/miscdevice.h>

#include <linux/poll.h>
#include <linux/kfifo.h>
#include <linux/version.h>
#include <linux/gpio.h>
#include <../arch/arm/mach-tegra/gpio-names.h>
#include <mach/board-cardhu-misc.h>
#include <linux/wakelock.h>
#include <linux/switch.h>
//Mars debug
#include <linux/proc_fs.h>
/*
 * This is a driver for the Atmel maXTouch Object Protocol
 *
 * When the driver is loaded, mxt_init is called.
 * mxt_driver registers the "mxt_driver" structure in the i2c subsystem
 * The mxt_idtable.name string allows the board support to associate
 * the driver with its own data.
 *
 * The i2c subsystem will call the mxt_driver.probe == mxt_probe
 * to detect the device.
 * mxt_probe will reset the maXTouch device, and then
 * determine the capabilities of the I2C peripheral in the
 * host processor (needs to support BYTE transfers)
 *
 * If OK; mxt_probe will try to identify which maXTouch device it is
 * by calling mxt_identify.
 *
 * If a known device is found, a linux input device is initialized
 * the "mxt" device data structure is allocated,
 * as well as an input device structure "mxt->input"
 * "mxt->client" is provided as a parameter to mxt_probe.
 *
 * mxt_read_object_table is called to determine which objects
 * are present in the device, and to determine their adresses.
 *
 *
 * Addressing an object:
 *
 * The object is located at a 16-bit address in the object address space.
 *
 * The address is provided through an object descriptor table in the beginning
 * of the object address space. This address can change between firmware
 * revisions, so it's important that the driver will make no assumptions
 * about addresses but instead reads the object table and gets the correct
 * addresses there.
 *
 * Each object type can have several instances, and the number of
 * instances is available in the object table as well.
 *
 * The base address of the first instance of an object is stored in
 * "mxt->object_table[object_type].chip_addr",
 * This is indexed by the object type and allows direct access to the
 * first instance of an object.
 *
 * Each instance of an object is assigned a "Report Id" uniquely identifying
 * this instance. Information about this instance is available in the
 * "mxt->report_id" variable, which is a table indexed by the "Report Id".
 *
 * The maXTouch object protocol supports adding a checksum to messages.
 * By setting the most significant bit of the maXTouch address,
 * an 8 bit checksum is added to all writes.
 *
 *
 * How to use driver.
 * -----------------
 * Example:
 * In arch/avr32/boards/atstk1000/atstk1002.c
 * an "i2c_board_info" descriptor is declared.
 * This contains info about which driver ("mXT224"),
 * which i2c address and which pin for CHG interrupts are used.
 *
 * In the "atstk1002_init" routine, "i2c_register_board_info" is invoked
 * with this information. Also, the I/O pins are configured, and the I2C
 * controller registered is on the application processor.
 *
 *
 */
#define WRITE_MEM_OK		0x01
#define WRITE_MEM_FAILED	0x02
#define READ_MEM_OK		0x01
#define READ_MEM_FAILED	0x02

#define FW_WAITING_BOOTLOAD_COMMAND 0xC0
#define FW_WAITING_FRAME_DATA       0x80
#define FW_FRAME_CRC_CHECK          0x02
#define FW_FRAME_CRC_PASS           0x04
#define FW_FRAME_CRC_FAIL           0x03
#define I2C_M_WR 0
#define I2C_MAX_SEND_LENGTH     600
#define MXT_IOC_MAGIC 0xF3
#define MXT_IOC_MAXNR 4
#define MXT_POLL_DATA _IOR(MXT_IOC_MAGIC,2,int )
#define MXT_FW_UPDATE _IOR(MXT_IOC_MAGIC,3,int )
#define MXT_FW_VERSION _IOR(MXT_IOC_MAGIC,4,int)
#define MXT_IOCTL_START_HEAVY 2
#define MXT_IOCTL_START_NORMAL 1
#define MXT_IOCTL_END 0
#define START_NORMAL	(HZ/5)
#define START_HEAVY	(HZ/200)

//Mars_Kao@20120411
#define PROC_FS_NAME	"dbg_x"
#define PROC_FS_MAX_LEN	8
static struct proc_dir_entry *dbgProcFile;
static bool touch_flag = false;

static int poll_mode=0;
struct delayed_work mxt_poll_data_work;
static struct workqueue_struct *sensor_work_queue;
struct i2c_client *mxt_client;
struct mxt_data *globe_mxt=NULL;
u32 fw_checksum;
u8 ver_major;
u8 ver_minor;
u32 cfg_crc;
static int debug = NO_DEBUG;
static int comms;
static unsigned int resume_flag;
static bool delta_flag; 
static bool cfg_flag;
static u8 touch_vendor_id;
static const  u32 touch_panel_config_checksum[2][4] = {
    {
	[HANNSTOUCH_SINTEK_PANEL] = 0x9d3d50, // Hannstar/Sintek
	[TPK_75_OHM_PANEL] = 0x3a66c2,            // TPK 75 ohm
	[TPK_50_OHM_PANEL] = 0x3a66c2,           //  TPK 50 ohm
    }, 
    { // for firmware version 2
	[HANNSTOUCH_SINTEK_PANEL] = 0x361b04, // Hannstar/Sintek
	[TPK_75_OHM_PANEL] = 0x4ba693,            // TPK 75 ohm
	[TPK_50_OHM_PANEL] = 0x3599c4,           //  TPK 50 ohm
    },
};

module_param(debug, int, 0644);
module_param(comms, int, 0644);

MODULE_PARM_DESC(debug, "Activate debugging output");
MODULE_PARM_DESC(comms, "Select communications mode");
static int d_flag=0;

/* Mapping from report id to object type and instance */
struct report_id_map {
	u8 object;
	u8 instance;
/*
 * This is the first report ID belonging to object. It enables us to
 * find out easily the touch number: each touch has different report
 * ID (which are assigned to touches in increasing order). By
 * subtracting the first report ID from current, we get the touch
 * number.
 */
	u8 first_rid;
};

#define USB_NO_Cable 0
#define USB_DETECT_CABLE 1 
#define USB_SHIFT 0
#define AC_SHIFT 1 
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)
unsigned pre_usb_cable_status=0;
unsigned now_usb_cable_status=0;

#define I2C_RETRY_COUNT 5
#define I2C_PAYLOAD_SIZE 254


/* Maps a report ID to an object type (object type number). */
#define	REPORT_ID_TO_OBJECT(rid, mxt)			\
	(((rid) == 0xff) ? 0 : mxt->rid_map[rid].object)

/* Maps a report ID to an object type (string). */
#define	REPORT_ID_TO_OBJECT_NAME(rid, mxt)			\
	object_type_name[REPORT_ID_TO_OBJECT(rid, mxt)]

/* Returns non-zero if given object is a touch object */
#define IS_TOUCH_OBJECT(object) \
	((object == MXT_TOUCH_MULTITOUCHSCREEN_T9) || \
	 (object == MXT_TOUCH_KEYARRAY_T15) ||	\
	 (object == MXT_TOUCH_PROXIMITY_T23) || \
	 (object == MXT_TOUCH_SINGLETOUCHSCREEN_T10) || \
	 (object == MXT_TOUCH_XSLIDER_T11) || \
	 (object == MXT_TOUCH_YSLIDER_T12) || \
	 (object == MXT_TOUCH_XWHEEL_T13) || \
	 (object == MXT_TOUCH_YWHEEL_T14) || \
	 (object == MXT_TOUCH_KEYSET_T31) || \
	 (object == MXT_TOUCH_XSLIDERSET_T32) ? 1 : 0)

#define mxt_debug(level, ...) \
	do { \
		if (debug >= (level)) \
			pr_debug(__VA_ARGS__); \
	} while (0)

static const u8 *object_type_name[] = {
	[0] = "Reserved",
	[5] = "GEN_MESSAGEPROCESSOR_T5",
	[6] = "GEN_COMMANDPROCESSOR_T6",
	[7] = "GEN_POWERCONFIG_T7",
	[8] = "GEN_ACQUIRECONFIG_T8",
	[9] = "TOUCH_MULTITOUCHSCREEN_T9",
	[15] = "TOUCH_KEYARRAY_T15",
	[18] = "SPT_COMMSCONFIG_T18",
	[19] = "SPT_GPIOPWM_T19",
	[20] = "PROCI_GRIPFACESUPPRESSION_T20",
	[22] = "PROCG_NOISESUPPRESSION_T22",
	[23] = "TOUCH_PROXIMITY_T23",
	[24] = "PROCI_ONETOUCHGESTUREPROCESSOR_T24",
	[25] = "SPT_SELFTEST_T25",
	[27] = "PROCI_TWOTOUCHGESTUREPROCESSOR_T27",
	[28] = "SPT_CTECONFIG_T28",
	[37] = "DEBUG_DIAGNOSTICS_T37",
	[38] = "SPT_USER_DATA_T38",
	[40] = "PROCI_GRIPSUPPRESSION_T40",
	[41] = "PROCI_PALMSUPPRESSION_T41",
	[42] = "PROCI_TOUCHSUPPRESSION_T42",
	[43] = "SPT_DIGITIZER_T43",
	[44] = "SPT_MESSAGECOUNT_T44",
	[46] = "SPT_CTECONFIG_T46",
	[47] = "PROCI_STYLUS_T47",
	[48] = "PROCG_NOISESUPPRESSION_T48",
	[52] = "TOUCH_PROXKEY_T52",
	[53] = "GEN_DATASOURCE_T53",
	[55] = "PROCI_ADAPTIVETHRESHOLD_T55",
	[56] = "PROCI_SHIELDLESS_T56", 
	[57] = "PROCI_EXTRATOUCHAREA_T57"
};
typedef struct
{
	u16 size_id;
	u16  pressure;
	u16 x;
	u16 y;
} report_finger_info_struct;
static report_finger_info_struct fingerInfo[10]={0};

//static u16 get_object_address(uint8_t object_type,
//			      uint8_t instance,
//			      struct mxt_object *object_table, int max_objs);

static int mxt_write_ap(struct mxt_data *mxt, u16 ap);

static int mxt_read_block_wo_addr(struct i2c_client *client,
				  u16 length, u8 *value);
/* Routines for memory access within a 16 bit address space */

static int mxt_read_block(struct i2c_client *client, u16 addr, u16 length,
			  u8 *value);
//static int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value);
static int mxt_write_block(struct i2c_client *client, u16 addr, u16 length,
			   u8 *value);
static int mxt_init_Boot(struct mxt_data *mxt);
static int __devinit mxt_read_object_table(struct i2c_client *client,
					   struct mxt_data *mxt,
					   u8 *raw_id_data);
/* TODO: */
/* Bootloader specific function prototypes. */
/*
static int mxt_read_byte_bl(struct i2c_client *client, u8 * value)
	{ return 0; }
static int mxt_read_block_bl(struct i2c_client *client, u16 length,
		u8 * value) { return 0; }
static int mxt_write_byte_bl(struct i2c_client *client, u8 value)
	{ return 0; }
static int mxt_write_block_bl(struct i2c_client *client, u16 length,
		u8 *value) { return 0; }
*/

static u8 mxt_valid_interrupt_dummy(void)
{
	return 1;
}
#define DBG_MODULE	0x00000001
#define DBG_CDEV	0x00000002
#define DBG_PROC	0x00000004
#define DBG_PARSER	0x00000020
#define DBG_SUSP	0x00000040
#define DBG_CONST	0x00000100
#define DBG_IDLE	0x00000200
#define DBG_WAKEUP	0x00000400
static unsigned int DbgLevel = DBG_MODULE|DBG_SUSP| DBG_CDEV|DBG_PROC;
#define TOUCH_DBG(level, fmt, args...)  { if( (level&DbgLevel)>0 ) \
					printk( KERN_DEBUG "[touch_char]: " fmt, ## args); }

struct touch_char_dev
{
	int OpenCnts;
	struct cdev cdev;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	struct kfifo* pCharKFiFo;
	spinlock_t CharFiFoLock;
#else
	struct kfifo CharKFiFo;
#endif
	unsigned char *pFiFoBuf;
	struct semaphore sem;
	wait_queue_head_t fifo_inq;
};
static int global_major = 0; // dynamic major by default 
static int global_minor = 0;
static struct touch_char_dev *p_char_dev = NULL;	// allocated in probe
static atomic_t touch_char_available = ATOMIC_INIT(1);
static atomic_t wait_command_ack = ATOMIC_INIT(0);
static struct class *touch_class;
// ioctl command
#define TOUCH_SELFTEST_CMD  1
// number 2 is curse number. 
#define TOUCH_READ_T28_DATA  3
#define TOUCH_WRITE_T28_CMD  4
#define TOUCH_WRITE_T6_DIAGNOSTIC 5
#define TOUCH_READ_T37_OBJECT  6

#define FIFO_SIZE		PAGE_SIZE
static unsigned int mTouchCharCmd = -1; 

static int touch_cdev_open(struct inode *inode, struct file *filp)
{
	struct touch_char_dev *cdev;
       /*
       if(p_touch_serial_dev == NULL){
	     printk("[touch_char]: No touch char device!\n");
	     return -ENODEV;	
	}
	*/     
	   
	cdev = container_of(inode->i_cdev, struct touch_char_dev, cdev);
	if( cdev == NULL )
	{
        	TOUCH_DBG(DBG_CDEV, " No such char device node \n");
		return -ENODEV;
	}
	
	if( !atomic_dec_and_test(&touch_char_available) )
	{
		atomic_inc(&touch_char_available);
		return -EBUSY; /* already open */
	}

	cdev->OpenCnts++;
	filp->private_data = cdev;// Used by the read and write metheds
	TOUCH_DBG(DBG_CDEV, " CDev open done!\n");
	try_module_get(THIS_MODULE);
	return 0;
}

static int touch_cdev_release(struct inode *inode, struct file *filp)
{
	struct touch_char_dev *cdev; // device information

	cdev = container_of(inode->i_cdev, struct touch_char_dev, cdev);
        if( cdev == NULL )
        {
                TOUCH_DBG(DBG_CDEV, " No such char device node \n");
                return -ENODEV;
        }

	atomic_inc(&touch_char_available); /* release the device */

	filp->private_data = NULL;
	cdev->OpenCnts--;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	kfifo_reset( cdev->pCharKFiFo );
#else
	kfifo_reset( &cdev->CharKFiFo );
#endif
	TOUCH_DBG(DBG_CDEV, " CDev release done!\n");
	module_put(THIS_MODULE);
	return 0;
}

#define MAX_READ_BUF_LEN	50
static char fifo_read_buf[MAX_READ_BUF_LEN];
static ssize_t touch_cdev_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	int read_cnt, ret, fifoLen;
	struct touch_char_dev *cdev = file->private_data;
	 u8 temp[130];
	
	if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;

	if(mTouchCharCmd > 0){
	    switch(mTouchCharCmd){
	    case TOUCH_READ_T28_DATA:
	              mxt_read_block(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, globe_mxt), 6, temp);
	              TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_READ_T28_DATA\n");
	              ret = copy_to_user(buf, temp, 6) ? -EFAULT : 6;
			 mTouchCharCmd = 0;
			 up(&cdev->sem);
	              return ret;
	    case TOUCH_READ_T37_OBJECT:
	             mxt_read_block(globe_mxt->client, MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, globe_mxt), 130, temp);
			TOUCH_DBG(DBG_CDEV, " Execute command: MXT_DEBUG_DIAGNOSTIC_T37\n");
			ret = copy_to_user(buf, temp, 130) ? -EFAULT : 130;
			mTouchCharCmd = 0;
			up(&cdev->sem);
			return ret;
	    }
	}   
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	fifoLen = kfifo_len(cdev->pCharKFiFo);
#else
	fifoLen = kfifo_len(&cdev->CharKFiFo);
#endif

	while( fifoLen<1 ) /* nothing to read */
	{
		up(&cdev->sem); /* release the lock */
		if( file->f_flags & O_NONBLOCK )
			return -EAGAIN;
	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
		if( wait_event_interruptible(cdev->fifo_inq, kfifo_len( cdev->pCharKFiFo )>0) )
	#else
		if( wait_event_interruptible(cdev->fifo_inq, kfifo_len( &cdev->CharKFiFo )>0) )
	#endif
		{
			return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
		}
		if( down_interruptible(&cdev->sem) )
			return -ERESTARTSYS;

	#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	      fifoLen = kfifo_len(cdev->pCharKFiFo);
      #else
	      fifoLen = kfifo_len(&cdev->CharKFiFo);
      #endif
	}

	if(count > MAX_READ_BUF_LEN)
		count = MAX_READ_BUF_LEN;

	TOUCH_DBG(DBG_CDEV, " \"%s\" reading: real fifo data\n", current->comm);
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	read_cnt = kfifo_get(cdev->pCharKFiFo, fifo_read_buf, count);
#else
	read_cnt = kfifo_out(&cdev->CharKFiFo, fifo_read_buf, count);
#endif

	ret = copy_to_user(buf, fifo_read_buf, read_cnt)?-EFAULT:read_cnt;

	up(&cdev->sem);
	
	return ret;
}

static ssize_t touch_cdev_write(struct file *file, const char __user *buf, size_t count, loff_t *offset)
{
	struct touch_char_dev *cdev = file->private_data;
	int ret=0, written=0;
	unsigned char c;
      
	if( down_interruptible(&cdev->sem) )
		return -ERESTARTSYS;
	TOUCH_DBG(DBG_CDEV, "Start Write the SIGLIM value.\n", written);
	if(globe_mxt == NULL) 
	{
		ret = -ENODEV;
		goto out;
	}

	if(count > 4) // for four byte T25 HI/LO  SIGLIM 
		count = 4;
	
	while(count--) {
		if(get_user(c, buf++)) 
		{
			ret = -EFAULT;
			goto out;
		}
	      // write T25 HISIGLIM/LOSIGLIM
	      TOUCH_DBG(DBG_CDEV, "T25[%d] =  0X%02X\n", (written + 2), c);
		if(mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, globe_mxt) + 2+ written , c)) 
		{
			ret = -EIO;
			goto out;
		}
		written++;
	};

out:
	up(&cdev->sem);
	TOUCH_DBG(DBG_CDEV, " SIGLIM writing %d bytes.\n", written);

	if(ret!=0)
		return ret;
	else
		return written;
}




#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static int touch_cdev_ioctl (struct inode *inode, struct file *file, unsigned int cmd, unsigned long args)
{	
	struct touch_char_dev *cdev = file->private_data;
	int ret=0;
	TOUCH_DBG(DBG_CDEV, " Handle device ioctl command version 36\n");  
	if(globe_mxt == NULL)
		return -EFAULT;
      
	u8 cmd_code = args & 0xFFUL;  
	switch (cmd)
	{
		case TOUCH_SELFTEST_CMD:
			// start the test
			//mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, globe_mxt), 0);
	             //msleep(25);
			 mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, globe_mxt) + 0, 0x03);
			 mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, globe_mxt) + 1, cmd_code);
		       TOUCH_DBG(DBG_CDEV, " Start the self test with byte: %X\n", cmd_code);
			break;
		/*
		case TOUCH_READ_T28_DATA:
			mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, globe_mxt), 0x0);
			TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_READ_T28_DATA\n");
			break;
		case TOUCH_WRITE_T28_CMD:
			mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, globe_mxt) + 1, cmd_code);
			TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_WRITE_T28_CMD 0x%02X\n", cmd_code);
			break;
		*/
		case TOUCH_WRITE_T6_DIAGNOSTIC:
			mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, globe_mxt) + MXT_ADR_T6_DIAGNOSTIC, cmd_code);
			TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_WRITE_T6_DIAGNOSTIC 0x%02X\n", cmd_code);
			break;
		case TOUCH_READ_T37_OBJECT:
			TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_READ_T37_OBJECT\n");
			break;
		default:
			ret = -ENOTTY;
			break;
	}
       mTouchCharCmd = cmd;
	return ret;
}

#else
static int touch_cdev_ioctl (struct file *file, unsigned int cmd, unsigned long args)
{	
	struct touch_char_dev *cdev = file->private_data;
	int ret=0;
	TOUCH_DBG(DBG_CDEV, " Handle device ioctl command version 36\n");  
	if(globe_mxt == NULL)
		return -EFAULT;
      
	u8 cmd_code = args & 0xFFUL;  
	switch (cmd)
	{
		case TOUCH_SELFTEST_CMD:
			// start the test
			//mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, globe_mxt), 0);
	             //msleep(25);
			 mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, globe_mxt) + 0, 0x03);
			 mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_SELFTEST_T25, globe_mxt) + 1, cmd_code);
		       TOUCH_DBG(DBG_CDEV, " Start the self test with byte: %X\n", cmd_code);
			break;
		case TOUCH_READ_T28_DATA:
			mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, globe_mxt), 0x0);
			TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_READ_T28_DATA\n");
			break;
		case TOUCH_WRITE_T28_CMD:
			mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T28, globe_mxt) + 1, cmd_code);
			TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_WRITE_T28_CMD 0x%02X\n", cmd_code);
			break;
		case TOUCH_WRITE_T6_DIAGNOSTIC:
			mxt_write_byte(globe_mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, globe_mxt) + MXT_ADR_T6_DIAGNOSTIC, cmd_code);
			TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_WRITE_T6_DIAGNOSTIC 0x%02X\n", cmd_code);
			break;
		case TOUCH_READ_T37_OBJECT:
			TOUCH_DBG(DBG_CDEV, " Execute command: TOUCH_READ_T37_OBJECT\n");
			break;
		default:
			ret = -ENOTTY;
			break;
	}
       mTouchCharCmd = cmd;
	return ret;
}
#endif

static unsigned int touch_cdev_poll(struct file *filp, struct poll_table_struct *wait)
{
	struct touch_char_dev *cdev = filp->private_data;
	unsigned int mask = 0;
	int fifoLen;
	
	down(&cdev->sem);
	poll_wait(filp, &cdev->fifo_inq,  wait);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	fifoLen = kfifo_len(cdev->pCharKFiFo);
#else
	fifoLen = kfifo_len(&cdev->CharKFiFo);
#endif

	if( fifoLen > 0 )
		mask |= POLLIN | POLLRDNORM;    /* readable */
	if( (FIFO_SIZE - fifoLen) > 0 )
		mask |= POLLOUT | POLLWRNORM;   /* writable */

	up(&cdev->sem);
	return mask;
}


static const struct file_operations touch_cdev_fops = {
	.owner	= THIS_MODULE,
	.read	= touch_cdev_read,
	.write	= touch_cdev_write,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
      .ioctl=touch_cdev_ioctl,
#else 
      .unlocked_ioctl=touch_cdev_ioctl, 
#endif
	.poll	= touch_cdev_poll,
	.open	= touch_cdev_open,
	.release= touch_cdev_release,
};

static struct touch_char_dev* setup_chardev(dev_t dev)
{
	struct touch_char_dev *pCharDev;
	int result;

	pCharDev = kmalloc(1*sizeof(struct touch_char_dev), GFP_KERNEL);
	if(!pCharDev) 
		goto fail_cdev;
	memset(pCharDev, 0, sizeof(struct touch_char_dev));

	pCharDev->pFiFoBuf = kmalloc(sizeof(unsigned char)*FIFO_SIZE, GFP_KERNEL);
	if(!pCharDev->pFiFoBuf)
		goto fail_fifobuf;
	memset(pCharDev->pFiFoBuf, 0, sizeof(unsigned char)*FIFO_SIZE);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
	spin_lock_init( &pCharDev->CharFiFoLock );
	pCharDev->pCharKFiFo = kfifo_init(pCharDev->pFiFoBuf, FIFO_SIZE, GFP_KERNEL, &pCharDev->CharFiFoLock);
	if( pCharDev->pCharKFiFo==NULL )
		goto fail_kfifo;
#else
	kfifo_init(&pCharDev->CharKFiFo, pCharDev->pFiFoBuf, FIFO_SIZE);
	if( !kfifo_initialized(&pCharDev->CharKFiFo) )
		goto fail_kfifo;
#endif
	
	pCharDev->OpenCnts = 0;
	cdev_init(&pCharDev->cdev, &touch_cdev_fops);
	pCharDev->cdev.owner = THIS_MODULE;
	sema_init(&pCharDev->sem, 1);
	init_waitqueue_head(&pCharDev->fifo_inq);

	result = cdev_add(&pCharDev->cdev, dev, 1);
	if(result)
	{
		TOUCH_DBG(DBG_MODULE, " Failed at cdev added\n");
		goto fail_kfifo;
	}

	return pCharDev; 

fail_kfifo:
	kfree(pCharDev->pFiFoBuf);
fail_fifobuf:
	kfree(pCharDev);
fail_cdev:
	return NULL;
}

static void exit_touch_char_dev(void)
{
	dev_t devno = MKDEV(global_major, global_minor);
	
	TOUCH_DBG(DBG_MODULE, " Exit driver ...\n");

	if(p_char_dev)
	{
		if( p_char_dev->pFiFoBuf )
			kfree(p_char_dev->pFiFoBuf);
	
		cdev_del(&p_char_dev->cdev);
		kfree(p_char_dev);
		p_char_dev = NULL;
	}

	unregister_chrdev_region( devno, 1);

	if(!IS_ERR(touch_class))
	{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
		class_device_destroy(touch_class, devno);
#else
		device_destroy(touch_class, devno);
#endif 
		class_destroy(touch_class);
	}
/*
	if(input_dev)
	{
		input_unregister_device(input_dev);
		input_dev = NULL;
	}

	serio_unregister_driver(&touch_serio_drv);
*/
	TOUCH_DBG(DBG_MODULE, " Exit driver done!\n");
}

static int init_touch_char_dev(void){
       int result;
	dev_t devno = 0;

	TOUCH_DBG(DBG_MODULE, " Driver init ...\n");

	// Asking for a dynamic major unless directed otherwise at load time.
	if(global_major) 
	{
		devno = MKDEV(global_major, global_minor);
		result = register_chrdev_region(devno, 1, "touch_debug");
	} 
	else 
	{
		result = alloc_chrdev_region(&devno, global_minor, 1, "touch_debug");
		global_major = MAJOR(devno);
	}

	if (result < 0)
	{
		TOUCH_DBG(DBG_MODULE, " Cdev can't get major number\n");
		return 0;
	}

	// allocate the character device
	p_char_dev = setup_chardev(devno);
	if(!p_char_dev) 
	{
		result = -ENOMEM;
		goto fail;
	}

	touch_class = class_create(THIS_MODULE, "touch_debug");
	if(IS_ERR(touch_class))
	{
		TOUCH_DBG(DBG_MODULE, " Failed in creating class.\n");
		result = -EFAULT;
		goto fail;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
	class_device_create(touch_class, NULL, devno, NULL, "touch_debug");
#else
	device_create(touch_class, NULL, devno, NULL, "touch_debug");
#endif
	TOUCH_DBG(DBG_MODULE, " Register touch_debug cdev, major: %d \n",global_major);

      TOUCH_DBG(DBG_MODULE, " Driver init done!\n");
	return 0;
      fail:	
	exit_touch_char_dev();
	return result;
}

static ssize_t store_d_print(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	int print_flag;

	sscanf(buf, "%d\n",&print_flag);
	d_flag=print_flag;

	return count;
}

static ssize_t show_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", data->status);
}

static ssize_t dump_T7(struct device *dev, struct device_attribute *devattr, char *buf)
{
	int i;
	int err;
	u8 tmp[34];
	char tmpstr[100];
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

	sprintf(buf,"");

	err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, data), 3, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T7 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		for (i=0; i<=2; i++){
			sprintf(tmpstr,"T7 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, data), 10, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T8 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		for (i=0; i<=9; i++){
			sprintf(tmpstr,"T8 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

	err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, data), 35, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T9 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		for (i=0; i<=34; i++){
			sprintf(tmpstr,"T9 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_TOUCH_KEYARRAY_T15, data), 11, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T15 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		for (i=0; i< 11; i++){
			sprintf(tmpstr,"T15 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}

       err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_TOUCH_PROXKEY_T52, data), 15, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T52 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		for (i=0; i < 15; i++){
			sprintf(tmpstr,"T52 byte[%d] = %d\n",i,tmp[i]);
			strncat (buf,tmpstr,strlen(tmpstr));
		}
	}
	  
	err = mxt_read_block(data->client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE,(u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read Family ID error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	else{
		sprintf(tmpstr, "Family ID is %d configuration checksum is %lx\n",tmp[0], data->configuration_crc);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

	return strlen(buf);
}
static ssize_t dump_T48(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	int i;
	int err;
	u8 tmp[74];
	char tmpstr[100];
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);

	sprintf(buf,"");

      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, data), 2, (u8 *) tmp);
      if (err < 0){
		sprintf(tmpstr, "read T18 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else	  
	for (i=0; i< 2; i++){
		sprintf(tmpstr,"T18 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_GPIOPWM_T19, data), 16, (u8 *) tmp);
      if (err < 0){
		sprintf(tmpstr, "read T19 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else	  
	for (i=0; i< 16; i++){
		sprintf(tmpstr,"T19 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	  
      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24, data), 19, (u8 *) tmp);
      if (err < 0){
		sprintf(tmpstr, "read T24 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else	  
	for (i=0; i< 19; i++){
		sprintf(tmpstr,"T24 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27, data), 7, (u8 *) tmp);
      if (err < 0){
		sprintf(tmpstr, "read T27 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else	  
	for (i=0; i< 7; i++){
		sprintf(tmpstr,"T27 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_GRIPSUPPRESSION_T40, data), 5, (u8 *) tmp);
      if (err < 0){
		sprintf(tmpstr, "read T40 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else	  
	for (i=0; i< 5; i++){
		sprintf(tmpstr,"T40 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
			  
      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, data), 8, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T42 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else  
	for (i=0; i< 10; i++){
		sprintf(tmpstr,"T42 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_DIGITIZER_T43, data), 7, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T43 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else  
	for (i=0; i< 7; i++){
		sprintf(tmpstr,"T43 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	  
      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_CTECONFIG_T46, data), 9, (u8 *) tmp);
      if (err < 0){
		sprintf(tmpstr, "read T46 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else	  
	for (i=0; i< 9; i++){
		sprintf(tmpstr,"T46 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}

      err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCI_STYLUS_T47, data), 10, (u8 *) tmp);
      if (err < 0){
		sprintf(tmpstr, "read T47 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else	  
	for (i=0; i< 10; i++){
		sprintf(tmpstr,"T47 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	} 
	  
	err = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, data), 74, (u8 *) tmp);
	if (err < 0){
		sprintf(tmpstr, "read T48 cfg error, ret %d\n",err);
		strncat (buf,tmpstr,strlen(tmpstr));
	}else
	for (i=0; i< 74; i++){
		sprintf(tmpstr,"T48 byte[%d] = %d\n",i,tmp[i]);
		strncat (buf,tmpstr,strlen(tmpstr));
	}
	
	return strlen(buf);

}

static ssize_t show_FW_version(struct device *dev, struct device_attribute *devattr, char *buf)
{
	return sprintf(buf, "Touch Firmware Version: %d.%d, checksum is %d\n", ver_major, ver_minor, fw_checksum);
}

static ssize_t store_mode2(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data*data = i2c_get_clientdata(client);
	int cfg[3];
	char *pch;

	sscanf(buf, "%d%d%d\n",&cfg[0], &cfg[1], &cfg[2]);
		
	mxt_write_byte(data->client, MXT_BASE_ADDR(cfg[0], data)+cfg[1],cfg[2]);

	printk("Touch: cfg[0]=%d, cfg[1]=%d, cfg[2]=%d\n",cfg[0],cfg[1],cfg[2]);
	
	mxt_write_byte(data->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, data) + MXT_ADR_T6_BACKUPNV,MXT_CMD_T6_BACKUP);
	return count;
}
DEVICE_ATTR(cfg_ep101, S_IRUGO | S_IWUSR, NULL, store_mode2);
DEVICE_ATTR(d_print, S_IRUGO | S_IWUSR, NULL, store_d_print);
DEVICE_ATTR(atmel_touchpanel_status, 0755, show_status, NULL);
DEVICE_ATTR(FW_version, 0755, show_FW_version, NULL);
DEVICE_ATTR(dump_T7, 0755, dump_T7, NULL);
DEVICE_ATTR(dump_T48_ep101, 0755, dump_T48, NULL);
static struct attribute *mxt_attr[] = {
	&dev_attr_d_print.attr,
	&dev_attr_atmel_touchpanel_status.attr,
	&dev_attr_dump_T7.attr,
	&dev_attr_dump_T48_ep101.attr,
	&dev_attr_FW_version.attr,
	&dev_attr_cfg_ep101.attr,
	NULL
};
ssize_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count,
			loff_t *ppos, u8 debug_command)
{
	int i;
	u16 *data;
	u16 diagnostics_reg;
	int offset = 0;
	int size;
	int read_size;
	int error;
	char *buf_start;
	u16 debug_data_addr;
	u16 page_address;
	u8 page;
	u8 debug_command_reg;

	data = mxt->debug_data;
	if (data == NULL)
		return -EIO;

	/* If first read after open, read all data to buffer. */
	if (mxt->current_debug_datap == 0) {

		diagnostics_reg = MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6,
						mxt) + MXT_ADR_T6_DIAGNOSTIC;
		if (count > (mxt->device_info.num_nodes * 2))
			count = mxt->device_info.num_nodes;

		debug_data_addr = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
		    MXT_ADR_T37_DATA;
		page_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
		    MXT_ADR_T37_PAGE;
		error = mxt_read_block(mxt->client, page_address, 1, &page);
		if (error < 0)
			return error;
		mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		while (page != 0) {
			error = mxt_write_byte(mxt->client,
					       diagnostics_reg,
					       MXT_CMD_T6_PAGE_DOWN);
			if (error < 0)
				return error;
			/* Wait for command to be handled; when it has, the
			   register will be cleared. */
			debug_command_reg = 1;
			while (debug_command_reg != 0) {
				error = mxt_read_block(mxt->client,
						       diagnostics_reg, 1,
						       &debug_command_reg);
				if (error < 0)
					return error;
				mxt_debug(DEBUG_TRACE,
					  "Waiting for debug diag command "
					  "to propagate...\n");

			}
			error = mxt_read_block(mxt->client, page_address, 1,
					       &page);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		}

		/*
		 * Lock mutex to prevent writing some unwanted data to debug
		 * command register. User can still write through the char
		 * device interface though. TODO: fix?
		 */

		mutex_lock(&mxt->debug_mutex);
		/* Configure Debug Diagnostics object to show deltas/refs */
		error = mxt_write_byte(mxt->client, diagnostics_reg,
				       debug_command);

		/* Wait for command to be handled; when it has, the
		 * register will be cleared. */
		debug_command_reg = 1;
		while (debug_command_reg != 0) {
			error = mxt_read_block(mxt->client,
					       diagnostics_reg, 1,
					       &debug_command_reg);
			if (error < 0)
				return error;
			mxt_debug(DEBUG_TRACE, "Waiting for debug diag command "
				  "to propagate...\n");

		}

		if (error < 0) {
			printk(KERN_WARNING
			       "Error writing to maXTouch device!\n");
			return error;
		}

		size = mxt->device_info.num_nodes * sizeof(u16);

		while (size > 0) {
			read_size = size > 128 ? 128 : size;
			mxt_debug(DEBUG_TRACE,
				  "Debug data read loop, reading %d bytes...\n",
				  read_size);
			error = mxt_read_block(mxt->client,
					       debug_data_addr,
					       read_size,
					       (u8 *) &data[offset]);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error reading debug data\n");
				goto error;
			}
			offset += read_size / 2;
			size -= read_size;

			/* Select next page */
			error = mxt_write_byte(mxt->client, diagnostics_reg,
					       MXT_CMD_T6_PAGE_UP);
			if (error < 0) {
				printk(KERN_WARNING
				       "Error writing to maXTouch device!\n");
				goto error;
			}
		}
		mutex_unlock(&mxt->debug_mutex);
	}

	buf_start = buf;
	i = mxt->current_debug_datap;

	while (((buf - buf_start) < (count - 6)) &&
	       (i < mxt->device_info.num_nodes)) {

		mxt->current_debug_datap++;
		if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
			buf += sprintf(buf, "%d: %5d\n", i,
				       (u16) le16_to_cpu(data[i]));
		else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
			buf += sprintf(buf, "%d: %5d\n", i,
				       (s16) le16_to_cpu(data[i]));
		i++;
	}

	return buf - buf_start;
 error:
	mutex_unlock(&mxt->debug_mutex);
	return error;
}

ssize_t deltas_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_DELTAS_MODE);
}

ssize_t refs_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	return debug_data_read(file->private_data, buf, count, ppos,
			       MXT_CMD_T6_REFERENCES_MODE);
}

int debug_data_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	int i;
	mxt = inode->i_private;
	if (mxt == NULL)
		return -EIO;

      if( !atomic_dec_and_test(&touch_char_available) )
	{
		atomic_inc(&touch_char_available);
		return -EBUSY; /* already open */
	}
	
	mxt->current_debug_datap = 0;
	mxt->debug_data = kmalloc(mxt->device_info.num_nodes * sizeof(u16),
				  GFP_KERNEL);
	if (mxt->debug_data == NULL)
		return -ENOMEM;

	for (i = 0; i < mxt->device_info.num_nodes; i++)
		mxt->debug_data[i] = 7777;

	file->private_data = mxt;
	return 0;
}

int debug_data_release(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = file->private_data;
	kfree(mxt->debug_data);
	
	atomic_inc(&touch_char_available); /* release the device */
	return 0;
}

const struct file_operations delta_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = deltas_read,
};

const struct file_operations refs_fops = {
	.owner = THIS_MODULE,
	.open = debug_data_open,
	.release = debug_data_release,
	.read = refs_read,
};

int mxt_memory_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = container_of(inode->i_cdev, struct mxt_data, cdev);
	if (mxt == NULL)
		return -EIO;
	file->private_data = mxt;
	return 0;
}

int mxt_message_open(struct inode *inode, struct file *file)
{
	struct mxt_data *mxt;
	mxt = container_of(inode->i_cdev, struct mxt_data, cdev_messages);
	if (mxt == NULL)
		return -EIO;
	file->private_data = mxt;
	return 0;
}

ssize_t mxt_memory_read(struct file *file, char *buf, size_t count,
			loff_t *ppos)
{
	int i;
	struct mxt_data *mxt;

	mxt = file->private_data;
	if (mxt->valid_ap) {
		mxt_debug(DEBUG_TRACE, "Reading %d bytes from current ap\n",
			  (int)count);
		i = mxt_read_block_wo_addr(mxt->client, count, (u8 *) buf);
	} else {
		mxt_debug(DEBUG_TRACE, "Address pointer changed since set;"
			  "writing AP (%d) before reading %d bytes",
			  mxt->address_pointer, (int)count);
		i = mxt_read_block(mxt->client, mxt->address_pointer, count,
				   buf);
	}

	return i;
}

ssize_t mxt_memory_write(struct file *file, const char *buf, size_t count,
			 loff_t *ppos)
{
	int i;
	int whole_blocks;
	int last_block_size;
	struct mxt_data *mxt;
	u16 address;

	mxt = file->private_data;
	address = mxt->address_pointer;

	mxt_debug(DEBUG_TRACE, "mxt_memory_write entered\n");
	whole_blocks = count / I2C_PAYLOAD_SIZE;
	last_block_size = count % I2C_PAYLOAD_SIZE;

	for (i = 0; i < whole_blocks; i++) {
		mxt_debug(DEBUG_TRACE, "About to write to %d...", address);
		mxt_write_block(mxt->client, address, I2C_PAYLOAD_SIZE,
				(u8 *) buf);
		address += I2C_PAYLOAD_SIZE;
		buf += I2C_PAYLOAD_SIZE;
	}

	mxt_write_block(mxt->client, address, last_block_size, (u8 *) buf);

	return count;
}

long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
static long mxt_ioctl(struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	int retval;
	struct mxt_data *mxt;

	retval = 0;
	mxt = file->private_data;

	switch (cmd) {
	case MXT_SET_ADDRESS:
		retval = mxt_write_ap(mxt, (u16) arg);
		if (retval >= 0) {
			mxt->address_pointer = (u16) arg;
			mxt->valid_ap = 1;
		}
		break;
	case MXT_RESET:
		retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_RESET, 1);
		break;
	case MXT_CALIBRATE:
		retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_CALIBRATE, 1);

		break;
	case MXT_BACKUP:
		retval = mxt_write_byte(mxt->client,
					MXT_BASE_ADDR
					(MXT_GEN_COMMANDPROCESSOR_T6,
					 mxt) + MXT_ADR_T6_BACKUPNV,
					MXT_CMD_T6_BACKUP);
		break;
	case MXT_NONTOUCH_MSG:
		mxt->nontouch_msg_only = 1;
		break;
	case MXT_ALL_MSG:
		mxt->nontouch_msg_only = 0;
		break;
	default:
		return (long)(-EIO);
	}

	return (long)retval;
}

/*
 * Copies messages from buffer to user space.
 *
 * NOTE: if less than (mxt->message_size * 5 + 1) bytes requested,
 * this will return 0!
 *
 */
ssize_t mxt_message_read(struct file *file, char *buf, size_t count,
			 loff_t *ppos)
{
	int i;
	struct mxt_data *mxt;
	char *buf_start;

	mxt = file->private_data;
	if (mxt == NULL)
		return -EIO;
	buf_start = buf;

	mutex_lock(&mxt->msg_mutex);
	/* Copy messages until buffer empty, or 'count' bytes written */
	while ((mxt->msg_buffer_startp != mxt->msg_buffer_endp) &&
	       ((buf - buf_start) < (count - 5 * mxt->message_size - 1))) {

		for (i = 0; i < mxt->message_size; i++) {
			buf += sprintf(buf, "[%2X] ",
				       *(mxt->messages + mxt->msg_buffer_endp *
					 mxt->message_size + i));
		}
		buf += sprintf(buf, "\n");
		if (mxt->msg_buffer_endp < MXT_MESSAGE_BUFFER_SIZE)
			mxt->msg_buffer_endp++;
		else
			mxt->msg_buffer_endp = 0;
	}
	mutex_unlock(&mxt->msg_mutex);
	return buf - buf_start;
}

const struct file_operations mxt_message_fops = {
	.owner = THIS_MODULE,
	.open = mxt_message_open,
	.read = mxt_message_read,
};

const struct file_operations mxt_memory_fops = {
	.owner = THIS_MODULE,
	.open = mxt_memory_open,
	.read = mxt_memory_read,
	.write = mxt_memory_write,
	.unlocked_ioctl = mxt_ioctl,
};

/* Writes the address pointer (to set up following reads). */

int mxt_write_ap(struct mxt_data *mxt, u16 ap)
{
	struct i2c_client *client;
	__le16 le_ap = cpu_to_le16(ap);
	client = mxt->client;
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	if (i2c_master_send(client, (u8 *) &le_ap, 2) == 2) {
		mxt_debug(DEBUG_TRACE, "Address pointer set to %d\n", ap);
		return 0;
	} else {
		mxt_debug(DEBUG_INFO, "Error writing address pointer!\n");
		return -EIO;
	}
}

/* Calculates the 24-bit CRC sum. */
static u32 CRC_24(u32 crc, u8 byte1, u8 byte2)
{
	static const u32 crcpoly = 0x80001B;
	u32 result;
	u32 data_word;

	data_word = ((((u16) byte2) << 8u) | byte1);
	result = ((crc << 1u) ^ data_word);
	if (result & 0x1000000)
		result ^= crcpoly;
	return result;
}

/* Returns object address in mXT chip, or zero if object is not found */
u16 get_object_address(uint8_t object_type,
			      uint8_t instance,
			      struct mxt_object *object_table, int max_objs)
{
	uint8_t object_table_index = 0;
	uint8_t address_found = 0;
	uint16_t address = 0;
	struct mxt_object *obj;

	while ((object_table_index < max_objs) && !address_found) {
		obj = &object_table[object_table_index];
		if (obj->type == object_type) {
			address_found = 1;
			/* Are there enough instances defined in the FW? */
			if (obj->instances >= instance) {
				address = obj->chip_addr +
				    (obj->size + 1) * instance;
			} else {
				return 0;
			}
		}
		object_table_index++;
	}
	return address;
}

/* Return the pointer of mxt object*/
const struct mxt_object* get_object(uint8_t object_type, 
	                 const struct mxt_data *mxt){
    uint8_t object_table_index = 0;
    int max_objs = mxt->device_info.num_objs;
    struct mxt_object *object_table = mxt->object_table;
    struct mxt_object *obj;

    while(object_table_index < max_objs){
        obj = &object_table[object_table_index];
        if(obj->type == object_type)
            return obj; 
		
        object_table_index++;
    }	
	
    return NULL; 
}

/*
 * Reads a block of bytes from given address from mXT chip. If we are
 * reading from message window, and previous read was from message window,
 * there's no need to write the address pointer: the mXT chip will
 * automatically set the address pointer back to message window start.
 */

static int mxt_read_block(struct i2c_client *client,
			  u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg msg[2];
	__le16 le_addr;
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	if (mxt != NULL) {
		if ((mxt->last_read_addr == addr) &&
		    (addr == mxt->msg_proc_addr)) {
			if (i2c_master_recv(client, value, length) == length)
				return length;
			else
				return -EIO;
		} else {
			mxt->last_read_addr = addr;
		}
	}

	mxt_debug(DEBUG_TRACE, "Writing address pointer & reading %d bytes "
		  "in on i2c transaction...\n", length);

	le_addr = cpu_to_le16(addr);
	msg[0].addr = client->addr;
	msg[0].flags = 0x00;
	msg[0].len = 2;
	msg[0].buf = (u8 *) &le_addr;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = length;
	msg[1].buf = (u8 *) value;
	if (i2c_transfer(adapter, msg, 2) == 2)
		return length;
	else
		return -EIO;

}

/* Reads a block of bytes from current address from mXT chip. */

static int mxt_read_block_wo_addr(struct i2c_client *client,
				  u16 length, u8 *value)
{

	if (i2c_master_recv(client, value, length) == length) {
		mxt_debug(DEBUG_TRACE, "I2C block read ok\n");
		return length;
	} else {
		mxt_debug(DEBUG_INFO, "I2C block read failed\n");
		return -EIO;
	}

}

/* Writes one byte to given address in mXT chip. */

int mxt_write_byte(struct i2c_client *client, u16 addr, u8 value)
{
	struct {
		__le16 le_addr;
		u8 data;

	} i2c_byte_transfer;

	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	i2c_byte_transfer.le_addr = cpu_to_le16(addr);
	i2c_byte_transfer.data = value;
	if (i2c_master_send(client, (u8 *) &i2c_byte_transfer, 3) == 3)
		return 0;
	else
		return -EIO;
}

static int mxt_write_byte_bl(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	struct i2c_adapter *adapter = client->adapter;
	struct i2c_msg wmsg;
	//unsigned char wbuf[3];
	unsigned char data[I2C_MAX_SEND_LENGTH];
	int ret,i;

//	printk("Touch: length = %d\n",length);
	if(length+2 > I2C_MAX_SEND_LENGTH)
	{
		printk("[TSP][ERROR] %s() data length error\n", __FUNCTION__);
		return -ENODEV;
	}

	wmsg.addr = 0x27;
	wmsg.flags = I2C_M_WR;
	wmsg.len = length;
	wmsg.buf = data;

	for (i = 0; i < length; i++)
	{
		data[i] = *(value+i);
	}

	//	printk("%s, %s\n",__func__,wbuf);
	ret = i2c_transfer(adapter, &wmsg, 1);
	return ret;

}

static int mxt_write_mem_bl(struct i2c_client *client, u16 start, u16 size, u8 *mem)
{
	int ret;

	ret = mxt_write_byte_bl(client, start, size, mem);
	if(ret < 0){
		printk("boot write mem fail: %d \n",ret);
		return(WRITE_MEM_FAILED);
	}
	else
		return(WRITE_MEM_OK);
}

static int mxt_read_mem_bl(struct i2c_client *client, u16 start, u16 size, u8 *mem)
{
	struct i2c_msg rmsg;
	int ret;

	rmsg.addr=0x27;
	rmsg.flags = I2C_M_RD;
	rmsg.len = size;
	rmsg.buf = mem;
	ret = i2c_transfer(client->adapter, &rmsg, 1);

	return ret;

}

/* Writes a block of bytes (max 256) to given address in mXT chip. */
static int mxt_write_block(struct i2c_client *client,
			   u16 addr, u16 length, u8 *value)
{
	int i;
	struct {
		__le16 le_addr;
		u8 data[256];

	} i2c_block_transfer;

	struct mxt_data *mxt;

	mxt_debug(DEBUG_TRACE, "Writing %d bytes to %d...", length, addr);
	if (length > 256)
		return -EINVAL;
	mxt = i2c_get_clientdata(client);
	if (mxt != NULL)
		mxt->last_read_addr = -1;
	for (i = 0; i < length; i++)
		i2c_block_transfer.data[i] = *value++;
	i2c_block_transfer.le_addr = cpu_to_le16(addr);
	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
	if (i == (length + 2))
		return length;
	else
		return -EIO;
}

/* Calculates the CRC value for mXT infoblock. */
int calculate_infoblock_crc(u32 *crc_result, u8 *data, int crc_area_size)
{
	u32 crc = 0;
	int i;

	for (i = 0; i < (crc_area_size - 1); i = i + 2)
		crc = CRC_24(crc, *(data + i), *(data + i + 1));
	/* If uneven size, pad with zero */
	if (crc_area_size & 0x0001)
		crc = CRC_24(crc, *(data + i), 0);
	/* Return only 24 bits of CRC. */
	*crc_result = (crc & 0x00FFFFFF);

	return 0;
}

void mxt_hw_reset(void){
    gpio_set_value(TEGRA_GPIO_PH6, 0);
    msleep(1);
    gpio_set_value(TEGRA_GPIO_PH6, 1);
    msleep(280);
}

static int mxt_init_Boot(struct mxt_data *mxt)
{
    int ret; 
    u8 firmware_id[MXT_ID_BLOCK_SIZE];
	
    dev_info(&mxt->client->dev, "Touch: start doing FW update\n");
    wake_lock(&mxt->wakelock);
    disable_irq(mxt->irq);
    cancel_delayed_work(&mxt->dwork);
    ret = mxt_update_firmware(mxt, false);
    if(ret > 0)  goto finish_init_boot;
    msleep(4000);
    mxt_hw_reset();
    mxt_identify(mxt->client, mxt, firmware_id);
    mxt_read_object_table(mxt->client, mxt, firmware_id);
    init_touch_config(mxt, touch_vendor_id);
    mxt_write_byte(mxt->client,MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt)+ MXT_ADR_T6_BACKUPNV,
					MXT_CMD_T6_BACKUP);
finish_init_boot:
    enable_irq(mxt->irq);
    wake_unlock(&mxt->wakelock);
    return ret;
}

void force_release_pos(struct mxt_data *mxt)
{
	printk("Touch: force release position\n");
	int i;
	for (i=0; i < 10; i++){
		if (fingerInfo[i].pressure ==0) continue;

		input_report_abs(mxt->input, ABS_MT_TRACKING_ID, i);
		input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR, 0);
		fingerInfo[i].pressure=0;
		input_mt_sync(mxt->input);
	}

	input_sync(mxt->input);
}

void process_T9_message(u8 *message, struct mxt_data *mxt, int last_touch)
{

	struct input_dev *input;
	u8 status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8 touch_size = 255;
	u8 touch_number;
	u8 amplitude;
	u8 report_id;

	static int stored_size[10];
	static int stored_x[10];
	static int stored_y[10];
	int i;
	int active_touches = 0;
      static int origin_x;
      static int origin_y;
	int del_x, del_y;
	
	/*
	 * If the 'last_touch' flag is set, we have received
		all the touch messages
	 * there are available in this cycle, so send the
		events for touches that are
	 * active.
	 */
	if (last_touch) {
		input_report_key(mxt->input, BTN_TOUCH, 1);
		for (i = 0; i < 10; i++) {
			if (fingerInfo[i].pressure) {
				active_touches++;
				input_report_abs(mxt->input, ABS_MT_TRACKING_ID,
						 i);
				input_report_abs(mxt->input, ABS_MT_TOUCH_MAJOR,
						 fingerInfo[i].pressure);
				input_report_abs(mxt->input, ABS_MT_PRESSURE,
					       fingerInfo[i].pressure);
				if(touch_flag){
					input_report_abs(mxt->input, ABS_MT_POSITION_X,
                                                (mxt->max_x_val-fingerInfo[i].x));
				}else{
					input_report_abs(mxt->input, ABS_MT_POSITION_X,
                                                fingerInfo[i].x);
				}

				input_report_abs(mxt->input, ABS_MT_POSITION_Y,
						 fingerInfo[i].y);
				input_mt_sync(mxt->input);
			if(unlikely(d_flag))
				printk("Touch: In last_touch decision, stored_size[%d]=%d, stored_x[%d]=%d, stored_y[%d]=%d\n",i,stored_size[i],i,stored_x[i],i,stored_y[i]);
			}
		}
		if (active_touches)
			input_sync(mxt->input);
		else {
			input_mt_sync(mxt->input);
			input_sync(mxt->input);
			if(unlikely(d_flag))
				printk("Touch: no touch and send uevent!\n");
		}

	} else {

		input = mxt->input;
		status = message[MXT_MSG_T9_STATUS];
		report_id = message[0];
                touch_number = message[MXT_MSG_REPORTID] -
                            mxt->rid_map[report_id].first_rid;
		if (status & MXT_MSGB_T9_SUPPRESS) {
			/* Touch has been suppressed by grip/face */
			/* detection                              */
                        stored_size[touch_number] = 0;
			fingerInfo[touch_number].pressure=0;
			mxt_debug(DEBUG_TRACE, "SUPRESS");
		} else {
			xpos = message[MXT_MSG_T9_XPOSMSB] * 16 +
			    ((message[MXT_MSG_T9_XYPOSLSB] >> 4) & 0xF);
			ypos = message[MXT_MSG_T9_YPOSMSB] * 16 +
			    ((message[MXT_MSG_T9_XYPOSLSB] >> 0) & 0xF);
			if (mxt->max_x_val < 1024)
				xpos >>= 2;
			if (mxt->max_y_val < 1024)
				ypos >>= 2;

			stored_x[touch_number] = xpos;
			stored_y[touch_number] = ypos;
			fingerInfo[touch_number].x=xpos;
			fingerInfo[touch_number].y=ypos;
                   if(delta_flag && touch_number == 0){
		          origin_x = xpos;
                       origin_y = ypos;
			    delta_flag = false;
		      }
			if (status & MXT_MSGB_T9_DETECT) {
				/*
				 * TODO: more precise touch size calculation?
				 * mXT224 reports the number of touched nodes,
				 * so the exact value for touch ellipse major
				 * axis length would be 2*sqrt(touch_size/pi)
				 * (assuming round touch shape).
				 */
				touch_size = message[MXT_MSG_T9_TCHAREA];
				if (touch_size <= 15)
					touch_size = touch_size << 4;
				else
					touch_size = 255;
				
				if (!touch_size)
					touch_size = 1;

				stored_size[touch_number] = touch_size;
				fingerInfo[touch_number].pressure=touch_size;

				if (status & MXT_MSGB_T9_AMP)
					/* Amplitude of touch has changed */
					amplitude =
					    message[MXT_MSG_T9_TCHAMPLITUDE];
			}

			if (status & MXT_MSGB_T9_RELEASE) {
				/* The previously reported touch has
					been removed. */
				stored_size[touch_number] = 0;
				fingerInfo[touch_number].pressure=0;
				if(resume_flag){
				    if(touch_number == 0){
						del_x = abs(stored_x[0] -origin_x);
						del_y = abs(stored_y[0] -origin_y);

						if ((del_x > 25 || del_y > 40) && --resume_flag == 0){
							mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+6, 0);
							mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+7, 0x01);
							mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+8, 0);
							mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt)+9, 0);
							mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 0x31);
							mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt) + 7, 0x3C);
							printk( "Touch: disable T8 config, delta x= %d, delta y = %d\n",del_x,del_y);
						}
						else
							delta_flag = true;
					}    	
				}

				if(unlikely(d_flag))
					printk("Touch: finger release in touch id=%d\n", touch_number);
			}
		}

		if (status & MXT_MSGB_T9_SUPPRESS) {
			mxt_debug(DEBUG_TRACE, "SUPRESS");
		} else {
			if (status & MXT_MSGB_T9_DETECT) {
				mxt_debug(DEBUG_TRACE, "DETECT:%s%s%s%s",
					  ((status & MXT_MSGB_T9_PRESS) ?
					   " PRESS" : ""),
					  ((status & MXT_MSGB_T9_MOVE) ? " MOVE"
					   : ""),
					  ((status & MXT_MSGB_T9_AMP) ? " AMP" :
					   ""),
					  ((status & MXT_MSGB_T9_VECTOR) ?
					   " VECT" : ""));

			} else if (status & MXT_MSGB_T9_RELEASE) {
				mxt_debug(DEBUG_TRACE, "RELEASE");
			}
		}
		mxt_debug(DEBUG_TRACE, "X=%d, Y=%d, TOUCHSIZE=%d",
			  xpos, ypos, touch_size);
	}
	return;
}

int process_message(u8 *message, u8 object, struct mxt_data *mxt)
{
	struct i2c_client *client;
	u8 status;
	u16 xpos = 0xFFFF;
	u16 ypos = 0xFFFF;
	u8 event;
	u8 length;
	u8 report_id;
	u8 buf[3];
	u32 cfg_crc;
	static u8 noise_count = 0;

	client = mxt->client;
	length = mxt->message_size;
	report_id = message[0];
	  
	if ((mxt->nontouch_msg_only == 0) || (!IS_TOUCH_OBJECT(object))) {
		mutex_lock(&mxt->msg_mutex);
		/* Copy the message to buffer */
		if (mxt->msg_buffer_startp < MXT_MESSAGE_BUFFER_SIZE)
			mxt->msg_buffer_startp++;
		else
			mxt->msg_buffer_startp = 0;

		if (mxt->msg_buffer_startp == mxt->msg_buffer_endp) {
			mxt_debug(DEBUG_TRACE,
				  "Message buf full, discarding last entry.\n");
			if (mxt->msg_buffer_endp < MXT_MESSAGE_BUFFER_SIZE)
				mxt->msg_buffer_endp++;
			else
				mxt->msg_buffer_endp = 0;
		}
		memcpy((mxt->messages + mxt->msg_buffer_startp * length),
		       message, length);
		mutex_unlock(&mxt->msg_mutex);
	}

	switch (object) {
	case MXT_GEN_COMMANDPROCESSOR_T6:
		status = message[1];
		if (status & MXT_MSGB_T6_COMSERR)
			dev_err(&client->dev, "maXTouch checksum error\n");
		if (status & MXT_MSGB_T6_CFGERR) {
			/*
			 * Configuration error. A proper configuration
			 * needs to be written to chip and backed up. Refer
			 * to protocol document for further info.
			 */
			dev_err(&client->dev, "maXTouch configuration error\n");
		}
		if (status & MXT_MSGB_T6_CAL) {
			/* Calibration in action, no need to react */
			dev_info(&client->dev,
				 "maXTouch calibration in progress\n");
		}
		if (status & MXT_MSGB_T6_SIGERR) {
			/*
			 * Signal acquisition error, something is seriously
			 * wrong, not much we can in the driver to correct
			 * this
			 */
			dev_err(&client->dev, "maXTouch acquisition error\n");
		}
		if (status & MXT_MSGB_T6_OFL) {
			/*
			 * Cycle overflow, the acquisition is too short.
			 * Can happen temporarily when there's a complex
			 * touch shape on the screen requiring lots of
			 * processing.
			 */
			dev_err(&client->dev, "maXTouch cycle overflow\n");
		}
		if (status & MXT_MSGB_T6_RESET) {
			/* Chip has reseted, no need to react. */
			dev_info(&client->dev, "maXTouch chip reset\n");
		}
		if (status == 0) {
			/* Chip status back to normal. */
			dev_info(&client->dev, "maXTouch status normal\n");
		}

             if (cfg_flag){
		    buf[0]=message[2];
		    buf[1]=message[3];
		    buf[2]=message[4];
		    cfg_crc= buf[2] << 16 | buf[1] <<8 | buf[0];
		    mxt->configuration_crc = cfg_crc;
		    printk("Touch: configuration checksum is %lx\n",cfg_crc);

		    if (cfg_crc != touch_panel_config_checksum[(mxt->device_info.major == 0x02) ? 1 : 0][touch_vendor_id]){
		        printk("Touch: start BACKUP\n");
	              mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_COMMANDPROCESSOR_T6, mxt) + MXT_ADR_T6_BACKUPNV,MXT_CMD_T6_BACKUP);
		    }else
	              printk("Touch: config is BACKUP already\n");

		    cfg_flag =false;
	      }
		
		break;

	case MXT_TOUCH_MULTITOUCHSCREEN_T9:
		mxt->hasTouch = true;
		process_T9_message(message, mxt, 0);
		break;

	case MXT_SPT_GPIOPWM_T19:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving GPIO message\n");
		break;

	case MXT_PROCI_GRIPFACESUPPRESSION_T20:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				 "Receiving face suppression msg\n");
		break;

	case MXT_PROCG_NOISESUPPRESSION_T22:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				 "Receiving noise suppression msg\n");
		status = message[MXT_MSG_T22_STATUS];
		if (status & MXT_MSGB_T22_FHCHG) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					 "maXTouch: Freq changed\n");
		}
		if (status & MXT_MSGB_T22_GCAFERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					 "maXTouch: High noise " "level\n");
		}
		if (status & MXT_MSGB_T22_FHERR) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					 "maXTouch: Freq changed - "
					 "Noise level too high\n");
		}
		break;

	case MXT_PROCI_ONETOUCHGESTUREPROCESSOR_T24:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				 "Receiving one-touch gesture msg\n");

		event = message[MXT_MSG_T24_STATUS] & 0x0F;
		xpos = message[MXT_MSG_T24_XPOSMSB] * 16 +
		    ((message[MXT_MSG_T24_XYPOSLSB] >> 4) & 0x0F);
		ypos = message[MXT_MSG_T24_YPOSMSB] * 16 +
		    ((message[MXT_MSG_T24_XYPOSLSB] >> 0) & 0x0F);
		xpos >>= 2;
		ypos >>= 2;

		switch (event) {
		case MT_GESTURE_RESERVED:
			break;
		case MT_GESTURE_PRESS:
			break;
		case MT_GESTURE_RELEASE:
			break;
		case MT_GESTURE_TAP:
			break;
		case MT_GESTURE_DOUBLE_TAP:
			break;
		case MT_GESTURE_FLICK:
			break;
		case MT_GESTURE_DRAG:
			break;
		case MT_GESTURE_SHORT_PRESS:
			break;
		case MT_GESTURE_LONG_PRESS:
			break;
		case MT_GESTURE_REPEAT_PRESS:
			break;
		case MT_GESTURE_TAP_AND_PRESS:
			break;
		case MT_GESTURE_THROW:
			break;
		default:
			break;
		}
		break;

	case MXT_SPT_SELFTEST_T25:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving Self-Test msg\n");
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,33)
		    kfifo_put(p_char_dev->pCharKFiFo, message, 7);
                 TOUCH_DBG(DBG_CDEV, " Get the new T25 data with bytes %d \n", kfifo_len(p_char_dev->pCharKFiFo));
#else
		    kfifo_in(&p_char_dev->CharKFiFo, message, 7);
                 TOUCH_DBG(DBG_CDEV, " Get the new T25 data with bytes %d \n", kfifo_len(&p_char_dev->CharKFiFo));
#endif
		    wake_up_interruptible( &p_char_dev->fifo_inq );
		if (message[MXT_MSG_T25_STATUS] == MXT_MSGR_T25_OK) {
			if (debug >= DEBUG_TRACE)
				dev_info(&client->dev,
					 "maXTouch: Self-Test OK\n");

		} else {
			dev_err(&client->dev,
				"maXTouch: Self-Test Failed [%02x]:"
				"{%02x,%02x,%02x,%02x,%02x}\n",
				message[MXT_MSG_T25_STATUS],
				message[MXT_MSG_T25_STATUS + 0],
				message[MXT_MSG_T25_STATUS + 1],
				message[MXT_MSG_T25_STATUS + 2],
				message[MXT_MSG_T25_STATUS + 3],
				message[MXT_MSG_T25_STATUS + 4]
			    );
		}
		break;

	case MXT_PROCI_TWOTOUCHGESTUREPROCESSOR_T27:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev,
				 "Receiving 2-touch gesture message\n");
		break;

	case MXT_SPT_CTECONFIG_T28:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "Receiving CTE message...\n");
		status = message[MXT_MSG_T28_STATUS];
		if (status & MXT_MSGB_T28_CHKERR)
			dev_err(&client->dev,
				"maXTouch: Power-Up CRC failure\n");

		break;
	case MXT_PROCG_NOISESUPPRESSION_T48: // the nosie message 
		dev_info(&client->dev, "Noise Message with bytes: 0x%02X:0x%02X:0x%02X:0x%02X:0x%02X\n",
			      message[1], message[2], message[3], message[4], message[5]);
		break;
	default:
		if (debug >= DEBUG_TRACE)
			dev_info(&client->dev, "maXTouch: Type=%d Unknown message!\n", object);

		break;
	}

	return 0;
}

static void setInterruptable(struct i2c_client *client, bool interrupt){
    struct mxt_data *mxt = i2c_get_clientdata(client);
    down(&mxt->sem);
    mxt->interruptable = interrupt;
    up(&mxt->sem);
}
/*
 * Processes messages when the interrupt line (CHG) is asserted. Keeps
 * reading messages until a message with report ID 0xFF is received,
 * which indicates that there is no more new messages.
 *
 */

static void mxt_worker(struct work_struct *work)
{
	struct mxt_data *mxt;
	struct i2c_client *client;

	u8 *message;
	u16 message_length;
	u16 message_addr;
	u8 report_id;
	u8 object;
	int error;
	int i;
	char *message_string;
	char *message_start;

	message = NULL;
	mxt = container_of(work, struct mxt_data, dwork.work);
	disable_irq(mxt->irq);
	client = mxt->client;
	message_addr = mxt->msg_proc_addr;
	message_length = mxt->message_size;

	if (message_length < 256) {
		message = kmalloc(message_length, GFP_KERNEL);
		if (message == NULL) {
			dev_err(&client->dev, "Error allocating memory\n");
			setInterruptable(client, false);
			return;
		}
	} else {
		dev_err(&client->dev,
			"Message length larger than 256 bytes not supported\n");
		setInterruptable(client, false);
		return;
	}

	mxt_debug(DEBUG_TRACE, "maXTouch worker active: \n");
	do {
		/* Read next message, reread on failure. */
		/* -1 TO WORK AROUND A BUG ON 0.9 FW MESSAGING, needs */
		/* to be changed back if checksum is read */
		mxt->message_counter++;
		for (i = 1; i < I2C_RETRY_COUNT; i++) {
			error = mxt_read_block(client,
					       message_addr,
					       message_length - 1, message);
			if (error >= 0)
				break;
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"Failure reading maxTouch device\n");
		}
		if (error < 0) {
			kfree(message);
			setInterruptable(client, false);
			return;
		}

		if (mxt->address_pointer != message_addr)
			mxt->valid_ap = 0;
		report_id = message[0];

		if (debug >= DEBUG_RAW) {
			mxt_debug(DEBUG_RAW, "%s message [msg count: %08x]:",
				  REPORT_ID_TO_OBJECT_NAME(report_id, mxt),
				  mxt->message_counter);
			/* 5 characters per one byte */
			message_string = kmalloc(message_length * 5,
						 GFP_KERNEL);
			if (message_string == NULL) {
				dev_err(&client->dev,
					"Error allocating memory\n");
				kfree(message);
				setInterruptable(client, false);
				return;
			}
			message_start = message_string;
			for (i = 0; i < message_length; i++) {
				message_string +=
				    sprintf(message_string,
					    "0x%02X ", message[i]);
			}
			mxt_debug(DEBUG_RAW, "%s", message_start);
			kfree(message_start);
		}

		if ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)) {
			memcpy(mxt->last_message, message, message_length);
			mxt->new_msgs = 1;
			smp_wmb();
			/* Get type of object and process the message */
			object = mxt->rid_map[report_id].object;
			process_message(message, object, mxt);
		}
		mxt_debug(DEBUG_TRACE, "chgline: %d\n", mxt->read_chg());
	} while (comms ? (mxt->read_chg() == 0) :
		 ((report_id != MXT_END_OF_MESSAGES) && (report_id != 0)));

	/* All messages processed, send the events) */
	if(mxt->hasTouch){
	    process_T9_message(NULL, mxt, 1);
	    mxt->hasTouch = false;
	}
	

	kfree(message);
	enable_irq(mxt->irq);
	/* Make sure we don't miss any interrupts and read changeline. */
	if (mxt->read_chg() == 0)
		schedule_delayed_work(&mxt->dwork, 0);
}

/*
 * The maXTouch device will signal the host about a new message by asserting
 * the CHG line. This ISR schedules a worker routine to read the message when
 * that happens.
 */

static irqreturn_t mxt_irq_handler(int irq, void *_mxt)
{
	struct mxt_data *mxt = _mxt;

	if(unlikely(d_flag))
		printk("Touch: IRQ detect\n");
	mxt->irq_counter++;
	if (mxt->valid_interrupt()) {
		/* Send the signal only if falling edge generated the irq. */
		//cancel_delayed_work(&mxt->dwork);
		schedule_delayed_work(&mxt->dwork, 0);
		mxt->valid_irq_counter++;
	} else {
		mxt->invalid_irq_counter++;
		return IRQ_NONE;
	}

	return IRQ_HANDLED;
}

static int recovery_from_bootMode(struct i2c_client *client){
    u8 buf[MXT_ID_BLOCK_SIZE];
    int ret;
    int identified;
    int retry = 40;
    int times;
    unsigned char data[] = {0x01, 0x01};
    struct i2c_msg wmsg;
    
    wmsg.addr = 0x27;
    wmsg.flags = I2C_M_WR;
    wmsg.len = 2;
    wmsg.buf = data;
    dev_err(&client->dev, "---------Touch: Try to leave the bootloader mode!\n");
	/*Write two nosense bytes to I2C address "0x35" in order to force touch to leave the bootloader mode.*/
    i2c_transfer(client->adapter, &wmsg, 1);
    mdelay(10);
	
    /* Read Device info to check if chip is valid */
    for(times = 0; times < retry; times++ ){
        ret = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE, (u8 *) buf); 
	  if(ret >= 0)
	      break;

	  dev_err(&client->dev, "Retry addressing I2C address 0x%02X with %d times\n", client->addr,times+1); 	 
	  msleep(25);
    }	
	
    if(ret >= 0){
        dev_err(&client->dev, "---------Touch: Successfully leave the bootloader mode!\n");
		ret = 0;
    }    
    return ret;
}
static bool isInBootLoaderMode(struct i2c_client *client){
    u8 buf[2];
	int ret;
	int identified;
	int retry = 2;
	int times;
	struct i2c_msg rmsg;

	rmsg.addr=0x27;
	rmsg.flags = I2C_M_RD;
	rmsg.len = 2;
	rmsg.buf = buf;
	
    /* Read 2 byte from boot loader I2C address to make sure touch chip is in bootloader mode */   
	for(times = 0; times < retry; times++ ){
	     ret = i2c_transfer(client->adapter, &rmsg, 1); 
	     if(ret >= 0)
		 	break;
		 	  	 
	     mdelay(25);
	}
	dev_err(&client->dev, "The touch is %s in bootloader mode.\n", (ret < 0 ? "not" : "indeed"));
	return ret >= 0;
}
/******************************************************************************/
/* Initialization of driver                                                   */
/******************************************************************************/

int __devinit mxt_identify(struct i2c_client *client,
				  struct mxt_data *mxt, u8 * id_block_data)
{
	u8 buf[7];
	int error;
	int identified;
      int retry = 2;
      int times;
	      
	identified = 0;
	/* Read Device info to check if chip is valid */
       for(times = 0; times < retry; times++ ){
	     error = mxt_read_block(client, MXT_ADDR_INFO_BLOCK, MXT_ID_BLOCK_SIZE,
			       (u8 *) buf); 
	     if(error >= 0)
		 	break;
		  	 
	     mdelay(25);
	 }

	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Failure accessing maXTouch device\n");
		return -EIO;
	}

	memcpy(id_block_data, buf, MXT_ID_BLOCK_SIZE);

	mxt->device_info.family_id = buf[0];
	mxt->device_info.variant_id = buf[1];
	mxt->device_info.major = ((buf[2] >> 4) & 0x0F);
	mxt->device_info.minor = (buf[2] & 0x0F);
	mxt->device_info.build = buf[3];
	mxt->device_info.x_size = buf[4];
	mxt->device_info.y_size = buf[5];
	mxt->device_info.num_objs = buf[6];
	mxt->device_info.num_nodes = mxt->device_info.x_size *
	    mxt->device_info.y_size;

	/*
	 * Check Family & Variant Info; warn if not recognized but
	 * still continue.
	 */

	/* MXT224 */
	if (mxt->device_info.family_id == MXT224_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT224");

		if (mxt->device_info.variant_id == MXT224_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else if (mxt->device_info.variant_id ==
			   MXT224_UNCAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Uncalibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n", mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}

		/* MXT1386 */
	} else if (mxt->device_info.family_id == MXT1386_FAMILYID) {
		strcpy(mxt->device_info.family_name, "mXT1386");

		if (mxt->device_info.variant_id == MXT1386_CAL_VARIANTID) {
			strcpy(mxt->device_info.variant_name, "Calibrated");
		} else {
			dev_err(&client->dev,
				"Warning: maXTouch Variant ID [%d] not "
				"supported\n", mxt->device_info.variant_id);
			strcpy(mxt->device_info.variant_name, "UNKNOWN");
			/* identified = -ENXIO; */
		}
		/* Unknown family ID! */
	} else {
		dev_err(&client->dev,
			"Warning: maXTouch Family ID [%d] not supported\n",
			mxt->device_info.family_id);
		strcpy(mxt->device_info.family_name, "UNKNOWN");
		strcpy(mxt->device_info.variant_name, "UNKNOWN");
		/* identified = -ENXIO; */
	}

	dev_info(&client->dev,
		 "Atmel maXTouch (Family %s (%X), Variant %s (%X)) Firmware "
		 "version [%d.%d] Build %d\n",
		 mxt->device_info.family_name,
		 mxt->device_info.family_id,
		 mxt->device_info.variant_name,
		 mxt->device_info.variant_id,
		 mxt->device_info.major,
		 mxt->device_info.minor, mxt->device_info.build);
	dev_info(&client->dev,
		 "Atmel maXTouch Configuration "
		 "[X: %d] x [Y: %d]\n",
		 mxt->device_info.x_size, mxt->device_info.y_size);
	ver_major=mxt->device_info.major;
	ver_minor=mxt->device_info.minor;
	return identified;
}

/*
 * Reads the object table from maXTouch chip to get object data like
 * address, size, report id. For Info Block CRC calculation, already read
 * id data is passed to this function too (Info Block consists of the ID
 * block and object table).
 *
 */
static int __devinit mxt_read_object_table(struct i2c_client *client,
					   struct mxt_data *mxt,
					   u8 *raw_id_data)
{
	u16 report_id_count;
	u8 buf[MXT_OBJECT_TABLE_ELEMENT_SIZE];
	u8 *raw_ib_data;
	u8 object_type;
	u16 object_address;
	u16 object_size;
	u8 object_instances;
	u8 object_report_ids;
	u16 object_info_address;
	u32 crc;
	u32 calculated_crc;
	int i;
	int error;

	u8 object_instance;
	u8 object_report_id;
	u8 report_id;
	int first_report_id;
	int ib_pointer;
	struct mxt_object *object_table;

	mxt_debug(DEBUG_TRACE, "maXTouch driver reading configuration\n");

	object_table = kzalloc(sizeof(struct mxt_object) *
			       mxt->device_info.num_objs, GFP_KERNEL);
	if (object_table == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_object_table_alloc;
	}

	raw_ib_data = kmalloc(MXT_OBJECT_TABLE_ELEMENT_SIZE *
			      mxt->device_info.num_objs + MXT_ID_BLOCK_SIZE,
			      GFP_KERNEL);
	if (raw_ib_data == NULL) {
		printk(KERN_WARNING "maXTouch: Memory allocation failed!\n");
		error = -ENOMEM;
		goto err_ib_alloc;
	}

	/* Copy the ID data for CRC calculation. */
	memcpy(raw_ib_data, raw_id_data, MXT_ID_BLOCK_SIZE);
	ib_pointer = MXT_ID_BLOCK_SIZE;

	mxt->object_table = object_table;

	mxt_debug(DEBUG_TRACE, "maXTouch driver Memory allocated\n");

	object_info_address = MXT_ADDR_OBJECT_TABLE;

	report_id_count = 0;
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		mxt_debug(DEBUG_TRACE, "Reading maXTouch at [0x%04x]: ",
			  object_info_address);

		error = mxt_read_block(client, object_info_address,
				       MXT_OBJECT_TABLE_ELEMENT_SIZE, buf);

		if (error < 0) {
			mxt->read_fail_counter++;
			dev_err(&client->dev,
				"maXTouch Object %d could not be read\n", i);
			error = -EIO;
			goto err_object_read;
		}

		memcpy(raw_ib_data + ib_pointer, buf,
		       MXT_OBJECT_TABLE_ELEMENT_SIZE);
		ib_pointer += MXT_OBJECT_TABLE_ELEMENT_SIZE;

		object_type = buf[0];
		object_address = (buf[2] << 8) + buf[1];
		object_size = buf[3] + 1;
		object_instances = buf[4] + 1;
		object_report_ids = buf[5];
		mxt_debug(DEBUG_TRACE, "Type=%03d, Address=0x%04x, "
			  "Size=0x%02x, %d instances, %d report id's\n",
			  object_type,
			  object_address,
			  object_size, object_instances, object_report_ids);

		/* TODO: check whether object is known and supported? */

		/* Save frequently needed info. */
		if (object_type == MXT_GEN_MESSAGEPROCESSOR_T5) {
			mxt->msg_proc_addr = object_address;
			mxt->message_size = object_size;
			printk(KERN_ALERT "message length: %d", object_size);
		}

		object_table[i].type = object_type;
		object_table[i].chip_addr = object_address;
		object_table[i].size = object_size;
		object_table[i].instances = object_instances;
		object_table[i].num_report_ids = object_report_ids;
		report_id_count += object_instances * object_report_ids;

		object_info_address += MXT_OBJECT_TABLE_ELEMENT_SIZE;
	}

	mxt->rid_map =
	    kzalloc(sizeof(struct report_id_map) * (report_id_count + 1),
		    /* allocate for report_id 0, even if not used */
		    GFP_KERNEL);
	if (mxt->rid_map == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_rid_map_alloc;
	}

	mxt->messages = kzalloc(mxt->message_size * MXT_MESSAGE_BUFFER_SIZE,
				GFP_KERNEL);
	if (mxt->messages == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_msg_alloc;
	}

	mxt->last_message = kzalloc(mxt->message_size, GFP_KERNEL);
	if (mxt->last_message == NULL) {
		printk(KERN_WARNING "maXTouch: Can't allocate memory!\n");
		error = -ENOMEM;
		goto err_msg_alloc;
	}

	mxt->report_id_count = report_id_count;
	if (report_id_count > 254) {	/* 0 & 255 are reserved */
		dev_err(&client->dev,
			"Too many maXTouch report id's [%d]\n",
			report_id_count);
		error = -ENXIO;
		goto err_max_rid;
	}

	/* Create a mapping from report id to object type */
	report_id = 1;		/* Start from 1, 0 is reserved. */

	/* Create table associating report id's with objects & instances */
	for (i = 0; i < mxt->device_info.num_objs; i++) {
		for (object_instance = 0;
		     object_instance < object_table[i].instances;
		     object_instance++) {
			first_report_id = report_id;
			for (object_report_id = 0;
			     object_report_id < object_table[i].num_report_ids;
			     object_report_id++) {
				mxt->rid_map[report_id].object =
				    object_table[i].type;
				mxt->rid_map[report_id].instance =
				    object_instance;
				mxt->rid_map[report_id].first_rid =
				    first_report_id;
				report_id++;
			}
		}
	}

	/* Read 3 byte CRC */
	error = mxt_read_block(client, object_info_address, 3, buf);
	if (error < 0) {
		mxt->read_fail_counter++;
		dev_err(&client->dev, "Error reading CRC\n");
	}

	crc = (buf[2] << 16) | (buf[1] << 8) | buf[0];

	if (calculate_infoblock_crc(&calculated_crc, raw_ib_data, ib_pointer)) {
		printk(KERN_WARNING "Error while calculating CRC!\n");
		calculated_crc = 0;
	}
	kfree(raw_ib_data);

	mxt_debug(DEBUG_TRACE, "\nReported info block CRC = 0x%6X\n", crc);
	mxt_debug(DEBUG_TRACE, "Calculated info block CRC = 0x%6X\n\n",
		  calculated_crc);

	if (crc == calculated_crc) {
		mxt->info_block_crc = crc;
		fw_checksum=mxt->info_block_crc;
	} else {
		mxt->info_block_crc = 0;
		printk(KERN_ALERT "maXTouch: Info block CRC invalid!\n");
	}

	if (debug >= DEBUG_VERBOSE) {

		dev_info(&client->dev, "maXTouch: %d Objects\n",
			 mxt->device_info.num_objs);

		for (i = 0; i < mxt->device_info.num_objs; i++) {
			dev_info(&client->dev, "Type:\t\t\t[%d]: %s\n",
				 object_table[i].type,
				 object_type_name[object_table[i].type]);
			dev_info(&client->dev, "\tAddress:\t0x%04X\n",
				 object_table[i].chip_addr);
			dev_info(&client->dev, "\tSize:\t\t%d Bytes\n",
				 object_table[i].size);
			dev_info(&client->dev, "\tInstances:\t%d\n",
				 object_table[i].instances);
			dev_info(&client->dev, "\tReport Id's:\t%d\n",
				 object_table[i].num_report_ids);
		}
	}

	return 0;

 err_max_rid:
	kfree(mxt->last_message);
 err_msg_alloc:
	kfree(mxt->rid_map);
 err_rid_map_alloc:
 err_object_read:
	kfree(raw_ib_data);
 err_ib_alloc:
	kfree(object_table);
 err_object_table_alloc:
	return error;
}

#if defined(CONFIG_PM)
static void mxt_start(struct mxt_data *mxt)
{
      char buf[2];
      int wait_count = 10;
      int error;

      force_release_pos(mxt);
		
	//dummy read to wake up from deep sleep mode
	mxt_read_block(mxt->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, mxt), 1, buf);
	mdelay(25);
	error = mxt_read_block(mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt), 1, buf);
      
      if(error < 0){ // start hardware reset
          gpio_direction_output(TEGRA_GPIO_PH6, 0);
	    mdelay(1);
	    gpio_direction_output(TEGRA_GPIO_PH6, 1);
          do{
	        mdelay(30);
	        dev_info(&mxt->client->dev, "Wait CHG count=%d\n", wait_count);
          }while(--wait_count && mxt->read_chg() != 0);
	    error = mxt_read_block(mxt->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, mxt), 1, buf);
	}
	
	if(error < 0 && isInBootLoaderMode(mxt->client)) // start boot loader recovery mode
            recovery_from_bootMode(mxt->client);
	
      mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt), 0x0F);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt)+1, 0xFF);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, mxt), 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt) + 6, 0x05);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt) + 7, 0x50);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt) + 8, 0x06);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRECONFIG_T8, mxt) + 9, 0xC0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt) + 7, 0x50);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 0x8F);	
	resume_flag = 1; 
	delta_flag = true;
	schedule_delayed_work(&mxt->dwork, 0);
	if(!mxt->interruptable){
	    enable_irq(mxt->irq);
	    setInterruptable(mxt->client, true);
	}
    enable_irq(mxt->irq);
}

static void mxt_stop(struct mxt_data *mxt)
{
      disable_irq(mxt->irq);
      mxt_write_byte(mxt->client,
		MXT_BASE_ADDR(MXT_TOUCH_MULTITOUCHSCREEN_T9, mxt), 0x0);
	// Go into Deep sleep Mode
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt), 0);
	mxt_write_byte(mxt->client, MXT_BASE_ADDR(MXT_GEN_POWERCONFIG_T7, mxt) + 1, 0);
}

static int mxt_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct mxt_data *mxt = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		enable_irq_wake(mxt->irq);
	else
		mxt_stop(mxt);

	return 0;
}

static int mxt_resume(struct i2c_client *client)
{
	struct mxt_data *mxt = i2c_get_clientdata(client);

	if (device_may_wakeup(&client->dev))
		disable_irq_wake(mxt->irq);
	else
		mxt_start(mxt);

	return 0;
}

static void update_noise_state(){
    if(globe_mxt == NULL)
        return;

    struct i2c_client *client = globe_mxt->client;
    down(&globe_mxt->sem);	
    if((pre_usb_cable_status == USB_AC_Adapter) && now_usb_cable_status < USB_AC_Adapter){
	  mxt_write_byte(client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, globe_mxt) + 2 , 0x42);
	  dev_info(&client->dev, "touch_callback  set charger off\n");
    } else if(pre_usb_cable_status < USB_AC_Adapter && (now_usb_cable_status == USB_AC_Adapter)){
        mxt_write_byte(client, MXT_BASE_ADDR(MXT_PROCG_NOISESUPPRESSION_T48, globe_mxt) + 2 , 0x62);
        dev_info(&client->dev, "touch_callback  set charger on\n");
    }
    up(&globe_mxt->sem);
}

#if defined(CONFIG_TOUCHSCREEN_ELAN_TF_3K)
extern void touch_callback_elan(unsigned cable_status);
#endif

void touch_callback(unsigned cable_status){	 		
    printk("Touch charger state: %d\n",  cable_status);
    pre_usb_cable_status = now_usb_cable_status;
    now_usb_cable_status = cable_status;
    update_noise_state(); // update the noise state
#if defined(CONFIG_TOUCHSCREEN_ELAN_TF_3K)
    touch_callback_elan(cable_status);
#endif
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *es)
{
	struct mxt_data *mxt;
	mxt = container_of(es, struct mxt_data, early_suspend);

	if (mxt_suspend(mxt->client, PMSG_SUSPEND) != 0)
		dev_err(&mxt->client->dev, "%s: failed\n", __func__);
	printk(KERN_WARNING "MXT Early Suspended\n");
}

static void mxt_early_resume(struct early_suspend *es)
{
	struct mxt_data *mxt;
	mxt = container_of(es, struct mxt_data, early_suspend);

	if (mxt_resume(mxt->client) != 0)
		dev_err(&mxt->client->dev, "%s: failed\n", __func__);
	printk(KERN_WARNING "MXT Early Resumed\n");
}
#endif

#else
#define mxt_suspend NULL
#define mxt_resume NULL
#endif

static void  mxt_poll_data(struct work_struct * work)
{
	bool status;
	u8 count[7];
	status = mxt_read_block(mxt_client, 0, 7, (u8 *)count);
	if(status < 0)
		printk("Read touch sensor data fail\n");

	if(poll_mode ==0)
		msleep(5);

	queue_delayed_work(sensor_work_queue, &mxt_poll_data_work, poll_mode);
}

int mxt_stress_open(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	  if( !atomic_dec_and_test(&touch_char_available) )
	{
		atomic_inc(&touch_char_available);
		return -EBUSY; /* already open */
	}
	return 0;          /* success */
}


int mxt_stress_release(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	atomic_inc(&touch_char_available); /* release the device */
	return 0;          /* success */
}

int mxt_stress_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct mxt_data *mxt;
	u8 firmware_id[MXT_ID_BLOCK_SIZE];
	int err = 1;
	mxt = globe_mxt;

	printk("%s\n", __func__, cmd);
	if (_IOC_TYPE(cmd) != MXT_IOC_MAGIC)
	return -ENOTTY;
	if (_IOC_NR(cmd) > MXT_IOC_MAXNR)
	return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
	err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
	err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;
	switch (cmd) {
		case MXT_POLL_DATA:
			if (arg == MXT_IOCTL_START_HEAVY){
				printk("touch sensor heavey\n");
			poll_mode = START_HEAVY;
			queue_delayed_work(sensor_work_queue, &mxt_poll_data_work, poll_mode);
			}
			else if (arg == MXT_IOCTL_START_NORMAL){
				printk("touch sensor normal\n");
				poll_mode = START_NORMAL;
				queue_delayed_work(sensor_work_queue, &mxt_poll_data_work, poll_mode);
			}
			else if  (arg == MXT_IOCTL_END){
			printk("touch sensor end\n");
			cancel_delayed_work_sync(&mxt_poll_data_work);
			}
			else
				return -ENOTTY;
			break;
		case MXT_FW_UPDATE:
			printk("+MXT_FW_UPDATE\n");
			return mxt_init_Boot(mxt);
		case MXT_FW_VERSION:
			dev_info(&mxt->client->dev, "Get Firmware firmware version!\n");
			mxt_identify(mxt->client, mxt, firmware_id);
			err = copy_to_user((char __user *)arg, firmware_id + 2, 1);
			return err;
		break;
	default: /* redundant, as cmd was checked against MAXNR */
	return -ENOTTY;
		}
	return 0;
}

	struct file_operations mxt_fops = {
		.owner =    THIS_MODULE,
		.unlocked_ioctl =	mxt_stress_ioctl,
		.open =		mxt_stress_open,
		.release =	mxt_stress_release,
		};

static ssize_t mxt_touch_switch_name(struct switch_dev *sdev, char *buf)
{
      struct mxt_data *mxt = globe_mxt;
	return sprintf(buf, "MXT-%d.%d build-%u\n", 
		        mxt->device_info.major, mxt->device_info.minor, mxt->device_info.build);
}

static ssize_t mxt_touch_switch_state(struct switch_dev *sdev, char *buf)
{ 
      	return sprintf(buf, "%s\n", "0");
}

static int __devinit mxt_probe(struct i2c_client *client,
			       const struct i2c_device_id *id)
{
	struct mxt_data *mxt;
	struct mxt_platform_data *pdata;
	struct input_dev *input;
	u8 *id_data;
	int error;
	int err;

	resume_flag = 0;
	delta_flag = false;
	cfg_flag = true;
	mxt_debug(DEBUG_INFO, "mXT224: mxt_probe\n");

	if (client == NULL) {
		pr_debug("maXTouch: client == NULL\n");
		return -EINVAL;
	} else if (client->adapter == NULL) {
		pr_debug("maXTouch: client->adapter == NULL\n");
		return -EINVAL;
	} else if (&client->dev == NULL) {
		pr_debug("maXTouch: client->dev == NULL\n");
		return -EINVAL;
	} else if (&client->adapter->dev == NULL) {
		pr_debug("maXTouch: client->adapter->dev == NULL\n");
		return -EINVAL;
	} else if (id == NULL) {
		pr_debug("maXTouch: id == NULL\n");
		return -EINVAL;
	}
      
	mxt_debug(DEBUG_INFO, "maXTouch driver\n");
	mxt_debug(DEBUG_INFO, "\t \"%s\"\n", client->name);
	mxt_debug(DEBUG_INFO, "\taddr:\t0x%04x\n", client->addr);
	mxt_debug(DEBUG_INFO, "\tirq:\t%d\n", client->irq);
	mxt_debug(DEBUG_INFO, "\tflags:\t0x%04x\n", client->flags);
	mxt_debug(DEBUG_INFO, "\tadapter:\"%s\"\n", client->adapter->name);
	mxt_debug(DEBUG_INFO, "\tdevice:\t\"%s\"\n", client->dev.init_name);

	/* Check if the I2C bus supports BYTE transfer */
	error = i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE);
	dev_info(&client->dev, "RRC:  i2c_check_functionality = %i\n", error);
	error = 0xff;
/*
	if (!error) {
		dev_err(&client->dev, "maXTouch driver\n");
		dev_err(&client->dev, "\t \"%s\"\n", client->name);
		dev_err(&client->dev, "\taddr:\t0x%04x\n", client->addr);
		dev_err(&client->dev, "\tirq:\t%d\n", client->irq);
		dev_err(&client->dev, "\tflags:\t0x%04x\n", client->flags);
		dev_err(&client->dev, "\tadapter:\"%s\"\n",
			client->adapter->name);
		dev_err(&client->dev, "\tdevice:\t\"%s\"\n",
			client->dev.init_name);
		dev_err(&client->dev, "%s adapter not supported\n",
				dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}
*/
	mxt_debug(DEBUG_TRACE, "maXTouch driver functionality OK\n");

	/* Allocate structure - we need it to identify device */
	mxt = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (mxt == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_mxt_alloc;
	}
	id_data = kmalloc(MXT_ID_BLOCK_SIZE, GFP_KERNEL);
	if (id_data == NULL) {
		dev_err(&client->dev, "insufficient memory\n");
		error = -ENOMEM;
		goto err_id_alloc;
	}

	input = input_allocate_device();
	if (!input) {
		dev_err(&client->dev, "error allocating input device\n");
		error = -ENOMEM;
		goto err_input_dev_alloc;
	}

	/* Initialize Platform data */

	pdata = client->dev.platform_data;
	if (pdata == NULL) {
		dev_err(&client->dev, "platform data is required!\n");
		error = -EINVAL;
		goto err_pdata;
	}
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "Platform OK: pdata = 0x%08x\n",
		       (unsigned int)pdata);

	mxt->read_fail_counter = 0;
	mxt->message_counter = 0;
	mxt->max_x_val = pdata->max_x;
	mxt->max_y_val = pdata->max_y;

	/* Get data that is defined in board specific code. */
	mxt->init_hw = pdata->init_platform_hw;
	mxt->exit_hw = pdata->exit_platform_hw;
	mxt->read_chg = pdata->read_chg;

	if (pdata->valid_interrupt != NULL)
		mxt->valid_interrupt = pdata->valid_interrupt;
	else
		mxt->valid_interrupt = mxt_valid_interrupt_dummy;

	if (mxt->init_hw != NULL)
		mxt->init_hw();

	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver identifying chip\n");

	mxt->client = client;
	if(isInBootLoaderMode(client)){
	    mxt_update_firmware(mxt, true);
	    msleep(200);
	} 
             
	   
	if (mxt_identify(client, mxt, id_data) < 0) {
		dev_err(&client->dev, "Chip could not be identified\n");
		error = -ENODEV;
		goto err_identify;
	}
	/* Chip is valid and active. */
	if (debug >= DEBUG_TRACE)
		printk(KERN_INFO "maXTouch driver allocating input device\n");

	mxt->client = client;
	mxt->input = input;
	mxt->hasTouch = false;

	mxt_client=mxt->client;
	INIT_DELAYED_WORK(&mxt->dwork, mxt_worker);
	mutex_init(&mxt->debug_mutex);
	mutex_init(&mxt->msg_mutex);
	mxt_debug(DEBUG_TRACE, "maXTouch driver creating device name\n");

	snprintf(mxt->phys_name,
		 sizeof(mxt->phys_name), "%s/input0", dev_name(&client->dev)
	    );
	input->name = "atmel-maxtouch";
	input->phys = mxt->phys_name;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;

	mxt_debug(DEBUG_INFO, "maXTouch name: \"%s\"\n", input->name);
	mxt_debug(DEBUG_INFO, "maXTouch phys: \"%s\"\n", input->phys);
	mxt_debug(DEBUG_INFO, "maXTouch driver setting abs parameters\n");

	set_bit(BTN_TOUCH, input->keybit);

	/* Single touch */
	input_set_abs_params(input, ABS_X, 0, mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_Y, 0, mxt->max_y_val, 0, 0);
	input_set_abs_params(input, ABS_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE,
			     0, 0);
	input_set_abs_params(input, ABS_TOOL_WIDTH, 0, MXT_MAX_REPORTED_WIDTH,
			     0, 0);

	/* Multitouch */
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, mxt->max_x_val, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, mxt->max_y_val, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_TOUCH_SIZE,
			     0, 0);
	input_set_abs_params(input, ABS_MT_PRESSURE, 0, MXT_MAX_REPORTED_PRESSURE, 
		           0, 0);
	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, MXT_MAX_NUM_TOUCHES,
			     0, 0);

	__set_bit(EV_ABS, input->evbit);
	__set_bit(EV_SYN, input->evbit);
	__set_bit(EV_KEY, input->evbit);
	//__set_bit(EV_TOUCH, input->evbit);

	mxt_debug(DEBUG_TRACE, "maXTouch driver setting client data\n");
	sema_init(&mxt->sem, 1); 
	mxt->interruptable = true;
	i2c_set_clientdata(client, mxt);
	mxt->status = 0;
	mxt_debug(DEBUG_TRACE, "maXTouch driver setting drv data\n");
	input_set_drvdata(input, mxt);
	mxt_debug(DEBUG_TRACE, "maXTouch driver input register device\n");
	error = input_register_device(mxt->input);
	if (error < 0) {
		dev_err(&client->dev, "Failed to register input device\n");
		goto err_register_device;
	}

	error = mxt_read_object_table(client, mxt, id_data);
	if (error < 0)
		goto err_read_ot;
 
      touch_vendor_id = tegra3_query_touch_module_pcbid();	 	
	dev_info(&client->dev, "The touch vendor is 0x%02X\n", touch_vendor_id);
	init_touch_config(mxt, touch_vendor_id); // start config  
	/* Create debugfs entries. */
	mxt->debug_dir = debugfs_create_dir("maXTouch", NULL);
	if (mxt->debug_dir == ERR_PTR(-ENODEV)) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "debugfs not enabled in kernel\n");
	} else if (mxt->debug_dir == NULL) {
		printk(KERN_WARNING "error creating debugfs dir\n");
	} else {
		mxt_debug(DEBUG_TRACE, "created \"maXTouch\" debugfs dir\n");

		debugfs_create_file("deltas", S_IRUSR, mxt->debug_dir, mxt,
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, mxt->debug_dir, mxt,
				    &refs_fops);
	}

	/* Create character device nodes for reading & writing registers */
	mxt->mxt_class = class_create(THIS_MODULE, "maXTouch_memory");
	/* 2 numbers; one for memory and one for messages */
	error = alloc_chrdev_region(&mxt->dev_num, 0, 2, "maXTouch_memory");
	mxt_debug(DEBUG_VERBOSE,
		  "device number %d allocated!\n", MAJOR(mxt->dev_num));
	if (error)
		printk(KERN_WARNING "Error registering device\n");
	cdev_init(&mxt->cdev, &mxt_memory_fops);
	cdev_init(&mxt->cdev_messages, &mxt_message_fops);

	mxt_debug(DEBUG_VERBOSE, "cdev initialized\n");
	mxt->cdev.owner = THIS_MODULE;
	mxt->cdev_messages.owner = THIS_MODULE;

	error = cdev_add(&mxt->cdev, mxt->dev_num, 1);
	if (error)
		printk(KERN_WARNING "Bad cdev\n");

	error = cdev_add(&mxt->cdev_messages, mxt->dev_num + 1, 1);
	if (error)
		printk(KERN_WARNING "Bad cdev\n");

	mxt_debug(DEBUG_VERBOSE, "cdev added\n");

	device_create(mxt->mxt_class, NULL, MKDEV(MAJOR(mxt->dev_num), 0), NULL,
		      "maXTouch");

	device_create(mxt->mxt_class, NULL, MKDEV(MAJOR(mxt->dev_num), 1), NULL,
		      "maXTouch_messages");

	mxt->msg_buffer_startp = 0;
	mxt->msg_buffer_endp = 0;
	  // add the touch char device
      init_touch_char_dev();  

	/* Allocate the interrupt */
	mxt_debug(DEBUG_TRACE, "maXTouch driver allocating interrupt...\n");
	mxt->irq = client->irq;
	mxt->valid_irq_counter = 0;
	mxt->invalid_irq_counter = 0;
	mxt->irq_counter = 0;
	if (mxt->irq) {
		/* Try to request IRQ with falling edge first. This is
		 * not always supported. If it fails, try with any edge. */
		error = request_irq(mxt->irq,
				    mxt_irq_handler,
				    IRQF_TRIGGER_FALLING,
				    client->dev.driver->name, mxt);
		if (error < 0) {
			/* TODO: why only 0 works on STK1000? */
			error = request_irq(mxt->irq,
					    mxt_irq_handler,
					    0, client->dev.driver->name, mxt);
		}

		if (error < 0) {
			dev_err(&client->dev,
				"failed to allocate irq %d\n", mxt->irq);
			goto err_irq;
		}
	}
	wake_lock_init(&mxt->wakelock, WAKE_LOCK_SUSPEND, "mxt_touch");
#ifdef CONFIG_HAS_EARLYSUSPEND
	mxt->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
	mxt->early_suspend.suspend = mxt_early_suspend;
	mxt->early_suspend.resume = mxt_early_resume;
	register_early_suspend(&mxt->early_suspend);
#endif

	sensor_work_queue = create_singlethread_workqueue("i2c_touchsensor_wq");
	if(!sensor_work_queue){
		pr_err("touch_probe: Unable to create workqueue");
		goto err_irq;
		}

	INIT_DELAYED_WORK(&mxt_poll_data_work, mxt_poll_data);
	mxt->misc_dev.minor  = MISC_DYNAMIC_MINOR;
	mxt->misc_dev.name = "touchpanel";
	mxt->misc_dev.fops = &mxt_fops;
	err = misc_register(&mxt->misc_dev);
		if (err) {
			pr_err("tegra_acc_probe: Unable to register %s \\misc device\n", mxt->misc_dev.name);
		goto misc_register_device_failed;
			}

      mxt->touch_sdev.name = "touch";
      mxt->touch_sdev.print_name = mxt_touch_switch_name;
	mxt->touch_sdev.print_state = mxt_touch_switch_state;
	if(switch_dev_register(&mxt->touch_sdev) < 0){
		dev_info(&client->dev, "switch_dev_register for dock failed!\n");
		//goto exit;
	}
	switch_set_state(&mxt->touch_sdev, 0);
	  
	if (debug > DEBUG_INFO)
		dev_info(&client->dev, "touchscreen, irq %d\n", mxt->irq);

	/* Schedule a worker routine to read any messages that might have
	 * been sent before interrupts were enabled. */
	cancel_delayed_work(&mxt->dwork);
	schedule_delayed_work(&mxt->dwork, 0);
	kfree(id_data);

mxt->attrs.attrs = mxt_attr;
	error = sysfs_create_group(&client->dev.kobj, &mxt->attrs);
	if (error) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		goto exit_remove;
	}
	mxt->status=1;
	globe_mxt = mxt;
	update_noise_state();
	return 0;

 exit_remove:
		sysfs_remove_group(&client->dev.kobj, &mxt->attrs);
 err_irq:
	kfree(mxt->rid_map);
	kfree(mxt->object_table);
	kfree(mxt->last_message);
misc_register_device_failed:
	misc_deregister(&mxt->misc_dev);
 err_read_ot:
 err_register_device:
 err_identify:
 err_pdata:
	input_free_device(input);
 err_input_dev_alloc:
	kfree(id_data);
 err_id_alloc:
	if (mxt->exit_hw != NULL)
		mxt->exit_hw();
	kfree(mxt);
 err_mxt_alloc:
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *mxt;

	mxt = i2c_get_clientdata(client);

	/* Remove debug dir entries */
	debugfs_remove_recursive(mxt->debug_dir);

	if (mxt != NULL) {

		if (mxt->exit_hw != NULL)
			mxt->exit_hw();

		if (mxt->irq)
			free_irq(mxt->irq, mxt);

		unregister_chrdev_region(mxt->dev_num, 2);
		device_destroy(mxt->mxt_class, MKDEV(MAJOR(mxt->dev_num), 0));
		device_destroy(mxt->mxt_class, MKDEV(MAJOR(mxt->dev_num), 1));
		cdev_del(&mxt->cdev);
		cdev_del(&mxt->cdev_messages);
		cancel_delayed_work_sync(&mxt->dwork);
		input_unregister_device(mxt->input);
		class_destroy(mxt->mxt_class);
		debugfs_remove(mxt->debug_dir);
		wake_lock_destroy(&mxt->wakelock);

		kfree(mxt->rid_map);
		kfree(mxt->object_table);
		kfree(mxt->last_message);
	}
	sysfs_remove_group(&client->dev.kobj, &mxt->attrs);
	exit_touch_char_dev(); // remove the char device
	kfree(mxt);

	i2c_set_clientdata(client, NULL);
	if (debug >= DEBUG_TRACE)
		dev_info(&client->dev, "Touchscreen unregistered\n");

	return 0;
}

static const struct i2c_device_id mxt_idtable[] = {
	{"maXTouch", 0,},
	{}
};

MODULE_DEVICE_TABLE(i2c, mxt_idtable);

static struct i2c_driver mxt_driver = {
	.driver = {
		   .name = "maXTouch",
		   .owner = THIS_MODULE,
		   },

	.id_table = mxt_idtable,
	.probe = mxt_probe,
	.remove = __devexit_p(mxt_remove),
#if !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = mxt_suspend,
	.resume = mxt_resume,
#endif
};

static int atmel_proc_read(char *buffer, char **buffer_location, off_t offset, int buffer_length, int *eof, void *data )
{
        int ret;

        printk("call proc_read!\n");

        if(offset > 0)  /* we have finished to read, return 0 */
                ret  = 0;
        else
		ret = sprintf(buffer, "Touch Flag Value: %s\n",touch_flag ?"TRUE":"FALSE");
        return ret;
}


static int atmel_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
        char procfs_buffer_size = 0;
        int i, ret;
        unsigned char procfs_buf[8+1] = {0};
        unsigned int command;

        printk("call proc write!\n");

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
                touch_flag = false;
		break;
      case 0xFF:
                //set the tag to reverse x axis
                touch_flag = true;
                break;
        }
        printk("Run command: 0x%08X  result:%d\n", command, ret);

        return count; // procfs_buffer_size;
}


static int __init mxt_init(void)
{
	int err;
	printk(KERN_INFO "%s+ #####\n", __func__);
	err = i2c_add_driver(&mxt_driver);
	if (err) {
		printk(KERN_WARNING "Adding maXTouch driver failed "
		       "(errno = %d)\n", err);
	} else {
		mxt_debug(DEBUG_TRACE, "Successfully added driver %s\n",
			  mxt_driver.driver.name);
		//Mars Add
		dbgProcFile = create_proc_entry(PROC_FS_NAME, 0666, NULL);
		if (dbgProcFile == NULL) 
		{
			remove_proc_entry(PROC_FS_NAME, NULL);
			printk(" Could not initialize /proc/%s\n", PROC_FS_NAME);
		}
		else
		{
			printk("==========================\n");
			dbgProcFile->read_proc = atmel_proc_read;
			dbgProcFile->write_proc = atmel_proc_write;
			printk(" /proc/%s created\n", PROC_FS_NAME);
		}
	}
	printk(KERN_INFO "%s- #####\n", __func__);
	return err;
}

static void __exit mxt_cleanup(void)
{
	i2c_del_driver(&mxt_driver);
}


module_init(mxt_init);
module_exit(mxt_cleanup);

MODULE_AUTHOR("Iiro Valkonen");
MODULE_DESCRIPTION("Driver for Atmel maXTouch Touchscreen Controller");
MODULE_LICENSE("GPL");
