/*
 * linux/drivers/video/backlight/pwm_bl.c
 *
 * simple PWM based backlight control, board code has to setup
 * 1) pin configuration so PWM waveforms can output
 * 2) platform_data being correctly configured
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/fb.h>
#include <linux/backlight.h>
#include <linux/err.h>
#include <linux/pwm.h>
#include <linux/pwm_backlight.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include "../gpio-names.h"
#include "../tegra/dc/dc_priv.h"
#include <linux/i2c.h>
#include <mach/board-cardhu-misc.h>

#define ENABLE_DEBUG_I2C_P1801 0        // define 1 to enable DEBUG statements
#if ENABLE_DEBUG_I2C_P1801
#define printk_DEBUG printk
#else
#define printk_DEBUG(format, args...) ((void)0)
#endif

/* For MIPI bridge IC */
#define DISPLAY_TABLE_END        1 /* special number to indicate this is end of table */
#define DISPLAY_MAX_RETRIES   3 /* max counter for retry I2C access */
#define DISPLAY_WAIT_MS          0 /* special number to indicate this is wait time require */
static int client_count = 0;
static struct i2c_client        *client_panel;
static struct display_reg {
        u16 addr;
        u16 val;
};
int I2C_command_flag = 1;

static int client_count_p1801 = 0;
static struct i2c_client        *client_panel_p1801;

#define MAX_LOOPS_RETRIES         20
u8 scalar_fw_version;
u8 scalar_fw_subversion;
u8 p1801_panel_type;
extern int scalar_update_status;	// 2: proceeding, -1: fail
#define scalar_status                   TEGRA_GPIO_PH4  //0:android, 1: windows
extern int brightness_flag;

#define cardhu_bl_enb			TEGRA_GPIO_PH2
static int P1801_writebacklight_flag = 0;

#define ME301T_panel_type_ID1 TEGRA_GPIO_PH7 //GMI_AD15
#define ME301T_panel_type_ID2 TEGRA_GPIO_PK7 //GMI_A19
int ME301T_panel_8v = 0;

static tegra_dc_bl_output cardhu_bl_output_measured_P1801 = {
	0, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 77, 77,
	77, 77, 77, 77, 77, 77, 78, 79,
	80, 81, 82, 83, 84, 85, 86, 87,
	88, 89, 90, 91, 92, 93, 94, 95,
	96, 97, 98, 99, 100, 101, 102, 103,
	104, 105, 106, 107, 108, 109, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 125, 126, 127,
	128, 129, 130, 131, 132, 133, 134, 135,
	136, 137, 138, 139, 140, 141, 142, 143,
	144, 145, 146, 147, 148, 149, 150, 151,
	152, 153, 154, 155, 156, 157, 158, 159,
	160, 161, 162, 163, 164, 165, 166, 167,
	168, 169, 170, 171, 172, 173, 174, 175,
	176, 177, 178, 179, 180, 181, 182, 183,
	184, 185, 186, 187, 188, 189, 190, 191,
	192, 193, 194, 195, 196, 197, 198, 199,
	200, 201, 202, 203, 204, 205, 206, 207,
	208, 209, 210, 211, 212, 213, 214, 215,
	216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231,
	232, 233, 234, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};
static p_tegra_dc_bl_output bl_output_P1801 = cardhu_bl_output_measured_P1801;

static tegra_dc_bl_output cardhu_bl_output_measured_ME301T = {
	0, 28, 28, 28, 28, 28, 28, 28,
	28, 28, 28, 28, 28, 28, 28, 28,
	28, 28, 28, 28, 28, 28, 28, 28,
	28, 28, 28, 28, 28, 29, 30, 31,
	32, 33, 34, 35, 36, 37, 38, 39,
	40, 41, 42, 43, 44, 45, 46, 47,
	48, 49, 50, 51, 52, 53, 54, 55,
	56, 57, 58, 59, 60, 61, 62, 63,
	64, 65, 66, 67, 68, 69, 70, 71,
	72, 73, 74, 75, 76, 77, 78, 79,
	80, 81, 82, 83, 84, 85, 86, 87,
	88, 89, 90, 91, 92, 93, 94, 95,
	96, 97, 98, 99, 100, 101, 102, 103,
	104, 105, 106, 107, 108, 109, 110, 111,
	112, 113, 114, 115, 116, 117, 118, 119,
	120, 121, 122, 123, 124, 125, 126, 127,
	128, 129, 130, 131, 132, 133, 134, 135,
	136, 137, 138, 139, 140, 141, 142, 143,
	144, 145, 146, 147, 148, 149, 150, 151,
	152, 153, 154, 155, 156, 157, 158, 159,
	160, 161, 162, 163, 164, 165, 166, 167,
	168, 169, 170, 171, 172, 173, 174, 175,
	176, 177, 178, 179, 180, 181, 182, 183,
	184, 185, 186, 187, 188, 189, 190, 191,
	192, 193, 194, 195, 196, 197, 198, 199,
	200, 201, 202, 203, 204, 205, 206, 207,
	208, 209, 210, 211, 212, 213, 214, 215,
	216, 217, 218, 219, 220, 221, 222, 223,
	224, 225, 226, 227, 228, 229, 230, 231,
	232, 233, 234, 235, 236, 237, 238, 239,
	240, 241, 242, 243, 244, 245, 246, 247,
	248, 249, 250, 251, 252, 253, 254, 255
};
static p_tegra_dc_bl_output bl_output_ME301T = cardhu_bl_output_measured_ME301T;

static atomic_t sd_brightness = ATOMIC_INIT(255);
extern struct tegra_dc *tegra_dcs[TEGRA_MAX_DC];

struct pwm_bl_data {
	struct pwm_device	*pwm;
	struct device		*dev;
	unsigned int		period;
	unsigned int		lth_brightness;
	int			(*notify)(struct device *,
					  int brightness);
	void			(*notify_after)(struct device *,
					int brightness);
	int			(*check_fb)(struct device *, struct fb_info *);
};

static int display_write_reg(struct i2c_client *client, u16 addr, u16 val)
{
        int err;
        struct i2c_msg msg;
        unsigned char data[4];
        int retry = 0;

        if (!client->adapter)
                return -ENODEV;

        data[0] = (u8) (addr >> 8);
        data[1] = (u8) (addr & 0xff);
        data[2] = (u8) (val >> 8);
        data[3] = (u8) (val & 0xff);

        msg.addr = client->addr;
        msg.flags = 0;
        msg.len = 4;
        msg.buf = data;
        do {
                err = i2c_transfer(client->adapter, &msg, 1);
                if (err == 1)
                        return 0;
                retry++;
                pr_err("display bridge IC : i2c transfer failed, retrying %x %x\n",
                               addr, val);
                pr_err("display bridge IC : i2c transfer failed, count %x \n",
                               msg.addr);
                //      msleep(3);
        } while (retry <= DISPLAY_MAX_RETRIES);

        return err;
}

static int display_write_table(struct i2c_client *client, const struct display_reg table[])
{
        int err;
        const struct display_reg *next;
        u16 val;

        //pr_info("DISPLAY_write_table %s\n",__func__);
        for (next = table; next->addr != DISPLAY_TABLE_END; next++) {
                if (next->addr == DISPLAY_WAIT_MS) {
                        msleep(next->val);
                        continue;
                }

                val = next->val;

                err = display_write_reg(client, next->addr, val);
                if (err)
                        return err;
        }
        return 0;
}

static int display_read_reg(struct i2c_client *client, u16 addr, u16 *val)
{
        int err;
        struct i2c_msg msg[2];
        unsigned char data[4];

        if (!client->adapter)
                return -ENODEV;

        msg[0].addr = 0x07;
        msg[0].flags = 0;
        msg[0].len = 2;
        msg[0].buf = data;

        /* high byte goes out first */
        data[0] = (u8) (addr >> 8);
        data[1] = (u8) (addr & 0xff);

        msg[1].addr = 0x07;
        msg[1].flags = 1;

        msg[1].len = 2;
        msg[1].buf = data + 2;

        err = i2c_transfer(client->adapter, msg, 2);

        printk("Check register high=%x \n",data[2]);
        printk("Check register low=%x \n",data[3]);

        if (err != 2)
                return -EINVAL;
        /*
                memcpy(val, data+2, 1);
                *val=*val&0xff;
        */
        return 0;
}

static int display_read_table(struct i2c_client *client,
                                   const struct display_reg table[])
{
        int err;
        const struct display_reg *next;
        u16 val;

        pr_info("DISPLAY_read_table %s\n",__func__);
        for (next = table; next->addr != DISPLAY_TABLE_END; next++) {
                if (next->addr == DISPLAY_WAIT_MS) {
                        msleep(next->val);
                        continue;
                }

                val = 0;

                err = display_read_reg(client, next->addr, val);
                if (err)
                        return err;
        }
        return 0;
}

static int P1801_write_backlight(struct i2c_client *client, u16 val_no_DIDIM, u16 val)
{
        int err;
        struct i2c_msg msg;
        unsigned char data[7];
        int retry = 0;

        if (!client->adapter)
                return -ENODEV;

        data[0] = 0x51;        //Source address
        data[1] = 0x84;        //length: 0x80 OR n: Last 4 bits indicates the number of following bytes (excluding checksum)
        data[2] = 0x03;        //Set VCP command
        data[3] = 0x10;        //VCP opcode, brightness=10
        data[4] = (u8) (val_no_DIDIM & 0xff);        //High Byte
        data[5] = (u8) (val & 0xff);        //Low Byte
        data[6] = 0x6e^data[0]^data[1]^data[2]^data[3]^data[4]^data[5];        //Check sum: Simple XOR of all preceding bytes (including the first)

        msg.addr = client->addr;
        msg.flags = 0;
        msg.len = 7;
        msg.buf = data;
        printk_DEBUG("%s: msg.addr=%x, data{0,1,2,3,4,5,6}={%x,%x,%x,%x,%x,%x,%x}, val_no_DIDIM=%d, val=%d\n", __func__, msg.addr, data[0], data[1], data[2], data[3], data[4], data[5], data[6], val_no_DIDIM, val);

        do {
                err = i2c_transfer(client->adapter, &msg, 1);
                if (err == 1)
                        return 0;
                retry++;
                pr_err("%s : i2c transfer failed, set backlight=%d(%x), retry time %d\n", __func__, val, val, retry);
        } while (retry <= DISPLAY_MAX_RETRIES);

        return err;
}

static int P1801_read_backlight(struct i2c_client *client, u16 *val)
{
        int err;
        struct i2c_msg msg[2];
        unsigned char data[16];

        if (!client->adapter)
                return -ENODEV;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = 5;
        msg[0].buf = data;

        data[0] = 0x51;        //Source address
        data[1] = 0x82;        //length
        data[2] = 0x01;        //Get VCP Feature command
        data[3] = 0x10;        //VCP opcode, brightness=10
        data[4] = 0x6e^data[0]^data[1]^data[2]^data[3];        //Check sum
        printk_DEBUG("%s: msg.addr=%x, data{0,1,2,3,4}={%x,%x,%x,%x,%x}\n", __func__, msg[0].addr, data[0], data[1], data[2], data[3], data[4]);

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;        //read data, from slave to master

        msg[1].len = 11;        //include check sum byte
        msg[1].buf = data + 5;

        printk_DEBUG("%s: msg[0] transfer start\n", __func__);
        err = i2c_transfer(client->adapter, msg, 1);
        if (err != 1)
                return -EINVAL;

        //from ddc/ci spec, need to wait 40ms in order to enable the decoding and preparation of the reply message in Monitor
        msleep(40);

        printk_DEBUG("%s: msg[1] transfer start\n", __func__);
        err = i2c_transfer(client->adapter, msg+1, 1);
        if (err != 1)
                return -EINVAL;


        printk_DEBUG("%s, check Source address=%x, length=%x, VCP feature reply opcode=%x\n", __func__,data[5], data[6], data[7]);
        printk_DEBUG("%s, check Result code=%x, VCP code=%x, VCP type code=%x\n", __func__,data[8], data[9], data[10]);
        printk_DEBUG("%s, check brightness max high=%x, max low=%x\n", __func__,data[11], data[12]);
        printk_DEBUG("%s, check brightness high=%x, low=%x\n", __func__,data[13], data[14]);
        printk_DEBUG("%s, check check sum=%x\n", __func__,data[15]);
        printk("%s, brightness high(PC)=%d, low(Pad)=%d\n", __func__,data[13], data[14]);

        memcpy(val, data+13, 1);
        printk_DEBUG("%s, check brightness value low=%x, brightness=%d\n", __func__,*val, *val);
        *val=*val&0xff;
        printk_DEBUG("%s, check brightness value low=%x, brightness=%d\n", __func__,*val, *val);

        if(data[6]!=0x88)		//invalid length
		return -EINVAL;

        return 0;
}

static int P1801_read_scalar_info(struct i2c_client *client)
{
        int err;
        struct i2c_msg msg[2];
        unsigned char data[16];

        if (!client->adapter)
                return -ENODEV;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = 5;
        msg[0].buf = data;

        data[0] = 0x51;        //Source address
        data[1] = 0x82;        //length
        data[2] = 0x01;        //Get VCP Feature command
        data[3] = 0xC9;        //VCP opcode, brightness=10, scalar_info=C9
        data[4] = 0x6e^data[0]^data[1]^data[2]^data[3];        //Check sum
        printk_DEBUG("%s: msg.addr=%x, data{0,1,2,3,4}={%x,%x,%x,%x,%x}\n", __func__, msg[0].addr, data[0], data[1], data[2], data[3], data[4]);

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;        //read data, from slave to master

        msg[1].len = 11;        //include check sum byte
        msg[1].buf = data + 5;

        printk_DEBUG("%s: msg[0] transfer start\n", __func__);
        err = i2c_transfer(client->adapter, msg, 1);
        if (err != 1)
                return -EINVAL;

        //from ddc/ci spec, need to wait 40ms in order to enable the decoding and preparation of the reply message in Monitor
        msleep(40);

        printk_DEBUG("%s: msg[1] transfer start\n", __func__);
        err = i2c_transfer(client->adapter, msg+1, 1);
        if (err != 1)
                return -EINVAL;

        printk_DEBUG("%s, check Source address=%x, length=%x, VCP feature reply opcode=%x\n", __func__,data[5], data[6], data[7]);
        printk_DEBUG("%s, check Result code=%x, VCP code=%x, VCP type code=%x\n", __func__,data[8], data[9], data[10]);
        printk_DEBUG("%s, check brightness max high=%x, max low=%x\n", __func__,data[11], data[12]);
        printk_DEBUG("%s, check brightness high=%x, low=%x\n", __func__,data[13], data[14]);
        printk_DEBUG("%s, check check sum=%x\n", __func__,data[15]);

        if(data[6]!=0x88 || data[11]!=0xFF || data[12]!= 0xFF)       //invalid length or invalid data
            return -EINVAL;

        scalar_fw_version = data[13];
        scalar_fw_subversion = (data[14] & 0xF0) >> 4;
        p1801_panel_type = data[14] & 0x0F;
        printk("%s: scalar_fw_version=%d, scalar_fw_subversion=%d, p1801_panel_type=%d\n", __func__, scalar_fw_version, scalar_fw_subversion, p1801_panel_type);

        return 0;
}

static void init_mipi_bridge(void) {
        int err;
        int bus = 0;
        struct i2c_msg msg[2];
        unsigned char data[4]={0,0,0,0};
        struct i2c_board_info	*info;
        struct i2c_adapter				*adapter;
        struct display_reg display_table[71] =
        {
                {0x0002, 0x0001}, //SYSctl, S/W Reset
                {DISPLAY_WAIT_MS, 0x05},
                {0x0002, 0x0000}, //SYSctl, S/W Reset release
                //{0x0010, 0xFFFD}, //GPIO1
                //{0x0014, 0x0002},
                {0x0016, 0x309F}, //PLL Control Register 0 (PLL_PRD,PLL_FBD)
                {0x0018, 0x0203}, //PLL_FRS,PLL_LBWS, PLL oscillation enable
                {DISPLAY_WAIT_MS, 0x05},
                {0x0018, 0x0213}, //PLL_FRS,PLL_LBWS, PLL clock out enable
                {0x0006, 0x012C}, //FIFO Control Register
                {0x0008, 0x0037}, //DSI-TX Format setting
                {0x0050, 0x003E}, //DSI-TX Pixel stream packet Data Type setting
                {0x0140, 0x0000}, //D-PHY Clock lane enable
                {0x0142, 0x0000},
                {0x0144, 0x0000}, //D-PHY Data lane0 enable
                {0x0146, 0x0000},
                {0x0148, 0x0000}, //D-PHY Data lane1 enable
                {0x014A, 0x0000},
                {0x014C, 0x0000}, //D-PHY Data lane2 enable
                {0x014E, 0x0000},
                {0x0150, 0x0000}, //D-PHY Data lane3 enable
                {0x0152, 0x0000},
                {0x0100, 0x0203},
                {0x0102, 0x0000},
                {0x0104, 0x0203},
                {0x0106, 0x0000},
                {0x0108, 0x0203},
                {0x010A, 0x0000},
                {0x010C, 0x0203},
                {0x010E, 0x0000},
                {0x0110, 0x0203},
                {0x0112, 0x0000},
                {0x0210, 0x1964}, //LINEINITCNT
                {0x0212, 0x0000},
                {0x0214, 0x0005}, //LPTXTIMECNT
                {0x0216, 0x0000},
                {0x0218, 0x2801}, //TCLK_HEADERCNT
                {0x021A, 0x0000},
                {0x021C, 0x0000}, //TCLK_TRAILCNT
                {0x021E, 0x0000},
                {0x0220, 0x0C06}, //THS_HEADERCNT
                {0x0222, 0x0000},
                {0x0224, 0x4E20}, //TWAKEUPCNT
                {0x0226, 0x0000},
                {0x0228, 0x000B}, //TCLK_POSTCNT
                {0x022A, 0x0000},
                {0x022C, 0x0005}, //THS_TRAILCNT
                {0x022E, 0x0000},
                {0x0230, 0x0005}, //HSTXVREGCNT
                {0x0232, 0x0000},
                {0x0234, 0x001F}, //HSTXVREGEN enable
                {0x0236, 0x0000},
                {0x0238, 0x0001}, //DSI clock Enable/Disable during LP
                {0x023A, 0x0000},
                {0x023C, 0x0005}, //BTACNTRL1
                {0x023E, 0x0005}, //Lucien something wrong
                {0x0204, 0x0001}, //STARTCNTRL
                {0x0206, 0x0000},
                {0x0620, 0x0001}, //Sync Pulse/Sync Event mode setting
                {0x0622, 0x0020}, //V Control Register1
                {0x0624, 0x001A}, //V Control Register2
                {0x0626, 0x04B0}, //V Control Register3
                {0x0628, 0x015E}, //H Control Register1
                {0x062A, 0x00FA}, //H Control Register2
                {0x062C, 0x1680}, //H Control Register3
                {0x0518, 0x0001}, //DSI Start
                {0x051A, 0x0000},
                {0x0500, 0x0086}, //DSI lane setting, DSI mode=HS
                {0x0502, 0xA300}, //bit set
                {0x0500, 0x8000}, //Switch to DSI mode
                {0x0502, 0xC300},
                {0x0004, 0x0044}, //Configuration Control Register
                {DISPLAY_TABLE_END, 0x0000}
        };

        if (client_count==0) {
                printk("Check create a new adapter \n");
                info = kzalloc(sizeof(struct i2c_board_info), GFP_KERNEL);
                info->addr = 0x07;
                adapter = i2c_get_adapter(bus);
                if (!adapter) {
                        printk("can't get adpater for bus %d\n", bus);
                        err = -EBUSY;
                        kfree(info);
                }

                client_panel = i2c_new_device(adapter, info);
                i2c_put_adapter(adapter);
                client_count++;
                kfree(info);
        }

        if (I2C_command_flag==0){
                I2C_command_flag=1;
                msg[0].addr = 0x07;
                msg[0].flags = 0;
                msg[0].len = 2;
                msg[0].buf = data;

                // high byte goes out first
                data[0] = 0;
                data[1] = 0;

                msg[1].addr = 0x07;
                msg[1].flags = 1;

                msg[1].len = 2;
                msg[1].buf = data + 2;

                err = i2c_transfer(client_panel->adapter, msg, 2);
                //printk("Check Chip ID=%x \n",data[2]);
                //printk("Check RevID=%x \n",data[3]);

                err = display_write_table(client_panel, display_table);
                printk("========================\n");
                printk("========================\n");
                //printk("Check start to read register\n");
                //err = display_read_table(client_panel, display_table);
                //mdelay(1000);

                if (gpio_get_value(TEGRA_GPIO_PI6)==0){	//panel is panasonic
                        printk("Panel is panasonic");
                        mdelay(35);
                }
                else{								//panel is hydis
                        printk("Panel is hydis");
                        mdelay(70);
                }
        }
}


static int pwm_backlight_update_status(struct backlight_device *bl)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;
	int max = bl->props.max_brightness;
	static int bl_enable_sleep_control = 0; // sleep only when suspend or resume
        int error = 0;
	int retry = 0;
	static int brightness_no_DIDIM = 100;

	if (bl->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bl->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

        if (tegra3_get_project_id()==TEGRA3_PROJECT_P1801 && client_count_p1801==0){
                int bus = 3;	// the same with hdmi edid bus
                struct i2c_board_info	*info;
                struct i2c_adapter		*adapter;

                printk("%s: P1801 create a new adapter for backlight setting, bus=%d\n", __func__, bus);
                info = kzalloc(sizeof(struct i2c_board_info), GFP_KERNEL);
                info->addr = 0x37;	// DDC/CI packets are transmitted using the I2C address 0x37
                adapter = i2c_get_adapter(bus);
                if (!adapter) {
                        printk("can't get adpater for bus %d\n", bus);
                        kfree(info);
                }

                client_panel_p1801 = i2c_new_device(adapter, info);
                i2c_put_adapter(adapter);

                if (!client_panel_p1801) {
                        printk("can't create new device for P1801 scalar i2c\n");
                        kfree(info);
                }

		msleep(100);
		error = P1801_read_scalar_info(client_panel_p1801);
		while (error != 0x00 && retry < MAX_LOOPS_RETRIES)
		{
			msleep(100);
			error = P1801_read_scalar_info(client_panel_p1801);
			retry += 1;
		}

                client_count_p1801++;
                kfree(info);
        }

	if(brightness_flag) {
		brightness_no_DIDIM = brightness;
		brightness_flag = 0;
	}

	if (brightness == 0) {
                if (tegra3_get_project_id()==TEGRA3_PROJECT_P1801
                                && client_count_p1801 !=0
                                && bl_enable_sleep_control
                                && scalar_update_status!=2
                                && scalar_update_status!=-1
                                && P1801_writebacklight_flag) {
                        //printk_DEBUG("%s: call P1801_write_backlight, brightness_no_DIDIM=%d, brightness=%d\n"
                        //                , __func__, brightness_no_DIDIM, brightness);
                        //error = P1801_write_backlight(client_panel_p1801, brightness_no_DIDIM, brightness);
                        //printk_DEBUG("%s: write error? =%d\n", __func__, error);
                        P1801_writebacklight_flag = 0;
                }

		if(tegra3_get_project_id()==TEGRA3_PROJECT_ME301T && ME301T_panel_8v) {
			if(bl_enable_sleep_control)
			{
				bl_enable_sleep_control = 0;
				pwm_config(pb->pwm, 0, pb->period);
				pwm_disable(pb->pwm);
				msleep(50);
				printk("%s: pwm disable (ME301T 8V panel)\n", __func__);
			}
			if (pb->notify)
				brightness = pb->notify(pb->dev, brightness);
		}
		else {
			if (pb->notify)
				brightness = pb->notify(pb->dev, brightness);
			if(bl_enable_sleep_control)
			{
				msleep(10);
				bl_enable_sleep_control = 0;
				pwm_config(pb->pwm, 0, pb->period);
				pwm_disable(pb->pwm);
				printk("%s: pwm disable\n", __func__);
			}
			I2C_command_flag=0;
		}
	} else {
		//move this part from board-cardhu-panel.c, cardhu_backlight_notify
		//because we need to enable LCD_BL_PWM before LCD_BL_EN
		//add DIDIM effect as below
		if(tegra_dcs[0])
			sd_brightness = *(tegra_dcs[0]->out->sd_settings->sd_brightness);
		int cur_sd_brightness = atomic_read(&sd_brightness);
		brightness = (brightness * cur_sd_brightness) / 255;

                //P1801 need to map the brightness below 30% to 30% (77)
                if (tegra3_get_project_id()==TEGRA3_PROJECT_P1801) {
                        if (WARN_ON(ARRAY_SIZE(cardhu_bl_output_measured_P1801) != 256))
                                pr_err("bl_output array does not have 256 elements\n");

                        /* Apply any backlight response curve */
                        if (brightness > 255 || brightness_no_DIDIM > 255) {
                                pr_info("Error: Brightness > 255!\n");
                        } else {
                                brightness = bl_output_P1801[brightness];
                                brightness_no_DIDIM = bl_output_P1801[brightness_no_DIDIM];
                        }
                }

		//ME301T need to map the brightness below 10% to 10% (28)
		if (tegra3_get_project_id()==TEGRA3_PROJECT_ME301T) {
			if (WARN_ON(ARRAY_SIZE(cardhu_bl_output_measured_ME301T) != 256))
				pr_err("bl_output array does not have 256 elements\n");

			/* Apply any backlight response curve */
			if (brightness > 255) {
				pr_info("Error: Brightness > 255!\n");
			} else {
				brightness = bl_output_ME301T[brightness];
			}
		}

		if (tegra3_get_project_id()==0x4 ){
                        init_mipi_bridge();
		}

                if (tegra3_get_project_id()==TEGRA3_PROJECT_P1801
                                && client_count_p1801 !=0
                                && scalar_update_status!=2
                                && scalar_update_status!=-1
                                && P1801_writebacklight_flag) {

                        if(!gpio_get_value(scalar_status)) {
                                printk_DEBUG("%s: call P1801_write_backlight, brightness_no_DIDIM=%d, brightness=%d\n"
                                                , __func__, brightness_no_DIDIM, brightness);
                                //the max value of p1801 hdmi display backlight is 255, need not convert
                                error = P1801_write_backlight(client_panel_p1801, brightness_no_DIDIM, brightness);
                                printk_DEBUG("%s: write error? =%d\n", __func__, error);
                        } else {
                                printk_DEBUG("%s: scalar display windows, cannot change backlight."
                                                , __func__);
                                printk_DEBUG("brightness_no_DIDIM=%d, brightness=%d\n"
                                                , brightness_no_DIDIM, brightness);
                        }
                }

		brightness = pb->lth_brightness +
			(brightness * (pb->period - pb->lth_brightness) / max);

		if(tegra3_get_project_id()==TEGRA3_PROJECT_ME301T && ME301T_panel_8v) {
			gpio_set_value(cardhu_bl_enb, !!brightness);
			if(!bl_enable_sleep_control)
			{
				msleep(50);
				bl_enable_sleep_control = 1;
				printk("%s: pwm enable (ME301T 8V panel)\n", __func__);
			}
			pwm_config(pb->pwm, brightness, pb->period);
			pwm_enable(pb->pwm);
		}
		else {
			pwm_config(pb->pwm, brightness, pb->period);
			pwm_enable(pb->pwm);
			if(!bl_enable_sleep_control)
			{
				msleep(10);
				bl_enable_sleep_control = 1;
				printk("%s: pwm enable\n", __func__);
			}

			gpio_set_value(cardhu_bl_enb, !!brightness);
		}
	}

	if (pb->notify_after)
		pb->notify_after(pb->dev, brightness);

	return 0;
}

static int pwm_backlight_get_brightness(struct backlight_device *bl)
{
        if (tegra3_get_project_id()==TEGRA3_PROJECT_P1801 && client_count_p1801 !=0) {
                int brightness = 0;
                int error = 0;
                error = P1801_read_backlight(client_panel_p1801, &brightness);
                printk_DEBUG("%s: brightness=%d, read error? = %d\n", __func__, brightness, error);
                if(error)
                        return error;
                else {
                        P1801_writebacklight_flag = 1;
                        return brightness;
                }
        }
        return bl->props.brightness;
}

static int pwm_backlight_check_fb(struct backlight_device *bl,
				  struct fb_info *info)
{
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	return !pb->check_fb || pb->check_fb(pb->dev, info);
}

static const struct backlight_ops pwm_backlight_ops = {
	.update_status	= pwm_backlight_update_status,
	.get_brightness	= pwm_backlight_get_brightness,
	.check_fb	= pwm_backlight_check_fb,
};

static int pwm_backlight_probe(struct platform_device *pdev)
{
	struct backlight_properties props;
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl;
	struct pwm_bl_data *pb;
	int ret;

	if (!data) {
		dev_err(&pdev->dev, "failed to find platform data\n");
		return -EINVAL;
	}

	if (data->init) {
		ret = data->init(&pdev->dev);
		if (ret < 0)
			return ret;
	}

	pb = kzalloc(sizeof(*pb), GFP_KERNEL);
	if (!pb) {
		dev_err(&pdev->dev, "no memory for state\n");
		ret = -ENOMEM;
		goto err_alloc;
	}

	if(tegra3_get_project_id() == TEGRA3_PROJECT_ME301T){
		printk("ME301T change pwm to 20K\n");
		data->pwm_period_ns = 50000;
	}

	if(gpio_get_value(ME301T_panel_type_ID1) && gpio_get_value(ME301T_panel_type_ID2)) {
		printk("ME301T 8V panel\n");
		ME301T_panel_8v = 1;
	}

	pb->period = data->pwm_period_ns;
	pb->notify = data->notify;
	pb->notify_after = data->notify_after;
	pb->check_fb = data->check_fb;
	pb->lth_brightness = data->lth_brightness *
		(data->pwm_period_ns / data->max_brightness);
	pb->dev = &pdev->dev;

	pb->pwm = pwm_request(data->pwm_id, "backlight");
	if (IS_ERR(pb->pwm)) {
		dev_err(&pdev->dev, "unable to request PWM for backlight\n");
		ret = PTR_ERR(pb->pwm);
		goto err_pwm;
	} else
		dev_dbg(&pdev->dev, "got pwm for backlight\n");

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = data->max_brightness;
	bl = backlight_device_register(dev_name(&pdev->dev), &pdev->dev, pb,
				       &pwm_backlight_ops, &props);
	if (IS_ERR(bl)) {
		dev_err(&pdev->dev, "failed to register backlight\n");
		ret = PTR_ERR(bl);
		goto err_bl;
	}

	bl->props.brightness = data->dft_brightness;
	backlight_update_status(bl);

	platform_set_drvdata(pdev, bl);
	return 0;

err_bl:
	pwm_free(pb->pwm);
err_pwm:
	kfree(pb);
err_alloc:
	if (data->exit)
		data->exit(&pdev->dev);
	return ret;
}

static int pwm_backlight_remove(struct platform_device *pdev)
{
	struct platform_pwm_backlight_data *data = pdev->dev.platform_data;
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	backlight_device_unregister(bl);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	pwm_free(pb->pwm);
	kfree(pb);
	if (data->exit)
		data->exit(&pdev->dev);
	return 0;
}

#ifdef CONFIG_PM
static int pwm_backlight_suspend(struct platform_device *pdev,
				 pm_message_t state)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);
	struct pwm_bl_data *pb = dev_get_drvdata(&bl->dev);

	if (pb->notify)
		pb->notify(pb->dev, 0);
	pwm_config(pb->pwm, 0, pb->period);
	pwm_disable(pb->pwm);
	if (pb->notify_after)
		pb->notify_after(pb->dev, 0);
	return 0;
}

static int pwm_backlight_resume(struct platform_device *pdev)
{
	struct backlight_device *bl = platform_get_drvdata(pdev);

	backlight_update_status(bl);
	return 0;
}
#else
#define pwm_backlight_suspend	NULL
#define pwm_backlight_resume	NULL
#endif

static struct platform_driver pwm_backlight_driver = {
	.driver		= {
		.name	= "pwm-backlight",
		.owner	= THIS_MODULE,
	},
	.probe		= pwm_backlight_probe,
	.remove		= pwm_backlight_remove,
	.suspend	= pwm_backlight_suspend,
	.resume		= pwm_backlight_resume,
};

static int __init pwm_backlight_init(void)
{
	return platform_driver_register(&pwm_backlight_driver);
}
module_init(pwm_backlight_init);

static void __exit pwm_backlight_exit(void)
{
	platform_driver_unregister(&pwm_backlight_driver);
}
module_exit(pwm_backlight_exit);

MODULE_DESCRIPTION("PWM based Backlight Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:pwm-backlight");

