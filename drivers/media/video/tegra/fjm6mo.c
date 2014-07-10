#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <linux/switch.h>
#include <media/yuv_sensor.h>
#include <mach/board-cardhu-misc.h>

#define CHECK_WRITE(x) {if (x) _T("err=%d",x);}
#define FJM6MO_DEBUG
//#undef FJM6MO_DEBUG
#ifdef FJM6MO_DEBUG
static int indent;
//extern int indent;
//#define LOG_TAG "FJM6MO"
#define BLANK "                    "
#define NEWLINE "\n"
#define PRINTF printk
#define _E(fmt, args...)  if (s_flag & FJM6MO_FLAG_DEBUG){PRINTF("%.*s+%s (%d):" fmt NEWLINE, indent, BLANK,__FUNCTION__,__LINE__, ##args); indent++;}
#define _X(fmt, args...)  if (s_flag & FJM6MO_FLAG_DEBUG){indent--; PRINTF("%.*s-%s (%d):" fmt NEWLINE, indent, BLANK,__FUNCTION__,__LINE__, ##args);}
#define _T(fmt, args...)  if (s_flag & FJM6MO_FLAG_DEBUG){PRINTF("%.*s%s (%d):" fmt NEWLINE, indent, BLANK,__FUNCTION__,__LINE__, ##args);}
#else
#define _E(fmt, args...)  while(0){}
#define _X(fmt, args...)  while(0){}
#define _T(fmt, args...)  while(0){}
#endif

#define FJM6MO_SENSOR_NAME        "fjm6mo"
#define FJM6MO_SDEV_NAME          "camera"
#define FIRMWARE_VERSION          0x9888
#define FJM6MO_FLAG_USE_CAPTURE   (0x1U << 0)
#define FJM6MO_FLAG_STOP_CAPTURE  (0x1U << 1)
#define FJM6MO_FLAG_DEBUG         (0x1U << 31)
#define FJM6MO_PARA_STATUS_ERR    0
#define FJM6MO_PARA_STATUS_PAR    0x01
#define FJM6MO_PARA_STATUS_MON    0x02
#define FJM6MO_PARA_STATUS_CAP    0x03
#define MAX_LOOPS_RETRIES         50

#define FLASH_ROM_ADDRESS         0x10000000
#define FLASH_ROM_FACT_ADDRESS    0x101F8000
#define FJM6MO_RAM_ADDRESS        0x68000000
#define FJM6MO_SECTOR_SIZE_TEMP   0x10000
#define FJM6MO_SECTOR_SIZE        0x0000
#define FJM6MO_SECTOR_SIZE_8      0x2000
#define FJM6MO_SECTOR_SIZE_1k     0x0200
#define FJM6MO_CHECKSUM_SIZE      0x8000

struct sensor_info {
    int mode;
    struct i2c_client *i2c_client;
    struct yuv_sensor_platform_data *pdata;
};
struct switch_dev   fjm6mo_sdev;
static struct sensor_info *info;

//extern unsigned int factory_mode;
static unsigned int factory_mode=0;

static bool update_mode = false;
static bool capture_mode = false;
static bool caf_mode = false;
static bool initial_mode = false;
static int ae_mode = 0;
static int awb_mode = 0;
static int touch_mode = TOUCH_STATUS_OFF;
static int fjm6mo_status = 0;
static int fjm6mo_update_status = 0;
static int fjm6mo_calibration_status = 0;
static unsigned char golden_value[4] = {0xFF, 0xFF, 0xFF, 0xFF};
static unsigned int s_flag = FJM6MO_FLAG_USE_CAPTURE;
static unsigned int preview_x = 0;
static unsigned int preview_y = 0;
static unsigned int version_num = 0xffffffff;
static unsigned int register_value = 0xffffffff;
static unsigned int shading_table = 0;
static unsigned int cur_camera_id = 0;

static u8 fw_chip_erase_pin1[16] = {
    0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x1f,
    0x76, 0x00, 0x18, 0x00, 0x00, 0x00, 0xff, 0xff
};
static u8 fw_chip_erase_pin2[16] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
static u8 fw_chip_erase_pin3[16] = {
    0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x3f,
    0xff, 0x00, 0x20, 0x11, 0x00, 0x00, 0xff, 0xff
};

int tegra_camera_mclk_on_off(int on);
int yuv_sensor_power_on_reset_pin(void);
int yuv_sensor_power_off_reset_pin(void);
static int sensor_change_status(E_M6MO_Status status);

static int fjm6mo_write_memory(struct i2c_client *client, u8* send_buf, u32 byte_size, u32 addr, u32 write_size)
{
    int err, retry = 0;
    struct i2c_msg msg;
    unsigned char data[600];
    memset(data, 66, 600);

    if (!client->adapter)
        return -ENODEV;

    data[0] = 0x0;
    data[1] = 4;
    data[2] = (u8)((addr & 0xFF000000) >> 24);
    data[3] = (u8)((addr & 0x00FF0000) >> 16);
    data[4] = (u8)((addr & 0x0000FF00) >>  8);
    data[5] = (u8)( addr & 0x000000FF);
    data[6] = (u8)((write_size & 0xFF00) >> 8);
    data[7] = (u8)(write_size & 0x00FF);

//    pr_info("addr:%x %x %x %x\n", data[2], data[3], data[4], data[5]);

    memcpy(data+8, send_buf, write_size);

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 8 + write_size;
    msg.buf = data;

    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1)
            return 0;
        retry += 1;
        pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n", addr, write_size);
        pr_err("yuv_sensor : i2c transfer failed, count %x \n", msg.addr);
    } while (retry <= SENSOR_MAX_RETRIES);
    CHECK_WRITE(err);
    return err;
}
static int fjm6mo_read_memory(struct i2c_client *client, u32 addr, u32 read_size, u32* val)
{
    int err;
    struct i2c_msg msg[2];
    unsigned char data[600];
    memset(data, 66, 600);

    if (!client->adapter)
        return -ENODEV;

    data[0] = 0x0;
    data[1] = 3;
    data[2] = (u8)((addr & 0xFF000000) >> 24);
    data[3] = (u8)((addr & 0x00FF0000) >> 16);
    data[4] = (u8)((addr & 0x0000FF00) >>  8);
    data[5] = (u8)( addr & 0x000000FF);
    data[6] = (u8)((read_size & 0xFF00) >> 8);
    data[7] = (u8)(read_size & 0x00FF);

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 8;
    msg[0].buf = data;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = (u16)(read_size & 0xFFFF) + 3;
    msg[1].buf = data + 8;

    err = i2c_transfer(client->adapter, msg, 2);
    if (err != 2)
        return -EINVAL;

    memcpy(val, data+11, read_size);

    return 0;
}

static int fjm6mo_write_register(struct i2c_client *client, u8 byte_num, u8 cat, u8 byte, u32 val)
{
    int err;
    struct i2c_msg msg;
    unsigned char data[32];
    int retry = 0;
    memset(data, 66, 32);

    if (!client->adapter)
        return -ENODEV;

    data[1] = 2;
    data[2] = cat;
    data[3] = byte;

    switch(byte_num)
    {
        case 1:
        {
            data[0] = 5;
            data[4] = (u8)(val & 0xff);
            break;
        }
        case 2:
        {
            data[0] = 6;
            data[4] = (u8)((val & 0xff00) >> 8);
            data[5] = (u8)(val & 0x00ff);
            break;
        }
        case 4:
        {
            data[0] = 8;
            data[4] = (u8)((val & 0xff000000) >> 24);
            data[5] = (u8)((val & 0x00ff0000) >> 16);
            data[6] = (u8)((val & 0x0000ff00) >> 8);
            data[7] = (u8)(val & 0x000000ff);
            break;
        }
        default:
            break;
    }

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = data[0];
    msg.buf = data;

//    printk("Cat:0x%x, Byte:0x%x, Value:0x%x\n", data[2], data[3], val);

    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1)
            return 0;
        retry++;
        pr_err("yuv_sensor : i2c transfer failed, retrying %x %x\n", cat, byte);
        pr_err("yuv_sensor : i2c transfer failed, count %x \n", msg.addr);
    } while (retry <= SENSOR_MAX_RETRIES);
    CHECK_WRITE(err);
    return err;
}

static int fjm6mo_read_register(struct i2c_client *client, u8 cat, u8 byte, u8 byte_num, u32* val)
{
    u32 err, ret;
    struct i2c_msg msg[2];
    unsigned char data[37];
    int retry = 0;

    memset(data, 66, 37);

    if (!client->adapter)
        return -ENODEV;

    do{
        data[0] = 5;
        data[1] = 1;
        data[2] = cat;
        data[3] = byte;
        data[4] = byte_num;
        data[5] = 0;

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = data[0];
        msg[0].buf = data;

        msg[1].addr = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = byte_num + 1;
        msg[1].buf = data + 5;

        retry += 1;

        err = i2c_transfer(client->adapter, msg, 2);
//        printk("!!!!!!read register: data[5]:0x%x data[6]:0x%x!!!!!!\n", data[5], data[6]);
        if (err != 2)
            return -EINVAL;
    }while(((data[5] & 0xfa) == 0xfa) && retry < MAX_LOOPS_RETRIES);

    switch(byte_num)
    {
        case 1:
        {
            ret = data[6];
            memcpy(val, data+6, byte_num);
            *val = *val & 0xff;
//            printk("read register: cat:0x%x byte:0x%x data[5]:0x%x data[6]:0x%x\n", data[2], data[3], data[5], data[6]);
            break;
        }
        case 2:
        {
            ret  = data[6] << 8;
            ret += data[7];
            swap(*(data+6),*(data+7));
            memcpy(val, data+6, byte_num);
            *val = *val & 0xffff;
//            printk("read register: cat:0x%x byte:0x%x data[5]:0x%x data[6]:0x%x data[7]:0x%x val:0x%x ret:0x%x\n", data[2], data[3], data[5], data[6], data[7], *val, ret);
            break;
        }
        case 4:
        {
            ret  = data[6] << 24;
            ret += data[7] << 16;
            ret += data[8] << 8;
            ret += data[9];
            swap(*(data+6),*(data+9));
            swap(*(data+7),*(data+8));
            memcpy(val, data+6, byte_num);
            *val = *val & 0xffffffff;
//            printk("read register: cat:0x%x byte:0x%x data[5]:0x%x data[6]:0x%x data[7]:0x%x data[8]:0x%x data[9]:0x%x val:0x%x ret:0x%x\n", data[2], data[3], data[5], data[6], data[7], data[8], data[9], *val, ret);
            break;
        }
        default:
            break;
    }

    return ret;
}
static int isp_interrupt(u8 interrupt_flag)
{
    int retry = 0;
    u32 temp;
    //Enable all type of interrupt factor
    msleep(10);
    pr_info("isp_interrupt\n");
    fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x10, 0xff);
    do{
        msleep(10);
        fjm6mo_read_register(info->i2c_client, 0x00, 0x1C, 0x01, &temp);
        retry += 1;
    } while((temp != interrupt_flag) && retry < (MAX_LOOPS_RETRIES * 2));
    if(temp != interrupt_flag)
        return -ENOMEM;
    else
        return 0;
}
static int fw_initial_purpose()
{
    int err = 0;

    pr_info("fjm6mo fw_initial_purpose\n");

    err = fjm6mo_write_memory(info->i2c_client, fw_chip_erase_pin1, 1, 0x50000300,  0x0010);
    if(err)
        return err;
    err = fjm6mo_write_memory(info->i2c_client, fw_chip_erase_pin2, 1, 0x50000100,  0x0010);
    if(err)
        return err;
    err = fjm6mo_write_memory(info->i2c_client, fw_chip_erase_pin3, 1, 0x50000200,  0x0010);
    if(err)
        return err;
    return 0;
}
static int fw_rom_erase(custom_fw_update_rom_package rom_cmd)
{
    u32 temp;
    int err, retry = 0;
    printk("fjm6mo fw_rom_erase set address\n");
    err = fjm6mo_write_register(info->i2c_client, 4, 0x0f, 0x00, rom_cmd.flash_rom_start_address);
    if(err){
        pr_err("fw_rom_erase set address error\n");
        return -ENOMEM;
    }
    if(rom_cmd.cmd)
        err = fjm6mo_write_register(info->i2c_client, 1, 0x0f, 0x06, 0x02);
    else
        err = fjm6mo_write_register(info->i2c_client, 1, 0x0f, 0x06, 0x01);
    if(err){
        pr_err("fw_rom_erase set chip-erase error\n");
        return -ENOMEM;
    }
    do{
        msleep(500);
        fjm6mo_read_register(info->i2c_client, 0x0f, 0x06, 0x01, &temp);
        retry += 1;
    } while (temp != 0x00 && retry < MAX_LOOPS_RETRIES);
    if(temp != 0){
        pr_err("fw_rom_erase chip-erase fail\n");
        return -ENOMEM;
    }
    return 0;
}
static int isp_power_on()
{
    if (!info || !info->pdata || !info->pdata->power_on)
        return -ENOMEM;

    info->pdata->power_on();
    tegra_camera_mclk_on_off(1);
    yuv_sensor_power_on_reset_pin();
    return 0;
}
static int isp_power_off()
{
    if (!info || !info->pdata || !info->pdata->power_on)
        return -ENOMEM;

    shading_table = 0;
    yuv_sensor_power_off_reset_pin();
    tegra_camera_mclk_on_off(0);
    info->pdata->power_off();
    return 0;
}
static int isp_cam_start()
{
    int err;
    msleep(10);
    pr_info("isp_cam_start\n");
    fjm6mo_write_register(info->i2c_client, 1, 0x0F, 0x12, 0x01);
    err = isp_interrupt(INT_STATUS_MODE);
    if(err){
        pr_err("isp_cam_start error");
        return -ENOMEM;
    }
    else{
        if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T){
            printk("set_camera as 0x%X\n", cur_camera_id);
            fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x3F, cur_camera_id);
            err = isp_interrupt(INT_STATUS_MODE);
            if(err){
                pr_err("set_camera error");
                return -ENOMEM;
            }
            fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x3E, 0x03);
            if(cur_camera_id == 0)
                fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x24);
            else
                fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x28);
        }
        else
            fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x24);
        return 0;
    }
}
static int isp_calibration_init(u32 table)
{
    u32 temp;
    int err;
    err = isp_power_on();
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_POWERON_FAIL;
        return -ENOMEM;
    }
    err = isp_cam_start();
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_INIT_FAIL;
        return -ENOMEM;
    }
    fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x07, 0x00);
    fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x00, 0x06);
    fjm6mo_write_register(info->i2c_client, 1, 0x0E, 0x31, table-1);
    if(factory_mode == 2)
        fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x01);
    else
        fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x02);
    fjm6mo_write_register(info->i2c_client, 1, 0x0E, 0x30, table);
    err = sensor_change_status(E_M6MO_Status_Monitor);
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_MONITOR_FAIL;
        return -ENOMEM;
    }
    msleep(3000);
    fjm6mo_read_register(info->i2c_client, 0x3, 0xE, 0x02, &temp);
    if(temp > 0x0078){
        pr_err("Sensor gain:0x%x\n", temp);
        fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_FAIL;
    }
    else
        fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_OK;
    return 0;
}
static int isp_calibration_capture(u32 table)
{
    u32 temp;
    int err;
    err = sensor_change_status(E_M6MO_Status_Capture);
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_CAPTURE_FAIL;
        return -ENOMEM;
    }
    err = sensor_change_status(E_M6MO_Status_Parameter);
    msleep(100);
    fjm6mo_write_register(info->i2c_client, 1, 0x0E, 0x3C, table);
    msleep(500);
    fjm6mo_read_register(info->i2c_client, 0xE, 0x3D, 0x02, &temp);
    if(temp == 0xABCD)
        fjm6mo_calibration_status = CALIBRATION_ISP_OK;
    else{
        pr_err("Calibration checksum:0x%x\n", temp);
        fjm6mo_calibration_status = CALIBRATION_ISP_CHECKSUM_FAIL;
    }
    return 0;
}
static int isp_calibration_aging(u32 table)
{
    u32 temp;
    int err;
    err = isp_power_on();
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_POWERON_FAIL;
        return -ENOMEM;
    }
    err = isp_cam_start();
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_INIT_FAIL;
        return -ENOMEM;
    }
    fjm6mo_write_register(info->i2c_client, 1, 0x0E, 0x3C, table);
    msleep(500);
    fjm6mo_read_register(info->i2c_client, 0xE, 0x3D, 0x02, &temp);
    if(temp == 0xABCD)
        fjm6mo_calibration_status = CALIBRATION_ISP_OK;
    else{
        pr_err("Calibration checksum:0x%x\n", temp);
        fjm6mo_calibration_status = CALIBRATION_ISP_CHECKSUM_FAIL;
    }
    err = isp_power_off();
    return 0;
}
static int isp_calibration_golden_init()
{
    u32 temp;
    int err;
    err = isp_power_on();
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_POWERON_FAIL;
        return -ENOMEM;
    }
    err = isp_cam_start();
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_INIT_FAIL;
        return -ENOMEM;
    }
    fjm6mo_write_register(info->i2c_client, 2, 0x0E, 0x27, 0xFFFF);
    fjm6mo_write_register(info->i2c_client, 2, 0x0E, 0x2B, 0xFFFF);
    fjm6mo_write_register(info->i2c_client, 2, 0x0E, 0x29, 0xFFFF);
    fjm6mo_write_register(info->i2c_client, 2, 0x0E, 0x2D, 0xFFFF);
    fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x24);
    if(factory_mode == 2)
        fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x01);
    else
        fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x02);
    fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x07, 0x00);
    err = sensor_change_status(E_M6MO_Status_Monitor);
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_MONITOR_FAIL;
        return -ENOMEM;
    }
    fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x47, 0x02);
    msleep(3000);
    fjm6mo_read_register(info->i2c_client, 0x3, 0xE, 0x02, &temp);
    if(temp > 0x0078){
        pr_err("Sensor gain:0x%x\n", temp);
        fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_FAIL;
    }
    else
        fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_OK;
    return 0;
}
static int isp_calibration_golden_result()
{
    u32 temp;
    int err;
    msleep(5000);
    fjm6mo_read_register(info->i2c_client, 0x0E, 0x1B, 0x01, &temp);
    if(temp != 0x01)
        fjm6mo_calibration_status = CALIBRATION_ISP_GOLDEN_FAIL;
    else{
        golden_value[0] = fjm6mo_read_register(info->i2c_client, 0x0E, 0x3F, 0x01, &temp);
        golden_value[1] = fjm6mo_read_register(info->i2c_client, 0x0E, 0x40, 0x01, &temp);
        golden_value[2] = fjm6mo_read_register(info->i2c_client, 0x0E, 0x41, 0x01, &temp);
        golden_value[3] = fjm6mo_read_register(info->i2c_client, 0x0E, 0x42, 0x01, &temp);
        fjm6mo_calibration_status = CALIBRATION_ISP_OK;
    }
    return 0;
}
static int isp_calibration_pgain_init(u32 val0, u32 val1, u32 val2, u32 val3)
{
    u32 temp;
    u32 r_gr, b_gb;
    int err;
    err = isp_power_on();
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_POWERON_FAIL;
        return -ENOMEM;
    }
    err = isp_cam_start();
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_INIT_FAIL;
        return -ENOMEM;
    }
    r_gr = ((val0 & 0x00FF) << 8) | (val1 & 0x00FF);
    b_gb = ((val2 & 0x00FF) << 8) | (val3 & 0x00FF);
    fjm6mo_write_register(info->i2c_client, 2, 0x0E, 0x27, r_gr);
    fjm6mo_write_register(info->i2c_client, 2, 0x0E, 0x2B, r_gr);
    fjm6mo_write_register(info->i2c_client, 2, 0x0E, 0x29, b_gb);
    fjm6mo_write_register(info->i2c_client, 2, 0x0E, 0x2D, b_gb);
    fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x24);
    if(factory_mode == 2)
        fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x01);
    else
        fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x02);
    fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x07, 0x00);
    err = sensor_change_status(E_M6MO_Status_Monitor);
    if(err){
        fjm6mo_calibration_status = CALIBRATION_ISP_MONITOR_FAIL;
        return -ENOMEM;
    }
    fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x47, 0x01);
    msleep(3000);
    fjm6mo_read_register(info->i2c_client, 0x3, 0xE, 0x02, &temp);
    if(temp > 0x0078){
        pr_err("Sensor gain:0x%x\n", temp);
        fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_FAIL;
    }
    else
        fjm6mo_calibration_status = CALIBRATION_LIGHT_SOURCE_OK;
    return 0;
}
static int isp_calibration_pgain_result(int wait_time)
{
    u32 temp;
    int err;
    msleep(wait_time * 1000);
    fjm6mo_read_register(info->i2c_client, 0x0E, 0x1B, 0x01, &temp);
    if(temp == 0x05)
        fjm6mo_calibration_status = CALIBRATION_ISP_OK;
    else{
        pr_err("Pgain fail:0x%x\n", temp);
        fjm6mo_calibration_status = CALIBRATION_ISP_PGAIN_FAIL;
    }
    return 0;
}

static int fw_checksum(u32* sum)
{
    u32 start_rom_address = FLASH_ROM_ADDRESS;
    u32 count = 0x001f8000 / FJM6MO_CHECKSUM_SIZE;
    u32 i = 0, j =0, temp_result = 0;
    u32 status, result;
    u16 ret_sum = 0;

    for(i = 0; i < count ; i += 1){
        fjm6mo_write_register(info->i2c_client, 4, 0x0f, 0x00, start_rom_address);
        fjm6mo_write_register(info->i2c_client, 2, 0x0f, 0x04, FJM6MO_CHECKSUM_SIZE);
        fjm6mo_write_register(info->i2c_client, 1, 0x0f, 0x09, 0x02);
        for(j = 0; j < 200 ; j +=1){
            msleep(5);
            result = fjm6mo_read_register(info->i2c_client, 0x0f, 0x09, 0x01, &status);
            if(status == 0x00){
                temp_result = fjm6mo_read_register(info->i2c_client, 0x0f, 0x0A, 0x02, &result);
                ret_sum += result;
                break;
            }
        }
        start_rom_address += FJM6MO_CHECKSUM_SIZE;
    }

    *sum = ret_sum;

    pr_info("fw_checksum:0x%x\n", ret_sum);

    return 0;
}

static int fw_crc_checksum(u32* sum)
{
    u32 start_rom_address = FLASH_ROM_ADDRESS;
    u32 count = 0x001f8000;
    u32 i = 0, j =0, times = 0, value;
    u16 CRC = 0xFFFF;
    u32 write_size = FJM6MO_SECTOR_SIZE_1k;
    u8 rom[FJM6MO_SECTOR_SIZE_1k];

    while(1){
        fjm6mo_read_memory(info->i2c_client, start_rom_address, write_size, rom);
        for(times = 0; times < write_size ; times += 2){
            CRC = (unsigned char)(CRC >> 8) | (CRC << 8);
            CRC ^= rom[times+1];
            CRC ^= (unsigned char)(CRC & 0xff) >> 4;
            CRC ^= (CRC << 8) << 4;
            CRC ^= ((CRC & 0xff) << 4) << 1;

            CRC = (unsigned char)(CRC >> 8) | (CRC << 8);
            CRC ^= rom[times];
            CRC ^= (unsigned char)(CRC & 0xff) >> 4;
            CRC ^= (CRC << 8) << 4;
            CRC ^= ((CRC & 0xff) << 4) << 1;
        }
        if((start_rom_address % FJM6MO_CHECKSUM_SIZE) == 0)
            pr_info("CRC checksum:0x%x 0x%x\n", start_rom_address, CRC);
        start_rom_address += write_size;
        if(start_rom_address >= 0x101f8000)
            break;
    }
    pr_info("CRC checksum:0x%x\n", CRC);

}

static char* get_fw_file_name()
{
    char *pFile;
    if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
        pFile = "/system/etc/firmware/camera/04-RS_M6Mo.bin";
    else if(tegra3_get_project_id() == TEGRA3_PROJECT_TF201)
        pFile = "/system/etc/firmware/camera/00-RS_M6Mo.bin";
    else{
        pFile = "/system/etc/firmware/camera/Wrong_Project_File";
        pr_err("Wrong project name\n");
    }
    return pFile;
}

static int bin_checksum(u32* sum)
{
    u32 start_rom_address = FLASH_ROM_ADDRESS;
    u32 count = 0x001f8000 / FJM6MO_CHECKSUM_SIZE;
    u32 i = 0, ret_sum = 0;
    u16 result = 0;
    u32 write_size = FJM6MO_SECTOR_SIZE_1k;
    u16 temp_result = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    char *pFile = get_fw_file_name();
    u8 buf[FJM6MO_SECTOR_SIZE_1k];
    int line_count = 0, times = 0;
    int err;

    pr_info("filp_open RS_M6Mo.bin\n");
    fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    memset(buf, 66, write_size);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            printk("file RS_M6Mo.bin start to read!\n");
            while (fp->f_op && fp->f_op->read) {
                fp->f_op->read(fp, buf, write_size, &fp->f_pos);
                for(times = 0; times < write_size ; times += 2){
                    temp_result = (buf[times] << 8) + buf[times+1];
                    result += temp_result;
                }
                line_count += 1;
                if(line_count % 64 == 0)
                    pr_info("start_rom_address:0x%x result=0x%x\n", start_rom_address, result);
                start_rom_address += write_size;
                if(start_rom_address >= 0x101f8000)
                    break;
            }
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
    }
    else if(PTR_ERR(fp) == -ENOENT){
        pr_err("file RS_M6Mo.bin not found error\n");
        return -ENOMEM;
    }
    else{
        pr_err("file RS_M6Mo.bin open error\n");
        return -ENOMEM;
    }

    *sum = result;

    pr_info("bin_checksum:0x%x\n", result);

    return 0;

}

static int fw_compare()
{
    u32 status;
    u32 start_rom_address = FLASH_ROM_ADDRESS;
    u32 write_size = FJM6MO_SECTOR_SIZE_1k;
    u16 i = 0, times = 0;
    u8 result = 0x01;

    struct file *fp = NULL;
    mm_segment_t old_fs;
    char *pFile = get_fw_file_name();
    u8 buf[FJM6MO_SECTOR_SIZE_1k];
    u8 rom[FJM6MO_SECTOR_SIZE_1k];
    int line_count = 0;
    int err;

    pr_info("filp_open RS_M6Mo.bin\n");
    fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    memset(buf, 66, write_size);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            printk("file RS_M6Mo.bin start to read!\n");
            while (fp->f_op && fp->f_op->read) {
                if(line_count % 1024 == 0)
                    pr_info("start_rom_address:0x%x \n", start_rom_address);
                fp->f_op->read(fp, buf, write_size, &fp->f_pos);
                line_count += 1;
                fjm6mo_read_memory(info->i2c_client, start_rom_address, write_size, rom);
                for(times = 0; times < write_size ; times += 1){
                    if(buf[times] != rom[times]){
                        pr_err("address:0x%x times:%d is different buf:0x%x rom:0x%x\n ", start_rom_address, times, buf[times], rom[times]);
                        break;
                    }
                }
                start_rom_address += write_size;
                if(start_rom_address >= 0x101f8000)
                    break;
            }
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
        return 0;
    }
    else if(PTR_ERR(fp) == -ENOENT){
        pr_err("file RS_M6Mo.bin not found error\n");
        return -ENOMEM;
    }
    else{
        pr_err("file RS_M6Mo.bin open error\n");
        return -ENOMEM;
    }
}

static int fw_init()
{
    int err;
    //Set M6MO pin(1) pin(2) pin(3)
    fjm6mo_update_status = 2;
    err = fw_initial_purpose();
    if(err){
        pr_err("fw_initial_purpose error");
        fjm6mo_update_status = 0;
        return -ENOMEM;
    }
    return 0;
}
static int fw_erase(custom_fw_update_rom_package rom_cmd)
{
    int err;
    //Set Falsh Rom address for chip erase
    err = fw_rom_erase(rom_cmd);
    if(err){
        pr_err("fw_rom_erase error");
        fjm6mo_update_status = 0;
        return -ENOMEM;
    }
    return 0;
}
static int fw_program(custom_fw_update_rom_package rom_cmd)
{
    int retry = 0;
    u32 result;
    if(rom_cmd.cmd){
        //Set flash rom address
        fjm6mo_write_register(info->i2c_client, 4, 0x0f, 0x00, rom_cmd.flash_rom_start_address);
        //Set programming byte
        fjm6mo_write_register(info->i2c_client, 2, 0x0f, 0x04, rom_cmd.program_size);
        //Clear m6mo internal ram
        fjm6mo_write_register(info->i2c_client, 1, 0x0f, 0x08, 0x01);
        retry = 0;
        do{
            msleep(10);
            fjm6mo_read_register(info->i2c_client, 0x0f, 0x08, 0x01, &result);
            retry += 1;
        } while (result != 0x00 && retry < MAX_LOOPS_RETRIES);
    }
    else{
        //Program
        fjm6mo_write_register(info->i2c_client, 1, 0x0f, 0x07, 0x01);
        retry = 0;
        do{
            msleep(50);
            fjm6mo_read_register(info->i2c_client, 0x0f, 0x07, 0x01, &result);
            retry += 1;
        } while (result != 0x00 && retry < MAX_LOOPS_RETRIES);
    }
    if(result != 0){
        pr_err("fw_program error\n");
        return -ENOMEM;
    }
    return 0;
}
static int fw_version()
{
    int err;
    u32 result = 0;
    u8 val[2] = {0xf6, 0xf6};
    err = fjm6mo_read_memory(info->i2c_client, 0x1016FFFC, 2, val);
    if(err)
        return -ENOMEM;
    version_num = val[0] << 16;
    version_num += val[1] << 8;
    err = fjm6mo_read_memory(info->i2c_client, 0x1016FFE4, 1, &result);
    if(err)
        return -ENOMEM;
    version_num += result;
    switch_set_state(&fjm6mo_sdev, !(fjm6mo_sdev.state));
    return err;
}
static void fw_update_fail()
{
    update_mode = false;
    version_num = 0xffffff;
    switch_set_state(&fjm6mo_sdev, !(fjm6mo_sdev.state));
}
static int fw_update()
{
    u32 status;
    u32 start_rom_address = FLASH_ROM_ADDRESS;
    u32 start_ram_address = FJM6MO_RAM_ADDRESS;
    u32 write_size = FJM6MO_SECTOR_SIZE_1k;
    u32 sector_size = FJM6MO_SECTOR_SIZE;
    u16 i = 0, times = 0;
    u8 result = 0x01;
    int retry = 0;
    custom_fw_update_rom_package fw_update;
    fw_update.cmd = 1;
    fw_update.flash_rom_start_address = start_rom_address;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    char *pFile = get_fw_file_name();
    u8 buf[FJM6MO_SECTOR_SIZE_1k];
    int line_count = 0;
    int err;

    //Set M6MO pin(1) pin(2) pin(3)
    fjm6mo_update_status = 2;
    update_mode = true;
    err = fw_initial_purpose();
    if(err){
        pr_err("fw_initial_purpose error");
        fjm6mo_update_status = 0;
        return -ENOMEM;
    }

    //Set Falsh Rom address for chip erase
    err = fw_rom_erase(fw_update);
    if(err){
        pr_err("fw_rom_erase error");
        fjm6mo_update_status = 0;
        return -ENOMEM;
    }

    pr_info("filp_open RS_M6Mo.bin\n");
    fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
    memset(buf, 66, write_size);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            printk("file RS_M6Mo.bin start to read!\n");
            while (fp->f_op && fp->f_op->read) {
                fp->f_op->read(fp, buf, write_size, &fp->f_pos);
                line_count += 1;
                start_rom_address += write_size;
                if(start_rom_address >= 0x101f8000)
                    break;
            }
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
    }
    else if(PTR_ERR(fp) == -ENOENT){
        pr_err("file RS_M6Mo.bin not found error\n");
        fjm6mo_update_status = 0;
        return -ENOMEM;
    }
    else{
        pr_err("file RS_M6Mo.bin open error\n");
        fjm6mo_update_status = 0;
        return -ENOMEM;
    }

    fp = filp_open(pFile, O_RDONLY, 0);
    if( !IS_ERR_OR_NULL(fp) ){
        start_rom_address = FLASH_ROM_ADDRESS;
        write_size = FJM6MO_SECTOR_SIZE_1k;
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        printk("file RS_M6Mo.bin start to update!\n");

        for(i = 0; i < line_count; i += 1){
            if(i % 128 == 0 || (i % 16 == 0 && sector_size == FJM6MO_SECTOR_SIZE_8)){
                pr_info("start rom address:0x%x\n", start_rom_address);
                //Set flash rom address
                fjm6mo_write_register(info->i2c_client, 4, 0x0f, 0x00, start_rom_address);
                //Set programming byte
                fjm6mo_write_register(info->i2c_client, 2, 0x0f, 0x04, sector_size);
                //Clear m6mo internal ram
                fjm6mo_write_register(info->i2c_client, 1, 0x0f, 0x08, 0x01);
                retry = 0;
                do{
                    msleep(10);
                    result = fjm6mo_read_register(info->i2c_client, 0x0f, 0x08, 0x01, &status);
                    retry += 1;
                } while (result != 0x00 && retry < MAX_LOOPS_RETRIES);
            }
            //Send programmed firmware
            memset(buf, 0, write_size);
            if(fp->f_op && fp->f_op->read)
                fp->f_op->read(fp, buf, write_size, &fp->f_pos);
            else{
                pr_err("Read RS_M6Mo.bin fail\n");
                break;
            }
            fjm6mo_write_memory(info->i2c_client, buf, 1, start_ram_address,  write_size);
            start_ram_address += write_size;
            if( i % 128 == 127 || (i % 16 == 15 && sector_size == FJM6MO_SECTOR_SIZE_8)){
                start_ram_address = FJM6MO_RAM_ADDRESS;
                //Program
                fjm6mo_write_register(info->i2c_client, 1, 0x0f, 0x07, 0x01);
                retry = 0;
                do{
                    msleep(50);
                    result = fjm6mo_read_register(info->i2c_client, 0x0f, 0x07, 0x01, &status);
                    retry += 1;
                } while (result != 0x00 && retry < MAX_LOOPS_RETRIES);
                if(sector_size == 0){
                    start_rom_address += 0x10000;
                }
                else{
                    start_rom_address += sector_size;
                }
                if(start_rom_address >= 0x101f0000)
                    sector_size = FJM6MO_SECTOR_SIZE_8;
            }
            if(start_rom_address >= 0x101f8000)
                break;
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
    }
    else{
        pr_err("file RS_M6Mo.bin open error");
        fjm6mo_update_status = 0;
        return -ENOMEM;
    }
    pr_info("start_rom_address:0x%x start_ram_address:0x%x line_count:%d\n", start_rom_address, start_ram_address, line_count);

    err = fw_version();
    if(err){
        pr_err("isp query version error \n");
        fjm6mo_update_status = 0;
        return -ENOMEM;
    }
    fjm6mo_update_status = 1;
    update_mode = false;
    return 0;
}

static int sensor_get_status(void)
{
    u32 buffer;
    fjm6mo_read_register(info->i2c_client, 0x00, 0x0B, 0x01, &buffer);
    return buffer;
}
static int sensor_change_status(E_M6MO_Status status)
{
    u8 parameter;
    int result = 0x00, retry = 0, err;
    u32 buffer;

    switch(status)
    {
        case E_M6MO_Status_Parameter:
        {
            parameter = FJM6MO_PARA_STATUS_PAR;
            break;
        }
        case E_M6MO_Status_Monitor:
        {
            parameter = FJM6MO_PARA_STATUS_MON;
            break;
        }
        case E_M6MO_Status_Capture:
        {
            parameter = FJM6MO_PARA_STATUS_CAP;
            break;
        }
        default:
        {
            parameter = FJM6MO_PARA_STATUS_ERR;
            break;
        }
    }

    pr_info("fjm6mo sensor_change_status: %d\n", parameter);

    if(parameter == FJM6MO_PARA_STATUS_MON)
    {
        //Stop continuous ouput capture mode
        if(capture_mode){
            _T("Stop continuous ouput frame from capture mode\n");
            capture_mode = false;
            if(!(s_flag & 0x2))
                fjm6mo_write_register(info->i2c_client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
            err = isp_interrupt(INT_STATUS_CAPTURE);
            if(err)
                pr_err("Select image frame fail\n");
        }

        //Go to the monitor mode
        fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x0B, parameter);
        err = isp_interrupt(INT_STATUS_MODE);
        if(err){
            pr_err("Change to monitor mode fail\n");
            _T("Change to monitor mode fail\n");
            if(factory_mode == 2)
                return -ENOMEM;
        }
        fjm6mo_read_register(info->i2c_client, 0x0A, 0x02, 0x01, &buffer);
        if(buffer == 0x2)
        {
            pr_info("Release autofocus\n");
            if(!caf_mode)
                fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x03);
            else {
                fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x41, 0x04);
                fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x01);
            }
            //Set af_window back to center or face_window
            fjm6mo_read_register(info->i2c_client, 0x09, 0x00, 0x01, &buffer);
            if(buffer == 0x1)
                fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x40, 0x03);
            else{
                fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x40, 0x01);
                if(factory_mode == 2)
                    fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x40, 0x06);
            }
        }
        fjm6mo_read_register(info->i2c_client, 0x03, 0x0, 0x01, &buffer);
        if(buffer != ae_mode){
            pr_info("Set ae_lock %d\n", ae_mode);
            fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0, ae_mode);
        }
        fjm6mo_read_register(info->i2c_client, 0x06, 0x0, 0x01, &buffer);
        if(buffer != awb_mode){
            pr_info("Set awb_lock %d\n", awb_mode);
            fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x0, awb_mode);
        }
    }
    else if(parameter == FJM6MO_PARA_STATUS_PAR){
        fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x0B, parameter);
        retry = 0;
        do{
            msleep(10);
            result = sensor_get_status();
            retry += 1;
        }while(result != 0x1 && retry < MAX_LOOPS_RETRIES);
        if(result != 0x1){
            pr_err("Change to parameter mode fail\n");
            _T("Change to parameter mode fail\n");
            if(factory_mode == 2)
                return -ENOMEM;
        }
    }
    else if(parameter == FJM6MO_PARA_STATUS_CAP){
        int int_sticky = 0x88;
        fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x10, 0xFF);
        fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x0B, parameter);
        retry = 0;
        do{
            msleep(10);
            result = fjm6mo_read_register(info->i2c_client, 0x00, 0x1C, 0x01, &buffer);
            CHECK_WRITE((result<0));
            if ((buffer & 0x80) && (buffer != 0xfa))
            {
                int_sticky &= ~0x80;
            }
            if ((buffer & 0x08) && (buffer != 0xfa))
            {
                int_sticky &= ~0x08;
            }
            retry += 1;
        } while(int_sticky != 0x00 && retry < MAX_LOOPS_RETRIES);
        if(int_sticky != 0x00){
            pr_err("Change to capture mode fail\n");
            _T("Change to capture mode fail\n");
            return -ENOMEM;
        }
    }

    return 0;
}

static int sensor_set_mode(struct sensor_info *info, struct sensor_mode *mode)
{
    int err = 0, retry = 0;
    u32 status;
    u8 result = 0x01;

    pr_info("%s: xres %u yres %u\n",__func__, mode->xres, mode->yres);

    status = sensor_get_status();
//+
    if ((s_flag & 0x1) && (status == FJM6MO_PARA_STATUS_MON) && (mode->xres==3264 && mode->yres == 2448)) //using capture mode for 8MP streaming.
    {
        u32 temp=0;
        u32 int_sticky=0x88;
        capture_mode = true;

        _T("===========set_mode: 8MP-capture\n");
        printk("===========set_mode: 8MP-capture\n");
        err = fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x10, 0x88); //enable interrupt signal: capture interrupt.
        CHECK_WRITE(err);
        err = fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x0B, 0x03); // start single capture
        CHECK_WRITE(err);
        do{
            retry += 1;
            msleep(10);
            result = fjm6mo_read_register(info->i2c_client, 0x00, 0x1C, 0x01, &temp);
            CHECK_WRITE((result<0));
            _T("result=%d",result);
            if ((temp & 0x80) && (temp != 0xfa))
            {
                int_sticky &= ~0x80;
            }
            if ((temp & 0x08) && (temp != 0xfa))
            {
                int_sticky &= ~0x08;
            }
        } while(int_sticky != 0x00 && retry < (MAX_LOOPS_RETRIES*4));
        _T("capture done. temp=0x%X",temp);

        _T("start transfer...");
        printk("start transfer...\n");
        err = fjm6mo_write_register(info->i2c_client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
        CHECK_WRITE(err);
        err = fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x00, 0x00); //YUV422
        CHECK_WRITE(err);
        err = fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x1, 0x25); // 8MP output
        CHECK_WRITE(err);
        err = fjm6mo_write_register(info->i2c_client, 1, 0x0C, 0x9, 0x01); // start transfer
        CHECK_WRITE(err);

        return 0;
    }
//-

    if((preview_x != mode->xres) || (preview_y != mode->yres)){
        if(status != FJM6MO_PARA_STATUS_PAR){
            err = sensor_change_status(E_M6MO_Status_Parameter);
            status = sensor_get_status();
            if(err)
                return err;
        }
        if (mode->xres == 640 && mode->yres == 480){
            printk("===========set_mode: 640x480\n");
            err = fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x17);
            if(err)
                return err;
        }
        else if(mode->xres == 1280 && mode->yres == 960){
            printk("===========set_mode: 1280X960\n");
            err = fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x24);
            if(err)
                return err;
            // Dynamic frame rate
            err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0A, 0x0);
            if(err)
                return err;
        }
        else if(mode->xres == 3264 && mode->yres == 2448){
            printk("===========set_mode: 3264x2448\n");
            err = fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x27);
            if(err)
                return err;
        }
        else if(mode->xres == 1920 && mode->yres == 1080){
            printk("===========set_mode: 1920x1080\n");
            err = fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x28);
            if(err)
                return err;
            // Fix 30fps
            err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0A, 0x01);
            if(err)
                return err;
        }
        else{
            pr_err("===========wrong mode\n");
            return -EFAULT;
        }
        preview_x = mode->xres;
        preview_y = mode->yres;
    }

    if(status != FJM6MO_PARA_STATUS_MON)
        err = sensor_change_status(E_M6MO_Status_Monitor);
    if(err)
        return err;
    return 0;
}

static void sensor_start_af(struct sensor_info *info)
{
    int err;
    u32 result, buffer;
    // if CAF on
    if(caf_mode){
        result = fjm6mo_read_register(info->i2c_client, 0x0A, 0x17, 0x01, &buffer);
        //0 search 1 stable
        pr_info("CAF search status:0x%x\n", buffer);
        fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x2e, 0x1);
        fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x0);
        //Stop CAF
        err = isp_interrupt(INT_STATUS_AF);
        if(err)
            pr_err("CAF stop interrupt error");
        if(touch_mode == TOUCH_STATUS_ON){
            fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x41, 0x3);
            fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x1);
        }
    }
    else
        fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x1);
}

static int sensor_initial(struct sensor_info *info, u32 camera_id)
{
    int err, result;
    fw_version();
    if(((version_num&0xffffff) != 0xffffff)){
        pr_info("sensor_initial start\n");
        result = fjm6mo_write_register(info->i2c_client, 1, 0x0F, 0x12, 0x01);
        err = isp_interrupt(INT_STATUS_MODE);
        if(!err){
            initial_mode = true;
            if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T){
                printk("set_camera as 0x%X\n", camera_id);
                fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x3F, camera_id);
                err = isp_interrupt(INT_STATUS_MODE);
                if(err){
                    pr_err("set_camera error");
                    return -ENOMEM;
                }
                fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x3E, 0x03);
                if(camera_id == 1){
                    preview_x = 1920;
                    preview_y = 1080;
                    fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x28);
                }
            }
            if(camera_id == 0){
                preview_x = 1280;
                preview_y = 960;
                fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x01, 0x24);
            }
            //Continues ouput the same frame
            if(!(s_flag & 0x2))
                fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x31, 0x01);
            //Dynamic frame rate
            fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x02, 0x01);
            fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0A, 0x00);
            //Set Flickering auto or 60HZ
            if(factory_mode == 2){
                fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x01);
                err = fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x40, 0x06);
            }
            else
                fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x04);
            //Set Shadding on
            fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x07, 0x01);
            //Set shading table for calibration
            if(shading_table != 0){
                fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x2C, 0x01);
                fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x07, 0x02);
                fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x30, shading_table);
                fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x31, shading_table);
            }
            //Set flash to auto
            fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x1F, 0x02);
            //Set whitebalance to auto
            //fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x02, 0x01);
            switch_set_state(&fjm6mo_sdev, !(fjm6mo_sdev.state));
            return 0;
        }
        else{
            pr_err("sensor_initial error : isp cam start no interrupt\n");
            _T("sensor_initial error : no interrupt");
        }
    }
    return -EBUSY;
}

static long sensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct sensor_info *info = file->private_data;
    int err = 0, retry = 0;
    u32 result;
    u32 buffer;

    switch (cmd)
    {
        case SENSOR_CUSTOM_IOCTL_INITIAL:
        {
            u32 camera_id;
            if (copy_from_user(&camera_id, (const void __user *)arg, sizeof(camera_id))) {
                return -EFAULT;
            }
            if(!initial_mode){
                err = sensor_initial(info, camera_id);
                if(err)
                    return err;
            }
            break;
        }
        case SENSOR_IOCTL_SET_MODE:
        {
            struct sensor_mode mode;
            if (copy_from_user(&mode, (const void __user *)arg, sizeof(struct sensor_mode))) {
                return -EFAULT;
            }
            err = sensor_set_mode(info, &mode);
            if(err)
                return err;
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_AF_MODE:
        {
            custom_af_cmd_package AF_cmd;
            if (copy_from_user(&AF_cmd,(const void __user *)arg, sizeof(AF_cmd)))
                return -EFAULT;
            switch(AF_cmd.cmd)
            {
                case AF_CMD_START:
                {_T("Start autofocus");
                    //Start autofocus
                    if(touch_mode == TOUCH_STATUS_DONE)
                        break;
                    sensor_start_af(info);
                    pr_info("Start autofocus\n");
                    break;
                }
                case AF_CMD_ABORT:
                    break;
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_GET_AF_MODE:
        {
            custom_af_cmd_package AF_cmd;
            if (copy_from_user(&AF_cmd,(const void __user *)arg, sizeof(AF_cmd)))
            {
                return -EFAULT;
            }
            pr_info("Get AF status\n");
            switch(AF_cmd.cmd)
            {
                case AF_CMD_GET_AF_STATUS:
                {_T("SENSOR_CUSTOM_IOCTL_GET_AF_MODE: AF_CMD_GET_AF_STATUS");
                    if((!caf_mode && (touch_mode != TOUCH_STATUS_DONE)) || (touch_mode == TOUCH_STATUS_ON)){
                        result = fjm6mo_read_register(info->i2c_client, 0x00, 0x1C, 0x01, &buffer);
                        if(buffer == 2){
                            AF_cmd.data = 1; //Locked
                            //Read AF result
                            result = fjm6mo_read_register(info->i2c_client, 0x0A, 0x03, 0x01, &buffer);
                            pr_info("Read AF result\n");
                            switch(buffer)
                            {
                                case 0:
                                    pr_info("AF operating\n");
                                    break;
                                case 1:
                                    pr_info("AF success\n");
                                    break;
                                case 2:
                                    pr_info("AF fail\n");
                                    break;
                                case 3:
                                    pr_info("AF stopped at edge\n");
                                    break;
                            }
                            if(touch_mode == TOUCH_STATUS_ON){
                                if(caf_mode){
                                    fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x41, 0x6);
                                    fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x01);
                                }
                                touch_mode = TOUCH_STATUS_DONE;
                            }
                            AF_cmd.data = buffer;
                        }
                        else
                            AF_cmd.data = 0; //busy
                    }
                    else if(caf_mode){
                        fjm6mo_read_register(info->i2c_client, 0x0A, 0x17, 0x01, &buffer);
                        pr_info("Read CAF result %d\n", buffer);
                        if(buffer == 1)
                            AF_cmd.data = 1; //success
                        else
                            AF_cmd.data = 2; //fail
                    }
                    copy_to_user((const void __user *)arg, &AF_cmd, sizeof(AF_cmd));
                    break;
                }
                default:
                    pr_info("AF cmd %d not implemented yet\n",AF_cmd.cmd);
                return -1;
            }
            return 0;
        }
        case SENSOR_IOCTL_SET_COLOR_EFFECT:
        {
            u16 coloreffect;
            if (copy_from_user(&coloreffect,(const void __user *)arg,
                    sizeof(coloreffect))) {
                return -EFAULT;
            }
            printk("SET_COLOR_EFFECT as %d\n", coloreffect);
            switch(coloreffect)
            {
                u16 val;
                case YUV_ColorEffect_None:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x0);
                    break;
                case YUV_ColorEffect_Sepia:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x01);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x09, 0xDB);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0A, 0x18);
                    break;
                case YUV_ColorEffect_Mono:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x01);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x09, 0x00);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0A, 0x00);
                    break;
                case YUV_ColorEffect_Negative:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x02);
                    break;
                case YUV_ColorEffect_Vivid:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x04);
                    break;
                case YUV_ColorEffect_WaterColor:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x03);
                    break;
                case YUV_ColorEffect_Red:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x01);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x09, 0x00);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0A, 0x6B);
                    break;
                case YUV_ColorEffect_Blue:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x01);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x09, 0x40);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0A, 0x00);
                    break;
                case YUV_ColorEffect_Yellow:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0B, 0x01);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x09, 0x80);
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x0A, 0x00);
                    break;
                default:
                    break;
            }

            if (err)
                return err;
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
            if(whitebalance != YUV_Whitebalance_Auto)
                err = fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x02, 0x02);
            else
                err = fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x02, 0x01);

            switch(whitebalance)
            {
                case YUV_Whitebalance_Incandescent:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x03, 0x01);
                    break;
                case YUV_Whitebalance_Daylight:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x03, 0x04);
                    break;
                case YUV_Whitebalance_Fluorescent:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x03, 0x02);
                    break;
                case YUV_Whitebalance_CloudyDaylight:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x03, 0x05);
                    break;
                case YUV_Whitebalance_WarmFluorescent:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x03, 0x03);
                    break;
                case YUV_Whitebalance_Shade:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x03, 0x06);
                    break;
                default:
                    break;
            }
            if (err)
                return err;
            return 0;
        }
        case SENSOR_CUSTOM_IOCTL_SET_EV:
        {
            short ev;
            if (copy_from_user(&ev,(const void __user *)arg, sizeof(short))){
                return -EFAULT;
            }
            printk("SET_EV as %d\n", ev);
            switch(ev)
            {
                case -6:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x0A);
                    break;
                case -5:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x0E);
                    break;
                case -4:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x11);
                    break;
                case -3:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x14);
                    break;
                case -2:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x18);
                    break;
                case -1:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x1B);
                    break;
                case 0:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x1E);
                    break;
                case 1:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x21);
                    break;
                case 2:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x24);
                    break;
                case 3:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x28);
                    break;
                case 4:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x2B);
                    break;
                case 5:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x2E);
                    break;
                case 6:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x32);
                    break;
                default:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x09, 0x1E);
                    break;
            }
            if (err)
                return err;
            return 0;
        }
        case SENSOR_CUSTOM_IOCTL_GET_EV:
        {
            short ev;
            fjm6mo_read_register(info->i2c_client, 0x03, 0x09, 0x01, &buffer);
            printk("GET_EV as value:0x%X\n", buffer);
            switch(buffer)
            {
                case 0x0A:
                    ev = -6;
                    break;
                case 0x0E:
                    ev = -5;
                    break;
                case 0x11:
                    ev = -4;
                    break;
                case 0x14:
                    ev = -3;
                    break;
                case 0x18:
                    ev = -2;
                    break;
                case 0x1B:
                    ev = -1;
                    break;
                case 0x1E:
                    ev = 0;
                    break;
                case 0x21:
                    ev = 1;
                    break;
                case 0x24:
                    ev = 2;
                    break;
                case 0x28:
                    ev = 3;
                    break;
                case 0x2B:
                    ev = 4;
                    break;
                case 0x2E:
                    ev = 5;
                    break;
                case 0x32:
                    ev = 6;
                    break;
                default:
                    ev = 0;
                    break;
            }
            if (copy_to_user((const void __user *)arg, &ev, sizeof(short)))
                return -EFAULT;

            return 0;
        }
        case SENSOR_CUSTOM_IOCTL_GET_ET:
        {
            printk("SENSOR_CUSTOM_IOCTL_GET_ET \n");
            custom_et_value_package ET;
            u32 exposure;
            u32 vts;

            if(capture_mode){
                pr_info("Select Image number of frame\n");
                _T("Stop continuous ouput frame from capture mode\n");
                if(!(s_flag & 0x2))
                    fjm6mo_write_register(info->i2c_client, 1, 0x0C, 0x06, 0x01); //Cap sel frame = 1
                err = isp_interrupt(INT_STATUS_CAPTURE);
                if(err)
                    pr_err("Select image frame fail\n");
                else
                    capture_mode = false;
            }
            CHECK_WRITE(err);

            fjm6mo_read_register(info->i2c_client, 0x07, 0x00, 0x04, &exposure);
            fjm6mo_read_register(info->i2c_client, 0x07, 0x04, 0x04, &vts);
            ET.exposure = exposure;
            ET.vts = vts;

            printk("GET_ET: exposure = %d \n", exposure);
            printk("GET_ET: vts = %d \n", vts);

            if (copy_to_user((const void __user *)arg, &ET, sizeof(ET)))
                return -EFAULT;
            return 0;
        }
        case SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF:
        {
            custom_touch_af_cmd_package touch_af;
            u32 af_w, af_h, af_x, af_y;
            if (copy_from_user(&touch_af,(const void __user *)arg, sizeof(custom_touch_af_cmd_package))){
                return -EFAULT;
            }
            if(touch_af.zoom){
                touch_mode = TOUCH_STATUS_ON;
                af_w = touch_af.win_w;
                af_h = touch_af.win_h;
                //printk("SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF: af_w:0x%x af_h:0x%x af_x:0x%x af_y:0x%x\n", touch_af.win_w, touch_af.win_h, touch_af.win_x, touch_af.win_y);
                af_x = touch_af.win_x;
                af_y = touch_af.win_y;
                printk("SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF: af_w:0x%x af_h:0x%x af_x:0x%x af_y:0x%x\n", af_w, af_h, af_x, af_y);
                err = fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x40, 0x04);
                err = fjm6mo_write_register(info->i2c_client, 2, 0x0A, 0x22, af_w);
                err = fjm6mo_write_register(info->i2c_client, 2, 0x0A, 0x24, af_h);
                err = fjm6mo_write_register(info->i2c_client, 2, 0x0A, 0x2A, af_x);
                err = fjm6mo_write_register(info->i2c_client, 2, 0x0A, 0x2C, af_y);
            }
            else{
                if(touch_mode != TOUCH_STATUS_OFF){
                    touch_mode = TOUCH_STATUS_OFF;
                    printk("SENSOR_CUSTOM_IOCTL_SET_TOUCH_AF: Cancel touch af\n");
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x40, 0x01);
                    if(!caf_mode){
                        fjm6mo_read_register(info->i2c_client, 0x0A, 0x03, 0x01, &buffer);
                        if(buffer == 0){
                            //Stop auto focus
                            fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x0);
                            err = isp_interrupt(INT_STATUS_AF);
                            if(err)
                                pr_err("Touch af stop interrupt error");
                        }
                        if(tegra3_get_project_id() == TEGRA3_PROJECT_TF201)
                            fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x3);
                    }
                }
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_FLASH_STATUS:
        {
            u8 flash_led_type;
            if (copy_from_user(&flash_led_type,(const void __user *)arg,
                    sizeof(flash_led_type))) {
                return -EFAULT;
            }
            printk("SET_FLASH_LED as %d\n", flash_led_type);
            switch(flash_led_type)
            {
                u16 val;
                case YUV_FlashControlOn:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x1F, 0x03);
                    break;
                case YUV_FlashControlOff:
                    fjm6mo_read_register(info->i2c_client, 0x02, 0x39, 0x01, &buffer);
                    if(buffer != 3)
                        err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x39, 0x03);
                    else
                        err = fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x1F, 0x00);
                    break;
                case YUV_FlashControlAuto:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x1F, 0x02);
                    break;
                case YUV_FlashControlTorch:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x39, 0x00);
                    break;
                default:
                    break;
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_GET_FLASH_STATUS:
        {
            short flash_status;
            if (copy_from_user(&flash_status,(const void __user *)arg,
                    sizeof(flash_status))) {
                return -EFAULT;
            }
            fjm6mo_read_register(info->i2c_client, 0x07, 0x2A, 0x02, &buffer);
            if (buffer == 0x20)
                flash_status = 0;
            else
                flash_status = 2;
            printk("GET_FLASH_STATUS as value:%d\n", flash_status);
            if (copy_to_user((const void __user *)arg, &flash_status, sizeof(short)))
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
            switch(scene_mode)
            {
                u16 val;
                case YUV_SceneMode_Invalid:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0x0);
                    break;
                case YUV_SceneMode_Auto:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0x1);
                    break;
                case YUV_SceneMode_Portrait:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0x2);
                    break;
                case YUV_SceneMode_Landscape:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0x3);
                    break;
                case YUV_SceneMode_Sports:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0x4);
                    break;
                case YUV_SceneMode_Night:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0x5);
                    break;
                case YUV_SceneMode_Sunset:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0x6);
                    break;
                case YUV_SceneMode_Snow:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0xA);
                    break;
                case YUV_SceneMode_Party:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0xE);
                    break;
                case YUV_SceneMode_BackLight:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x37, 0x8);
                    break;
                default:
                    break;
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_GET_ISO:
        {
            short iso;
            fjm6mo_read_register(info->i2c_client, 0x07, 0x28, 0x02, &buffer);
            if(buffer < 75)
                iso = 50;
            else if(buffer < 140)
                iso = 100;
            else if(buffer < 280)
                iso = 200;
            else if(buffer < 560)
                iso = 400;
            else
                iso = 800;
            printk("GET_ISO as value:%d\n", iso);
            if (copy_to_user((const void __user *)arg, &iso, sizeof(short)))
                return -EFAULT;

            return 0;
        }
        case SENSOR_CUSTOM_IOCTL_SET_ISO:
        {
            u8 iso;
            if (copy_from_user(&iso,(const void __user *)arg,
                    sizeof(iso))) {
                return -EFAULT;
            }
            printk("SET_ISO as %d\n", iso);
            switch(iso)
            {
                u16 val;
                case YUV_ISO_AUTO:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x05, 0x0);
                    break;
                case YUV_ISO_50:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x05, 0x1);
                    break;
                case YUV_ISO_100:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x05, 0x2);
                    break;
                case YUV_ISO_200:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x05, 0x3);
                    break;
                case YUV_ISO_400:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x05, 0x4);
                    break;
                case YUV_ISO_800:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x05, 0x5);
                    break;
                case YUV_ISO_1600:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x05, 0x6);
                    break;
                default:
                    break;
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_FRAME_RATE:
        {
            u8 frame_rate;
            if (copy_from_user(&frame_rate,(const void __user *)arg,
                    sizeof(frame_rate))) {
                return -EFAULT;
            }
            printk("SET_FRAME_RATE as %d\n", frame_rate);
            switch(frame_rate)
            {
                u16 val;
                case YUV_FRAME_RATE_AUTO:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0A, 0x00);
                    break;
                case YUV_FRAME_RATE_30:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0A, 0x01);
                    break;
                case YUV_FRAME_RATE_15:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0A, 0x14);
                    break;
                case YUV_FRAME_RATE_7:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0A, 0x15);
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
            switch(flickering)
            {
                u16 val;
                case YUV_ANTIBANGING_OFF:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x4);
                    break;
                case YUV_ANTIBANGING_AUTO:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x0);
                    break;
                case YUV_ANTIBANGING_50HZ:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x1);
                    break;
                case YUV_ANTIBANGING_60HZ:
                    err = fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x06, 0x2);
                    break;
                default:
                    break;
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_SHADING:
        {
            u8 shading;
            if (copy_from_user(&shading,(const void __user *)arg,
                    sizeof(shading))) {
                return -EFAULT;
            }
            fjm6mo_read_register(info->i2c_client, 0x01, 0x07, 0x01, &buffer);
            if(shading != buffer && buffer != 0x2){
                printk("SET_SHADING as %d\n", shading);
                if(sensor_get_status() != E_M6MO_Status_Parameter)
                    sensor_change_status(E_M6MO_Status_Parameter);
                if(shading)
                    fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x07, 0x1);
                else
                    fjm6mo_write_register(info->i2c_client, 1, 0x01, 0x07, 0x0);
                sensor_change_status(E_M6MO_Status_Monitor);
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_CONTINUOUS_AF:
        {
            u8 continuous_af;
            if (copy_from_user(&continuous_af,(const void __user *)arg,
                    sizeof(continuous_af))) {
                return -EFAULT;
            }
            printk("SET_CONTINUOUS_AF as %d\n", continuous_af);
            fjm6mo_read_register(info->i2c_client, 0x0A, 0x41, 0x01, &buffer);
            if(continuous_af){
                caf_mode = true;
                if(buffer != 0x4){
                    if(touch_mode != TOUCH_STATUS_OFF){
                        fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x41, 0x6);
                        fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x1);
                    }
                    else{
                        fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x41, 0x4);
                        fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x1);
                    }
                }
            }
            else{
                caf_mode = false;
                if(buffer == 0x4){
                    fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x10, 0xff);
                    fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x0);
                    err = isp_interrupt(INT_STATUS_AF);
                    if(err)
                        pr_err("CAF stop error: no interrupt\n");
                    if(tegra3_get_project_id() == TEGRA3_PROJECT_TF201)
                        fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x02, 0x3);
                    msleep(30);
                    fjm6mo_write_register(info->i2c_client, 1, 0x0A, 0x41, 0x3);
                }
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_AE_WINDOW:
        {
            u16 zoom_value;
            if (copy_from_user(&zoom_value,(const void __user *)arg,
                    sizeof(zoom_value))) {
                return -EFAULT;
            }
            printk("SET_AE_WINDOW as 0x%X\n", zoom_value);
            fjm6mo_write_register(info->i2c_client, 2, 0x03, 0x34, zoom_value);
            fjm6mo_write_register(info->i2c_client, 2, 0x03, 0x36, zoom_value);
            fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x38, 0x01);
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_WDR:
        {
            u16 wdr;
            if (copy_from_user(&wdr,(const void __user *)arg,
                    sizeof(wdr))) {
                return -EFAULT;
            }
            wdr = wdr & 0x00ff;
            printk("SET_WDR as 0x%X\n", wdr);
            if(wdr){
                fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x38, 0x01);
                fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x39, 0x00);
            }
            else{
                fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x38, 0x00);
                fjm6mo_write_register(info->i2c_client, 1, 0x0B, 0x39, 0x00);
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_AE_LOCK:
        {
            u32 ae_lock;
            if (copy_from_user(&ae_lock,(const void __user *)arg,
                    sizeof(ae_lock))) {
                return -EFAULT;
            }
            ae_mode = ae_lock;
            printk("SET_AE_LOCK as 0x%x\n", ae_lock);
            fjm6mo_write_register(info->i2c_client, 1, 0x03, 0x0, ae_lock);
            break;
        }
        case SENSOR_CUSTOM_IOCTL_GET_AE_LOCK:
        {
            u32 ae_lock;
            fjm6mo_read_register(info->i2c_client, 0x03, 0x0, 0x01, &ae_lock);
            printk("GET_AE_LOCK as 0x%x\n", ae_lock);
            if (copy_to_user((const void __user *)arg, &ae_lock, sizeof(int)))
                return -EFAULT;
            break;
        }
        case SENSOR_CUSTOM_IOCTL_SET_AWB_LOCK:
        {
            u32 awb_lock;
            if (copy_from_user(&awb_lock,(const void __user *)arg,
                    sizeof(awb_lock))) {
                return -EFAULT;
            }
            awb_mode = awb_lock;
            printk("SET_AWB_LOCK as 0x%x\n", awb_lock);
            fjm6mo_write_register(info->i2c_client, 1, 0x06, 0x0, awb_lock);
            break;
        }
        case SENSOR_CUSTOM_IOCTL_GET_AWB_LOCK:
        {
            u32 awb_lock;
            fjm6mo_read_register(info->i2c_client, 0x06, 0x0, 0x01, &awb_lock);
            printk("GET_AWB_LOCK as 0x%x\n", awb_lock);
            if (copy_to_user((const void __user *)arg, &awb_lock, sizeof(int)))
                return -EFAULT;
            break;
        }
        case SENSOR_CUSTOM_IOCTL_FW_UPDATE_INIT:
        {
            custom_fw_update_rom_package rom_cmd;
            if (copy_from_user(&rom_cmd,(const void __user *)arg,
                    sizeof(rom_cmd))) {
                return -EFAULT;
            }
            printk("FW_UPDATE_INIT\n");
            update_mode = true;
            isp_power_off();
            msleep(10);
            isp_power_on();
            msleep(10);
            err = fw_init();
            if(err){
                update_mode = false;
                return -ENOMEM;
            }
            err = fw_erase(rom_cmd);
            if(err){
                fw_update_fail();
                return -ENOMEM;
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_FW_UPDATE_PROGRAM:
        {
            custom_fw_update_rom_package rom_cmd;
            if (copy_from_user(&rom_cmd,(const void __user *)arg,
                    sizeof(rom_cmd))) {
                fw_update_fail();
                return -EFAULT;
            }
            if(rom_cmd.cmd)
                printk("FW_UPDATE_PROGRAM:0x%x\n", rom_cmd.flash_rom_start_address);
            err = fw_program(rom_cmd);
            if(err){
                fw_update_fail();
                return -ENOMEM;
            }
            if((!rom_cmd.cmd) && (rom_cmd.flash_rom_start_address == 0x10160000)){
                err = fw_version();
                update_mode = false;
                if(err){
                    pr_err("isp query version error\n");
                    return -ENOMEM;
                }
            }
            break;
        }
        case SENSOR_CUSTOM_IOCTL_FW_UPDATE_DATA:
        {
            custom_fw_update_data_package fw_update_cmd;
            if (copy_from_user(&fw_update_cmd,(const void __user *)arg,
                    sizeof(fw_update_cmd))) {
                fw_update_fail();
                return -EFAULT;
            }
            err = fjm6mo_write_memory(info->i2c_client, fw_update_cmd.data, fw_update_cmd.data_byte, fw_update_cmd.start_address, fw_update_cmd.data_length);
            if(err){
                fw_update_fail();
                return -ENOMEM;
            }
            break;
        }
        default:
            break;
    }
    return 0;
}

static int sensor_open(struct inode *inode, struct file *file)
{
    u32 result, temp;
    int retry = 0, err;
    file->private_data = info;
    pr_info("yuv %s\n",__func__);
    if(update_mode)
        return -EBUSY;
    if (info->pdata && info->pdata->power_on)
        info->pdata->power_on();
    yuv_sensor_power_on_reset_pin();
    return 0;
}

int fjm6mo_sensor_release(struct inode *inode, struct file *file)
{
    if (info->pdata && info->pdata->power_off)
    {
        pr_info("Sensor power off\n");
        if(update_mode)
            fw_update_fail();
        shading_table = 0;
        initial_mode = false;
        capture_mode = false;
        caf_mode = false;
        touch_mode = TOUCH_STATUS_OFF;
        ae_mode = 0;
        awb_mode = 0;
        yuv_sensor_power_off_reset_pin();
        info->pdata->power_off();
    }
    file->private_data = NULL;
    return 0;
}

static const struct file_operations sensor_fileops = {
    .owner = THIS_MODULE,
    .open = sensor_open,
    .unlocked_ioctl = sensor_ioctl,
    .release = fjm6mo_sensor_release,
};

static struct miscdevice sensor_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = FJM6MO_SENSOR_NAME,
    .fops = &sensor_fileops,
};

static ssize_t fjm6mo_switch_name(struct switch_dev *sdev, char *buf)
{
    if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
        return sprintf(buf, "TF700T-%06X\n", version_num);
    else
        return sprintf(buf, "TF201-%06X\n", version_num);
}

static ssize_t fjm6mo_switch_state(struct switch_dev *sdev, char *buf)
{
    return sprintf(buf, "%s\n", "0");
}

static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err=0, retry = 0;
    u32 result = 0, temp;
    u8 val[2] = {0xf6, 0xf6};

    pr_info("yuv %s, compiled at %s %s\n",__func__,__DATE__,__TIME__);

    info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
    if (!info) {
        pr_err("yuv_sensor : Unable to allocate memory!\n");
        return -ENOMEM;
    }

    err = misc_register(&sensor_device);
    if (err) {
        pr_err("yuv_sensor : Unable to register misc device!\n");
        kfree(info);
        return err;
    }
    info->pdata = client->dev.platform_data;
    info->i2c_client = client;
    i2c_set_clientdata(client, info);

    err = isp_power_on();
    if(err)
        return -ENOMEM;

    msleep(10);
    err = fjm6mo_read_memory(info->i2c_client, 0x1016FFFC, 2, val);
    if(!err){
        version_num = val[0] << 16;
        version_num += val[1] << 8;
        err = fjm6mo_read_memory(info->i2c_client, 0x1016FFE4, 1, &result);
        if(err)
            return -ENOMEM;
        version_num += result;
        fjm6mo_status = 1;
    }

#if 0
    err = isp_cam_start();
    if(err)
        return -ENOMEM;
    //Go to the monitor mode
    fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x10, 0xff);
    fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x0B, 0x2);
    //Wait interrupt
    err = isp_interrupt(INT_STATUS_MODE);
    if(err)
        pr_info("Enter monitor mode fail\n");
    //read sensor data
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x34, 0x30);
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x35, 0x0A);
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x33, 0x01);
    result = fjm6mo_read_register(info->i2c_client, 0x0D, 0x36, 0x01, &temp);
    if(temp == 0x88)
        fjm6mo_status = 1;
    else
        fjm6mo_status = 0;
#endif

    fjm6mo_sdev.name = FJM6MO_SDEV_NAME;
    fjm6mo_sdev.print_name = fjm6mo_switch_name;
    fjm6mo_sdev.print_state = fjm6mo_switch_state;
    if(switch_dev_register(&fjm6mo_sdev) < 0){
        pr_err("switch_dev_register for camera failed!\n");
    }
    switch_set_state(&fjm6mo_sdev, 0);

    pr_info("fjm6mo check version number 0x%x %d\n", version_num, fjm6mo_status);

    err = isp_power_off();
    if(err)
        return -ENOMEM;

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
    { FJM6MO_SENSOR_NAME, 0 },
};

MODULE_DEVICE_TABLE(i2c, sensor_id);

static struct i2c_driver sensor_i2c_driver = {
    .driver = {
        .name = FJM6MO_SENSOR_NAME,
        .owner = THIS_MODULE,
    },
    .probe = sensor_probe,
    .remove = sensor_remove,
    .id_table = sensor_id,
};

static int __init sensor_init(void)
{
    if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T || (tegra3_get_project_id() == TEGRA3_PROJECT_TF201)){
        printk(KERN_INFO "yuv fjm6mo %s+ #####\n", __func__);
        int ret;
        ret = i2c_add_driver(&sensor_i2c_driver);
        printk(KERN_INFO "yuv fjm6mo %s- #####\n", __func__);
        return ret;
    }
    return 0;
}

static void __exit sensor_exit(void)
{
    if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T || (tegra3_get_project_id() == TEGRA3_PROJECT_TF201)){
        pr_info("yuv fjm6mo %s\n",__func__);
        i2c_del_driver(&sensor_i2c_driver);
        switch_dev_unregister(&fjm6mo_sdev);
    }
}

module_init(sensor_init);
module_exit(sensor_exit);


#define CONFIG_I2C_READ_WRITE
#ifdef CONFIG_I2C_READ_WRITE
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <mach/clk.h>

#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];

int yuv_sensor_power_on_reset_pin();
int yuv_sensor_power_off_reset_pin();

static ssize_t i2c_set_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}
static int i2c_camera(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char *bp = debugTxtBuf;
    int len, retry = 0, len_read;
    int arg[2];
    int ret=-EINVAL, err;
    u32 result = 0, temp;
    u8 val[2] = {0xf6, 0xf6};
    u16 version_num = 0;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x", &arg[0]);
    pr_info("0 is open_camera 1 is close_camera 2 is firmware update\n");
    pr_info("command is arg1=%x \n", arg[0]);

    *ppos = len;

    switch(arg[0])
    {
        case 0:
        {
            pr_info("fjm6mo power_on\n");
            err = isp_power_on();
            if(err)
                return -ENOMEM;
            break;
        }
        case 1:
        {
            pr_info("fjm6mo power_off\n");
            err = isp_power_off();
            if(err)
                return -ENOMEM;
            break;
        }
        case 2:
        {
            fjm6mo_update_status = 0;
            pr_info("fjm6mo power_on\n");
            err = isp_power_on();
            if(err)
                return -ENOMEM;
            pr_info("fjm6mo firmware update start\n");
            fw_update();
            pr_info("fjm6mo power_off\n");
            err = isp_power_off();
            if(err)
                return -ENOMEM;
            fjm6mo_update_status = 1;
            break;
        }
        case 3:
        {
            pr_info("fjm6mo rear start and enter monitor mode\n");
            cur_camera_id = 0;
            err = isp_cam_start();
            if(err)
                return -ENOMEM;
            result = sensor_change_status(E_M6MO_Status_Monitor);
            break;
        }
        case 4:
        {
            pr_info("m6mo checksum\n");
            fw_checksum(&temp);
            break;
        }
        case 5:
        {
            pr_info("m6mo compare\n");
            fw_compare();
            break;
        }
        case 6:
        {
            pr_info("m6mo binary checksum\n");
            bin_checksum(&temp);
            break;
        }
        case 7:
        {
            result = fjm6mo_read_memory(info->i2c_client, 0x1016FFFC, 2, val);
            version_num = val[0] << 8;
            version_num += val[1];
            fjm6mo_read_memory(info->i2c_client, 0x1016FFE4, 1, &result);
            pr_info("fjm6mo check version number 0x%x-0x%x\n", version_num, result);
            break;
        }
        case 8:
        {
            s_flag += FJM6MO_FLAG_STOP_CAPTURE;
            pr_info("fjm6mo ouput test pattern s_flag=0x%x\n", s_flag);
            break;
        }
        case 9:
        {
            s_flag = s_flag & ~FJM6MO_FLAG_STOP_CAPTURE;
            pr_info("fjm6mo ouput test pattern s_flag=0x%x\n", s_flag);
            break;
        }
        case 10:
        {
            pr_info("fjm6mo power_on\n");
            err = isp_power_on();
            if(err)
                return -ENOMEM;
            pr_info("fjm6mo start\n");
            cur_camera_id = 0;
            err = isp_cam_start();
            if(err)
                return -ENOMEM;
            pr_info("Led on\n");
            result = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x39, 0x00);
            break;
        }
        case 11:
        {
            pr_info("Led off\n");
            result = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x39, 0x03);
            pr_info("fjm6mo power_off\n");
            err = isp_power_off();
            if(err)
                return -ENOMEM;
            break;
        }
        case 12:
        {
            pr_info("fjm6mo power_on\n");
            err = isp_power_on();
            if(err)
                return -ENOMEM;
            pr_info("fjm6mo start\n");
            cur_camera_id = 0;
            err = isp_cam_start();
            if(err)
                return -ENOMEM;
            pr_info("Flash on\n");
            result = fjm6mo_write_register(info->i2c_client, 1, 0x02, 0x39, 0x02);
            msleep(1000);
            pr_info("fjm6mo power_off\n");
            err = isp_power_off();
            if(err)
                return -ENOMEM;
            break;
        }
        case 13:
        {
            pr_info("fjm6mo front start and enter monitor mode\n");
            cur_camera_id = 1;
            err = isp_cam_start();
            if(err)
                return -ENOMEM;
            result = sensor_change_status(E_M6MO_Status_Monitor);
            break;
        }
        case 14:
        {
            pr_info("m6mo crc checksum\n");
            fw_crc_checksum(&temp);
            break;
        }
        default:
            break;
    }

    return len;    /* the end */
}

static int i2c_calibration_shading(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char *bp = debugTxtBuf;
    int len, retry = 0, wait_time;
    char light[10], cmd[10];
    int ret=-EINVAL, err;
    u32 result = 0, temp;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%s %s", light, cmd);

    pr_info("command is light=%s cmd=%s\n", light, cmd);

    *ppos = len;

    if(strncmp(light, "2800", 4) == 0){
        if(strncmp(cmd, "start", 5) == 0)
            err = isp_calibration_init(1);
        else if(strncmp(cmd, "capture", 7) == 0)
            err = isp_calibration_capture(1);
        else if(strncmp(cmd, "aging", 7) == 0)
            err = isp_calibration_aging(1);
    }
    else if(strncmp(light, "3500", 4) == 0){
        if(strncmp(cmd, "start", 5) == 0)
            err = isp_calibration_init(2);
        else if(strncmp(cmd, "capture", 7) == 0)
            err = isp_calibration_capture(2);
        else if(strncmp(cmd, "aging", 7) == 0)
            err = isp_calibration_aging(2);
    }
    else if(strncmp(light, "5000", 4) == 0){
        if(strncmp(cmd, "start", 5) == 0)
            err = isp_calibration_init(3);
        else if(strncmp(cmd, "capture", 7) == 0)
            err = isp_calibration_capture(3);
        else if(strncmp(cmd, "aging", 7) == 0)
            err = isp_calibration_aging(3);
    }
    else if(strncmp(light, "golden", 6) == 0){
        if(strncmp(cmd, "start", 5) == 0)
            err = isp_calibration_golden_init();
        else if(strncmp(cmd, "capture", 7) == 0)
            err = isp_calibration_golden_result();
    }
    else if(strncmp(light, "pgain", 5) == 0){
        sscanf(debugTxtBuf, "%s %d", light, &wait_time);
        err = isp_calibration_pgain_result(wait_time);
    }
    else if(strncmp(light, "shading", 7) == 0)
    {
        if(strncmp(cmd, "2800", 4) == 0)
            shading_table = 1;
        else if(strncmp(cmd, "3500", 4) == 0)
            shading_table = 2;
        else if(strncmp(cmd, "5000", 4) == 0)
            shading_table = 3;
    }

    return len;    /* the end */
}

static int i2c_calibration_pgain(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char *bp = debugTxtBuf;
    int len, retry = 0;
    char light[10], cmd[10];
    int ret=-EINVAL, err;
    u32 val0 = 0xff;
    u32 val1 = 0xff;
    u32 val2 = 0xff;
    u32 val3 = 0xff;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%s %s 0x%X 0x%X 0x%X 0x%X", light, cmd, &val0, &val1, &val2, &val3);

    pr_info("command is light=%s cmd=%s\n", light, cmd);
    pr_info("golden value input is 0x%X 0x%X 0x%X 0x%X\n", val0, val1, val2, val3);

    *ppos = len;

    if(strncmp(light, "pgain", 4) == 0){
        if(strncmp(cmd, "start", 5) == 0)
            err = isp_calibration_pgain_init(val0, val1, val2, val3);
    }

    return len;    /* the end */
}

static int i2c_set_cur_camera_id(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char *bp = debugTxtBuf;
    int len, ret=-EINVAL;
    unsigned int camera_id = cur_camera_id;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%X", &camera_id);
    pr_info("cur_camera_id = 0x%X, command is set_cur_camera_id = 0x%X\n", cur_camera_id, camera_id);
    cur_camera_id = camera_id;

    *ppos = len;

    return len;    /* the end */
}

static int i2c_firmware_update_status(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len = 0;
    char *bp = debugTxtBuf;

    if (*ppos)
        return 0;    /* the end */
    if(fjm6mo_update_status == 1)
        len = snprintf(bp, DBG_TXT_BUF_SIZE, "Firmware update success\n");
    else if(fjm6mo_update_status == 2)
        len = snprintf(bp, DBG_TXT_BUF_SIZE, "Firmware update unfinish\n");
    else
        len = snprintf(bp, DBG_TXT_BUF_SIZE, "Firmware update fail\n");

    if (copy_to_user(buf, debugTxtBuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static int i2c_firmware_version(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len = 0, err;
    char *bp = debugTxtBuf;

    if (*ppos)
        return 0;    /* the end */

    err = isp_power_on();
    if(err)
        return -ENOMEM;

    msleep(10);
    err = fw_version();
    if(err)
        return -ENOMEM;

    err = isp_power_off();
    if(err)
        return -ENOMEM;

    len = snprintf(bp, DBG_TXT_BUF_SIZE, "Firmware version is 0x%X\n", version_num);

    if (copy_to_user(buf, debugTxtBuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static int i2c_camera_ack(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len = 0, retry = 0, err;
    char *bp = debugTxtBuf;
    u32 temp, result;

    if (*ppos)
        return 0;    /* the end */

    err = isp_power_on();
    if(err)
        return -ENOMEM;
    msleep(10);
    cur_camera_id = 0;
    err = isp_cam_start();
    if(err)
        return -ENOMEM;
    //Go to the monitor mode
    fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x10, 0xff);
    fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x0B, 0x2);
    //Wait interrupt
    err = isp_interrupt(INT_STATUS_MODE);
    if(err)
        pr_info("Enter monitor mode fail\n");
    //read sensor data
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x34, 0x30);
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x35, 0x0A);
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x33, 0x01);
    result = fjm6mo_read_register(info->i2c_client, 0x0D, 0x36, 0x01, &temp);
    if(temp == 0x88)
        fjm6mo_status = 1;
    else
        fjm6mo_status = 0;
    err = isp_power_off();
    if(err)
        return -ENOMEM;

    len = snprintf(bp, DBG_TXT_BUF_SIZE, "%d\n", fjm6mo_status);

    if (copy_to_user(buf, debugTxtBuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static int i2c_vga_camera_ack(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len = 0, retry = 0, err;
    char *bp = debugTxtBuf;
    u32 temp, result;

    if (*ppos)
        return 0;    /* the end */

    err = isp_power_on();
    if(err)
        return -ENOMEM;
    msleep(10);
    cur_camera_id = 1;
    err = isp_cam_start();
    if(err)
        return -ENOMEM;
    //Go to the monitor mode
    fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x10, 0xff);
    fjm6mo_write_register(info->i2c_client, 1, 0x00, 0x0B, 0x2);
    //Wait interrupt
    err = isp_interrupt(INT_STATUS_MODE);
    if(err)
        pr_info("Enter monitor mode fail\n");
    //read sensor data
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x34, 0x30);
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x35, 0x0A);
    result = fjm6mo_write_register(info->i2c_client, 1, 0x0D, 0x33, 0x01);
    result = fjm6mo_read_register(info->i2c_client, 0x0D, 0x36, 0x01, &temp);
    if(temp == 0x27)
        fjm6mo_status = 1;
    else
        fjm6mo_status = 0;
    err = isp_power_off();
    if(err)
        return -ENOMEM;

    len = snprintf(bp, DBG_TXT_BUF_SIZE, "%d\n", fjm6mo_status);

    if (copy_to_user(buf, debugTxtBuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static int i2c_calibration_ack(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len = 0;
    char *bp = debugTxtBuf;

    if (*ppos)
        return 0;    /* the end */

    // 0:Fail 1:Pass
    // 10:power 11:init 12:monitor 13:light 14:capture
    // 15:checksum 16:pgain 17:golden

    switch(fjm6mo_calibration_status)
    {
        case CALIBRATION_ISP_POWERON_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 10\n");
            break;
        case CALIBRATION_ISP_INIT_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 11\n");
            break;
        case CALIBRATION_ISP_MONITOR_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 12\n");
            break;
        case CALIBRATION_LIGHT_SOURCE_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 13\n");
            break;
        case CALIBRATION_LIGHT_SOURCE_OK:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "1 0\n");
            break;
        case CALIBRATION_ISP_CAPTURE_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 14\n");
            break;
        case CALIBRATION_ISP_CHECKSUM_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 15\n");
            break;
        case CALIBRATION_ISP_PGAIN_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 16\n");
            break;
        case CALIBRATION_ISP_GOLDEN_FAIL:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 17\n");
            break;
        case CALIBRATION_ISP_OK:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "1 1\n");
            break;
        default:
            len = snprintf(bp, DBG_TXT_BUF_SIZE, "0 0\n");
            break;
    }

    if (copy_to_user(buf, debugTxtBuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static int i2c_calibration_golden_value(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len = 0, err;
    char *bp = debugTxtBuf;

    if (*ppos)
        return 0;    /* the end */

    len = snprintf(bp, DBG_TXT_BUF_SIZE, "0x%X 0x%X 0x%X 0x%X\n", golden_value[0], golden_value[1], golden_value[2], golden_value[3]);

    if (copy_to_user(buf, debugTxtBuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static int i2c_read_value(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len = 0;
    char *bp = debugTxtBuf;

    if (*ppos)
        return 0;    /* the end */

    len = snprintf(bp, DBG_TXT_BUF_SIZE, "Register version is 0x%X\n", register_value);

    if (copy_to_user(buf, debugTxtBuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}

static int i2c_camera_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len;
    u32 arg[3];
    u32 result = 0, temp;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x %x %x", &arg[0], &arg[1], &arg[2]);
    pr_info("command is cat=%x byte=%x number=%x \n", arg[0], arg[1], arg[2]);

    *ppos = len;

    result = fjm6mo_read_register(info->i2c_client, arg[0], arg[1], arg[2], &temp);
    register_value = temp;

    pr_info("register value: 0x%x\n", temp);

    return len;    /* the end */
}

static int i2c_camera_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len;
    u32 arg[4];

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x %x %x %x", &arg[0], &arg[1], &arg[2], &arg[3]);
    pr_info("command is number=%x cat=%x byte=%x value=%x\n", arg[0], arg[1], arg[2], arg[3]);

    *ppos = len;

    fjm6mo_write_register(info->i2c_client, arg[0], arg[1], arg[2], arg[3]);

    return len;    /* the end */
}

static int i2c_camera_read_memory(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len;
    u32 arg[4];
    int ret=-EINVAL;
    u32 result = 0, temp = 0;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x %x", &arg[0], &arg[1]);
    pr_info("command is address=%x number=%x \n", arg[0], arg[1]);

    *ppos = len;

    result = fjm6mo_read_memory(info->i2c_client, arg[0], arg[1], &temp);

    pr_info("memory read value at 0x%x: 0x%x\n", arg[0], temp);

    return len;    /* the end */
}

static int i2c_camera_write_memory(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len;
    u32 arg[5];
    u32 result = 0, temp;

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%x %x %x %x", &arg[0], &arg[1], &arg[2], &arg[3]);
    pr_info("command is value=%x byte_size=%x address=%x write_size:%x\n", arg[0], arg[1], arg[2], arg[3]);

    *ppos = len;

    result= fjm6mo_write_memory(info->i2c_client, arg, arg[1], arg[2], arg[3]);
    pr_info("memory write value at 0x%x: 0x%x\n", arg[2], arg[0]);

    return len;    /* the end */
}

static const struct file_operations i2c_open_camera = {
    .open = i2c_set_open,
    .write = i2c_camera,
};
static const struct file_operations isp_calibration_shading = {
    .open = i2c_set_open,
    .write = i2c_calibration_shading,
};
static const struct file_operations isp_calibration_pgain = {
    .open = i2c_set_open,
    .write = i2c_calibration_pgain,
};
static const struct file_operations isp_firmware_update_status = {
    .open = i2c_set_open,
    .read = i2c_firmware_update_status,
};
static const struct file_operations isp_firmware_version = {
    .open = i2c_set_open,
    .read = i2c_firmware_version,
};
static const struct file_operations isp_set_cur_camera_id = {
    .open = i2c_set_open,
    .write = i2c_set_cur_camera_id,
};
static const struct file_operations camera_status = {
    .open = i2c_set_open,
    .read = i2c_camera_ack,
};
static const struct file_operations vga_status = {
    .open = i2c_set_open,
    .read = i2c_vga_camera_ack,
};
static const struct file_operations calibration_status = {
    .open = i2c_set_open,
    .read = i2c_calibration_ack,
};
static const struct file_operations calibration_golden_value = {
    .open = i2c_set_open,
    .read = i2c_calibration_golden_value,
};
static const struct file_operations read_register_value = {
    .open = i2c_set_open,
    .read = i2c_read_value,
};
static const struct file_operations i2c_read_register = {
    .open = i2c_set_open,
    .write = i2c_camera_read,
};
static const struct file_operations i2c_write_register = {
    .open = i2c_set_open,
    .write = i2c_camera_write,
};
static const struct file_operations i2c_read_memory = {
    .open = i2c_set_open,
    .write = i2c_camera_read_memory,
};
static const struct file_operations i2c_write_memory = {
    .open = i2c_set_open,
    .write = i2c_camera_write_memory,
};

static int __init tegra_i2c_debuginit(void)
{
    struct dentry *dent = debugfs_create_dir("fjm6mo", NULL);
    if(factory_mode == 2){
        (void) debugfs_create_file("i2c_open_camera", 0777,
                dent, NULL, &i2c_open_camera);
        (void) debugfs_create_file("isp_calibration_shading", 0777,
            dent, NULL, &isp_calibration_shading);
        (void) debugfs_create_file("isp_calibration_pgain", 0777,
            dent, NULL, &isp_calibration_pgain);
        (void) debugfs_create_file("calibration_status", 0777,
            dent, NULL, &calibration_status);
        (void) debugfs_create_file("calibration_golden_value", 0777,
            dent, NULL, &calibration_golden_value);
        (void) debugfs_create_file("isp_firmware_update_status", 0777,
                dent, NULL, &isp_firmware_update_status);
        (void) debugfs_create_file("isp_firmware_version", 0777,
                dent, NULL, &isp_firmware_version);
        (void) debugfs_create_file("isp_set_cur_camera_id", 0777,
                dent, NULL, &isp_set_cur_camera_id);
        (void) debugfs_create_file("camera_status", 0777,
                dent, NULL, &camera_status);
        (void) debugfs_create_file("vga_status", 0777,
                dent, NULL, &vga_status);
        (void) debugfs_create_file("read_register_value", 0777,
                dent, NULL, &read_register_value);
        (void) debugfs_create_file("i2c_read_register", 0777,
                dent, NULL, &i2c_read_register);
        (void) debugfs_create_file("i2c_write_register", 0777,
                dent, NULL, &i2c_write_register);
        (void) debugfs_create_file("i2c_read_memory", 0777,
                dent, NULL, &i2c_read_memory);
        (void) debugfs_create_file("i2c_write_memory", 0777,
                dent, NULL, &i2c_write_memory);
    }
    else{
        (void) debugfs_create_file("i2c_open_camera", 0700,
                dent, NULL, &i2c_open_camera);
        (void) debugfs_create_file("isp_calibration_shading", 0700,
            dent, NULL, &isp_calibration_shading);
        (void) debugfs_create_file("isp_calibration_pgain", 0700,
            dent, NULL, &isp_calibration_pgain);
        (void) debugfs_create_file("calibration_status", 0700,
            dent, NULL, &calibration_status);
        (void) debugfs_create_file("calibration_golden_value", 0700,
            dent, NULL, &calibration_golden_value);
        (void) debugfs_create_file("isp_firmware_update_status", 0700,
                dent, NULL, &isp_firmware_update_status);
        (void) debugfs_create_file("isp_firmware_version", 0700,
                dent, NULL, &isp_firmware_version);
        (void) debugfs_create_file("isp_set_cur_camera_id", 0700,
                dent, NULL, &isp_set_cur_camera_id);
        (void) debugfs_create_file("camera_status", 0700,
                dent, NULL, &camera_status);
        (void) debugfs_create_file("vga_status", 0700,
                dent, NULL, &vga_status);
        (void) debugfs_create_file("read_register_value", 0700,
                dent, NULL, &read_register_value);
        (void) debugfs_create_file("i2c_read_register", 0700,
                dent, NULL, &i2c_read_register);
        (void) debugfs_create_file("i2c_write_register", 0700,
                dent, NULL, &i2c_write_register);
        (void) debugfs_create_file("i2c_read_memory", 0700,
                dent, NULL, &i2c_read_memory);
        (void) debugfs_create_file("i2c_write_memory", 0700,
                dent, NULL, &i2c_write_memory);
    }

    debugfs_create_x32("flag",0755, dent, &s_flag);
    return 0;
}

late_initcall(tegra_i2c_debuginit);
#endif

