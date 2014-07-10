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
#include <mach/board-cardhu-misc.h>
#include <linux/gpio.h>
#include "../gpio-names.h"

#define SCALAR_NAME        "scalar"
#define SCALAR_MAX_RETRIES        3  //max counter for retry I2C access
#define MAX_LOOPS_RETRIES         100
#define MAX_WRITE_SIZE        256
#define VERIFY_CHECK_SIZE        256  // unlimited in data sheet
#define FLASH_MEMORY_SIZE        0x040000  // 256Kbyte: 0x000000~0x03FFFF
#define LINE_SIZE      16
#define FW_READ_TO_FILE_NAME        "/data/FW_read_test.bin"
#define EN_VDD_BL TEGRA_GPIO_PH3

typedef struct
{
    unsigned short fw_version;
    unsigned short fw_subversion;
    unsigned short fw_version_checksum;
    unsigned short panel_type;
    char fw_version_string[20];
    char* binfile_path;
} fw_update_rom_package;

enum {
    ASUS_CUSTOM_IOCTL_FW_VERSION = 100,
    ASUS_CUSTOM_IOCTL_FW_UPDATE_PROGRAM,
    ASUS_CUSTOM_IOCTL_FW_UPDATE_ENTER,
    ASUS_CUSTOM_IOCTL_FW_UPDATE_EXIT,
    ASUS_CUSTOM_IOCTL_FW_TEST_BIN,
    ASUS_CUSTOM_IOCTL_FW_VERSION_TO_KERNEL,
};

#define SCALAR_CUSTOM_IOCTL_FW_VERSION _IOWR('o', ASUS_CUSTOM_IOCTL_FW_VERSION, fw_update_rom_package)
#define SCALAR_CUSTOM_IOCTL_FW_UPDATE_PROGRAM _IOWR('o', ASUS_CUSTOM_IOCTL_FW_UPDATE_PROGRAM, fw_update_rom_package)
#define SCALAR_CUSTOM_IOCTL_FW_UPDATE_ENTER _IOWR('o', ASUS_CUSTOM_IOCTL_FW_UPDATE_ENTER, fw_update_rom_package)
#define SCALAR_CUSTOM_IOCTL_FW_UPDATE_EXIT _IOWR('o', ASUS_CUSTOM_IOCTL_FW_UPDATE_EXIT, fw_update_rom_package)
#define SCALAR_CUSTOM_IOCTL_FW_TEST_BIN _IOWR('o', ASUS_CUSTOM_IOCTL_FW_TEST_BIN, fw_update_rom_package)
#define SCALAR_CUSTOM_IOCTL_FW_VERSION_TO_KERNEL _IOWR('o', ASUS_CUSTOM_IOCTL_FW_VERSION_TO_KERNEL, fw_update_rom_package)

static u16 version_num = 0xffff;
static u8 original_status = 0x00;
static u8 version_buf[20];
int scalar_update_status = 0;

struct scalar_info {
    int mode;
    struct i2c_client *i2c_client;
};
struct switch_dev   scalar_sdev;
static struct scalar_info *info;

//extern unsigned int factory_mode;
extern u8 scalar_fw_version;
extern u8 scalar_fw_subversion;
extern u8 p1801_panel_type;

static int scalar_isp_writebytes(struct i2c_client *client, u8* send_buf, u32 write_size)
{
    int err, retry = 0;
    struct i2c_msg msg;

    if (!client->adapter)
        return -ENODEV;

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = write_size;
    msg.buf = send_buf;

    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1)
                return 0;
        retry++;
        pr_err("%s : i2c transfer failed, retry time %d\n", __func__, retry);
    } while (retry <= SCALAR_MAX_RETRIES);

    return -1;
}
static int scalar_isp_readbytes(struct i2c_client *client, u8* val, u32 read_size)
{
    int err;
    struct i2c_msg msg;

    if (!client->adapter)
        return -ENODEV;

    msg.addr = client->addr;
    msg.flags = I2C_M_RD;
    msg.len = read_size;
    msg.buf = val;

    err = i2c_transfer(client->adapter, &msg, 1);
    if (err != 1)
        return -EINVAL;

    return 0;
}
static int scalar_debug_writebytes(struct i2c_client *client, u8* send_buf, u32 write_size)
{
    int err, retry = 0;
    struct i2c_msg msg;

    if (!client->adapter)
        return -ENODEV;

    msg.addr = 0x59;
    msg.flags = 0;
    msg.len = write_size;
    msg.buf = send_buf;

    do {
        err = i2c_transfer(client->adapter, &msg, 1);
        if (err == 1)
                return 0;
        retry++;
        pr_err("%s : i2c transfer failed, retry time %d\n", __func__, retry);
    } while (retry <= SCALAR_MAX_RETRIES);

    return -1;
}
static int scalar_debug_readbytes(struct i2c_client *client, u8* val, u32 read_size)
{
    int err;
    struct i2c_msg msg;

    if (!client->adapter)
        return -ENODEV;

    msg.addr = 0x59;
    msg.flags = I2C_M_RD;
    msg.len = read_size;
    msg.buf = val;

    err = i2c_transfer(client->adapter, &msg, 1);
    if (err != 1)
        return -EINVAL;

    return 0;
}

static int enter_isp_mode()
{
    printk("%s+\n", __func__);
    int err = 0;
    u8 data[5] = {0x4D, 0x53, 0x54, 0x41, 0x52};
    err = scalar_isp_writebytes(info->i2c_client, data, 5);
    scalar_update_status = 2;
    printk("%s-\n", __func__);
    return err;
}
static int exit_isp_mode()
{
    printk("%s+\n", __func__);
    int err = 0;
    u8 data[1] = {0x24};
    err = scalar_isp_writebytes(info->i2c_client, data, 1);
    if(err) {
        pr_err("exit_isp_mode error\n");
        scalar_update_status = -1;
        return err;
    }
    gpio_set_value(EN_VDD_BL, 0);
    msleep(2000);
    gpio_set_value(EN_VDD_BL, 1);
    scalar_update_status = 1;
    printk("%s-\n", __func__);
    return 0;
}
static int enter_serial_debug_mode()
{
    u8 data1[5] = {0x53, 0x45, 0x52, 0x44, 0x42};  //enter Serial Debug mode
    u8 data2[4] = {0x10, 0xC0, 0xC1, 0x53};  //enter Single Step mode
    scalar_debug_writebytes(info->i2c_client, data1, 5);
    scalar_debug_writebytes(info->i2c_client, data2, 4);
    return 0;
}
static int exit_serial_debug_mode()
{
    u8 data2[4] = {0x10, 0xC0, 0xC1, 0xFF};  //exit Single Step mode
    u8 data1[1] = {0x45};  //exit Serial Debug mode
    scalar_debug_writebytes(info->i2c_client, data2, 4);
    scalar_debug_writebytes(info->i2c_client, data1, 1);
    return 0;
}
static u8 read_status()
{
    u8 data1[2] = {0x10, 0x05};
    u8 data2[1] = {0x11};
    u8 data3 = 0xFF;
    u8 data4[1] = {0x12};
    scalar_isp_writebytes(info->i2c_client, data1, 2);
    scalar_isp_writebytes(info->i2c_client, data2, 1);
    scalar_isp_readbytes(info->i2c_client, &data3, 1);
    scalar_isp_writebytes(info->i2c_client, data4, 1);
    return data3;
}
static int write_status(u8 val)
{
    u8 data1[3] = {0x10, 0x01, val};
    u8 data2[1] = {0x12};
    scalar_isp_writebytes(info->i2c_client, data1, 3);
    scalar_isp_writebytes(info->i2c_client, data2, 1);
    return 0;
}
static int write_enable()
{
    u8 data1[2] = {0x10, 0x06};
    u8 data2[1] = {0x12};
    scalar_isp_writebytes(info->i2c_client, data1, 2);
    scalar_isp_writebytes(info->i2c_client, data2, 1);
    return 0;
}
static int write_disable()
{
    u8 data1[2] = {0x10, 0x04};
    u8 data2[1] = {0x12};
    scalar_isp_writebytes(info->i2c_client, data1, 2);
    scalar_isp_writebytes(info->i2c_client, data2, 1);
    return 0;
}
void write_update_progress_to_file(int start, int portion, int page_processed, int total_page_num)
{
    struct file *fp_progress = NULL;
    mm_segment_t old_fs;
    loff_t offset = 0;
    char str_progress[4];
    int percentage = 0;

    //blanking start from 0, portion 25; programming start from 25, portion 50; verify start from 75, portion 25
    percentage = start + portion * page_processed /total_page_num;

    printk("%s: page_processed:%d, percentage= %d;\n", __func__, page_processed, percentage);
    fp_progress = filp_open("/data/scalar_fw_update_progress", O_RDWR | O_CREAT, S_IRUGO | S_IWUGO);
    if ( IS_ERR_OR_NULL(fp_progress) ){
        filp_close(fp_progress, NULL);
        printk("%s: open %s fail\n", __FUNCTION__, "/data/scalar_fw_update_progress");
    }
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    offset = 0;
    if (fp_progress->f_op != NULL && fp_progress->f_op->write != NULL){
        sprintf(str_progress, "%d\n", percentage);
        fp_progress->f_op->write(fp_progress, str_progress, strlen(str_progress), &offset);
    }else
        pr_err("%s: f_op might be null\n", __func__);
    set_fs(old_fs);
    filp_close(fp_progress, NULL);
}
static int chip_erase()
{
    printk("%s+\n", __func__);

    u8 temp;
    u32 retry = 0;
    u8 data1[2] = {0x10, 0xC7};
    u8 data2[1] = {0x12};

    //read status and keep it
    original_status = read_status();
    printk("original status=%x\n", original_status);

    //disable write protect by writing 0x00 to status register
    write_enable();
    write_status(0x00);
    write_disable();

    //wait for flash ready
    retry = 0;
    temp = read_status();
    while (temp != 0x00 && retry < MAX_LOOPS_RETRIES) {
        msleep(10);
        temp = read_status();
        retry += 1;
    }
    if(temp)
        printk("wait status fail 1\n");

    //chip erase, write 0xFF to all the data in flash
    write_enable();
    scalar_isp_writebytes(info->i2c_client, data1, 2);
    scalar_isp_writebytes(info->i2c_client, data2, 1);

    //wait for flash ready
    retry = 0;
    temp = read_status();
    while (temp != 0x00 && retry < MAX_LOOPS_RETRIES) {
        msleep(10);
        temp = read_status();
        retry += 1;
    }
    if(temp)
        printk("wait status fail 2\n");

    printk("%s-\n", __func__);
    return 0;
}
static int blanking_check()
{
    printk("%s+\n", __func__);

    u32 addr;
    u32 i;
    u32 total_page = FLASH_MEMORY_SIZE >> 8;  //page size is 256 Byte
    u32 page_processed;

    for (addr=0x000000; addr<0x03FFFF; addr+=VERIFY_CHECK_SIZE) {
        u8 data1[5] = {0x10, 0x03, (u8)((addr & 0xFF0000) >> 16), (u8)((addr & 0x00FF00) >> 8), (u8)((addr & 0x0000FF))};
        u8 data2[1] = {0x11};
        u8 data3[VERIFY_CHECK_SIZE];
        u8 data4[1] = {0x12};

        //read data from flash
        scalar_isp_writebytes(info->i2c_client, data1, 5);
        scalar_isp_writebytes(info->i2c_client, data2, 1);
        scalar_isp_readbytes(info->i2c_client, data3, VERIFY_CHECK_SIZE);
        scalar_isp_writebytes(info->i2c_client, data4, 1);

        //compare to 0xFF
        for(i=0; i< VERIFY_CHECK_SIZE; i++) {
            if(data3[i]!=0xFF) {
                printk("blanking check fail at (addr  %x)+(i  %x)\n", addr, i);
                scalar_update_status = -1;
                return -1;
            }
        }

        //update progress
        page_processed = addr >> 8;
        if( (page_processed+1)%256 == 0)
            write_update_progress_to_file(0, 25, page_processed+1, total_page);
    }

    printk("%s-\n", __func__);
    return 0;
}
//page programming
static int programming(char * pFile)
{
    printk("%s+\n", __func__);
    u32 addr, i;
    u8 temp;
    u32 retry = 0;
    u32 checksum = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    u8 buf_write[MAX_WRITE_SIZE+5];
    u32 total_page = FLASH_MEMORY_SIZE >> 8;  //page size is 256 Byte
    u32 page_processed;

    fp = filp_open(pFile, O_RDONLY , 0);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            printk("file start to read!\n");
            for (addr=0x000000; addr<FLASH_MEMORY_SIZE; addr+=MAX_WRITE_SIZE) {
                u8 data2[1] = {0x12};

                buf_write[0] = 0x10;
                buf_write[1] = 0x02;
                buf_write[2] = (u8)((addr & 0xFF0000) >> 16);
                buf_write[3] = (u8)((addr & 0x00FF00) >> 8);
                buf_write[4] = (u8)(addr & 0x0000FF);

                //read data from bin file
                fp->f_op->read(fp, buf_write+5, MAX_WRITE_SIZE, &fp->f_pos);
                for(i=5; i< MAX_WRITE_SIZE+5; i++) {
                    checksum += buf_write[i];
                }

                //wait for flash ready
                retry = 0;
                temp = read_status();
                while (temp != 0x00 && retry < MAX_LOOPS_RETRIES)
                {
                    msleep(10);
                    temp = read_status();
                    retry += 1;
                }

                // write data to flash
                write_enable();
                scalar_isp_writebytes(info->i2c_client, buf_write, MAX_WRITE_SIZE+5);
                scalar_isp_writebytes(info->i2c_client, data2, 1);

                //update progress
                page_processed = addr >> 8;
                if( (page_processed+1)%128 == 0)
                    write_update_progress_to_file(25, 50, page_processed+1, total_page);
            }
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
    }
    else if(PTR_ERR(fp) == -ENOENT){
        pr_err("file not found error\n");
        scalar_update_status = -1;
        return -ENOMEM;
    }
    else{
        pr_err("file open error\n");
        scalar_update_status = -1;
        return -ENOMEM;
    }

    printk("%s: file checksum=%x, last 2 byte is %x\n", __func__, checksum, checksum & 0x0000FFFF);
    printk("%s-\n", __func__);

    return 0;
}
static int verify(char * pFile)
{
    printk("%s+\n", __func__);

    u32 addr, i;
    u32 checksum = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    u8 buf[VERIFY_CHECK_SIZE];
    u32 total_page = FLASH_MEMORY_SIZE >> 8;  //page size is 256 Byte
    u32 page_processed;

    fp = filp_open(pFile, O_RDONLY, 0);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            printk("file start to read!\n");
            for (addr=0x000000; addr<0x03FFFF; addr+=VERIFY_CHECK_SIZE) {
                u8 data1[5] = {0x10, 0x03, (u8)((addr & 0xFF0000) >> 16), (u8)((addr & 0x00FF00) >> 8), (u8)((addr & 0x0000FF))};
                u8 data2[1] = {0x11};
                u8 data3[VERIFY_CHECK_SIZE];
                u8 data4[1] = {0x12};

                //read data form bin file
                fp->f_op->read(fp, buf, VERIFY_CHECK_SIZE, &fp->f_pos);

                //read data from flash
                scalar_isp_writebytes(info->i2c_client, data1, 5);
                scalar_isp_writebytes(info->i2c_client, data2, 1);
                scalar_isp_readbytes(info->i2c_client, data3, VERIFY_CHECK_SIZE);
                scalar_isp_writebytes(info->i2c_client, data4, 1);

                //compare, compute flash data checksum (fw version)
                for(i=0; i< VERIFY_CHECK_SIZE; i++) {
                    checksum += data3[i];
                    if(data3[i]!=buf[i]) {
                        scalar_update_status = -1;
                        return -1;
                    }
                }

                //update progress
                page_processed = addr >> 8;
                if( (page_processed+1)%256 == 0)
                    write_update_progress_to_file(75, 25, page_processed+1, total_page);
            }
        }
        set_fs(old_fs);
        filp_close(fp, NULL);
    }
    else if(PTR_ERR(fp) == -ENOENT){
        pr_err("file not found error\n");
        scalar_update_status = -1;
        return -ENOMEM;
    }
    else{
        pr_err("file open error\n");
        scalar_update_status = -1;
        return -ENOMEM;
    }

    printk("%s: fw checksum=%x, last 2 byte is %x\n", __func__, checksum, checksum & 0x0000FFFF);
    printk("%s-\n", __func__);
    return 0;
}
static int test_read_FW()
{
    printk("%s+\n", __func__);
    u32 addr;
    u32 i;
    u32 checksum = 0;

    for (addr=0x000000; addr<FLASH_MEMORY_SIZE; addr+=LINE_SIZE) {
        u8 data1[5] = {0x10, 0x03, (u8)((addr & 0xFF0000) >> 16), (u8)((addr & 0x00FF00) >> 8), (u8)((addr & 0x0000FF))};
        u8 data2[1] = {0x11};
        u8 data3[LINE_SIZE];
        u8 data4[1] = {0x12};

        //read data from flash
        scalar_isp_writebytes(info->i2c_client, data1, 5);
        scalar_isp_writebytes(info->i2c_client, data2, 1);
        scalar_isp_readbytes(info->i2c_client, data3, LINE_SIZE);
        scalar_isp_writebytes(info->i2c_client, data4, 1);
        for(i=0; i< LINE_SIZE; i+=16) {
            //printk("%s: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x\n", __func__, data3[i], data3[i+1], data3[i+2], data3[i+3], data3[i+4], data3[i+5], data3[i+6], data3[i+7], data3[i+8], data3[i+9], data3[i+10], data3[i+11], data3[i+12], data3[i+13], data3[i+14], data3[i+15]);
        }
        for(i=0; i< LINE_SIZE; i++) {
            checksum += data3[i];
        }
    }
    printk("%s: fw checksum=%x, last 2 byte is %x\n", __func__, checksum, checksum & 0x0000FFFF);
    printk("%s-\n", __func__);
    return 0;
}
static int test_read_FW_to_file()
{
    printk("%s+\n", __func__);
    u32 addr;
    u32 i;
    u32 checksum = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    char *pFile = FW_READ_TO_FILE_NAME;

    fp = filp_open(pFile, O_RDWR, S_IRWXU|S_IRWXG|S_IRWXO);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);

        if(fp->f_op != NULL && fp->f_op->write != NULL){
            printk("file start to write!\n");
            for (addr=0x000000; addr<FLASH_MEMORY_SIZE; addr+=LINE_SIZE) {
                u8 data1[5] = {0x10, 0x03, (u8)((addr & 0xFF0000) >> 16), (u8)((addr & 0x00FF00) >> 8), (u8)((addr & 0x0000FF))};
                u8 data2[1] = {0x11};
                u8 data3[LINE_SIZE];
                u8 data4[1] = {0x12};

                //read data from flash
                scalar_isp_writebytes(info->i2c_client, data1, 5);
                scalar_isp_writebytes(info->i2c_client, data2, 1);
                scalar_isp_readbytes(info->i2c_client, data3, LINE_SIZE);
                scalar_isp_writebytes(info->i2c_client, data4, 1);

                //write date to file
                fp->f_op->write(fp, data3, LINE_SIZE, &fp->f_pos);

                //compute flash data checksum
                for(i=0; i< LINE_SIZE; i++) {
                    checksum += data3[i];
                }
            }
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

    printk("%s: fw checksum=%x, last 2 byte is %x\n", __func__, checksum, checksum & 0x0000FFFF);
    printk("%s-\n", __func__);

    return 0;
}
static int test_read_file(char * pFile)
{
    printk("%s+\n", __func__);
    u32 addr;
    u32 i;
    u32 checksum = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    u8 buf[LINE_SIZE];

    fp = filp_open(pFile, O_RDONLY, 0);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);

        if(fp->f_op != NULL && fp->f_op->read != NULL){
            printk("file start to read!\n");
            for (addr=0x000000; addr<FLASH_MEMORY_SIZE; addr+=LINE_SIZE) {
                fp->f_op->read(fp, buf, LINE_SIZE, &fp->f_pos);
                for(i=0; i< LINE_SIZE; i++) {
                    checksum += buf[i];
                }
            }
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

    printk("%s: checksum=%x, last 2 byte is %x\n", __func__, checksum, checksum & 0x0000FFFF);
    printk("%s-\n", __func__);

    return 0;
}
static int test_open_bin(char * pFile, u16* version)
{
    printk("%s+\n", __func__);
    u32 addr;
    u32 i;
    u32 checksum = 0;
    struct file *fp = NULL;
    mm_segment_t old_fs;
    u8 buf[LINE_SIZE];

    fp = filp_open(pFile, O_RDONLY, 0);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);

        if(fp->f_op != NULL && fp->f_op->read != NULL){
            printk("file start to read!\n");
            for (addr=0x000000; addr<FLASH_MEMORY_SIZE; addr+=LINE_SIZE) {
                fp->f_op->read(fp, buf, LINE_SIZE, &fp->f_pos);
                for(i=0; i< LINE_SIZE; i++) {
                    checksum += buf[i];
                }
            }
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

    *version = checksum & 0x0000FFFF;
    printk("%s: checksum=%x, last 2 byte is %x\n", __func__, checksum, *version);
    printk("%s-\n", __func__);

    return 0;
}
static int read_gpio(u8 addrH, u8 addrL)
{
    printk("%s+\n", __func__);

    u8 data1[1] = {0x35};
    u8 data2[1] = {0x71};
    u8 data3[3] = {0x10, addrH, addrL};
    u8 reg_data;

    //MST_IicBusCtrl
    scalar_debug_writebytes(info->i2c_client, data1, 1);
    scalar_debug_writebytes(info->i2c_client, data2, 1);

    //DisableWriteProtect
    enter_serial_debug_mode();
    //MST_ReadScalerReg
    scalar_debug_writebytes(info->i2c_client, data3, 3);
    scalar_debug_readbytes(info->i2c_client, &reg_data, 1);
    exit_serial_debug_mode();

    printk("%s: reg_data=%x\n", __func__, reg_data);
    printk("%s-\n", __func__);
    return 0;
}
static int write_gpio(u8 addrH, u8 addrL, u8 val)
{
    printk("%s+\n", __func__);

    u8 data1[1] = {0x35};
    u8 data2[1] = {0x71};
    u8 data3[3] = {0x10, addrH, addrL};
    u8 data4[4] = {0x10, addrH, addrL, val};
    u8 reg_data;

    //MST_IicBusCtrl
    scalar_debug_writebytes(info->i2c_client, data1, 1);
    scalar_debug_writebytes(info->i2c_client, data2, 1);

    //DisableWriteProtect
    enter_serial_debug_mode();
    //MST_ReadScalerReg
    scalar_debug_writebytes(info->i2c_client, data3, 3);
    scalar_debug_readbytes(info->i2c_client, &reg_data, 1);
    //MST_WriteScalerReg
    scalar_debug_writebytes(info->i2c_client, data4, 4);
    exit_serial_debug_mode();

    printk("%s: reg_data=%x, val=%x\n", __func__, reg_data, val);
    printk("%s-\n", __func__);
    return 0;
}
static int write_scalar_reg(u8 addrH, u8 addrL, u8 val)
{
    printk("%s+\n", __func__);

    u8 data[4] = {0x10, addrH, addrL, val};

    //MST_WriteScalerReg
    scalar_debug_writebytes(info->i2c_client, data, 4);

    printk("%s-\n", __func__);
    return 0;
}
static int keep_gpio_and_enter_isp_mode()
{
    printk("%s+\n", __func__);
    int err = 0;
    u8 data1[1] = {0x35};
    u8 data2[1] = {0x71};
    u8 data3[4] = {0x10, 0xC0, 0x36, 0xFF};
    u8 data4[4] = {0x10, 0xC0, 0x37, 0xFF};
    u8 data5[4] = {0x10, 0xC0, 0xF2, 0x60};
    u8 data6[4] = {0x10, 0xC0, 0xF7, 0x40};

    scalar_update_status = 2;

    //MST_IicBusCtrl
    scalar_debug_writebytes(info->i2c_client, data1, 1);
    scalar_debug_writebytes(info->i2c_client, data2, 1);

    //keep screen on and output normally
    enter_serial_debug_mode();
    //MST_WriteScalerReg
    scalar_debug_writebytes(info->i2c_client, data3, 4);
    scalar_debug_writebytes(info->i2c_client, data4, 4);
    scalar_debug_writebytes(info->i2c_client, data5, 4);
    scalar_debug_writebytes(info->i2c_client, data6, 4);
    exit_serial_debug_mode();

    //enter isp mode
    err = enter_isp_mode();
    if(err) {
        pr_err("enter_isp_mode error\n");
        scalar_update_status = -1;
        return err;
    }

    printk("%s-\n", __func__);
    return 0;
}
static short fw_version()
{
    printk("%s+\n", __func__);
    u32 addr;
    u32 i;
    u32 checksum = 0;

    for (addr=0x000000; addr<0x03FFFF; addr+=VERIFY_CHECK_SIZE) {
        u8 data1[5] = {0x10, 0x03, (u8)((addr & 0xFF0000) >> 16), (u8)((addr & 0x00FF00) >> 8), (u8)((addr & 0x0000FF))};
        u8 data2[1] = {0x11};
        u8 data3[VERIFY_CHECK_SIZE];
        u8 data4[1] = {0x12};

        //read data from flash
        scalar_isp_writebytes(info->i2c_client, data1, 5);
        scalar_isp_writebytes(info->i2c_client, data2, 1);
        scalar_isp_readbytes(info->i2c_client, data3, VERIFY_CHECK_SIZE);
        scalar_isp_writebytes(info->i2c_client, data4, 1);

        //compute checksum
        for(i=0; i< VERIFY_CHECK_SIZE; i++) {
            checksum += data3[i];
        }
    }

    version_num = checksum & 0x0000FFFF;

    printk("%s: checksum=%x, last 2 byte is %x\n", __func__, checksum, version_num);
    printk("%s-\n", __func__);

    return version_num;
}
static int fw_update(char * filename)
{
    printk("%s+\n", __func__);

    int err = 0;

    //1. enter isp mode
    //keep_gpio_and_enter_isp_mode();

    //2. write protect, chip erase
    chip_erase();

    //3. blanking check
    if(!err)
        err = blanking_check();
    else {
        write_status(original_status);
        return err;
    }

    // 4. programming
    if(!err)
        err = programming(filename);
    else {
        write_status(original_status);
        return err;
    }

    // 5. verify
    if(!err)
        err = verify(filename);
    else {
        write_status(original_status);
        return err;
    }

    // 6. write protect
    write_status(original_status);

    // 7. exit isp mode
    //exit_isp_mode();

    printk("%s-\n", __func__);
    return err;
}

static int get_version_from_system_prop()
{
    printk("%s+\n", __func__);
    struct file *fp = NULL;
    mm_segment_t old_fs;
    int i;

    for(i=0; i<20; i++)
        version_buf[i] = NULL;

    //only can be read by root, cannot be read in about page (permission is system)
    fp = filp_open("/data/property/persist.sys.fw.scalar.version", O_RDONLY, 0);
    if ( !IS_ERR_OR_NULL(fp) ){
        pr_info("filp_open success fp:%p\n", fp);
        old_fs = get_fs();
        set_fs(KERNEL_DS);
        if(fp->f_op != NULL && fp->f_op->read != NULL){
            printk("file start to read!\n");
            fp->f_op->read(fp, version_buf, 20, &fp->f_pos);
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
    printk("%s-, %s\n", __func__, version_buf);

    return 0;
}

static long scalar_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int err = 0;

    switch (cmd)
    {
        case SCALAR_CUSTOM_IOCTL_FW_VERSION:
        {
            fw_update_rom_package rom_cmd;
            printk("%s in case SCALAR_CUSTOM_IOCTL_FW_VERSION\n", __func__);
            rom_cmd.fw_version = scalar_fw_version;
            rom_cmd.fw_subversion = scalar_fw_subversion;
            rom_cmd.panel_type = p1801_panel_type;
            //rom_cmd.fw_version_checksum = fw_version();
            printk("%s: In scalar, fw_version=%X, fw_subversion=%X, panel_type=%d\n", __func__, rom_cmd.fw_version, rom_cmd.fw_subversion, rom_cmd.panel_type);
            if(copy_to_user((const void __user *)arg, &rom_cmd, sizeof(rom_cmd))) {
                printk("%s: pass arg fail\n", __func__);
                return -EFAULT;
            }
            return 0;
        }
        case SCALAR_CUSTOM_IOCTL_FW_UPDATE_PROGRAM:
        {
            fw_update_rom_package rom_cmd;
            printk("%s in case SCALAR_CUSTOM_IOCTL_FW_UPDATE_PROGRAM\n", __func__);
            if (copy_from_user(&rom_cmd,(const void __user *)arg, sizeof(rom_cmd))) {
                printk("%s: get arg error\n", __func__);
                return -EFAULT;
            }
            printk("binfile_path=%s\n",rom_cmd.binfile_path);

            err = fw_update(rom_cmd.binfile_path);

            return err;
        }
        case SCALAR_CUSTOM_IOCTL_FW_UPDATE_ENTER:
        {
            err = keep_gpio_and_enter_isp_mode();

            return err;
        }
        case SCALAR_CUSTOM_IOCTL_FW_UPDATE_EXIT:
        {
            err = exit_isp_mode();

            return err;
        }
        case SCALAR_CUSTOM_IOCTL_FW_TEST_BIN:
        {
            fw_update_rom_package rom_cmd;
            printk("%s in case SCALAR_CUSTOM_IOCTL_FW_TEST_BIN\n", __func__);
            if (copy_from_user(&rom_cmd,(const void __user *)arg, sizeof(rom_cmd))) {
                printk("%s: get arg error\n", __func__);
                return -EFAULT;
            }
            printk("binfile_path=%s\n",rom_cmd.binfile_path);

            err = test_open_bin(rom_cmd.binfile_path, &rom_cmd.fw_version_checksum);

            printk("%s: In bin file, fw version checksum is %X\n", __func__, rom_cmd.fw_version_checksum);
            if(copy_to_user((const void __user *)arg, &rom_cmd, sizeof(rom_cmd))) {
                printk("%s: pass arg fail\n", __func__);
                return -EFAULT;
            }
            return err;
        }
        case SCALAR_CUSTOM_IOCTL_FW_VERSION_TO_KERNEL:
        {
            fw_update_rom_package rom_cmd;
            //u32 scalar_fw_checksum;
            int i;
            printk("%s in case SCALAR_CUSTOM_IOCTL_FW_VERSION_TO_KERNEL\n", __func__);
            if (copy_from_user(&rom_cmd,(const void __user *)arg, sizeof(rom_cmd))) {
                printk("%s: get arg error\n", __func__);
                return -EFAULT;
            }

            for(i=0; i<20; i++) {
                version_buf[i] = rom_cmd.fw_version_string[i];
            }
            printk("fw_version_string=%s, version_buf=%s\n",rom_cmd.fw_version_string, version_buf);

            return 0;
        }
        default:
            return -EINVAL;
    }
    return 0;
}

static const struct file_operations scalar_fileops = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = scalar_ioctl,
};

static struct miscdevice scalar_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = SCALAR_NAME,
    .fops = &scalar_fileops,
};

static ssize_t scalar_switch_name(struct switch_dev *sdev, char *buf)
{
    //get_version_from_system_prop();
    return sprintf(buf, "%s\n", version_buf);
}

static ssize_t scalar_switch_state(struct switch_dev *sdev, char *buf)
{
    return sprintf(buf, "%s\n", "0");
}

static int scalar_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err=0;

    pr_info("%s, compiled at %s %s\n",__func__,__DATE__,__TIME__);

    info = kzalloc(sizeof(struct scalar_info), GFP_KERNEL);
    if (!info) {
        pr_err("scalar : Unable to allocate memory!\n");
        return -ENOMEM;
    }

    // for ioctl, communicate with userspace
    err = misc_register(&scalar_device);
    if (err) {
        pr_err("scalar : Unable to register misc device!\n");
        kfree(info);
        return err;
    }
    info->i2c_client = client;
    i2c_set_clientdata(client, info);

    //TODO: read version

    scalar_sdev.name = SCALAR_NAME;
    scalar_sdev.print_name = scalar_switch_name;
    scalar_sdev.print_state = scalar_switch_state;
    if(switch_dev_register(&scalar_sdev) < 0){
        pr_err("switch_dev_register for scalar failed!\n");
    }
    switch_set_state(&scalar_sdev, 0);

    return 0;
}
static int scalar_remove(struct i2c_client *client)
{
    struct sensor_info *info;

    pr_info("%s\n",__func__);
    info = i2c_get_clientdata(client);
    misc_deregister(&scalar_device);
    kfree(info);
    return 0;
}

static const struct i2c_device_id scalar_id[] = {
    { SCALAR_NAME, 0 },
};

MODULE_DEVICE_TABLE(i2c, scalar_id);

static struct i2c_driver scalar_i2c_driver = {
    .driver = {
        .name = SCALAR_NAME,
        .owner = THIS_MODULE,
    },
    .probe = scalar_probe,
    .remove = scalar_remove,
    .id_table = scalar_id,
};

static int __init scalar_init(void)
{
    if(tegra3_get_project_id() == TEGRA3_PROJECT_P1801){
        printk(KERN_INFO "P1801 scalar: %s+ #####\n", __func__);
        int ret;
        ret = i2c_add_driver(&scalar_i2c_driver);
        printk(KERN_INFO "P1801 scalar: %s- #####\n", __func__, ret);
        return ret;
    }
    return 0;
}

static void __exit scalar_exit(void)
{
    if(tegra3_get_project_id() == TEGRA3_PROJECT_P1801){
        pr_info("P1801 scalar: %s\n",__func__);
        i2c_del_driver(&scalar_i2c_driver);
        switch_dev_unregister(&scalar_sdev);
    }
}

module_init(scalar_init);
module_exit(scalar_exit);

#define CONFIG_I2C_READ_WRITE
#ifdef CONFIG_I2C_READ_WRITE
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <mach/clk.h>

#define DBG_TXT_BUF_SIZE 256
static char debugTxtBuf[DBG_TXT_BUF_SIZE];

static ssize_t i2c_set_open(struct inode *inode, struct file *file)
{
    file->private_data = inode->i_private;
    return 0;
}
static int i2c_scalar_write(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    char *bp = debugTxtBuf;
    int len;
    int arg[2];
    int err;
    char filename[DBG_TXT_BUF_SIZE];

    if (*ppos)
        return 0;    /* the end */

//+ parsing......
    len=( count > DBG_TXT_BUF_SIZE-1 ) ? ( DBG_TXT_BUF_SIZE-1 ) : (count);
    if ( copy_from_user( debugTxtBuf, buf, len ) )
        return -EFAULT;

    debugTxtBuf[len] = 0; //add string end

    sscanf(debugTxtBuf, "%d %s", &arg[0], filename);
    pr_info("0 is enter_isp_mode, 1 is exit_isp_mode, 2 is firmware_update\n");
    pr_info("6 is to dump current FW to /data/FW_read_test.bin\n");
    pr_info("command is %d, filename is %s\n", arg[0], filename);

    *ppos = len;

    switch(arg[0])
    {
        case 0:
        {
            pr_info("enter_isp_mode\n");
            err = enter_isp_mode();
            if(err)
                return -ENOMEM;
            break;
        }
        case 1:
        {
            pr_info("exit_isp_mode\n");
            err = exit_isp_mode();
            if(err)
                return -ENOMEM;
            break;
        }
        case 2:
        {
            pr_info("firmware_update\n");
            err = fw_update(filename);
            if(err)
                return -ENOMEM;
            break;
        }
        case 3:
        {
            pr_info("enter_serial_debug_mode\n");
            err = enter_serial_debug_mode();
            if(err)
                return -ENOMEM;
            break;
        }
        case 4:
        {
            pr_info("exit_serial_debug_mode\n");
            err = exit_serial_debug_mode();
            if(err)
                return -ENOMEM;
            break;
        }
        case 5:
        {
            pr_info("test read FW\n");
            err = test_read_FW();
            if(err)
                return -ENOMEM;
            break;
        }
        case 6:
        {
            pr_info("test read FW to file\n");
            err = test_read_FW_to_file();
            if(err)
                return -ENOMEM;
            break;
        }
        case 7:
        {
            pr_info("keep gpio setting and enter isp mode\n");
            err = keep_gpio_and_enter_isp_mode();
            if(err)
                return -ENOMEM;
            break;
        }
        case 8:
        {
            pr_info("test read file\n");
            err = test_read_file(filename);
            if(err)
                return -ENOMEM;
            break;
        }
        case 9:
        {
            pr_info("test chip erase\n");
            err = chip_erase();
            if(err)
                return -ENOMEM;
            break;
        }
        case 10:
        {
            pr_info("test blanking check\n");
            err = blanking_check();
            if(err)
                return -ENOMEM;
            break;
        }
        case 11:
        {
            pr_info("test programming\n");
            err = programming(filename);
            if(err)
                return -ENOMEM;
            break;
        }
        case 12:
        {
            pr_info("test verify\n");
            err = verify(filename);
            if(err)
                return -ENOMEM;
            break;
        }
        case 13:
        {
            pr_info("test write protect\n");
            err = write_status(original_status);
            if(err)
                return -ENOMEM;
            break;
        }
        case 14:
        {
            pr_info("test read gpio\n");
            read_gpio(0xC0, 0x36);
            read_gpio(0xC0, 0x37);
            read_gpio(0xC0, 0xF2);
            read_gpio(0xC0, 0xF7);
            break;
        }
        case 15:
        {
            pr_info("test write gpio\n");
            write_gpio(0xC0, 0x36, 0xFF);
            write_gpio(0xC0, 0x37, 0xFF);
            write_gpio(0xC0, 0xF2, 0x60);
            write_gpio(0xC0, 0xF7, 0x40);
            break;
        }
        case 16:
        {
            pr_info("test fw_version\n");
            fw_version();
            break;
        }
        case 17:
        {
            pr_info("test get_version_from_system_prop\n");
            get_version_from_system_prop();
            break;
        }
        default:
            break;
    }

    return len;    /* the end */
}
static int i2c_scalar_firmware_update_status_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int len = 0;
    char *bp = debugTxtBuf;

    if (*ppos)
        return 0;    /* the end */
    if(scalar_update_status == 0)
        len = snprintf(bp, DBG_TXT_BUF_SIZE, "Never do firmware update\n");
    else if(scalar_update_status == 1)
        len = snprintf(bp, DBG_TXT_BUF_SIZE, "Firmware update success\n");
    else if(scalar_update_status == 2)
        len = snprintf(bp, DBG_TXT_BUF_SIZE, "Firmware update unfinished\n");
    else
        len = snprintf(bp, DBG_TXT_BUF_SIZE, "Firmware update fail\n");

    if (copy_to_user(buf, debugTxtBuf, len))
        return -EFAULT;

    *ppos += len;
    return len;
}
static const struct file_operations i2c_scalar = {
    .open = i2c_set_open,
    .write = i2c_scalar_write,
};
static const struct file_operations i2c_scalar_firmware_update_status = {
    .open = i2c_set_open,
    .read = i2c_scalar_firmware_update_status_read,
};
static int __init tegra_i2c_debuginit(void)
{
    struct dentry *dent = debugfs_create_dir("scalar", NULL);
    if(0/*factory_mode == 2*/){
        (void) debugfs_create_file("i2c_scalar", 0777,
                dent, NULL, &i2c_scalar);
        (void) debugfs_create_file("i2c_scalar_firmware_update_status", 0777,
                dent, NULL, &i2c_scalar_firmware_update_status);
    }
    else{
        (void) debugfs_create_file("i2c_scalar", 0700,
                dent, NULL, &i2c_scalar);
        (void) debugfs_create_file("i2c_scalar_firmware_update_status", 0700,
                dent, NULL, &i2c_scalar_firmware_update_status);
    }

    return 0;
}

late_initcall(tegra_i2c_debuginit);
#endif

