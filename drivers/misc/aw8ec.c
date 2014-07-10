/*
 * ASUS Dock EC driver for p1801.  plus splashtop gpio:PI3 and w8 detect gpio:PI6, scalar_status gpio:PH4
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/cdev.h>
#include <linux/gpio_event.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <asm/gpio.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/power_supply.h>
#include <../gpio-names.h>
#include <mach/board-cardhu-misc.h>

#include "aw8ec.h"
#include "../input/asusec/elan_i2c_asus.h"

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

/*
 * functions declaration
 */
static int aw8ec_i2c_write_data(struct i2c_client *client, u16 data);
static int aw8ec_i2c_read_data(struct i2c_client *client);
static void aw8ec_reset_dock(void);
static int aw8ec_is_init_running(void);
static int aw8ec_chip_init(struct i2c_client *client);
static void aw8ec_dock_status_report(void);
static void aw8ec_w8_status_report(void);
static void aw8ec_scalar_status_report(void);
static void aw8ec_lid_report_function(struct work_struct *dat);
static void aw8ec_work_function(struct work_struct *dat);
static void aw8ec_dock_init_work_function(struct work_struct *dat);
static void aw8ec_fw_update_work_function(struct work_struct *dat);
static int __devinit aw8ec_probe(struct i2c_client *client,
		const struct i2c_device_id *id);
static int __devexit aw8ec_remove(struct i2c_client *client);
static int aw8ec_kp_key_mapping(int x);
static ssize_t aw8ec_show_dock(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_tp_status_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_info_show(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_store_led(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t aw8ec_charging_led_store(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t aw8ec_led_show(struct device *class,
	struct device_attribute *attr,char *buf);
static ssize_t aw8ec_store_ec_wakeup(struct device *class,
		struct device_attribute *attr,const char *buf, size_t count);
static ssize_t aw8ec_show_drain(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show_dock_battery(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show_dock_battery_status(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show_dock_battery_all(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show_dock_control_flag(struct device *class,
		struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show_lid_status(struct device *class,
		struct device_attribute *attr,char *buf);
//new
static ssize_t aw8ec_show_w8_status(struct device *class,
             struct device_attribute *attr,char *buf);
static ssize_t aw8ec_show_scalar_status(struct device *class,
             struct device_attribute *attr,char *buf);
static int aw8ec_keypad_get_response(struct i2c_client *client, int res);
static int aw8ec_keypad_enable(struct i2c_client *client);
static int aw8ec_touchpad_get_response(struct i2c_client *client, int res);
static int aw8ec_touchpad_enable(struct i2c_client *client);
static int aw8ec_touchpad_disable(struct i2c_client *client);
//static int aw8ec_touchpad_reset(struct i2c_client *client);
static int aw8ec_suspend(struct i2c_client *client, pm_message_t mesg);
static int aw8ec_resume(struct i2c_client *client);
static int aw8ec_open(struct inode *inode, struct file *flip);
static int aw8ec_release(struct inode *inode, struct file *flip);
static long aw8ec_ioctl(struct file *flip, unsigned int cmd, unsigned long arg);
static void aw8ec_enter_factory_mode(void);
static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static void BuffPush(char data);
static int aw8ec_input_device_create(struct i2c_client *client);
static int aw8ec_lid_input_device_create(struct i2c_client *client);
static ssize_t aw8ec_switch_name(struct switch_dev *sdev, char *buf);
static ssize_t aw8ec_switch_state(struct switch_dev *sdev, char *buf);
//new
static ssize_t aw8ec_aio_switch_name(struct switch_dev * sdev, char * buf);
static ssize_t aw8ec_aio_switch_state(struct switch_dev *sdev, char *buf);
static ssize_t aw8ec_scalar_switch_name(struct switch_dev * sdev, char * buf);
static ssize_t aw8ec_scalar_switch_state(struct switch_dev *sdev, char *buf);
static int aw8ec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);
static int aw8ec_dock_battery_get_capacity(union power_supply_propval *val);
static int aw8ec_dock_battery_get_status(union power_supply_propval *val);
static int aw8ec_dock_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val);
/*-----------------------------------------*/
#if DOCK_SPEAKER
extern int audio_dock_event(int status);
#endif
#if BATTERY_DRIVER
extern int docking_callback(int status);
#endif
#if DOCK_USB
extern void fsl_dock_ec_callback(void);
#endif
/*
* extern variable
*/
extern unsigned int factory_mode;

/*
 * global variable
 */
bool p1801isDockIn = 0;
EXPORT_SYMBOL(p1801isDockIn);

static unsigned int aw8ec_apwake_gpio = TEGRA_GPIO_PS7;
static unsigned int aw8ec_ecreq_gpio = TEGRA_GPIO_PQ6;
static unsigned int aw8ec_dock_in_gpio = TEGRA_GPIO_PU4;
static unsigned int aw8ec_hall_sensor_gpio = TEGRA_GPIO_PS6;
static unsigned int aw8ec_splash_gpio = TEGRA_GPIO_PI3;
static unsigned int aw8ec_w8_gpio = TEGRA_GPIO_PI6;
static unsigned int aw8ec_scalar_status_gpio = TEGRA_GPIO_PH4;



static char host_to_ec_buffer[EC_BUFF_LEN];
static char ec_to_host_buffer[EC_BUFF_LEN];
static int h2ec_count;
static int buff_in_ptr;	  // point to the next free place
static int buff_out_ptr;	  // points to the first data


static struct i2c_client dockram_client;
static struct class *aw8ec_class;
static struct device *aw8ec_device ;
static struct aw8ec_chip *ec_chip;

struct cdev *aw8ec_cdev ;
static dev_t aw8ec_dev ;
static int aw8ec_major = 0 ;
static int aw8ec_minor = 0 ;
static int w8_lock_state = 0;

static struct workqueue_struct *aw8ec_wq;
struct delayed_work aw8ec_stress_work;

static const struct i2c_device_id aw8ec_id[] = {
	{"aw8ec", 0},
	{}
};

static enum power_supply_property aw8ec_dock_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CAPACITY,
};

static struct power_supply aw8ec_power_supply[] = {
	{
		.name		= "dock_battery",
		.type		= POWER_SUPPLY_TYPE_DOCK_BATTERY,
		.properties	= aw8ec_dock_properties,
		.num_properties	= ARRAY_SIZE(aw8ec_dock_properties),
		.get_property	= aw8ec_dock_battery_get_property,
	},
};

MODULE_DEVICE_TABLE(i2c, aw8ec_id);

struct file_operations aw8ec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = aw8ec_ioctl,
	.open = aw8ec_open,
	.write = ec_write,
	.read = ec_read,
	.release = aw8ec_release,
};


static struct i2c_driver aw8ec_driver = {
	.class	= I2C_CLASS_HWMON,
	.driver	 = {
		.name = "aw8ec",
		.owner = THIS_MODULE,
	},
	.probe	 = aw8ec_probe,
	.remove	 = __devexit_p(aw8ec_remove),
	.suspend = aw8ec_suspend,
	.resume = aw8ec_resume,
	.id_table = aw8ec_id,
};


static DEVICE_ATTR(ec_status, S_IWUSR | S_IRUGO, aw8ec_show,NULL);
static DEVICE_ATTR(ec_tp_status, S_IWUSR | S_IRUGO, aw8ec_tp_status_show,NULL);
static DEVICE_ATTR(ec_info, S_IWUSR | S_IRUGO, aw8ec_info_show,NULL);
static DEVICE_ATTR(ec_dock, S_IWUSR | S_IRUGO, aw8ec_show_dock,NULL);
static DEVICE_ATTR(ec_dock_led, S_IWUSR | S_IRUGO, aw8ec_led_show,aw8ec_store_led);
static DEVICE_ATTR(ec_charging_led, S_IWUSR | S_IRUGO, NULL, aw8ec_charging_led_store);
static DEVICE_ATTR(ec_wakeup, S_IWUSR | S_IRUGO, NULL,aw8ec_store_ec_wakeup);
static DEVICE_ATTR(ec_dock_discharge, S_IWUSR | S_IRUGO, aw8ec_show_drain,NULL);
static DEVICE_ATTR(ec_dock_battery, S_IWUSR | S_IRUGO, aw8ec_show_dock_battery,NULL);
static DEVICE_ATTR(ec_dock_battery_status, S_IWUSR | S_IRUGO, aw8ec_show_dock_battery_status,NULL);
static DEVICE_ATTR(ec_dock_battery_all, S_IWUSR | S_IRUGO, aw8ec_show_dock_battery_all,NULL);
static DEVICE_ATTR(ec_dock_control_flag, S_IWUSR | S_IRUGO, aw8ec_show_dock_control_flag,NULL);
static DEVICE_ATTR(ec_lid, S_IWUSR | S_IRUGO, aw8ec_show_lid_status,NULL);
static DEVICE_ATTR(ec_w8, S_IWUSR | S_IRUGO, aw8ec_show_w8_status,NULL);  //new
static DEVICE_ATTR(ec_scalar_status, S_IWUSR | S_IRUGO, aw8ec_show_scalar_status,NULL);  //new

static struct attribute *aw8ec_smbus_attributes[] = {
	&dev_attr_ec_status.attr,
	&dev_attr_ec_tp_status.attr,
	&dev_attr_ec_info.attr,
	&dev_attr_ec_dock.attr,
	&dev_attr_ec_dock_led.attr,
	&dev_attr_ec_charging_led.attr,
	&dev_attr_ec_wakeup.attr,
	&dev_attr_ec_dock_discharge.attr,
	&dev_attr_ec_dock_battery.attr,
	&dev_attr_ec_dock_battery_status.attr,
	&dev_attr_ec_dock_battery_all.attr,
	&dev_attr_ec_dock_control_flag.attr,
	&dev_attr_ec_lid.attr,
	&dev_attr_ec_w8.attr,
	&dev_attr_ec_scalar_status.attr,
NULL
};


static const struct attribute_group aw8ec_smbus_group = {
	.attrs = aw8ec_smbus_attributes,
};

static int aw8ec_kp_sci_table[]={0, KEY_SLEEP, KEY_WLAN, KEY_BLUETOOTH,
		aw8ec_KEY_TOUCHPAD, KEY_BRIGHTNESSDOWN, KEY_BRIGHTNESSUP, aw8ec_KEY_AUTOBRIGHT,
		KEY_CAMERA, -9, -10, -11,
		-12, -13, -14, -15,
		KEY_WWW, aw8ec_KEY_SETTING, KEY_PREVIOUSSONG, KEY_PLAYPAUSE,
		KEY_NEXTSONG, KEY_MUTE, KEY_VOLUMEDOWN, KEY_VOLUMEUP};

/*
 * functions definition
 */
static void aw8ec_dockram_init(struct i2c_client *client){
	dockram_client.adapter = client->adapter;
	dockram_client.addr = 0x1b;
	dockram_client.detected = client->detected;
	dockram_client.dev = client->dev;
	dockram_client.driver = client->driver;
	dockram_client.flags = client->flags;
	strcpy(dockram_client.name,client->name);
}

static int aw8ec_dockram_write_data(int cmd, int length)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	ret = i2c_smbus_write_i2c_block_data(&dockram_client, cmd, length, ec_chip->i2c_dm_data);
	if (ret < 0) {
		aw8ec_ERR("Fail to read dockram data, status %d\n", ret);
	}
	return ret;
}

static int aw8ec_dockram_read_data(int cmd)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(&dockram_client, cmd, 32, ec_chip->i2c_dm_data);
	if (ret < 0) {
		aw8ec_ERR("Fail to read dockram data, status %d\n", ret);
	}
	return ret;
}

static int aw8ec_i2c_write_data(struct i2c_client *client, u16 data)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	ret = i2c_smbus_write_word_data(client, 0x64, data);
	if (ret < 0) {
		aw8ec_ERR("Fail to write data, status %d\n", ret);
	}
	return ret;
}

static int aw8ec_i2c_read_data(struct i2c_client *client)
{
	int ret = 0;

	if (ec_chip->dock_in == 0){
		return -1;
	}

	ret = i2c_smbus_read_i2c_block_data(client, 0x6A, 8, ec_chip->i2c_data);
	if (ret < 0) {
		aw8ec_ERR("Fail to read data, status %d\n", ret);
	}
	return ret;
}

static int aw8ec_keypad_get_response(struct i2c_client *client, int res)
{
	int retry = aw8ec_RETRY_COUNT;

	while(retry-- > 0){
		aw8ec_i2c_read_data(client);
		aw8ec_I2C_DATA(ec_chip->i2c_data, ec_chip->index);
		if ((ec_chip->i2c_data[1] & aw8ec_OBF_MASK) &&
			(!(ec_chip->i2c_data[1] & aw8ec_AUX_MASK))){
			if (ec_chip->i2c_data[2]  == res){
				goto get_aw8ec_keypad_i2c;
			}
		}
		msleep(CONVERSION_TIME_MS/5);
	}
	return -1;

get_aw8ec_keypad_i2c:
	return 0;

}

static int aw8ec_keypad_enable(struct i2c_client *client)
{
	int retry = aw8ec_RETRY_COUNT;

	while(retry-- > 0){
		aw8ec_i2c_write_data(client, 0xF400);
		if(!aw8ec_keypad_get_response(client, aw8ec_PS2_ACK)){
			goto keypad_enable_ok;
		}
	}
	aw8ec_ERR("fail to enable keypad");
	return -1;

keypad_enable_ok:
	return 0;
}

static int aw8ec_keypad_disable(struct i2c_client *client)
{
	int retry = aw8ec_RETRY_COUNT;

	while(retry-- > 0){
		aw8ec_i2c_write_data(client, 0xF500);
		if(!aw8ec_keypad_get_response(client, aw8ec_PS2_ACK)){
			goto keypad_disable_ok;
		}
	}

	aw8ec_ERR("fail to disable keypad");
	return -1;

keypad_disable_ok:
	return 0;
}

static void aw8ec_keypad_led_on(struct work_struct *dat)
{
	ec_chip->kbc_value = 1;
	aw8ec_INFO("send led cmd 1\n");
	msleep(250);
	aw8ec_i2c_write_data(ec_chip->client, 0xED00);
}


static void aw8ec_keypad_led_off(struct work_struct *dat)
{
	ec_chip->kbc_value = 0;
	aw8ec_INFO("send led cmd 1\n");
	msleep(250);
	aw8ec_i2c_write_data(ec_chip->client, 0xED00);
}


static int aw8ec_touchpad_get_response(struct i2c_client *client, int res)
{
	int retry = aw8ec_RETRY_COUNT;

	msleep(CONVERSION_TIME_MS);
	while(retry-- > 0){
		aw8ec_i2c_read_data(client);
		aw8ec_I2C_DATA(ec_chip->i2c_data, ec_chip->index);
		if ((ec_chip->i2c_data[1] & aw8ec_OBF_MASK) &&
			(ec_chip->i2c_data[1] & aw8ec_AUX_MASK)){
			if (ec_chip->i2c_data[2] == res){
				goto get_aw8ec_touchpad_i2c;
			}
		}
		msleep(CONVERSION_TIME_MS/5);
	}

	aw8ec_ERR("fail to get touchpad response");
	return -1;

get_aw8ec_touchpad_i2c:
	return 0;

}

static int aw8ec_touchpad_enable(struct i2c_client *client)
{
	ec_chip->tp_wait_ack = 1;
	aw8ec_i2c_write_data(client, 0xF4D4);
	return 0;
}

static int aw8ec_touchpad_disable(struct i2c_client *client)
{
	int retry = 5;

	while(retry-- > 0){
		aw8ec_i2c_write_data(client, 0xF5D4);
		if(!aw8ec_touchpad_get_response(client, aw8ec_PS2_ACK)){
			goto touchpad_disable_ok;
		}
	}

	aw8ec_ERR("fail to disable touchpad");
	return -1;

touchpad_disable_ok:
	return 0;
}

static void aw8ec_fw_clear_buf(void){
	int i;

	for (i = 0; i < 64; i++){
		i2c_smbus_read_byte_data(&dockram_client, 0);
	}
}

static void aw8ec_fw_reset_ec_op(void){
	char i2c_data[32];
	int i;
	int r_data[32];

	aw8ec_fw_clear_buf();

	i2c_data[0] = 0x01;
	i2c_data[1] = 0x21;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static void aw8ec_fw_address_set_op(void){
	char i2c_data[32];
	int i;
	int r_data[32];

	aw8ec_fw_clear_buf();

	i2c_data[0] = 0x05;
	i2c_data[1] = 0xa0;
	i2c_data[2] = 0x00;
	i2c_data[3] = 0x00;
	i2c_data[4] = 0x02;
	i2c_data[5] = 0x00;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static void aw8ec_fw_enter_op(void){
	char i2c_data[32];
	int i;
	int r_data[32];

	aw8ec_fw_clear_buf();

	i2c_data[0] = 0x05;
	i2c_data[1] = 0x10;
	i2c_data[2] = 0x55;
	i2c_data[3] = 0xaa;
	i2c_data[4] = 0xcd;
	i2c_data[5] = 0xbe;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*4);
}

static int aw8ec_fw_cmp_id(void){
	char i2c_data[32];
	int i;
	int r_data[32];
	int ret_val = 0;

	aw8ec_fw_clear_buf();

	i2c_data[0] = 0x01;
	i2c_data[1] = 0xC0;
	for (i = 0; i < i2c_data[0]+1 ; i++){
		i2c_smbus_write_byte_data(&dockram_client, i2c_data[i],0);
	}
	msleep(CONVERSION_TIME_MS*10);

	for (i = 0; i < 5; i++){
		r_data[i] = i2c_smbus_read_byte_data(&dockram_client, 0);
	}

	for (i = 0; i < 5; i++){
		aw8ec_NOTICE("r_data[%d] = 0x%x\n", i, r_data[i]);
	}

	if (r_data[0] == 0xfa &&
		r_data[1] == 0xf0 &&
		r_data[2] == 0x12 &&
		r_data[3] == 0xef &&
		r_data[4] == 0x12){
		ret_val = 0;
	} else {
		ret_val = 1;
	}

	return ret_val;
}

static void aw8ec_fw_reset(void){

	if (aw8ec_fw_cmp_id() == 0){
		aw8ec_fw_enter_op();
		aw8ec_fw_address_set_op();
		aw8ec_fw_reset_ec_op();
		aw8ec_fw_clear_buf();
		if (ec_chip->re_init == 0){
			queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_dock_init_work, HZ/2);
			ec_chip->re_init = 1;
		}
	}
}
static int aw8ec_i2c_test(struct i2c_client *client){
	return aw8ec_i2c_write_data(client, 0x0000);
}

static void aw8ec_reset_dock(void){
	ec_chip->dock_init = 0;
	aw8ec_NOTICE("send EC_Request\n");
	gpio_set_value(aw8ec_ecreq_gpio, 0);
	msleep(20);
	gpio_set_value(aw8ec_ecreq_gpio, 1);
}
static int aw8ec_is_init_running(void){
	int ret_val;

	mutex_lock(&ec_chip->dock_init_lock);
	ret_val = ec_chip->dock_init;
	ec_chip->dock_init = 1;
	mutex_unlock(&ec_chip->dock_init_lock);
	return ret_val;
}

static void aw8ec_clear_i2c_buffer(struct i2c_client *client){
	int i;
	for ( i=0; i<8; i++){
		aw8ec_i2c_read_data(client);
	}
}
static int aw8ec_chip_init(struct i2c_client *client)
{
	int ret_val = 0;
	int i;
       /*
	if(aw8ec_is_init_running()){
		return 0;
	}
      */
	wake_lock(&ec_chip->wake_lock);
	memset(ec_chip->ec_model_name, 0, 32);
	memset(ec_chip->ec_version, 0, 32);
	disable_irq_nosync(gpio_to_irq(aw8ec_apwake_gpio));
       /*
	for ( i = 0; i < 3; i++){
		ret_val = aw8ec_i2c_test(client);
		if (ret_val < 0)
			msleep(1000);
		else
			break;
	}
	if(ret_val < 0){
		goto fail_to_access_ec;
	}

	for ( i=0; i<8; i++){
		aw8ec_i2c_read_data(client);
	}

	if (aw8ec_dockram_read_data(0x01) < 0){
		goto fail_to_access_ec;
	}
	*/
	strcpy(ec_chip->ec_model_name, &ec_chip->i2c_dm_data[1]);
	aw8ec_NOTICE("Model Name: %s\n", ec_chip->ec_model_name);
	/*
	if (aw8ec_dockram_read_data(0x02) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->ec_version, &ec_chip->i2c_dm_data[1]);
	aw8ec_NOTICE("EC-FW Version: %s\n", ec_chip->ec_version);

	if (aw8ec_dockram_read_data(0x03) < 0){
		goto fail_to_access_ec;
	}
	aw8ec_INFO("EC-Config Format: %s\n", &ec_chip->i2c_dm_data[1]);

	if (aw8ec_dockram_read_data(0x04) < 0){
		goto fail_to_access_ec;
	}
	strcpy(ec_chip->dock_pid, &ec_chip->i2c_dm_data[1]);
	aw8ec_INFO("PID Version: %s\n", ec_chip->dock_pid);

	if (aw8ec_dockram_read_data(0x0A) < 0){
		goto fail_to_access_ec;
	}
	*/
	ec_chip->dock_behavior = ec_chip->i2c_dm_data[2] & 0x02;
	aw8ec_NOTICE("EC-FW Behavior: %s\n", ec_chip->dock_behavior ?
		"susb on when receive ec_req" : "susb on when system wakeup");

	ec_chip->tf_dock = 1;
       /*
	if(factory_mode == 2)
		aw8ec_enter_factory_mode();

	if(aw8ec_input_device_create(client)){
		goto fail_to_access_ec;
	}

	if (201){
		if (ec_chip->init_success == 0){
			msleep(750);
		}
		aw8ec_clear_i2c_buffer(client);
		aw8ec_touchpad_disable(client);
	}
	*/
	aw8ec_keypad_disable(client);

#if TOUCHPAD_ELAN
#if TOUCHPAD_MODE
     /*
      if (1){
		aw8ec_clear_i2c_buffer(client);
		if ((!elantech_detect(ec_chip)) && (!elantech_init(ec_chip))){
		    ec_chip->touchpad_member = ELANTOUCHPAD;
		} else {
			ec_chip->touchpad_member = -1;
		}
	}
	*/
#endif
#endif

	aw8ec_NOTICE("touchpad and keyboard init\n");
	ec_chip->d_index = 0;

	aw8ec_keypad_enable(client);
	aw8ec_clear_i2c_buffer(client);

	enable_irq(gpio_to_irq(aw8ec_apwake_gpio));
	ec_chip->init_success = 1;
       /*
	if ((201) && ec_chip->tp_enable){
		aw8ec_touchpad_enable(client);
	}
	*/

	ec_chip->status = 1;
	aw8ec_dock_status_report();
       aw8ec_w8_status_report();
	wake_unlock(&ec_chip->wake_lock);
	return 0;

fail_to_access_ec:
	if (aw8ec_dockram_read_data(0x00) < 0){
		aw8ec_NOTICE("No EC detected\n");
		ec_chip->dock_in = 0;
	} else {
		aw8ec_NOTICE("Need EC FW update\n");
		aw8ec_fw_reset();
	}
	enable_irq(gpio_to_irq(aw8ec_apwake_gpio));
	wake_unlock(&ec_chip->wake_lock);
	return -1;

}


static irqreturn_t aw8ec_interrupt_handler(int irq, void *dev_id){

	int gpio = irq_to_gpio(irq);

	if (gpio == aw8ec_apwake_gpio){
		disable_irq_nosync(irq);
		if (ec_chip->op_mode){
			queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_fw_update_work, 0);
		}
		else{
			if (ec_chip->suspend_state){
				ec_chip->wakeup_lcd = 1;
				ec_chip->ap_wake_wakeup = 1;
			}
			queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_work, 0);
		}
	}
	else if (gpio == aw8ec_dock_in_gpio){
		ec_chip->dock_in = 0;
		ec_chip->dock_det++;
		queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_dock_init_work, 0);
	} else if (gpio == aw8ec_hall_sensor_gpio){
		queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_hall_sensor_work, 0);
	} else if (gpio == aw8ec_w8_gpio){
		queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_w8_work, 0);
	}else if (gpio == aw8ec_scalar_status_gpio){
		queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_scalar_status_work, 0);
	}
	return IRQ_HANDLED;
}

static int aw8ec_irq_hall_sensor(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_hall_sensor_gpio;
	unsigned irq = gpio_to_irq(aw8ec_hall_sensor_gpio);
	const char* label = "aw8ec_hall_sensor" ;
	unsigned int pad_pid = tegra3_get_project_id();

	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		aw8ec_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, aw8ec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		aw8ec_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}

	if ((pad_pid == TEGRA3_PROJECT_TF300T) || (pad_pid == TEGRA3_PROJECT_TF300TG)){
		aw8ec_NOTICE("Disable hall sensor wakeup function due to pid = %u\n", pad_pid);
	} else {
		enable_irq_wake(irq);
	}

	aw8ec_INFO("LID irq = %d, rc = %d\n", irq, rc);

	if (gpio_get_value(gpio)){
		aw8ec_NOTICE("LID open\n");
	} else{
		aw8ec_NOTICE("LID close\n");
	}

	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}

static int aw8ec_irq_w8_in(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_w8_gpio;
	unsigned irq = gpio_to_irq(gpio);
	const char* label = "aw8ec_w8" ;

	aw8ec_INFO("aw8ec_irq_w8_in\n");
	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		aw8ec_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, aw8ec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		aw8ec_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	aw8ec_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}

static int aw8ec_irq_scalar_status(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_scalar_status_gpio;
	unsigned irq = gpio_to_irq(gpio);
	const char* label = "aw8ec_scalar_status" ;

	aw8ec_INFO("aw8ec_irq_scalar_statusn\n");
	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		aw8ec_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, aw8ec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		aw8ec_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	aw8ec_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}
static int aw8ec_irq_dock_in(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_dock_in_gpio;
	unsigned irq = gpio_to_irq(aw8ec_dock_in_gpio);
	const char* label = "aw8ec_dock_in" ;

	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		aw8ec_ERR("gpio_request failed for input %d\n", gpio);
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, aw8ec_interrupt_handler,IRQF_SHARED|IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING/*|IRQF_TRIGGER_HIGH|IRQF_TRIGGER_LOW*/, label, client);
	if (rc < 0) {
		aw8ec_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	aw8ec_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
	return rc;
}

static int aw8ec_irq(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_apwake_gpio;
	unsigned irq = gpio_to_irq(aw8ec_apwake_gpio);
	const char* label = "aw8ec_input" ;

	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		aw8ec_ERR("gpio_request failed for input %d\n", gpio);
		goto err_request_input_gpio_failed;
	}

	rc = gpio_direction_input(gpio) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_input failed for input %d\n", gpio);
		goto err_gpio_direction_input_failed;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = request_irq(irq, aw8ec_interrupt_handler,/*IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING|IRQF_TRIGGER_HIGH|*/IRQF_TRIGGER_LOW, label, client);
	if (rc < 0) {
		aw8ec_ERR("Could not register for %s interrupt, irq = %d, rc = %d\n", label, irq, rc);
		rc = -EIO;
		goto err_gpio_request_irq_fail ;
	}
	enable_irq_wake(irq);
	aw8ec_INFO("request irq = %d, rc = %d\n", irq, rc);

	return 0 ;

err_gpio_request_irq_fail :
	gpio_free(gpio);
err_gpio_direction_input_failed:
err_request_input_gpio_failed :
	return rc;
}

static int aw8ec_irq_ec_request(struct i2c_client *client)
{
	int rc = 0 ;
	unsigned gpio = aw8ec_ecreq_gpio;
	unsigned irq = gpio_to_irq(aw8ec_apwake_gpio);
	const char* label = "aw8ec_request" ;

	aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	rc = gpio_request(gpio, label);
	if (rc) {
		aw8ec_ERR("gpio_request failed for input %d\n", gpio);
		goto err_exit;
	}

	rc = gpio_direction_output(gpio, 1) ;
	if (rc) {
		aw8ec_ERR("gpio_direction_output failed for input %d\n", gpio);
		goto err_exit;
	}
	aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

	return 0 ;

err_exit:
	return rc;
}


static int aw8ec_irq_splash_request(struct i2c_client *client)
{
        int rc = 0 ;
        unsigned gpio = aw8ec_splash_gpio;
        unsigned irq = gpio_to_irq(gpio);
        const char* label = "aw8ec_splash" ;

        aw8ec_INFO("gpio = %d, irq = %d\n", gpio, irq);
        aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

        rc = gpio_request(gpio, label);
        if (rc) {
                aw8ec_ERR("gpio_request failed for input %d\n", gpio);
                goto err_exit;
        }

        rc = gpio_direction_output(gpio, 1) ;
        if (rc) {
                aw8ec_ERR("gpio_direction_output failed for input %d\n", gpio);
                goto err_exit;
        }
        aw8ec_INFO("GPIO = %d , state = %d\n", gpio, gpio_get_value(gpio));

        return 0 ;

err_exit:
        return rc;
}


static int aw8ec_kp_key_mapping(int x)
{
	switch (x){
		case aw8ec_KEYPAD_ESC:
			return KEY_BACK;

		case aw8ec_KEYPAD_KEY_WAVE:
			return KEY_GRAVE;

		case aw8ec_KEYPAD_KEY_1:
			return KEY_1;

		case aw8ec_KEYPAD_KEY_2:
			return KEY_2;

		case aw8ec_KEYPAD_KEY_3:
			return KEY_3;

		case aw8ec_KEYPAD_KEY_4:
			return KEY_4;

		case aw8ec_KEYPAD_KEY_5:
			return KEY_5;

		case aw8ec_KEYPAD_KEY_6:
			return KEY_6;

		case aw8ec_KEYPAD_KEY_7:
			return KEY_7;

		case aw8ec_KEYPAD_KEY_8:
			return KEY_8;

		case aw8ec_KEYPAD_KEY_9:
			return KEY_9;

		case aw8ec_KEYPAD_KEY_0:
			return KEY_0;

		case aw8ec_KEYPAD_KEY_MINUS:
			return KEY_MINUS;

		case aw8ec_KEYPAD_KEY_EQUAL:
			return KEY_EQUAL;

		case aw8ec_KEYPAD_KEY_BACKSPACE:
			return KEY_BACKSPACE;

		case aw8ec_KEYPAD_KEY_TAB:
			return KEY_TAB;

		case aw8ec_KEYPAD_KEY_Q:
			return KEY_Q;

		case aw8ec_KEYPAD_KEY_W:
			return KEY_W;

		case aw8ec_KEYPAD_KEY_E:
			return KEY_E;

		case aw8ec_KEYPAD_KEY_R:
			return KEY_R;

		case aw8ec_KEYPAD_KEY_T:
			return KEY_T;

		case aw8ec_KEYPAD_KEY_Y:
			return KEY_Y;

		case aw8ec_KEYPAD_KEY_U:
			return KEY_U;

		case aw8ec_KEYPAD_KEY_I:
			return KEY_I;

		case aw8ec_KEYPAD_KEY_O:
			return KEY_O;

		case aw8ec_KEYPAD_KEY_P:
			return KEY_P;

		case aw8ec_KEYPAD_KEY_LEFTBRACE:
			return KEY_LEFTBRACE;

		case aw8ec_KEYPAD_KEY_RIGHTBRACE:
			return KEY_RIGHTBRACE;

		case aw8ec_KEYPAD_KEY_BACKSLASH:
			return KEY_BACKSLASH;

		case aw8ec_KEYPAD_KEY_CAPSLOCK:
			return KEY_CAPSLOCK;

		case aw8ec_KEYPAD_KEY_A:
			return KEY_A;

		case aw8ec_KEYPAD_KEY_S:
			return KEY_S;

		case aw8ec_KEYPAD_KEY_D:
			return KEY_D;

		case aw8ec_KEYPAD_KEY_F:
			return KEY_F;

		case aw8ec_KEYPAD_KEY_G:
			return KEY_G;

		case aw8ec_KEYPAD_KEY_H:
			return KEY_H;

		case aw8ec_KEYPAD_KEY_J:
			return KEY_J;

		case aw8ec_KEYPAD_KEY_K:
			return KEY_K;

		case aw8ec_KEYPAD_KEY_L:
			return KEY_L;

		case aw8ec_KEYPAD_KEY_SEMICOLON:
			return KEY_SEMICOLON;

		case aw8ec_KEYPAD_KEY_APOSTROPHE:
			return KEY_APOSTROPHE;

		case aw8ec_KEYPAD_KEY_ENTER:
			return KEY_ENTER;

		case aw8ec_KEYPAD_KEY_LEFTSHIFT:
			return KEY_LEFTSHIFT;

		case aw8ec_KEYPAD_KEY_Z:
			return KEY_Z;

		case aw8ec_KEYPAD_KEY_X:
			return KEY_X;

		case aw8ec_KEYPAD_KEY_C:
			return KEY_C;

		case aw8ec_KEYPAD_KEY_V:
			return KEY_V;

		case aw8ec_KEYPAD_KEY_B:
			return KEY_B;

		case aw8ec_KEYPAD_KEY_N:
			return KEY_N;

		case aw8ec_KEYPAD_KEY_M:
			return KEY_M;

		case aw8ec_KEYPAD_KEY_COMMA:
			return KEY_COMMA;

		case aw8ec_KEYPAD_KEY_DOT:
			return KEY_DOT;

		case aw8ec_KEYPAD_KEY_SLASH:
			return KEY_SLASH;

		case aw8ec_KEYPAD_KEY_RIGHTSHIFT:
			return KEY_RIGHTSHIFT;

		case aw8ec_KEYPAD_KEY_LEFT:
			return KEY_LEFT;

		case aw8ec_KEYPAD_KEY_RIGHT:
			return KEY_RIGHT;

		case aw8ec_KEYPAD_KEY_UP:
			return KEY_UP;

		case aw8ec_KEYPAD_KEY_DOWN:
			return KEY_DOWN;

		case aw8ec_KEYPAD_RIGHTWIN:
			return KEY_SEARCH;

		case aw8ec_KEYPAD_LEFTCTRL:
			return KEY_LEFTCTRL;

		case aw8ec_KEYPAD_LEFTWIN:
			return KEY_HOMEPAGE;

		case aw8ec_KEYPAD_LEFTALT:
			return KEY_LEFTALT;

		case aw8ec_KEYPAD_KEY_SPACE:
			return KEY_SPACE;

		case aw8ec_KEYPAD_RIGHTALT:
			return KEY_RIGHTALT;

		case aw8ec_KEYPAD_WINAPP:
			return KEY_MENU;

		case aw8ec_KEYPAD_RIGHTCTRL:
			return KEY_RIGHTCTRL;

		case aw8ec_KEYPAD_HOME:
			return KEY_HOME;

		case aw8ec_KEYPAD_PAGEUP:
			return KEY_PAGEUP;

		case aw8ec_KEYPAD_PAGEDOWN:
			return KEY_PAGEDOWN;

		case aw8ec_KEYPAD_END:
			return KEY_END;

		//--- JP keys
		case aw8ec_YEN:
			return KEY_YEN;

		case aw8ec_RO:
			return KEY_RO;

		case aw8ec_MUHENKAN:
			return KEY_MUHENKAN;

		case aw8ec_HENKAN:
			return KEY_HENKAN;

		case aw8ec_HIRAGANA_KATAKANA:
			return KEY_KATAKANAHIRAGANA;

		//--- UK keys
		case aw8ec_EUROPE_2:
			return KEY_102ND;

		default:
			return -1;
	}
}

static void aw8ec_reset_counter(unsigned long data){
	ec_chip->d_index = 0;
}

static int aw8ec_tp_control(int arg){

	int ret_val = 0;

	if(arg == aw8ec_TP_ON){
		if (ec_chip->tp_enable == 0){
			ec_chip->tp_wait_ack = 1;
			ec_chip->tp_enable = 1;
			aw8ec_i2c_write_data(ec_chip->client, 0xF4D4);
			ec_chip->d_index = 0;
		}
		if (ec_chip->touchpad_member == -1){
			ec_chip->susb_on = 1;
			ec_chip->init_success = -1;
			aw8ec_reset_dock();
		}
		ret_val = 0;
	} else if (arg == aw8ec_TP_OFF){
		ec_chip->tp_wait_ack = 1;
		ec_chip->tp_enable = 0;
		aw8ec_i2c_write_data(ec_chip->client, 0xF5D4);
		ec_chip->d_index = 0;
		ret_val = 0;
	} else
		ret_val = -ENOTTY;

	return ret_val;

}
#if (!TOUCHPAD_MODE)
static void aw8ec_tp_rel(void){

	ec_chip->touchpad_data.x_sign = (ec_chip->ec_data[0] & X_SIGN_MASK) ? 1:0;
	ec_chip->touchpad_data.y_sign = (ec_chip->ec_data[0] & Y_SIGN_MASK) ? 1:0;
	ec_chip->touchpad_data.left_btn = (ec_chip->ec_data[0] & LEFT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.right_btn = (ec_chip->ec_data[0] & RIGHT_BTN_MASK) ? 1:0;
	ec_chip->touchpad_data.delta_x =
		(ec_chip->touchpad_data.x_sign) ? (ec_chip->ec_data[1] - 0xff):ec_chip->ec_data[1];
	ec_chip->touchpad_data.delta_y =
		(ec_chip->touchpad_data.y_sign) ? (ec_chip->ec_data[2] - 0xff):ec_chip->ec_data[2];

	input_report_rel(ec_chip->indev, REL_X, ec_chip->touchpad_data.delta_x);
	input_report_rel(ec_chip->indev, REL_Y, (-1) * ec_chip->touchpad_data.delta_y);
	input_report_key(ec_chip->indev, BTN_LEFT, ec_chip->touchpad_data.left_btn);
	input_report_key(ec_chip->indev, KEY_BACK, ec_chip->touchpad_data.right_btn);
	input_sync(ec_chip->indev);

}
#endif

#if TOUCHPAD_MODE
static void aw8ec_tp_abs(void){
	unsigned char SA1,A1,B1,SB1,C1,D1;
	static unsigned char SA1_O=0,A1_O=0,B1_O=0,SB1_O=0,C1_O=0,D1_O=0;
	static int Null_data_times = 0;

	if ((ec_chip->tp_enable) && (ec_chip->touchpad_member == ELANTOUCHPAD)){
		SA1= ec_chip->ec_data[0];
		A1 = ec_chip->ec_data[1];
		B1 = ec_chip->ec_data[2];
		SB1= ec_chip->ec_data[3];
		C1 = ec_chip->ec_data[4];
		D1 = ec_chip->ec_data[5];
		aw8ec_INFO("SA1=0x%x A1=0x%x B1=0x%x SB1=0x%x C1=0x%x D1=0x%x \n",SA1,A1,B1,SB1,C1,D1);
		if ( (SA1 == 0xC4) && (A1 == 0xFF) && (B1 == 0xFF) &&
		     (SB1 == 0x02) && (C1 == 0xFF) && (D1 == 0xFF)){
			Null_data_times ++;
			goto aw8ec_tp_abs_end;
		}

		if(!(SA1 == SA1_O && A1 == A1_O && B1 == B1_O &&
		   SB1 == SB1_O && C1 == C1_O && D1 == D1_O)) {
			elantech_report_absolute_to_related(ec_chip, &Null_data_times);
		}

aw8ec_tp_abs_end:
		SA1_O = SA1;
		A1_O = A1;
		B1_O = B1;
		SB1_O = SB1;
		C1_O = C1;
		D1_O = D1;
	} else if (ec_chip->touchpad_member == -1){
		ec_chip->susb_on = 1;
		ec_chip->init_success = -1;
		aw8ec_reset_dock();
	}
}
#endif

static void aw8ec_touchpad_processing(void){
	int i;
	int length = 0;
	int tp_start = 0;
	aw8ec_I2C_DATA(ec_chip->i2c_data,ec_chip->index);

#if TOUCHPAD_MODE
	length = ec_chip->i2c_data[0];
	if (ec_chip->tp_wait_ack){
		ec_chip->tp_wait_ack = 0;
		tp_start = 1;
		ec_chip->d_index = 0;
	} else {
		tp_start = 0;
	}

	for( i = tp_start; i < length - 1 ; i++){
		ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+2];
		ec_chip->d_index++;
		if (ec_chip->d_index == 6){
			aw8ec_tp_abs();
			ec_chip->d_index = 0;
		}
	}


	if (ec_chip->d_index)
		mod_timer(&ec_chip->aw8ec_timer,jiffies+(HZ * 1/20));
#else
	length = ec_chip->i2c_data[0];
	for( i = 0; i < length -1 ; i++){
		ec_chip->ec_data[ec_chip->d_index] = ec_chip->i2c_data[i+2];
		ec_chip->d_index++;
		if (ec_chip->d_index == 3){
			aw8ec_tp_rel();
			ec_chip->d_index = 0;
		}
	}
#endif
}

static void aw8ec_kp_wake(void){
	aw8ec_NOTICE("ASUSEC WAKE\n");
	if (aw8ec_input_device_create(ec_chip->client)){
		return ;
	}
	input_report_key(ec_chip->indev, KEY_MENU, 1);
	input_sync(ec_chip->indev);
	input_report_key(ec_chip->indev, KEY_MENU, 0);
	input_sync(ec_chip->indev);
}

static void aw8ec_kp_smi(void){
	if (ec_chip->i2c_data[2] == aw8ec_SMI_HANDSHAKING){
		aw8ec_NOTICE("aw8ec_SMI_HANDSHAKING\n");
		ec_chip->ec_in_s3 = 0;
		p1801isDockIn = 1;
		if (ec_chip->susb_on){
			aw8ec_chip_init(ec_chip->client);
		}
	} else if (ec_chip->i2c_data[2] == aw8ec_SMI_RESET){
		aw8ec_NOTICE("aw8ec_SMI_RESET\n");
		ec_chip->init_success = 0;
		aw8ec_dock_init_work_function(NULL);
	} else if (ec_chip->i2c_data[2] == aw8ec_SMI_WAKE){
		aw8ec_kp_wake();
		aw8ec_NOTICE("aw8ec_SMI_WAKE\n");
	} else if (ec_chip->i2c_data[2] == aw8ec_SMI_ADAPTER_EVENT){
		aw8ec_NOTICE("aw8ec_SMI_ADAPTER_EVENT\n");
#if DOCK_USB
		fsl_dock_ec_callback();
#endif
	} else if (ec_chip->i2c_data[2] == aw8ec_SMI_BACKLIGHT_ON){
		aw8ec_NOTICE("aw8ec_SMI_BACKLIGHT_ON\n");
		ec_chip->susb_on = 1;
		aw8ec_reset_dock();
	}
}

static void aw8ec_kp_kbc(void){
	if (ec_chip->i2c_data[2] == aw8ec_PS2_ACK){
		if (ec_chip->kbc_value == 0){
			aw8ec_INFO("send led cmd 2\n");
			aw8ec_i2c_write_data(ec_chip->client, 0x0000);
		} else {
			aw8ec_INFO("send led cmd 2\n");
			aw8ec_i2c_write_data(ec_chip->client, 0x0400);
		}
	}
}
static void aw8ec_kp_sci(void){
	int ec_signal = ec_chip->i2c_data[2];

	ec_chip->keypad_data.input_keycode = aw8ec_kp_sci_table[ec_signal];
	if(ec_chip->keypad_data.input_keycode > 0){
		aw8ec_INFO("input_keycode = 0x%x\n", ec_chip->keypad_data.input_keycode);

		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 1);
		input_sync(ec_chip->indev);
		input_report_key(ec_chip->indev, ec_chip->keypad_data.input_keycode, 0);
		input_sync(ec_chip->indev);

	}else{
		aw8ec_INFO("Unknown ec_signal = 0x%x\n", ec_signal);
	}
}
static void aw8ec_kp_key(void){
	int scancode = 0;

	if (ec_chip->i2c_data[2] == aw8ec_KEYPAD_KEY_EXTEND){
		ec_chip->keypad_data.extend = 1;
		ec_chip->bc = 3;
	}else{
		ec_chip->keypad_data.extend = 0;
		ec_chip->bc = 2;
	}
	if(ec_chip->i2c_data[ec_chip->bc] == aw8ec_KEYPAD_KEY_BREAK){
		ec_chip->keypad_data.value = 0;
		ec_chip->bc++;
	}else{
		ec_chip->keypad_data.value = 1;
	}

	if (ec_chip->keypad_data.extend == 1){
		scancode = ((aw8ec_KEYPAD_KEY_EXTEND << 8) | ec_chip->i2c_data[ec_chip->bc]);
	} else {
		scancode = ec_chip->i2c_data[ec_chip->bc];
	}
	if (ec_chip->i2c_data[0] == 6){
		if ((ec_chip->i2c_data[2] == 0xE0) &&
			(ec_chip->i2c_data[3] == 0xF0) &&
			(ec_chip->i2c_data[4] == 0x12)){
			scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
			ec_chip->keypad_data.value = 1;
		}
		else if ((ec_chip->i2c_data[2] == 0xE0) &&
			(ec_chip->i2c_data[3] == 0xF0) &&
			(ec_chip->i2c_data[4] == 0x59)){
			scancode = ec_chip->i2c_data[5] << 8 | ec_chip->i2c_data[6];
			ec_chip->keypad_data.value = 1;
		}
	}
	aw8ec_INFO("scancode = 0x%x\n", scancode);
	ec_chip->keypad_data.input_keycode = aw8ec_kp_key_mapping(scancode);
	if(ec_chip->keypad_data.input_keycode > 0){
		aw8ec_INFO("input_keycode = 0x%x, input_value = %d\n",
				ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);

		input_report_key(ec_chip->indev,
			ec_chip->keypad_data.input_keycode, ec_chip->keypad_data.value);
		input_sync(ec_chip->indev);

	}else{
		aw8ec_INFO("Unknown scancode = 0x%x\n", scancode);
	}

}

static void aw8ec_keypad_processing(void){

	aw8ec_I2C_DATA(ec_chip->i2c_data,ec_chip->index);
	if (ec_chip->i2c_data[1] & aw8ec_KBC_MASK)
		aw8ec_kp_kbc();
	else if (ec_chip->i2c_data[1] & aw8ec_SCI_MASK)
		aw8ec_kp_sci();
	else
		aw8ec_kp_key();
}

static void aw8ec_dock_status_report(void){
	aw8ec_INFO("dock_in = %d\n", ec_chip->dock_in);
	printk("aw8ec_dock_status_report sucessed\n");
	switch_set_state(&ec_chip->dock_sdev, gpio_get_value(aw8ec_dock_in_gpio) ? 0 : 13);
#if BATTERY_DRIVER
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_pad_battery_report_work, 0);
#endif
#if DOCK_SPEAKER
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_audio_report_work, 0);
#endif
}

static void aw8ec_w8_status_report(void){
	aw8ec_INFO("w8_on = %d\n", ec_chip->w8_on);
	printk("aw8ec_w8_status_report sucessed\n");
	switch_set_state(&ec_chip->w8_sdev, gpio_get_value(aw8ec_w8_gpio) ? 0 : 1);
}

static void aw8ec_scalar_status_report(void){
	aw8ec_INFO("dispaly win8 = %d\n", ec_chip->scalar_status);
	switch_set_state(&ec_chip->scalar_status_sdev, gpio_get_value(aw8ec_scalar_status_gpio) ? 1 : 0);
}

static int aw8ec_get_version_num(void){
	int i;
	int v_num = 0;
	int v_len = strlen(ec_chip->ec_version);
	char *v_str = ec_chip->ec_version;

	if (ec_chip->tf_dock){
		for ( i = v_len - 4; i < v_len; i++)
			v_num = v_num * 10 + v_str[i] - '0';
	}
	aw8ec_INFO("v_num = %d\n", v_num);
	return v_num ;
}
#if BATTERY_DRIVER
static void aw8ec_pad_battery_report_function(struct work_struct *dat)
{
	int ret_val = 0;
	int dock_in = ec_chip->dock_in;

	ret_val = docking_callback(dock_in);
	aw8ec_NOTICE("dock_in = %d, ret_val = %d\n", dock_in, ret_val);
	if (ret_val < 0)
		queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_pad_battery_report_work, 2*HZ);
}
#endif

#if DOCK_SPEAKER
static void aw8ec_audio_report_function(struct work_struct *dat)
{
	int ret_val = 0;
	int dock_in = ec_chip->dock_in;
	int dock_speaker = (!strncmp(ec_chip->ec_version, "TF201-", 6));

	ret_val = audio_dock_event(dock_in && dock_speaker);
	aw8ec_NOTICE("dock_in = %d, dock_speaker = %d, ret_val = %d\n", dock_in, dock_speaker, ret_val);
	if (ret_val < 0)
		queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_audio_report_work, 2*HZ);
}
#endif

static void aw8ec_lid_report_function(struct work_struct *dat)
{
	int value = 0;

	if (ec_chip->lid_indev == NULL){
		aw8ec_ERR("LID input device doesn't exist\n");
		return;
	}
	msleep(CONVERSION_TIME_MS);
	value = gpio_get_value(aw8ec_hall_sensor_gpio);
	input_report_switch(ec_chip->lid_indev, SW_LID, !value);
	input_sync(ec_chip->lid_indev);
	aw8ec_NOTICE("SW_LID report value = %d\n", !value);
}

static void aw8ec_stresstest_work_function(struct work_struct *dat)
{
	aw8ec_i2c_read_data(ec_chip->client);
	if (ec_chip->i2c_data[1] & aw8ec_OBF_MASK){
		if (ec_chip->i2c_data[1] & aw8ec_AUX_MASK){
			aw8ec_touchpad_processing();
		}else{
			aw8ec_keypad_processing();
		}
	}

	queue_delayed_work(aw8ec_wq, &aw8ec_stress_work, HZ/ec_chip->polling_rate);
}

static void aw8ec_dock_init_work_function(struct work_struct *dat)
{
	int gpio = aw8ec_dock_in_gpio;
	int irq = gpio_to_irq(gpio);
	int i = 0;
	int d_counter = 0;
	int gpio_state = 0;

	aw8ec_INFO("Dock-init function\n");
	wake_lock(&ec_chip->wake_lock_init);
	if (201){
		aw8ec_NOTICE("TF201 dock-init\n");
		if (ec_chip->dock_det){
			gpio_state = gpio_get_value(gpio);
			for(i = 0; i < 40; i++){
				msleep(50);
				if (gpio_state == gpio_get_value(gpio)){
					d_counter++;
				} else {
					gpio_state = gpio_get_value(gpio);
					d_counter = 0;
				}
				if (d_counter > 4){
					break;
				}
			}
			ec_chip->dock_det--;
			ec_chip->re_init = 0;
		}

		mutex_lock(&ec_chip->input_lock);
		if (gpio_get_value(gpio)){
			aw8ec_NOTICE("No dock detected\n");
			ec_chip->dock_in = 0;
			p1801isDockIn = 0;
			ec_chip->init_success = 0;
			ec_chip->tp_enable = 1;
			ec_chip->tf_dock = 0;
			ec_chip->op_mode = 0;
			ec_chip->dock_behavior = 0;
			memset(ec_chip->ec_model_name, 0, 32);
			memset(ec_chip->ec_version, 0, 32);
			ec_chip->touchpad_member = -1;
			if (ec_chip->indev){
				input_unregister_device(ec_chip->indev);
				ec_chip->indev = NULL;
			}
			if (ec_chip->private->abs_dev){
				input_unregister_device(ec_chip->private->abs_dev);
				ec_chip->private->abs_dev = NULL;
			}
			aw8ec_dock_status_report();
		} else {
			aw8ec_NOTICE("Dock-in detected\n");
                    aw8ec_dock_status_report();
			if (gpio_get_value(aw8ec_hall_sensor_gpio) || (!ec_chip->status)){
				if (ec_chip->init_success == 0){
					if ((!ec_chip->tf_dock) || (!ec_chip->dock_behavior)){
						ec_chip->susb_on = 1;
						msleep(200);
						aw8ec_reset_dock();
					}
				}
			} else {
				aw8ec_NOTICE("Keyboard is closed\n");
			}
		}
		mutex_unlock(&ec_chip->input_lock);
	}
	wake_unlock(&ec_chip->wake_lock_init);
}

static void aw8ec_fw_update_work_function(struct work_struct *dat)
{
	int smbus_data;
	int gpio = aw8ec_apwake_gpio;
	int irq = gpio_to_irq(gpio);

	mutex_lock(&ec_chip->lock);
	smbus_data = i2c_smbus_read_byte_data(&dockram_client, 0);
	enable_irq(irq);
	BuffPush(smbus_data);
	mutex_unlock(&ec_chip->lock);
}


static void aw8ec_w8_report_function(struct work_struct *dat)
{
     /*
        int smbus_data_p1801;
        int gpio = aw8ec_w8_gpio;
        int irq = gpio_to_irq(gpio);

        mutex_lock(&ec_chip->lock);
        smbus_data_p1801 = i2c_smbus_read_byte_data(&dockram_client, 0);
        enable_irq(irq);
        BuffPush(smbus_data_p1801);
        mutex_unlock(&ec_chip->lock);
        */

        int gpio = aw8ec_w8_gpio;
	int irq = gpio_to_irq(gpio);

	aw8ec_INFO("w8-init function\n");
	wake_lock(&ec_chip->wake_lock_init);

		mutex_lock(&ec_chip->input_lock);
		if (gpio_get_value(gpio)){
			aw8ec_NOTICE("w8 off\n");
			ec_chip->w8_on = 0;
			aw8ec_w8_status_report();
			if(w8_lock_state){
				w8_lock_state=0;
				wake_unlock(&ec_chip->wake_lock_w8);
				printk(KERN_INFO "aw8ec wake_unlock\n");
			}

		} else {
			w8_lock_state=1;
			aw8ec_NOTICE("w8 on\n");
			aw8ec_w8_status_report();
			wake_lock(&ec_chip->wake_lock_w8);
		}
		mutex_unlock(&ec_chip->input_lock);
		wake_unlock(&ec_chip->wake_lock_init);

}

static void aw8ec_scalar_status_report_function(struct work_struct *dat)
{
	int gpio = aw8ec_scalar_status_gpio;
	int irq = gpio_to_irq(gpio);

	aw8ec_INFO("scalar-init function\n");
	wake_lock(&ec_chip->wake_lock_init);

		mutex_lock(&ec_chip->input_lock);
		if (gpio_get_value(gpio)){
			aw8ec_NOTICE("dispaly win8\n");
			ec_chip->scalar_status = 1;
			aw8ec_scalar_status_report();
		} else {
			aw8ec_NOTICE("display PAD on\n");
			aw8ec_scalar_status_report();
		}
		mutex_unlock(&ec_chip->input_lock);
		wake_unlock(&ec_chip->wake_lock_init);
}

static void aw8ec_work_function(struct work_struct *dat)
{
	int gpio = aw8ec_apwake_gpio;
	int irq = gpio_to_irq(gpio);
	int ret_val = 0;

	ec_chip->dock_in = gpio_get_value(aw8ec_dock_in_gpio) ? 0 : 1;

	if (ec_chip->wakeup_lcd){
		if (gpio_get_value(aw8ec_hall_sensor_gpio)){
			ec_chip->wakeup_lcd = 0;
			wake_lock_timeout(&ec_chip->wake_lock, 3*HZ);
			msleep(500);
		}
	}

	ret_val = aw8ec_i2c_read_data(ec_chip->client);
	enable_irq(irq);

	if (ret_val < 0){
		return ;
	}

	if (ec_chip->i2c_data[1] & aw8ec_OBF_MASK){
		if (ec_chip->i2c_data[1] & aw8ec_SMI_MASK){
			aw8ec_kp_smi();
			return ;
		}
	}

	mutex_lock(&ec_chip->input_lock);
	if (ec_chip->indev == NULL){
		mutex_unlock(&ec_chip->input_lock);
		return;
	}
	if (ec_chip->i2c_data[1] & aw8ec_OBF_MASK){
		if (ec_chip->i2c_data[1] & aw8ec_AUX_MASK){
			if (ec_chip->private->abs_dev)
				aw8ec_touchpad_processing();
		}else{
			aw8ec_keypad_processing();
		}
	}
	mutex_unlock(&ec_chip->input_lock);
}

static void aw8ec_keypad_set_input_params(struct input_dev *dev)
{
	int i = 0;
	set_bit(EV_KEY, dev->evbit);
	for ( i = 0; i < 246; i++)
		set_bit(i,dev->keybit);

	input_set_capability(dev, EV_LED, LED_CAPSL);
}

static void aw8ec_lid_set_input_params(struct input_dev *dev)
{
	set_bit(EV_SW, dev->evbit);
	set_bit(SW_LID, dev->swbit);
}

static int aw8ec_input_device_create(struct i2c_client *client){
	int err = 0;

	if (ec_chip->indev){
		return 0;
	}
	ec_chip->indev = input_allocate_device();
	if (!ec_chip->indev) {
		aw8ec_ERR("input_dev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->indev->name = "aw8ec";
	ec_chip->indev->phys = "/dev/input/aw8ec";
	ec_chip->indev->dev.parent = &client->dev;
	ec_chip->indev->event = aw8ec_event;

	aw8ec_keypad_set_input_params(ec_chip->indev);
	err = input_register_device(ec_chip->indev);
	if (err) {
		aw8ec_ERR("input registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->indev);
	ec_chip->indev = NULL;
exit:
	return err;

}

static int aw8ec_lid_input_device_create(struct i2c_client *client){
	int err = 0;

	ec_chip->lid_indev = input_allocate_device();
	if (!ec_chip->lid_indev) {
		aw8ec_ERR("lid_indev allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	ec_chip->lid_indev->name = "lid_input";
	ec_chip->lid_indev->phys = "/dev/input/lid_indev";
	ec_chip->lid_indev->dev.parent = &client->dev;

	aw8ec_lid_set_input_params(ec_chip->lid_indev);
	err = input_register_device(ec_chip->lid_indev);
	if (err) {
		aw8ec_ERR("lid_indev registration fails\n");
		goto exit_input_free;
	}
	return 0;

exit_input_free:
	input_free_device(ec_chip->lid_indev);
	ec_chip->lid_indev = NULL;
exit:
	return err;

}

static int __devinit aw8ec_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err = 0;
	aw8ec_INFO("aw8ec probe\n");
	err = sysfs_create_group(&client->dev.kobj, &aw8ec_smbus_group);
	if (err) {
		aw8ec_ERR("Unable to create the sysfs\n");
		goto exit;
	}

	ec_chip = kzalloc(sizeof (struct aw8ec_chip), GFP_KERNEL);
	if (!ec_chip) {
		aw8ec_ERR("Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}
	ec_chip->private = kzalloc(sizeof(struct elantech_data), GFP_KERNEL);
	if (!ec_chip->private) {
		aw8ec_ERR("Memory allocation (elantech_data) fails\n");
		err = -ENOMEM;
		goto exit;
	}

	i2c_set_clientdata(client, ec_chip);
	ec_chip->client = client;
	ec_chip->client->driver = &aw8ec_driver;
	ec_chip->client->flags = 1;

	mutex_init(&ec_chip->lock);
	mutex_init(&ec_chip->kbc_lock);
	mutex_init(&ec_chip->input_lock);
	mutex_init(&ec_chip->dock_init_lock);

	init_timer(&ec_chip->aw8ec_timer);
	ec_chip->aw8ec_timer.function = aw8ec_reset_counter;

	wake_lock_init(&ec_chip->wake_lock, WAKE_LOCK_SUSPEND, "aw8ec_wake");
	wake_lock_init(&ec_chip->wake_lock_init, WAKE_LOCK_SUSPEND, "aw8ec_wake_init");
	wake_lock_init(&ec_chip->wake_lock_w8, WAKE_LOCK_SUSPEND, "aw8ec_wake_w8");

	ec_chip->status = 0;
	ec_chip->dock_det = 0;
	ec_chip->dock_in = 0;
	ec_chip->w8_on= 0;  //ne
	ec_chip->scalar_status= 0;
	ec_chip->dock_init = 0;
	ec_chip->d_index = 0;
	ec_chip->suspend_state = 0;
	ec_chip->init_success = 0;
	ec_chip->wakeup_lcd = 0;
	ec_chip->tp_wait_ack = 0;
	ec_chip->tp_enable = 1;
	ec_chip->re_init = 0;
	ec_chip->ec_wakeup = 0;
	ec_chip->dock_behavior = 0;
	ec_chip->ec_in_s3 = 1;
	ec_chip->susb_on = 1;
	ec_chip->indev = NULL;
	ec_chip->lid_indev = NULL;
	ec_chip->private->abs_dev = NULL;
	aw8ec_dockram_init(client);

	cdev_add(aw8ec_cdev,aw8ec_dev,1) ;

	ec_chip->dock_sdev.name = DOCK_SDEV_NAME;
	ec_chip->dock_sdev.print_name = aw8ec_switch_name;
	ec_chip->dock_sdev.print_state = aw8ec_switch_state;

	ec_chip->w8_sdev.name = W8_SDEV_NAME;
	ec_chip->w8_sdev.print_name = aw8ec_aio_switch_name;
	ec_chip->w8_sdev.print_state = aw8ec_aio_switch_state;

	ec_chip->scalar_status_sdev.name = SCALAR_SDEV_NAME;
	ec_chip->scalar_status_sdev.print_name = aw8ec_scalar_switch_name;
	ec_chip->scalar_status_sdev.print_state = aw8ec_scalar_switch_state;

	if(switch_dev_register(&ec_chip->dock_sdev) < 0){
		aw8ec_ERR("switch_dev_register for dock failed!\n");
		goto exit;
	}

	if(switch_dev_register(&ec_chip->w8_sdev) < 0){
		aw8ec_ERR("switch_w8dec_register for dock failed!\n");
		goto exit;
	}

	if(switch_dev_register(&ec_chip->scalar_status_sdev) < 0){
		aw8ec_ERR("switch_scalar_status__register for P1801 failed!\n");
		goto exit;
	}
	switch_set_state(&ec_chip->dock_sdev, 0);
	switch_set_state(&ec_chip->w8_sdev, 0);
	switch_set_state(&ec_chip->scalar_status_sdev, 0);

	err = power_supply_register(&client->dev, &aw8ec_power_supply[0]);
	if (err){
		aw8ec_ERR("fail to register power supply for dock\n");
		goto exit;
	}

	aw8ec_lid_input_device_create(ec_chip->client);
	aw8ec_wq = create_singlethread_workqueue("aw8ec_wq");
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_w8_work, aw8ec_w8_report_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_scalar_status_work, aw8ec_scalar_status_report_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_hall_sensor_work, aw8ec_lid_report_function);
#if BATTERY_DRIVER
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_pad_battery_report_work, aw8ec_pad_battery_report_function);
#endif
#if DOCK_SPEAKER
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_audio_report_work, aw8ec_audio_report_function);
#endif
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_work, aw8ec_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_dock_init_work, aw8ec_dock_init_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_fw_update_work, aw8ec_fw_update_work_function);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_led_on_work, aw8ec_keypad_led_on);
	INIT_DELAYED_WORK_DEFERRABLE(&ec_chip->aw8ec_led_off_work, aw8ec_keypad_led_off);
	INIT_DELAYED_WORK_DEFERRABLE(&aw8ec_stress_work, aw8ec_stresstest_work_function);

	aw8ec_irq_dock_in(client);
	aw8ec_irq_ec_request(client);
	aw8ec_irq_hall_sensor(client);
	aw8ec_irq(client);
	aw8ec_irq_splash_request(client);
	aw8ec_irq_w8_in(client);
	aw8ec_irq_scalar_status(client);

	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_dock_init_work, 0);
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_hall_sensor_work, 0);
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_w8_work, 0);
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_scalar_status_work, 0);

	return 0;

exit:
	return err;
}

static int __devexit aw8ec_remove(struct i2c_client *client)
{
	struct aw8ec_chip *chip = i2c_get_clientdata(client);

	dev_dbg(&client->dev, "%s()\n", __func__);
	input_unregister_device(chip->indev);
	kfree(chip);
	return 0;
}

static ssize_t aw8ec_info_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t aw8ec_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;
	ret_val = aw8ec_i2c_test(ec_chip->client);
	if(ret_val >= 0){
		return sprintf(buf, "1\n");
	} else {
		return sprintf(buf, "0\n");
	}
}

static ssize_t aw8ec_tp_status_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", (ec_chip->touchpad_member == ELANTOUCHPAD));
}

static ssize_t aw8ec_show_dock(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "dock detect = %d\n", ec_chip->dock_in);
}

static ssize_t aw8ec_store_led(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if (buf[0] == '0')
			queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_led_off_work, 0);
		else
			queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_led_on_work, 0);
	}

	return 0 ;
}

static ssize_t aw8ec_charging_led_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		aw8ec_dockram_read_data(0x0A);
		if (buf[0] == '0'){
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ret_val = aw8ec_dockram_write_data(0x0A,9);
			if (ret_val < 0)
				aw8ec_NOTICE("Fail to diable led test\n");
			else
				aw8ec_NOTICE("Diable led test\n");
		} else if (buf[0] == '1'){
			aw8ec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] | 0x02;
			ret_val = aw8ec_dockram_write_data(0x0A,9);
			if (ret_val < 0)
				aw8ec_NOTICE("Fail to enable orange led test\n");
			else
				aw8ec_NOTICE("Enable orange led test\n");
		} else if (buf[0] == '2'){
			aw8ec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] & 0xF9;
			ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] | 0x04;
			ret_val = aw8ec_dockram_write_data(0x0A,9);
			if (ret_val < 0)
				aw8ec_NOTICE("Fail to enable green led test\n");
			else
				aw8ec_NOTICE("Enable green led test\n");
		}
	} else {
		aw8ec_NOTICE("Fail to enter led test\n");
	}

	return count;
}


static ssize_t aw8ec_led_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret_val = 0;

	aw8ec_dockram_read_data(0x0A);
	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[6] = ec_chip->i2c_dm_data[6] | 0x01;
	ret_val = aw8ec_dockram_write_data(0x0A,9);
	if (ret_val < 0)
		return sprintf(buf, "Fail to EC LED Blink\n");
	else
		return sprintf(buf, "EC LED Blink\n");
}

static ssize_t aw8ec_store_ec_wakeup(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	if (buf[0] == '0'){
		ec_chip->ec_wakeup = 0;
		aw8ec_NOTICE("Set EC shutdown when PAD in LP0\n");
	}
	else{
		ec_chip->ec_wakeup = 1;
		aw8ec_NOTICE("Keep EC active when PAD in LP0\n");
	}

	return 0 ;
}

static ssize_t aw8ec_show_drain(struct device *class,struct device_attribute *attr,char *buf)
{
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		aw8ec_dockram_read_data(0x0A);

		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x8;
		aw8ec_dockram_write_data(0x0A,9);
		aw8ec_NOTICE("discharging 15 seconds\n");
		return sprintf(buf, "discharging 15 seconds\n");
	}

	return 0;
}

static ssize_t aw8ec_show_dock_battery(struct device *class,struct device_attribute *attr,char *buf)
{
	int bat_percentage = 0;
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = aw8ec_dockram_read_data(0x14);

		if (ret_val < 0)
			return sprintf(buf, "-1\n");
		else{
			bat_percentage = (ec_chip->i2c_dm_data[14] << 8 )| ec_chip->i2c_dm_data[13];
			return sprintf(buf, "%d\n", bat_percentage);
		}
	}

	return sprintf(buf, "-1\n");
}

static ssize_t aw8ec_show_dock_battery_status(struct device *class,struct device_attribute *attr,char *buf)
{
	int bat_percentage = 0;
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if (ec_chip->ec_in_s3 && ec_chip->status){
			msleep(200);
		}

		ret_val = aw8ec_dockram_read_data(0x0A);

		if (ret_val < 0){
			return sprintf(buf, "-1\n");
		}
		else {
			if (ec_chip->i2c_dm_data[1] & 0x4)
				return sprintf(buf, "1\n");
			else
				return sprintf(buf, "0\n");;
		}
	}
	return sprintf(buf, "-1\n");
}


static ssize_t aw8ec_show_dock_battery_all(struct device *class,struct device_attribute *attr,char *buf)
{
	int i = 0;
	char temp_buf[64];
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = aw8ec_dockram_read_data(0x14);

		if (ret_val < 0)
			return sprintf(buf, "fail to get dock-battery info\n");
		else{
			sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
			strcpy(buf, temp_buf);
			for (i = 1; i < 17; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
			}
			return strlen(buf);
		}
	}

	return sprintf(buf, "fail to get dock-battery info\n");
}

static ssize_t aw8ec_show_dock_control_flag(struct device *class,struct device_attribute *attr,char *buf)
{
	int i = 0;
	char temp_buf[64];
	int ret_val = 0;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		ret_val = aw8ec_dockram_read_data(0x0A);

		if (ret_val < 0)
			return sprintf(buf, "fail to get control-flag info\n");
		else{
			sprintf(temp_buf, "byte[0] = 0x%x\n", ec_chip->i2c_dm_data[i]);
			strcpy(buf, temp_buf);
			for (i = 1; i < 9; i++){
				sprintf(temp_buf, "byte[%d] = 0x%x\n", i, ec_chip->i2c_dm_data[i]);
				strcat(buf, temp_buf);
			}
			return strlen(buf);
		}
	}

	return sprintf(buf, "fail to get control-flag info\n");
}

static ssize_t aw8ec_show_lid_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(aw8ec_hall_sensor_gpio));
}


static ssize_t aw8ec_show_w8_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(aw8ec_w8_gpio));
}

static ssize_t aw8ec_show_scalar_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(aw8ec_scalar_status_gpio));
}

static int aw8ec_suspend(struct i2c_client *client, pm_message_t mesg){
	int ret_val;

	aw8ec_NOTICE("aw8ec_suspend+\n");
	ec_chip->susb_on = 0;
	flush_workqueue(aw8ec_wq);
	if (ec_chip->dock_in && (ec_chip->ec_in_s3 == 0)){
		ret_val = aw8ec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}

		aw8ec_dockram_read_data(0x0A);

		ec_chip->i2c_dm_data[0] = 8;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0xDF;
		ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x22;
		if (ec_chip->ec_wakeup){
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x80;
		} else {
			ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x7F;
		}
		aw8ec_dockram_write_data(0x0A,9);
	}

fail_to_access_ec:
	flush_workqueue(aw8ec_wq);
	ec_chip->suspend_state = 1;
	ec_chip->dock_det = 0;
	ec_chip->init_success = 0;
	ec_chip->ec_in_s3 = 1;
	ec_chip->touchpad_member = -1;
	aw8ec_NOTICE("aw8ec_suspend-\n");
	return 0;
}

static int aw8ec_resume(struct i2c_client *client){

	printk("aw8ec_resume+\n");
	if ((gpio_get_value(aw8ec_dock_in_gpio) == 0) && gpio_get_value(aw8ec_apwake_gpio))
		aw8ec_reset_dock();

	ec_chip->suspend_state = 0;
	//queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_hall_sensor_work, 0);
	aw8ec_lid_report_function(NULL);
	wake_lock(&ec_chip->wake_lock_init);
	ec_chip->init_success = 0;
	queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_dock_init_work, 0);

	printk("aw8ec_resume-\n");
	return 0;
}

static int aw8ec_set_wakeup_cmd(void){
	int ret_val = 0;

	if (ec_chip->dock_in){
		ret_val = aw8ec_i2c_test(ec_chip->client);
		if(ret_val >= 0){
			aw8ec_dockram_read_data(0x0A);
			ec_chip->i2c_dm_data[0] = 8;
			if (ec_chip->ec_wakeup){
				ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x80;
			} else {
				ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] & 0x7F;
			}
			aw8ec_dockram_write_data(0x0A,9);
		}
	}
	return 0;
}
static ssize_t aw8ec_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t aw8ec_switch_state(struct switch_dev *sdev, char *buf)
{

	return sprintf(buf, "%s\n", (gpio_get_value(aw8ec_dock_in_gpio) ? "0" : "13"));
}

//new
static ssize_t aw8ec_aio_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}


//new
static ssize_t aw8ec_aio_switch_state(struct switch_dev *sdev, char *buf)
{

	return sprintf(buf, "%s\n", (gpio_get_value(aw8ec_w8_gpio)? "0" : "1"));

}

static ssize_t aw8ec_scalar_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ec_chip->ec_version);
}

static ssize_t aw8ec_scalar_switch_state(struct switch_dev *sdev, char *buf)
{
       //gpio low:display PAD, high: dispaly win8
	return sprintf(buf, "%s\n", (gpio_get_value(aw8ec_scalar_status_gpio)? "1" : "0"));

}



static int aw8ec_open(struct inode *inode, struct file *flip){
	aw8ec_NOTICE(" ");
	return 0;
}
static int aw8ec_release(struct inode *inode, struct file *flip){
	aw8ec_NOTICE(" ");
	return 0;
}
static long aw8ec_ioctl(struct file *flip,
					unsigned int cmd, unsigned long arg){
	int err = 1;
	char *envp[3];
	char name_buf[64];
	int env_offset = 0;
	int length = 0;
	int ret=0;
	printk(KERN_INFO "%d+ #####splahstop\n", gpio_get_value(aw8ec_splash_gpio));


	if (_IOC_TYPE(cmd) != aw8ec_IOC_MAGIC){
	 return -ENOTTY;
	}
	if (_IOC_NR(cmd) > aw8ec_IOC_MAXNR){
	return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ){
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if (_IOC_DIR(cmd) & _IOC_WRITE){
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
     }
	if (err){
        return -EFAULT;
	}
        printk(KERN_INFO "start switch");
	switch (cmd) {
		case aw8ec_POLLING_DATA:
			if (arg == aw8ec_IOCTL_HEAVY){
				aw8ec_NOTICE("heavy polling\n");
				ec_chip->polling_rate = 80;
				queue_delayed_work(aw8ec_wq, &aw8ec_stress_work, HZ/ec_chip->polling_rate);
                   }
			else if (arg == aw8ec_IOCTL_NORMAL){
				aw8ec_NOTICE("normal polling\n");
				ec_chip->polling_rate = 10;
				queue_delayed_work(aw8ec_wq, &aw8ec_stress_work, HZ/ec_chip->polling_rate);
			}
			else if  (arg == aw8ec_IOCTL_END){
				aw8ec_NOTICE("polling end\n");
			cancel_delayed_work_sync(&aw8ec_stress_work) ;
			}
			else
				return -ENOTTY;
			break;
		case aw8ec_FW_UPDATE:
			if (ec_chip->dock_in){
				aw8ec_NOTICE("aw8ec_FW_UPDATE\n");
				buff_in_ptr = 0;
				buff_out_ptr = 0;
				h2ec_count = 0;
				ec_chip->suspend_state = 0;
				ec_chip->status = 0;
				aw8ec_reset_dock();
				wake_lock_timeout(&ec_chip->wake_lock, 3*60*HZ);
				msleep(3000);
				ec_chip->op_mode = 1;
				ec_chip->i2c_dm_data[0] = 0x02;
				ec_chip->i2c_dm_data[1] = 0x55;
				ec_chip->i2c_dm_data[2] = 0xAA;
				i2c_smbus_write_i2c_block_data(&dockram_client, 0x40, 3, ec_chip->i2c_dm_data);
				ec_chip->init_success = 0;
				ec_chip->dock_behavior = 0;
				ec_chip->tf_dock = 0;
				msleep(1000);
			} else {
				aw8ec_NOTICE("No dock detected\n");
				return -1;
			}
			break;
		case aw8ec_INIT:
			msleep(500);
			ec_chip->status = 0;
			ec_chip->op_mode = 0;
			queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_dock_init_work, 0);
			msleep(2500);
			aw8ec_NOTICE("aw8ec_INIT - EC version: %s\n", ec_chip->ec_version);
			length = strlen(ec_chip->ec_version);
			ec_chip->ec_version[length] = NULL;
			snprintf(name_buf, sizeof(name_buf), "SWITCH_NAME=%s", ec_chip->ec_version);
			envp[env_offset++] = name_buf;
			envp[env_offset] = NULL;
			kobject_uevent_env(&ec_chip->dock_sdev.dev->kobj, KOBJ_CHANGE, envp);
			break;
		case aw8ec_TP_CONTROL:
			aw8ec_NOTICE("aw8ec_TP_CONTROL\n");
			if ((ec_chip->op_mode == 0) && ec_chip->dock_in){
				err = aw8ec_tp_control(arg);
				return err;
			}
			else
				return -ENOTTY;
		case aw8ec_EC_WAKEUP:
			msleep(500);
			aw8ec_NOTICE("aw8ec_EC_WAKEUP, arg = %d\n", arg);
			if (arg == aw8ec_EC_OFF){
				ec_chip->ec_wakeup = 0;
				aw8ec_NOTICE("Set EC shutdown when PAD in LP0\n");
				return aw8ec_set_wakeup_cmd();
			}
			else if (arg == aw8ec_EC_ON){
				ec_chip->ec_wakeup = 1;
				aw8ec_NOTICE("Keep EC active when PAD in LP0\n");
				return aw8ec_set_wakeup_cmd();
			}
			else {
				aw8ec_ERR("Unknown argument");
				return -ENOTTY;
			}
		case aw8ec_FW_DUMMY:
			aw8ec_NOTICE("aw8ec_FW_DUMMY\n");
			ec_chip->i2c_dm_data[0] = 0x02;
			ec_chip->i2c_dm_data[1] = 0x55;
			ec_chip->i2c_dm_data[2] = 0xAA;
			i2c_smbus_write_i2c_block_data(&dockram_client, 0x40, 3, ec_chip->i2c_dm_data);
			return 0;
            case aw8ec_DETECT_SPLASHTOP:
                    aw8ec_NOTICE("aw8ec_DETECT_SPLASHTOP\n");
                    if(arg==aw8ec_SPLASHTOP_ON){
                            gpio_set_value(aw8ec_splash_gpio, 0);
                            msleep(20);
                            ret=gpio_get_value(aw8ec_splash_gpio);
                            if(ret){
                            printk(KERN_INFO"splashtop on failed\n");
                            return -ENOTTY;
                            }
                            else {
                            printk(KERN_INFO"splashtop on sucessed\n");
                            return ret;
                            }
                    }
                     if(arg==aw8ec_SPLASHTOP_OFF){
                            gpio_set_value(aw8ec_splash_gpio, 1);
                            msleep(20);
                            ret=gpio_get_value(aw8ec_splash_gpio);
                            if(ret) {
                            printk(KERN_INFO"splashtop off sucessed\n");
                            return ret;
                            }
                            else {
                            printk(KERN_INFO"splashtop off failed\n");
                            return -ENOTTY;
                            }
                      }
        default:
            return -ENOTTY;
	}
    return 0;
}

static void aw8ec_enter_factory_mode(void){

	aw8ec_NOTICE("Entering factory mode\n");
	aw8ec_dockram_read_data(0x0A);
	ec_chip->i2c_dm_data[0] = 8;
	ec_chip->i2c_dm_data[5] = ec_chip->i2c_dm_data[5] | 0x40;
	aw8ec_dockram_write_data(0x0A,9);
}

static int BuffDataSize(void)
{
    int in = buff_in_ptr;
    int out = buff_out_ptr;

    if (in >= out)
    {
        return (in - out);
    }
    else
    {
        return ((EC_BUFF_LEN - out) + in);
    }
}
static void BuffPush(char data)
{

    if (BuffDataSize() >= (EC_BUFF_LEN -1))
    {
        aw8ec_ERR("Error: EC work-buf overflow \n");
        return;
    }

    ec_to_host_buffer[buff_in_ptr] = data;
    buff_in_ptr++;
    if (buff_in_ptr >= EC_BUFF_LEN)
    {
        buff_in_ptr = 0;
    }
}

static char BuffGet(void)
{
    char c = (char)0;

    if (BuffDataSize() != 0)
    {
        c = (char) ec_to_host_buffer[buff_out_ptr];
        buff_out_ptr++;
         if (buff_out_ptr >= EC_BUFF_LEN)
         {
             buff_out_ptr = 0;
         }
    }
    return c;
}

static ssize_t ec_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int i = 0;
    int ret;
    char tmp_buf[EC_BUFF_LEN];
	static int f_counter = 0;
	static int total_buf = 0;

	mutex_lock(&ec_chip->lock);
	mutex_unlock(&ec_chip->lock);

    while ((BuffDataSize() > 0) && count)
    {
        tmp_buf[i] = BuffGet();
        count--;
        i++;
		f_counter = 0;
		total_buf++;
    }

    ret = copy_to_user(buf, tmp_buf, i);
    if (ret == 0)
    {
        ret = i;
    }


    return ret;
}

static ssize_t ec_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
    int err;
    int i;

    if (h2ec_count > 0)
    {                   /* There is still data in the buffer that */
        return -EBUSY;  /* was not sent to the EC */
    }
    if (count > EC_BUFF_LEN)
    {
        return -EINVAL; /* data size is too big */
    }

    err = copy_from_user(host_to_ec_buffer, buf, count);
    if (err)
    {
        aw8ec_ERR("ec_write copy error\n");
        return err;
    }

    h2ec_count = count;
    for (i = 0; i < count ; i++)
    {
		i2c_smbus_write_byte_data(&dockram_client, host_to_ec_buffer[i],0);
    }
    h2ec_count = 0;
    return count;

}

static int aw8ec_event(struct input_dev *dev, unsigned int type, unsigned int code, int value){
	aw8ec_INFO("type = 0x%x, code = 0x%x, value = 0x%x\n", type, code, value);
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if ((type == EV_LED) && (code == LED_CAPSL)){
			if(value == 0){
				queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_led_off_work, 0);
				return 0;
			} else {
				queue_delayed_work(aw8ec_wq, &ec_chip->aw8ec_led_on_work, 0);
				return 0;
			}
		}
	}
	return -ENOTTY;
}

static int aw8ec_dock_battery_get_capacity(union power_supply_propval *val){
	int bat_percentage = 0;
	int ret_val = 0;

	val->intval = -1;

	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if (ec_chip->ec_in_s3 && ec_chip->status){
			msleep(200);
		}

		ret_val = aw8ec_dockram_read_data(0x14);

		if (ret_val < 0){
			return -1;
		}
		else {
			bat_percentage = (ec_chip->i2c_dm_data[14] << 8 )| ec_chip->i2c_dm_data[13];
			bat_percentage = ((bat_percentage >= 100) ? 100 : bat_percentage);

			if(bat_percentage >70 && bat_percentage <80)
				bat_percentage-=1;
			else if(bat_percentage >60&& bat_percentage <=70)
				bat_percentage-=2;
			else if(bat_percentage >50&& bat_percentage <=60)
				bat_percentage-=3;
			else if(bat_percentage >30&& bat_percentage <=50)
				bat_percentage-=4;
			else if(bat_percentage >=0&& bat_percentage <=30)
				bat_percentage-=5;

			bat_percentage = ((bat_percentage <= 0) ? 0 : bat_percentage);
			val->intval = bat_percentage;
			aw8ec_NOTICE("dock battery level = %d\n", bat_percentage);
			return 0;
		}
	}
	return -1;
}

static int aw8ec_dock_battery_get_status(union power_supply_propval *val){
	int bat_percentage = 0;
	int ret_val = 0;

	val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
	if ((ec_chip->op_mode == 0) && (ec_chip->dock_in)){
		if (ec_chip->ec_in_s3 && ec_chip->status){
			msleep(200);
		}

		ret_val = aw8ec_dockram_read_data(0x0A);

		if (ret_val < 0){
			return -1;
		}
		else {
			if (ec_chip->i2c_dm_data[1] & 0x4)
				val->intval = POWER_SUPPLY_STATUS_CHARGING;
			return 0;
		}
	}
	return -1;
}

static int aw8ec_dock_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp,
	union power_supply_propval *val)
{
	switch (psp) {
		case POWER_SUPPLY_PROP_CAPACITY:
			if(aw8ec_dock_battery_get_capacity(val) < 0)
				goto error;
			break;
		case POWER_SUPPLY_PROP_STATUS:
			if(aw8ec_dock_battery_get_status(val) < 0)
				goto error;
			break;
		default:
			return -EINVAL;
	}
	return 0;

error:
	return -EINVAL;
}
/*
int aw8ec_is_ac_over_10v_callback(void){

	int ret_val;

	aw8ec_NOTICE("access dockram\n");
	if (ec_chip->dock_in){
		msleep(250);
		ret_val = aw8ec_i2c_test(ec_chip->client);
		if(ret_val < 0){
			goto fail_to_access_ec;
		}
		aw8ec_dockram_read_data(0x0A);
		aw8ec_NOTICE("byte[1] = 0x%x\n", ec_chip->i2c_dm_data[1]);

		return ec_chip->i2c_dm_data[1] & 0x20;
	}

fail_to_access_ec:
	aw8ec_NOTICE("dock doesn't exist or fail to access ec\n");
	return -1;
}
EXPORT_SYMBOL(aw8ec_is_ac_over_10v_callback);
*/
static int __init aw8ec_init(void)
{
	int err_code = 0;

	printk(KERN_INFO "%s+ #####3333\n", __func__);
	if (aw8ec_major) {
		aw8ec_dev = MKDEV(aw8ec_major, aw8ec_minor);
		err_code = register_chrdev_region(aw8ec_dev, 1, "aw8ec");
	} else {
		err_code = alloc_chrdev_region(&aw8ec_dev, aw8ec_minor, 1,"aw8ec");
		aw8ec_major = MAJOR(aw8ec_dev);
	}

	aw8ec_NOTICE("cdev_alloc\n") ;
	aw8ec_cdev = cdev_alloc() ;
	aw8ec_cdev->owner = THIS_MODULE ;
	aw8ec_cdev->ops = &aw8ec_fops ;

	err_code=i2c_add_driver(&aw8ec_driver);
	if(err_code){
		aw8ec_ERR("i2c_add_driver fail\n") ;
		goto i2c_add_driver_fail ;
	}
	aw8ec_class = class_create(THIS_MODULE, "aw8ec");
	if(aw8ec_class <= 0){
		aw8ec_ERR("aw8ec_class create fail\n");
		err_code = -1;
		goto class_create_fail ;
	}
	aw8ec_device = device_create(aw8ec_class, NULL, MKDEV(aw8ec_major, aw8ec_minor), NULL, "aw8ec" );
	if(aw8ec_device <= 0){
		aw8ec_ERR("aw8ec_device create fail\n");
		err_code = -1;
		goto device_create_fail ;
	}

	aw8ec_INFO("return value %d\n", err_code) ;
	return 0;

device_create_fail :
	class_destroy(aw8ec_class) ;
class_create_fail :
	i2c_del_driver(&aw8ec_driver);
i2c_add_driver_fail :
	printk(KERN_INFO "%s- #####\n", __func__);
	return err_code;

}

static void __exit aw8ec_exit(void)
{
	device_destroy(aw8ec_class,MKDEV(aw8ec_major, aw8ec_minor)) ;
	class_destroy(aw8ec_class) ;
	i2c_del_driver(&aw8ec_driver);
	unregister_chrdev_region(aw8ec_dev, 1);
	switch_dev_unregister(&ec_chip->dock_sdev);
}

module_init(aw8ec_init);
module_exit(aw8ec_exit);
