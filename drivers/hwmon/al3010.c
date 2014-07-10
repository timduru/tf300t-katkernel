#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>

#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/miscdevice.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/fs.h>

#include <linux/ktime.h>

#include <linux/gpio.h>
#include <../gpio-names.h>
#include <mach/board-cardhu-misc.h>

#define AL3010_DRV_NAME	"al3010"
#define DRIVER_VERSION		"1.0"

#define AL3010_NUM_CACHABLE_REGS	9

#define	AL3010_ALS_COMMAND	4	//ref al3010_reg[AL3010_NUM_CACHABLE_REGS]
#define	AL3010_RAN_MASK	0x70
#define	AL3010_RAN_SHIFT	(4)

#define AL3010_MODE_COMMAND	0	//ref al3010_reg[AL3010_NUM_CACHABLE_REGS]
#define AL3010_MODE_SHIFT	(0)
#define AL3010_MODE_MASK	0x07

#define AL3010_POW_MASK		0x01
#define AL3010_POW_UP		0x01
#define AL3010_POW_DOWN		0x00
#define AL3010_POW_SHIFT	(0)

#define	AL3010_ADC_LSB	0x0c
#define	AL3010_ADC_MSB	0x0d

#define AL3010_IOC_MAGIC 0xF3
#define AL3010_IOC_MAXNR 2
#define AL3010_POLL_DATA _IOR(AL3010_IOC_MAGIC,2,int )

#define AL3010_IOCTL_START_HEAVY 2
#define AL3010_IOCTL_START_NORMAL 1
#define AL3010_IOCTL_END 0

#define START_NORMAL    (HZ)
#define START_HEAVY     (HZ)

#define CAL_ALS_PATH "/data/lightsensor/AL3010_Config.ini"

#define ALS_CAM_POWER_2V85_EN_GPIO TEGRA_GPIO_PR7

bool flagLoadAl3010Config = false;

static int calibration_base_lux = 1000;
static int calibration_regs = 525;	//default calibration ADC value , average 525 from TF201_S_MMI_LS_20110910204452_report.xls
static int default_calibration_regs = 525;

static int poll_mode=0;
struct delayed_work al3010_poll_data_work;
static struct workqueue_struct *sensor_work_queue;
struct i2c_client *al3010_client;

static struct timeval t_first_poll_time;
static bool light_sensor_ready = false;
static bool catch_first_poll_time = false;
static int time_for_sensor_ready = 100; //milisecond

static u8 al3010_reg[AL3010_NUM_CACHABLE_REGS] = 
	{0x00,0x01,0x0c,0x0d,0x10,0x1a,0x1b,0x1c,0x1d};

static int al3010_range[4] = {77806,19452,4863,1216};

struct al3010_data {
	struct i2c_client *client;
	struct mutex lock;
	struct miscdevice misc_dev;
	u8 reg_cache[AL3010_NUM_CACHABLE_REGS];
	u8 power_state_before_suspend;
};

static int al3010_update_calibration();
static int al3010_init_client(struct i2c_client *client);
static void control_als_cam_2v85(int enable);
static int al3010_later_init(struct i2c_client *client);
static int al3010_get_power_state(struct i2c_client *client);
static int al3010_set_power_state(struct i2c_client *client, int state);
static int al3010_get_range(struct i2c_client *client);
static int al3010_set_range(struct i2c_client *client, int range);

static int init_check_count = 10;
static int init_check_max_times = 10;
static int last_revise_lux = -1;
static struct timeval t_first_poll_noise_time;
static bool catch_first_poll_noise_time = false;
static int noise_waiting_time = 1000; //milisecond
static int last_report_lux = -1;
//static int al3010_state = 0;
/*
 * disable/enable ALS Camera 2V85
 */
static void control_als_cam_2v85(int enable){
	if( tegra3_get_project_id() == TEGRA3_PROJECT_TF201 ){
//		tegra_gpio_enable(ALS_CAM_POWER_2V85_EN_GPIO);
		int ret=0;
		ret = gpio_request(ALS_CAM_POWER_2V85_EN_GPIO, "gpio_pr7");
		if (ret < 0){
			pr_err("%s: gpio_request failed for gpio %s\n",__func__, "TEGRA_GPIO_PR7");
		}
		gpio_direction_output(ALS_CAM_POWER_2V85_EN_GPIO, enable);
		pr_info("gpio %d set to %d\n", ALS_CAM_POWER_2V85_EN_GPIO, gpio_get_value(ALS_CAM_POWER_2V85_EN_GPIO));
	}
}

/*
 * al3010 later init function
 */
static int al3010_later_init(struct i2c_client *client){
	int err = 0;

	//+++ limit check al3010 init time
	if( init_check_count > 0){
        init_check_count--;
    }else{
        return -1;
    }
    //---

	if (al3010_get_power_state(client) != 0x01){
		printk("light sensor info : al3010 power off , init in al3010_later_init \n");

		err = al3010_set_power_state(client, 1);
		if(err){
			printk("light sensor err : al3010 set power up err\n");
			return err;
		}

		err = al3010_set_range(client, 1);	//set range to 19000 lux
		if(err){
			printk("light sensor err : al3010 set range err\n");
			return err;
		}
	}

	return err;
}

/*
 * register access helpers
 */

static int __al3010_read_reg(struct i2c_client *client,
			       u32 reg, u8 mask, u8 shift)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	return (data->reg_cache[reg] & mask) >> shift;
}

static int __al3010_write_reg(struct i2c_client *client,
				u32 reg, u8 mask, u8 shift, u8 val)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;

	if (reg >= AL3010_NUM_CACHABLE_REGS)
		return -EINVAL;

	mutex_lock(&data->lock);

	tmp = data->reg_cache[reg];
	tmp &= ~mask;
	tmp |= val << shift;

	int addr = al3010_reg[reg];
	ret = i2c_smbus_write_byte_data(client, addr, tmp);
	if (!ret)
		data->reg_cache[reg] = tmp;

	mutex_unlock(&data->lock);
	return ret;
}

/*
 * internally used functions
 */

/* range */
static int al3010_get_range(struct i2c_client *client)
{
	int tmp;
	tmp = __al3010_read_reg(client, AL3010_ALS_COMMAND,
											AL3010_RAN_MASK, AL3010_RAN_SHIFT);;
	return al3010_range[tmp];
}

static int al3010_set_range(struct i2c_client *client, int range)
{
	return __al3010_write_reg(client, AL3010_ALS_COMMAND, 
											AL3010_RAN_MASK, AL3010_RAN_SHIFT, range);
}

/* resolution */
static int al3010_get_resolution(struct i2c_client *client)
{
	return 0;
}

static int al3010_set_resolution(struct i2c_client *client, int res)
{
	return 0;
}

/* mode */
static int al3010_get_mode(struct i2c_client *client)
{
	return __al3010_read_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT);
}

static int al3010_set_mode(struct i2c_client *client, int mode)
{
	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
		AL3010_MODE_MASK, AL3010_MODE_SHIFT, mode);
}

/* power_state */
static int al3010_set_power_state(struct i2c_client *client, int state)
{
	return __al3010_write_reg(client, AL3010_MODE_COMMAND,
				AL3010_POW_MASK, AL3010_POW_SHIFT, 
				state ? AL3010_POW_UP : AL3010_POW_DOWN);
}

static int al3010_get_power_state(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	u8 cmdreg = data->reg_cache[AL3010_MODE_COMMAND];
	return (cmdreg & AL3010_POW_MASK) >> AL3010_POW_SHIFT;
}

static int al3010_get_adc_value(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int lsb, msb, range;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, AL3010_ADC_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3010_ADC_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	if(!flagLoadAl3010Config){
		al3010_update_calibration();
		flagLoadAl3010Config = true;
	}

	//range = al3010_get_range(client);
	//printk("light sesnor info : calibration_base_lux = %d\n",calibration_base_lux);
	//printk("light sesnor info : calibration_regs = %d\n",calibration_regs);
	return (u32)( ( ((msb << 8) | lsb) * calibration_base_lux ) /calibration_regs);
	//return (u32)(((msb << 8) | lsb) * range) >> 16;
}

static int al3010_get_reg_value(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int lsb, msb, range;

	mutex_lock(&data->lock);
	lsb = i2c_smbus_read_byte_data(client, AL3010_ADC_LSB);

	if (lsb < 0) {
		mutex_unlock(&data->lock);
		return lsb;
	}

	msb = i2c_smbus_read_byte_data(client, AL3010_ADC_MSB);
	mutex_unlock(&data->lock);

	if (msb < 0)
		return msb;

	range = al3010_get_range(client);
	return (u16)((msb << 8) | lsb);
	//return (u32)(((msb << 8) | lsb) * range) >> 16;
}

/*
 * light sensor calibration
 */

static int al3010_update_calibration()
{
	char buf[256];
	int calibration_value = 0;
	mm_segment_t oldfs;
	oldfs=get_fs();
	set_fs(get_ds());
	memset(buf, 0, sizeof(u8)*256);
	struct file *fp = NULL;
	fp=filp_open(CAL_ALS_PATH, O_RDONLY, 0);
	if (!IS_ERR(fp)) {
		int ret = 0;
		ret = fp->f_op->read(fp, buf, sizeof(buf), &fp->f_pos);
		//printk("light sensor info : ret = %d , f_pos = %d",ret ,fp->f_pos);
		//printk("light sensor info : AL3010_Config content is :%s\n", buf);
		sscanf(buf,"%d\n", &calibration_value);
		//printk("light sensor info : calibration_value= %d\n",calibration_value);
		if(calibration_value > 0){
			calibration_regs = calibration_value;
		}
		filp_close(fp, NULL);
		set_fs(oldfs);
		return 0;
	}else{
		return -1;
	}
}

/*
 * sysfs layer
 */

/* power state */
static ssize_t al3010_show_power_state(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	al3010_later_init(client);
	return sprintf(buf, "%d\n", al3010_get_power_state(client));
}

/* lux */
static ssize_t al3010_show_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	al3010_later_init(client);

	return sprintf(buf, "%d\n", al3010_get_adc_value(client));
}

/* reg */
static ssize_t al3010_show_reg(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	al3010_later_init(client);

	return sprintf(buf, "%d\n", al3010_get_reg_value(client));
}

/* refresh calibration */
static ssize_t al3010_refresh_calibration(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"%d\n",al3010_update_calibration());
}

/* revise lux */
// for none parallel light
static ssize_t al3010_show_revise_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	if (al3010_get_power_state(client) != 0x01){
		int ret = al3010_later_init(client);
		if(ret != 0){
			return sprintf(buf, "%d\n", -1);
		}
	}

	//+++ wait al3010 wake up
	if(light_sensor_ready == false){
		int tmp_lux = al3010_get_adc_value(client);
		if( tmp_lux > 0){
			light_sensor_ready = true;
			catch_first_poll_time = true;
		}else if(catch_first_poll_time == false){
			do_gettimeofday(&t_first_poll_time);
			catch_first_poll_time = true;
			printk("light sensor debug : al3010 first poll timestamp , tv_sec = %d\n",t_first_poll_time.tv_sec);
			printk("light sensor debug : al3010 first poll timestamp , tv_usec = %d\n",t_first_poll_time.tv_usec);
			return sprintf(buf, "%d\n", -1);
		}else{
			struct timeval t_current_time;
			int diff_time = 0;
			do_gettimeofday(&t_current_time);

			printk("light sensor debug : al3010 current timestamp , tv_sec = %d\n",t_current_time.tv_sec);
			printk("light sensor debug : al3010 current timestamp , tv_usec = %d\n",t_current_time.tv_usec);

			diff_time = ( (t_current_time.tv_sec-t_first_poll_time.tv_sec)*1000000 + (t_current_time.tv_usec-t_first_poll_time.tv_usec) )/1000;
			if(diff_time > time_for_sensor_ready){
				light_sensor_ready = true;
			}else{
				return sprintf(buf, "%d\n", -1);
			}
		}
	}
	//---
	int revise_lux = al3010_get_adc_value(client)*2;
	//+++ filter noise value in range lux < 10
	if( (last_revise_lux >= 0) && (last_revise_lux < 10) ){
		bool is_noise = true;
		if( (revise_lux >= 10) && (revise_lux <= 30) ){
			// check first time & check require update lux
			if(catch_first_poll_noise_time){
				struct timeval t_current_noise_time;
				int diff_noise_time = 0;
				do_gettimeofday(&t_current_noise_time);
				diff_noise_time = ( (t_current_noise_time.tv_sec-t_first_poll_noise_time.tv_sec)*1000000 +
						(t_current_noise_time.tv_usec-t_first_poll_noise_time.tv_usec) )/1000;
				if( diff_noise_time > noise_waiting_time ){
					is_noise  = false;
				}
			}else{
				catch_first_poll_noise_time = true;
				do_gettimeofday(&t_first_poll_noise_time);
			}

			if( is_noise ){
				revise_lux = last_revise_lux;
			}
		}else{
			if( revise_lux > 30 ){
				printk("light sensor debug : al3010 stop filter noise lux > 30 , lux = %d\n",revise_lux);
			}
			// reset first time check
			catch_first_poll_noise_time = false;
		}

	}
        //---
	last_revise_lux = revise_lux;
	//+++ add thresholds for reading light sensor event
	if ( last_report_lux > -1 ){
		int diffValue = revise_lux - last_report_lux;
		if( revise_lux < 100 ){
			if( (diffValue < 5) && (diffValue > -5) )
				revise_lux = last_report_lux;
		}else if( revise_lux >= 100 && revise_lux < 1000 ){
			if( (diffValue < 50) && (diffValue > -50) )
				revise_lux = last_report_lux;
		}else if( revise_lux >= 1000 && revise_lux < 10000 ){
			if( (diffValue < 100) && (diffValue > -100) )
				revise_lux = last_report_lux;
		}else if( revise_lux >= 10000 ){
			if( (diffValue < 500) && (diffValue > -500) )
				revise_lux = last_report_lux;
		}
	}
	last_report_lux = revise_lux;
	//---
	return sprintf(buf, "%d\n", revise_lux );
}

/* default lux */
static ssize_t al3010_show_default_lux(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

        al3010_later_init(client);
    int show_lux_value = al3010_get_adc_value(client);
    int show_default_lux_value = (show_lux_value*calibration_regs)/default_calibration_regs;
	return sprintf(buf, "%d\n", show_default_lux_value);
}

static SENSOR_DEVICE_ATTR(show_reg, 0644, al3010_show_reg, NULL, 1);
static SENSOR_DEVICE_ATTR(show_lux, 0644, al3010_show_lux, NULL, 2);
static SENSOR_DEVICE_ATTR(lightsensor_status, 0644, al3010_show_power_state, NULL, 3);
static SENSOR_DEVICE_ATTR(refresh_cal, 0644, al3010_refresh_calibration, NULL, 4);
static SENSOR_DEVICE_ATTR(show_revise_lux, 0644, al3010_show_revise_lux, NULL, 5);
static SENSOR_DEVICE_ATTR(show_default_lux, 0644, al3010_show_default_lux, NULL, 6);

static struct attribute *al3010_attributes[] = {
	&sensor_dev_attr_show_reg.dev_attr.attr,
	&sensor_dev_attr_show_lux.dev_attr.attr,
	&sensor_dev_attr_lightsensor_status.dev_attr.attr,
	&sensor_dev_attr_refresh_cal.dev_attr.attr,
	&sensor_dev_attr_show_revise_lux.dev_attr.attr,
	&sensor_dev_attr_show_default_lux.dev_attr.attr,
	NULL
};

static const struct attribute_group al3010_attr_group = {
	.attrs = al3010_attributes,
};

static int al3010_init_client(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	int err = 0;
	/* set defaults */
	err = al3010_set_power_state(client, 1);
	if(err){
		printk("light sensor err : al3010 set power up err\n");
		return err;
	}
	err = al3010_set_range(client, 1);	//set range to 19000 lux
	if(err){
		printk("light sensor err : al3010 set range err\n");
		return err;
	}
	//al3010_set_resolution(client, 0);
	//al3010_set_mode(client, 0);

	return 0;
}

/**
 * i2c stress test
 */
int al3010_open(struct inode *inode, struct file *filp)
{
	printk("light sensor info : %s\n", __func__);
	return 0;
}

int al3010_release(struct inode *inode, struct file *filp)
{
	printk("light sensor info : %s\n", __func__);
	return 0;
}

int al3010_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;

	if (_IOC_TYPE(cmd) != AL3010_IOC_MAGIC)
	return -ENOTTY;
	if (_IOC_NR(cmd) > AL3010_IOC_MAXNR)
	return -ENOTTY;

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));

	if (err)
		return -EFAULT;

	switch (cmd) {
		case AL3010_POLL_DATA:
			if (arg == AL3010_IOCTL_START_HEAVY){
				printk("light sensor info : ioctl heavy\n");
				poll_mode = START_HEAVY;
				queue_delayed_work(sensor_work_queue, &al3010_poll_data_work, poll_mode);
			}
			else if (arg == AL3010_IOCTL_START_NORMAL){
				printk("light sensor info : ioctl normal\n");
				poll_mode = START_NORMAL;
				queue_delayed_work(sensor_work_queue, &al3010_poll_data_work, poll_mode);
			}
			else if  (arg == AL3010_IOCTL_END){
				printk("light sensor info : ioctl end\n");
				cancel_delayed_work_sync(&al3010_poll_data_work);
			}
			else
				return -ENOTTY;
			break;
		default: /* redundant, as cmd was checked against MAXNR */
			return -ENOTTY;
	}

	return 0;
}
struct file_operations al3010_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = al3010_ioctl,
	.open =	al3010_open,
	.release = al3010_release,
};

static void  al3010_poll_data(struct work_struct * work)
{
	int lux = al3010_get_adc_value(al3010_client);
	//printk("FOR TEST , al3010_poll_data light sensor lux = %d\n",lux);
	if(poll_mode ==0)
		msleep(5);

	queue_delayed_work(sensor_work_queue, &al3010_poll_data_work, poll_mode);
}

/*
 * I2C layer
 */

static int __devinit al3010_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct al3010_data *data;
	int err = 0;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE))
		return -EIO;

	data = kzalloc(sizeof(struct al3010_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	i2c_set_clientdata(client, data);
	mutex_init(&data->lock);

	/* initialize the AL3010 chip */
	//err = al3010_init_client(client);
	//if (err){
		//printk("light sensor err : al3010 init fail in probe \n");
		//goto exit_kfree;
	//}

	/* register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &al3010_attr_group);
	if (err){
		printk("light sensor err : al3010 init sysfs fail\n");
		goto exit_kfree;
	}

	/* register device node */

	printk("light sensor info : al3010 probe successed\n");
	dev_info(&client->dev, "driver version %s enabled\n", DRIVER_VERSION);

	/* init for i2c stress test */
	sensor_work_queue = create_singlethread_workqueue("i2c_lightsensor_wq");
	if(!sensor_work_queue){
		printk("al3010_probe: Unable to create workqueue");
		goto exit_kfree;
	}
	INIT_DELAYED_WORK(&al3010_poll_data_work, al3010_poll_data);
	al3010_client = client;
	data->misc_dev.minor  = MISC_DYNAMIC_MINOR;
	data->misc_dev.name = "lightsensor";
	data->misc_dev.fops = &al3010_fops;
	err = misc_register(&data->misc_dev);
	if (err){
		printk("light sensor err : Unable to register %s\misc device\n",
				data->misc_dev.name);
		goto exit_kfree;
	}

	return 0;

exit_kfree:
	kfree(data);
	return err;
}

static int __devexit al3010_remove(struct i2c_client *client)
{
	struct al3010_data *data = i2c_get_clientdata(client);
	misc_deregister(&data->misc_dev);

	sysfs_remove_group(&client->dev.kobj, &al3010_attr_group);
	al3010_set_power_state(client, 0);
	kfree(i2c_get_clientdata(client));
	printk("light sensor info : al3010 remove successed\n");
	return 0;
}

#ifdef CONFIG_PM
static int al3010_suspend(struct i2c_client *client, pm_message_t mesg)
{
	printk("al3010_suspend+\n");
	int ret = 0;
	struct al3010_data *data = i2c_get_clientdata(client);

	data->power_state_before_suspend = al3010_get_power_state(client);
	al3010_set_power_state(client, 0);
	light_sensor_ready = false;
	catch_first_poll_time = false;
	control_als_cam_2v85(0);	//disable 2V85
	init_check_count = 0;
	printk("al3010_suspend-\n");
	return ret;
}

static int al3010_resume(struct i2c_client *client)
{
	printk("al3010_resume+\n");
	control_als_cam_2v85(1);	//enable 2V85
	int i;
	int ret=0;
	struct al3010_data *data = i2c_get_clientdata(client);
	//al3010_init_client(client);
	init_check_count = init_check_max_times;
	printk("al3010_resume-\n");
	return ret;
}

#else
#define al3010_suspend	NULL
#define al3010_resume		NULL
#endif /* CONFIG_PM */

static const struct i2c_device_id al3010_id[] = {
	{ "al3010", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, al3010_id);

static struct i2c_driver al3010_driver = {
	.driver = {
		.name	= AL3010_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.suspend = al3010_suspend,
	.resume	= al3010_resume,
	.probe	= al3010_probe,
	.remove	= __devexit_p(al3010_remove),
	.id_table = al3010_id,
};

static int __init al3010_init(void)
{
	printk(KERN_INFO "%s+ #####\n", __func__);
	printk("light sensor info : al3010 init \n");
	control_als_cam_2v85(1);	//enable 2V85
	int ret = i2c_add_driver(&al3010_driver);
	init_check_count = init_check_max_times;
	printk(KERN_INFO "%s- #####\n", __func__);
	return ret;
}

static void __exit al3010_exit(void)
{
	printk("light sensor info : al3010 exit \n");
	i2c_del_driver(&al3010_driver);
}

MODULE_AUTHOR("yc");
MODULE_DESCRIPTION("test version v1.0");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(al3010_init);
module_exit(al3010_exit);

