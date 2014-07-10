#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>

#include <asm/gpio.h>

#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <asm/ioctl.h>
#include <asm/uaccess.h>
#include <linux/delay.h>

//----
#include <linux/input.h>
#include <linux/debugfs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/gpio.h>
//----

//#include <mach/board-ventana-misc.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include "dsp.h"
#include <mach/board-cardhu-misc.h>

#undef DUMP_REG

#define BYPASS_DSP_FOR_NORMAL_RECORDING

#define DSP_IOC_MAGIC	0xf3
#define DSP_IOC_MAXNR	3
#define DSP_CONTROL	_IOW(DSP_IOC_MAGIC, 1,int)
#define DSP_RECONFIG	_IOW(DSP_IOC_MAGIC, 2,int)
#define DSP_STRESS_TEST	_IOW(DSP_IOC_MAGIC, 3,int)
#define DSP_IOCTL_START_HEAVY (2)
#define DSP_IOCTL_START_NORMAL (1)
#define DSP_IOCTL_STOP (0)
#define START_NORMAL (HZ/2)
#define START_HEAVY (HZ/20)

#define START_RECORDING 1
#define END_RECORDING 0
#define PLAYBACK 2
#define INPUT_SOURCE_NORMAL 	    100
#define INPUT_SOURCE_VR 			101
#define OUTPUT_SOURCE_NORMAL		200
#define OUTPUT_SOURCE_VOICE            201
#define INPUT_SOURCE_NO_AGC 300
#define INPUT_SOURCE_AGC 301

#define HEADPHONE_NO_MIC	0
#define HEADSET_WITH_MIC	1

#define TIME_WAKEUP_TO_PROGRAMMING     20
#define TIME_RESET 10
#define MAX_RETRY (5)
#define DEVICE_NAME		"dsp_fm34"

#define DSP_POWER_1V8_EN_GPIO_TF201 TEGRA_GPIO_PU5
#define DSP_POWER_1V8_EN_GPIO_TF201X TEGRA_GPIO_PP3

struct i2c_client *fm34_client;

static int fm34_probe(struct i2c_client *client,
			   const struct i2c_device_id *id);
static int fm34_remove(struct i2c_client *client);
static int fm34_suspend(struct device *dev);
static int fm34_resume(struct device *dev);
static void fm34_reconfig(void) ;
extern int hs_micbias_power(int on);


extern bool headset_alive;

static const struct i2c_device_id fm34_id[] = {
	{DEVICE_NAME, 0},
	{}
};

struct i2c_client *fm34_client;
struct fm34_chip *dsp_chip;
struct delayed_work poll_dsp_work;
struct delayed_work config_dsp_work;
static int count_base = 1;
static int count_100 = 0;
static int poll_rate = 0;
bool bConfigured=false;
int PID=0;
static int input_source=INPUT_SOURCE_NORMAL;
static int output_source=OUTPUT_SOURCE_NORMAL;
static int input_agc = INPUT_SOURCE_NO_AGC;
bool isRecording = false;

EXPORT_SYMBOL(isRecording);

MODULE_DEVICE_TABLE(i2c, fm34_id);

static const struct dev_pm_ops fm34_dev_pm_ops = {
	.suspend	= fm34_suspend,
	.resume		= fm34_resume,
};

static struct i2c_driver fm34_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= DEVICE_NAME,
		.pm	= &fm34_dev_pm_ops,
	},
	.probe		= fm34_probe,
	.remove		= fm34_remove,
	.id_table	= fm34_id,
};

unsigned int fm34_i2c_read(unsigned char AdrHi,unsigned char AdrLo, unsigned char *data)
{
	unsigned int ret = 0;
	u8 cmd_addr[5] = {0};
	u8 cmd_lo[4] = {0};
	u8 cmd_hi[4] = {0};

	cmd_addr[0] = 0xFC;
	cmd_addr[1] = 0xF3;
	cmd_addr[2] = 0x37;
	cmd_addr[3] = AdrHi;
	cmd_addr[4] = AdrLo;

	cmd_lo[0] = 0xFC;
	cmd_lo[1] = 0xF3;
	cmd_lo[2] = 0x60;
	cmd_lo[3] = 0x25;

	cmd_hi[0] = 0xFC;
	cmd_hi[1] = 0xF3;
	cmd_hi[2] = 0x60;
	cmd_hi[3] = 0x26;

	//set read addr
	ret = i2c_master_send(dsp_chip->client, cmd_addr, 5);
	if(ret != 5){
		FM34_INFO("DSP set read reg  fail\n");
	}

	ret = i2c_master_send(dsp_chip->client, cmd_lo, 4);
	if(ret != 4){
		FM34_INFO("DSP send read low byte addr fail\n");
	}

	ret = i2c_master_recv(dsp_chip->client, data, 1);
	if(ret != 1){
		FM34_INFO("DSP read low byte fail, data: %d\n", data[0]);
	}

	ret = i2c_master_send(dsp_chip->client, cmd_hi, 4);
	if(ret != 4){
		FM34_INFO("DSP send read high byte  addr fail\n");
	}

	ret = i2c_master_recv(dsp_chip->client, (data + 1), 1);
	if(ret != 1){
		FM34_INFO("DSP read high byte fail,  data: %d\n", data[1]);
	}

	return ret;
}

int fm34_i2c_retry(struct i2c_client *fm34_i2c_client, u8* parameter, size_t size)
{
	int retry = 0;
	int ret = -1;

        ret = i2c_master_send(fm34_i2c_client, parameter, size);
        msleep(5);

	while(retry < MAX_RETRY && ret < 0){
		retry++;
		FM34_INFO("i2c no ack retry time = %d\n", retry);
		ret = i2c_master_send(fm34_i2c_client, parameter, size);
		msleep(5);
	}

	if(retry == MAX_RETRY)
		FM34_INFO("i2c retry fail, exceed maximum retry times = %d\n", MAX_RETRY);

	return ret;

}

void fm34_reset_DSP(void)
{
	gpio_set_value(TEGRA_GPIO_PO3, 0);
	msleep(TIME_RESET);
	FM34_INFO("GPIO = %d , state = %d\n", TEGRA_GPIO_PO3, gpio_get_value(TEGRA_GPIO_PO3));

	gpio_set_value(TEGRA_GPIO_PO3, 1);
	FM34_INFO("GPIO = %d , state = %d\n", TEGRA_GPIO_PO3, gpio_get_value(TEGRA_GPIO_PO3));

	return;
}

int fm34_config_DSP(void)
{
	int ret=0;
	struct i2c_msg msg[3];
	u8 buf1;
	int config_length;
	u8 *config_table;

	if(!bConfigured){
		fm34_reset_DSP();
		msleep(100);

		gpio_set_value(TEGRA_GPIO_PBB6, 1); // Enable DSP
		msleep(TIME_WAKEUP_TO_PROGRAMMING);

		//access chip to check if acknowledgement.
		buf1=0xC0;
		/* Write register */
		msg[0].addr = dsp_chip->client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = &buf1;

		ret = i2c_transfer(dsp_chip->client->adapter, msg, 1);
		if(ret < 0){
			FM34_INFO("DSP NOack, Failed to read 0x%x: %d\n", buf1, ret);
			msleep(50);
			fm34_reset_DSP();
			return ret;
		}
		else
			FM34_INFO("DSP ACK,  read 0x%x: %d\n", buf1, ret);

		if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T){
			FM34_INFO("Load TF700T DSP parameters\n");
			config_length= sizeof(input_parameter_TF700T);
			config_table= (u8 *)input_parameter_TF700T;
		}else if(tegra3_get_project_id() == TEGRA3_PROJECT_TF201){
			config_length= sizeof(input_parameter_TF201);
			config_table= (u8 *)input_parameter_TF201;
		}else{
			FM34_INFO("Load  DSP parameters\n");
			config_length= sizeof(input_parameter);
			config_table= (u8 *)input_parameter;
		}

		ret = fm34_i2c_retry(dsp_chip->client, config_table, config_length);
		FM34_INFO("config_length = %d\n", config_length);

		if(ret == config_length){
			FM34_INFO("DSP configuration is done\n");
			bConfigured=true;
		}

		msleep(100);
		gpio_set_value(TEGRA_GPIO_PBB6, 0);
	}


	return ret;
}
EXPORT_SYMBOL(fm34_config_DSP);

static ssize_t fm34_show(struct device *class, struct device_attribute *attr, char *buf)
{
	struct fm34_chip *data = i2c_get_clientdata(to_i2c_client(class));

	return sprintf(buf, "%d\n", data->status);
}


static unsigned int fm34_read_chipID(void)
{
	unsigned char data[2] = {0};
	fm34_i2c_read(0x38, 0x00, data);

	return ((data[1]  << 8) | data[0]);


}

static int fm34_check_i2c(struct i2c_client *client)
{
	int ret = -1;
	struct i2c_msg msg[3];
	u8 buf1;

	//reset dsp
	gpio_set_value(TEGRA_GPIO_PO3, 0);
	msleep(10);
	gpio_set_value(TEGRA_GPIO_PO3, 1);


	msleep(100);

	gpio_set_value(TEGRA_GPIO_PBB6, 1); // Enable DSP
	msleep(20);

	//access chip to check if acknowledgement.
	buf1=0xC0;
	/* Write register */
	msg[0].addr = dsp_chip->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf1;

	ret = i2c_transfer(dsp_chip->client->adapter, msg, 1);
	if(ret < 0){
		//FM34_INFO("DSP NOack, Failed to read 0x%x: %d\n", buf1, ret);
		msleep(50);
		//reset dsp
		gpio_set_value(TEGRA_GPIO_PO3, 0);
		msleep(10);
		gpio_set_value(TEGRA_GPIO_PO3, 1);
		return ret;
	}
	else if(ret == 1){ /* reg 0xC0 value should be 1 */
		//FM34_INFO("DSP ack, read 0x%x: %d\n", buf1, ret);
		return 0;
	}
}


static int dsp_stress(void)
{

	unsigned int chipID = 0;

	chipID = fm34_read_chipID();
	if(chipID != 0x395B)
		printk("DSP stress test: tegra-i2c.0 wrong chip ID = 0x%x\n", chipID);

	count_base = count_base+1;

	if (count_base == 100){
		count_base = 0;
		count_100 = count_100 + 1;
		printk("DSP stress test: count = %d (* 100), chip ID = 0x%x\n",count_100, chipID);
	}

	schedule_delayed_work(&poll_dsp_work, poll_rate);

	return 0;
}

static int fm34_chip_init(struct i2c_client *client)
{
	int rc = 0;

	//config RST# pin, default HIGH.
	rc = gpio_request(TEGRA_GPIO_PO3, "fm34_reset");
	if (rc) {
		FM34_ERR("gpio_request failed for input %d\n", TEGRA_GPIO_PO3);
	}

	rc = gpio_direction_output(TEGRA_GPIO_PO3, 1) ;
	if (rc) {
		FM34_ERR("gpio_direction_output failed for input %d\n", TEGRA_GPIO_PO3);
	}
	FM34_INFO("GPIO = %d , state = %d\n", TEGRA_GPIO_PO3, gpio_get_value(TEGRA_GPIO_PO3));

	gpio_set_value(TEGRA_GPIO_PO3, 1);

	//config PWDN# pin, default HIGH.
	rc = gpio_request(TEGRA_GPIO_PBB6, "fm34_pwdn");
	if (rc) {
		FM34_ERR("gpio_request failed for input %d\n", TEGRA_GPIO_PBB6);
	}

	rc = gpio_direction_output(TEGRA_GPIO_PBB6, 1) ;
	if (rc) {
		FM34_ERR("gpio_direction_output failed for input %d\n", TEGRA_GPIO_PBB6);
	}
	FM34_INFO("GPIO = %d , state = %d\n", TEGRA_GPIO_PBB6, gpio_get_value(TEGRA_GPIO_PBB6));

	gpio_set_value(TEGRA_GPIO_PBB6, 1);

	return 0;
}

int fm34_open(struct inode *inode, struct file *filp)
{
	return 0;          /* success */
}


int fm34_release(struct inode *inode, struct file *filp)
{
	return 0;          /* success */
}

long fm34_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int retval = 0;
	static int recording_enabled = -1;

	if (_IOC_TYPE(cmd) != DSP_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > DSP_IOC_MAXNR) return -ENOTTY;

	/*
	 * the direction is a bitmask, and VERIFY_WRITE catches R/W
	 * transfers. `Type' is user-oriented, while
	 * access_ok is kernel-oriented, so the concept of "read" and
	 * "write" is reversed
	 * access_ok: 1 (successful, accessable)
	 */

	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;

       /* cmd: the ioctl commend user-space asked */
	switch(cmd){
		case DSP_STRESS_TEST:
			printk("DSP: STRESS_TEST: %lu (1: Start, 0: Stop)\n",arg);
			if(arg==DSP_IOCTL_START_HEAVY){
				gpio_set_value(TEGRA_GPIO_PBB6, 1);
				msleep(TIME_WAKEUP_TO_PROGRAMMING);
				poll_rate = START_HEAVY;
				schedule_delayed_work(&poll_dsp_work, poll_rate);
			}else if(arg==DSP_IOCTL_START_NORMAL){
				gpio_set_value(TEGRA_GPIO_PBB6, 1);
				msleep(TIME_WAKEUP_TO_PROGRAMMING);
				poll_rate = START_NORMAL;
				schedule_delayed_work(&poll_dsp_work,poll_rate);
			}else if(arg==DSP_IOCTL_STOP){
				cancel_delayed_work_sync(&poll_dsp_work);
				gpio_set_value(TEGRA_GPIO_PBB6, 0);
			}
			break;
		case DSP_CONTROL:
			fm34_config_DSP();

			switch(arg){
				case START_RECORDING:
					gpio_set_value(TEGRA_GPIO_PBB6, 1);
					msleep(TIME_WAKEUP_TO_PROGRAMMING);
					if(headset_alive){/*Headset mode*/
							FM34_INFO("Start recording(AMIC), bypass DSP\n");
							fm34_i2c_retry(dsp_chip->client, (u8 *)bypass_parameter,
										sizeof(bypass_parameter));
							gpio_set_value(TEGRA_GPIO_PBB6, 0);

					}else{/*Handsfree mode*/
#ifdef BYPASS_DSP_FOR_NORMAL_RECORDING
						FM34_INFO("Start recording(DMIC), ");
						isRecording = true;
						if(input_source==INPUT_SOURCE_VR){
							FM34_INFO("enable DSP since VR case (INPUT_SOURCE_VR)\n");
							fm34_i2c_retry(dsp_chip->client, (u8 *)enable_parameter,
										sizeof(enable_parameter));

							FM34_INFO("Disable Noise Suppression\n");
							if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
								fm34_i2c_retry(dsp_chip->client, (u8 *)TF700T_disable_NS,
										sizeof(TF700T_disable_NS));
							else
								fm34_i2c_retry(dsp_chip->client, (u8 *)TF201_disable_NS,
										sizeof(TF201_disable_NS));
						}
						else if(output_source==OUTPUT_SOURCE_VOICE || input_agc==INPUT_SOURCE_AGC){
							FM34_INFO("enable DSP since VOICE case (OUTPUT_SOURCE_VOICE)\n");
							fm34_i2c_retry(dsp_chip->client, (u8 *)enable_parameter,
										sizeof(enable_parameter));

							FM34_INFO("Enable Noise Suppression\n");
							if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
								fm34_i2c_retry(dsp_chip->client, (u8 *)TF700T_enable_NS,
										sizeof(TF700T_enable_NS));
							else
								fm34_i2c_retry(dsp_chip->client, (u8 *)TF201_enable_NS,
										sizeof(TF201_enable_NS));
						}
						else{
							FM34_INFO("bypass DSP since NORMAL recording\n");
							fm34_i2c_retry(dsp_chip->client, (u8 *)bypass_parameter,
										sizeof(bypass_parameter));
							gpio_set_value(TEGRA_GPIO_PBB6, 0);
						}

#else
#ifdef BYPASS_DSP_FOR_VR
                                                FM34_INFO("Start recording(DMIC), ");
                                                if(input_source==INPUT_SOURCE_VR){
                                                        FM34_INFO("bypass DSP since BYPASS_DSP_FOR_VR\n");
                                                        fm34_i2c_retry(dsp_chip->client, (u8 *)bypass_parameter,
										sizeof(bypass_parameter));
                                                        gpio_set_value(TEGRA_GPIO_PBB6, 0);
                                                }
                                                else{
							FM34_INFO("enable DSP\n");
							fm34_i2c_retry(dsp_chip->client, (u8 *)enable_parameter,
										 sizeof(enable_parameter));

							FM34_INFO("Enable Noise Suppression\n");
							if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
								fm34_i2c_retry(dsp_chip->client, (u8 *)TF700T_enable_NS,
										sizeof(TF700T_enable_NS));
							else
								fm34_i2c_retry(dsp_chip->client, (u8 *)TF201_enable_NS,
										sizeof(TF201_enable_NS));

                                               }
#else

							FM34_INFO("Start recording(DMIC), enable DSP\n");
							fm34_i2c_retry(dsp_chip->client, (u8 *)enable_parameter,
										sizeof(enable_parameter));

							if(input_source==INPUT_SOURCE_VR){

								FM34_INFO("Disable Noise Suppression\n");
								if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
									fm34_i2c_retry(dsp_chip->client, (u8 *)TF700T_disable_NS,
										sizeof(TF700T_disable_NS));
								else
									fm34_i2c_retry(dsp_chip->client, (u8 *)TF201_disable_NS,
                                                                                sizeof(TF201_disable_NS));

							}
							else{
								FM34_INFO("Enable Noise Suppression\n");
								if(tegra3_get_project_id() == TEGRA3_PROJECT_TF700T)
									fm34_i2c_retry(dsp_chip->client, (u8 *)TF700T_enable_NS,
										sizeof(TF700T_enable_NS));
								else
									fm34_i2c_retry(dsp_chip->client, (u8 *)TF201_enable_NS,
										sizeof(TF201_enable_NS));
							}
#endif  //BYPASS_DSP_FOR_VR
#endif  //BYPASS_DSP_FOR_NORMAL_RECORDING

					}
					recording_enabled = START_RECORDING;
					break;

				case END_RECORDING:
					isRecording = false;
					if(recording_enabled == START_RECORDING){
						FM34_INFO("End recording, bypass DSP\n");
						gpio_set_value(TEGRA_GPIO_PBB6, 1);
						msleep(TIME_WAKEUP_TO_PROGRAMMING);
						fm34_i2c_retry(dsp_chip->client, (u8 *)bypass_parameter, sizeof(bypass_parameter));
						recording_enabled = END_RECORDING;
						gpio_set_value(TEGRA_GPIO_PBB6, 0);
					}
					else{
						FM34_INFO("End recording, but do nothing with DSP because no recording activity found\n");
						recording_enabled = END_RECORDING;
					}
					break;

				case INPUT_SOURCE_NORMAL:
				case INPUT_SOURCE_VR:
					FM34_INFO("Input source = %s\n", arg==INPUT_SOURCE_NORMAL? "NORMAL" : "VR");
					input_source=arg;
					break;
				case OUTPUT_SOURCE_NORMAL:
				case OUTPUT_SOURCE_VOICE:
					FM34_INFO("Output source = %s\n", arg==OUTPUT_SOURCE_NORMAL? "NORMAL" : "VOICE");
					output_source=arg;
					break;
				case INPUT_SOURCE_AGC:
				case INPUT_SOURCE_NO_AGC:
					printk("Input AGC = %s\n",	 arg == INPUT_SOURCE_AGC ? "AGC" : "NON-AGC");
					input_agc = arg;
					break;
				case PLAYBACK:
					FM34_INFO("Do nothing because playback path always be bypassed after a DSP patch\n");
				default:
				break;
			}
		break;

		case DSP_RECONFIG:
			FM34_INFO("DSP ReConfig parameters\n");
			fm34_reconfig();
		break;

	  default:  /* redundant, as cmd was checked against MAXNR */
		return -ENOTTY;
	}
	return retval;
}


struct file_operations fm34_fops = {
	.owner =    THIS_MODULE,
	.unlocked_ioctl =	fm34_ioctl,
	.open =		fm34_open,
	.release =	fm34_release,
};


static SENSOR_DEVICE_ATTR(dsp_status, S_IRUGO, fm34_show, NULL, 1);

static struct attribute *fm34_attr[] = {
	&sensor_dev_attr_dsp_status.dev_attr.attr,
	NULL
};


static int fm34_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct fm34_chip *data;
	int err;

	dev_dbg(&client->dev, "%s()\n", __func__);

	data = kzalloc(sizeof (struct fm34_chip), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Memory allocation fails\n");
		err = -ENOMEM;
		goto exit;
	}

	dsp_chip=data;
	data->status = 0;

	i2c_set_clientdata(client, data);
	data->client = client;
	fm34_client= data->client;

	data->misc_dev.minor  = MISC_DYNAMIC_MINOR;
	data->misc_dev.name = DEVICE_NAME;
	data->misc_dev.fops = &fm34_fops;
	err = misc_register(&data->misc_dev);
		if (err) {
			pr_err("tegra_acc_probe: Unable to register %s misc device\n", data->misc_dev.name);
		goto exit_free;
			}

	/* Register sysfs hooks */
	data->attrs.attrs = fm34_attr;
	err = sysfs_create_group(&client->dev.kobj, &data->attrs);
	if (err) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		goto exit_free;
	}

	fm34_chip_init(dsp_chip->client);
	if( fm34_check_i2c(dsp_chip->client) == 0)
       	data->status = 1;
	else
		data->status = 0;

	bConfigured=false;
	INIT_DELAYED_WORK(&poll_dsp_work, dsp_stress);
	INIT_DELAYED_WORK(&config_dsp_work, fm34_reconfig);
	schedule_delayed_work(&config_dsp_work, 0);
	pr_info("%s()\n", __func__);

	return 0;

exit_free:
	kfree(data);
exit:
	return err;
}

static int fm34_remove(struct i2c_client *client)
{
	struct fm34_chip *data = i2c_get_clientdata(client);

	misc_deregister(&data->misc_dev);
	dev_dbg(&client->dev, "%s()\n", __func__);
	pr_info("%s()\n", __func__);
	sysfs_remove_group(&client->dev.kobj, &data->attrs);

	kfree(data);
	return 0;
}

void fm34_reconfig(void)
{
	FM34_INFO("ReConfigure DSP\n");
	bConfigured=false;
	fm34_config_DSP();
}
void fm34_power_switch(int state)
{
	unsigned dsp_1v8_power_control;
	u32 project_info = tegra3_get_project_id();

	if(project_info == TEGRA3_PROJECT_TF201)
		dsp_1v8_power_control = DSP_POWER_1V8_EN_GPIO_TF201;
	else if(project_info == TEGRA3_PROJECT_TF300T)
		dsp_1v8_power_control = DSP_POWER_1V8_EN_GPIO_TF201X;
	else
		return;
	if(state){
		gpio_set_value(dsp_1v8_power_control, 1);
		schedule_delayed_work(&config_dsp_work, 0);
	}
	else{
		gpio_set_value(dsp_1v8_power_control, 0);
		bConfigured=false;
	}

}

void fm34_power_switch_init(void)
{
	unsigned dsp_1v8_power_control;
	int ret = 0;
	u32 project_info = tegra3_get_project_id();

	if(project_info == TEGRA3_PROJECT_TF201)
		dsp_1v8_power_control = DSP_POWER_1V8_EN_GPIO_TF201;
	else if(project_info == TEGRA3_PROJECT_TF300T)
		dsp_1v8_power_control = DSP_POWER_1V8_EN_GPIO_TF201X;
	else
		return;

	//Enalbe dsp power 1.8V
	ret = gpio_request(dsp_1v8_power_control, "dsp_power_1v8_en");
	if (ret < 0)
		pr_err("%s: gpio_request failed for gpio %s\n",
			__func__, "DSP_POWER_1V8_EN_GPIO");
	gpio_direction_output(dsp_1v8_power_control, 1);
	pr_info("gpio %d set to %d\n", dsp_1v8_power_control, gpio_get_value(dsp_1v8_power_control));

}


static int fm34_suspend(struct device *dev)
{
	printk("fm34_suspend+\n");
	int ret =0;

	gpio_set_value(TEGRA_GPIO_PBB6, 0); /* Bypass DSP*/
	u32 project_info = tegra3_get_project_id();
	fm34_power_switch(0);
	if(project_info == TEGRA3_PROJECT_TF201){
		printk("%s(): project TF201\n", __func__);
		gpio_set_value(TEGRA_GPIO_PO3, 0);
		//Set DAP2_FS to low in LP0 for voltage leaking.
		ret = gpio_request(TEGRA_GPIO_PA2, "dap2_fs");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "DAP2_FS");
		gpio_direction_output(TEGRA_GPIO_PA2, 0);
		gpio_free(TEGRA_GPIO_PA2);
		//Set AUDIO_MCLK to low in LP0 for voltage leaking.
		ret = gpio_request(TEGRA_GPIO_PW4, "audio_mclk");
		if (ret < 0)
			pr_err("%s: gpio_request failed for gpio %s\n",
				__func__, "AUDIO_MCLK");
		gpio_direction_output(TEGRA_GPIO_PW4, 0);
		gpio_free(TEGRA_GPIO_PW4);
	}
	gpio_set_value(TEGRA_GPIO_PBB6, 0); /* Bypass DSP*/
	printk("fm34_suspend-\n");
	return 0;
}
static int fm34_resume(struct device *dev)
{
	printk("fm34_resume+\n");
	u32 project_info = tegra3_get_project_id();

	if(project_info == TEGRA3_PROJECT_TF201){
		gpio_set_value(TEGRA_GPIO_PO3, 1);
	}
	fm34_power_switch(1);
	printk("fm34_resume-\n");
	return 0;
}

static int __init fm34_init(void)
{
	unsigned int project_info = 0;
	project_info = tegra3_get_project_id();

	if(project_info == TEGRA3_PROJECT_TF500T ||
		project_info == TEGRA3_PROJECT_P1801 ||
		project_info == TEGRA3_PROJECT_ME301T ||
		project_info == TEGRA3_PROJECT_ME301TL ||
		project_info == TEGRA3_PROJECT_ME570T)
		return 0;
	else{
		printk(KERN_INFO "%s+ #####\n", __func__);
		int ret;

		//Enalbe dsp power 1.8V
		fm34_power_switch_init();

		pr_info("%s()\n", __func__);
		ret = i2c_add_driver(&fm34_driver);

		printk(KERN_INFO "%s- #####\n", __func__);
		return ret;
	}
}

static void __exit fm34_exit(void)
{
	pr_info("%s()\n", __func__);
	i2c_del_driver(&fm34_driver);
}

module_init(fm34_init);
module_exit(fm34_exit);
