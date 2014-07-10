#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/wakelock.h>
#include "asus_gps.h"
#include "board-cardhu.h"
#include <mach/board-cardhu-misc.h>

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

extern struct platform_device *dock_port_device_info(void);
extern bool gps_dongle_flag;

static struct wake_lock gps_config_wake_lock;

static struct platform_device *asus_gps_dock_port_device;
static int asus_gps_probe(struct platform_device *pdev);
static void asus_gps_shutdown(struct platform_device *pdev);
static int asus_gps_suspend(struct platform_device *pdev, pm_message_t state);
static int asus_gps_resume(struct platform_device *pdev);
static struct platform_device asus_gps_device = {
		.name = "asus_gps",
};

static struct platform_driver asus_gps_driver = {
		.probe = asus_gps_probe,
		.shutdown = asus_gps_shutdown,
		.suspend = asus_gps_suspend,
		.resume = asus_gps_resume,
		.driver = {
			.name = "asus_gps",
			.owner = THIS_MODULE,
		},
};

static int asus_gps_probe(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
	return 0;
}

static void asus_gps_shutdown(struct platform_device *pdev)
{
	pr_info("%s\n", __func__);
}

static int asus_gps_suspend(struct platform_device *pdev, pm_message_t state)
{
	pr_info("%s+\n", __func__);
	asus_gps_dock_port_device = dock_port_device_info();

	if(gps_dongle_flag == true && asus_gps_dock_port_device != NULL) {
		tegra_usb3_utmip_host_unregister(asus_gps_dock_port_device);
		asus_gps_dock_port_device = NULL;
		gps_dongle_flag = false;
	}
	pr_info("%s-\n", __func__);
	return 0;
}

static int asus_gps_resume(struct platform_device *pdev)
{
	pr_info("%s+\n", __func__);
	wake_lock(&gps_config_wake_lock);
	if(gps_dongle_flag == false && asus_gps_dock_port_device == NULL)
		tegra_usb3_utmip_host_register();
	wake_unlock(&gps_config_wake_lock);
	pr_info("%s-\n", __func__);
	return 0;
}

static int __init asus_gps_init(void)
{
	int rc = 0;
	u32 project_info = tegra3_get_project_id();

	pr_info("%s+\n", __func__);

	if(project_info != TEGRA3_PROJECT_TF201)
		goto failed;

	rc = platform_device_register(&asus_gps_device);
	if(rc) {
		printk("platform_device_register failed\n");
		goto failed;
	}

	rc = platform_driver_register(&asus_gps_driver);
	if(rc) {
		printk("platform_driver_register failed\n");
		goto failed;
	}
	wake_lock_init(&gps_config_wake_lock, WAKE_LOCK_SUSPEND, "gps_config_wake_lock");

	pr_info("%s-\n", __func__);
	return 0;
failed:
	return rc;
}

static void __exit asus_gps_exit(void)
{
	pr_info("%s+\n", __func__);
	wake_lock_destroy(&gps_config_wake_lock);
	platform_driver_unregister(&asus_gps_driver);
	platform_device_unregister(&asus_gps_device);
	pr_info("%s-\n", __func__);
}

module_init(asus_gps_init);
module_exit(asus_gps_exit);

