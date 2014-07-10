#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/switch.h>
#include <linux/delay.h>
#include <linux/wakelock.h>

#include "board-cardhu.h"
#include "baseband-xmm-power.h"
#include "include/mach/board-cardhu-misc.h"

#include "pm-irq.h"
#include "ril.h"
#include "ril_modem_crash.h"

static struct device *dev;
static struct workqueue_struct *workqueue;
static struct work_struct crash_work;
static struct work_struct hang_work;
static struct wake_lock wakelock;
static struct switch_dev crash_sdev;
static struct switch_dev hang_sdev;
static int do_crash_dump = 0;

/* sysfs functions */

static ssize_t show_cdump_state(struct device *class,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", do_crash_dump);
}

static ssize_t store_cdump_state(struct device *class, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int enable;

	if (sscanf(buf, "%u", &enable) != 1)
		return -EINVAL;

	if ((enable != 1) && (enable != 0))
		return -EINVAL;

	RIL_INFO("enable: %d\n", enable);

	if (enable)
		do_crash_dump = 1;
	else
		do_crash_dump = 0;

	return strnlen(buf, count);
}

static struct device_attribute tg_modem_crash_list[] = {
	__ATTR(crash_dump_onoff, _ATTR_MODE, show_cdump_state, store_cdump_state),
	__ATTR_NULL,
};

/**
 *  Use "modem crash" to present IMC modem crash.
 *  Use "modem hang" to present Qualcomm modem crash.
 */

/* modem crash switch functions */
static ssize_t crash_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "crash_dump_det");
}

static ssize_t crash_print_state(struct switch_dev *sdev, char *buf)
{
	int state = -1;
	if (switch_get_state(sdev))
		state = 1;
	else
		state = 0;

	return sprintf(buf, "%d\n", state);
}

static int ril_crash_switch_init(void)
{
	int rc = 0;
	/* register switch class */
	crash_sdev.name = "crash_dump_det";
	crash_sdev.print_name = crash_print_name;
	crash_sdev.print_state = crash_print_state;

	rc = switch_dev_register(&crash_sdev);

	if (rc < 0)
		goto failed;

	return 0;

failed:
	return rc;
}

static void ril_crash_switch_exit(void)
{
	switch_dev_unregister(&crash_sdev);
}

static void ril_crash_dump_work(struct work_struct *work)
{
	disable_irq(gpio_to_irq(XMM_GPIO_IPC_HSIC_SUS_REQ));

	wake_lock(&wakelock);

	baseband_modem_crash_dump(0);
	msleep(200);
	gpio_set_value(USB_SW_SEL, 1);
	mdelay(5);
	gpio_set_value(MOD_VBUS_ON, 1);
	mdelay(5);
	baseband_modem_crash_dump(1);

	switch_set_state(&crash_sdev, 1);

	wake_unlock(&wakelock);
}

/* modem hang switch functions */
static ssize_t hang_print_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "mod_hang_det");
}

static ssize_t hang_print_state(struct switch_dev *sdev, char *buf)
{
	int state = -1;
	if (switch_get_state(sdev))
		state = 1;
	else
		state = 0;

	return sprintf(buf, "%d\n", state);
}

static int ril_hang_switch_init(void)
{
	int rc = 0;
	/* register switch class */
	hang_sdev.name = "mod_hang_det";
	hang_sdev.print_name = hang_print_name;
	hang_sdev.print_state = hang_print_state;

	rc = switch_dev_register(&hang_sdev);

	if (rc < 0)
		goto failed;

	return 0;

failed:
	return rc;
}

static void ril_hang_switch_exit(void)
{
	switch_dev_unregister(&hang_sdev);
}

static void ril_mod_hang_work(struct work_struct *work)
{
	switch_set_state(&hang_sdev, 1);
	hang_sdev.state = 0;
}

/* IRQ Handlers */
irqreturn_t ril_ipc_sus_req_irq(int irq, void *dev_id)
{
	int value;

	if (do_crash_dump) {
		value = gpio_get_value(XMM_GPIO_IPC_HSIC_SUS_REQ);
		if (value) {
			RIL_INFO("do_crash_dump is on!\n");
			queue_work(workqueue, &crash_work);
		}
	}
	return IRQ_HANDLED;
}

irqreturn_t ril_mod_hang_irq(int irq, void *dev_id)
{
	int value = gpio_get_value(MOD_HANG);

	if (value) {
		RIL_INFO("Modem hang!\n");
		queue_work(workqueue, &hang_work);
	}
	return IRQ_HANDLED;
}

/* init and exit functions */
int ril_modem_crash_init(struct device *target_device, struct workqueue_struct *queue)
{
	int i, err;

	workqueue = queue;
	dev = target_device;

	if (project_id == TEGRA3_PROJECT_TF300TG) {
		/* init variable */
		INIT_WORK(&crash_work, ril_crash_dump_work);
		wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "baseband_xmm_ril");

		// XMM_GPIO_IPC_HSIC_SUS_REQ is not wakeup soruce.
		// Modem will wake AP first.
		err = request_irq(gpio_to_irq(XMM_GPIO_IPC_HSIC_SUS_REQ),
			ril_ipc_sus_req_irq,
			IRQF_TRIGGER_RISING,
			"IPC_MOD_SUS_REQ",
			NULL);
		if (err < 0) {
			pr_err("%s - request irq IPC_MOD_SUS_REQ failed\n", __func__);
			goto failed_irq;
		}

		/* create sysfs */
		for (i = 0; i < (ARRAY_SIZE(tg_modem_crash_list) - 1); ++i) {
			err = device_create_file(dev, &tg_modem_crash_list[i]);
			if (err < 0) {
				RIL_ERR("%s: create file of [%d] failed, err = %d\n",
						__func__, i, err);
				goto failed_create_sysfs;
			}
		}

		/* register crash dump switch */
		err = ril_crash_switch_init();
		if (err < 0) {
			pr_err("%s - register modem crash switch failed\n",
				__func__);
			goto failed_switch_init;
		}
	} else if (project_id == TEGRA3_PROJECT_TF300TL) {
		/* init variable */
		INIT_WORK(&hang_work, ril_mod_hang_work);

		err = request_irq(gpio_to_irq(MOD_HANG),
			ril_mod_hang_irq,
			IRQF_TRIGGER_RISING,
			"MOD_HANG",
			NULL);
		if (err < 0) {
			pr_err("%s - request irq MOD_HANG failed\n", __func__);
			goto failed_irq;
		}
		tegra_pm_irq_set_wake_type(gpio_to_irq(MOD_HANG),
			IRQF_TRIGGER_RISING);
		enable_irq_wake(gpio_to_irq(MOD_HANG));

		/* register modem hang switch */
		err = ril_hang_switch_init();
		if (err < 0) {
			pr_err("%s - register modem hang switch failed\n",
				__func__);
			goto failed_switch_init;
		}
	}
	RIL_INFO("init successfully\n");
	return 0;

failed_switch_init:
	if (project_id == TEGRA3_PROJECT_TF300TG) {
		while (i >= 0) {
			device_remove_file(dev, &tg_modem_crash_list[i]);
			--i;
		}
	}
failed_create_sysfs:
	if (project_id == TEGRA3_PROJECT_TF300TG) {
		free_irq(gpio_to_irq(XMM_GPIO_IPC_HSIC_SUS_REQ), NULL);
	} else if (project_id == TEGRA3_PROJECT_TF300TL) {
		free_irq(gpio_to_irq(MOD_HANG), NULL);
	}
failed_irq:
	return err;
}

void ril_modem_crash_exit(void)
{
	int i;
	if (project_id == TEGRA3_PROJECT_TF300TG) {
		ril_crash_switch_exit();
		for (i = 0; i < (ARRAY_SIZE(tg_modem_crash_list) - 1); ++i) {
			device_remove_file(dev, &tg_modem_crash_list[i]);
		}
		free_irq(gpio_to_irq(XMM_GPIO_IPC_HSIC_SUS_REQ), NULL);
		wake_lock_destroy(&wakelock);
	} else if (project_id == TEGRA3_PROJECT_TF300TL) {
		ril_hang_switch_exit();
		free_irq(gpio_to_irq(MOD_HANG), NULL);
	}
}

