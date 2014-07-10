#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/switch.h>

#include <mach/board-cardhu-misc.h>

#include "baseband-xmm-power.h"
#include "ril.h"
#include "ril_proximity.h"

#define NAME_RIL_PROX "ril_proximity"

//**** external symbols

int proximity_enabled;
spinlock_t proximity_irq_lock;

//**** constants

#define _ATTR_MODE S_IRUSR | S_IWUSR | S_IRGRP


//**** local variable declaration

static struct device *dev;

static struct workqueue_struct *workqueue;
static struct work_struct prox_task;

static struct switch_dev prox_sdev;

//**** callbacks for switch device

static ssize_t print_prox_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "prox_sar_det");
}

static ssize_t print_prox_state(struct switch_dev *sdev, char *buf)
{
	int state = -1;
	if (switch_get_state(sdev))
		state = 1;
	else
		state = 0;

	return sprintf(buf, "%d\n", state);
}

static void proximity_disable_irq(void)
{
	unsigned long flags;

	spin_lock_irqsave(&proximity_irq_lock, flags);
	if (proximity_enabled) {
		disable_irq_nosync(gpio_to_irq(SAR_DET_3G));
		proximity_enabled = 0;
	}
	spin_unlock_irqrestore(&proximity_irq_lock, flags);
}

static void proximity_enable_irq(int value)
{
	int check_value;
	unsigned long flags;

	spin_lock_irqsave(&proximity_irq_lock, flags);
	if (!proximity_enabled) {
		enable_irq(gpio_to_irq(SAR_DET_3G));
		proximity_enabled = 1;
	}
	spin_unlock_irqrestore(&proximity_irq_lock, flags);

	/* check if the state of gpio is correct */
	check_value = gpio_get_value(SAR_DET_3G);
	if (value != check_value) {
		RIL_INFO("re-notify RIL with state %d\n", check_value);
		if (!check_value)
			switch_set_state(&prox_sdev, 1);
		else
			switch_set_state(&prox_sdev, 0);
	}
}

//**** IRQ event handler
int check_sar_det_3g()
{
	int value = gpio_get_value(SAR_DET_3G);

	RIL_INFO("SAR_DET_3G: %d\n", value);

	if (!value)
		switch_set_state(&prox_sdev, 1);
	else
		switch_set_state(&prox_sdev, 0);

	return value;
}

static void ril_proximity_work_handle(struct work_struct *work)
{
	int value = check_sar_det_3g();
	proximity_enable_irq(value);
}

irqreturn_t ril_proximity_interrupt_handle(int irq, void *dev_id)
{
	proximity_disable_irq();
	queue_work(workqueue, &prox_task);

	return IRQ_HANDLED;
}

//**** sysfs callback functions

static ssize_t show_prox_enabled(struct device *class,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", proximity_enabled);
}

static ssize_t store_prox_enabled(struct device *class, struct device_attribute *attr,
		const char *buf, size_t count)
{
	int enable;

	if (sscanf(buf, "%u", &enable) != 1)
		return -EINVAL;

	if ((enable != 1) && (enable != 0))
		return -EINVAL;

	RIL_INFO("enable: %d\n", enable);

	/* when enabled, report the current status immediately.
	   when disabled, set state to 0 to sync with RIL */
	if (enable) {
		queue_work(workqueue, &prox_task);
	} else {
		proximity_disable_irq();
		switch_set_state(&prox_sdev, 0);
	}

	return strnlen(buf, count);
}


//**** sysfs list

static struct device_attribute device_attr_list[] = {
	__ATTR(prox_onoff, _ATTR_MODE, show_prox_enabled, store_prox_enabled),
	__ATTR_NULL,
};


//**** initialize and finalize

int ril_proximity_init(struct device *target_device, struct workqueue_struct *queue)
{
	u32 project = tegra3_get_project_id();
	int rc = 0, i = 0;

	dev = target_device;

	proximity_enabled = 0;

	// init work queue and works
	workqueue = queue;
	INIT_WORK(&prox_task, ril_proximity_work_handle);

	// create sysfses
	for (i = 0; i < (ARRAY_SIZE(device_attr_list) - 1); ++i) {
		rc = device_create_file(dev, &device_attr_list[i]);
		if (rc < 0) {
			RIL_ERR("%s: create file of [%d] failed, err = %d\n",
					__func__, i, rc);
			goto failed_create_sysfs;
		}
	}

	/* register switch class */
	prox_sdev.name = NAME_RIL_PROX;
	prox_sdev.print_name = print_prox_name;
	prox_sdev.print_state = print_prox_state;

	rc = switch_dev_register(&prox_sdev);

	if (rc < 0)
		goto failed_register_switch_class;

	return 0;

failed_register_switch_class:
failed_create_sysfs:
	while (i >= 0) {
		device_remove_file(dev, &device_attr_list[i]);
		--i;
	}
	return rc;
}

void ril_proximity_exit(void)
{
	int i = 0;

	// destroy switch devices
	switch_dev_unregister(&prox_sdev);

	// destroy sysfses
	for (i = 0; i < (ARRAY_SIZE(device_attr_list) - 1); ++i) {
		device_remove_file(dev, &device_attr_list[i]);
	}
}

