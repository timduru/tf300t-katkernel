#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/tty.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "include/mach/board-cardhu-misc.h"
#include "pm-irq.h"
#include "ril.h"
#include "ril_wakeup.h"
#include "../usb/serial/qcserial.h"

static struct wake_lock wakeup_control_lock;
static struct workqueue_struct *workqueue;
static struct work_struct wakeup_work;

void ril_wakeup_suspend(void)
{
	if (project_id == TEGRA3_PROJECT_TF300TL) {
		gpio_set_value(MOD_WAKE_IND, 0);
		RIL_INFO("Set MOD_WAKE_IND to %d\n", gpio_get_value(MOD_WAKE_IND));
	}
}
EXPORT_SYMBOL(ril_wakeup_suspend);

void ril_wakeup_resume(void)
{
	if (project_id == TEGRA3_PROJECT_TF300TL) {
		gpio_set_value(MOD_WAKE_IND, 1);
		RIL_INFO("Set MOD_WAKE_IND to %d\n", gpio_get_value(MOD_WAKE_IND));
	}
}
EXPORT_SYMBOL(ril_wakeup_resume);

static void wakeup_control_work(struct work_struct *work)
{
	int result = 0;
	struct usb_interface *intf = get_usb_interface();

	RIL_INFO("wake lock 2 seconds\n");
	wake_lock_timeout(&wakeup_control_lock, 2 * HZ);

	if (intf != NULL) {
		RIL_INFO("resume interface\n");
		result = usb_autopm_get_interface_async(intf);
		if (result == 0) {
			usb_autopm_put_interface_no_suspend(intf);
		}
	}
}

static irqreturn_t mod_wakeup_handler(int irq, void *dev_id)
{
	queue_work(workqueue, &wakeup_work);
	return IRQ_HANDLED;
}

static int request_wakeup_control_irq(void)
{
	int irq = 0, rc = 0;

	irq = gpio_to_irq(MOD_WAKE_AP);
	rc = request_irq(irq, mod_wakeup_handler, IRQF_TRIGGER_RISING, "mod_wake_ap", NULL);
	if (rc < 0) {
		RIL_ERR("Could not register for interrupt, irq = %d, rc = %d\n", irq, rc);
		goto failed;
	}
	return 0;

failed:
	return rc;
}

int init_wakeup_control(struct workqueue_struct *queue)
{
	int rc = 0;
	RIL_INFO("wakeup control init\n");

	/* request irq */
	rc = request_wakeup_control_irq();
	if (rc < 0)
		goto failed;

	/* set wakeup source */
	tegra_pm_irq_set_wake_type(gpio_to_irq(MOD_WAKE_AP),
			IRQF_TRIGGER_RISING);
	enable_irq_wake(gpio_to_irq(MOD_WAKE_AP));

	/* wakelock init and work init */
	workqueue = queue;
	INIT_WORK(&wakeup_work, wakeup_control_work);
	wake_lock_init(&wakeup_control_lock, WAKE_LOCK_SUSPEND, "mod_wake_ap");

	RIL_INFO("wakeup control successfully\n");
	return 0;

failed:
	return rc;
}

void free_wakeup_control(void)
{
	free_irq(gpio_to_irq(MOD_WAKE_AP), NULL);
	wake_lock_destroy(&wakeup_control_lock);
}

