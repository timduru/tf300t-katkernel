#ifndef RIL_PROXIMITY_H
#define RIL_PROXIMITY_H

extern spinlock_t proximity_irq_lock;
extern int proximity_enabled;

int ril_proximity_init(struct device *target_device, struct workqueue_struct *queue);
void ril_proximity_exit(void);
irqreturn_t ril_proximity_interrupt_handle(int irq, void *dev_id);
int check_sar_det_3g();

#endif
