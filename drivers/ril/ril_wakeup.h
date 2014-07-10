#ifndef _RIL_WAKEUP_H
#define _RIL_WAKEUP_H

int init_wakeup_control(struct workqueue_struct *queue);
void free_wakeup_control(void);
void ril_wakeup_suspend(void);
void ril_wakeup_resume(void);

#endif

