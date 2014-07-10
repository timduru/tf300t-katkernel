/*
 * ASUS dump last_kmsg.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/fcntl.h>
#include <asm/uaccess.h>
#include <linux/rtc.h>
#include <linux/syscalls.h>
#include <mach/iomap.h>
#include <linux/delay.h>

MODULE_DESCRIPTION("Asus dump last_kmsg");
MODULE_LICENSE("GPL");

#define DATA_LOGS		"/data/logs"
#define DATA_LOGS_RAMDUMP	"/data/logs/ramdump"
#define DATA_MEDIA_RAMDUMP	"/data/media/ramdump"
#define DATA_LOGS_LAST_KMSG	"/data/logs/last_kmsg"

struct workqueue_struct *dump_last_kmsg_work_queue=NULL;
struct work_struct dump_last_kmsg_work;

static char rd_log_file[256];
static char rd_kernel_time[256];

struct timespec ts;
struct rtc_time tm;

extern unsigned int boot_reason;

static void last_kmsg_get_time(void)
{
	getnstimeofday(&ts);
	rtc_time_to_tm(ts.tv_sec, &tm);
	sprintf(rd_kernel_time, "%d-%02d-%02d-%02d%02d%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
}

static int last_kmsg_log_filename(void)
{
	struct file *fp;
	int err;
	mm_segment_t old_fs;
	fp = filp_open(DATA_LOGS , O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);
	if (PTR_ERR(fp) == -ENOENT) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		err = sys_mkdir(DATA_LOGS,0777);
		if (err < 0) {
			set_fs(old_fs);
			return -ENOENT;
		}
		set_fs(old_fs);
		strcpy(rd_log_file, DATA_LOGS_LAST_KMSG);
	} else {
		filp_close(fp,NULL);
		strcpy(rd_log_file, DATA_LOGS_LAST_KMSG);
	}
	return 0;

}

static void last_kmsg_work_function(struct work_struct *dat)
{
	char *buffer;
	struct file *fp;
	struct file *fp_proc;
	struct file *fp_mounts;
	mm_segment_t old_fs;
	ssize_t result;
	//**************************************
	//mmcblk0p8 just for t30 branch
	//**************************************
	char mount_point[32] = "mmcblk0p8";
	#define PMC_RST_STATUS_WDT (1)
	#define PMC_RST_STATUS_SW   (3)

	printk(KERN_INFO "dump last_kmsg starting\n");

	buffer=kmalloc(SZ_1M, GFP_KERNEL);
	memset(buffer, 0, SZ_1M);

	if(!buffer){
		printk(KERN_INFO "last_kmsg_work_function:alloc buffer fail!\n");
		return;
	}

	if (last_kmsg_log_filename() < 0){
		printk(KERN_INFO "%s folder doesn't exist, and create fail !\n", DATA_LOGS);
		kfree(buffer);
		return ;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	while(1)
	{
		fp_mounts = filp_open("/proc/mounts" , O_RDONLY, 0);
		if (PTR_ERR(fp_mounts) == -ENOENT)
		{
			printk(KERN_INFO "last_kmsg_work_function:open /proc/mounts fail!\n");
			msleep(1000);
		}
		else
		{
			vfs_read(fp_mounts, buffer, SZ_1M, &fp_mounts->f_pos);

			if(!strstr(buffer, mount_point))
			{
				printk(KERN_INFO "last_kmsg_work_function:mmcblk0p8 was not mounted yet!\n");
				filp_close(fp_mounts,NULL);
				msleep(1000);
			}
			else
			{
				break;
			}
		}
	}

	memset(buffer, 0, SZ_1M);

	last_kmsg_get_time();
	strcat(rd_log_file, rd_kernel_time);

	if (boot_reason==PMC_RST_STATUS_WDT)
	{
		strcat(rd_log_file,".log_wdt");
	}
	else if (boot_reason==PMC_RST_STATUS_SW )
	{
		strcat(rd_log_file,".log_SwReboot");
	}
	else
	{
		strcat(rd_log_file,".log_normal");
	}

	fp_proc = filp_open("/proc/last_kmsg" , O_RDONLY, 0);
	if (PTR_ERR(fp_proc) == -ENOENT)
	{
		printk(KERN_INFO "last_kmsg_work_function:last_kmsg is empty!\n");
		filp_close(fp_mounts,NULL);
		set_fs(old_fs);
		kfree(buffer);
		return ;
	}

	fp = filp_open(rd_log_file , O_APPEND | O_RDWR | O_CREAT, S_IRWXU|S_IRWXG|S_IRWXO);
	if (PTR_ERR(fp) == -ENOENT)
	{
		printk(KERN_INFO "last_kmsg_work_function:open log file fail!\n");
		filp_close(fp_mounts,NULL);
		filp_close(fp_proc,NULL);
		set_fs(old_fs);
		kfree(buffer);
		return ;
	}

	result=vfs_read(fp_proc, buffer, SZ_1M, &fp_proc->f_pos);
	if( result < 0 )
	{
		printk(KERN_INFO "last_kmsg_work_function:read last_kmsg fail!\n");
	}
	else
	{
		result=vfs_write(fp, buffer, result, &fp->f_pos);
		if( result < 0 )
			printk(KERN_INFO "last_kmsg_work_function:write last_kmsg fail!\n");
	}

	filp_close(fp_mounts,NULL);
	filp_close(fp_proc,NULL);
	filp_close(fp,NULL);
	set_fs(old_fs);
	kfree(buffer);
	printk(KERN_INFO "last_kmsg file: %s\n", rd_log_file);
	return;

}

static int __init rd_init(void)
{
#ifdef CONFIG_DEBUG_ASUS
	printk(KERN_INFO "rd_init: last_kmsg_work_function\n");
	INIT_WORK(&dump_last_kmsg_work,  last_kmsg_work_function);
	dump_last_kmsg_work_queue = create_singlethread_workqueue("dump_last_kmsg_workqueue");
	queue_work(dump_last_kmsg_work_queue, &dump_last_kmsg_work);
	return 0;
#else
	//not user-debug so don't dump last_kmsg
	return 0;
#endif
}

static void __exit rd_exit(void)
{
	destroy_workqueue(dump_last_kmsg_work_queue);
}


module_init(rd_init);
module_exit(rd_exit);


