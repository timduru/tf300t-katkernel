
# Kernel makefile for motions sensors
#
#

# MPU

obj-y += mpu3050.o

ccflags-y := -DMPU_CURRENT_BUILD_MPU3050

mpu3050-objs += mldl_cfg.o
mpu3050-objs += mlsl-kernel.o
mpu3050-objs += mpuirq.o
mpu3050-objs += mpu-dev.o
mpu3050-objs += timerirq.o
mpu3050-objs += slaveirq.o
mpu3050-objs += mldl_print_cfg.o

EXTRA_CFLAGS += -Idrivers/misc/inv_mpu
EXTRA_CFLAGS += -D__C99_DESIGNATED_INITIALIZER
EXTRA_CFLAGS += -DINV_CACHE_DMP=1
obj-y			+= accel/
obj-y			+= compass/
obj-y			+= pressure/

