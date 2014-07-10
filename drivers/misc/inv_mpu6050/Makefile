
# Kernel makefile for motions sensors
#
#

obj-y += mpu6050b1.o

ccflags-y := -DMPU_CURRENT_BUILD_MPU6050B1

mpu6050b1-objs += mldl_cfg.o
mpu6050b1-objs += mlsl-kernel.o
mpu6050b1-objs += mpu-dev.o
mpu6050b1-objs += mpuirq.o
mpu6050b1-objs += timerirq.o
mpu6050b1-objs += slaveirq.o
mpu6050b1-objs += mldl_print_cfg.o
mpu6050b1-objs += accel/mpu6050.o
obj-y += compass/

EXTRA_CFLAGS += -Idrivers/misc/inv_mpu6050
EXTRA_CFLAGS += -D__C99_DESIGNATED_INITIALIZER
EXTRA_CFLAGS += -DINV_CACHE_DMP=1

