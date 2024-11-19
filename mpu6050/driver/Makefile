obj-m +=  debug_mpu6050.o				
debug_mpu6050-objs := mpu6050.o mpu6050_i2c.o mpu6050_config.o mpu6050_chrdev.o 

KERN_DIR := $(KERNEL_DIR)

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C$(KERN_DIR) M=$(PWD) modules

clean:
	make -C $(KERN_DIR) M=$(PWD) clean
