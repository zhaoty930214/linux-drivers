obj-m +=  my_mpu6050.o
my_mpu6050-objs := mpu6050.o	mpu6050_i2c.o


KERN_DIR := $(KERNEL_DIR)

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C$(KERN_DIR) M=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
