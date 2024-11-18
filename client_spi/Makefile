#obj-m += client_spi.o
obj-m +=  my_mpu6050.o
#obj-m += st7796u.o
client_spi-objs := zynq_client_spi.o client_spi_chrdev.o
st7796u-objs := spi_screen.o
my_mpu6050-objs := mpu6050.o


KERN_DIR := $(KERNEL_DIR)

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C$(KERN_DIR) M=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
