obj-m += client_spi.o
client_spi-objs := zynq_client_spi.o client_spi_chrdev.o

KERN_DIR := $(KERNEL_DIR)

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C$(KERN_DIR) M=$(PWD) modules
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
