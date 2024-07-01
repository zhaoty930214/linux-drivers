obj-m += axi_intrs.o
axi_intrs-objs := axi_intr.o empty.o

KERN_DIR := $(KERNEL_DIR)

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C$(KERN_DIR) M=$(PWD) modules
	

clean:
	make -C $(KERN_DIR) M=$(PWD) clean
	rm app

app: app_intr.o
	arm-linux-gnueabihf-gcc app_intr.c -o app