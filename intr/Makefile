obj-m += axi_intrs.o
axi_intrs-objs := axi_intr.o

KERN_DIR := /home/mevion/work/linux_learn/linux_source_path/modules_linux-xlnx-xilinx-v2021.2/linux-xlnx-xilinx-v2021.2

all:
	make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C$(KERN_DIR) M=$(PWD) modules

clean:
	make -C $(KERN_DIR) M=$(PWD) clean
