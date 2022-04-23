KERNEL_SOURCE=/lib/modules/$(shell uname -r)/build

obj-m += xgmac.o

all:
	make -C $(KERNEL_SOURCE) M=$(PWD) modules

clean:
	make -C $(KERNEL_SOURCE) M=$(PWD) clean

load: all
	-sudo rmmod xgmac
	sudo insmod ./xgmac.ko
