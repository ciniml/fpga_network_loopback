.PHONY: all modules clean load

LINUX_DIR ?= /lib/modules/$(shell uname -r)/build

obj-m += xgmac.o

all: modules

modules:
	make -C $(LINUX_DIR) M=$(PWD) modules

modules_install:
	make -C $(LINUX_DIR) M=$(PWD) modules

clean:
	make -C $(LINUX_DIR) M=$(PWD) clean

load: all
	-sudo rmmod xgmac
	sudo insmod ./xgmac.ko
