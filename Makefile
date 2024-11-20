MODULE=gpioserdev
CROSS_COMPILER=aarch64-linux-gnu-

obj-m  := $(MODULE).o
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD    := $(shell pwd)

all: build

build:
	$(MAKE) -C $(KERNELDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILER) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNELDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILER) M=$(PWD) clean

load:
	sudo insmod $(MODULE).ko

unload:
	sudo rmmod $(MODULE)

dist-clean:
	rm *~ -rf