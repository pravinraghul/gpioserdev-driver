MODULE=gpioserdev
CROSS_COMPILER=aarch64-linux-gnu-
DTOVERLAY=gpioserdev_dtoverlay.dtbo

obj-m  := $(MODULE).o
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD    := $(shell pwd)


all:

build: dt-build mod-build

clean: dt-clean mod-clean dist-clean

load: dt-load mod-load

mod-build:
	$(MAKE) -C $(KERNELDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILER) M=$(PWD) modules

mod-clean:
	$(MAKE) -C $(KERNELDIR) ARCH=arm CROSS_COMPILE=$(CROSS_COMPILER) M=$(PWD) clean

mod-load:
	sudo insmod $(MODULE).ko

mod-unload:
	sudo rmmod $(MODULE)

%.dtbo: %.dts
	dtc -I dts -O dtb -o $@ $<

dt-build: $(DTOVERLAY)

dt-clean:
	rm *.dtb *.dtbo -rf

dt-load:
	sudo dtoverlay $(DTOVERLAY)

dist-clean:
	rm *~ -rf
