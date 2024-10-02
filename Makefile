obj-m  := gpioserdev.o
# should be this "/lib/modules/$(shell uname -r)/build
KERNELDIR ?= /lib/modules/6.6.51-v7+/build
PWD    := $(shell pwd)

all:

build:
	$(MAKE) -C $(KERNELDIR) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- M=$(PWD) modules
clean:
	$(MAKE) -C $(KERNELDIR) ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- M=$(PWD) clean