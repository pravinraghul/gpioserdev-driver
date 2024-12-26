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
	rm -f *.o *.ko *.mod.* *.symvers *.order $(MODULE).dtbo

load: load-dtbo mod-load chmod

mod-load:
	sudo insmod $(MODULE).ko

unload:
	sudo rmmod $(MODULE)

chmod:
	sudo chmod 766 /dev/$(MODULE)
	sudo chmod 766 /sys/module/$(MODULE)/parameters/delay_us
	sudo chmod 766 /sys/module/$(MODULE)/parameters/data_order

$(MODULE).dtbo: $(MODULE).dts
	dtc -I dts -O dtb -o $(MODULE).dtbo $(MODULE).dts

load-dtbo: $(MODULE).dtbo
	sudo dtoverlay $(MODULE).dtbo

dist-clean:
	rm *~ -rf
