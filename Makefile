CONFIG_SUPPORT_ESP_SERIAL = n
CONFIG_TEST_RAW_TP := n
CONFIG_ENABLE_MONITOR_PROCESS = n

# Toolchain Path
CROSS_COMPILE := arm-linux-gnueabihf-
# Linux Kernel header
KERNEL := /lib/modules/$(shell uname -r)/build
# Architecture
ARCH := arm

#Default interface is sdio
MODULE_NAME=esp32c3-spi


EXTRA_CFLAGS += -I$(PWD)/include -I$(PWD)

EXTRA_CFLAGS += -I$(PWD)/spi
module_objects += spi/esp_spi.o

PWD := $(shell pwd)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := main.o esp_stats.o $(module_objects)


all: clean
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o sdio/*.o spi/*.o *.ko
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean
