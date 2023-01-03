CONFIG_SUPPORT_ESP_SERIAL = n
CONFIG_TEST_RAW_TP := n
CONFIG_ENABLE_MONITOR_PROCESS = n

# Toolchain Path
CROSS_COMPILE := arm-linux-gnueabi-
# Linux Kernel header
KERNEL := /lib/modules/$(shell uname -r)/build
# Architecture
ARCH := arm

#Default interface is sdio
MODULE_NAME=esp32c3-spi

#Targets passed overrrides default value

ifeq ($(CONFIG_SUPPORT_ESP_SERIAL), y)
	EXTRA_CFLAGS += -DCONFIG_SUPPORT_ESP_SERIAL
endif

ifeq ($(CONFIG_TEST_RAW_TP), y)
	EXTRA_CFLAGS += -DCONFIG_TEST_RAW_TP
endif

ifeq ($(CONFIG_ENABLE_MONITOR_PROCESS), y)
	EXTRA_CFLAGS += -DCONFIG_ENABLE_MONITOR_PROCESS
endif

EXTRA_CFLAGS += -I$(PWD)/common/include -I$(PWD)


ifeq ($(MODULE_NAME), esp32c3-spi)
	EXTRA_CFLAGS += -I$(PWD)/spi
	module_objects += spi/esp_spi.o
endif

PWD := $(shell pwd)

obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := main.o esp_stats.o esp_cmd.o esp_wpa_utils.o esp_cfg80211.o $(module_objects)


all: clean
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) modules

clean:
	rm -rf *.o sdio/*.o spi/*.o *.ko
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL) M=$(PWD) clean
