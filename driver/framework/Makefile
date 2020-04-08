CROSS_COMPILE ?= mips-linux-uclibc-gnu-
KDIR := ../../../Main/kernel
MODULE_NAME := isp_capture

all: modules

.PHONY: modules clean

EXTRA_CFLAGS += -I$(PWD)/include
$(MODULE_NAME)-objs := isvp_capture.o  \
			apical-isp/apical_dynamic_calibrations.o apical-isp/apical_static_calibrations.o \
			apical-isp/apical_calibrations.o apical-isp/apical_calibrations_init.o apical-isp/tx-isp-load-parameters.o \
			apical-isp/apical_custom_initialization.o apical-isp/ApicalDefault.o \
			apical-isp/system_isp_io.o apical-isp/log.o apical-isp/system_io.o \
			apical-isp/system_i2c.o apical-isp/system_spi.o apical-isp/system_timer.o apical-isp/system_uart.o \
			apical-isp/system_semaphore.o apical-isp/sensor_drv.o apical-isp/system_chardev.o

$(MODULE_NAME)-objs += apical-isp/libapical_isp.a

obj-m := $(MODULE_NAME).o

modules:
	@$(MAKE) -C $(KDIR) M=$(shell pwd) $@

clean:
	@rm -rf *.o *~ .depend .*.cmd  *.mod.c .tmp_versions *.ko *.symvers modules.order
