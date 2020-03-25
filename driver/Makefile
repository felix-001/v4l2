CROSS_COMPILE ?= mips-linux-uclibc-gnu-
KDIR := ../../Main/kernel
MODULE_NAME := isp_capture

all: modules

.PHONY: modules clean

$(MODULE_NAME)-objs := isvp_capture.o
obj-m := $(MODULE_NAME).o

modules:
	@$(MAKE) -C $(KDIR) M=$(shell pwd) $@

clean:
	@rm -rf *.o *~ .depend .*.cmd  *.mod.c .tmp_versions *.ko *.symvers modules.order
