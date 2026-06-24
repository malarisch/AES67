# Out-of-tree Makefile for the AES67 FPGA Ethernet + PHC kernel module.
#
#   make                 build against the running kernel
#   make KDIR=... ARCH=... CROSS_COMPILE=...   cross-build for the target SoC
#   make regs            regenerate aes67_regs.h from the LiteX csr.csv
#   make clean

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

CSV  ?= ../../litex_soc/build/aes67_bridge/csr.csv

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

regs:
	./gen_regs.py $(CSV) > aes67_regs.h

.PHONY: default clean regs
