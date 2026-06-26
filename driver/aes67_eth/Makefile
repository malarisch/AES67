# Out-of-tree Makefile for the AES67 FPGA Ethernet + PHC kernel module.
#
#   make                 build against the running kernel
#   make KDIR=... ARCH=... CROSS_COMPILE=...   cross-build for the target SoC
#   make dtbo            compile the device-tree overlay (aes67-overlay.dtbo)
#   make regs            regenerate aes67_regs.h from the LiteX csr.csv
#   make install         build + install the module, overlay and autoload entry
#   make uninstall       remove what `install` placed
#   make clean
#
# `install`/`uninstall` write under /lib/modules, /boot and /etc — run them as
# root (sudo). For a cross/staged install, set INSTALL_MOD_PATH (and matching
# OVERLAY_DIR) to a staging root, e.g.:
#   make install KDIR=<target-headers> ARCH=arm64 \
#       CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=/srv/rootfs

KDIR ?= /lib/modules/$(shell uname -r)/build
PWD  := $(shell pwd)

CSV  ?= ../../litex_soc/build/aes67_bridge/csr.csv

# Device-tree overlay (source -> compiled blob).
DTC     ?= dtc
DTS     := dts/aes67-overlay.dts
DTBO    := dts/aes67-overlay.dtbo

# Persistent install locations. INSTALL_MOD_PATH stages the module tree (passed
# through to the kernel's modules_install); DESTDIR stages the overlay/autoload
# files the same way. OVERLAY_DIR autodetects the Raspberry Pi boot layout.
INSTALL_MOD_PATH ?=
DESTDIR          ?= $(INSTALL_MOD_PATH)
OVERLAY_DIR      ?= $(shell test -d /boot/firmware/overlays && echo /boot/firmware/overlays || echo /boot/overlays)
MODLOAD_DIR      := /etc/modules-load.d
MODULE           := aes67_eth_drv
# Kernel release this module is installed for (the <ver> in /lib/modules/<ver>),
# derived from KDIR so it is correct for both native and cross installs.
KRELEASE         ?= $(notdir $(patsubst %/,%,$(dir $(KDIR))))

default:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

dtbo: $(DTBO)

$(DTBO): $(DTS)
	$(DTC) -@ -I dts -O dtb -o $@ $<

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f $(DTBO)

regs:
	./gen_regs.py $(CSV) > aes67_regs.h

# Build everything, then install it persistently:
#  * the .ko into the kernel module tree (+ depmod, via modules_install),
#  * the compiled overlay into the boot overlay directory,
#  * an /etc/modules-load.d entry so the module loads on every boot.
install: default $(DTBO)
	$(MAKE) -C $(KDIR) M=$(PWD) INSTALL_MOD_PATH=$(INSTALL_MOD_PATH) modules_install
	# The kernel's modules_install skips depmod when the headers package ships no
	# System.map (e.g. Raspberry Pi OS). Run it ourselves so modprobe — and thus
	# the boot-time autoload below — can actually resolve the module by name.
	depmod $(if $(INSTALL_MOD_PATH),-b $(INSTALL_MOD_PATH)) $(KRELEASE)
	install -d $(DESTDIR)$(OVERLAY_DIR)
	install -m 0644 $(DTBO) $(DESTDIR)$(OVERLAY_DIR)/aes67-overlay.dtbo
	install -d $(DESTDIR)$(MODLOAD_DIR)
	echo $(MODULE) > $(DESTDIR)$(MODLOAD_DIR)/aes67.conf
	@echo
	@echo "Installed module '$(MODULE)' (for kernel $(KRELEASE)) and overlay aes67-overlay.dtbo."
	@echo "Ensure 'dtoverlay=aes67-overlay' is in config.txt, then reboot."

uninstall:
	rm -f $(DESTDIR)$(OVERLAY_DIR)/aes67-overlay.dtbo
	rm -f $(DESTDIR)$(MODLOAD_DIR)/aes67.conf
	rm -f $(INSTALL_MOD_PATH)/lib/modules/$(KRELEASE)/updates/$(MODULE).ko*
	depmod $(if $(INSTALL_MOD_PATH),-b $(INSTALL_MOD_PATH)) $(KRELEASE)
	@echo "Removed overlay, autoload entry and module for kernel $(KRELEASE)."

.PHONY: default dtbo clean regs install uninstall
