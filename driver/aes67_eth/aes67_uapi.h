/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Userspace ABI for the AES67 control char device (/dev/aes67ctl).
 *
 * Once the kernel module owns the SPI/Wishbone bus, userspace (the aes67d
 * daemon, aes67cfg) reaches the FPGA registers through these ioctls instead of
 * opening spidev directly. The Rust `KernelTransport` backend mirrors these
 * definitions — keep the two in sync.
 */
#ifndef _UAPI_AES67_IOCTL_H
#define _UAPI_AES67_IOCTL_H

#include <linux/types.h>
#include <linux/ioctl.h>

/* One Wishbone word transfer by byte address (the address straight out of the
 * LiteX csr.csv). For PEEK, @val receives the read value. */
struct aes67_wb_xfer {
	__u32 addr;
	__u32 val;
};

#define AES67_IOC_MAGIC 0xA6
#define AES67_IOC_PEEK  _IOWR(AES67_IOC_MAGIC, 1, struct aes67_wb_xfer)
#define AES67_IOC_POKE  _IOW(AES67_IOC_MAGIC, 2, struct aes67_wb_xfer)

#endif /* _UAPI_AES67_IOCTL_H */
