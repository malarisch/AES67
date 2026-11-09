/*
 * AES70 / OCP.1 control service — public interface.
 *
 * Exposes the device (identity, analog card channels, AES67 streams,
 * PTP clock, network) as an AES70 object tree over an OCP.1 TCP server
 * (AES70-3 clause 8.4.3.1) advertised as _oca._tcp via DNS-SD.
 * Design notes: soc_firmware/docs/aes70-control.md.
 */

#ifndef AES70_H_
#define AES70_H_

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Firmware version reported through OcaDeviceManager / OcaFirmwareManager
 * (kept in step with /api/system). */
#define AES70_FW_VERSION_MAJOR 1
#define AES70_FW_VERSION_MINOR 0
#define AES70_FW_VERSION_BUILD 0
#define AES70_FW_VERSION_STR   "1.0.0"

/* AES70 standard version advertised (OcaDeviceManager.OcaVersion and the
 * DNS-SD protovers TXT record): 3 = AES70-2023. */
#define AES70_OCA_VERSION 3

/**
 * @brief Build the device model and start the OCP.1 server thread.
 *
 * Call once the stored configuration is loaded and the card manager
 * has detected the analog card (the object tree is built from what is
 * present at this point).
 *
 * @param iface  Network interface (MAC address for the network block)
 * @return 0 on success, negative errno on error
 */
int aes70_start(struct net_if *iface);

/** Address transition: the Network block's IP sensor follows it. */
void aes70_notify_ip_ready(const struct in_addr *addr);

struct aes70_status {
	bool     running;
	uint16_t port;
	int      objects;
	int      sessions;        /* connected controllers */
	int      subscriptions;
};

void aes70_get_status(struct aes70_status *st);

#ifdef __cplusplus
}
#endif

#endif /* AES70_H_ */
