/*
 * AES70 device model builder (internal to the aes70 module).
 */

#ifndef AES70_DEVICE_H_
#define AES70_DEVICE_H_

#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Reset the core and populate the object tree from the current device
 * state (detected card, stream tables, configuration). */
int aes70_device_build(struct net_if *iface);

/* Node IP for the Network block's IPAddress sensor. */
void aes70_device_set_ip(const struct in_addr *addr);

#ifdef __cplusplus
}
#endif

#endif /* AES70_DEVICE_H_ */
