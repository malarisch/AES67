#ifndef IEEE1588_UTILS_H
#define IEEE1588_UTILS_H

#include <stdint.h>

/**
 * @brief Build EUI-64 clock identity from 48-bit MAC address.
 *
 * IEEE 1588 Section 7.5.2.2.2:
 *   Clock ID = MAC[0] (with U/L bit toggled) | MAC[1] | MAC[2]
 *              | 0xFF | 0xFE | MAC[3] | MAC[4] | MAC[5]
 */
static inline void mac_to_clock_identity(const uint8_t mac[6],
					 uint8_t clock_id[8])
{
	clock_id[0] = mac[0] ^ 0x02;
	clock_id[1] = mac[1];
	clock_id[2] = mac[2];
	clock_id[3] = 0xFF;
	clock_id[4] = 0xFE;
	clock_id[5] = mac[3];
	clock_id[6] = mac[4];
	clock_id[7] = mac[5];
}

/**
 * @brief Convert Ethernet speed code to human-readable string.
 *
 * @param speed_code  0=10M, 1=100M, 2=1G
 * @return Static string representation
 */
static inline const char *eth_speed_to_text(uint8_t speed_code)
{
	switch (speed_code) {
	case 0:  return "10M";
	case 1:  return "100M";
	case 2:  return "1G";
	default: return "?";
	}
}

#endif /* IEEE1588_UTILS_H */
