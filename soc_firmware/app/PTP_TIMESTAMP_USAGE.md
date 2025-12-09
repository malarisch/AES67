# PTP Timestamp Support for AES67

This document describes the PTP timestamping functionality added to support IEEE 1588 PTPv2 for AES67 compliance.

## Overview

The implementation consists of three components:

1. **FPGA Timestamper Module** (`FPGA/ptp_timestamper.vhd`)
   - Captures hardware timestamps for TX and RX Ethernet frames
   - Provides 64-bit nanosecond timestamps
   - Synchronizes timestamps across clock domains

2. **FPGA SPI Client Extension** (`FPGA/spi_ethernet_client.vhd`)
   - Exposes timestamps via SPI registers
   - Provides acknowledge mechanism to clear valid flags

3. **MCU Driver API** (`soc_firmware/app/drivers/eth_spi_basic/`)
   - Read TX/RX timestamps via SPI
   - Acknowledge timestamps after reading

## FPGA Implementation

### Timestamper Module

The `ptp_timestamper.vhd` module captures timestamps at the start of frame transmission/reception:

- **TX Timestamp**: Captured when `tx_enable` rises (start of transmission)
- **RX Timestamp**: Captured when `rx_frame` rises (start of reception)
- **Timestamp Counter**: Expects a 64-bit nanosecond counter input (`ptp_timestamp_ns_i`)

### SPI Register Map

New registers added to the SPI Ethernet client:

| Address | Register | Description |
|---------|----------|-------------|
| 0x03-0x0A | TX_TS[0:7] | TX timestamp (64-bit, little-endian) |
| 0x0B | TX_TS_STATUS | TX timestamp status (bit 0: valid) |
| 0x23-0x2A | RX_TS[0:7] | RX timestamp (64-bit, little-endian) |
| 0x2B | RX_TS_STATUS | RX timestamp status (bit 0: valid) |

**Note**: Writing 0x01 to the status registers acknowledges and clears the valid flag.

## MCU Driver API

### Data Structure

```c
struct eth_spi_basic_timestamp {
    uint64_t ns;      /* Nanoseconds */
    bool valid;       /* Timestamp valid flag */
};
```

### Functions

#### Read TX Timestamp
```c
int eth_spi_basic_get_tx_timestamp(const struct device *dev, 
                                    struct eth_spi_basic_timestamp *ts);
```
Reads the latest TX packet timestamp.

**Returns:**
- `0` on success
- Negative error code on failure

#### Read RX Timestamp
```c
int eth_spi_basic_get_rx_timestamp(const struct device *dev, 
                                    struct eth_spi_basic_timestamp *ts);
```
Reads the latest RX packet timestamp.

**Returns:**
- `0` on success
- Negative error code on failure

#### Acknowledge TX Timestamp
```c
int eth_spi_basic_ack_tx_timestamp(const struct device *dev);
```
Clears the TX timestamp valid flag, allowing capture of the next timestamp.

**Returns:**
- `0` on success
- Negative error code on failure

#### Acknowledge RX Timestamp
```c
int eth_spi_basic_ack_rx_timestamp(const struct device *dev);
```
Clears the RX timestamp valid flag, allowing capture of the next timestamp.

**Returns:**
- `0` on success
- Negative error code on failure

## Usage Example

```c
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <inttypes.h>
#include "../drivers/eth_spi_basic/eth_spi_basic.h"

void ptp_timestamp_example(const struct device *eth_dev)
{
    struct eth_spi_basic_timestamp tx_ts, rx_ts;
    int ret;
    
    /* Read TX timestamp after sending a packet */
    ret = eth_spi_basic_get_tx_timestamp(eth_dev, &tx_ts);
    if (ret == 0 && tx_ts.valid) {
        printk("TX timestamp: %" PRIu64 " ns\n", tx_ts.ns);
        
        /* Acknowledge to allow next timestamp capture */
        eth_spi_basic_ack_tx_timestamp(eth_dev);
    }
    
    /* Read RX timestamp after receiving a packet */
    ret = eth_spi_basic_get_rx_timestamp(eth_dev, &rx_ts);
    if (ret == 0 && rx_ts.valid) {
        printk("RX timestamp: %" PRIu64 " ns\n", rx_ts.ns);
        
        /* Acknowledge to allow next timestamp capture */
        eth_spi_basic_ack_rx_timestamp(eth_dev);
    }
}
```

## Integration with Zephyr PTP Stack

To integrate with Zephyr's PTP subsystem, you would need to:

1. **Provide a PTP clock device** that exposes:
   - `gettime()`: Read current PTP time
   - `settime()`: Set PTP time
   - `adjtime()`: Adjust PTP time
   - `adjfreq()`: Adjust PTP frequency

2. **Implement timestamp callbacks** in the Ethernet driver:
   - Hook into TX completion to capture timestamps
   - Hook into RX handler to capture timestamps
   - Pass timestamps to the PTP stack

3. **Add PTP message handling**:
   - Identify PTP packets (EtherType 0x88F7)
   - Extract sequence IDs for timestamp correlation
   - Provide timestamps to PTP stack for clock synchronization

## Hardware Requirements

The FPGA must provide:

1. **PTP Counter**: A 64-bit nanosecond counter connected to `ptp_timestamp_ns_i`
   - Should be synchronized to a stable reference clock
   - Increments at 1 ns resolution
   - Can be adjusted by PTP servo

2. **Clock Domains**: Three clock domains are used:
   - System clock (`clk_sys_i`): SPI and control logic
   - TX clock (`mac_tx_clock_i`): Ethernet MAC TX
   - RX clock (`mac_rx_clock_i`): Ethernet MAC RX

## Timestamp Accuracy

The timestamp capture has minimal latency:
- **TX**: Captured at start of preamble transmission
- **RX**: Captured at start of frame detection
- **Latency**: ~2-3 clock cycles for synchronization

For precise PTP operation, ensure:
- Stable reference clock for the timestamp counter
- Minimal jitter in the capture logic
- Consistent PHY delays (compensated in PTP stack)

## Testing

To verify timestamp functionality:

1. **Basic Test**: Check that timestamps increment
2. **TX Test**: Send packets and verify TX timestamps change
3. **RX Test**: Receive packets and verify RX timestamps change
4. **Acknowledge Test**: Verify valid flag clears after acknowledge

## References

- IEEE 1588-2008: Precision Time Protocol
- AES67-2018: High-performance streaming audio over IP interoperability standard
- Zephyr RTOS Network Stack Documentation
