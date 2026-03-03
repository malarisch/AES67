# AES67

This project is an attempt for a full AES67 Hardware/Embedded implementation. A FPGA and MCU is used, currently for the dev setup a Cyclone 10LP and STM32H753ZI. The STM32 uses Zephyr. Some code was LLM generated, but human-checked and debugged. 

For transparency, this is primarely a learning project for me. I have never worked with FPGAs before and the only embedded experiences I've had before were not more than attaching a temperature sensor to an ESP32.

## How it works

Media Processing is offloaded onto the FPGA. The FPGA has the Ethernet PHY attached to it and uses a fork of the fork (https://github.com/xn--nding-jua/FPGA_Ethernet) of the YOL Ethernet MAC (https://github.com/yol/ethernet_mac). For communication with the MCU, the FMC peripheral of the STM32H7 is used for DMA. A Zephyr driver and the FPGA counterpart was implemented for Ethernet on the MCU via FMC and Control Signals. TODO: Only specific packet types shall be forwarded to the MCU to not overload it. (My implementation of) FMC is quite slow, so a lot of network traffic can easily overload it
On the FPGA Side I implemented PTPv2 in Leader and Follower Mode. I've modified the Eth MAC to output signals at SOF Delimiter, at which a timestamp is taken and latched (48bit Seconds, 32bit ns). From the PTP a local running wallclock (48b/32b) is disciplined. From the Wallclock the media sample counter for RTP Packets is calculated, as well as the media clocks for the audio interface. As the media clocks are quiet jittery due to the nature of integer arithmetic and my incompetence, a ppb correction signal for an external PLL is calculated. For that I use a Si5351A, that is controlled by the MCU via I2C. A custom zephyr driver for the chip was implemented (one-shot vibe coded by Claude Opus 4.6 and then manually verified by me - I'm honestly impressed).

## What is working
- Ethernet RX + TX on the FPGA
- Ethernet RX + TX on the MCU via FPGA/FMC
- Configuring Networking via MCU (Mac addr, IP addr via DHCP)
- PTPv2 Leader and Follower Mode; BMC
- Derive Media Clocks from PTP
- Si5351A Driver
- Sending RTP Audio Packets
- Single I2S Audio in
- Webserver for device config
- I2S in 48k/24bit
- Audio TX for 48k/24bit
- Audio RX for 48k/24bit
- Internal Audio Routing Matrix
- Webinterface for configuration
- FPGA and MCU reset recovery

## Todo list


- Audio Buffer (TX Path Done)
- RTCP, SDP, SAP (mDNS?) on the MCU (DONE)
- Tune PI Controller on the FPGA further (currently, it reaches a solid lock but jitters at +-30ns difference)

- FPGA optimizations - current design uses quite a lot of ressources
  - PTP implementation - use RAM for Packet generation instead of Registers
  - PTP implementation - Servo synthesizes to 1600 LUTs eventhough it is not _that_ big 
- Phase jump handling
- RGMII Support on the Ethernet MAC (my entire testing and development is done via an LAN8720 via RMII)
- Custom RMII -> MII Converter (I'm using the Altera IP for now but want the project to be as vendor independent as possible)
- FPGA bitstream upload


