# AES67

This project is an attempt for a full AES67 Hardware/Embedded implementation. A FPGA and MCU is used, currently for the dev setup a Cyclone 10LP and STM32H753ZI. The STM32 uses Zephyr. A some boilerplate code was LLM generated. 

Roadmap:

FPGA:
- (DONE) Send Ethernet Frames
- (DONE) Receive Ethernet Frames
- (DONE) Send RTP Audio Packets
- Receive RTP Audio Packets
- (DONE) SPI Ethernet Interface, but a bit slow
- (DONE) FMC Ethernet Access for STM32H7
- (DONE) Add Timestamping to Ethernet MAC
  
- Clock Servo for Audio Clocks derived from PTPv2
- Add Control Registers to SPI
- Move MDIO to MCU

MCU
- (DONE) Custom Ethernet SPI Driver for communication to/from FPGA
- (DONE) Add Timestamping Support to SPI Driver
- Add Timestamping Support to FMC Driver
- SDP Support
- SAP Support
- mDNS Support
- Managment (Web?)interface
- FPGA Bitstream Upload
