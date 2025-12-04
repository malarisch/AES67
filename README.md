# AES67

Roadmap:

FPGA:
- (DONE) Send Ethernet Frames
- (DONE) Receive Ethernet Frames
- (DONE) Send RTP Audio Packets
- Receive RTP Audio Packets
- (DONE) SPI Ethernet Interface
- Add Timestamping to Ethernet MAC
- Clock Servo for Audio Clocks derived from PTPv2
- Add Control Registers to SPI
- Move MDIO to MCU

MCU
- (DONE) Custom Ethernet SPI Driver for communication to/from FPGA
- Add Timestamping Support to SPI Driver
- SDP Support
- SAP Support
- mDNS Support
- Managment (Web?)interface
- FPGA Bitstream Upload
