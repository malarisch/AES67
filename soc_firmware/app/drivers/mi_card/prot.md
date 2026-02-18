I2C Protokoll-Analyse (MI Card - 8-Kanal ADC Board)
I2C-Geräte auf dem Bus
Gerät	Adresse	Adressierung	Beschreibung
LPC Microcontroller	0x40	1-Byte	Preamp-Steuerung (Gain, Phantom, HPF)
DSP Chip	0x14	2-Byte (Big-Endian)	Audio Processing
LPC Register-Map
Register	Adresse	Beschreibung
LPC_CHN_BASE	0x30	Kanalregister (2 Bytes pro Kanal)
LPC_GLB_REG	0x40	Globales Steuerregister
LPC_SOFT_ID	0x70	Software ID
LPC_BOARD_ID	0x71	Board ID (0x01 für MI)
LPC_HARD_REV	0x72	Hardware Revision
Kanal-Register Format (2 Bytes bei 0x30 + 2*channel)
High Byte:

Bit 0: Attenuation (PAD -20dB)
Bit 1: Phantom Power (48V)
Bit 6: Common Mode
Bit 7: DC Coupling
Low Byte:

Bits 0-5: Gain (0-63)
Global Register (0x40)
Bit 3 (0x08): NRST - Reset Release
Bit 6 (0x40): F96KHZ - 96 kHz Mode
Bit 7 (0x80): HPF - High-Pass Filter
Gain-Tabelle
Der Gain reicht von -6 dB bis +66 dB (1 dB Schritte):

Index 0-23: Mit PAD (Attenuation ON)
Index 24-72: Ohne PAD (Attenuation OFF)