# Todo

## Akut
 - 100 MBIT ETHERNET TESTEN!!!!!!!
 - RESET STRUKTUR ÜBERARBEITEN !!!!
 - Alle State machines vernünftig recovern lassen!
 - Komplette FPGA Generic Konfiguration am Control Plane Interface Exposen
 - std_logic vs std_ulogic mess überarbeiten
 - ungenutze register entfernen
 - eth_ram überarbeiten, Gowin EDA hat Probleme den RAM korrekt zu inferieren!
 - Timing für TX Packet Buffer RAMs machen Probleme (manchmal, je nachdem wie sich quartus fühlt)
 - PTP RX Packet buffer abschaffen, einfach direkt von der Wire lesen. Ist sowieso alles sequenziell.
 - Modulresets für die einzelenen FPGA Module
 - Audio TX und PTP dürfen erst anfangen zu arbeiten, wenn IP, MAC und PTP Prios konfiguriert
 - Audio TX Packet FIFO besser pipelinen
 - External MCU Ethernet RX Buffer auf Ringbuffer umbauen
 - Paketfilter für MCU strenger gestalten
 - Verifiziere PTP BMC
 - Build System
   - Alles mit Generices und generates ausstatten
     - Signal Metering
     - PTP BMC auf FPGA oder MCU
     - Ethernet für MCU
   - Verifiziere korrektes Build auf allen Boards mit verschiedenen Konfigurationen
   - Build auf Gowin
   - Build auf Lattice
   - Maybe über LiteX Builder? 
- ESP32 PSRAM Zephyr fixen
- DOKU AKTUALISIEREN!!!!!

---

## Geplant

  - Neben SPI für externe MCU auch UART - sollte relativ Plug & Play sein
  - Repo Aufräumen
  - Dumme Claude kommentare entfernen
  - TDM Mux/Demux direkt in die Audio RX/TX Pfade integrieren (einfach in echtzeit aus dem RAM rausshiften): Spart einen haufen Ressourcen und 1 Sample Latenz
  - Dynamische RX Buffer length
    - LPF für Sample interpolation!
  - weitere samplerates
  - Prüfsummen für externes MCU Interface

---

## Nice to have
  - Generic Option: PTP auf FPGA aus, auf FPGA nur PTP Pakete timestampen. Berechnungen macht die MCU und gibt die Korrekturen an die FPGA Wallclock. Grund: Ressourcen
  - Wenn SPI Mode ggf. auch Netzwerk per SLIP/UART
  - Linux Binary zur Konfiguration per SPI
  - Option: Sample Buffer auf externen RAM
