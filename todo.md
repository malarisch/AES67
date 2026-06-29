# Todo

## Akut
 
 - Komplette FPGA Generic Konfiguration am Control Plane Interface Exposen
 - 100 Mbit Eth auf Gigabit Phy fixen
 - Paketfilter für MCU strenger gestalten
 - Build System
   - Build auf Gowin
   - Build auf Lattice
   - Maybe über LiteX Builder? 
- ESP32 PSRAM Zephyr fixen
- DOKU AKTUALISIEREN!!!!!

---

## Geplant

  - Repo Aufräumen
  - Dumme Claude kommentare entfernen
  - TDM Mux/Demux direkt in die Audio RX/TX Pfade integrieren (einfach in echtzeit aus dem RAM rausshiften): Spart einen haufen Ressourcen und 1 Sample Latenz
  - weitere samplerates
  - Gowin weiter debuggen - irgendwo macht die Gowin EDA den Ethernet Clock tree kaputt
---

## Nice to have

  - Linux Binary zur Konfiguration per SPI
  - Option: Sample Buffer auf externen RAM
