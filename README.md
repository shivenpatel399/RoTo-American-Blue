# v1.0.0.alpha


# Wiring (v.1.0.0.gamma)

Keep 3.3V on High
- VCC
Keep GND on Low
- GND
- PS0
- PS1
Other Connections:
- SCL -> SCL (D21)
- SDA -> SDA (D20)

# Code (v.1.0.0.gamma)

Address: 0x4B (Make sure to edit Adafruit_BNO08x.h file for address 0x4A)

Baud Rate: 115200

Comment out SPI mode's CS, INT, and RST definitions

