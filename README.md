# v1.0.0.alpha
----------------------
Test Code is provided to test different functions

Basic Movement does not use gyro (unlike Test Code) and moves the motors

# v1.0.0.beta
----------------------

Includes all the Library Files and documents. It can be edited, but no need. Includes Madgwick and Mahony Algorithms
Robotic Code is uploaded under this version that was used in NorCal States 2024 (5th Place Solo ggwp)

# v1.0.0.gamma
----------------------

Logic Code to be made
Currently working on Gyro code

* Wiring (v.1.0.0.gamma)

Keep 3.3V on High
- VCC
Keep GND on Low
- GND
- PS0
- PS1
Other Connections:
- SCL -> SCL (D21)
- SDA -> SDA (D20)

* Code (v.1.0.0.gamma)

Address: 0x4B (Make sure to edit Adafruit_BNO08x.h file for address 0x4A)

Baud Rate: 115200

Comment out SPI mode's CS, INT, and RST definitions

