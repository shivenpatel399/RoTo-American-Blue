# American High School Blue Robot Tour
----------------------
All the resources used from the American High School Blue Robot Tour Seasons (Codes, Libraries, Logic, Files, Libraries, etc.)

**Written by Shiven Patel**

## Designs
----------------------

I have attached 3 designs of the robots. 2 of them are from between regionals and states. The last one is my current 2025 Prototypes.

## v1.0.0.beta (2024 Season - NorCal States)
----------------------

Includes all the Library Files and documents. It can be edited, but you don't need to worry. Includes Madgwick and Mahony Algorithms

Robotic Code is uploaded under this version that was used in NorCal States 2024

Sensor: GY-91

**Code can run ONLY under Arduino GIGA R1 WIFI**

## v1.0.0.ev3 (2024 Season - Mira Loma/BARSO)
----------------------
Bot Code used for EV3 Bot (Used in Mira Loma Invitational and BARSO)

Sensor: EV3 Gyro

MicroPython is used under VS Code (So make sure to install EV3 Python Extension in VS Code)

Same maze-solving and PD/PID logic as Arduino

**Only works with EV3 Mindstorms Hardware**

## v1.0.0.gamma (2025 Season - Work In Progress)
----------------------

Logic Code to be made

Currently working on Gyro code

Sensor: BNO08x

**Wiring (v.1.0.0.gamma)**

Keep 3.3V on High:

- VCC

Keep GND on Low:

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

**Code can run ONLY under Arduino Due**

## v1.0.0.testcodes
----------------------
Test Code is provided to test different functions

Sensor: GY-91

Basic Movement does not use gyro (unlike Test Code) and moves the motors

**Code can work under Arduino UNO R4 Minima or Arduino GIGA R1 WIFI**

## Achievements
----------------------

**Here are some of my achievements:**

- 2nd at Mira Loma Invitational
- 3rd at Bay Area Regional Science Olympiad
- 5th at NorCal States Science Olympiad
