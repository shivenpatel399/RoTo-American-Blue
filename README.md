# American High School Blue Robot Tour
----------------------
All the resources used from the American High School Blue Robot Tour 2024 Season (Codes, Libraries, Logic, Files, Libraries, etc.)

## v1.0.0.alpha
----------------------
Test Code is provided to test different functions

Sensor: GY-91

Basic Movement does not use gyro (unlike Test Code) and moves the motors

**Code can work under Arduino UNO R4 Minima or Arduino GIGA R1 WIFI**

## v1.0.0.beta
----------------------

Includes all the Library Files and documents. It can be edited, but you don't need to worry. Includes Madgwick and Mahony Algorithms

Robotic Code is uploaded under this version that was used in NorCal States 2024

Sensor: GY-91

**Code can run ONLY under Arduino GIGA R1 WIFI**

## v1.0.0.ev3
----------------------
Bot Code used for EV3 Bot (Used in Mira Loma Invitational and BARSO)

Sensor: EV3 Gyro

MicroPython is used under VS Code (So make sure to install EV3 Python Extension in VS Code)

Same maze-solving and PD/PID logic as Arduino

**Only works with EV3 Mindstorms Hardware**

## v1.0.0.gamma
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

**Code can run ONLY under Arduino Due
**

## Achievements
----------------------

2nd Robot Tour **Solo** @Mira Loma Invi 2024 (Link: https://www.duosmium.org/results/2024-01-13_mira_loma_invitational_c/)

3rd Robot Tour **Solo** @BARSO 2024 (Link: https://www.duosmium.org/results/2024-02-03_nCA_bay_area_regional_c/)

5th Robot Tour **Solo** @NorCal States 2024 (Link: https://www.duosmium.org/results/2024-04-06_nCA_states_c/)
