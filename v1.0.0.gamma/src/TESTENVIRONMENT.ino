/*
TESTENVIRONMENT.ino
Shiven Patel

Copyright (c) 2024 SHIVEN PATEL

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

// This Sketch was only for test, no real code. Reference Code Logics here

#include <Wire.h>    //Include wire library 
#include <MPU6050_light.h>  //Include library for MPU communication
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define MPU6050_ADDR 0x68

const byte MOTOR_A = 3;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 2;  // Motor 1 Interrupt Pin - INT 0 - Left Motor
// const byte GYRO_INT = 4;

float stepcount = 480.00;

volatile int counter_A = 0;
volatile int counter_B = 0;
unsigned long myTime;

float mod_gyro_angle = 0;
float lastError = 0.0; // Reset Previous Error iteration to 0 (For Integration and Derivative)
float integral = 0.0; // Measures Error Growth, but is being resetted at the beginning
float derivative = 0.0;
float e = 0;
float correction = 0;
float powerLeft = 0;
float powerRight = 0;
float first_angle = 0;
float end_angle = 0;
float total_e = 0;
float expected_angle = 0;
float target_angle = 0;
float target = 0;
float gain = 0;
float cangle = 0;
float thecorrection = 0;
float theerror = 0;
float errordetector = 0;
float iterator = 0;
float angavrg = 0;
bool modAngleSetup = false; 
float modError = 0;


unsigned long lasttime;
unsigned long thistime;


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// Motor A connections
#define enA 13
#define in1 11
#define in2 12
// Motor B connections
#define enB 8
#define in3 10
#define in4 9
// Switch Connection
#define SWITCH_PIN 40

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire1, OLED_RESET);
MPU6050 mpu(Wire);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  Serial.print(F("MPU6050 status: "));
  Serial.println(F("Calculating offsets, do not move MPU6050"));   
  delay(1000);
  mpu.begin();
  mpu.calcOffsets();
  float offset = mpu.getGyroZoffset();
  float offseta = mpu.getAccZoffset();
  modError = float(modErrorDetect());
  attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING); // Attach interupts for encoders
  attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);
  // attachInterrupt(digitalPinToInterrupt (GYRO_INT), ISR_GYRO_READ, RISING);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
	// Turn off motors - Initial state
	digitalWrite(in1, LOW); // Start motors at no movement
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Welcome Shiven!"));
  display.println(F("Wait for Gyro..."));
  display.display();
  delay(2000); 
  while (digitalRead(SWITCH_PIN) == LOW) {
    mpu.update();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(mpu.getAngleZ()));
    display.display();
  }
  lasttime = micros();
  PD(317,65,4.66);
  PDBACK(70, 75, 4.66);
  thistime = micros();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Time: ");
  display.println(thistime-lasttime);
  display.display();


  // Left needs speed of 50
  // Right needs speed of 51
  
  
  
  
  
  
}

void loop() {
  // no code here
  /*
  mpu.update();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" 1st Angle: ");
  display.println(first_angle);
  display.print(" Angle: ");
  display.println(float(mpu.getAngleZ()));
  display.display();
  */
}

void gyro_calibration() {
  // Gyro Reset
  mpu.begin();
  mpu.calcOffsets(); // Calculate offsets
  e = 0;
}

void PD(int thesteps, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  mpu.update();
  mock_mod_angle(float(mpu.getAngleZ()));
  target = mod_gyro_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    mpu.update();
    thecorrection = target - float(mpu.getAngleZ());
    powerLeft = power - (gain * thecorrection);
    powerRight = power + (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(mpu.getAngleZ()));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.display();
    } 
  stop();
}

void PDBACK(int thesteps, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  mpu.update();
  mock_mod_angle(float(mpu.getAngleZ()));
  target = mod_gyro_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    mpu.update();
    thecorrection = target - float(mpu.getAngleZ());
    powerLeft = power + (gain * thecorrection);
    powerRight = power - (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(mpu.getAngleZ()));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.display();
    } 
  stop();
}

void PDTime(int time, int power, float gain) {
  counter_A = 0;
  counter_B = 0;
  mpu.update();
  mock_mod_angle(float(mpu.getAngleZ()));
  target = mod_gyro_angle;
  lasttime = micros();
  thistime = micros();
  while (time*1000000 > thistime - lasttime) {
    mpu.update();
    thecorrection = target - float(mpu.getAngleZ());
    powerLeft = power - (gain * thecorrection);
    powerRight = power + (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(float(mpu.getAngleZ()));
    display.print(" Target: ");
    display.println(target);
    display.print(" Error: ");
    display.println(thecorrection);
    display.print(" Time: ");
    display.println(thistime - lasttime);
    display.display();
    thistime = micros();
    } 
  stop();
}



void left(float speed) { // 32 is official speed with 85 degrees
  mpu.update();
  float angle = mpu.getAngleZ();
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  mpu.update();
  angle = mpu.getAngleZ();
  expected_angle = mod_gyro_angle + 88.5;
  lasttime = micros();
  thistime = micros();
  while (float(mpu.getAngleZ()) <  expected_angle) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    mpu.update();
  }
  thistime = micros();
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Angle: ");
  display.println(float(mpu.getAngleZ()));
  display.print(" Time: ");
  display.println(thistime-lasttime);
  display.display();
  delay(50);
  
}


void right(float speed) { // 32 is official speed with 85 degrees
  mpu.update();
  float angle = mpu.getAngleZ();
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  mpu.update();
  angle = mpu.getAngleZ();
  expected_angle = mod_gyro_angle - 88.5;
  lasttime = micros();
  thistime = micros();
  while (float(mpu.getAngleZ()) >  expected_angle) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    mpu.update();
  }
  thistime = micros();
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Angle: ");
  display.println(float(mpu.getAngleZ()));
  display.print(" Time: ");
  display.println(thistime-lasttime);
  display.display();
  delay(50);
  
}

void leftTime(float speed) { // 32 is official speed with 85 degrees
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  lasttime = micros();
  thistime = micros();
  while (thistime-lasttime < 1100000) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    mpu.update();
    thistime = micros();
  }
  thistime = micros();
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(50);
  
}

void rightTime(float speed) { // 32 is official speed with 85 degrees
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  lasttime = micros();
  thistime = micros();
  while (thistime-lasttime < 1100000) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    mpu.update();
    thistime = micros();
  }
  thistime = micros();
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(50);
  
}

void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}







void ISR_countA()  {
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  {
  counter_B++;  // increment Motor B counter value
}





float modErrorDetect() {
  mpu.update();
  float errorInitial = float(mpu.getAngleZ());
  return errorInitial;
}

void mock_mod_angle(float degrees) {
  if (degrees > -10 && degrees < 10) {
    mod_gyro_angle = 0 + modError;
  } 
  else if (degrees > 78.5 && degrees < 98.5) {
    mod_gyro_angle = 88.5 + modError;
  }
  else if (degrees > 167 && degrees < 187) {
    mod_gyro_angle = 177 + modError;
  }
  else if (degrees > 255.5 && degrees < 275.5)  {
    mod_gyro_angle = 265.5 + modError;
  }
  else if (degrees > 344 && degrees < 364) {
    mod_gyro_angle = 354 + modError;
  }
  else if (degrees < -78.5 && degrees > -98.5) {
    mod_gyro_angle = -88.5 + modError;
  }
  else if (degrees < -167 && degrees > -187) {
    mod_gyro_angle = -177 + modError;
  }
  else if (degrees < -256.5 && degrees > -276.5)  {
    mod_gyro_angle = -265.5 + modError;
  }
  else if (degrees < -344 && degrees > -364) {
    mod_gyro_angle = -354 + modError;
  }
}



 