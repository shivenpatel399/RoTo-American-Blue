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

const byte MOTOR_A = 2;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 3;  // Motor 1 Interrupt Pin - INT 0 - Left Motor
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


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// Motor A connections
#define enA 13
#define in1 12
#define in2 11
// Motor B connections
#define enB 8
#define in3 10
#define in4 9
// Switch Connection
#define SWITCH_PIN 7

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu(Wire1);

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(F("Calculating offsets, do not move MPU6050"));   
  delay(1000);
  mpu.calcOffsets(); // Calculate offsets
  float offset = mpu.getGyroZoffset();
  float offseta = mpu.getAccZoffset();
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
  delay(5000); // Pause for 2 seconds
  /*
  display.print(" Gyro Z: ");
  display.println(offset);
  display.print(" Acc Z: ");
  display.println(offseta);
  display.display();
  delay(5000); // Pause for 2 seconds
  */

  /*
  while (1) {
    mpu.update();
    e = mpu.getAngleZ();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Angle: ");
    display.println(e);
    display.display();
  }
  */
  // stop();
  // PID_revised(1200, 70, 0.14, 0.0925, 0.265);
  
  //PID(1028, 50, 0.292, 0.10905, 0.31855);
  
  // slight_step(5, 80);
  // right(32);
  while (digitalRead(SWITCH_PIN) == HIGH) {
    // do nothing
  }
  delay(1000);
  halfstep();
  PD(1062, 75, 4.66);
  PD(72, 60, 4.315);
  left(32);
  PD(990, 75, 4.66);
  PD(72, 60, 4.315);
  left(32);
  PD(990, 75, 4.66);
  PD(72, 60, 4.315);
  left(32);
  PD(990, 75, 4.66);
  
  
  
  
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
void halfstep() { // Input the amount of steps robot encoder should take and its original power.
  PD(230, 50, 4.12);
  PD(230, 75, 4.66);
}
void gyro_calibration() {
  // Gyro Reset
  mpu.begin();
  mpu.calcOffsets(); // Calculate offsets
  e = 0;
}

void half() {
  const unsigned long starting_timer = millis();
  unsigned long current_timer = millis();
  PID(460, 75, 0.293, 0.1091, 0.3186);
  current_timer = millis();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Time: ");
  display.println(current_timer-starting_timer);
  display.display();

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
    powerLeft = power + (gain * thecorrection);
    powerRight = power - (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    mpu.update();
    } 
  stop();
}
void final_right() {
  PID(72, 60, 0.29, 0.10825, 0.318);
  right(32);
  PID(1006, 60, 0.29, 0.10825, 0.318);

}

void final_left() {
  
  PID(72, 60, 0.29, 0.10825, 0.318);
  left(32);
  PID(1006, 60, 0.29, 0.10825, 0.318);
  
}
/*
float total_e = 0;
void PID_distancetimetester(int time, int power, float Kp, float Ki, float Kd) { // Input the amount of steps robot encoder should take and its original power.
  // mpu.update(); // Updates Gyro Reading for new readings
  counter_A = 0; // Reset both counters for DC motors to 0
  counter_B = 0;
  const unsigned long start_millis = millis();
  unsigned long current_millis = millis();
  while ((current_millis - start_millis) < time) { // Program will run in while loop until the counters reach the same amount as the desired step.
    mpu.update();
    mock_mod_angle(float(mpu.getAngleZ()));
    e = mod_gyro_angle - float(mpu.getAngleZ()); // Takes the current gyro angle
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Error: ");
    display.println(e);
    if (e == 0) { // If the error is 0, we tell the robot to not worry about correction. Setting Error to 0 will set Integral and Derivative to 0 automatically.
      integral = 0;
    } else { // But the there is error, we compare and combine current error to the previous error
      integral = integral + e;
      total_e = total_e + 1;
    }
    display.print(" Integral: ");
    display.println(integral);
    derivative = e - lastError; // This determines the change in error over time to increase rate of change.
    display.print(" Derivative: ");
    display.println(derivative);
    correction = ((Kp * e) + (Ki * integral) + (Kd * derivative)) * -1; // Combine the determination of error, comparison and combination of error, and change in error multiply by -1 to make each motor work inversely
    display.print(" Correction: ");
    display.println(correction);
    powerLeft = power - correction; // Even if motor Left is taking positives, it will always work inversel regardless positive or negative to powerRight.
    powerRight = power + correction; // Refer back to powerLeft

    analogWrite(enA, powerLeft); // Take the changed power readings and apply them to change motor PWM for each. Numbers can be 49 and 51 (example)
    analogWrite(enB, powerRight);
      
    digitalWrite(in1, LOW); // Initilize and move the motors
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    lastError = e; // Recall this is going in a loop, so we need to cycle lastError with the loop's current Error to make improvements.
    display.print(" Last Error: ");
    display.println(lastError);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Errored: ");
    display.print(total_e);
    display.display();
    current_millis = millis();
    }
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); // Counters equal to the desired steps, we stop all motors.
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
*/
void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void time_left() {
  analogWrite(enA, 41);
  analogWrite(enB, 41);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  delay(1000);
  stop();
}

void PID(int steps, int power, float Kp, float Ki, float Kd) { // Input the amount of steps robot encoder should take and its original power.
  mpu.update(); // Updates Gyro Reading for new readings
  counter_A = 0; // Reset both counters for DC motors to 0
  counter_B = 0;
  target_angle = float(mpu.getAngleZ());
  while ((steps * 2) > counter_A + counter_B) { // Program will run in while loop until the counters reach the same amount as the desired step.
    mpu.update();
    e = target_angle - float(mpu.getAngleZ()); // Takes the current gyro angle
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Error: ");
    display.println(e);
    if (e == 0) { // If the error is 0, we tell the robot to not worry about correction. Setting Error to 0 will set Integral and Derivative to 0 automatically.
      integral = 0;
    } else { // But the there is error, we compare and combine current error to the previous error
      integral = integral + e;
      total_e = total_e + 1;
    }
    derivative = e - lastError; // This determines the change in error over time to increase rate of change.
    correction = ((Kp * e) + (Ki * integral) + (Kd * derivative)) * -1; // Combine the determination of error, comparison and combination of error, and change in error multiply by -1 to make each motor work inversely
    powerLeft = power - correction; // Even if motor Left is taking positives, it will always work inversel regardless positive or negative to powerRight.
    powerRight = power + correction; // Refer back to powerLeft

    analogWrite(enA, powerLeft); // Take the changed power readings and apply them to change motor PWM for each. Numbers can be 49 and 51 (example)
    analogWrite(enB, powerRight);
      
    digitalWrite(in1, LOW); // Initilize and move the motors
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);

    lastError = e; // Recall this is going in a loop, so we need to cycle lastError with the loop's current Error to make improvements.
    display.print(" Last Error: ");
    display.println(lastError);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Errored: ");
    display.print(total_e);
    display.display();
  stop();
  }
}


void PIDBACK(int steps, int power, float Kp, float Ki, float Kd) { // Input the amount of steps robot encoder should take and its original power.
  // mpu.update(); // Updates Gyro Reading for new readings
  counter_A = 0; // Reset both counters for DC motors to 0
  counter_B = 0;
  while ((steps * 2) > counter_A + counter_B) { // Program will run in while loop until the counters reach the same amount as the desired step.
    mpu.update();
    mock_mod_angle(float(mpu.getAngleZ()));
    e = mod_gyro_angle - float(mpu.getAngleZ()); // Takes the current gyro angle
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Error: ");
    display.println(e);
    if (e == 0) { // If the error is 0, we tell the robot to not worry about correction. Setting Error to 0 will set Integral and Derivative to 0 automatically.
      integral = 0;
    } else { // But the there is error, we compare and combine current error to the previous error
      integral = integral + e;
      total_e = total_e + 1;
    }
    display.print(" Integral: ");
    display.println(integral);
    derivative = e - lastError; // This determines the change in error over time to increase rate of change.
    display.print(" Derivative: ");
    display.println(derivative);
    correction = ((Kp * e) + (Ki * integral) + (Kd * derivative)) * -1; // Combine the determination of error, comparison and combination of error, and change in error multiply by -1 to make each motor work inversely
    display.print(" Correction: ");
    display.println(correction);
    powerLeft = power + correction; // Even if motor Left is taking positives, it will always work inversel regardless positive or negative to powerRight.
    powerRight = power - correction; // Refer back to powerLeft

    analogWrite(enA, powerLeft); // Take the changed power readings and apply them to change motor PWM for each. Numbers can be 49 and 51 (example)
    analogWrite(enB, powerRight);
      
    digitalWrite(in1, HIGH); // Initilize and move the motors
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);

    lastError = e; // Recall this is going in a loop, so we need to cycle lastError with the loop's current Error to make improvements.
    display.print(" Last Error: ");
    display.println(lastError);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Errored: ");
    display.print(total_e);
    display.display();
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW); // Counters equal to the desired steps, we stop all motors.
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
    
  }
}

void ISR_countA()  {
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  {
  counter_B++;  // increment Motor B counter value
}
void new_right(float speed, int milliseconds) {
  mpu.update();
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(milliseconds);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void right(float speed) { // 32 is official speed with 85 degrees
  mpu.update();
  float angle = mpu.getAngleZ();
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  mpu.update();
  angle = mpu.getAngleZ();
  expected_angle = mod_gyro_angle + 90;
  while (float(mpu.getAngleZ()) <  expected_angle) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    mpu.update();
  }
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
  display.display();
  delay(50);
  
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
    powerLeft = power - (gain * thecorrection);
    powerRight = power + (gain * thecorrection);
    analogWrite(enA, powerLeft);
    analogWrite(enB, powerRight);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    mpu.update();
    } 
  stop();
}
void left(int speed) {
  mpu.update();
  float angle = mpu.getAngleZ();
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  mpu.update();
  angle = mpu.getAngleZ();
  expected_angle = mod_gyro_angle - 90;
  while (float(mpu.getAngleZ()) > expected_angle) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    mpu.update();
  }
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(50);
}


void mock_mod_angle(float degrees) {
  if (degrees > -10 && degrees < 10) {
    mod_gyro_angle = 0;
  } 
  else if (degrees > 80 && degrees < 100) {
    mod_gyro_angle = 90;
  }
  else if (degrees > 170 && degrees < 190) {
    mod_gyro_angle = 180;
  }
  else if (degrees > 260 && degrees < 280)  {
    mod_gyro_angle = 270;
  }
  else if (degrees > 350 && degrees < 370) {
    mod_gyro_angle = 360;
  }
  else if (degrees < -80 && degrees > -100) {
    mod_gyro_angle = -90;
  }
  else if (degrees < -170 && degrees > -190) {
    mod_gyro_angle = -180;
  }
  else if (degrees < -260 && degrees > -280)  {
    mod_gyro_angle = -270;
  }
  else if (degrees < -350 && degrees > -370) {
    mod_gyro_angle = -360;
  }
}

/*
void ISR_GYRO_READ() {
  mpu.update();
  e = mpu.getAngleZ();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Angle: ");
  display.println(e);
  display.display();
  delay(10);
}
*/
 /*

 display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" RSeconds: ");
  display.println(remaining_seconds);
  display.print(" TDistance: ");
  display.println(total_distance);
  display.print(" TimeOverDist: ");
  display.println(timeoverdist);
  display.print(" Power Need: ");
  display.println(powerneeded);
  display.print(" Counts: ");
  display.println(counts);
  display.print(" Gs: ");
  display.print(kpGain);
  display.print(" ");
  display.print(kiGain);
  display.print(" ");
  display.println(kdGain);
  display.display();

  // Needed for Printing Debug
 */


