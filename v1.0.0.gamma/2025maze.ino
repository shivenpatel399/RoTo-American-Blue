// Libraries
#include <Wire.h>    //Include wire library 
#include <MPU6050_light.h>  //Include library for MPU communication
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <cmath>
#include <math.h>

// Defining Components ~ I can add timer if I want to
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// ~ Switch Pin connection
#define SWITCH_PIN 7
// ~ Motor A connections
#define enA 13
#define in1 12
#define in2 11
// ~ Motor B connections
#define enB 8
#define in3 10
#define in4 9
// ~ Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1
// ~ Motor Interrupt Declaration Components
const byte MOTOR_A = 2;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 3;  // Motor 1 Interrupt Pin - INT 0 - Left Motor

// Objects ~ I can add Touch Sensor later...
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
MPU6050 mpu(Wire1);

// Variables
float mod_gyro_angle = 0;
float stepcount = 480.00;
volatile int counter_A = 0;
volatile int counter_B = 0;
String Position = "North";
float mod_angle = 0;
int Turns = 0;
float Straights = 0;
bool moving = false;
float remaining_seconds = 0;
float total_distance = 0;
float speed = 0;
float e = 0;
float integral = 0;
float derivative = 0;
float lastError = 0;
double kpGain = 0;
double kiGain = 0;
double kdGain = 0;
float Gain = 0;
float gain = 0;
float target = 0;
float correction = 0;
float powerLeft = 0;
float powerRight = 0;
int cycle = 0;
int final_change = 0;
float normal = 0;
float circumference = 20.00;
int routeSteps[] = {}; // Input Steps here for Route // 14,24,23,13,12,22,21,11,21,22,32,42,41,31,41,42,32,33,34,44,43,44,34,33,23,24,14
float distovertime = 0;
float powerneeded = 0;
float counts = 0;
float total_e = 0;
float expected_angle = 0;
float thecorrection = 0;
float erroring_angle = 0;


// Functions

void startRobot() {
  
  // Start Arduino Setup
  Serial.begin(9600);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  // Gyro Calibrating
  gyro_calibration();
  // Counts Interrupts
  attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING); // Attach interupts for encoders
  attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
     
  // Initialize Motors
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

}

void gyro_calibration() { // Critical Change Point
  // Gyro Reset
  mpu.begin();
  mpu.calcOffsets(); // Calculate offsets
  e = 0;
}

void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
/*
void PID(int steps, int power, float Kp, float Ki, float Kd) { // Input the amount of steps robot encoder should take and its original power.
  mpu.update(); // Updates Gyro Reading for new readings
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
  mpu.update(); // Updates Gyro Reading for new readings
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
    derivative = e - lastError; // This determines the change in error over time to increase rate of change.
    correction = ((Kp * e) + (Ki * integral) + (Kd * derivative)) * -1; // Combine the determination of error, comparison and combination of error, and change in error multiply by -1 to make each motor work inversely
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
  stop();
    
    
  }
}
*/
void PD(int thesteps, int power, float gained) { // Critical Change Point
  counter_A = 0;
  counter_B = 0;
  mpu.update();
  mock_mod_angle(float(mpu.getAngleZ()));
  target = mod_gyro_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    mpu.update();
    thecorrection = target - float(mpu.getAngleZ());
    powerLeft = power + (gained * thecorrection);
    powerRight = power - (gained * thecorrection);
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

void PDBACK(int thesteps, int power, float gainer) { // Critical Change Point
  counter_A = 0;
  counter_B = 0;
  mpu.update();
  mock_mod_angle(float(mpu.getAngleZ()));
  target = mod_gyro_angle;
  while ((thesteps * 2) > counter_A + counter_B) {
    mpu.update();
    thecorrection = target - float(mpu.getAngleZ());
    powerLeft = power - (gainer * thecorrection);
    powerRight = power + (gainer * thecorrection);
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
void halfstep() { // Input the amount of steps robot encoder should take and its original power. 
  // Critical Change Point
  PD(260, 50, 4.12);
  PD(260, 75, 4.66);
}

void ISR_countA()  {

  // Counts for Encoder Motor A
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  {

  // Counts for Encoder Motor B
  counter_B++;  // increment Motor B counter value
}

void mock_mod_angle(float degrees) { // Critical Change Point
  if (degrees > -10 && degrees < 10) {
    mod_gyro_angle = 0;
  } 
  else if (degrees > 64 && degrees < 114) {
    mod_gyro_angle = 88.5;
  }
  else if (degrees > 153 && degrees < 203) {
    mod_gyro_angle = 177;
  }
  else if (degrees > 242 && degrees < 292)  {
    mod_gyro_angle = 265.5;
  }
  else if (degrees > 331 && degrees < 381) {
    mod_gyro_angle = 354;
  }
  else if (degrees < -64 && degrees > -114) {
    mod_gyro_angle = -88.5;
  }
  else if (degrees < -153 && degrees > -203) {
    mod_gyro_angle = -177;
  }
  else if (degrees < -242 && degrees > -292)  {
    mod_gyro_angle = -265.5;
  }
  else if (degrees < -331 && degrees > -381) {
    mod_gyro_angle = -354;
  }
}
void right(){// Critical Change Point
  PD(72, 60, 4.315);
  subRight(32);
  PD(counts-72, powerneeded, gain);
}
void subRight(int speed) { // 32 is official speed with 85 degrees
  mpu.update();
  float angle = mpu.getAngleZ();
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  mpu.update();
  angle = mpu.getAngleZ();
  expected_angle = mod_gyro_angle + 88.5; // Critical Change Point
  while (float(mpu.getAngleZ()) < expected_angle) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    mpu.update();
  }
  stop();
  delay(50);
  
}
void left() {// Critical Change Point
  PD(72, 60, 4.315);
  subLeft(32);
  PD(counts-72, powerneeded, gain);
}
void subLeft(int speed) {
  mpu.update();
  float angle = mpu.getAngleZ();
  mock_mod_angle(angle);
  analogWrite(enA, speed);
  analogWrite(enB, speed);
  mpu.update();
  angle = mpu.getAngleZ();
  expected_angle = mod_gyro_angle - 88.5; // Critical Change Point
  while (float(mpu.getAngleZ()) > expected_angle) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    mpu.update();
  }
  stop();
  delay(50);
}


void north() { // Critical Change Point and other parts

  // North Conditional 
  if (moving == false) {
    if (Position == "North") {
      Straights++;
      Position = "North";
    }
    else if (Position == "East") {
      Turns++;
      Straights++;
      Position = "North";
    }
    else if (Position == "West") {
      Turns++;
      Straights++;
      Position = "West";
    }
    else if (Position == "South") {
      Straights++;
      Position = "South";
    }
  }
  else if (moving == true) {
    if (Position == "North") {
      PD(counts, powerneeded, gain);
      Position = "North";
    }
    else if (Position == "East") {
      left();
      Position = "North";
    }
    else if (Position == "West") {
      right();
      Position = "North";
    }
    else if (Position == "South") {
      PDBACK(counts, powerneeded, gain);
      Position = "South";
    }
  }
}

void south() {

  // South Conditional
  if (moving == false) {
    if (Position == "North") {
      Straights++;
      Position = "North";
    }
    else if (Position == "East") {
      Turns++;
      Straights++;
      Position = "South";
    }
    else if (Position == "West") {
      Turns++;
      Straights++;
      Position = "South";
    }
    else if (Position == "South") {
      Straights++;
      Position = "South";
    }
  }
  else if (moving == true) {
    if (Position == "North") {
      PDBACK(counts, powerneeded, gain);
      Position = "North";
    }
    else if (Position == "East") {
      right();
      Position = "South";
    }
    else if (Position == "West") {
      left();
      Position = "South";
    }
    else if (Position == "South") {
      PD(counts, powerneeded, gain);
      Position = "South";
    }
  }
}

void east() {

  // East Conditional
  if (moving == false) {
    if (Position == "North") {
      Turns++;
      Straights++;
      Position = "East";
    }
    else if (Position == "East") {
      Straights++;
      Position = "East";
    }
    else if (Position == "West") {
      Straights++;
      Position = "West";
    }
    else if (Position == "South") {
      Turns++;
      Straights++;
      Position = "East";
    }
  }
  else if (moving == true) {
    if (Position == "North") {
      right();
      Position = "East";
    }
    else if (Position == "East") {
      PD(counts, powerneeded, gain);
      Position = "East";
    }
    else if (Position == "West") {
      PDBACK(counts, powerneeded, gain);
      Position = "West";
    }
    else if (Position == "South") {
      left();
      Position = "East";
    }
  }
}

void west() {

  // West Conditional
  if (moving == false) {
    if (Position == "North") {
      Turns++;
      Straights++;
      Position = "West";
    }
    else if (Position == "East") {
      Straights++;
      Position = "East";
    }
    else if (Position == "West") {
      Straights++;
      Position = "West";
    }
    else if (Position == "South") {
      Turns++;
      Straights++;
      Position = "West";
    }
  }
  else if (moving == true) {
    if (Position == "North") {
      left();
      Position = "West";
    }
    else if (Position == "East") {
      PDBACK(counts, powerneeded, gain);
      Position = "East";
    }
    else if (Position == "West") {
      PD(counts, powerneeded, gain);
      Position = "West";
    }
    else if (Position == "South") {
      right();
      Position = "West";
    }
  }
}

void speedTime_control(float Time) { // Critical Change Point
  
  // Speed over Time control
  remaining_seconds = Time - 2.08 - (Turns * 1.44523) - (cycle * 0.35);
  total_distance = Straights * 50;
  distovertime = total_distance/remaining_seconds;
  powerneeded = -0.829 + (3.25*distovertime) + (-0.0104 * pow(distovertime,2));
  counts = 1301 + (-2.64 * powerneeded) + ((-6.61 * pow(10,-3)) * pow(powerneeded,2));
  gain = 3.11 + (0.0185 * powerneeded) + ((2.91 * pow(10,-5)) * (pow(powerneeded,2))); 
}


void executeRobot() {
  // Actual Maze Solving Portion of the Code
  cycle = (sizeof(routeSteps)/sizeof(routeSteps[0])) - 1;
  for (int i = 0; i < cycle; i++) {
    if (routeSteps[i+1]-routeSteps[i] == 1) {
      east();
    }
    else if (routeSteps[i+1]-routeSteps[i] == -1) {
      west();
    }
    else if (routeSteps[i+1]-routeSteps[i] == 10) {
      north();
    }
    else if (routeSteps[i+1]-routeSteps[i] == -10) {
      south();
    }
  }
  moving = true;
  Position = "North";
  speedTime_control();
  // Add a timer start statement if I want to...
  while (digitalRead(SWITCH_PIN) == HIGH) {
    // do nothing
  }
  delay(750);
  halfstep();
  for (int j = 0; j < cycle; j++) {
    delay(350); // Increase Delay for accurate angle stop, tradeoff is speed. Make it around 400-500
    if (routeSteps[j+1]-routeSteps[j] == 1) {
      east();
    }
    else if (routeSteps[j+1]-routeSteps[j] == -1) {
      west();
    }
    else if (routeSteps[j+1]-routeSteps[j] == 10) {
      north();
    }
    else if (routeSteps[j+1]-routeSteps[j] == -10) {
      south();
    }

  }

}

/* 

Tasks:
*/

void setup() { 
  
  // Basic Void Setup Function for Arduino (Required to run)
  // Wire and Starting the Whole Robot in one function.
  startRobot();
  // Start the Code
  executeRobot();
  
  
  
}

void loop() {
  /*
   Basic code to run repeatedly
  */
}
