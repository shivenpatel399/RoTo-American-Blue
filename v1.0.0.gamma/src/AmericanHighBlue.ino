
// Libraries
#include <Arduino.h>
#include <Wire.h>    //Include wire library 
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <cmath>
#include <math.h>

// Defining Components ~ I can add timer if I want to
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
// ~ Switch Pin connection
#define SWITCH_PIN 7

// ~ Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET -1

const int E1 = 3; ///<Motor1 Speed
const int E2 = 11;///<Motor2 Speed
const int E3 = 5; ///<Motor3 Speed
const int E4 = 6; ///<Motor4 Speed

const int M1 = 4; ///<Motor1 Direction
const int M2 = 12;///<Motor2 Direction
const int M3 = 8; ///<Motor3 Direction
const int M4 = 7; ///<Motor4 Direction

const byte enA = 19;
const byte enB = 18;
const byte enC = 17;
const byte enD = 16;

volatile int counter_A = 0;
volatile int counter_B = 0;
volatile int counter_C = 0;
volatile int counter_D = 0;

bool checker = false;
unsigned long lasttime;
unsigned long thistime;

int powerLeft = 0;
int powerRight = 0;

float derivative = 0;
float integral = 0;
float total_e = 0;
float e = 0;
float lastError = 0;

float Straights = 0;
float powerneeded = 0;
float gain = 0;
float remaining_seconds = 0;
float total_distance = 0;
float distovertime = 0;
int cycle = 0;
int routeSteps[] = {11,12,22,21};

float Kproportional = 0;
float Kintegral = 0;
float Kderivative = 0;

bool moving = false;

#define BNO08X_CS 36
#define BNO08X_INT 32


// #define FAST_MODE

// For SPI mode, we also need a RESET 
#define BNO08X_RESET 34
// but not for I2C or UART
// #define BNO08X_RESET -1



struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;


float thecorrection = 0;
float correction = 0;

float powerA = 0;
float powerB = 0;
float powerC = 0;
float powerD = 0;

float stepcount = 176.00;
float circumference = 15.707;


double kpGain = 0;
double kiGain = 0;
double kdGain = 0;
float currentAngle = 0;
float target = 0;
int targetIterator = 0;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}


// Functions

void startRobot() {
  
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  attachInterrupt(digitalPinToInterrupt(enA), ISR_countA, RISING); 
  attachInterrupt(digitalPinToInterrupt(enB), ISR_countB, RISING);
  attachInterrupt(digitalPinToInterrupt(enC), ISR_countC, RISING);
  attachInterrupt(digitalPinToInterrupt (enD), ISR_countD, RISING);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  pinMode(enA, INPUT_PULLUP);
  pinMode(enB, INPUT_PULLUP);
  pinMode(enC, INPUT_PULLUP);
  pinMode(enD, INPUT_PULLUP);
  for(int i=3;i<9;i++)
    pinMode(i,OUTPUT);
  for(int i=11;i<13;i++)
    pinMode(i,OUTPUT);

  if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
    while (1) { delay(10); }
  }
  setReports(reportType, reportIntervalUs);
  delay(100);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Welcome Shiven!"));
  display.println(F("Code will run shortly..."));
  display.display();
  delay(2500);
  targetAngle();

}


void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void targetAngle() {
  for (targetIterator = 0; targetIterator < 9; targetIterator++) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    target = ypr.yaw;
  }
}

void PDnorth(int thesteps, int power, float gained) {
  counter_B = 0;
  counter_D = 0;
  lasttime = micros();
  while ((thesteps * 2) > counter_B + counter_D) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    currentAngle = float(ypr.yaw);
    thecorrection = target - currentAngle;
    powerB = power + (gained * thecorrection);
    powerD = power - (gained * thecorrection);
    M2_back(powerB);
    M4_advance(powerD);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" PowerB: ");
    display.println(powerB);
    display.print(" PowerD: ");
    display.println(powerD);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.display();
	  delay(10);
    } 
  thistime = micros();
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Expect: ");
  display.println(target);
  display.print(" Currnt: ");
  display.println(currentAngle);
  display.print(" PowerB: ");
  display.println(powerB);
  display.print(" PowerD: ");
  display.println(powerD);
  display.print(" Ang Diff: ");
  display.println(abs(target-currentAngle));
  display.print(" Time: ");
  display.println(abs(thistime-lasttime));
  display.display();
}

void PDsouth(int thesteps, int power, float gained) {
  counter_B = 0;
  counter_D = 0;
  lasttime = micros();
  while ((thesteps * 2) > counter_B + counter_D) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    currentAngle = float(ypr.yaw);
    thecorrection = target - currentAngle;
    powerB = power - (gained * thecorrection);
    powerD = power + (gained * thecorrection);
    M2_advance(powerB);
    M4_back(powerD);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" PowerB: ");
    display.println(powerB);
    display.print(" PowerD: ");
    display.println(powerD);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.display();
	  delay(10);
    } 
  thistime = micros();
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Expect: ");
  display.println(target);
  display.print(" Currnt: ");
  display.println(currentAngle);
  display.print(" PowerB: ");
  display.println(powerB);
  display.print(" PowerD: ");
  display.println(powerD);
  display.print(" Ang Diff: ");
  display.println(abs(target-currentAngle));
  display.print(" Time: ");
  display.println(abs(thistime-lasttime));
  display.display();
}

void PDeast(int thesteps, int power, float gained) {
  counter_A = 0;
  counter_C = 0;
  lasttime = micros();
  while ((thesteps * 2) > counter_A + counter_C) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    currentAngle = float(ypr.yaw);
    thecorrection = target - currentAngle;
    powerA = power - (gained * thecorrection);
    powerC = power + (gained * thecorrection);
    M1_advance(powerA);
    M3_back(powerC);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" PowerA: ");
    display.println(powerA);
    display.print(" PowerC: ");
    display.println(powerC);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.display();
	  delay(10);
    } 
  thistime = micros();
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Expect: ");
  display.println(target);
  display.print(" Currnt: ");
  display.println(currentAngle);
  display.print(" PowerA: ");
  display.println(powerA);
  display.print(" PowerC: ");
  display.println(powerC);
  display.print(" Ang Diff: ");
  display.println(abs(target-currentAngle));
  display.print(" Time: ");
  display.println(abs(thistime-lasttime));
  display.display();
}
void PDwest(int thesteps, int power, float gained) {
  counter_A = 0;
  counter_C = 0;
  lasttime = micros();
  while ((thesteps * 2) > counter_A + counter_C) {
    if (bno08x.wasReset()) {
      setReports(reportType, reportIntervalUs);
    }
  
    if (bno08x.getSensorEvent(&sensorValue)) {
      // in this demo only one report type will be received depending on FAST_MODE define (above)
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }
    }
    currentAngle = float(ypr.yaw);
    thecorrection = target - currentAngle;
    powerA = power + (gained * thecorrection);
    powerC = power - (gained * thecorrection);
    M1_back(powerA);
    M3_advance(powerC);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Expect: ");
    display.println(target);
    display.print(" Currnt: ");
    display.println(currentAngle);
    display.print(" PowerA: ");
    display.println(powerA);
    display.print(" PowerC: ");
    display.println(powerC);
    display.print(" Ang Diff: ");
    display.println(abs(target-currentAngle));
    display.display();
	  delay(10);
    } 
  thistime = micros();
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Expect: ");
  display.println(target);
  display.print(" Currnt: ");
  display.println(currentAngle);
  display.print(" PowerA: ");
  display.println(powerA);
  display.print(" PowerC: ");
  display.println(powerC);
  display.print(" Ang Diff: ");
  display.println(abs(target-currentAngle));
  display.print(" Time: ");
  display.println(abs(thistime-lasttime));
  display.display();
}



void ISR_countA()  {
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  {
  counter_B++;  // increment Motor B counter value
}

void ISR_countC()  {
  counter_C++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countD()  {
  counter_D++;  // increment Motor B counter value
}


int CMtoSteps(float cm) {

  int result;  // Final calculation result. 
  float cm_step = circumference / stepcount;  // CM per Step
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result

}

void halfstep() { // Input the amount of steps robot encoder should take and its original power. 
  // Critical Change Point
  PDnorth(CMtoSteps(15), powerneeded, gain); // Make sure to make these constants so we know what to put in time logic
}

void laststep() { // Input the amount of steps robot encoder should take and its original power. 
  // Critical Change Point
  PDsouth(CMtoSteps(25), powerneeded, gain); // Make sure to make these constants so we know what to put in time logic
}


void north() { // Critical Change Point and other parts
  if (moving == false) {
    Straights++;
  }
  else if (moving == true) {
    PDnorth(CMtoSteps(50), powerneeded, gain);
  }
}

void south() { // Critical Change Point and other parts
  if (moving == false) {
    Straights++;
  }
  else if (moving == true) {
    PDsouth(CMtoSteps(50), powerneeded, gain);
  }
}

void east() { // Critical Change Point and other parts
  if (moving == false) {
    Straights++;
  }
  else if (moving == true) {
    PDeast(CMtoSteps(50), powerneeded, gain);
  }
}

void west() { // Critical Change Point and other parts
  if (moving == false) {
    Straights++;
  }
  else if (moving == true) {
    PDwest(CMtoSteps(50), powerneeded, gain);
  }
}

void speedTime_control(float Time) { // Critical Change Point
  
  // Speed over Time control
  remaining_seconds = Time - (cycle * 0.25); // Add time for Half step
  total_distance = Straights * 50; // no half step cuz halfstep is constant
  distovertime = total_distance/remaining_seconds;
  powerneeded = -0.829 + (3.25*distovertime) + (-0.0104 * pow(distovertime,2)); // please edit this
  // counts = 1301 + (-2.64 * powerneeded) + ((-6.61 * pow(10,-3)) * pow(powerneeded,2));
  gain = 3.11 + (0.0185 * powerneeded) + ((2.91 * pow(10,-5)) * (pow(powerneeded,2))); // yea this too
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
  speedTime_control(4.5);
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
  laststep();

}

void M1_advance(char Speed) { ///<Motor1 Advance 
  digitalWrite(M1,LOW);
  analogWrite(E1,Speed);
}

void M2_advance(char Speed) { ///<Motor2 Advance 
  digitalWrite(M2,HIGH);
  analogWrite(E2,Speed);
}

void M3_advance(char Speed) { ///<Motor3 Advance
  digitalWrite(M3,LOW);
  analogWrite(E3,Speed);
}

void M4_advance(char Speed) { ///<Motor4 Advance
  digitalWrite(M4,HIGH);
  analogWrite(E4,Speed);
} 


void M1_back(char Speed) {///<Motor1 Back off
  digitalWrite(M1,HIGH);
  analogWrite(E1,Speed);
}

void M2_back(char Speed) { ///<Motor2 Back off
  digitalWrite(M2,LOW);
  analogWrite(E2,Speed);
}

void M3_back(char Speed) { ///<Motor3 Back off
  digitalWrite(M3,HIGH);
  analogWrite(E3,Speed);
}

void M4_back(char Speed) { ///<Motor4 Back off
  digitalWrite(M4,LOW);
  analogWrite(E4,Speed);
} 

void stopWait() {
  M1_advance(0);
  M2_advance(0);
  M3_advance(0);
  M4_advance(0);
  delay(500); ///<Delay 2S
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

/*

41 42 43 44
31 32 33 34
21 22 23 24
11 12 13 14

This is like the framework for our logic. its very logical. We will start coding the backup algo if neccesary. 

The steps list, that is where we input the coordinates the robot should go through. 11, 12, 22, 21 is not the actual path. 

differences from one value to another are calculated and input into Moves list. Move is a starting variable for the loop

here, we used step 1 to make the half step. But i think i should put it after the for loop

We have another for loop, where the actual action begins. If it detects either 1, -1, 10, or -10, it will perform one of the 4 functions with those 4 conditionals based on chacking the list and the gyro angle (for the 4 conditions)

The 4 conditionals matter here because robot position can vary (it can face 0, 90, 180, or 270 degrees based on the gyro) and if we don't use conditions, the robot will take these uneccesary moves and this will flag us a Stalling Penalty

Once one cycle is done, it adds i and removes the move that is first on the list, so it will be different for each cycle. The program will stop when i = len of Moves

Once we code speed, if everything goes as planned (Speed, Movement, Gates, Etc.), we should land on the target reaached at perfect time, covering 3 gates, giving us a score ranging 55-60 (Lowest Possible)

Thats the array shown of the map that is integrated into the Robot's "Brain"

I need to write speed formula. Its a matter of testing and recording data.


The main functions should look something like this in Python Format:



def blahblahblah():
    if angle is 0:
        it will do this
    elif 90:
        or that
    elif 180:
        or that
    elif 270:
        and that'
These are used for up, down, left, right.
I will finish this today or tomorrow after my bot finishes charging.




To DO
Already have done: Half Distance and PID, PID Straights and Normal Straights, Turn communication, Obstacle Detection Modify,
Need to do: Final Step Modify, Speed Modify
*/
