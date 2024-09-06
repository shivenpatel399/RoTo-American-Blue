#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BNO08x.h>
#include <cmath>


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

float powerA = 0;
float powerB = 0;
float powerC = 0;
float powerD = 0;

float stepcount = 176.00;
float circumference = 18.221;

float integral = 0;
float derivative = 0;
float lastError = 0;
double kpGain = 0;
double kiGain = 0;
double kdGain = 0;
float Gain = 0;
float gain = 0;
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



void setup() {
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
  northplus(CMtoSteps(50),110);
  eastplus(CMtoSteps(50),110);
  southplus(CMtoSteps(50),110);
  westplus(CMtoSteps(50),110);
  
} 


void loop() {
  // put code here
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

void PDeast(int thesteps, int power, float gained) {
  counter_A = 0;
  counter_C = 0;
  targetAngle();
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
  targetAngle();
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

void eastplus(float steps, int speed) {
  counter_A = 0;
  counter_C = 0;
  while ((steps*2) > counter_A + counter_C) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
	  M1_advance(speed);
    M3_back(speed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter C: ");
    display.println(counter_C);
    display.display();
	  delay(10);
    }
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter A: ");
  display.println(counter_A);
  display.print(" Counter C: ");
  display.println(counter_C);
  display.print(" Speed: ");
  display.println(speed);
  display.display();

}

void westplus(float steps, int speed) {
  counter_A = 0;
  counter_C = 0;
  while ((steps*2) > counter_A + counter_C) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
	  M1_back(speed);
    M3_advance(speed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter C: ");
    display.println(counter_C);
    display.display();
	  delay(10);
    }
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter A: ");
  display.println(counter_A);
  display.print(" Counter C: ");
  display.println(counter_C);
  display.print(" Speed: ");
  display.println(speed);
  display.display();

}

void northplus(float steps, int speed) {
  counter_B = 0;
  counter_D = 0;
  while ((steps*2) > counter_B + counter_D) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
	  M2_back(speed);
    M4_advance(speed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Counter D: ");
    display.println(counter_D);
    display.display();
	  delay(10);
    }
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter B: ");
  display.println(counter_B);
  display.print(" Counter D: ");
  display.println(counter_D);
  display.print(" Speed: ");
  display.println(speed);
  display.display();

}

void southplus(float steps, int speed) {
  counter_B = 0;
  counter_D = 0;
  while ((steps*2) > counter_B + counter_D) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
	  M4_back(speed);
    M2_advance(speed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Counter D: ");
    display.println(counter_D);
    display.display();
	  delay(10);
    }
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter B: ");
  display.println(counter_B);
  display.print(" Counter D: ");
  display.println(counter_D);
  display.print(" Speed: ");
  display.println(speed);
  display.display();

}

void northx(float steps, int speed) {
  counter_A = 0;
  counter_B = 0;
  counter_C = 0;
  counter_D = 0;
  while ((steps*4) > counter_A + counter_B + counter_C + counter_D) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
	  M1_advance(speed);
    M2_back(speed);
    M3_back(speed);
    M4_advance(speed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Counter C: ");
    display.println(counter_C);
    display.print(" Counter D: ");
    display.println(counter_D);
    display.display();
	  delay(10);
    }
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter A: ");
  display.println(counter_A);
  display.print(" Counter B: ");
  display.println(counter_B);
  display.print(" Counter C: ");
  display.println(counter_C);
  display.print(" Counter D: ");
  display.println(counter_D);
  display.print(" Speed: ");
  display.println(speed);
  display.display();

}

void southx(float steps, int speed) {
  counter_A = 0;
  counter_B = 0;
  counter_C = 0;
  counter_D = 0;
  while ((steps*4) > counter_A + counter_B + counter_C + counter_D) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
	  M1_back(speed);
    M2_advance(speed);
    M3_advance(speed);
    M4_back(speed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Counter C: ");
    display.println(counter_C);
    display.print(" Counter D: ");
    display.println(counter_D);
    display.display();
	  delay(10);
    }
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter A: ");
  display.println(counter_A);
  display.print(" Counter B: ");
  display.println(counter_B);
  display.print(" Counter C: ");
  display.println(counter_C);
  display.print(" Counter D: ");
  display.println(counter_D);
  display.print(" Speed: ");
  display.println(speed);
  display.display();

}


void eastx(float steps, int speed) {
  counter_A = 0;
  counter_B = 0;
  counter_C = 0;
  counter_D = 0;
  while ((steps*4) > counter_A + counter_B + counter_C + counter_D) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
	  M1_advance(speed);
    M2_advance(speed);
    M3_back(speed);
    M4_back(speed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Counter C: ");
    display.println(counter_C);
    display.print(" Counter D: ");
    display.println(counter_D);
    display.display();
	  delay(10);
    }
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter A: ");
  display.println(counter_A);
  display.print(" Counter B: ");
  display.println(counter_B);
  display.print(" Counter C: ");
  display.println(counter_C);
  display.print(" Counter D: ");
  display.println(counter_D);
  display.print(" Speed: ");
  display.println(speed);
  display.display();

}

void westx(float steps, int speed) {
  counter_A = 0;
  counter_B = 0;
  counter_C = 0;
  counter_D = 0;
  while ((steps*4) > counter_A + counter_B + counter_C + counter_D) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
	  M1_back(speed);
    M2_back(speed);
    M3_advance(speed);
    M4_advance(speed);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Counter C: ");
    display.println(counter_C);
    display.print(" Counter D: ");
    display.println(counter_D);
    display.display();
	  delay(10);
    }
  stopWait();
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter A: ");
  display.println(counter_A);
  display.print(" Counter B: ");
  display.println(counter_B);
  display.print(" Counter C: ");
  display.println(counter_C);
  display.print(" Counter D: ");
  display.println(counter_D);
  display.print(" Speed: ");
  display.println(speed);
  display.display();

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

