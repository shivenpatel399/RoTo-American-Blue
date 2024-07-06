#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MPU6050_light.h>

// Motor A connections
#define enA 13
#define in1 12
#define in2 11
// Motor B connections
#define enB 8
#define in3 10
#define in4 9

const byte MOTOR_A = 2;  // Motor 2 Interrupt Pin - INT 1 - Right Motor
const byte MOTOR_B = 3;  // Motor 1 Interrupt Pin - INT 0 - Left Motor
MPU6050 mpu(Wire1);

float stepcount = 480.00;
float circumference = 20.40;

volatile int counter_A = 0;
volatile int counter_B = 0;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  attachInterrupt(digitalPinToInterrupt (MOTOR_A), ISR_countA, RISING); // Attach interupts for encoders
  attachInterrupt(digitalPinToInterrupt (MOTOR_B), ISR_countB, RISING);
  Serial.begin(9600);
  Wire.begin();
  Wire1.begin();
  Wire.setClock(400000);
  Wire1.setClock(400000);
  mpu.begin();
  mpu.calcOffsets();
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
	// Set all the motor control pins to outputs
	pinMode(enA, OUTPUT);
	pinMode(enB, OUTPUT);
	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(in3, OUTPUT);
	pinMode(in4, OUTPUT);
	
	// Turn off motors - Initial state
	digitalWrite(in1, LOW);
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
  delay(5000);
  directionControl(844,120);


  
}

void loop() {
  //ok
}
float e = 0.0;
// This function lets you control spinning direction of motors
void directionControl(float steps, int speed) {
  counter_A = 0;
  counter_B = 0;
  while ((steps*2) > counter_A + counter_B) {
    	// Set motors to maximum speed
	  // For PWM maximum possible values are 0 to 255
    mpu.update();
    e = mpu.getAngleZ();
	  analogWrite(enA, speed); // Set PWM to 105
	  analogWrite(enB, speed);

	  // Turn on motor A & B
	  digitalWrite(in1, LOW); // Move Motors Forward
	  digitalWrite(in2, HIGH);
	  digitalWrite(in3, HIGH);
	  digitalWrite(in4, LOW);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print(" Counter A: ");
    display.println(counter_A);
    display.print(" Counter B: ");
    display.println(counter_B);
    display.print(" Angle: ");
    display.println(e);
    display.display();
	  delay(10);
    }
  mpu.update();
  analogWrite(enA, 0);
  analogWrite(enB, 0);
	digitalWrite(in1, LOW); // Stop motors
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(" Counter A: ");
  display.println(counter_A);
  display.print(" Counter B: ");
  display.println(counter_B);
  display.print(" Angle: ");
  display.println(e);
  display.print(" Speed: ");
  display.println(speed);
  display.display();



}

int CMtoSteps(float cm) {

  int result;  // Final calculation result.
  float circumference = cm; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result

}

void ISR_countA()  {
  counter_A++;  // increment Motor A counter value
} 

// Motor B pulse count ISR
void ISR_countB()  {
  counter_B++;  // increment Motor B counter value
}
