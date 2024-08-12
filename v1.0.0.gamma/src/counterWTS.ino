#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

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

float stepcount = 176.00;
float circumference = 18.221;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


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
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(F("Welcome Shiven!"));
  display.println(F("Code will run shortly..."));
  display.display();
  delay(5000);

} 


void loop() {
  // put code here
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Counter A: "));
  display.println((counter_A));
  display.print(F("Counter B: "));
  display.println((counter_B));
  display.print(F("Counter C: "));
  display.println((counter_C));
  display.print(F("Counter D: "));
  display.println(counter_D);
  display.display();
  delay(100);
  /*
  Serial.print("Counter D: ");
  Serial.println(counter_D);
  */
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
  float circumference = cm; // Calculate wheel circumference in cm
  float cm_step = circumference / stepcount;  // CM per Step
  
  float f_result = cm / cm_step;  // Calculate result as a float
  result = (int) f_result; // Convert to an integer (note this is NOT rounded)
  
  return result;  // End and return result

}

void directionControlAB(float steps, int speed) {
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
