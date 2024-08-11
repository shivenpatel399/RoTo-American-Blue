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

const byte enA = 22;
const byte enB = 24;
const byte enC = 26;
const byte enD = 2;

volatile int counter_A = 0;
volatile int counter_B = 0;
volatile int counter_C = 0;
volatile int counter_D = 0;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);




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

void ForwardPlus() {
  M1_advance(100);
  M3_back(100);
  delay(2000); ///<Delay 2S
  M1_advance(0);
  M3_advance(0);
  delay(2000); ///<Delay 2S
}
 
void RightPlus() {
  M2_advance(100);
  M4_back(100);
  delay(2000); ///<Delay 2S
  M2_advance(0);
  M4_advance(0);
  delay(2000); ///<Delay 2S
}

void stopWait() {
  M1_advance(0);
  M2_advance(0);
  M3_advance(0);
  M4_advance(0);
  delay(5000); ///<Delay 2S
}

void counterDetection() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(F("Counter A: "));
  display.println(F(counter_A));
  display.print(F("Counter B: "));
  display.println(F(counter_B));
  display.print(F("Counter C: "));
  display.println(F(counter_C));
  display.print(F("Counter D: "));
  display.println(F(counter_D));
  display.display();
  delay(100);
}


void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  attachInterrupt(digitalPinToInterrupt(enA), ISR_countA, RISING); 
  attachInterrupt(digitalPinToInterrupt(enB), ISR_countB, RISING);
  attachInterrupt(digitalPinToInterrupt(enC), ISR_countC, RISING);
  attachInterrupt(digitalPinToInterrupt(enD), ISR_countD, RISING);
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

void loop() {
  // put code here
  counterDetection();
} 