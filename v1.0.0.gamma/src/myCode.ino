#include <SoftwareSerial.h>

// Create a software serial port
SoftwareSerial mySerial(10, 11); // RX, TX

// Buffer to store incoming data
char dataBuffer[256];
int bufferIndex = 0;

void setup() {
  Serial.begin(115200); // Initialize Serial Monitor
  mySerial.begin(115200); // Initialize software serial
  Serial.println("WT901 Sensor Data:");
}

void loop() {
  // Read incoming bytes from WT901
  while (mySerial.available() > 0) {
    char incomingByte = mySerial.read();
    dataBuffer[bufferIndex++] = incomingByte;

    // Check for end of data packet (assuming '\n' signifies end)
    if (incomingByte == '\n') {
      dataBuffer[bufferIndex] = '\0';
      parseData(dataBuffer);
      bufferIndex = 0;
    }
  }
}

void parseData(char* data) {
  float accX, accY, accZ;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;

  // Parse the data (adjust according to your data format)
  sscanf(data, "ACC:%f,%f,%f;GYRO:%f,%f,%f;MAG:%f,%f,%f",
         &accX, &accY, &accZ,
         &gyroX, &gyroY, &gyroZ,
         &magX, &magY, &magZ);

  // Print parsed data
  Serial.print("Accelerometer: X=");
  Serial.print(accX);
  Serial.print(" Y=");
  Serial.print(accY);
  Serial.print(" Z=");
  Serial.println(accZ);

  Serial.print("Gyroscope: X=");
  Serial.print(gyroX);
  Serial.print(" Y=");
  Serial.print(gyroY);
  Serial.print(" Z=");
  Serial.println(gyroZ);

  Serial.print("Magnometer: X=");
  Serial.print(magX);
  Serial.print(" Y=");
  Serial.print(magY);
  Serial.print(" Z=");
  Serial.println(,agZ);
