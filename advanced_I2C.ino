/*
Advanced_I2C.ino
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

#include "MPU9250.h"
#include "SensorFusion.cpp" //SF

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;

SF fusion;

float gx, gy, gz, ax, ay, az, mx, my, mz;
float pitch, roll, yaw;
float deltat;


void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}

void loop() {
  // now read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD

  IMU.readSensor();

  ax = IMU.getAccelX_mss();
  Serial.print(ax ,6);
  Serial.print("\t");
  ay = IMU.getAccelY_mss();
  Serial.print(ay,6);
  Serial.print("\t");
  az = IMU.getAccelZ_mss();
  Serial.print(az,6);
  Serial.print("\t");
  gx = IMU.getGyroX_rads();
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  gy = IMU.getGyroY_rads();
  Serial.print(gy,6);
  Serial.print("\t");
  gz = IMU.getGyroZ_rads();
  Serial.print(gz,6);
  Serial.print("\t");
  mx = IMU.getMagX_uT();
  Serial.print(mx,6);
  Serial.print("\t");
  my = IMU.getMagY_uT();
  Serial.print(my,6);
  Serial.print("\t");
  mz = IMU.getMagZ_uT();
  Serial.print(mz,6);
  Serial.print("\t");
  delay(20);


  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update

  // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  yaw = fusion.getYaw();

  Serial.print("Pitch:\t"); Serial.println(pitch);
  Serial.print("Roll:\t"); Serial.println(roll);
  Serial.print("Yaw:\t"); Serial.println(yaw);
  Serial.println();
}