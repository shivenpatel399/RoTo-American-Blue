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

#include <Wire.h>
#include <SPI.h>
#define betaDef 0.1f        	// 2 * proportional gain
#define twoKpDef (2.0f * 0.5f)	// 2 * proportional gain
#define twoKiDef (2.0f * 0.0f)	// 2 * integral gain

static float invSqrt(float x);
void vectorCross(float A[3], float B[3], float cross[3]);
void computeAngles();
float roll, pitch, yaw;
int status;
float gx, gy, gz, ax, ay, az, mx, my, mz;
float Now,lastUpdate,deltat;
float _copyQuat[4];	// copy buffer to protect the quaternion values since getters!=setters
float beta = betaDef;		
float twoKp = twoKpDef;	
float twoKi = twoKiDef;	
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;
bool anglesComputed = 0;
uint8_t _address;
TwoWire *_i2c;
const uint32_t _i2cRate = 400000; // 400 kHz
size_t _numBytes; // number of bytes received from I2C
 // spi
SPIClass *_spi;
uint8_t _csPin;
bool _useSPI;  bool _useSPIHS;
const uint8_t SPI_READ = 0x80;
const uint32_t SPI_LS_CLOCK = 1000000;  // 1 MHz
const uint32_t SPI_HS_CLOCK = 15000000; // 15 MHz
// track success of interacting with sensor
int _status;
// buffer for reading from sensor
uint8_t _buffer[21];
// data counts
int16_t _axcounts,_aycounts,_azcounts;
int16_t _gxcounts,_gycounts,_gzcounts;
int16_t _hxcounts,_hycounts,_hzcounts;
int16_t _tcounts;
// data buffer
float _ax, _ay, _az;
float _gx, _gy, _gz;
float _hx, _hy, _hz;
float _t;
// wake on motion
uint8_t _womThreshold;
// scale factors
float _accelScale;
float _gyroScale;
float _magScaleX, _magScaleY, _magScaleZ;
const float _tempScale = 333.87f;
const float _tempOffset = 21.0f;
// configuration
enum GyroRange
{
  GYRO_RANGE_250DPS,
  GYRO_RANGE_500DPS,
  GYRO_RANGE_1000DPS,
  GYRO_RANGE_2000DPS
};
enum AccelRange
{
  ACCEL_RANGE_2G,
  ACCEL_RANGE_4G,
  ACCEL_RANGE_8G,
  ACCEL_RANGE_16G    
};
enum DlpfBandwidth
{
  DLPF_BANDWIDTH_184HZ,
  DLPF_BANDWIDTH_92HZ,
  DLPF_BANDWIDTH_41HZ,
  DLPF_BANDWIDTH_20HZ,
  DLPF_BANDWIDTH_10HZ,
  DLPF_BANDWIDTH_5HZ
};
enum LpAccelOdr
{
  LP_ACCEL_ODR_0_24HZ = 0,
  LP_ACCEL_ODR_0_49HZ = 1,
  LP_ACCEL_ODR_0_98HZ = 2,
  LP_ACCEL_ODR_1_95HZ = 3,
  LP_ACCEL_ODR_3_91HZ = 4,
  LP_ACCEL_ODR_7_81HZ = 5,
  LP_ACCEL_ODR_15_63HZ = 6,
  LP_ACCEL_ODR_31_25HZ = 7,
  LP_ACCEL_ODR_62_50HZ = 8,
  LP_ACCEL_ODR_125HZ = 9,
  LP_ACCEL_ODR_250HZ = 10,
  LP_ACCEL_ODR_500HZ = 11
};
AccelRange _accelRange;
GyroRange _gyroRange;
DlpfBandwidth _bandwidth;
uint8_t _srd;
// gyro bias estimation
size_t _numSamples = 100;
double _gxbD, _gybD, _gzbD;
float _gxb, _gyb, _gzb;
// accel bias and scale factor estimation
double _axbD, _aybD, _azbD;
float _axmax, _aymax, _azmax;
float _axmin, _aymin, _azmin;
float _axb, _ayb, _azb;
float _axs = 1.0f;
float _ays = 1.0f;
float _azs = 1.0f;
// magnetometer bias and scale factor estimation
uint16_t _maxCounts = 1000;
float _deltaThresh = 0.3f;
uint8_t _coeff = 8;
uint16_t _counter;
float _framedelta, _delta;
float _hxfilt, _hyfilt, _hzfilt;
float _hxmax, _hymax, _hzmax;
float _hxmin, _hymin, _hzmin;
float _hxb, _hyb, _hzb;
float _hxs = 1.0f;
float _hys = 1.0f;
float _hzs = 1.0f;
float _avgs;
// transformation matrix
/* transform the accel and gyro axes to match the magnetometer axes */
const int16_t tX[3] = {0,  1,  0}; 
const int16_t tY[3] = {1,  0,  0};
const int16_t tZ[3] = {0,  0, -1};
// constants
const float G = 9.807f;
const float _d2r = 3.14159265359f/180.0f;
// MPU9250 registers
const uint8_t ACCEL_OUT = 0x3B;
const uint8_t GYRO_OUT = 0x43;
const uint8_t TEMP_OUT = 0x41;
const uint8_t EXT_SENS_DATA_00 = 0x49;
const uint8_t ACCEL_CONFIG = 0x1C;
const uint8_t ACCEL_FS_SEL_2G = 0x00;
const uint8_t ACCEL_FS_SEL_4G = 0x08;
const uint8_t ACCEL_FS_SEL_8G = 0x10;
const uint8_t ACCEL_FS_SEL_16G = 0x18;
const uint8_t GYRO_CONFIG = 0x1B;
const uint8_t GYRO_FS_SEL_250DPS = 0x00;
const uint8_t GYRO_FS_SEL_500DPS = 0x08;
const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
const uint8_t GYRO_FS_SEL_2000DPS = 0x18;
const uint8_t ACCEL_CONFIG2 = 0x1D;
const uint8_t ACCEL_DLPF_184 = 0x01;
const uint8_t ACCEL_DLPF_92 = 0x02;
const uint8_t ACCEL_DLPF_41 = 0x03;
const uint8_t ACCEL_DLPF_20 = 0x04;
const uint8_t ACCEL_DLPF_10 = 0x05;
const uint8_t ACCEL_DLPF_5 = 0x06;
const uint8_t CONFIG = 0x1A;
const uint8_t GYRO_DLPF_184 = 0x01;
const uint8_t GYRO_DLPF_92 = 0x02;
const uint8_t GYRO_DLPF_41 = 0x03;
const uint8_t GYRO_DLPF_20 = 0x04;
const uint8_t GYRO_DLPF_10 = 0x05;
const uint8_t GYRO_DLPF_5 = 0x06;
const uint8_t SMPDIV = 0x19;
const uint8_t INT_PIN_CFG = 0x37;
const uint8_t INT_ENABLE = 0x38;
const uint8_t INT_DISABLE = 0x00;
const uint8_t INT_PULSE_50US = 0x00;
const uint8_t INT_WOM_EN = 0x40;
const uint8_t INT_RAW_RDY_EN = 0x01;
const uint8_t PWR_MGMNT_1 = 0x6B;
const uint8_t PWR_CYCLE = 0x20;
const uint8_t PWR_RESET = 0x80;
const uint8_t CLOCK_SEL_PLL = 0x01;
const uint8_t PWR_MGMNT_2 = 0x6C;
const uint8_t SEN_ENABLE = 0x00;
const uint8_t DIS_GYRO = 0x07;
const uint8_t USER_CTRL = 0x6A;
const uint8_t I2C_MST_EN = 0x20;
const uint8_t I2C_MST_CLK = 0x0D;
const uint8_t I2C_MST_CTRL = 0x24;
const uint8_t I2C_SLV0_ADDR = 0x25;
const uint8_t I2C_SLV0_REG = 0x26;
const uint8_t I2C_SLV0_DO = 0x63;
const uint8_t I2C_SLV0_CTRL = 0x27;
const uint8_t I2C_SLV0_EN = 0x80;
const uint8_t I2C_READ_FLAG = 0x80;
const uint8_t MOT_DETECT_CTRL = 0x69;
const uint8_t ACCEL_INTEL_EN = 0x80;
const uint8_t ACCEL_INTEL_MODE = 0x40;
const uint8_t LP_ACCEL_ODR = 0x1E;
const uint8_t WOM_THR = 0x1F;
const uint8_t WHO_AM_I = 0x75;
const uint8_t FIFO_EN = 0x23;
const uint8_t FIFO_TEMP = 0x80;
const uint8_t FIFO_GYRO = 0x70;
const uint8_t FIFO_ACCEL = 0x08;
const uint8_t FIFO_MAG = 0x01;
const uint8_t FIFO_COUNT = 0x72;
const uint8_t FIFO_READ = 0x74;
// AK8963 registers
const uint8_t AK8963_I2C_ADDR = 0x0C;
const uint8_t AK8963_HXL = 0x03; 
const uint8_t AK8963_CNTL1 = 0x0A;
const uint8_t AK8963_PWR_DOWN = 0x00;
const uint8_t AK8963_CNT_MEAS1 = 0x12;
const uint8_t AK8963_CNT_MEAS2 = 0x16;
const uint8_t AK8963_FUSE_ROM = 0x0F;
const uint8_t AK8963_CNTL2 = 0x0B;
const uint8_t AK8963_RESET = 0x01;
const uint8_t AK8963_ASA = 0x10;
const uint8_t AK8963_WHO_AM_I = 0x00;
// fifo
bool _enFifoAccel,_enFifoGyro,_enFifoMag,_enFifoTemp;
size_t _fifoSize,_fifoFrameSize;
float _axFifo[85], _ayFifo[85], _azFifo[85];
size_t _aSize;
float _gxFifo[85], _gyFifo[85], _gzFifo[85];
size_t _gSize;
float _hxFifo[73], _hyFifo[73], _hzFifo[73];
size_t _hSize;
float _tFifo[256];
size_t _tSize;


int begin(){
  if( _useSPI ) { // using SPI for communication
    // use low speed SPI for register setting
    _useSPIHS = false;
    // setting CS pin to output
    pinMode(_csPin,OUTPUT);
    // setting CS pin high
    digitalWrite(_csPin,HIGH);
    // begin SPI communication
    _spi->begin();
  } else { // using I2C for communication
    // starting the I2C bus
    _i2c->begin();
    // setting the I2C clock
    _i2c->setClock(_i2cRate);
  }
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -1;
  }
  // enable I2C master mode
  if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
    return -2;
  }
  // set the I2C bus speed to 400 kHz
  if(writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
    return -3;
  }
  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
  writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  delay(1);
  // reset the AK8963
  writeAK8963Register(AK8963_CNTL2,AK8963_RESET);
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -4;
  }
  // check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
  if((whoAmI() != 113)&&(whoAmI() != 115)){
    return -5;
  }
  // enable accelerometer and gyro
  if(writeRegister(PWR_MGMNT_2,SEN_ENABLE) < 0){
    return -6;
  }
  // setting accel range to 16G as default
  if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
    return -7;
  }
  _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
  _accelRange = ACCEL_RANGE_16G;
  // setting the gyro range to 2000DPS as default
  if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
    return -8;
  }
  _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
  _gyroRange = GYRO_RANGE_2000DPS;
  // setting bandwidth to 184Hz as default
  if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ 
    return -9;
  } 
  if(writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
    return -10;
  }
  _bandwidth = DLPF_BANDWIDTH_184HZ;
  // setting the sample rate divider to 0 as default
  if(writeRegister(SMPDIV,0x00) < 0){ 
    return -11;
  } 
  _srd = 0;
  // enable I2C master mode
  if(writeRegister(USER_CTRL,I2C_MST_EN) < 0){
  	return -12;
  }
	// set the I2C bus speed to 400 kHz
	if( writeRegister(I2C_MST_CTRL,I2C_MST_CLK) < 0){
		return -13;
	}
	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963() != 72 ){
    return -14;
    }
  /* get the magnetometer calibration */
  // set AK8963 to Power Down
  if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -15;
  }
  delay(100); // long wait between AK8963 mode changes
  // set AK8963 to FUSE ROM access
  if(writeAK8963Register(AK8963_CNTL1,AK8963_FUSE_ROM) < 0){
    return -16;
  }
  delay(100); // long wait between AK8963 mode changes
  // read the AK8963 ASA registers and compute magnetometer scale factors
  readAK8963Registers(AK8963_ASA,3,_buffer);
  _magScaleX = ((((float)_buffer[0]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleY = ((((float)_buffer[1]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla
  _magScaleZ = ((((float)_buffer[2]) - 128.0f)/(256.0f) + 1.0f) * 4912.0f / 32760.0f; // micro Tesla 
  // set AK8963 to Power Down
  if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
    return -17;
  }
  delay(100); // long wait between AK8963 mode changes  
  // set AK8963 to 16 bit resolution, 100 Hz update rate
  if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
    return -18;
  }
  delay(100); // long wait between AK8963 mode changes
  // select clock source to gyro
  if(writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) < 0){
    return -19;
  }       
  // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
  readAK8963Registers(AK8963_HXL,7,_buffer);
  // estimate gyro bias
  if (calibrateGyro() < 0) {
    return -20;
  }
  // successful init, return 1
  return 1;
}

/* sets the accelerometer full scale range to values other than default */
int setAccelRange(AccelRange range) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(range) {
    case ACCEL_RANGE_2G: {
      // setting the accel range to 2G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G) < 0){
        return -1;
      }
      _accelScale = G * 2.0f/32767.5f; // setting the accel scale to 2G
      break; 
    }
    case ACCEL_RANGE_4G: {
      // setting the accel range to 4G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G) < 0){
        return -1;
      }
      _accelScale = G * 4.0f/32767.5f; // setting the accel scale to 4G
      break;
    }
    case ACCEL_RANGE_8G: {
      // setting the accel range to 8G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G) < 0){
        return -1;
      }
      _accelScale = G * 8.0f/32767.5f; // setting the accel scale to 8G
      break;
    }
    case ACCEL_RANGE_16G: {
      // setting the accel range to 16G
      if(writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G) < 0){
        return -1;
      }
      _accelScale = G * 16.0f/32767.5f; // setting the accel scale to 16G
      break;
    }
  }
  _accelRange = range;
  return 1;
}

/* sets the gyro full scale range to values other than default */
int setGyroRange(GyroRange range) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(range) {
    case GYRO_RANGE_250DPS: {
      // setting the gyro range to 250DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS) < 0){
        return -1;
      }
      _gyroScale = 250.0f/32767.5f * _d2r; // setting the gyro scale to 250DPS
      break;
    }
    case GYRO_RANGE_500DPS: {
      // setting the gyro range to 500DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS) < 0){
        return -1;
      }
      _gyroScale = 500.0f/32767.5f * _d2r; // setting the gyro scale to 500DPS
      break;  
    }
    case GYRO_RANGE_1000DPS: {
      // setting the gyro range to 1000DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS) < 0){
        return -1;
      }
      _gyroScale = 1000.0f/32767.5f * _d2r; // setting the gyro scale to 1000DPS
      break;
    }
    case GYRO_RANGE_2000DPS: {
      // setting the gyro range to 2000DPS
      if(writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS) < 0){
        return -1;
      }
      _gyroScale = 2000.0f/32767.5f * _d2r; // setting the gyro scale to 2000DPS
      break;
    }
  }
  _gyroRange = range;
  return 1;
}
int setDlpfBandwidth(DlpfBandwidth bandwidth) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  switch(bandwidth) {
    case DLPF_BANDWIDTH_184HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_184) < 0){ // setting gyro bandwidth to 184Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_92HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92) < 0){ // setting accel bandwidth to 92Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_92) < 0){ // setting gyro bandwidth to 92Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_41HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41) < 0){ // setting accel bandwidth to 41Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_41) < 0){ // setting gyro bandwidth to 41Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_20HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20) < 0){ // setting accel bandwidth to 20Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_20) < 0){ // setting gyro bandwidth to 20Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10) < 0) { // setting accel bandwidth to 10Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_10) < 0){ // setting gyro bandwidth to 10Hz
        return -2;
      }
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5) < 0){ // setting accel bandwidth to 5Hz
        return -1;
      } 
      if(writeRegister(CONFIG,GYRO_DLPF_5) < 0){ // setting gyro bandwidth to 5Hz
        return -2;
      }
      break;
    }
  }
  _bandwidth = bandwidth;
  return 1;
}
/* sets the sample rate divider to values other than default */
int setSrd(uint8_t srd) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the sample rate divider to 19 to facilitate setting up magnetometer */
  if(writeRegister(SMPDIV,19) < 0){ // setting the sample rate divider
    return -1;
  }
  if(srd > 9){
    // set AK8963 to Power Down
    if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -2;
    }
    delay(100); // long wait between AK8963 mode changes  
    // set AK8963 to 16 bit resolution, 8 Hz update rate
    if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS1) < 0){
      return -3;
    }
    delay(100); // long wait between AK8963 mode changes     
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,7,_buffer);
  } else {
    // set AK8963 to Power Down
    if(writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN) < 0){
      return -2;
    }
    delay(100); // long wait between AK8963 mode changes  
    // set AK8963 to 16 bit resolution, 100 Hz update rate
    if(writeAK8963Register(AK8963_CNTL1,AK8963_CNT_MEAS2) < 0){
      return -3;
    }
    delay(100); // long wait between AK8963 mode changes     
    // instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
    readAK8963Registers(AK8963_HXL,7,_buffer);    
  } 
  /* setting the sample rate divider */
  if(writeRegister(SMPDIV,srd) < 0){ // setting the sample rate divider
    return -4;
  } 
  _srd = srd;
  return 1; 
}

/* enables the data ready interrupt */
int enableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;
  /* setting the interrupt */
  if (writeRegister(INT_PIN_CFG,INT_PULSE_50US) < 0){ // setup interrupt, 50 us pulse
    return -1;
  }  
  if (writeRegister(INT_ENABLE,INT_RAW_RDY_EN) < 0){ // set to data ready
    return -2;
  }
  return 1;
}

/* disables the data ready interrupt */
int disableDataReadyInterrupt() {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(INT_ENABLE,INT_DISABLE) < 0){ // disable interrupt
    return -1;
  }  
  return 1;
}

int enableWakeOnMotion(float womThresh_mg,LpAccelOdr odr) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  // set AK8963 to Power Down
  writeAK8963Register(AK8963_CNTL1,AK8963_PWR_DOWN);
  // reset the MPU9250
  writeRegister(PWR_MGMNT_1,PWR_RESET);
  // wait for MPU-9250 to come back up
  delay(1);
  if(writeRegister(PWR_MGMNT_1,0x00) < 0){ // cycle 0, sleep 0, standby 0
    return -1;
  } 
  if(writeRegister(PWR_MGMNT_2,DIS_GYRO) < 0){ // disable gyro measurements
    return -2;
  } 
  if(writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184) < 0){ // setting accel bandwidth to 184Hz
    return -3;
  } 
  if(writeRegister(INT_ENABLE,INT_WOM_EN) < 0){ // enabling interrupt to wake on motion
    return -4;
  } 
  if(writeRegister(MOT_DETECT_CTRL,(ACCEL_INTEL_EN | ACCEL_INTEL_MODE)) < 0){ // enabling accel hardware intelligence
    return -5;
  } 
  _womThreshold = map(womThresh_mg, 0, 1020, 0, 255);
  if(writeRegister(WOM_THR,_womThreshold) < 0){ // setting wake on motion threshold
    return -6;
  }
  if(writeRegister(LP_ACCEL_ODR,(uint8_t)odr) < 0){ // set frequency of wakeup
    return -7;
  }
  if(writeRegister(PWR_MGMNT_1,PWR_CYCLE) < 0){ // switch to accel low power mode
    return -8;
  }
  return 1;
}

/* configures and enables the FIFO buffer  */
int enableFifo(bool accel,bool gyro,bool mag,bool temp) {
  // use low speed SPI for register setting
  _useSPIHS = false;
  if(writeRegister(USER_CTRL, (0x40 | I2C_MST_EN)) < 0){
    return -1;
  }
  if(writeRegister(FIFO_EN,(accel*FIFO_ACCEL)|(gyro*FIFO_GYRO)|(mag*FIFO_MAG)|(temp*FIFO_TEMP)) < 0){
    return -2;
  }
  _enFifoAccel = accel;
  _enFifoGyro = gyro;
  _enFifoMag = mag;
  _enFifoTemp = temp;
  _fifoFrameSize = accel*6 + gyro*6 + mag*7 + temp*2;
  return 1;
}
int readSensor() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // grab the data from the MPU9250
  if (readRegisters(ACCEL_OUT, 21, _buffer) < 0) {
    return -1;
  }
  // combine into 16 bit values
  _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];  
  _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
  _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
  _tcounts = (((int16_t)_buffer[6]) << 8) | _buffer[7];
  _gxcounts = (((int16_t)_buffer[8]) << 8) | _buffer[9];
  _gycounts = (((int16_t)_buffer[10]) << 8) | _buffer[11];
  _gzcounts = (((int16_t)_buffer[12]) << 8) | _buffer[13];
  _hxcounts = (((int16_t)_buffer[15]) << 8) | _buffer[14];
  _hycounts = (((int16_t)_buffer[17]) << 8) | _buffer[16];
  _hzcounts = (((int16_t)_buffer[19]) << 8) | _buffer[18];
  // transform and convert to float values
  _ax = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale) - _axb)*_axs;
  _ay = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale) - _ayb)*_ays;
  _az = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale) - _azb)*_azs;
  _gx = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
  _gy = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
  _gz = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
  _hx = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
  _hy = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
  _hz = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
  _t = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
  return 1;
}

/* returns the accelerometer measurement in the x direction, m/s/s */
float getAccelX_mss() {
  return _ax;
}

/* returns the accelerometer measurement in the y direction, m/s/s */
float getAccelY_mss() {
  return _ay;
}
/* returns the accelerometer measurement in the z direction, m/s/s */
float getAccelZ_mss() {
  return _az;
}

/* returns the gyroscope measurement in the x direction, rad/s */
float getGyroX_rads() {
  return _gx;
}

/* returns the gyroscope measurement in the y direction, rad/s */
float getGyroY_rads() {
  return _gy;
}

/* returns the gyroscope measurement in the z direction, rad/s */
float getGyroZ_rads() {
  return _gz;
}

/* returns the magnetometer measurement in the x direction, uT */
float getMagX_uT() {
  return _hx;
}

/* returns the magnetometer measurement in the y direction, uT */
float getMagY_uT() {
  return _hy;
}

/* returns the magnetometer measurement in the z direction, uT */
float getMagZ_uT() {
  return _hz;
}

/* returns the die temperature, C */
float getTemperature_C() {
  return _t;
}
int readFifo() {
  _useSPIHS = true; // use the high speed SPI for data readout
  // get the fifo size
  readRegisters(FIFO_COUNT, 2, _buffer);
  _fifoSize = (((uint16_t) (_buffer[0]&0x0F)) <<8) + (((uint16_t) _buffer[1]));
  // read and parse the buffer
  for (size_t i=0; i < _fifoSize/_fifoFrameSize; i++) {
    // grab the data from the MPU9250
    if (readRegisters(FIFO_READ,_fifoFrameSize,_buffer) < 0) {
      return -1;
    }
    if (_enFifoAccel) {
      // combine into 16 bit values
      _axcounts = (((int16_t)_buffer[0]) << 8) | _buffer[1];  
      _aycounts = (((int16_t)_buffer[2]) << 8) | _buffer[3];
      _azcounts = (((int16_t)_buffer[4]) << 8) | _buffer[5];
      // transform and convert to float values
      _axFifo[i] = (((float)(tX[0]*_axcounts + tX[1]*_aycounts + tX[2]*_azcounts) * _accelScale)-_axb)*_axs;
      _ayFifo[i] = (((float)(tY[0]*_axcounts + tY[1]*_aycounts + tY[2]*_azcounts) * _accelScale)-_ayb)*_ays;
      _azFifo[i] = (((float)(tZ[0]*_axcounts + tZ[1]*_aycounts + tZ[2]*_azcounts) * _accelScale)-_azb)*_azs;
      _aSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoTemp) {
      // combine into 16 bit values
      _tcounts = (((int16_t)_buffer[0 + _enFifoAccel*6]) << 8) | _buffer[1 + _enFifoAccel*6];
      // transform and convert to float values
      _tFifo[i] = ((((float) _tcounts) - _tempOffset)/_tempScale) + _tempOffset;
      _tSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoGyro) {
      // combine into 16 bit values
      _gxcounts = (((int16_t)_buffer[0 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[1 + _enFifoAccel*6 + _enFifoTemp*2];
      _gycounts = (((int16_t)_buffer[2 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[3 + _enFifoAccel*6 + _enFifoTemp*2];
      _gzcounts = (((int16_t)_buffer[4 + _enFifoAccel*6 + _enFifoTemp*2]) << 8) | _buffer[5 + _enFifoAccel*6 + _enFifoTemp*2];
      // transform and convert to float values
      _gxFifo[i] = ((float)(tX[0]*_gxcounts + tX[1]*_gycounts + tX[2]*_gzcounts) * _gyroScale) - _gxb;
      _gyFifo[i] = ((float)(tY[0]*_gxcounts + tY[1]*_gycounts + tY[2]*_gzcounts) * _gyroScale) - _gyb;
      _gzFifo[i] = ((float)(tZ[0]*_gxcounts + tZ[1]*_gycounts + tZ[2]*_gzcounts) * _gyroScale) - _gzb;
      _gSize = _fifoSize/_fifoFrameSize;
    }
    if (_enFifoMag) {
      // combine into 16 bit values
      _hxcounts = (((int16_t)_buffer[1 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[0 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
      _hycounts = (((int16_t)_buffer[3 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[2 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
      _hzcounts = (((int16_t)_buffer[5 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6]) << 8) | _buffer[4 + _enFifoAccel*6 + _enFifoTemp*2 + _enFifoGyro*6];
      // transform and convert to float values
      _hxFifo[i] = (((float)(_hxcounts) * _magScaleX) - _hxb)*_hxs;
      _hyFifo[i] = (((float)(_hycounts) * _magScaleY) - _hyb)*_hys;
      _hzFifo[i] = (((float)(_hzcounts) * _magScaleZ) - _hzb)*_hzs;
      _hSize = _fifoSize/_fifoFrameSize;
    }
  }
  return 1;
}
/* returns the accelerometer FIFO size and data in the x direction, m/s/s */
void getFifoAccelX_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_axFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the y direction, m/s/s */
void getFifoAccelY_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_ayFifo,_aSize*sizeof(float));
}

/* returns the accelerometer FIFO size and data in the z direction, m/s/s */
void getFifoAccelZ_mss(size_t *size,float* data) {
  *size = _aSize;
  memcpy(data,_azFifo,_aSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the x direction, rad/s */
void getFifoGyroX_rads(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gxFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the y direction, rad/s */
void getFifoGyroY_rads(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gyFifo,_gSize*sizeof(float));
}

/* returns the gyroscope FIFO size and data in the z direction, rad/s */
void getFifoGyroZ_rads(size_t *size,float* data) {
  *size = _gSize;
  memcpy(data,_gzFifo,_gSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the x direction, uT */
void getFifoMagX_uT(size_t *size,float* data) {
  *size = _hSize;
  memcpy(data,_hxFifo,_hSize*sizeof(float));
}
/* returns the magnetometer FIFO size and data in the y direction, uT */
void getFifoMagY_uT(size_t *size,float* data) {
  *size = _hSize;
  memcpy(data,_hyFifo,_hSize*sizeof(float));
}

/* returns the magnetometer FIFO size and data in the z direction, uT */
void getFifoMagZ_uT(size_t *size,float* data) {
  *size = _hSize;
  memcpy(data,_hzFifo,_hSize*sizeof(float));
}

/* returns the die temperature FIFO size and data, C */
void getFifoTemperature_C(size_t *size,float* data) {
  *size = _tSize;
  memcpy(data,_tFifo,_tSize*sizeof(float));  
}
int calibrateGyro() {
  // set the range, bandwidth, and srd
  if (setGyroRange(GYRO_RANGE_250DPS) < 0) {
    return -1;
  }
  if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
    return -2;
  }
  if (setSrd(19) < 0) {
    return -3;
  }

  // take samples and find bias
  _gxbD = 0;
  _gybD = 0;
  _gzbD = 0;
  for (size_t i=0; i < _numSamples; i++) {
    readSensor();
    _gxbD += (getGyroX_rads() + _gxb)/((double)_numSamples);
    _gybD += (getGyroY_rads() + _gyb)/((double)_numSamples);
    _gzbD += (getGyroZ_rads() + _gzb)/((double)_numSamples);
    delay(20);
  }
  _gxb = (float)_gxbD;
  _gyb = (float)_gybD;
  _gzb = (float)_gzbD;

  // set the range, bandwidth, and srd back to what they were
  if (setGyroRange(_gyroRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  return 1;
}

/* returns the gyro bias in the X direction, rad/s */
float getGyroBiasX_rads() {
  return _gxb;
}

/* returns the gyro bias in the Y direction, rad/s */
float getGyroBiasY_rads() {
  return _gyb;
}

/* returns the gyro bias in the Z direction, rad/s */
float getGyroBiasZ_rads() {
  return _gzb;
}

/* sets the gyro bias in the X direction to bias, rad/s */
void setGyroBiasX_rads(float bias) {
  _gxb = bias;
}

/* sets the gyro bias in the Y direction to bias, rad/s */
void setGyroBiasY_rads(float bias) {
  _gyb = bias;
}

/* sets the gyro bias in the Z direction to bias, rad/s */
void setGyroBiasZ_rads(float bias) {
  _gzb = bias;
}

/* finds bias and scale factor calibration for the accelerometer,
this should be run for each axis in each direction (6 total) to find
the min and max values along each */
int calibrateAccel() {
  // set the range, bandwidth, and srd
  if (setAccelRange(ACCEL_RANGE_2G) < 0) {
    return -1;
  }
  if (setDlpfBandwidth(DLPF_BANDWIDTH_20HZ) < 0) {
    return -2;
  }
  if (setSrd(19) < 0) {
    return -3;
  }

  // take samples and find min / max 
  _axbD = 0;
  _aybD = 0;
  _azbD = 0;
  for (size_t i=0; i < _numSamples; i++) {
    readSensor();
    _axbD += (getAccelX_mss()/_axs + _axb)/((double)_numSamples);
    _aybD += (getAccelY_mss()/_ays + _ayb)/((double)_numSamples);
    _azbD += (getAccelZ_mss()/_azs + _azb)/((double)_numSamples);
    delay(20);
  }
  if (_axbD > 9.0f) {
    _axmax = (float)_axbD;
  }
  if (_aybD > 9.0f) {
    _aymax = (float)_aybD;
  }
  if (_azbD > 9.0f) {
    _azmax = (float)_azbD;
  }
  if (_axbD < -9.0f) {
    _axmin = (float)_axbD;
  }
  if (_aybD < -9.0f) {
    _aymin = (float)_aybD;
  }
  if (_azbD < -9.0f) {
    _azmin = (float)_azbD;
  }

  if ((abs(_axmin) > 9.0f) && (abs(_axmax) > 9.0f)) {
    _axb = (_axmin + _axmax) / 2.0f;
    _axs = G/((abs(_axmin) + abs(_axmax)) / 2.0f);
  }
  if ((abs(_aymin) > 9.0f) && (abs(_aymax) > 9.0f)) {
    _ayb = (_axmin + _axmax) / 2.0f;
    _ays = G/((abs(_aymin) + abs(_aymax)) / 2.0f);
  }
  if ((abs(_azmin) > 9.0f) && (abs(_azmax) > 9.0f)) {
    _azb = (_azmin + _azmax) / 2.0f;
    _azs = G/((abs(_azmin) + abs(_azmax)) / 2.0f);
  }

  // set the range, bandwidth, and srd back to what they were
  if (setAccelRange(_accelRange) < 0) {
    return -4;
  }
  if (setDlpfBandwidth(_bandwidth) < 0) {
    return -5;
  }
  if (setSrd(_srd) < 0) {
    return -6;
  }
  return 1;  
}

/* returns the accelerometer bias in the X direction, m/s/s */
float getAccelBiasX_mss() {
  return _axb;
}

/* returns the accelerometer scale factor in the X direction */
float getAccelScaleFactorX() {
  return _axs;
}

/* returns the accelerometer bias in the Y direction, m/s/s */
float getAccelBiasY_mss() {
  return _ayb;
}

/* returns the accelerometer scale factor in the Y direction */
float getAccelScaleFactorY() {
  return _ays;
}

/* returns the accelerometer bias in the Z direction, m/s/s */
float getAccelBiasZ_mss() {
  return _azb;
}

/* returns the accelerometer scale factor in the Z direction */
float getAccelScaleFactorZ() {
  return _azs;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the X direction */
void setAccelCalX(float bias,float scaleFactor) {
  _axb = bias;
  _axs = scaleFactor;
}
void setAccelCalY(float bias,float scaleFactor) {
  _ayb = bias;
  _ays = scaleFactor;
}

/* sets the accelerometer bias (m/s/s) and scale factor in the Z direction */
void setAccelCalZ(float bias,float scaleFactor) {
  _azb = bias;
  _azs = scaleFactor;
}

/* finds bias and scale factor calibration for the magnetometer,
the sensor should be rotated in a figure 8 motion until complete */
int calibrateMag() {
  // set the srd
  if (setSrd(19) < 0) {
    return -1;
  }

  // get a starting set of data
  readSensor();
  _hxmax = getMagX_uT();
  _hxmin = getMagX_uT();
  _hymax = getMagY_uT();
  _hymin = getMagY_uT();
  _hzmax = getMagZ_uT();
  _hzmin = getMagZ_uT();

  // collect data to find max / min in each channel
  _counter = 0;
  while (_counter < _maxCounts) {
    _delta = 0.0f;
    _framedelta = 0.0f;
    readSensor();
    _hxfilt = (_hxfilt*((float)_coeff-1)+(getMagX_uT()/_hxs+_hxb))/((float)_coeff);
    _hyfilt = (_hyfilt*((float)_coeff-1)+(getMagY_uT()/_hys+_hyb))/((float)_coeff);
    _hzfilt = (_hzfilt*((float)_coeff-1)+(getMagZ_uT()/_hzs+_hzb))/((float)_coeff);
    if (_hxfilt > _hxmax) {
      _delta = _hxfilt - _hxmax;
      _hxmax = _hxfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hyfilt > _hymax) {
      _delta = _hyfilt - _hymax;
      _hymax = _hyfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hzfilt > _hzmax) {
      _delta = _hzfilt - _hzmax;
      _hzmax = _hzfilt;
      }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hxfilt < _hxmin) {
      _delta = abs(_hxfilt - _hxmin);
      _hxmin = _hxfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hyfilt < _hymin) {
      _delta = abs(_hyfilt - _hymin);
      _hymin = _hyfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_hzfilt < _hzmin) {
      _delta = abs(_hzfilt - _hzmin);
      _hzmin = _hzfilt;
    }
    if (_delta > _framedelta) {
      _framedelta = _delta;
    }
    if (_framedelta > _deltaThresh) {
      _counter = 0;
    } else {
      _counter++;
    }
    delay(20);
  }

  // find the magnetometer bias
  _hxb = (_hxmax + _hxmin) / 2.0f;
  _hyb = (_hymax + _hymin) / 2.0f;
  _hzb = (_hzmax + _hzmin) / 2.0f;

  // find the magnetometer scale factor
  _hxs = (_hxmax - _hxmin) / 2.0f;
  _hys = (_hymax - _hymin) / 2.0f;
  _hzs = (_hzmax - _hzmin) / 2.0f;
  _avgs = (_hxs + _hys + _hzs) / 3.0f;
  _hxs = _avgs/_hxs;
  _hys = _avgs/_hys;
  _hzs = _avgs/_hzs;

  // set the srd back to what it was
  if (setSrd(_srd) < 0) {
    return -2;
  }
  return 1;
}

/* returns the magnetometer bias in the X direction, uT */
float getMagBiasX_uT() {
  return _hxb;
}

/* returns the magnetometer scale factor in the X direction */
float getMagScaleFactorX() {
  return _hxs;
}

/* returns the magnetometer bias in the Y direction, uT */
float getMagBiasY_uT() {
  return _hyb;
}

/* returns the magnetometer scale factor in the Y direction */
float getMagScaleFactorY() {
  return _hys;
}

/* returns the magnetometer bias in the Z direction, uT */
float getMagBiasZ_uT() {
  return _hzb;
}

/* returns the magnetometer scale factor in the Z direction */
float getMagScaleFactorZ() {
  return _hzs;
}

/* sets the magnetometer bias (uT) and scale factor in the X direction */
void setMagCalX(float bias,float scaleFactor) {
  _hxb = bias;
  _hxs = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Y direction */
void setMagCalY(float bias,float scaleFactor) {
  _hyb = bias;
  _hys = scaleFactor;
}

/* sets the magnetometer bias (uT) and scale factor in the Z direction */
void setMagCalZ(float bias,float scaleFactor) {
  _hzb = bias;
  _hzs = scaleFactor;
}

/* writes a byte to MPU9250 register given a register address and data */
int writeRegister(uint8_t subAddress, uint8_t data){
  /* write data to device */
  if( _useSPI ){
    _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3)); // begin the transaction
    digitalWrite(_csPin,LOW); // select the MPU9250 chip
    _spi->transfer(subAddress); // write the register address
    _spi->transfer(data); // write the data
    digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
    _spi->endTransaction(); // end the transaction
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // write the register address
    _i2c->write(data); // write the data
    _i2c->endTransmission();
  }

  delay(10);
  
  /* read back the register */
  readRegisters(subAddress,1,_buffer);
  /* check the read back register against the written register */
  if(_buffer[0] == data) {
    return 1;
  }
  else{
    return -1;
  }
}
/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
int readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest){
  if( _useSPI ){
    // begin the transaction
    if(_useSPIHS){
      _spi->beginTransaction(SPISettings(SPI_HS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    else{
      _spi->beginTransaction(SPISettings(SPI_LS_CLOCK, MSBFIRST, SPI_MODE3));
    }
    digitalWrite(_csPin,LOW); // select the MPU9250 chip
    _spi->transfer(subAddress | SPI_READ); // specify the starting register address
    for(uint8_t i = 0; i < count; i++){
      dest[i] = _spi->transfer(0x00); // read the data
    }
    digitalWrite(_csPin,HIGH); // deselect the MPU9250 chip
    _spi->endTransaction(); // end the transaction
    return 1;
  }
  else{
    _i2c->beginTransmission(_address); // open the device
    _i2c->write(subAddress); // specify the starting register address
    _i2c->endTransmission(false);
    _numBytes = _i2c->requestFrom(_address, count); // specify the number of bytes to receive
    if (_numBytes == count) {
      for(uint8_t i = 0; i < count; i++){ 
        dest[i] = _i2c->read();
      }
      return 1;
    } else {
      return -1;
    }
  }
}

/* writes a register to the AK8963 given a register address and data */
int writeAK8963Register(uint8_t subAddress, uint8_t data){
  // set slave 0 to the AK8963 and set for write
	if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address 
	if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // store the data for write
	if (writeRegister(I2C_SLV0_DO,data) < 0) {
    return -3;
  }
  // enable I2C and send 1 byte
	if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1) < 0) {
    return -4;
  }
	// read the register and confirm
	if (readAK8963Registers(subAddress,1,_buffer) < 0) {
    return -5;
  }
	if(_buffer[0] == data) {
  	return 1;
  } else{
  	return -6;
  }
}

/* reads registers from the AK8963 */
int readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest){
  // set slave 0 to the AK8963 and set for read
	if (writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR | I2C_READ_FLAG) < 0) {
    return -1;
  }
  // set the register to the desired AK8963 sub address
	if (writeRegister(I2C_SLV0_REG,subAddress) < 0) {
    return -2;
  }
  // enable I2C and request the bytes
	if (writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | count) < 0) {
    return -3;
  }
	delay(1); // takes some time for these registers to fill
  // read the bytes off the MPU9250 EXT_SENS_DATA registers
	_status = readRegisters(EXT_SENS_DATA_00,count,dest); 
  return _status;
}

/* gets the MPU9250 WHO_AM_I register value, expected to be 0x71 */
int whoAmI(){
  // read the WHO AM I register
  if (readRegisters(WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}

/* gets the AK8963 WHO_AM_I register value, expected to be 0x48 */
int whoAmIAK8963(){
  // read the WHO AM I register
  if (readAK8963Registers(AK8963_WHO_AM_I,1,_buffer) < 0) {
    return -1;
  }
  // return the register value
  return _buffer[0];
}
void MahonyUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Use IMU algorithm if magnetometer measurement invalid
	// (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MahonyUpdate_imu(gx, gy, gz, ax, ay, az, deltat);
		return;
	}

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * deltat;
			integralFBy += twoKi * halfey * deltat;
			integralFBz += twoKi * halfez * deltat;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
  // Integrate rate of change of quaternion
	gx *= (0.5f * deltat);		// pre-multiply common factors
	gy *= (0.5f * deltat);
	gz *= (0.5f * deltat);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}
void MahonyUpdate_imu(float gx, float gy, float gz, float ax, float ay, float az, float deltat)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;

		// Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * deltat;
			integralFBy += twoKi * halfey * deltat;
			integralFBz += twoKi * halfez * deltat;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

    // Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * deltat);		// pre-multiply common factors
	gy *= (0.5f * deltat);
	gz *= (0.5f * deltat);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

void MadgwickUpdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float deltat) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickUpdate_imu(gx, gy, gz, ax, ay, az, deltat);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
    _2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}
  // Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

void MadgwickUpdate_imu(float gx, float gy, float gz, float ax, float ay, float az, float deltat) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
    s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * deltat;
	q1 += qDot2 * deltat;
	q2 += qDot3 * deltat;
	q3 += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

bool initQuat(float ax, float ay, float az, float mx, float my, float mz){
    // Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		float recipNorm;
		float N[3], D[3], E[3]; //Global vectors North, Down, East relative to sensor
		
		// Down is negative accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		D[0] = - ax * recipNorm;
		D[1] = - ay * recipNorm;
		D[2] = - az * recipNorm;
		
		// Magnetometer is not exatly perpendicular to Down, therefor not exatly North
		// we will calculate North later
		float m[3];
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		m[0] = mx * recipNorm;
		m[1] = mx * recipNorm;
		m[2] = mx * recipNorm;
        
		// Calculate East
		vectorCross(m, D, E);
		recipNorm = invSqrt(E[0] * E[0] + E[1] * E[1] + E[2] * E[2]);
		E[0] *= recipNorm;
		E[1] *= recipNorm;
		E[2] *= recipNorm;
		
		// Calculate North
		vectorCross(D, E, N);
		
		// Calculate Euler Parameter (quaternion) from the rotation matrix A=(N|D|E).
		// Using Shepperd algorithm (Woernle 2011)
		float Trace = N[0] + D[1] + E[2];
		float a[4] = {Trace, N[0], D[1], E[2]};
		float e[4];
		
		//find index of Largest Euler parameter
		int k=0;
		for (int i=1; i<4; i++){
			if (a[i] > a[k])
				k = i;
		}
    //calculate that parameter
		e[k] = sqrt(1 + 2 * a[k] - Trace)/2;
		
		switch (k){
			case 0:
				e[1] = (D[2] - E[1]) / (4 * e[0]);
				e[2] = (E[0] - N[2]) / (4 * e[0]);
				e[3] = (N[1] - D[0]) / (4 * e[0]);
				break;
			case 1:
				e[0] = (D[2] - E[1]) / (4 * e[1]);
				e[2] = (D[0] + N[1]) / (4 * e[1]);
				e[3] = (E[0] + N[2]) / (4 * e[1]);
				break;
			case 2:
				e[0] = (E[0] - N[2]) / (4 * e[2]);
				e[1] = (D[0] + N[1]) / (4 * e[2]);
				e[3] = (E[1] + D[2]) / (4 * e[2]);
				break;
			case 3:
				e[0] = (N[1] - D[0]) / (4 * e[3]);
				e[1] = (E[0] + N[2]) / (4 * e[3]);
				e[2] = (E[1] + D[2]) / (4 * e[3]);
		}
		
		// invert the quaternion rotation
		// we calculated the rotation of Global NDE relative to the sensor 
		// but we need the rotation of the Sensor relative to NDE
		q0 = e[0];
		q1 = -e[1];
		q2 = -e[2];
		q3 = -e[3];
		
		return true;
	} else return false;
}

void vectorCross(float A[3], float B[3], float cross[3])
{
	cross[0] = A[1] * B[2] - A[2] * B[1];
	cross[1] = A[2] * B[0] - A[0] * B[2];
	cross[2] = A[0] * B[1] - A[1] * B[0];
}

void computeAngles()
{
	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	anglesComputed = 1;
}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	union {
	  float    f;
	  uint32_t i;
	} conv = { .f = x };
	conv.i = 0x5f3759df - (conv.i >> 1);
	conv.f *= 1.5f - (halfx * conv.f * conv.f);
	conv.f *= 1.5f - (halfx * conv.f * conv.f);
	return conv.f;
}

float deltatUpdate (){
	Now = micros();
	deltat = ((Now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
	lastUpdate = Now;
	return deltat;
}

float getRoll() {
	if (!anglesComputed) computeAngles();
	return roll * RAD_TO_DEG;
}
float getPitch() {
	if (!anglesComputed) computeAngles();
	return pitch * RAD_TO_DEG;
}
float getYaw() {
	if (!anglesComputed) computeAngles();
	return yaw * RAD_TO_DEG + 180.0f;
}
float getRollRadians() {
	if (!anglesComputed) computeAngles();
	return roll;
}
float getPitchRadians() {
	if (!anglesComputed) computeAngles();
	return pitch;
}
float getYawRadians() {
	if (!anglesComputed) computeAngles();
	return yaw;
}
float getQuat() {
	memcpy(_copyQuat, &q0, sizeof(float)*4);
	return *_copyQuat;
}



void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  // start communication with IMU 
  status = begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-8G 
  setAccelRange(ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  setGyroRange(GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  setDlpfBandwidth(DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  setSrd(19);
}

void loop() {
  // now read the gyroscope, accelerometer (and magnetometer if you have it also)
  // NOTE: the gyroscope data have to be in radians
  // if you have them in degree convert them with: DEG_TO_RAD example: gx * DEG_TO_RAD

  readSensor();

  ax = getAccelX_mss();
  Serial.print(ax ,6);
  Serial.print("\t");
  ay = getAccelY_mss();
  Serial.print(ay,6);
  Serial.print("\t");
  az = getAccelZ_mss();
  Serial.print(az,6);
  Serial.print("\t");
  gx = getGyroX_rads();
  Serial.print(getGyroX_rads(),6);
  Serial.print("\t");
  gy = getGyroY_rads();
  Serial.print(gy,6);
  Serial.print("\t");
  gz = getGyroZ_rads();
  Serial.print(gz,6);
  Serial.print("\t");
  mx = getMagX_uT();
  Serial.print(mx,6);
  Serial.print("\t");
  my = getMagY_uT();
  Serial.print(my,6);
  Serial.print("\t");
  mz = getMagZ_uT();
  Serial.print(mz,6);
  Serial.print("\t");
  delay(20);

  /*
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update

  // fusion.MahonyUpdate(gx, gy, gz, ax, ay, az, deltat);  //mahony is suggested if there isn't the mag and the mcu is slow
  fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);  //else use the magwick, it is slower but more accurate

  pitch = fusion.getPitch();
  roll = fusion.getRoll();    //you could also use getRollRadians() ecc
  yaw = fusion.getYaw();
*/
  Serial.print("Pitch:\t"); Serial.println(pitch);
  Serial.print("Roll:\t"); Serial.println(roll);
  Serial.print("Yaw:\t"); Serial.println(yaw);
  Serial.println();
}
