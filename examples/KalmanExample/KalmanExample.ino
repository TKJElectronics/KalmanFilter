#include <Wire.h>
#include <Kalman.h>

#define gyroAddress 0x68
#define accAddress 0x53

Kalman kX;
//Kalman kY; // If you wanted to calculate the angle along the Y-axis, then you simply create another instance

double zeroValue[3];
unsigned long timer;
double accX;
double rateX;
double pitch;

void setup() {  
  Serial.begin(115200);
  Wire.begin();
  
  i2cWrite(accAddress,0x31,0x09); // Full resolution mode
  i2cWrite(accAddress,0x2D,0x08); // Setup ADXL345 for constant measurement mode
  
  i2cWrite(gyroAddress,0x16,0x1A); // This puts your gyro at +-2000deg/sec and 98Hz Low pass filter
  i2cWrite(gyroAddress,0x15,0x09); // This sets your gyro at 100Hz sample rate
  
  // Calibrate all sensors when the y-axis is facing downward/upward (reading either 1g or -1g), then the x-axis and z-axis will both start at 0g
  for (uint8_t i = 0; i < 100; i++) { // Take the average of 100 readings
    zeroValue[0] += readAccX();
    zeroValue[1] += readAccZ();
    zeroValue[2] += readGyroX();
    delay(10);
  }  
  zeroValue[0] /= 100;
  zeroValue[1] /= 100;
  zeroValue[2] /= 100;

  timer = micros();
  kX.setAngle(270); // The angle calculated by accelerometer starts at 270 degrees
}

void loop() {
  accX = getAngleX();
  rateX = getRateX();
  
  pitch = kX.getAngle(accX,rateX,double(micros() - timer)/1000000);  // Calculate the angle using the Kalman filter
  timer = micros();  
  
  Serial.print(accX);
  Serial.print('\t');
  Serial.print(rateX);
  Serial.print('\t');  
  Serial.println(pitch);
  
  delay(10);
}

double getAngleX() {
  double accXval = (double)readAccX()-zeroValue[0];
  double accZval = (double)readAccZ()-zeroValue[1];
  double angle = (atan2(accXval,accZval)+PI)*RAD_TO_DEG;
  return angle;  
}
double getRateX() {  
  double rate = -(((double)readGyroX()-zeroValue[2])/14.375);  
  return rate;
}

int readGyroX() {
  uint8_t* data = i2cRead(gyroAddress, 0x1F,2);
  return (int)((data[0] << 8) | data[1]);
}
int readAccX() {
  uint8_t* data = i2cRead(accAddress, 0x32,2);
  return (int)(data[0] | (data[1] << 8));
}
int readAccZ() {
  uint8_t* data = i2cRead(accAddress, 0x36,2);
  return (int)(data[0] | (data[1] << 8));
}
void i2cWrite(uint8_t address, uint8_t registerAddress, uint8_t data){
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.write(data);
  Wire.endTransmission();
}
uint8_t* i2cRead(uint8_t address, uint8_t registerAddress, uint8_t nbytes) {
  uint8_t data[nbytes];  
  Wire.beginTransmission(address);
  Wire.write(registerAddress);
  Wire.endTransmission();  
  Wire.beginTransmission(address);
  Wire.requestFrom(address, nbytes);
  for(uint8_t i = 0; i < nbytes; i++)
    data[i] = Wire.read();
  Wire.endTransmission();  
  return data;
}
