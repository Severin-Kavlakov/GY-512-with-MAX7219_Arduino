#include <Wire.h>
#include "LedControl.h"

/*
 Now we need a LedControl to work with.
 pin 12 is connected to the DataIn 
 pin 11 is connected to the CLK 
 pin 10 is connected to LOAD 
 We have only a single MAX72XX.
 */
LedControl lc=LedControl(12,11,10,1);

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t gyroX, gyroY, gyroZ; // gyroscope raw data

void DisplaySetup() {
  lc.shutdown(0,false); /* Wakeup call for MAX72XX */
  lc.setIntensity(0,8); /* Set brightness to medium */
  lc.clearDisplay(0);   /* Clear display */
}
void DisplayLoop() {





}

void GyroscopeSetup() {
  /*Gyroscope setup*/
  Wire.begin();                     // start I2C library
  Wire.beginTransmission(MPU_ADDR); // Begin transmission to I2C slave - the GY-521
  Wire.write(0x6B);                 // PWR_MGMT_1 - write to  Power Management register 0x6B B not 8 !!!!!!!!!!!!!!!!!!
  Wire.write(0);                    // Set to zero - wake up the MPU-6050
  Wire.endTransmission(true);       // End transmission
  Serial.begin(9600);               // start serial communication 9600 B per second
}
void GyroscopeLoop() {
  Wire.beginTransmission(MPU_ADDR);    // start communicating to MPU_ADDR
  Wire.write(0x43);                    // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false);         // The Arduino sends a restart -> the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 6, true); // request 6 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  gyroX = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  gyroY = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  gyroZ = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
/*
  // print data
  Serial.print("gX = "); Serial.println(gyroX);
  Serial.print("gY = "); Serial.println(gyroY);
  Serial.print("gZ = "); Serial.println(gyroZ);
  Serial.println();
*/
/*
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) 
*/
}


void setup() {
  GyroscopeSetup();
  DisplaySetup();

}
void loop() {
  GyroscopeLoop();
  DisplayLoop();
  delay(500); // delay, how often (in miliseconds) to read values
}