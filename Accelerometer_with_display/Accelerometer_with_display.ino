#include <Wire.h>       //I2C library
#include "LedControl.h" //MAX72XX library

unsigned long int prevTime = millis(); // start counting time in miliseconds, max is 4 294 967 295 ~~ 49,710269618055 days
unsigned long int currentTime;         // millis() will be called every loop()

LedControl lc=LedControl(12,11,10,1); // new class lc as LedControl; pin 12 -> DataIn;  pin 11 -> CLK;  pin 10 -> CS (LOAD);  We have 1 MAX72XX.

const int MPU_ADDR = 0x68;      // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
int16_t x1, y1, z1;             // accelerometer first read
int16_t x2, y2, z2;             // accelerometer second read
int16_t dX, dY, dZ;             // integers for recording DIFFERENCE between x,y,z_1 and x,y,z_2
int16_t dXYZ;                   // integers for recording ABSOLUTE VECTOR DIFFERENCE

unsigned int Gy521_ReadsPerSecond = 100;
unsigned int Display_FPS = 50;
unsigned int Gy521_ReadInterval = 1000 / Gy521_ReadsPerSecond;
unsigned int Display_WriteInterval = 1000 / Display_FPS;

// converts int16 to string, resulting strings will have THE SAME LENGHT in the debug monitor.
char temp_str[3]; // temporary variable for convert function
char* convert_int16_to_str(int16_t i){ sprintf(temp_str, "%6d", i); return temp_str; }


void readFromGy521(int16_t &x, int16_t &y, int16_t &z) { //can be negative and up to +- 32000; modifies original virables, doesn't return anything
  Wire.beginTransmission(MPU_ADDR);    // start communicating to MPU_ADDR
  Wire.write(0x3B);                    // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);         // The Arduino sends a restart -> the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 6, true); // request 6 registers

  x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B(ACCEL_XOUT_H) and 0x3C(ACCEL_XOUT_L)
  y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D(ACCEL_YOUT_H) and 0x3E(ACCEL_YOUT_L)
  z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F(ACCEL_ZOUT_H) and 0x40(ACCEL_ZOUT_L)
  
/* // print raw data

  Serial.print("gX = ");    Serial.print  (convert_int16_to_str(aX));
  Serial.print(" | gY = "); Serial.print  (convert_int16_to_str(aY));
  Serial.print(" | gZ = "); Serial.println(convert_int16_to_str(aZ));*/

/* // REGISTERS NAMES
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) */
}
void writeToDisplay() {

}


void setup() {
  /*Gy-521 setup*/
  Wire.begin();                     // start I2C library
  Wire.beginTransmission(MPU_ADDR); // Begin transmission to I2C slave ( GY-521 )
  Wire.write(0x6B);                 // PWR_MGMT_1 - write to  Power Management register 0x6B B not 8 !
  Wire.write(0);                    // Zero - wake up MPU-6050
  Wire.endTransmission(true);       // End transmission
  Serial.begin(19200);              // start serial communication 100 KB per second

  /*Display setup*/
  lc.shutdown(0,false); /* Wakeup call for MAX72XX */
  lc.setIntensity(0,8); /* Set brightness to medium */
  lc.clearDisplay(0);   /* Clear display */
}


enum State { WAIT_FIRST, WAIT_SECOND };
State currentState = WAIT_FIRST;

void loop() {
  currentTime = millis();

  if(currentState == WAIT_FIRST && (currentTime - prevTime >= Gy521_ReadInterval)) { 
    readFromGy521(x2, y2, z2);
    prevTime = currentTime;
    currentState = WAIT_SECOND;
  }

  if(currentTime - prevTime >= Display_WriteInterval) {
    writeToDisplay();
    prevTime = currentTime;
    currentState = WAIT_FIRST;
  }
}