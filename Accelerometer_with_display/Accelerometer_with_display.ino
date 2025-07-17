#include <Wire.h>       //I2C library
#include "LedControl.h" //MAX72XX library

// counting time in miliseconds, max is 4 294 967 295 ~~ 49,710269618055 days
unsigned long int prevTime;
unsigned long int currentTime;

// new class lc as LedControl; pin 12 - DataIn;  pin 11 - CLK;  pin 10 - CS (LOAD); We have 1 MAX72XX
LedControl lc=LedControl(12,11,10,1);

//I2C address of MPU-6050. If AD0 pin is HIGH -> I2C address = 0x69. Accelerometer's first and second readings. deltas of X,Y,Z and total delta of vector XYZ
const int MPU_ADDR = 0x68;
int16_t x1, y1, z1;
int16_t x2, y2, z2;
float dX, dY, dZ;
float dXYZ;

//timing virables
unsigned int Gy521_ReadsPerSecond = 10;
unsigned int Display_FPS = 10;
unsigned int Gy521_ReadInterval = Gy521_ReadsPerSecond/1000;
unsigned int Display_WriteInterval = Display_FPS/1000;

// converts int16 and float to string, resulting strings will have THE SAME LENGHT in the debug monitor. uses temporary virable
char temp_int_str[6];//int resulting char limit
char* convert_int16_to_str(int16_t i){ sprintf(temp_int_str, "%6d", i); return temp_int_str; }
char temp_float_str[9];//float resulting char limit
char* convert_float_to_str(float f) {
    dtostrf(f, 9, 2, temp_float_str);//float resulting char limit and 2 decimal points
    return temp_float_str;
}


void readFromGy521(int16_t &x, int16_t &y, int16_t &z) { //can be negative and up to +- 32000; modifies original virables, doesn't return anything
  Wire.beginTransmission(MPU_ADDR);    // start communicating to MPU_ADDR
  Wire.write(0x3B);                    // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);         // The Arduino sends a restart -> the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 6, true); // request 6 registers

  x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B(ACCEL_XOUT_H) and 0x3C(ACCEL_XOUT_L)
  y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D(ACCEL_YOUT_H) and 0x3E(ACCEL_YOUT_L)
  z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F(ACCEL_ZOUT_H) and 0x40(ACCEL_ZOUT_L)
  
 // print raw data
/*
  Serial.print("gX = ");    Serial.print  (convert_int16_to_str(x));
  Serial.print(" | gY = "); Serial.print  (convert_int16_to_str(y));
  Serial.print(" | gZ = "); Serial.println(convert_int16_to_str(z));*/

/* // REGISTERS NAMES
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) */
}
void calcTotalDelta() {  
  dX = fabs(x2-x1); dY = fabs(y2-y1); dZ = fabs(z2-z1);
  dXYZ = sqrt(dX*dX + dY*dY + dZ*dZ);
}

void setup() {
  /*Gy-521 setup*/
  Wire.begin();                     // start I2C library
  Wire.beginTransmission(MPU_ADDR); // Begin transmission to I2C slave ( GY-521 )
  Wire.write(0x6B);                 // PWR_MGMT_1 - write to  Power Management register 0x6B B not 8 !
  Wire.write(0);                    // Zero - wake up MPU-6050
  Wire.endTransmission(true);       // End transmission
  Serial.begin(9600);               // start serial communication 100 KB per second

  /*Display setup*/
  lc.shutdown(0,false); /* Wakeup call for MAX72XX */
  lc.setIntensity(0,8); /* Set brightness to medium */
  lc.clearDisplay(0);   /* Clear display */

  /* Callibration
  float Callibration[90];
  for(int i; i>=100; i++) {
    readFromGy521(x1, y1, z1);
    delay(Gy521_ReadInterval);
    readFromGy521(x2, y2, z2);
    calcTotalDelta();
  }
  */
}

enum State { WAIT_FIRST, WAIT_SECOND };
State currentState = WAIT_FIRST;

void loop() {
  currentTime = millis();

  if(currentState == WAIT_FIRST && (currentTime - prevTime >= Gy521_ReadInterval)) { 
    readFromGy521(x1, y1, z1);
    calcTotalDelta();

    prevTime = currentTime;
    currentState = WAIT_SECOND;
  }

  if(currentState == WAIT_SECOND && (currentTime - prevTime >= Gy521_ReadInterval)) { 
    readFromGy521(x2, y2, z2);
    calcTotalDelta();
    /*
    //output to serial monitor
    Serial.print("x1: "); Serial.print(convert_int16_to_str(x1));
    Serial.print(" | y1: "); Serial.print(convert_int16_to_str(y1));
    Serial.print(" | z1: "); Serial.print(convert_int16_to_str(z1));
    Serial.print(" |       x2: "); Serial.print(convert_int16_to_str(x2));
    Serial.print(" | y2: "); Serial.print(convert_int16_to_str(y2));
    Serial.print(" | z2: "); Serial.print(convert_int16_to_str(z2));
    */
    Serial.print(" |       dX: "); Serial.print(convert_int16_to_str(dX));
    Serial.print(" | dY: "); Serial.print(convert_int16_to_str(dY));
    Serial.print(" | dZ: "); Serial.print(convert_int16_to_str(dZ));
    
    Serial.print(" |           dXYZ: "); Serial.println(convert_float_to_str(dXYZ));
    
    prevTime = currentTime;
    currentState = WAIT_FIRST;
  }

  if(currentTime - prevTime >= Display_WriteInterval) {
    //display logic
    prevTime = currentTime;
  }
}