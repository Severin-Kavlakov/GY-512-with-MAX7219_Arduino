#include <Wire.h>          //I2C library
#include <LiquidCrystal.h> //1602 Liquid crystal library
/* 1602 LCD pinout
RS -     mega D7 PWM - nano D3 PWM
Enable - mega D8 PWM - nano D5 PWM
D4 - 	   mega D9 PWM - nano D6 PWM
D5 - 	   mega D10 PWM SS - nano D10 PWM SS
D6 - 	   mega D11 PWM MOSI - nano D11 PWM MOSI
D7 - 	   mega D12 PWM MISO - nano D12 no-pwm MISO
R/W - GND
VSS - GND
VCC - +5v */

LiquidCrystal lcd(3, 5, 6, 10, 11, 12); // initialize the library with the numbers of the interface pins

unsigned long int prevTime = millis(); // counting time in miliseconds, max is 4 294 967 295 ~~ 49,710269618055 day
unsigned long int currentTime;

const int MPU_ADDR = 0x68; // I2C address of MPU-6050.  If AD0 pin is HIGH -> I2C address = 0x69.
bool firstRead = true;     // first or second read of GY521
int16_t x1, y1, z1;        // first read
int16_t x2, y2, z2;        // second read
float dX, dY, dZ;          // deltas of X,Y,Z
float dXYZ;                // total delta of vectors XYZ
unsigned int Gy521_ReadsPerSecond = 100; //timing virables
unsigned int Gy521_ReadInterval = 1000/Gy521_ReadsPerSecond; //miliseconds

// converts int16 and float to strings, output has THE SAME LENGHT in SERIAL MONITOR
char temp_int_str[6];//int output char limit
char* convert_int16_to_str(int16_t i){ sprintf(temp_int_str, "%6d", i); return temp_int_str; }
char temp_float_str[9];//float output char limit
char* convert_float_to_str(float f) { dtostrf(f, sizeof(temp_float_str), 2, temp_float_str); return temp_float_str; }//2 decimals

void readFromGy521(int16_t &x, int16_t &y, int16_t &z) { // +-32768; modifies original virables, doesn't return anything
  Wire.beginTransmission(MPU_ADDR);    // start communicating to MPU_ADDR
  Wire.write(0x3B);                    // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);         // The Arduino sends a restart -> the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 6, true); // request 6 registers
  x = Wire.read()<<8 | Wire.read();    // reading registers: 0x3B(ACCEL_XOUT_H) and 0x3C(ACCEL_XOUT_L)
  y = Wire.read()<<8 | Wire.read();    // reading registers: 0x3D(ACCEL_YOUT_H) and 0x3E(ACCEL_YOUT_L)
  z = Wire.read()<<8 | Wire.read();    // reading registers: 0x3F(ACCEL_ZOUT_H) and 0x40(ACCEL_ZOUT_L)
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
  dX = abs(x2-x1); dY = abs(y2-y1); dZ = abs(z2-z1);
  dXYZ = sqrt(dX*dX + dY*dY + dZ*dZ); 
  dXYZ = (dXYZ/16384)*9806.65; // 2G = 32768 =>  1G = (16384/16384)*1G = 9806.65 mm/s^2
}


void setup() {
  /*Gy-521 setup*/
  Wire.begin();                     // start I2C library
  Wire.beginTransmission(MPU_ADDR); // Begin transmission to I2C slave ( GY-521 )
  Wire.write(0x6B);                 // PWR_MGMT_1 - write to  Power Management register 0x6B B not 8 !
  Wire.write(0);                    // Zero - wake up MPU-6050
  Wire.endTransmission(true);       // End transmission
  Serial.begin(9600);               // start serial communication 9600 b/second

  /*Display setup*/
  lcd.begin(16, 2); // set up the LCD's number of columns and rows
}
void loop() {
  currentTime = millis(); 

  //Gy521 FIRST read
  if(firstRead && (currentTime - prevTime >= Gy521_ReadInterval)) { 
    readFromGy521(x1, y1, z1);
    prevTime = currentTime;
    firstRead = false;
  }
  //Gy521 SECOND read 
  if(!firstRead && (currentTime - prevTime >= Gy521_ReadInterval)) { 
    readFromGy521(x2, y2, z2);
    calcTotalDelta();
    /* //output to serial monitor
    Serial.print("x1: "); Serial.print(convert_int16_to_str(x1));
    Serial.print(" | y1: "); Serial.print(convert_int16_to_str(y1));
    Serial.print(" | z1: "); Serial.print(convert_int16_to_str(z1));
    Serial.print(" |       x2: "); Serial.print(convert_int16_to_str(x2));
    Serial.print(" | y2: "); Serial.print(convert_int16_to_str(y2));
    Serial.print(" | z2: "); Serial.print(convert_int16_to_str(z2));
    Serial.print(" |       dX: "); Serial.print(convert_int16_to_str(dX));
    Serial.print(" | dY: "); Serial.print(convert_int16_to_str(dY));
    Serial.print(" | dZ: "); Serial.print(convert_int16_to_str(dZ));
    Serial.print(" |           dXYZ: "); Serial.println(convert_float_to_str(dXYZ)); */
    lcd.setCursor(0, 0); lcd.print(dXYZ);
    lcd.setCursor(0, 1); lcd.print("mm/s^2");

    prevTime = currentTime;
    firstRead = true;
  }
}