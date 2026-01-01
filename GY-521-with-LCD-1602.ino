#include <Wire.h>          //I2C library
#include <LiquidCrystal.h> //1602 Liquid crystal library
/* 1602 LCD pinout
  RS -     mega D7 PWM       - nano D_ PWM
  Enable - mega D8 PWM       - nano D_ PWM
  D4 - 	   mega D9 PWM       - nano D_ PWM
  D5 - 	   mega D10 PWM SS   - nano D10 PWM SS
  D6 - 	   mega D11 PWM MOSI - nano D11 PWM MOSI
  D7 - 	   mega D12 PWM MISO - nano D12 no-pwm MISO
  R/W - GND
  VSS - GND
  VCC - +5v */
LiquidCrystal lcd(7, 8, 9, 10, 11, 12); //initialize library with numbers of interface pins

const uint16_t Gy521_ReadsPerSecond = 200, Gy521_ReadInterval = 1000/Gy521_ReadsPerSecond; //miliseconds
uint32_t currentTime, prevTime, prevTime_2; //count time in miliseconds, max is 4 294 967 295 ~~ 49,710269618055 days

const int MPU_ADDR = 0x68; //I2C address of MPU-6050, If pin AD0 = HIGH -> I2C address = 0x69
bool firstRead = true;     //First, second read of GY521
int16_t x1, y1, z1, x2, y2, z2;
float dX, dY, dZ, dXYZ;    //Deltas of X,Y,Z         
float prevdXYZ;
float currentMaxdXYZ, finalMaxdXYZ;

const uint8_t potentiometerPin = A0;
const uint8_t bufferSizeSeconds[5] = {5, 10, 15, 30, 60};
uint8_t index = 0;

char finalMaxdXYZOutput[5];
char* convert_float_to_str(float f) { dtostrf(f, sizeof(finalMaxdXYZOutput), 0/*decimals*/, finalMaxdXYZOutput); return finalMaxdXYZOutput; }
char IndexOutput[2];
char* convert_uint16_to_str(uint16_t i) { sprintf(IndexOutput, "%2d", i); return IndexOutput; }

void readFromGy521(int16_t &x, int16_t &y, int16_t &z) { //+-32768 +; modifies original virables
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);    //Start communicating to MPU_ADDR
  Wire.write(0x3B);                    // start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);         // keep connection active
  Wire.requestFrom(0x68, 6, true);     //Request 6 registers
  x = Wire.read()<<8 | Wire.read();    // read: 0x3B(ACCEL_XOUT_H) and 0x3C(ACCEL_XOUT_L)
  y = Wire.read()<<8 | Wire.read();    // read: 0x3D(ACCEL_YOUT_H) and 0x3E(ACCEL_YOUT_L)
  z = Wire.read()<<8 | Wire.read();    // read: 0x3F(ACCEL_ZOUT_H) and 0x40(ACCEL_ZOUT_L)
/* // REGISTERS NAMES
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L) */
}
void calcTotalDelta() {//19620 parts max = 2G ; 1G = 16384 parts = 9.81m/s^2 = 9810mm/s^2
  dX = abs(x2-x1);
  dY = abs(y2-y1);
  dZ = abs(z2-z1);
  dXYZ = (sqrt(dX*dX + dY*dY + dZ*dZ)/16384)*9810; //sum if vectors dX, dY, dZ, in mm/s^2
}

void setup() {
  pinMode(potentiometerPin, INPUT);
  /*Gy-521 setup*/
  Wire.begin();                     //Start I2C library
  Wire.beginTransmission(MPU_ADDR); // begin transmission to I2C slave GY-521
  Wire.write(0x6B);                 // write to Power Management(PWR_MGMT_1) register 0x6B
  Wire.write(0);                    // wake up MPU-6050
  Wire.endTransmission(true);       //End transmission
  /*LCD setup - columns, rows*/
  lcd.begin(16, 2); 
  Serial.begin(9600);               //Start serial communication 9600 bits/second
}
void loop() {
  currentTime = millis();
  
  index = map( analogRead(potentiometerPin), 0, 1023, 0, 5 ); //choose buffer size in seconds

  if(currentTime - prevTime_2 >= bufferSizeSeconds[index]*1000) {
    finalMaxdXYZ = currentMaxdXYZ; Serial.print(currentTime); Serial.println(" ;    final max RESET");
    
    prevTime_2 = currentTime;
  }

  if(firstRead && (currentTime - prevTime >= Gy521_ReadInterval)) { //FIRST read
    readFromGy521(x1, y1, z1);

    prevTime = currentTime;
    firstRead = false;
  }
  if(!firstRead && (currentTime - prevTime >= Gy521_ReadInterval)) { //SECOND read
    readFromGy521(x2, y2, z2);
    calcTotalDelta();

    if(prevdXYZ > dXYZ){
      currentMaxdXYZ = prevdXYZ;

      prevdXYZ = dXYZ;
    }
    if(prevdXYZ < dXYZ) {
      currentMaxdXYZ = dXYZ;

      prevdXYZ = dXYZ;
    }

    if(currentMaxdXYZ > finalMaxdXYZ) {
      finalMaxdXYZ = currentMaxdXYZ; Serial.print(currentTime); Serial.println(" ; final max SET");
    }

    lcd.setCursor(0, 0); lcd.print("Max: "); lcd.print(convert_float_to_str(finalMaxdXYZ)); lcd.print("mm/s^2");
    lcd.setCursor(0, 1); lcd.print("Buffer: "); lcd.print(convert_uint16_to_str(bufferSizeSeconds[index])); lcd.print("s");

    prevTime = currentTime; //uses the same prevTime variable because only one of the 2 if's are executed at once
    firstRead = true;
  }
}
