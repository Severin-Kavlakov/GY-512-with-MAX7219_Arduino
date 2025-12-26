#include <Wire.h>          //I2C library
#include <LiquidCrystal.h> //1602 Liquid crystal library
/* 1602 LCD pinout
RS -     mega D7 PWM       - nano D3 PWM
Enable - mega D8 PWM       - nano D5 PWM
D4 - 	   mega D9 PWM       - nano D6 PWM
D5 - 	   mega D10 PWM SS   - nano D10 PWM SS
D6 - 	   mega D11 PWM MOSI - nano D11 PWM MOSI
D7 - 	   mega D12 PWM MISO - nano D12 no-pwm MISO
R/W - GND
VSS - GND
VCC - +5v */
LiquidCrystal lcd(7, 8, 9, 10, 11, 12); //initialize library with numbers of interface pins

unsigned long int prevTime = millis(); //count time in miliseconds, max is 4 294 967 295 ~~ 49,710269618055 days
unsigned long int currentTime;

const int MPU_ADDR = 0x68; //I2C address of MPU-6050, If pin AD0 = HIGH -> I2C address = 0x69
bool firstRead = true;     //First/second read of GY521
int16_t x1, y1, z1;        // first read
int16_t x2, y2, z2;        // second read
float dX, dY, dZ;          //Deltas of X,Y,Z
float dXYZ;                //Sum of vectors dX+dY+dZ

const uint16_t Gy521_ReadsPerSecond = 50;
const uint16_t Gy521_ReadInterval = 1000/Gy521_ReadsPerSecond;//miliseconds

const uint8_t n = 6;
const uint8_t SecondsToFindMaxdXYZ[n] = {5, 10, 15, 30, 60, 120};//seconds
const uint8_t buttonPin = 2;

uint16_t maxdXYZ;//NEED a function to find this max
uint16_t BufferArray[/*max in array * reads per second*/120*Gy521_ReadsPerSecond];
uint16_t ElementsFrom_BufferArray_ToFindMaxdXYZ = SecondsToFindMaxdXYZ[2]*Gy521_ReadsPerSecond;


/*Converts float to string, output has the SAME LENGHT in SERIAL MONITOR*/
char temp_float_str[6];//output char limit
char* convert_float_to_str(float f) {dtostrf(f, sizeof(temp_float_str), 0/*decimals*/, temp_float_str); return temp_float_str;}

void readFromGy521(int16_t &x, int16_t &y, int16_t &z) { //+-32768 +; modifies original virables
  Wire.begin();
  Wire.beginTransmission(0x68);        //Start communicating to MPU_ADDR
  Wire.write(0x3B);                    // starting register 0x3B (ACCEL_XOUT_H)
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
void calcTotalDelta() {//19620 parts max = 2G
  dX = abs(x2-x1); dY = abs(y2-y1); dZ = abs(z2-z1);
  dXYZ = ( sqrt(dX*dX + dY*dY + dZ*dZ) /16384)*9810; //1G =16384parts = 9.81m/s^2 = 9810mm/s^2
}

void setup() {
  pinMode(buttonPin, INPUT);
  /*Gy-521 setup*/
  Wire.begin();                     //Start I2C library
  Wire.beginTransmission(MPU_ADDR); // Begin transmission to I2C slave GY-521
  Wire.write(0x6B);                 // PWR_MGMT_1 - write to Power Management register 0x6B
  Wire.write(0);                    // wake up MPU-6050
  Wire.endTransmission(true);       //End transmission
  //Serial.begin(9600);               // start serial communication 9600 bits/second
  lcd.begin(16, 2); //LCD setup - columns, rows 
}
void loop() {
  currentTime = millis();

  if(firstRead && (currentTime - prevTime >= Gy521_ReadInterval)) { //Gyro FIRST read
    readFromGy521(x1, y1, z1);

    prevTime = currentTime;
    firstRead = false;
  }
  if(!firstRead && (currentTime - prevTime >= Gy521_ReadInterval)) { //Gyro SECOND read
    readFromGy521(x2, y2, z2);
    calcTotalDelta();

    //Serial.print("dXYZ: "); Serial.println(convert_float_to_str(dXYZ)); //output to serial monitor
    lcd.setCursor(0, 0); lcd.print("Max: "); lcd.print("19620"); lcd.print("mm/s^2"); //print result
    lcd.setCursor(0, 1); lcd.print("Buffer: "); lcd.print("256"); lcd.print("s"); //print the max

    prevTime = currentTime;
    firstRead = true;
  }
  //button check
}
//buffer array for dXYZ - array[seconds*reads per second]
//i=-1
//repeat until array[last] =! null
//  array[i+1] = dXYZ
// find max in array, print it
//check for button press, choose next option from SecondsToFindMaxdXYZ

/*QUEUE = first in first out
    4 -> 3 -> 2 -> 1
    /               \
  back-most recent  front-oldest

  INSERT/PUSH:
    5 -> 4 -> 3 -> 2 -> 1
    /                     \
  back-most recent        front-oldest

  DEQUEUE/POP:
    5 -> 4 -> 3 -> 2
    /               \
  back-most recent   front-oldest    
*/



