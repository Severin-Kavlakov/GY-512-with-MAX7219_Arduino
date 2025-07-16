int16_t x1 = 0, y1 = 0, z1 = 0;             // accelerometer first read
int16_t x2 = 3, y2 = 4, z2 = 5;             // accelerometer second read
int16_t dX, dY, dZ;             // integers for recording DIFFERENCE between x,y,z_1 and x,y,z_2
float dXYZ;                     // float for recording ABSOLUTE VECTOR DIFFERENCE

void calcTotalDeltas() {
  x1 = abs(x1); y1 = abs(y1); z1 = abs(z1);
  x2 = abs(x1); y2 = abs(y2); z2 = abs(z2);
  
  

  dX = x2-x1; dY = y2-y1; dZ = z2-z1;
  dXYZ = sqrt((dX*dX) + dY*dY + dZ*dZ);
  Serial.println(dXYZ);
}

void setup() {
  Serial.begin(9600);
}

void loop() {
  calcTotalDeltas();
  delay(100);
}
