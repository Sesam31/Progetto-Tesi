#include <Servo.h>

Servo servo[6];
//default 800/2200
const int inf=1800;
const int sup=900;
//int oriz[6] = {1500,1550,1500,1560,1560,1500};


void setup() {
  // put your setup code here, to run once:
  servo[0].attach(3, inf, sup);
  servo[1].attach(5, inf, sup);
  servo[2].attach(6, inf, sup);
  servo[3].attach(9, inf, sup);
  servo[4].attach(10, inf, sup);
  servo[5].attach(11, inf, sup);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  servo[0].writeMicroseconds(1500);
  
}
