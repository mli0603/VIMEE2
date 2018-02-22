#include <math.h>

const int YZERO1 = 340;
const int NEGONEG1 = 272;
const int ONEG1 = 409;                 // acc1

const int YZERO2 = 344;
const int NEGONEG2 = 281;
const int ONEG2 = 417;                 // acc2

const int XACC1 = A0;                  // acc1: x-axis of the accelerometer
const int YACC1 = A1;                  // acc1: y-axis of the accelerometer
const int XACC2 = A2;                  // acc2: x-axis of the accelerometer
const int YACC2 = A3;                  // acc2: y-axis of the accelerometer

float xacc1;
float yacc1;
float xacc2;
float yacc2;
float angle1;
float angle2;

void setup() {
  // initialize the serial communications:
  Serial.begin(9600);
}

void loop() {
  // print the sensor values:
  xacc1 = analogRead(XACC1);
  Serial.print(xacc1);
  Serial.print("\t");
  
  yacc1 = analogRead(YACC1);
  Serial.print(yacc1);
  Serial.print("\t");
  
  xacc2 = analogRead(XACC2);
  Serial.print(xacc2);
  Serial.print("\t");
  
  yacc2 = analogRead(YACC2);
  Serial.print(yacc2);
  Serial.print("\t");

  angle1 = acos(toAcc(xacc1,NEGONEG1,ONEG1));
  if (yacc1 < YZERO1)
    Serial.print(angle1);
  else
    Serial.print(-angle1);
  Serial.print("\t");
  
  angle2 = acos(toAcc(xacc2,NEGONEG2,ONEG2));
  if (yacc2 < YZERO2)
    Serial.print(angle2);
  else
    Serial.print(-angle2);
    
  Serial.println();
  // delay before next reading:
  delay(20);
}

float toAcc(int value, int negative, int positive) {
  // convert digital value to acceleration in g
  return 2.0*(value-negative)/(positive-negative)-1;
}

