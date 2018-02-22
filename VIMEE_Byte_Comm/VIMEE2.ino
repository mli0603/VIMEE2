// VIMEE V2
// Write data per byte

// Start byte
const char START = '<';
const char END = '>';


// pins
const int XACC1 = A0;                  // acc1: x-axis of the accelerometer
const int YACC1 = A1;                  // acc1: y-axis of the accelerometer
const int XACC2 = A2;                  // acc2: x-axis of the accelerometer
const int YACC2 = A3;                  // acc2: y-axis of the accelerometer

const int FSR1 = A4; 
const int FSR2 = A5; 

const int TRIGPIN1 = 2;
const int ECHOPIN1 = 3;
const int TRIGPIN2 = 4;
const int ECHOPIN2 = 5;

void setup() {
  // serial communication
  Serial.begin(9600);

  //Define inputs and outputs for ultrasound
  pinMode(TRIGPIN1, OUTPUT);
  pinMode(TRIGPIN2, OUTPUT);
  pinMode(ECHOPIN1, INPUT);
  pinMode(ECHOPIN2, INPUT);
}

void loop() {
  static int xacc1, yacc1, xacc2, yacc2, fsr1, fsr2;
  static long us1, us2;
  
  // read acceleartion
  xacc1 = analogRead(XACC1);
  yacc1 = analogRead(YACC1);
  xacc2 = analogRead(XACC2);
  yacc2 = analogRead(YACC2);
  // read FSR
  fsr1 = analogRead(FSR1);
  fsr2 = analogRead(FSR2);
  // read us
  us1 = readUS1();
//  us2 = readUS2();/
  
  // write acceleration
  Serial.write(START);
  Serial.print(us1);
//  writeInt(11);/
//  writeInt(xacc1);
//  writeInt(yacc1); 
//  writeInt(xacc2);
//  writeInt(yacc2);
//  // write fsr
//  writeInt(fsr1);
//  writeInt(fsr2); 
//  // write us
//  writeLong(us1);
//  writeLong(us2);
  Serial.write(END);
  Serial.println();

  delay(20);
}

// write int to serial, 2 byte
void writeInt(int data){
  byte buf[2];
  buf[0] = data & 255;
  buf[1] = (data >> 8)  & 255;
  Serial.write(buf, sizeof(buf));
}

// write long to serial, 4 byte
void writeLong(long data){
  byte buf[4];
  buf[0] = data & 255;
  buf[1] = (data >> 8)  & 255;
  buf[2] = (data >> 16) & 255;
  buf[3] = (data >> 24) & 255;
  Serial.write(buf, sizeof(buf));
}

// read ultrasound duration
long readUS1() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIGPIN1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN1, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIGPIN1, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ECHOPIN1, INPUT);
  return pulseIn(ECHOPIN1, HIGH, 50); 
}

// read ultrasound duration
long readUS2() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIGPIN2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGPIN2, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIGPIN2, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ECHOPIN2, INPUT);
  return pulseIn(ECHOPIN2, HIGH, 50); 
}

