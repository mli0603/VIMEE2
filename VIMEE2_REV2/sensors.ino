#include <Servo.h>

// Servo
Servo ser; 
const int MAX = 2400;   // open position
const int MIN = 750;    // close position

// flags that get flipped from ross msg
bool servo_open = false;
bool servo_close = false;
unsigned long last_time_servo = 0;
int servo_pos = 2*MIN;

// pins
const int XACC1 = A0;                  // acc1: x-axis of the accelerometer
const int ZACC1 = A1;                  // acc1: Z-axis of the accelerometer
const int XACC2 = A2;                  // acc2: x-axis of the accelerometer
const int YACC2 = A3;                  // acc2: y-axis of the accelerometer

const int FSR1 = A4; 
const int FSR2 = A5; 

const int TRIGPIN1 = 3;
const int ECHOPIN1 = 2;
const int TRIGPIN2 = 5;
const int ECHOPIN2 = 4;

const int SERVO = 6;

// accelerometer constants
const int YZERO1 = 340;
const int NEGONEG1 = 272;
const int ONEG1 = 409;                 // acc1

const int YZERO2 = 344;
const int NEGONEG2 = 281;
const int ONEG2 = 417;                 // acc2

// data rates (in ms)
byte sensor_sampling_rate = 20;
byte servo_movement_rate = 10;

void sensors_setup() {

  //Define inputs and outputs for ultrasound
  pinMode(TRIGPIN1, OUTPUT);
  pinMode(TRIGPIN2, OUTPUT);
  pinMode(ECHOPIN1, INPUT);
  pinMode(ECHOPIN2, INPUT);

  //Define servo pins
  pinMode(SERVO,OUTPUT);
  ser.attach(SERVO);
  
  // CLOSE servo on startup
  //  closeServo();/
  ser.writeMicroseconds(2*MIN);   
}

unsigned long last_time_sensors;
void sensors_loop() {
  if ( millis() > last_time_sensors + sensor_sampling_rate){
    last_time_sensors = millis();
    acc1_msg.data = readAngle(ZACC1,XACC1,NEGONEG1,ONEG1,YZERO1);
    acc2_msg.data = readAngle(XACC2,YACC2,NEGONEG2,ONEG2,YZERO2);
    us1_msg.data = readUS1();
    us2_msg.data = readUS2();
    fsr1_msg.data = analogRead(FSR1);
    fsr2_msg.data = analogRead(FSR2);
    
    acc1.publish( &acc1_msg );
    acc2.publish( &acc2_msg );
    us1.publish( &us1_msg );
    us2.publish( &us2_msg );
    fsr1.publish( &fsr1_msg );
    fsr2.publish( &fsr2_msg );
  }
  
  nh.spinOnce();
}

// read angle of accelerometer
//float readAngle(int xpin, int ypin, int negative, int positive, int yzero){
//  float acc = 2.0*(analogRead(xpin)-negative)/(positive-negative)-1;
//  if (analogRead(ypin) < yzero){
//    return acos(acc);
//  }
//  else{
//    return -acos(acc);
//  }
//}


// returns roll angle of accelerometer in rads Updated Mar31
float readAngle(int axis2, int downaxis, float negative, float positive, float zero){
  float downval= (analogRead(downaxis)-zero)/(positive-negative)*3.14159265358;      // (scaled by ratio of posG-negG (which is 180 degree in ADC vals), and Pi
  float val2= (analogRead(axis2)-zero)/(positive-negative)*3.14159265358;
  return atan2(downval,val2);
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
  return pulseIn(ECHOPIN1, HIGH, 20*1000);
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
  return pulseIn(ECHOPIN2, HIGH, 20*1000); 
}

void closeServo(){
   
  if (millis() > last_time_servo + servo_movement_rate){
    last_time_servo = millis();
    ser.write(servo_pos);              // tell servo to go to position in variable 'pos'
    servo_pos += 10;
  }   

}

void openServo(){
      
  if (millis() > last_time_servo + servo_movement_rate){
    last_time_servo = millis();
    ser.write(servo_pos);              // tell servo to go to position in variable 'pos'
    servo_pos -= 10;
  }   

}

void servoloop(){
  if (servo_open){
    openServo();
    if (servo_pos <= MIN*2) {
      servo_open = false;
      servo_pos = MIN*2;
    }
  } else if (servo_close){
    closeServo();
    if (servo_pos >= MAX) {
      servo_close = false;
      servo_pos = MAX;
    }
  }
}

void servo_setopenflag(){
  servo_open = true;
  servo_close = false;
}

void servo_setcloseflag(){
  servo_open = false;
  servo_close = true;
}

// blocking implementation
//void closeServo(){
//  for (int pos = MIN*2; pos <= MAX; pos += 10) { // goes from 0 degrees to 180 degrees
//    // in steps of 1 degree
//    ser.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
//    delay(10);                       // waits 15ms for the servo to reach the position
//  }
//}
//
//void openServo(){
//  for (int pos = MAX; pos >= MIN*2; pos -= 10) { // goes from 180 degrees to 0 degrees
//    ser.write(pos);              // tell servo to go to position in variable 'pos'
//    delay(10);                       // waits 15ms for the servo to reach the position
//  }
//}
