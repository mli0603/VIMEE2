  // VIMEE 
// Communication based on ros bridge
// to start communication, type in command "rosrun rosserial_python serial_node.py /dev/ttyACM0"

#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <Servo.h>

// Servo
Servo ser; 
const int MAX = 2400;
const int MIN = 544;

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

const int SERVO = A8;

// accelerometer constants
const int YZERO1 = 340;
const int NEGONEG1 = 272;
const int ONEG1 = 409;                 // acc1

const int YZERO2 = 344;
const int NEGONEG2 = 281;
const int ONEG2 = 417;                 // acc2

// ROS
ros::NodeHandle nh;

std_msgs::Float32 acc1_msg;
ros::Publisher acc1("acc1", &acc1_msg);
std_msgs::Float32 acc2_msg;
ros::Publisher acc2("acc2", &acc2_msg);
std_msgs::Int64 us1_msg;
ros::Publisher us1("us1", &us1_msg);
std_msgs::Int16 fsr1_msg;
ros::Publisher fsr1("fsr1", &fsr1_msg);
std_msgs::Int16 fsr2_msg;
ros::Publisher fsr2("fsr2", &fsr2_msg);

std_msgs::Bool servo_msg;

void servoCallback( const std_msgs::Bool& toggle_msg){
  if(toggle_msg.data)
    openServo();
  else
    closeServo();
}

ros::Subscriber<std_msgs::Bool> servo("servo", &servoCallback );

void setup() {
  // ros communication
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  // outgoing nodes
  nh.advertise(acc1);
  nh.advertise(acc2);
  nh.advertise(us1);
  nh.advertise(fsr1);
  nh.advertise(fsr2);
  // incoming node(s)
  nh.subscribe(servo);
   
  
//  // serial communication
//  Serial.begin(9600);

  //Define inputs and outputs for ultrasound
  pinMode(TRIGPIN1, OUTPUT);
  pinMode(TRIGPIN2, OUTPUT);
  pinMode(ECHOPIN1, INPUT);
  pinMode(ECHOPIN2, INPUT);

  //Define servo pins
  pinMode(SERVO,OUTPUT);
  ser.attach(SERVO);
}

void loop() {
  acc1_msg.data = readAngle(XACC1,YACC1,NEGONEG1,ONEG1,YZERO1);
  acc2_msg.data = readAngle(XACC2,YACC2,NEGONEG2,ONEG2,YZERO2);
  us1_msg.data = readUS1();
  fsr1_msg.data = analogRead(FSR1);
  fsr2_msg.data = analogRead(FSR2);
  
  acc1.publish( &acc1_msg );
  acc2.publish( &acc2_msg );
  us1.publish( &us1_msg );
  fsr1.publish( &fsr1_msg );
  fsr2.publish( &fsr2_msg );

  nh.spinOnce();
  delay(20);
}

// read angle of accelerometer
float readAngle(int xpin, int ypin, int negative, int positive, int yzero){
  float acc = 2.0*(analogRead(xpin)-negative)/(positive-negative)-1;
  if (analogRead(ypin) < yzero){
    return acos(acc);
  }
  else{
    return -acos(acc);
  }
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
  return pulseIn(ECHOPIN1, HIGH, 50000);
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
  return pulseIn(ECHOPIN2, HIGH, 50000); 
}

void openServo(){
  for (int pos = MIN*2; pos <= MAX; pos += 10) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    ser.writeMicroseconds(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}

void closeServo(){
  for (int pos = MAX; pos >= MIN*2; pos -= 10) { // goes from 180 degrees to 0 degrees
    ser.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
}
