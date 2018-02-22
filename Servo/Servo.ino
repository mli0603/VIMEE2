#include <Servo.h>
#include <ros.h>
#include <std_msgs/Bool.h>

Servo ser; 
const int SERVO = A8;
const int MAX = 2400;
const int MIN = 544;

ros::NodeHandle nh;

std_msgs::Bool servo_msg;

void servoCallback( const std_msgs::Bool& toggle_msg){
  if(toggle_msg.data)
    openServo();
  else
    closeServo();
}

ros::Subscriber<std_msgs::Bool> servo("servo", &servoCallback );


void setup() {

  pinMode(1,OUTPUT);
  ser.attach(SERVO); //analog pin 15

  nh.initNode();
  nh.subscribe(servo);
}

void loop() {
  nh.spinOnce();
  delay(1);
//  openServo();
//  delay(1000);
//  closeServo();
//  delay(1000);
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

