// Communication based on ros bridge
// to start communication, type in command "rosrun rosserial_python serial_node.py /dev/ttyACM0"

#include <ros.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>


// ROS
ros::NodeHandle nh;

std_msgs::Float32 acc1_msg;
std_msgs::Float32 acc2_msg;

ros::Publisher acc1("acc1", &acc1_msg); // Front Accelerometer
ros::Publisher acc2("acc2", &acc2_msg); // Top Accelerometer

std_msgs::Int64 us1_msg;
ros::Publisher us1("us1", &us1_msg);
std_msgs::Int64 us2_msg;
ros::Publisher us2("us2", &us2_msg);
std_msgs::Int16 fsr1_msg;
ros::Publisher fsr1("fsr1", &fsr1_msg);
std_msgs::Int16 fsr2_msg;
ros::Publisher fsr2("fsr2", &fsr2_msg);

// Servo
std_msgs::Bool servo_msg;
void servoCallback( const std_msgs::Bool& toggle_msg) {
  if (toggle_msg.data)
    openServo();
  else
    closeServo();
}

ros::Subscriber<std_msgs::Bool> servo("servo", &servoCallback );


void ROS_setup() {

  // ros communication
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  // outgoing nodes
  nh.advertise(acc1);
  nh.advertise(acc2);
  nh.advertise(us1);
  nh.advertise(us2);
  nh.advertise(fsr1);
  nh.advertise(fsr2);
  // incoming node(s)
  nh.subscribe(servo);
  nh.subscribe(motors);
}

