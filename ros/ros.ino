/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 acc1_msg;
ros::Publisher acc1("acc1", &acc1_msg);

void setup()
{
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(acc1);
}

void loop()
{
  acc1_msg.data = 1.22;
  acc1.publish( &acc1_msg );
  nh.spinOnce();
  delay(20);
}
