# VIMEE2
This work direcoty contains individual test files for each sensor. The integrated code file is in folder **VIMEE**

# Feature
Please run python script *processor.py* to process the raw data. To view data graphically, run **rqt** or **rqt_plot**. To save data, run **rqt** or **rosbag**.

To start communication with Arduino, run command "*rosrun rosserial_python serial_node.py [/dev/ttyACM0]*", the *[port]* needs to be changed based on your current Arduino connection.

To start python script, run command "*python processor.py*" in the work directory.

To open and close servo, type in command "*rostopic pub servo std_msgs/Bool [True/False]*", where *[True]* opens servo and *[False]* closes servo.
## Note
Please DO NOT type "*[*" and "*]*" in your command, it is just for illustration.

# TODO
Motor driver integration

Sensor data threshold

Robot state

# ROS
## Installation on your PC
Please follow the instruction on ROS website. For example, to install ROS Kinetics, the procedure can be found at http://wiki.ros.org/kinetic/Installation/Ubuntu

## Install Arduino library
The Arduino library installation can be found at http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
