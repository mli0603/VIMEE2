# VIMEE2
This work direcoty contains individual test files for each sensor. The integrated code file is in folder **VIMEE**

# Feature
Please run python script *processor.py* to process the raw data. 

To view data graphically, run **rqt** or **rqt_plot**. For example, after typing in "**rqt**", select plot, and then type in the topics you want to subscribe. You can add multiple topics at the same time to multiple plots.
![alt text](https://github.com/mli0603/VIMEE2/blob/master/img/rqt_plot.png)
![alt text](https://github.com/mli0603/VIMEE2/blob/master/img/rqt_plot_topic.png)

To save data, run **rqt** or **rosbag**. For example, after typing in "**rqt**", select bag, and then press save button to save selected topics. The best thign about ros bag is that it syncronizes data and you can play it back afterwards.
![alt text](https://github.com/mli0603/VIMEE2/blob/master/img/rqt_bag.png)
![alt text](https://github.com/mli0603/VIMEE2/blob/master/img/rqt_bag_save.png)

To start communication with Arduino, run command "**rosrun rosserial_python serial_node.py [/dev/ttyACM0]**", the *[port]* needs to be changed based on your current Arduino connection.

To start python script, run command "**python processor.py**" in the work directory.

To open and close servo, type in command "**rostopic pub servo std_msgs/Bool [True/False]**", where *[True]* opens servo and *[False]* closes servo.
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
button 
## Set Up Wifi (esp8266)

First, you must install the esp8266 add-on into your arduino ide in order to enable esp8266 flashing. The tutorial for how to set-up flash an esp8266 in the arduino ide can be found in the following links.

https://learn.sparkfun.com/tutorials/esp8266-thing-hookup-guide/installing-the-esp8266-arduino-addon

or alternative link

https://randomnerdtutorials.com/how-to-install-esp8266-board-arduino-ide/


The firmware to be flashed is under esproswifi directory. In order to flash, hold down the reset button on the arduino and both the reset and flash buttons on the shield. Click upload on the arduino IDE. When the arduino prompt changes from "Compiling" to "Uploading", immediately release the reset button, then the flash button in quick succession. 

If upload process is done correctly, the blue LED will start flickering and the arduino IDE will start the firmware flashing process. Note that the reset button on the arduino must be pushed down (grounded) during the entirety of the flashing process and is only released once the arduino ide indicated that upload has been completed.

In the esproswifi.ino, the baud rate (current default at 57600) should be changed according to the arduino ros baud rate. In addition, the SSID and password must be set depending on the network. Furthermore, the IP address must be set to the computer running roscore server (current default at 192.168.0.100). Then, the esp module must be reflashed.

Please note that during flashing, the TX pin of the esp module must be connecting to the TX of arduino, RX to RX. However, during normal operation with ROS, the TX pin of the esp must be connected to the RX of the arduino and the RX pin of the esp must be connected to the TX of the arduino.

In order to start the ros server, simply run in the linux terminal "roscore".

In a separate terminal, run... 

```
rosrun rosserial_python serial_node.py _port:=tcp _baud:=<insert baud rate here>
```

ex. 
```
rosrun rosserial_python serial_node.py _port:=tcp _baud:=57600
```

alternatively, if the default baud rate is being used you can run

```
rosrun rosserial_python serial_node.py tcp
```

## Note
When flashing either Arduino or the Wi-Fi module esp8266, put the two switches on the green development board both to flashing position (marked in black). After flashing is completed, the two switches should both be put back to the operation position.
