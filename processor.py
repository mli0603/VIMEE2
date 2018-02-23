#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
import sys
import select
import termios
import tty
from std_msgs.msg import Int16, Int64, Float32, Bool
import time
import numpy as np
from util import *

SER_CLOSE_CMD = 'close'
SER_OPEN_CMD = 'open'

acc1 = []
acc2 = []

# kalman filter constants

varVolt = 1.12184278324081E-05  # variance determined using excel and reading samples of raw sensor data
varProcess = 1e-8
Pc = 0.0
G = 0.0
P = 1.0
Xp = 0.0
Zp = 0.0
Xe = 0.0


# kalman filter

def kalmanFilter(val):
    global P, Pc, varProcess, G, Xp, Zp, Xe
    Pc = P + varProcess
    G = Pc / (Pc + varVolt)  # kalman gain
    P = (1 - G) * Pc
    Xp = Xe
    Zp = Xp
    Xe = G * (val - Zp) + Xp  # the kalman estimate
    return Xe


# accelerometer callback, convert to g (9.81 m/s^2) and then publish

def acc1_callback(data):
    if not np.isnan(data.data):
        if len(acc1) < WINDOWSIZE:
            acc1.append(data.data)
        else:
            acc1.pop(0)
            acc1.append(data.data)
        acc1_pub.publish(toDegree(sum(acc1) / len(acc1)))


def acc2_callback(data):
    if not np.isnan(data.data):
        if len(acc2) < WINDOWSIZE:
            acc2.append(data.data)
        else:
            acc2.pop(0)
            acc2.append(data.data)
        acc2_pub.publish(toDegree(sum(acc2) / len(acc2)))


# ultrasound callback, convert to cm, filter and then publish

def us1_callback(data):
    us1_pub.publish(kalmanFilter(toCm(data.data)))


def us2_callback(data):
    us2_pub.publish(kalmanFilter(toCm(data.data)))


# fsr callback, TODO: convert to force and then publish

def fsr1_callback(data):
    fsr1_pub.publish(data.data)


def fsr2_callback(data):
    fsr2_pub.publish(data.data)

# control servo
def servo_control(cmd):
    print 'Sending command {} to servo'.format(cmd)
    if cmd == SER_OPEN_CMD:
        print 'open servo'
        ser_pub.publish(True)
    elif cmd == SER_CLOSE_CMD:
        print 'close servo'
        ser_pub.publish(False)
    else:
        print 'unknown command'

# listener function that subscribes all incoming arduino topics

def listener():

    # initialize ros node

    rospy.init_node('processor', anonymous=True)

    rospy.Subscriber('acc1', Float32, acc1_callback)
    rospy.Subscriber('acc2', Float32, acc2_callback)
    rospy.Subscriber('us1', Int64, us1_callback)
    rospy.Subscriber('fsr1', Int16, fsr1_callback)
    rospy.Subscriber('fsr2', Int16, fsr2_callback)


# talker function that publishes all outgoing processed data topics

def talker():
    global acc1_pub, acc2_pub, us1_pub, us2_pub, fsr1_pub, fsr2_pub, ser_pub
    acc1_pub = rospy.Publisher('acc1_processed', Float32, queue_size=10)
    acc2_pub = rospy.Publisher('acc2_processed', Float32, queue_size=10)
    us1_pub = rospy.Publisher('us1_processed', Float32, queue_size=10)
    us2_pub = rospy.Publisher('us2_processed', Float32, queue_size=10)
    fsr1_pub = rospy.Publisher('fsr1_processed', Int16, queue_size=10)
    fsr2_pub = rospy.Publisher('fsr2_processed', Int16, queue_size=10)
    ser_pub = rospy.Publisher('servo', Bool, queue_size=10)


def getKey():
    tty.setraw(sys.stdin.fileno())
    (rlist, _, _) = select.select([sys.stdin], [], [], 0.005)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)

    print 'processing starts, all topics are up'
    talker()
    listener()

    while not rospy.is_shutdown():
        k = getKey()

        if k == 'q':
            print 'Quit'
            break
        elif k == 'o':
            # print 'Open servo'
            servo_control('open')
        elif k == 'c':
            # print 'Close servo'
            servo_control('close')

