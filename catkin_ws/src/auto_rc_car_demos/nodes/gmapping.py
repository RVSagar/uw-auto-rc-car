#!/usr/bin/env python2
import rospy
import numpy as np
import cv2
import math
import time

from std_msgs.msg import Float64
from auto_rc_car_api.clients import CreateRCCar

TIME_END = 5

angle_sub = rospy.Subscriber("/racecar/in/steering", Float64, steer_cb)
vel_sub = rospy.Subscriber("/racecar/in/velocity", Float64, vel_cb)

def steer_cb(msg = angle_sub):
    steering_angle = msg.data   

def vel_cb(msg = vel_sub):
    vel_msg = msg.data

def vel_to_rpm():


def publish_odom():
    rospy.init_node('odometry')
    pub_odom = rospy.Publisher('/racecar/out/odom', Float64)


class odometry:
    def __init__(self):
        rospy.init_node('odometry')
        self.pub_odom = rospy.Publisher('/racecar/out/odom', Float64)

        self.angle_sub = rospy.Subscriber("/racecar/in/steering", Float64, steer_cb)
        self.vel_sub = rospy.Subscriber("/racecar/in/velocity", Float64, vel_cb)

    def vel_to_rpm(self):
        self.vel_msg = 


if __name__ == "__main__":
    car = CreateRCCar()
    time_start = rospy.Time.now()  

    while(True):
        car.send_control(10, 0.5)

        if (rospy.Time.now() - time_start).to_sec() > TIME_END:
            car.send_control(0,0)
            break
