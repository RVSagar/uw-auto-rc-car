#!/usr/bin/env python2
import rospy
import numpy as np
import cv2
import math
import time

from auto_rc_car_api.clients import CreateRCCar

TIME_END = 12
LEFT = 0.5
RIGHT = -0.5
STRAIGHT = 0

def rad_to_deg(rad):
    deg = rad * (180 / math.pi)
    return deg

def length_of_object(angle, sl1, sl2):
    sl1_sqrd = math.pow(sl1, 2)
    sl2_sqrd = math.pow(sl2, 2)

    length = math.sqrt(sl1_sqrd + sl2_sqrd - (2*sl1*sl2*math.cos(angle)))
    return length


if __name__ == "__main__":
    car = CreateRCCar()
   
    time_start = rospy.Time.now()   

    # lidar = car.get_latest_lidar()

    # lidar_values = list(lidar[0].ranges)

    increment = (math.pi / 720)
    ang_rad = -(math.pi / 720)

    # for value in lidar_values:
    #     ang_rad += increment
    #     if type(value) == float and value != float("inf"):
    #         dist_arr.append(value)
    #         angle_arr.append(ang_rad)


    # angle = rad_to_deg(angle_arr[-1] - angle_arr[0])

    mid = 360

    # length = length_of_object(angle, dist_arr[0], dist_arr[-1])


    # turn_right = car.send_control(10, 0.5)
    # turn_left = car.send_control(10, 0)

    # time_right = time.time() + 5

    while(True):

        lidar = car.get_latest_lidar()

        list_of_ranges = list(lidar[0].ranges)

        dist_arr = []
        angle_arr = []

        for value in list_of_ranges:
            ang_rad += increment
            if type(value) == float and value != float("inf"):
                dist_arr.append(value)
                angle_arr.append(ang_rad)

        angle = angle_arr[-1] - angle_arr[0]

        length = length_of_object(angle, dist_arr[0], dist_arr[-1])

        car.send_control(10, STRAIGHT)

        if list_of_ranges[mid] < 1.5: 
            car.send_control(10, 0.5)


        print("center dist: {}       length: {}".format(list_of_ranges[mid], length))

        #print("First Dist: {}      Second Dist: {}      Angle: {}      Length: {}".format(dist_arr[0], dist_arr[-1], angle, length))

        # while(time.time() < time_right): 
        #     car.send_control(10, 0.5)
        
        # car.send_control(10, -0.5)

        if list_of_ranges[mid] < 0.5:
            car.send_control(0, 0)

        if (rospy.Time.now() - time_start).to_sec() > TIME_END:
            car.send_control(0,0)
            break