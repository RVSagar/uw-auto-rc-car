#!/usr/bin/env python2
import rospy
import numpy as np
import cv2

from auto_rc_car_api.client import AutoRCCarClient

if __name__ == "__main__":
    car = AutoRCCarClient()
    
    print("Starting")
    las, dt = car.get_latest_lidar()
    print(las)
    print(dt)