#!/usr/bin/env python2
import rospy
import numpy as np
import cv2

from auto_rc_car_api.client_factory import ClientGenerator

if __name__ == "__main__":
    car = ClientGenerator.CreateRCCarClient("sim", "local")
    t0 = rospy.Time.now().to_sec()
    
    print("Starting; Reading Laserscans, Printing dt")
    while not rospy.is_shutdown():
        car.send_control(1, 0.1) 
        las, dt = car.get_latest_lidar()

        print(dt)

        rospy.sleep(1.0)

        if rospy.Time.now().to_sec() > 30:
            exit()
