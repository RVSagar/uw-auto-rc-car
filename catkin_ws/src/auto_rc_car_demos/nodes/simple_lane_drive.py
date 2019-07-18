#!/usr/bin/env python2
import rospy
import numpy as np
import cv2

from auto_rc_car_api.client import AutoRCCarClient

if __name__ == "__main__":
    car = AutoRCCarClient()
    
    print("Starting")
    dist = 0
    threshold = 45
    speed = 0.4

    while not rospy.is_shutdown():
        

        img, dt = car.get_latest_camera_rgb()
        img = car.camera_to_cv_image(img)

        if img is None:
            car.software_sleep(0.25)
            continue

        

        rows, cols, _ = img.shape
        count = 0
        totX = 0
        for x in range(0, cols):
            for y in range(int(rows/2.5), rows):
                b,g,r = img[y, x]
                
                if b < threshold and g < threshold and r < threshold:
                    count += 1
                    totX += x

        centroid_x = totX / (count + 1);

        data = "Dist = %f     Data Transfer Time = %f" % (dist, dt)
        cv2.putText(img, data, (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
        cv2.line(img, (centroid_x, 0), (centroid_x, rows), (0, 0, 255), 2)
        cv2.imshow("Car Camera", img)
        cv2.waitKey(25)

        control = -5.0*(centroid_x - (cols/2.0))/cols
        print(control)

        car.send_control(speed, control)

    car.send_control(0, 0)