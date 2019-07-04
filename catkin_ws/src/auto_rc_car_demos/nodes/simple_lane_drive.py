#!/usr/bin/env python2
import rospy
import numpy as np
import cv2

from auto_rc_car_api.client import AutoRCCarClient

if __name__ == "__main__":
    car = AutoRCCarClient()
    
    print("Starting")
    dist = 0
    threshold = 25

    while not rospy.is_shutdown():
        

        img, dt = car.get_latest_camera_rgb()
        img = car.camera_to_cv_image(img)

        if img is None:
            car.software_sleep(0.25)
            continue

        data = "Dist = %f     Data Transfer Time = %f" % (dist, dt)


        _, thresh = cv2.threshold(img, threshold, 255, cv2.THRESH_BINARY_INV)

        rows, cols, _ = thresh.shape
        count = 0
        totX = 0
        for x in range(0, cols):
            for y in range(int(rows/2.5), rows):
                b,g,r = thresh[y, x]
                
                if b == 255:
                    count += 1
                    totX += x

        centroid_x = totX / (count + 1);


        cv2.putText(img, data, (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
        cv2.line(img, (centroid_x, 0), (centroid_x, rows), (0, 0, 255), 2)
        cv2.imshow("Car Camera", img)
        cv2.waitKey(25)

        control = -2.0*(centroid_x - (cols/2.0))/cols
        print(control)

        car.send_control(0.3, control)

    car.send_control(0, 0)