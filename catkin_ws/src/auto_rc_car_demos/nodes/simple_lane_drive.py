#!/usr/bin/env python2
import rospy
import numpy as np
import cv2
import math

from auto_rc_car_api.client import AutoRCCarClient


def get_black_centroid(img):
    rows, cols, _ = img.shape
    count = 0
    totX = 0

    drop_bottom_rows = 20

    for y in range(int(rows/2.5), rows-drop_bottom_rows):
        for x in range(0, cols):
            b,g,r = img[y, x]
            
            if b < threshold and g < threshold and r < threshold:
                count += 1
                totX += x

    

    centroid_x = int(totX / (count + 1))
    cv2.line(img, (centroid_x, 0), (centroid_x, rows), (0, 0, 255), 2)
    return centroid_x

def detect_sign(img):
    rows, cols, _ = img.shape
    print("rows=%d"%rows)
    print("cols=%d"%cols)
    midr = rows/2
    midc = cols/2

    grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grey = grey[0:midr, midc:]

    circles = cv2.HoughCircles(grey, cv2.HOUGH_GRADIENT, 1, 25, param1=30, param2=18)
    print(circles)

    #cv2.imshow("Car Camera", grey)
    #cv2.waitKey(25)

    if circles is None:
        return 'n', 0

    circles = np.round(circles[0, :]).astype("int")
    # loop over the (x, y) coordinates and radius of the circles
    for (x0, y0, r) in circles:
        if r < 8:
            print("too small")
            continue
        
        # draw the circle in the output image, then draw a rectangle
        # corresponding to the center of the circle
        print("midr=%d"%midr)
        print("midc=%d"%midc)

        x = midc+x0
        y = midr+y0
        sr = int(1.0/math.sqrt(2.0) * r)

        i_low = x - sr
        i_high = x + sr
        if i_low < 0 or i_high >= cols:
            print("i (%d, %d) out of range" % (i_low, i_high))
            continue

        j_low = y0 - sr
        j_high = y0 + sr
        if j_low < 0 or j_high >= rows:
            print("j (%d, %d) out of range" % (j_low, j_high))
            continue

        cv2.circle(img, (x , y0), r, (0, 255, 0), 4)
        cv2.rectangle(img, (x - 5, y0 - 5), (x + 5, y0 + 5), (0, 128, 255), -1)

        color_acc = [0,0,0]
        count = 0.0
        for i in range(i_low, i_high):
            for j in range(j_low, j_high):
                color = img[j, i]
                #print(color)
                color_acc = color_acc + color
                count = count + 1.0
        color_acc = color_acc / count
        print("avg_color: {}".format(color_acc))

        b = color_acc[0]
        g = color_acc[1]
        r = color_acc[2]

        tot = float(b + g + r)

        thresh = 0.5
        r = r / tot
        g = g / tot

        print("r=%f"%r)
        print("g=%f"%g)

        if r > thresh:
            return 'r', r

        if g > thresh:
            return 'g', g

    return 'n', 0


if __name__ == "__main__":
    car = AutoRCCarClient()
    
    print("Starting")
    dist = 0

    threshold = 25
    base_speed = 10.0

    while not rospy.is_shutdown():
        
        img, dt = car.get_latest_camera_rgb()
        img = car.camera_to_cv_image(img)

        dt_speed_scale = 1.0 / (1 + 50* dt)
        

        if img is None:
            car.software_sleep(0.25)
            continue    
        rows, cols, _ = img.shape    

        centroid_x = get_black_centroid(img)

        c, c_mag = detect_sign(img)

        if c == 'r':
            light_speed_scale = 0.0
        else:
            light_speed_scale = 1.0

        data = "Dist = %f     Data Transfer Time = %f" % (dist, dt)
        cv2.putText(img, data, (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
        

        control = -1.5*(centroid_x - (cols/2.0))/cols
        steer_speed_scale = 1.0 / (20.0 * control*control + 1.0)
        speed = base_speed * steer_speed_scale * dt_speed_scale * light_speed_scale
 
        print("dt=%f" % dt)
        print("speed=%f" % speed)
        print("steer=%f" % control)
        print("c=%s" % c)
        print("c_mag=%f" % c_mag)
        print("")

        car.send_control(speed, control)

        cv2.imshow("Car Camera", img)
        cv2.waitKey(25)

    car.send_control(0, 0)



