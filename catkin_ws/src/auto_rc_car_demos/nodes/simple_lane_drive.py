#!/usr/bin/env python2
import rospy
import numpy as np
import cv2
import math

from auto_rc_car_api.client_factory import ClientGenerator

from auto_rc_car_demos.basic_camera import BasicCameraCalc, HandCodedLaneFollower
from auto_rc_car_demos.lidar_calc import LidarCalc




if __name__ == "__main__":
    car = ClientGenerator.CreateRCCarClient(rospy.get_param("client_context_type"),
                                            rospy.get_param("client_comm_type"))
    basic_cam_calc = BasicCameraCalc()
    lidar_calc = LidarCalc()
    lane_follower = HandCodedLaneFollower()

    print("Starting")
    dist = 0

    threshold = 15
    base_speed = 8.0

    t_start = rospy.Time.now()
    timeout = rospy.get_param("timeout", -1)

    while not rospy.is_shutdown():
        
        # Exit conditions
        if timeout > 0 and (rospy.Time.now() - t_start).to_sec() > timeout:
            car.send_control(0,0)
            break

        # Get sensors
        img, dt = car.get_latest_camera_rgb()
        img = car.camera_to_cv_image(img)
        lidar, dt2 = car.get_latest_lidar()

        # Slow down if it has been 'a while' since last loop
        dt_speed_scale = 1.0 / (1 + 50* dt)
        

        if img is None:
            car.software_sleep(0.25)
            continue    
        rows, cols, _ = img.shape    

        # Resize Image for easier processing
        img_size = img.shape
        img_scale = 0.25
        newX = int(img_size[1]*img_scale)
        newY = int(img_size[0]*img_scale)
        img_red = cv2.resize(img, (newX, newY))



        # Get centroid of black pixels, use for steering
        centroid_x = basic_cam_calc.get_black_centroid(img_red, scaled_by=img_scale)
        centroid_percentage = (centroid_x / float(cols) - 0.5) * 2.0
        control_ang = -centroid_percentage * 0.25

        # OR

        control_ang = lane_follower.follow_lane(img)

        # Slow down if turning
        steer_speed_scale = 1.0 / (20.0 * control_ang*control_ang + 1.0)

        # Detect street lights
        c, c_mag = basic_cam_calc.detect_sign(img_red)
        if c == 'r':
            light_speed_scale = 0.0
        else:
            light_speed_scale = 1.0

        data = "Dist = %f     Data Transfer Time = %f" % (dist, dt)
        cv2.putText(img, data, (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))

        # Scale speeds based on detecting objects
        # Set defaults
        lidar_front_speed_scale = 1.0
        lidar_left_speed_scale = 1.0
        lidar_right_speed_scale = 1.0

        dist_front = -2
        dist_front_left = -2
        dist_front_right = -2
        dist_left = -2
        dist_right = -2
        try:
            sweep = 0.5
            front_limit = 1.5

            def dist_too_close(dist, close_limit):
                if dist > 0 and dist < close_limit:
                    return True
                return False

            # Stop if objects detected ahead
            dist_front_left = lidar_calc.lidar_data_too_close(lidar, -3.12, -3.12+sweep, front_limit)
            dist_front_right = lidar_calc.lidar_data_too_close(lidar, 3.14-sweep, 3.14, front_limit)
            if dist_too_close(dist_front_left, 0.15) or dist_too_close(dist_front_right, 0.15):
                lidar_front_speed_scale = 0.0
            
            # Slow if objects detected to either side
            dist_left = lidar_calc.lidar_data_too_close(lidar, -3, -0.6, 0.25)
            if dist_too_close(dist_left, 0.5):
                lidar_left_speed_scale = 0.5

            dist_right = lidar_calc.lidar_data_too_close(lidar, 0.6, 3, 0.25)
            if dist_too_close(dist_right, 0.5):
                lidar_right_speed_scale = 0.5

        except ZeroDivisionError as e:
            print(e)
        
        # Combine Speed Scales
        lidar_speed_scale = lidar_front_speed_scale * lidar_left_speed_scale * lidar_right_speed_scale
        speed = base_speed * steer_speed_scale * dt_speed_scale * light_speed_scale * lidar_speed_scale

        print("")
        print("dt=%f" % dt)
        print("speed=%f" % speed)
        print("steer_ang=%f" % control_ang)
        print("x_centroid=%f" % centroid_x)
        print("black centroid=%f" % centroid_percentage)
        print("c=%s" % c)
        print("c_mag=%f" % c_mag)
        print("dist_front_left=%f" % dist_front_left)
        print("dist_front_right=%f" % dist_front_right)
        print("dist_left=%f" % dist_left)
        print("dist_right=%f" % dist_right)
        
        # Send control
        if abs(speed) < 0.1:
            car.brake()
        else:
            car.send_control(speed, control_ang)

    # Extra brake at end
    car.send_control(0, 0)



