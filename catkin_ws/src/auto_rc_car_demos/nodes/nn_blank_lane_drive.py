#!/usr/bin/env python2
import rospy
import numpy as np
import cv2
import math
import rospkg

import keras.models

from auto_rc_car_api.client_factory import ClientGenerator

from auto_rc_car_demos.basic_camera import BasicCameraCalc, HandCodedLaneFollower
from auto_rc_car_demos.lidar_calc import LidarCalc

from auto_rc_car_msgs.msg import SimpleLaneFollowStatus



if __name__ == "__main__":
    rospack = rospkg.RosPack()

    car = ClientGenerator.CreateRCCarClient(rospy.get_param("client_context_type"),
                                            rospy.get_param("client_comm_type"))
    model = keras.models.load_model(rospack.get_path("road_dataset_generation") + "/test_model/test_model_blank.model")

    basic_cam_calc = BasicCameraCalc()
    lidar_calc = LidarCalc()
    lane_follower = HandCodedLaneFollower()

    status_msg = SimpleLaneFollowStatus()
    status_pub = rospy.Publisher("/status", SimpleLaneFollowStatus, queue_size=3)

    print("Starting")
    dist = 0

    threshold = 15
    base_speed = 8.0 #8.0
    steer_speed_scale = 26.0 #20.0

    t_start = rospy.Time.now()
    timeout = rospy.get_param("timeout", -1)

    while not rospy.is_shutdown():
        status_msg.time_up.data = rospy.Time.now() - t_start
        
        # Exit conditions
        if timeout > 0 and (rospy.Time.now() - t_start).to_sec() > timeout or rospy.is_shutdown():
            car.send_control(0,0)
            break

        # Get sensors
        img, dt = car.get_latest_camera_rgb()
        img = car.camera_to_cv_image(img)
        lidar, dt2 = car.get_latest_lidar()

        #cv2.imshow("Image", img)

        # Slow down if it has been 'a while' since last loop
        dt_speed_scale = 1.0 / (1 + 2* dt) #50
        status_msg.dt_speed_scale = dt_speed_scale
        

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
        

        grey = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        small = cv2.resize(grey, (80, 60))
        X = np.array(small)
        X = X.reshape([-1, 60, 80, 1])
        pred = model.predict(X)
        #print(pred)
        road_x0 = pred[0][0]
        road_yaw0 = pred[0][1]
        road_invRad = pred[0][2]
        print("Predict road: %.3f, %.3f, %.3f" % (road_x0, road_yaw0, road_invRad))
        
        ang_deg = 0.0
        control_ang = 0*road_invRad + road_yaw0 - 0.1*road_x0

        # Slow down if turning
        steer_speed_scale = 1.0 / (0.5* control_ang*control_ang + 1.0)

        status_msg.control_ang = control_ang
        status_msg.steer_ang_deg = ang_deg
        status_msg.steer_speed_scale = steer_speed_scale

        # Detect street lights
        c, c_mag = basic_cam_calc.detect_sign(img_red)
        if c == 'r':
            light_speed_scale = 0.0
        else:
            light_speed_scale = 1.0

        status_msg.sign = c
        status_msg.light_speed_scale = light_speed_scale

        data = "Dist = %f     Data Transfer Time = %f" % (dist, dt)
        cv2.putText(img, data, (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))

        # Scale speeds based on detecting objects
        # Set defaults
        lidar_front_speed_scale = 1.0
        lidar_left_speed_scale = 1.0
        lidar_right_speed_scale = 1.0

        dist_front_left = -2
        dist_front_right = -2
        dist_left = -2
        dist_right = -2
        try:
            sweep = 0.5
            front_limit = 0.75

            def dist_too_close(dist, close_limit):
                if dist > 1e-3 and dist < close_limit:
                    return True
                return False

            # Stop if objects detected ahead
            dist_front_left = lidar_calc.lidar_data_too_close(lidar, -3.12, -3.12+sweep, front_limit)
            dist_front_right = lidar_calc.lidar_data_too_close(lidar, 3.14-sweep, 3.14, front_limit)
            if dist_too_close(dist_front_left, 0.15) or dist_too_close(dist_front_right, 0.15):
                lidar_front_speed_scale = 0.5
            
            # Slow if objects detected to either side
            dist_left = lidar_calc.lidar_data_too_close(lidar, -3, -0.6, 0.25)
            if dist_too_close(dist_left, 0.5):
                lidar_left_speed_scale = 0.5

            dist_right = lidar_calc.lidar_data_too_close(lidar, 0.6, 3, 0.25)
            if dist_too_close(dist_right, 0.5):
                lidar_right_speed_scale = 0.5

        except ZeroDivisionError as e:
            print(e)

        
        status_msg.dist_front_left = dist_front_left
        status_msg.dist_front_right = dist_front_right
        status_msg.dist_left = dist_left
        status_msg.dist_right = dist_right

        status_msg.lidar_front_speed_scale = lidar_front_speed_scale
        status_msg.lidar_left_speed_scale = lidar_left_speed_scale
        status_msg.lidar_right_speed_scale = lidar_right_speed_scale
        
        # Combine Speed Scales
        lidar_speed_scale = lidar_front_speed_scale * lidar_left_speed_scale * lidar_right_speed_scale
        speed = base_speed * steer_speed_scale * dt_speed_scale * light_speed_scale * lidar_speed_scale

        status_msg.lidar_speed_scale = lidar_speed_scale
        status_msg.speed = speed
        
        status_pub.publish(status_msg)        
        
        # Send control
        if abs(speed) < 0.1:
            car.brake()
        else:
            car.send_control(speed, control_ang)
    
    #try to stop car on shutdown of script...doesn't seem to work?
    def shutdown():
        print ("shutdown time!")
        # Extra brake at end
        car.brake()
        car.send_control(0, 0)

    rospy.on_shutdown(shutdown)

    # Extra brake at end
    car.brake()
