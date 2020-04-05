#!/usr/bin/env python2
import rospy
import numpy as np
import cv2
import math
import time
import tf

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Float64
from auto_rc_car_api.clients import CreateRCCar

TIME_END = 5

class OdometryImplementation:
    def __init__(self):
        rospy.init_node('odometry_publisher')
        self.pub_odom = rospy.Publisher('odom', Odometry)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.angle_sub = rospy.Subscriber("/racecar/in/steering", Float64, steer_cb)
        self.vel_sub = rospy.Subscriber("/racecar/in/velocity", Float64, vel_cb)

    def vel_cb(self):
        self.vel_msg = self.vel_sub.data    
        return self.vel_msg 

    def steer_cb(self):
        self.steering_angle = self.angle_sub.data  
        return self.steering_angle
    
    def angular_velocity(self):
        wheel_base = 0.13
        self.ang_vel = vel_cb * math.tan(self.steer_cb) / wheel_base
        return self.ang_vel

    def main(self):
        current_time = rospy.Time.now()
        last_time = rospy.Time.now() 
        x = 0
        y = 0
        yaw = 0

        r = rospy.Rate(1.0)
        while not rospy.is_shutdown():
            current_time = rospy.Time.now()

            dt = (current_time - last_time).to_sec()

            delta_x = self.vel_cb() * math.cos(yaw)
            delta_y = self.vel_cb() * math.sin(yaw)
            
            x += delta_x * dt
            y += delta_y * dt

            yaw += self.angular_velocity() * dt  

            odom_quat = tf.tranformations.quaternion_from_euler(0, 0, yaw)

            #publishing the transforms to tf
            odom_broadcaster.sendTransform(
                (x, y, 0.),
                odom_quat,
                current_time,
                "base_link",
                "odom"
            )

            odom = Odometry()
            odem.header.stamp = current_time
            odom.header.frame_id = "odom"

            #setting position
            odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))

            #setting velocity 
            odom.child_frame_id = "base_link"
            odom.twist.twist.linear.x = self.vel_cb()
            odom.twist.twist.linear.y = 0.0 
            odom.twist.twist.angular.z = self.angular_velocity()

            self.pub_odom.publish(odom)

            last_time = current_time
            r.sleep()


if __name__ == "__main__":
    car = CreateRCCar()
    od1 = OdometryImplementation()
    od1.main()
    time_start = rospy.Time.now()  

    while(True):
        car.send_control(10, 0.5)

        if (rospy.Time.now() - time_start).to_sec() > TIME_END:
            car.send_control(0,0)
            break
