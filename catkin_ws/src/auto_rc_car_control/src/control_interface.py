#!/usr/bin/env python2

import rospy

from std_msgs.msg import Float64

class ControlInterface:
    def __init__(self):
        rospy.init_node('rc_car_controller_interface')

        self.left_vel_pub = rospy.Publisher('/racecar/internal/left_rear_wheel_velocity_controller/command', Float64, queue_size=5)
        self.right_vel_pub = rospy.Publisher('/racecar/internal/right_rear_wheel_velocity_controller/command', Float64, queue_size=5)
        self.left_front_vel_pub = rospy.Publisher('/racecar/internal/left_front_wheel_velocity_controller/command', Float64, queue_size=5)
        self.right_front_vel_pub = rospy.Publisher('/racecar/internal/right_front_wheel_velocity_controller/command', Float64, queue_size=5)
        self.vel_pubs = [self.left_vel_pub, self.right_vel_pub, self.left_front_vel_pub, self.right_front_vel_pub]
        
        self.vel_msg = Float64()
        self.vel_msg.data = 0

        self.left_steer_pub = rospy.Publisher('/racecar/internal/left_steering_position_controller/command', Float64, queue_size=5)
        self.right_steer_pub = rospy.Publisher('/racecar/internal/right_steering_position_controller/command', Float64, queue_size=5)
        self.steer_pubs = [self.left_steer_pub, self.right_steer_pub]

        self.steer_msg = Float64()
        self.steer_msg.data = 0

        self.vel_cmd_sub = rospy.Subscriber("/racecar/in/velocity", Float64, self.vel_cb)
        self.steer_cmd_sub = rospy.Subscriber("/racecar/in/steering", Float64, self.steer_cb)

    def main(self):
        while not rospy.is_shutdown():
            rospy.sleep(1.0)

    def vel_cb(self, msg):
        self.vel_msg.data = msg.data
        self.publish_wheel_speeds()

    def steer_cb(self, msg):
        self.steer_msg.data = msg.data
        self.publish_wheel_steering()

    def publish_wheel_speeds(self):
        for p in self.vel_pubs:
            p.publish(self.vel_msg)

    def publish_wheel_steering(self):
        for p in self.steer_pubs:
            p.publish(self.steer_msg)

if __name__ == "__main__":
    ci = ControlInterface()
    ci.main()