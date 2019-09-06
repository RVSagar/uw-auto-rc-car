#!/usr/bin/env python2

import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import JointState

class FakeJointStatePublisher:
    def __init__(self):
        rospy.init_node('fake_joint_state_publisher')
	self.pub = rospy.Publisher("/racecar/internal/joint_states", JointState, queue_size=5)

	self.msg = JointState()
	self.msg.header.frame_id = 'chassis_link'
	self.msg.name = ["rear_speed_joint", "left_rear_wheel_joint", "right_rear_wheel_joint", "front_steer_joint", "right_front_wheel_joint", "left_front_wheel_joint"]
	vals = 	[0 for n in self.msg.name]
	self.msg.position = vals
	self.msg.velocity = vals
	self.msg.effort = vals

	self.seq = 1

    def run(self):
	while not rospy.is_shutdown():
		self.msg.header.stamp = rospy.Time.now()
		self.msg.header.seq = self.seq
		print(self.msg)
		self.pub.publish(self.msg)
		rospy.sleep(0.1)

		self.seq = self.seq + 1


if __name__ == "__main__":
    fjsp = FakeJointStatePublisher()
    fjsp.run()
