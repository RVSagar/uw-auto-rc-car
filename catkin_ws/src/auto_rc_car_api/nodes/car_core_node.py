#!/usr/bin/env python2
import rospy

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from geometry_msgs.msg import Twist

from auto_rc_car_api.srv import carLidar, carLidarResponse
from auto_rc_car_api.srv import carCamera, carCameraResponse
from auto_rc_car_api.srv import carControls, carControlsResponse
from auto_rc_car_api.msg import carSteering


class RawDataModule:
    def __init__(self, rospy,
                 msg_topic, msg_type,
                 srv_topic, srv_type, srv_return_type):
        self.msg_topic = msg_topic
        self.msg_type = msg_type
        self.srv_topic = srv_topic
        self.srv_type = srv_type
        self.srv_return_type = srv_return_type

        self.data = self.msg_type()
        self.sub = rospy.Subscriber(self.msg_topic, self.msg_type, self.data_cb)
        self.srv = rospy.Service(self.srv_topic, self.srv_type, self.srv_cb)
    
    def data_cb(self, msg):
        self.data = msg

    def srv_cb(self, msg):
        return self.srv_return_type(self.data)

class AutoRCCar:

    def __init__(self):

        rospy.init_node('rc_car_core')

        self.lidar = RawDataModule(rospy,
                                   '/racecar/out/laser_scan', LaserScan,
                                   'racecar_api/lidar', carLidar, carLidarResponse)
        self.camera_rgb = RawDataModule(rospy,
                                   '/racecar/out/stereo_camera/rgb/image', Image,
                                   'racecar_api/camera/rgb', carCamera, carCameraResponse)

        self.camera_depth = RawDataModule(rospy,
                                   '/racecar/out/stereo_camera/depth/image', Image,
                                   'racecar_api/camera/depth', carCamera, carCameraResponse)

        self.camera_cloud = RawDataModule(rospy,
                                   '/racecar/out/stereo_camera/depth_cloud/points', PointCloud2,
                                   'racecar_api/camera/cloud', carCamera, carCameraResponse)

        self.controls = RawDataModule(rospy,
                                      '/racecar/api_internal/control', carSteering,
                                      'racecar_api/last_control', carControls, carControlsResponse)

        self.control_sub = rospy.Subscriber('/racecar/api_internal/control', carSteering, self.control_cb)
        self.control_pup = rospy.Publisher('/racecar/in/cmd_vel', Twist, queue_size=3)

        #self.laser_scan = None
        #self.laser_scan_sub = rospy.Subscriber('/racecar/out/laser_scan', LaserScan, self.laser_scan_cb)
        #self.laser_scan_service = rospy.Service('/racecar_api/lidar', Lidar, self.laser_scan_service_cb)

    def control_cb(self, msg):
        cmd = Twist()
        cmd.linear.x = msg.speed
        cmd.angular.z = msg.steer
        self.control_pup.publish(cmd)


    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)


if __name__ == "__main__":
    car = AutoRCCar()
    car.run()