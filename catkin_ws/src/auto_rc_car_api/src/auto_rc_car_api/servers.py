#!/usr/bin/env python2
import rospy

from std_msgs.msg import Float64

from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2

from geometry_msgs.msg import Twist

from auto_rc_car_api.srv import carLidar, carLidarResponse
from auto_rc_car_api.srv import carCamera, carCameraResponse
from auto_rc_car_api.srv import carControls, carControlsResponse
from auto_rc_car_api.msg import carSteering


class RemoteDataModule:
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


class AutoRCCarRemoteCore:

    def __init__(self):

        rospy.init_node('rc_car_core')
        print("Initializing Remote Core Node!")

        self.lidar = RemoteDataModule(rospy,
                                   '/racecar/out/laser_scan', LaserScan,
                                   'racecar_api/lidar', carLidar, carLidarResponse)
        self.camera_rgb = RemoteDataModule(rospy,
                                   '/racecar/out/stereo_camera/rgb/image', Image,
                                   'racecar_api/camera/rgb', carCamera, carCameraResponse)

        self.camera_depth = RemoteDataModule(rospy,
                                   '/racecar/out/stereo_camera/depth/image', Image,
                                   'racecar_api/camera/depth', carCamera, carCameraResponse)

        self.camera_cloud = RemoteDataModule(rospy,
                                   '/racecar/out/stereo_camera/depth_cloud/points', PointCloud2,
                                   'racecar_api/camera/cloud', carCamera, carCameraResponse)

        self.controls = RemoteDataModule(rospy,
                                      '/racecar/api_internal/control', carSteering,
                                      'racecar_api/last_control', carControls, carControlsResponse)


    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(1)
