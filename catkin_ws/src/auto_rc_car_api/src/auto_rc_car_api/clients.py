#!/usr/bin/env python2
import rospy
import numpy as np

from sensor_msgs.msg import LaserScan, Image, PointCloud2
from std_msgs.msg import Float64

from data_modules import RemoteDataModule, LocalDataModule, BaseDataModule
from auto_rc_car_api.msg import carSteering
from auto_rc_car_api.srv import carLidar, carCamera, carControls
from cv_bridge import CvBridge, CvBridgeError
import cv2





class AutoRCCarClientBase:
    def __init__(self):
        raise NotImplementedError

    def get_latest_lidar(self):
        return self.lidar.get()

    def get_latest_camera_rgb(self):
        return self.camera_rgb.get()

    def get_latest_camera_depth(self):
        return self.camera_depth.get()

    def get_latest_camera_cloud(self):
        return self.camera_cloud.get()

    def get_latest_controls(self):
        return self.controls.get()

    def camera_to_cv_image(self, data):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
            return cv_image
        except CvBridgeError as e:
            print(e)

    def get_time(self):
        return rospy.Time.now()

    def software_sleep(self, dt):
        rospy.sleep(dt)



class AutoRCCarClientRemote(AutoRCCarClientBase):

    def __init__(self):
        rospy.init_node('rc_car_client')
        print("Starting Client")

        self.lidar = RemoteDataModule(rospy, '/racecar_api/lidar', carLidar, lambda msg: msg.lidar)
        self.camera_rgb = RemoteDataModule(rospy, '/racecar_api/camera/rgb', carCamera, lambda msg: msg.camera)
        self.camera_depth = RemoteDataModule(rospy, '/racecar_api/camera/depth', carCamera, lambda msg: msg.camera)
        self.camera_cloud = RemoteDataModule(rospy, '/racecar_api/camera/cloud', carCamera, lambda msg: msg.cloud)
        self.controls = RemoteDataModule(rospy, '/racecar_api/last_control', carControls, lambda msg: msg.controls)

        self.control_pub = rospy.Publisher('/racecar/api_internal/control', carSteering, queue_size=3)

        print("Client Initialized")


    def send_control(self, speed, steer):
        msg = carSteering()
        msg.speed = speed
        msg.steer = steer
        self.control_pub.publish(msg)




class AutoRCCarClientLocal(AutoRCCarClientBase):

    def __init__(self):
        rospy.init_node('rc_car_client')
        print("Starting Client")

        self.lidar = LocalDataModule(rospy, '/racecar/out/laser_scan', LaserScan)
        self.camera_rgb = LocalDataModule(rospy, '/racecar/out/stereo_camera/rgb/image', Image)
        self.camera_depth = LocalDataModule(rospy, '/racecar/out/stereo_camera/depth/image', Image)
        self.camera_cloud = LocalDataModule(rospy, '/racecar/out/stereo_camera/depth_cloud/points', PointCloud2)
        self.controls = LocalDataModule(rospy, '/racecar/api_internal/control', carSteering)

        self.control_pub = rospy.Publisher('/racecar/api_internal/control', carSteering, queue_size=3)
        self.steer_pub = rospy.Publisher('/racecar/internal/steering_controller/command', Float64, queue_size=3)
        self.speed_pub = rospy.Publisher('/racecar/internal/speed_controller/command', Float64, queue_size=3)


        print("Client Initialized")

    def send_control(self, speed, steer):
        msg = carSteering()
        msg.speed = speed
        msg.steer = steer
        self.control_pub.publish(msg)
        self.steer_pub.publish(Float64(steer))
        self.speed_pub.publish(Float64(speed))

def CreateRCCar():
    client_type = rospy.get_param('/client_comm_type', 'remote')

    if client_type == 'remote':
        return AutoRCCarClientRemote()
    if client_type == 'local':
        return AutoRCCarClientLocal()
    
    raise Exception("Client comm type not found, or does not match available types")