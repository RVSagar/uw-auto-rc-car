#!/usr/bin/env python2
import rospy
import numpy as np

from std_msgs.msg import Float64

from auto_rc_car_api.msg import carSteering
from auto_rc_car_api.srv import carLidar, carCamera, carControls
from cv_bridge import CvBridge, CvBridgeError
import cv2

class AutoRCCarClient:

    def __init__(self):

        rospy.init_node('rc_car_client')
        rospy.sleep(0.5)

        lidar_topic = '/racecar_api/lidar'
        rospy.wait_for_service(lidar_topic)
        self.lidar_srv = rospy.ServiceProxy('/racecar_api/lidar', carLidar)

        camera_rgb_topic = '/racecar_api/camera/rgb'
        rospy.wait_for_service(camera_rgb_topic)
        self.camera_rgb_srv = rospy.ServiceProxy(camera_rgb_topic, carCamera)

        camera_depth_topic = '/racecar_api/camera/depth'
        rospy.wait_for_service(camera_depth_topic)
        self.camera_depth_srv = rospy.ServiceProxy(camera_depth_topic, carCamera)

        camera_cloud_topic = '/racecar_api/camera/rgb'
        rospy.wait_for_service(camera_cloud_topic)
        self.camera_cloud_srv = rospy.ServiceProxy(camera_cloud_topic, carCamera)

        control_topic = '/racecar_api/last_control'
        rospy.wait_for_service(control_topic)
        self.control_srv = rospy.ServiceProxy(control_topic, carControls)

        send_control_topic = '/racecar/api_internal/control'
        self.control_pub = rospy.Publisher(send_control_topic, carSteering, queue_size=3)


    def get_latest_lidar(self):
        data, dt = self.timed_service(self.lidar_srv)
        return data.lidar, dt

    def get_latest_camera_rgb(self):
        data, dt = self.timed_service(self.camera_rgb_srv)
        return data.camera, dt

    def get_latest_camera_depth(self):
        data, dt = self.timed_service(self.camera_depth_srv)
        return data.camera, dt

    def get_latest_camera_cloud(self):
        data, dt = self.timed_service(self.camera_cloud_srv)
        return data.cloud, dt

    def get_latest_controls(self):
        data, dt = self.timed_service(self.control_srv)
        return data.controls, dt

    def send_control(self, speed, steer):
        msg = carSteering()
        msg.speed = speed
        msg.steer = steer
        self.control_pub.publish(msg)

    def camera_to_cv_image(self, data):
        try:
            cv_image = CvBridge().imgmsg_to_cv2(data, "bgr8")
            return cv_image
        except CvBridgeError as e:
            print(e)

    def timed_service(self, srv):
        t1 = self.get_time()
        ret = srv()
        t2 = self.get_time()
        return ret, (t2 - t1).to_sec()

    def get_time(self):
        return rospy.Time.now()

    def software_sleep(self, dt):
        rospy.sleep(dt)


if __name__ == "__main__":
    car = AutoRCCarClient()
    
    rospy.sleep(1.0)

    print("Starting")
    dist = 0

    while not rospy.is_shutdown():

        if dist < 3:
            car.send_control(-0.1,0)
        elif dist > 5:
            car.send_control(0.1, 0)

        l, dt1 = car.get_latest_lidar()
        tot = 0
        num = 0
        i = 0
        for r in l.ranges:
            angle = l.angle_min + i*l.angle_increment

            if angle > -0.25 and angle < 0.25:
                if not np.isinf(r) and not np.isnan(r) and not r == 0.0:
                    tot += r
                    num += 1
            i += 1
        
        if num != 0:
            dist = tot / num

        img, dt2 = car.get_latest_camera()
        img = car.camera_to_cv_image(img)

        data = "Dist = %f     Data Transfer Time = %f" % (dist, dt1+dt2)
        cv2.putText(img, data, (0,30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0))
        cv2.imshow("Car Camera", img)
        cv2.waitKey(1000)