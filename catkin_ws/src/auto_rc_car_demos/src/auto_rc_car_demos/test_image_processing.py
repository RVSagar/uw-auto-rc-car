#!/usr/bin/env python
import cv2
import numpy as np
import logging
import math
import datetime
import sys
import rospkg
from auto_rc_car_demos.basic_camera import BasicCameraCalc, HandCodedLaneFollower

if __name__ == '__main__':
    lane_follower = HandCodedLaneFollower()

    rospack = rospkg.RosPack()
    package_dir = rospack.get_path("auto_rc_car_demos")

    lane_follower.test_photo(package_dir + '/test_images/test6.png')