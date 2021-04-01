#!/usr/bin/env python
import cv2
import numpy as np
import logging
import math
import datetime
import sys
from auto_rc_car_demos.basic_camera import BasicCameraCalc, HandCodedLaneFollower

if __name__ == '__main__':
    lane_follower = HandCodedLaneFollower()

    lane_follower.test_photo('test1.png')