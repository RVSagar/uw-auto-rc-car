#!/usr/bin/python
import rospy
import numpy as np
import math
import time

from std_msgs.msg import Float64

from world_objects import StopLight, Pedestrian, WorldController, WorldObserver

if __name__ == "__main__":
    print("Running World Control")
    rospy.init_node("concave_world_control")

    dt = 0.1
    WO = WorldObserver()

    light0 = StopLight("sign0", 10, 10)
    light1 = StopLight("sign1", 5, 5)
    stoplights = [light0, light1]

    ped0 = Pedestrian("pedestrian0", 1.5, 40, dt, WO)
    ped1 = Pedestrian("pedestrian1", 3, 28, dt, WO)
    pedestrians = [ped0, ped1]

    c = WorldController(stoplights, pedestrians, dt)
    c.run()

