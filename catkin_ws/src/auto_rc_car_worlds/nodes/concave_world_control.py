#!/usr/bin/python
import rospy
import numpy as np
import math
import time

from std_msgs.msg import Float64

from world_objects import StopLight, Pedestrian, WorldObserver, WorldController



if __name__ == "__main__":
    print("Running World Control")
    rospy.init_node("concave_world_control")

    dt = 0.1
    WO = WorldObserver()

    light0 = StopLight("sign0", 10, 10)
    light1 = StopLight("sign1", 8, 12)
    light2 = StopLight("sign2", 12, 8)
    light3 = StopLight("sign3", 7, 7)
    stoplights = [light0, light1, light2, light3]

    ped0 = Pedestrian("pedestrian0", 2, 20, dt, WO)
    ped1 = Pedestrian("pedestrian1", 1.75, 14, dt, WO)
    ped2 = Pedestrian("pedestrian2", 2, 18, dt, WO)
    ped3 = Pedestrian("pedestrian3", 1.75, 20, dt, WO)
    pedestrians = [ped0, ped1, ped2, ped3]

    c = WorldController(stoplights, pedestrians, dt)
    c.run()

