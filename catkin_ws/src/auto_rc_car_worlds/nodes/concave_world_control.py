#!/usr/bin/python
import rospy
import numpy as np
import math
import time

from std_msgs.msg import Float64

from world_objects import StopLight, Pedestrian


class WorldController:
    def __init__(self, stoplights, pedestrians):
        self.t0 = rospy.Time.now().to_sec()
        self.T = 0.1

        self.stoplights = stoplights
        self.pedestrians = pedestrians

    def run(self):
        while not rospy.is_shutdown():
            self.control_loop()
            rospy.sleep(self.T)

    def control_loop(self):
        t = rospy.Time.now().to_sec() - self.t0

        for light in self.stoplights:
            light.send(t)
        for ped in self.pedestrians:
            ped.send(t)



if __name__ == "__main__":
    print("Running World Control")
    rospy.init_node("concave_world_control")

    light0 = StopLight("sign0", 10, 10)
    light1 = StopLight("sign1", 10, 10)
    light2 = StopLight("sign2", 10, 10)
    stoplights = [light0, light1, light2]

    ped0 = Pedestrian("pedestrian0", [0, 0, 5, 5, 0], [0, 10, 20, 30, 40])
    pedestrians = [ped0]

    c = WorldController(stoplights, pedestrians)
    c.run()

