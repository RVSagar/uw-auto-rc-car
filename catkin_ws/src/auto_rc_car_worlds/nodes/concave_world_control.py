#!/usr/bin/python
import rospy
import numpy as np
import math
import time

from std_msgs.msg import Float64

class StopLight:
    def __init__(self, ns, time_green, time_red):
        self.time_green = time_green
        self.time_red = time_red
        
        self.period = self.time_green + self.time_red

        self.pub = rospy.Publisher("/" + ns + "/stoplight_controller/command", Float64, queue_size=1)

    def send(self, t):
        t = t % self.period
        if t < self.time_green:
            self.pub.publish(0)
        else:
            self.pub.publish(3.1415)

class Pedestrian:
    def __init__(self, ns, positions, times):
        self.positions = positions
        self.times = times

        self.period = self.times[-1]

        self.pub = rospy.Publisher("/" + ns + "/pedestrian_controller/command", Float64, queue_size=1)
    
    def send(self, t):
        t = t % self.period

        i = 0
        while t > self.times[i]:
            i += 1
        
        ts = self.times[i-1]
        tf = self.times[i]

        dt = t - ts
        T = tf - ts
        a = dt / T

        p = a * self.positions[i] + (1 - a)*self.positions[i-1]
        self.pub.publish(p)


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

    ped0 = Pedestrian("pedestrian0", [0, 4, 4, 0], [0, 10, 20, 30])
    pedestrians = [ped0]

    c = WorldController(stoplights, pedestrians)
    c.run()

