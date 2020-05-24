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