import rospy
import numpy as np
import math
import time

from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates

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
    def __init__(self, ns, amp, per, dt, world_obs):
        self.ns = ns
        self.t0 = 0.0
        self.dt = dt
        self.A = amp
        self.T = per

        self.pub = rospy.Publisher("/" + ns + "/pedestrian_controller/command", Float64, queue_size=1)
        self.world_obs = world_obs

        self.SAFE_DIST = 0.4 # Want pedestrians to not ram cars, don't want them to make safe decisons for us
        self.END_FRAC = 0.05
        self.COLL_ANG = 0.6
    
    def pos(self, t):
        return self.A*(0.5 - 0.5*math.cos(2*math.pi/self.T * t))

    def vel(self, t):
        return math.pi/self.T* self.A*math.sin(2*math.pi/self.T * t)

    def send(self, t):
        if self.can_move(t):
            self.t0 = self.t0 + self.dt

            p = self.pos(self.t0)
            self.pub.publish(p)

    def can_move(self, t):
        try:
            pose, twist = self.world_obs.get(self.ns)
            car_pose, car_twist = self.world_obs.get("racecar")
        except ValueError as e:
            return False

        w = pose.orientation.w
        yaw = 2.0 * math.acos(w)
        p = self.pos(self.t0)

        if p/self.A < self.END_FRAC or p/self.A > 1.0-self.END_FRAC:
            # At ends of path. Might mess with numerical stuff at low vel
            return True 

        dx = car_pose.position.x - pose.position.x - p*math.cos(yaw)
        dy = car_pose.position.y - pose.position.y - p*math.sin(yaw)
        dist = math.sqrt(dx*dx + dy*dy)
        if dist > self.SAFE_DIST:
            #print(self.ns + " far awar")
            return True

        v = self.vel(self.t0)
        if abs(v) < 0.01:
            #print(self.ns + " not moving fast")
            return True

        

        vx = v*math.cos(yaw)
        vy = v*math.sin(yaw)

        dot = vx*dx + vy*dy
        if dot < 0:
            #print(self.ns + " dot < 0")
            return True
        try:
            n = dot / abs(dist*v + 0.01)/1.01
            ang = math.acos(n)
        except ValueError as e:
            #print(e)
            #print(dot, dist, v, n)
            n = n / (abs(n) + 0.01)
            ang = math.acos(n)
        
        if ang < self.COLL_ANG or ang > math.pi-self.COLL_ANG:
            #print(self.ns + " in potential collision; ang=%f, dot=%f" % (ang, dot))
            return False

        #print(self.ns + " not in potential collision; ang=%f, dot=%f" % (ang, dot))

        return True


class WorldObserver:
    def __init__(self):
        self.world_state_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.link_state_cb)
        self.msg = None

        while self.msg is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def link_state_cb(self, msg):
        self.msg = msg

    def get(self, name):
        ind = self.msg.name.index(name)
        return self.msg.pose[ind], self.msg.twist[ind]

class WorldController:
    def __init__(self, stoplights, pedestrians, dt):
        self.t0 = rospy.Time.now().to_sec()
        self.T = dt

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