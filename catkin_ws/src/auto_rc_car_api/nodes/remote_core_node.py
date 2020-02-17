#!/usr/bin/env python2
import rospy

from auto_rc_car_api.servers import AutoRCCarRemoteCore

if __name__ == "__main__":
    car = AutoRCCarRemoteCore()
    car.run()