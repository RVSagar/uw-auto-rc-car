#!/usr/bin/env python2
import rospy

from auto_rc_car_api.clients import AutoRCCarClientLocal, AutoRCCarClientRemote
from control_modules import SimulationControlModule, RealControlModule


class Contexts:
    SIM = "sim"
    REAL = "real"

class Locations:
    LOCAL = "local"
    REMOTE = "remote"

class ClientGenerator:
    @staticmethod
    def CreateRCCarClient( sim_real, location):

        controls = None
        if sim_real == Contexts.SIM:
            controls = SimulationControlModule()
        elif sim_real == Contexts.SIM:
            context = RealControlModule()
        else:
            raise Exception("Please provide context 'sim' or 'real' to creation function")

        
        if location == Locations.LOCAL:
            return AutoRCCarClientLocal(controls)
        elif location == Locations.REMOTE:
            return AutoRCCarClientRemote(controls)
        raise Exception("Please provide location 'local' or 'remote' to creation function")
