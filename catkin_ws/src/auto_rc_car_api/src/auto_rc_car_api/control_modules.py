
import rospy

from std_msgs.msg import Float64

from auto_rc_car_api.msg import carSteering

class BaseControlModule:
    def __init__(self):
        self.max_steer = 1.0
        self.max_current = 7.0

        self.steer_pub = None
        self.speed_pub = None
        self.brake_pub = None

        self.last_control = None

    def get(self):
        return self.last_control

    def send_control(self, msg):
        self.last_control = msg
        
        speed = self.speed_to_signal(msg.speed)
        steer = self.steer_to_signal(msg.steer)

        self.steer_pub.publish(Float64(steer))
        self.speed_pub.publish(Float64(speed))


    def brake(self):
        self.send_control(0, 0)
        if self.brake_pub is not None:
            self.brake_pub.publish(1)

    def steer_to_signal(self, steer):
        raise NotImplementedError

    def speed_to_signal(self, speed):
        raise NotImplementedError



class SimulationControlModule(BaseControlModule):
    def __init__(self):
        self.steer_pub = rospy.Publisher('/racecar/internal/steering_controller/command', Float64, queue_size=3)
        self.speed_pub = rospy.Publisher('/racecar/internal/speed_controller/command', Float64, queue_size=3)
        self.speed_K = 1.0
        self.anti_deadband = 0
        self.max_speed = 99
        self.brake_pub = None
        self.control_pub = rospy.Publisher('/racecar/api_internal/control', carSteering, queue_size=3)

    def speed_to_signal(self, speed):
        return speed
    
    def steer_to_signal(self, steer):
        return steer


class RealControlModule(BaseControlModule):
    def __init__(self):
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=3)
        self.speed_pub = rospy.Publisher('/commands/motor/current', Float64, queue_size=3)
        self.brake_pub = rospy.Publisher('commands/motor/brake', Float64, queue_size=1)
        self.speed_K = -0.5
        self.anti_deadband = 1
        self.control_pub = rospy.Publisher('/racecar/api_internal/control', carSteering, queue_size=3)

    def steer_to_signal(self, steer):
        # Return servo position
        # Angle in should be -th to th max for steering
        if steer < -self.max_steer:
            steer = -self.max_steer
        if steer > self.max_steer:
            steer = self.max_steer
        
        # Correct direction
        steer = -steer
        # Transform to -1 to 1
        steer = steer / self.max_steer
        # Transform to 0-1
        steer = steer/2.0 + 0.5
        # Correct bias
        steer = steer - 0.055
        return steer

    def speed_to_signal(self, speed):
        # Return current
        current = speed * self.speed_K
        if current > 0.1 and current < self.anti_deadband:
            current = current + self.anti_deadband
        if current < -0.1 and current > -self.anti_deadband:
            current = current - self.anti_deadband
        
        if current < -self.max_current:
            current = -self.max_current
        if current > self.max_current:
            current = self.max_current
        
        return current 