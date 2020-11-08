
import rospy
import math

from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

from auto_rc_car_api.msg import carSteering

class APIOnlyJoyListener:
    def __init__(self):
        pass
    def api_allowed(self):
        return True
    def get_joy_command(self):
        cmd = carSteering()
        cmd.speed = 0
        cmd.steer = 0
        return cmd

class TeleopJoyListener:
    def __init__(self):
        self.last_msg_time = self.now()

        self.api_active = False
        self.joy_active = False

        self.msg = None

        self.sub = rospy.Subscriber('/joy', Joy, self.callback)


    def now(self):
        return rospy.Time.now().to_sec()

    def callback(self, msg):
        self.last_msg_time = self.now()

        self.joy_active = msg.buttons[4]
        self.api_active = msg.buttons[5]

        self.msg = msg

    def got_command_recently(self):
        return self.now() - self.last_msg_time < 0.25

    def get_joy_command(self):
        if self.got_command_recently() and self.joy_active:
            cmd = carSteering()
            cmd.speed = self.msg.axes[1]
            cmd.steer = self.msg.axes[0]
        else:
            cmd = carSteering()
            cmd.speed = 0
            cmd.steer = 0
        return cmd

    def api_allowed(self):
        return self.got_command_recently() and self.api_active and not self.joy_active



class BaseControlModule:
    def __init__(self, joy_module):
        self.max_steer = 1.0
        self.max_current = 7.0

        self.steer_pub = None
        self.speed_pub = None
        self.brake_pub = None

        self.last_control = None
        
        self.joy_module = joy_module

    def get(self):
        return self.last_control

    def send_control(self, msg):
        if not self.joy_module.api_allowed():
            msg = self.joy_module.get_joy_command()


        self.last_control = msg
        
        speed = self.speed_to_signal(msg.speed)
        steer = self.steer_to_signal(msg.steer)

        self.publish_speed(speed)
        self.publish_steer(steer)


    def brake(self):
        msg = carSteering()
        msg.speed = 0
        msg.steer = 0
        self.send_control(msg)
        if self.brake_pub is not None:
            self.brake_pub.publish(1)

    def steer_to_signal(self, steer):
        raise NotImplementedError

    def publish_steer(self, steer):
        raise NotImplementedError

    def speed_to_signal(self, speed):
        raise NotImplementedError

    def publish_speed(self, speed):
        raise NotImplementedError



class SimulationControlModule(BaseControlModule):
    def __init__(self, joy_module):
        self.steer_pub = rospy.Publisher('/racecar/internal/steering_controller/command', Float64, queue_size=3)
        self.left_front_wheel_pub = rospy.Publisher('/racecar/internal/left_front_controller/command', Float64, queue_size=1)
        self.right_front_wheel_pub = rospy.Publisher('/racecar/internal/right_front_controller/command', Float64, queue_size=1)
        self.left_rear_wheel_pub = rospy.Publisher('/racecar/internal/left_rear_controller/command', Float64, queue_size=1)
        self.right_rear_wheel_pub = rospy.Publisher('/racecar/internal/right_rear_controller/command', Float64, queue_size=1)
        self.speed_K = 1.0
        self.anti_deadband = 0
        self.max_speed = 99
        self.brake_pub = None
        self.control_pub = rospy.Publisher('/racecar/api_internal/control', carSteering, queue_size=3)
        self.joy_module = joy_module

    def speed_to_signal(self, speed):
        return speed
    
    def steer_to_signal(self, steer):
        return steer

    def publish_speed(self, speed):
        self.left_front_wheel_pub.publish(Float64(speed))
        self.right_front_wheel_pub.publish(Float64(speed))
        self.left_rear_wheel_pub.publish(Float64(speed))
        self.right_rear_wheel_pub.publish(Float64(speed))

    def publish_steer(self, steer):
        self.steer_pub.publish(Float64(steer))



class RealControlModule(BaseControlModule):
    def __init__(self, joy_module):
        self.steer_pub = rospy.Publisher('/commands/servo/position', Float64, queue_size=3)
        self.speed_pub = rospy.Publisher('/commands/motor/current', Float64, queue_size=3)
        self.brake_pub = rospy.Publisher('commands/motor/brake', Float64, queue_size=1)
        self.speed_K = -0.5
        self.anti_deadband = 1
        self.control_pub = rospy.Publisher('/racecar/api_internal/control', carSteering, queue_size=3)
        self.joy_module = joy_module

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

    def publish_speed(self, speed):
        self.speed_pub.publish(Float64(speed))

    def publish_steer(self, steer):
        self.speed_pub.publish(Float64(steer))
