import rclpy
from rclpy.node import Node 
from eer_messages.msg import ThrusterMultipliers
from std_msgs.msg import String
import json
import Adafruit_PCA9685
import threading

# Configure minimum, middle and maximum pulse lengths (out of 4096)
# Values should be adjusted so that the center arms the thrusters
MIN_SPEED = 1100
MAX_SPEED = 1900
CENTER_SPEED = 110

ACCELERATION = 75
#1900 1500 1100 maps to 180 90 0

class Thruster:
    """Thruster class."""
    def __init__(self, pwm, ch):
        """
        Setup the thruster or servo.

        :param pwm: I2C PWM driver controller object
        :param ch: pwm channel number for the thruster or servo controller
        """
        self.pwm = pwm
        self.ch = ch

        self.target = CENTER_SPEED
        self.current = CENTER_SPEED

        # Arm thruster
        pwm.set_pwm(ch, 0, CENTER_SPEED)

    def thruster_scale(self, thruster):
        """
        Scale thruster speed(-1.0 to 1.0) to pwm min/center/max limits.

        :param thruster: thruster to scale
        """
        if thruster >= 0:
            t = int(CENTER_SPEED + (MAX_SPEED - CENTER_SPEED) * thruster)
        else:
            t = int(CENTER_SPEED + (CENTER_SPEED - MIN_SPEED) * thruster)
        return t

    def fly(self, speed):
        """
        Drive thrusters using given speed parameters.

        :param speed: speed of thrusters with valid range between -1.0 to 1.0
        """
        self.target = self.thruster_scale(speed)

    def tick(self):
        if(self.current != self.target):
            if(abs(self.target - self.current) > ACCELERATION):
                direction = (self.target - self.current) / abs(self.target - self.current)
                self.current += int(direction * ACCELERATION)
                self.pwm.set_pwm(self.ch, 0, self.current)
            else:
                self.current = self.target
                self.pwm.set_pwm(self.ch, 0, self.current)

class ThrusterDataSubscriber(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 10)
        self.pilot_listener = self.create_subscription(String, 'controller_input', self.pilot_listener_callback, 10)

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

        self.accepted_actions = ("surge", "sway", "heave", "pitch", "roll", "yaw")

        self.pwm = Adafruit_PCA9685.PCA9685()
        self.pwm.set_pwm_freq(100)

        self.test_servo = Thruster(self.pwm, 0)

        threading.Thread(target=self.tick, daemon=True).start()

    def copilot_listener_callback(self, msg):
        self.get_logger().info(f"{msg} recieved in ThrusterControl")
        #TODO ROV Math

    def tick(self):
        while True:
            self.test_servo.tick()

    def pilot_listener_callback(self, msg):  
        controller_inputs = json.loads(msg.data)
        for controller_input in controller_inputs:
            if (controller_input[1] in self.accepted_actions):
                print(controller_input)
                if controller_input[1] == "surge":
                    self.get_logger().info(f"{msg} recieved in ThrusterControl")
                    self.test_servo.fly(controller_input[0])

    # def execute(self, thruster_values):
    #     print(thruster_values)
    #     for i in thruster_values:
    #         if i == 'for-port-vert':
    #             dt = self.ports.get(0)
    #             dt.fly(-thruster_values[i])
    #         elif i == 'for-star-vert':
    #             dt = self.ports.get(1)
    #             dt.fly(thruster_values[i])
    #         elif i == 'aft-port-vert':
    #             dt = self.ports.get(2)
    #             dt.fly(-thruster_values[i])
    #         elif i == 'aft-star-vert':
    #             dt = self.ports.get(3)
    #             dt.fly(thruster_values[i])
    #         elif i == 'for-port-horz':
    #             dt = self.ports.get(4)
    #             dt.fly(thruster_values[i])
    #         elif i == 'for-star-horz':
    #             dt = self.ports.get(5)
    #             dt.fly(thruster_values[i])
    #         elif i == 'aft-port-horz':
    #             dt = self.ports.get(6)
    #             dt.fly(thruster_values[i])
    #         elif i == 'aft-star-horz':
    #             dt = self.ports.get(7)
    #             dt.fly(thruster_values[i])


def main(args=None):
    rclpy.init(args=args)

    thruster_data_subscriber = ThrusterDataSubscriber()

    rclpy.spin(thruster_data_subscriber)

    thruster_data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
