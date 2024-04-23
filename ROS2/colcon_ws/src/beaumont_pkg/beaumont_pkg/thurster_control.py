import rclpy
from rclpy.node import Node 
from eer_messages.msg import ThrusterMultipliers
from std_msgs.msg import String
import json
from adafruit_pca9685 import PCA9685
import threading
import board

# Configure minimum, middle and maximum pulse lengths (out of 4096)
# Values should be adjusted so that the center arms the thrusters
MIN_SPEED = 3000
MAX_SPEED = 15000
CENTER_SPEED = 9000

THRUSTER = {
    "surge": 0,
    "sway": 1,
    "heave": 12,
    "pitch": 3,
    "roll": 4,
    "yaw": 2,
    "null": 6,
    "null": 7
}

# Below is what we should eventually have for the THRUSTER dictionary. Upon recieving pilot input, 
# rov math will be performed and translated into values corresponding to the dictionary below
# THRUSTER = {
#     "for-port-vert": 0,
#     "for-star-vert": 1,
#     "aft-port-vert": 2,
#     "aft-star-vert": 3,
#     "for-port-horz": 4,
#     "for-star-horz": 5,
#     "aft-port-horz": 6,
#     "aft-star-horz": 7
# }

ACCELERATION = 75

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
        pwm.channels[ch].duty_cycle = CENTER_SPEED

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
                self.pwm.channels[self.ch].duty_cycle = self.current
            else:
                self.current = self.target
                self.pwm.channels[self.ch].duty_cycle = self.current
        # print(f"Channel {self.ch}, duty cycle is {self.current}")

class ThrusterDataSubscriber(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 100)
        self.pilot_listener = self.create_subscription(String, 'controller_input', self.pilot_listener_callback, 1000)
        # self.multiple_query_service = self.create_service(Multipliers, "multipliers_query", self.multipliers_query_callback)

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

        self.power = 20
        self.sway = 0
        self.heave = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

        self.accepted_actions = ("surge", "sway", "heave", "pitch", "roll", "yaw")
        self.i2c = board.I2C()

        try:
            self.pwm = PCA9685(self.i2c)            
            self.ports = {}
            self.pwm.frequency = 100
        except:
            self.get_logger().error("NOTHING ON I2C BUS!")

        self.connected_devices = [] 
        for i in range(15): 
            try:
                self.ports[i] = Thruster(self.pwm, i)
                self.connected_devices.append(i)
            except:
                print("No thruster on channel ", i)

        threading.Thread(target=self.tick, daemon=True).start()
        

    def copilot_listener_callback(self, msg):
        self.get_logger().info(f"{msg} recieved in ThrusterControl")
        self.power = msg.power
        self.sway = msg.sway
        self.heave = msg.heave
        self.pitch = msg.pitch
        self.roll = msg.roll
        self.yaw = msg.yaw
        #TODO ROV Math

    def tick(self):
        while True:
            for i in self.connected_devices:
                self.ports[i].tick()

    def pilot_listener_callback(self, msg):  
        controller_inputs = json.loads(msg.data)
        for controller_input in controller_inputs:
            if (controller_input[1] in self.accepted_actions):
                # WOULD DO THRUSTER MATH HERE. CODE BELOW IS TEMPORARY
                if THRUSTER[controller_input[1]] in self.connected_devices:
                    self.ports[THRUSTER[controller_input[1]]].fly(controller_input[0])


def main(args=None):
    rclpy.init(args=args)

    thruster_data_subscriber = ThrusterDataSubscriber()

    rclpy.spin(thruster_data_subscriber)

    thruster_data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
