import rclpy
from rclpy.node import Node 
from eer_messages.msg import ThrusterMultipliers, PilotInput
from std_msgs.msg import String
import json
from adafruit_pca9685 import PCA9685
import threading
import board
from math import sqrt


# Configure minimum, middle and maximum pulse lengths (out of 4096)
# Values should be adjusted so that the center arms the thrusters
MIN_SPEED = 3000
MAX_SPEED = 15000
CENTER_SPEED = 9000

ACCELERATION = 75


THRUSTER = {
    "for-port-top": 0,
    "for-star-top": 1,
    "aft-port-top": 2,
    "aft-star-top": 3,
    "for-port-bot": 4,
    "for-star-bot": 5,
    "aft-port-bot": 6,
    "aft-star-bot": 7
}

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

class ThrusterDataSubscriber(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 100)
        self.pilot_listener = self.create_subscription(PilotInput, 'controller_input', self.pilot_listener_callback, 1)

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener
        
        self.thrusters_detected = False

        try:
            self.i2c = board.I2C()
        except:
            self.get_logger().error("No Hardware on I2C bus")

        #################################################
        ################### THRUSTERS ###################
        #################################################

        self.power_multiplier = 20
        self.surge_multiplier = 0
        self.sway_multiplier = 0
        self.heave_multiplier = 0
        self.pitch_multiplier = 0
        self.yaw_multiplier = 0

           
        self.ports = {}
        self.connected_thrusters = [] 
        
        # Connect to PCA9685
        try:
            self.pwm = PCA9685(self.i2c)         
            self.pwm.frequency = 100
        except:
            self.get_logger().error("CANNOT FIND PCA9685 ON I2C!")

        for i in range(15): # The PCA9685 can connect to up to 16 ESCs (16 thrusters). 
            try:
                # It is assumed that, if thrusters are connected, they will be
                # connected in a way that matches the THRUSTER global dictionary
                self.ports[i] = Thruster(self.pwm, i)
                self.connected_thrusters.append(i)
            except:
                print("No thruster on channel ", i)

        if len(self.connected_thrusters) == 8:
            for thruster in self.connected_thrusters:
                if thruster in THRUSTER.values():
                    continue
                else:
                    break
            self.thrusters_detected = True

        threading.Thread(target=self.tick_thrusters, daemon=True).start()

        #################################################
        ################### THRUSTERS ###################
        #################################################

    def copilot_listener_callback(self, msg):
        # self.get_logger().info(f"{msg} recieved in ThrusterControl")
        self.power_multiplier = float(msg.power/100)
        self.surge_multiplier = float(msg.surge/100)
        self.sway_multiplier = float(msg.sway/100)
        self.heave_multiplier = float(msg.heave/100)
        self.pitch_multiplier = float(msg.pitch/100)
        self.yaw_multiplier = float(msg.yaw/100)

    def tick_thrusters(self):
        while True:
            for i in self.connected_thrusters:
                self.ports[i].tick()

    def pilot_listener_callback(self, msg): 

        thruster_values = self.rov_math(msg)

        if self.thrusters_detected:
            for thruster_position in list(THRUSTER.keys()):
                self.ports(THRUSTER[thruster_position]).fly(thruster_values[thruster_position])

    def rov_math(self, controller_inputs):

        thruster_values = {}

        surge = controller_inputs.surge * self.power_multiplier * self.surge_multiplier * 0.01
        sway = controller_inputs.sway * self.power_multiplier * self.sway_multiplier * 0.01
        yaw = controller_inputs.yaw * self.power_multiplier * self.yaw_multiplier * 0.01

        if controller_inputs.heave_up:
            heave = self.power_multiplier * self.heave_multiplier
        elif controller_inputs.heave_down:
            heave = - self.power_multiplier * self.heave_multiplier
        else:
            heave = controller_inputs.heave * self.power_multiplier * self.heave_multiplier * 0.01  

        if controller_inputs.pitch_up:
            pitch = self.power_multiplier * self.pitch_multiplier
        elif controller_inputs.pitch_down:
            pitch = - self.power_multiplier * self.pitch_multiplier
        else:
            pitch = controller_inputs.pitch * self.power_multiplier * self.pitch_multiplier * 0.01  

        sum_of_magnitudes_of_linear_movements = abs(surge) + abs(sway) + abs(heave)
        sum_of_magnitudes_of_rotational_movements = abs(pitch) + abs(yaw)

        strafe_power = sqrt(surge**2 + sway**2 + heave**2)
        strafe_scaling_coefficient = strafe_power / (sum_of_magnitudes_of_linear_movements) if strafe_power else 0
        strafe_average_coefficient = strafe_power / (strafe_power + sum_of_magnitudes_of_rotational_movements) if strafe_power or sum_of_magnitudes_of_rotational_movements else 0  
        rotation_average_coefficient = sum_of_magnitudes_of_rotational_movements / (strafe_power + sum_of_magnitudes_of_rotational_movements) if strafe_power or sum_of_magnitudes_of_rotational_movements else 0

        # Calculations below are based on thruster positions
        thruster_values["for-port-bot"] = ((-surge)+(sway)+(heave)) * strafe_scaling_coefficient * strafe_average_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient
        thruster_values["for-star-bot"] = ((-surge)+(-sway)+(heave)) * strafe_scaling_coefficient * strafe_average_coefficient + ((pitch)+(-yaw)) * rotation_average_coefficient
        thruster_values["aft-port-bot"] = ((surge)+(sway)+(heave)) * strafe_scaling_coefficient * strafe_average_coefficient + ((-pitch)+(-yaw)) * rotation_average_coefficient
        thruster_values["aft-star-bot"] = ((surge)+(-sway)+(heave)) * strafe_scaling_coefficient * strafe_average_coefficient + ((-pitch)+(yaw)) * rotation_average_coefficient
        thruster_values["for-port-top"] = ((-surge)+(sway)+(-heave)) * strafe_scaling_coefficient * strafe_average_coefficient + ((-pitch)+(yaw)) * rotation_average_coefficient
        thruster_values["for-star-top"] = ((-surge)+(-sway)+(-heave)) * strafe_scaling_coefficient * strafe_average_coefficient + ((-pitch)+(-yaw)) * rotation_average_coefficient
        thruster_values["aft-port-top"] = ((surge)+(sway)+(-heave)) * strafe_scaling_coefficient * strafe_average_coefficient + ((pitch)+(-yaw)) * rotation_average_coefficient
        thruster_values["aft-star-top"] = ((surge)+(-sway)+(-heave)) * strafe_scaling_coefficient * strafe_average_coefficient + ((pitch)+(yaw)) * rotation_average_coefficient

        ####################################################################
        ############################## DEBUG ###############################
        ####################################################################

        # Calculations below will calculate and display the net movement in all directions based on vector analysis

        # from math import cos, pi

        # self.get_logger().info(f"""
        # Surge: {surge}\n
        # Sway: {sway}\n
        # Heave: {heave}\n
        # Pitch: {pitch}\n
        # Yaw: {yaw}""")

        # self.get_logger().info(f"""
        # Strafe Power: {strafe_power}\n
        # Strafe Scaling Coefficent: {strafe_scaling_coefficient}\n
        # Strafe Average Coefficient: {strafe_average_coefficient}\n
        # Rotation Average Coefficent: {rotation_average_coefficient}""")

        # self.get_logger().info(f"""
        # for port bot: {thruster_values["for-port-bot"]}\n
        # for star bot: {thruster_values["for-star-bot"]}\n
        # aft port bot: {thruster_values["aft-port-bot"]}\n
        # aft star bot: {thruster_values["aft-star-bot"]}\n
        # for port top: {thruster_values["for-port-top"]}\n
        # for star top: {thruster_values["for-star-top"]}\n
        # aft port top: {thruster_values["aft-port-top"]}\n
        # aft star top: {thruster_values["aft-star-top"]}
        # cos(pi/3): {cos(pi/3)}""")

        # net_surge = cos(pi/3)*((-thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
        # net_sway = cos(pi/3)*((thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (-thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))
        # net_heave = cos(pi/3)*((thruster_values["for-port-bot"]) + (thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))
        
        # net_pitch = cos(pi/4)*((thruster_values["for-port-bot"]) + (thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (-thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
        # net_yaw = cos(pi/4)*((thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))

        # self.get_logger().info(f"""
        # Net Surge: {net_surge}\n
        # Net Sway: {net_sway}\n
        # Net Heave: {net_heave}\n
        # Net Pitch: {net_pitch}\n
        # Net Yaw: {net_yaw}""")

        
        ####################################################################
        ####################################################################
        ####################################################################

        return thruster_values

def main(args=None):
    rclpy.init(args=args)

    thruster_data_subscriber = ThrusterDataSubscriber()

    rclpy.spin(thruster_data_subscriber)

    thruster_data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
