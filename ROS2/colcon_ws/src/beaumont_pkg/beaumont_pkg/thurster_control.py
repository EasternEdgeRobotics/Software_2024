import rclpy
from rclpy.node import Node 
from eer_messages.msg import ThrusterMultipliers
from eer_messages.srv import Multipliers
from std_msgs.msg import String
import json


class ThrusterDataSubscriber(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 10)
        self.pilot_listener = self.create_subscription(String, 'controller_input', self.pilot_listener_callback, 10)
        self.multiple_query_service = self.create_service(Multipliers, "multipliers_query", self.multipliers_query_callback)

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

        self.power = 20
        self.sway = 0
        self.heave = 0
        self.pitch = 0
        self.roll = 0
        self.yaw = 0

    def copilot_listener_callback(self, msg):
        self.get_logger().info(f"{msg} recieved in ThrusterControl")
        self.power = msg.power
        self.sway = msg.sway
        self.heave = msg.heave
        self.pitch = msg.pitch
        self.roll = msg.roll
        self.yaw = msg.yaw
        #TODO ROV Math

    def pilot_listener_callback(self, msg):  
        controller_input = json.loads(msg.data)
        thruster_directions = ("surge", "sway", "heave", "pitch", "roll", "yaw")
        if (controller_input[1] in thruster_directions):
            print(controller_input)
            #TODO ROV Math

    def multipliers_query_callback(self, request, response):
        response.power = self.power
        response.sway = self.sway
        response.heave = self.heave
        response.pitch = self.pitch
        response.roll = self.roll
        response.yaw = self.yaw
        return response

def main(args=None):
    rclpy.init(args=args)

    thruster_data_subscriber = ThrusterDataSubscriber()

    rclpy.spin(thruster_data_subscriber)

    thruster_data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
