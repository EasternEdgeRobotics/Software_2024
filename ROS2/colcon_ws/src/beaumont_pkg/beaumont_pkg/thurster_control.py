import rclpy
from rclpy.node import Node 
from eer_messages.msg import ThrusterMultipliers
from std_msgs.msg import String
import json


class ThrusterDataSubscriber(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 10)
        self.pilot_listener = self.create_subscription(String, 'controller_input', self.pilot_listener_callback, 10)

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

    def copilot_listener_callback(self, msg):
        self.get_logger().info(f"{msg} recieved in ThrusterControl")
        #TODO ROV Math

    def pilot_listener_callback(self, msg):  
        controller_input = json.loads(msg.data)
        thruster_directions = ("surge", "sway", "heave", "pitch", "roll", "yaw")
        if (controller_input[1] in thruster_directions):
            print(controller_input)
            #TODO ROV Math


def main(args=None):
    rclpy.init(args=args)

    thruster_data_subscriber = ThrusterDataSubscriber()

    rclpy.spin(thruster_data_subscriber)

    thruster_data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
