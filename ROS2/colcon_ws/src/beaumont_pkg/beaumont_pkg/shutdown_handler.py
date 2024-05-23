import os
import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class ShutdownHandler(Node):

    def __init__(self):
        super().__init__('shutdown_handler')
        self.subscription = self.create_subscription(
            bool,
            'shutdown',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        #TODO: figure out which ip is which
        os.exec("ssh pizero0@192.168.2.13 'poweroff'")
        os.exec("ssh pizero1@192.168.2.10 'poweroff'")
        os.exec("ssh pizero2@192.168.2.11 'poweroff'")
        os.exec("ssh pizero3@192.168.2.12 'poweroff'")
        os.exec("poweroff")



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = ShutdownHandler()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()