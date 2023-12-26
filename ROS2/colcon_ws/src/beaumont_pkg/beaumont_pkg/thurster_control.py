import rclpy
from rclpy.node import Node 
from eer_messages.msg import ThrusterVals


class ThrusterDataSubscriber(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.subscription = self.create_subscription(ThrusterVals, 'thruster_vals', self.listener_callback, 10)
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f"{msg} recieved in ThrusterControl")
        #TODO ROV Math

def main(args=None):
    rclpy.init(args=args)

    thruster_data_subscriber = ThrusterDataSubscriber()

    rclpy.spin(thruster_data_subscriber)

    thruster_data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
