import rclpy
from rclpy.node import Node 
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json
import threading
from time import time

class ToolingController(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.pilot_listener = self.create_subscription(String, 'controller_input', self.pilot_listener_callback, 10) 
        
        self.last_input_time = time()
        self.inactivity_timeout = 1 # 1 second

        self.left_claw_publisher = self.create_publisher(Twist, "/demo/left_claw", 10) 
        self.right_claw_publisher = self.create_publisher(Twist, "/demo/right_claw", 10) 

        self.accepted_actions = ("open_claw", "close_claw") #This script is only responsible for tooling control related actions

        self.bot_yaw_to_claw_yaw_factor = 90

        self.lock = threading.Lock() #Used to synchronize all the threads (concurrent processes) which are accessing and modifing the same variables

        self.tooling_timeout = threading.Thread(target=self.tooling_timeout, daemon=True)
        self.tooling_timeout.start()

        self.pilot_listener

    def pilot_listener_callback(self, msg):  
        '''Called when new controller input from pilot is recieved'''
        controller_inputs = json.loads(msg.data)   

        for controller_input in controller_inputs:

            if controller_input[1] in self.accepted_actions:
                self.process_action(controller_input)
        
    def process_action(self,action):
        '''Publishes on ROS topics to control tooling'''
        self.lock.acquire()

        self.last_input_time = time()

        if action[1] == "open_claw":

            left_claw_velocity = Twist()
            right_claw_velocity = Twist()

            left_claw_velocity.angular.z = -0.5
            right_claw_velocity.angular.z = 0.5

            self.left_claw_publisher.publish(left_claw_velocity)
            self.right_claw_publisher.publish(right_claw_velocity)

        elif action[1] == "close_claw":

            left_claw_velocity = Twist()
            right_claw_velocity = Twist()

            left_claw_velocity.angular.z = float(0.5)
            right_claw_velocity.angular.z = float(-0.5)

            self.left_claw_publisher.publish(left_claw_velocity)
            self.right_claw_publisher.publish(right_claw_velocity)

        self.lock.release()
            

    def tooling_timeout(self):
        '''Ensures to stop tooling after a certain timeout of no input is recieved'''
        while True:

            self.lock.acquire()

            if (time()-self.last_input_time > self.inactivity_timeout):
                velocity = Twist()
                velocity.angular.z = float(0)

                self.left_claw_publisher.publish(velocity)
                self.right_claw_publisher.publish(velocity)

            self.lock.release()



def main(args=None):
    rclpy.init(args=args)

    tooling_controller = ToolingController()


    rclpy.spin(tooling_controller)

    tooling_controller.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
