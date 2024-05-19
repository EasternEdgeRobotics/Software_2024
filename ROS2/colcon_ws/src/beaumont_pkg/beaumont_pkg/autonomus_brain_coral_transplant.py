import threading
import time

from eer_messages.action import AutoMode 
from eer_messages.msg import ThrusterMultipliers, PilotInput
from eer_messages.srv import Config

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import cv2 
import numpy as np
import urllib.request

from time import time, sleep
import json

TOOLING_CAMERA_INDEX = 2

class AutonomusBrainCoralTransplant(Node):
    """Autonomus movement node."""

    def __init__(self):
        super().__init__('autonomus_brain_coral_transplant')

        self.camera_urls_client = self.create_client(Config, 'camera_urls')

        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            AutoMode,
            'autonomus_brain_coral_transplant',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
        self.pilot_publisher = self.create_publisher(PilotInput, "controller_input", 1)
        self.copilot_publisher = self.create_publisher(ThrusterMultipliers, "thruster_multipliers", 1)

        self.tooling_camera_stream_url = json.loads(self.fetch_tooling_camera_stream_url())[TOOLING_CAMERA_INDEX]
        self.get_logger().info(f"Autonomus Brain Coral Transplant Node Fetched {self.tooling_camera_stream_url}")

    def fetch_tooling_camera_stream_url(self):
        
        camera_url_config_request = Config.Request()
        camera_url_config_request.state = 1 # GET urls from database
        camera_url_config_request.data = "" # Not used
        
        self.camera_urls_client.wait_for_service()
        
        future = self.camera_urls_client.call_async(camera_url_config_request)
        
        rclpy.spin_until_future_complete(self,future)
        
        return future.result().result

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def enter_autonomus_mode(self, goal_handle):
        # Append the seeds for the Fibonacci sequence
        feedback_msg = AutoMode.Feedback()
        feedback_msg.status = "idek man"

        # Start executing the action
        for i in range(25):
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                self.get_logger().info('Goal aborted or canceled')
                break
            
            controller_input = PilotInput()
            
            if i < 12:
                feedback_msg.status = "Surging Forward"
                controller_input.surge = 100
            else:
                feedback_msg.status = "Surging Back"
                controller_input.surge = -100

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            sleep(1)

        controller_input = PilotInput()
        feedback_msg.status = "Resetting"
        goal_handle.publish_feedback(feedback_msg)
        self.pilot_publisher.publish(controller_input)

        # Opening mjpeg camera stream using opencv (working)
		# stream = urllib.request.urlopen('http://localhost:8880/cam.mjpg')
		# bytes = b''

		# while True:
		# 	bytes += stream.read(1024)
		# 	a = bytes.find(b'\xff\xd8')
		# 	b = bytes.find(b'\xff\xd9')
		# 	if a != -1 and b != -1:
		# 		jpg = bytes[a:b+2]
		# 		bytes = bytes[b+2:]
		# 		i = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
		# 		cv2.imshow('i', i)
		# 		if cv2.waitKey(1) == 27:
		# 			exit(0)   


		# controller_inputs = json.loads(controller_input.data)
		# for controller_input in controller_inputs:
		# 	if controller_input[1] != "enter_auto_mode":
		# 		return
		# 	time_in_second_1 = 9
		# 	time_in_second_3 = 4
			
		# 	self.get_logger().info("In Auto Mode")

		# 	#the function for movement 
		# 	def time_to_move():
		# 		start_time = time()
		# 		while True: ### this while loop makes the ROV move up for 20 seconds
		# 			y_movement = [-1, "heave"]
		# 			y_movement_string = String()
		# 			y_movement_string.data = json.dumps([y_movement]) 
		# 			# self.autonomus_controls_publisher.publish(y_movement_string)
		# 			claw_movement1 = [1,"open_claw"] ### this is closing the claw all the time
		# 			claw_movement_string1 = String()
		# 			claw_movement_string1.data = json.dumps([claw_movement1]) 
		# 			# self.autonomus_controls_publisher.publish(claw_movement_string1)
		# 			if (time()-start_time) > time_in_second_3:
		# 				y_movement = [0, "heave"]
		# 				y_movement_string = String()
		# 				y_movement_string.data = json.dumps([y_movement]) 
		# 				# self.autonomus_controls_publisher.publish(y_movement_string)
		# 				break
				
		# 		start_time_2 = time()
		# 		while True: ### this while loop makes the ROV move forward for 10 seconds
		# 			x_movement = [-1,"surge"]
		# 			x_movement_string = String()
		# 			x_movement_string.data = json.dumps([x_movement])
		# 			# self.autonomus_controls_publisher.publish(x_movement_string)
		# 			claw_movement2 = [1,"open_claw"] ### this is closing the claw all the time
		# 			claw_movement_string2 = String()
		# 			claw_movement_string2.data = json.dumps([claw_movement2]) 
		# 			# self.autonomus_controls_publisher.publish(claw_movement_string2)
		# 			if (time()-start_time_2) > time_in_second_1:
		# 				x_movement = [-1,"surge"]
		# 				x_movement_string = String()
		# 				x_movement_string.data = json.dumps([x_movement])
		# 				# self.autonomus_controls_publisher.publish(x_movement_string)
		# 				break
			
		# 	time_to_move()
		# 	## then start checking if the red color is dectected to know when to drop the object
		# 		## will also have a while loop involving the if statment for the box
		# 	start_time = time()
		# 	while True:
		# 		if (time()-start_time) == 14:
		# 			break
		# 		if box is not None: ### this line make the bot identify the object with a bonding box 
		# 			x1, y1, x2, y2 = box
		# 			cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 5) ## this line rap the object with a bounding box of color yellow with depth 5
		# 			## this line should drop the object
		# 			claw_movement = [1,"close_claw"] ### this is opening the claw 
		# 			claw_movement_string = String()
		# 			claw_movement_string.data = json.dumps([claw_movement])
		# 			# self.autonomus_controls_publisher.publish(claw_movement_string)
		# 			# then break
		# 			break
		# 		else:
		# 			x_movement = [-1,"surge"]
		# 			x_movement_string = String()
		# 			x_movement_string.data = json.dumps([x_movement])
		# 			# self.autonomus_controls_publisher.publish(x_movement_string)



    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        self.enter_autonomus_mode(goal_handle)

        result = AutoMode.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif goal_handle.is_active:
            goal_handle.succeed()

            # Populate result message
            result.success = True

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = AutonomusBrainCoralTransplant()

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()