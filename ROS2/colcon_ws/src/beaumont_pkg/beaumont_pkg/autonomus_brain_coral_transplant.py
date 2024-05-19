import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from eer_messages.srv import Config
from eer_messages.action import AutoMode 

import cv2 
import numpy as np
import urllib.request

from time import time
import json

TOOLING_CAMERA_INDEX = 2

class AutonomusBrainCoralTransplant(Node):
	
	def __init__(self):
		super().__init__('autonomus_brain_coral_transplant')
		
		self.camera_urls_client = self.create_client(Config, 'camera_urls')

		self.autonomus_brain_coral_transplant_action_server = ActionServer(self, AutoMode, 'autonomus_brain_coral_transplant', self.autonomus_brain_coral_transplant)

		self.tooling_camera_stream_url = ""
		
	def autonomus_brain_coral_transplant(self, goal_handle):

		self.tooling_camera_stream_url = json.loads(self.fetch_tooling_camera_stream_url())[TOOLING_CAMERA_INDEX]
		self.get_logger().info(f"FETCHED {self.tooling_camera_stream_url}")

		goal_handle.succeed()

		result = AutoMode.Result()
		return result
	
	def fetch_tooling_camera_stream_url(self):

		camera_url_config_request = Config.Request()
		camera_url_config_request.state = 1 # GET urls from database
		camera_url_config_request.data = "" # Not used

		future = self.camera_urls_client.call_async(camera_url_config_request)
		rclpy.spin_until_future_complete(self, future)

		return future.result().result

	def enter_autonomus_mode(self, controller_input):
		
		'''Abdulmujeeb's autonomus movement code.'''
		controller_inputs = json.loads(controller_input.data)
		for controller_input in controller_inputs:
			if controller_input[1] != "enter_auto_mode":
				return
			time_in_second_1 = 9
			time_in_second_3 = 4
			
			self.get_logger().info("In Auto Mode")

			#the function for movement 
			def time_to_move():
				start_time = time()
				while True: ### this while loop makes the ROV move up for 20 seconds
					y_movement = [-1, "heave"]
					y_movement_string = String()
					y_movement_string.data = json.dumps([y_movement]) 
					# self.autonomus_controls_publisher.publish(y_movement_string)
					claw_movement1 = [1,"open_claw"] ### this is closing the claw all the time
					claw_movement_string1 = String()
					claw_movement_string1.data = json.dumps([claw_movement1]) 
					# self.autonomus_controls_publisher.publish(claw_movement_string1)
					if (time()-start_time) > time_in_second_3:
						y_movement = [0, "heave"]
						y_movement_string = String()
						y_movement_string.data = json.dumps([y_movement]) 
						# self.autonomus_controls_publisher.publish(y_movement_string)
						break
				
				start_time_2 = time()
				while True: ### this while loop makes the ROV move forward for 10 seconds
					x_movement = [-1,"surge"]
					x_movement_string = String()
					x_movement_string.data = json.dumps([x_movement])
					# self.autonomus_controls_publisher.publish(x_movement_string)
					claw_movement2 = [1,"open_claw"] ### this is closing the claw all the time
					claw_movement_string2 = String()
					claw_movement_string2.data = json.dumps([claw_movement2]) 
					# self.autonomus_controls_publisher.publish(claw_movement_string2)
					if (time()-start_time_2) > time_in_second_1:
						x_movement = [-1,"surge"]
						x_movement_string = String()
						x_movement_string.data = json.dumps([x_movement])
						# self.autonomus_controls_publisher.publish(x_movement_string)
						break
			
			time_to_move()
			## then start checking if the red color is dectected to know when to drop the object
				## will also have a while loop involving the if statment for the box
			start_time = time()
			while True:
				if (time()-start_time) == 14:
					break
				if box is not None: ### this line make the bot identify the object with a bonding box 
					x1, y1, x2, y2 = box
					cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 5) ## this line rap the object with a bounding box of color yellow with depth 5
					## this line should drop the object
					claw_movement = [1,"close_claw"] ### this is opening the claw 
					claw_movement_string = String()
					claw_movement_string.data = json.dumps([claw_movement])
					# self.autonomus_controls_publisher.publish(claw_movement_string)
					# then break
					break
				else:
					x_movement = [-1,"surge"]
					x_movement_string = String()
					x_movement_string.data = json.dumps([x_movement])
					# self.autonomus_controls_publisher.publish(x_movement_string)

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
	
		




def main(args=None):
	rclpy.init(args=args)

	autonomus_brain_coral_transplant = AutonomusBrainCoralTransplant()

	rclpy.spin(autonomus_brain_coral_transplant)
	
		
	autonomus_brain_coral_transplant.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
    main()
