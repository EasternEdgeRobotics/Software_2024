import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from PIL import Image as PIL_Image
import cv2 
from std_msgs.msg import String
from time import time 
import json
import threading


#ip_address = socket.gethostbyname(socket.gethostname()) #Get the ip address

ip_address = "localhost"

class SimulationAutonomusMovement(Node):
	
	def __init__(self):
		super().__init__('simulation_camera_subscriber')

		#Create a subscriber to listen to each camera on the bot
		#self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera0/image_raw', self.simulation_camera_0_listener_callback, 10)
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera1/image_raw', self.simulation_camera_1_listener_callback, 10)
		self.autonomus_controls_publisher = self.create_publisher(String, "controller_input", 10)
		self.pilot_listener = self.create_subscription(String, 'controller_input', self.enter_autonomus_mode, 10)
		self.lock = threading.Lock()
	
		self.bridge = CvBridge()

	def convert_to_cv_image(self, msg, camera_number):
		
		'''Converts ROS image into OpenCV image then stores it in a global variable'''
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
		except CvBridgeError as e:
			print(e)

		hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		lower_range_red = np.array([100, 185, 40]) #Need to tune this to JUST detect the red
		upper_range_red = np.array([180, 255, 255])

		mask = cv2.inRange(hsvImage,lower_range_red ,upper_range_red )

		convert_to_pillow = PIL_Image.fromarray(mask)
		global box
		box = convert_to_pillow.getbbox()

	def enter_autonomus_mode(self, controller_input):
		
		'''Converts ROS image into OpenCV image then stores it in a global variable'''
		controller_inputs = json.loads(controller_input.data)
		for controller_input in controller_inputs:
			if controller_input[1] != "enter_auto_mode":
				return
			time_in_second_1 = 14 
			start_time = time()
			#the function for movement 
			def time_to_move(time):
				while True:
					x_movement = [0.5,"surge"]
					x_movement_string = String()
					x_movement_string.data = json.dumps(x_movement)
					self.autonomus_controls_publisher.publish(x_movement_string)
					if time()-start_time > time_in_second_1:
						x_movement = [0,"surge"]
						x_movement_string = String()
						x_movement_string.data = json.dumps(x_movement)
						self.autonomus_controls_publisher.publish(x_movement_string)
						break
			# # move for 15 seconds part (using a while loop)
			time_to_move(time_in_second_1)
			## then start checking if the red color is dectected to know when to drop the object
				## will also have a while loop involving the if statment for the box
			time_in_second_2 = 2
			while True:
				if box is not None: ### this line make the bot identify the object with a bonding box 
					x1, y1, x2, y2 = box
					cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 5) ## this line rap the object with a bounding box of color yellow with depth 5
					## this line should drop the object
					claw_movement = [1,"open_claw"]
					claw_movement_string = String()
					claw_movement_string.data = json.dumps(claw_movement)
					self.autonomus_controls_publisher.publish(claw_movement_string)
					# then break
					break
				else:
					time_to_move(time_in_second_2)
		

		
	# def simulation_camera_0_listener_callback(self, msg):
	# 	self.convert_to_cv_image(msg,0)

	def simulation_camera_1_listener_callback(self, msg):
		self.convert_to_cv_image(msg,1)
	
		




def main(args=None):
	rclpy.init(args=args)

	simulation_autonomous_movement = SimulationAutonomusMovement()

	rclpy.spin(simulation_autonomous_movement)
	
		
	simulation_autonomous_movement.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
    main()
