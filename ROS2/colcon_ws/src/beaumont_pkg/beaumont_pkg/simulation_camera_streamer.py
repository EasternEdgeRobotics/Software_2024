import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import socket
import numpy as np
from PIL import Image as PIL_Image
import cv2 
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

camera_captures = {0: None, 1: None, 2: None, 3: None}

#ip_address = socket.gethostbyname(socket.gethostname()) #Get the ip address

ip_address = "localhost"

class SimulationCameraStreamer(Node):
	
	def __init__(self):
		super().__init__('simulation_camera_streamer')

		#Create a subscriber to listen to each camera on the bot
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera0/image_raw', self.simulation_camera_0_listener_callback, 10)
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera1/image_raw', self.simulation_camera_1_listener_callback, 10)
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera2/image_raw', self.simulation_camera_2_listener_callback, 10)
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera3/image_raw', self.simulation_camera_3_listener_callback, 10)
		self.bridge = CvBridge()

	def convert_to_cv_image(self, msg, camera_number):
		'''Converts ROS image into OpenCV image then stores it in a global variable'''
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
			if camera_number == 1 or camera_number == 2:
				cv_image = cv2.flip(cv_image,0)
				cv_image = cv2.flip(cv_image,1)
		except CvBridgeError as e:
			print(e)

		hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
		
		lower_range_red = np.array([0, 250, 0]) #Need to tune this to JUST detect the red
		upper_range_red = np.array([0, 255, 0])

		mask = cv2.inRange(hsvImage,lower_range_red ,upper_range_red )

		convert_to_pillow = PIL_Image.fromarray(mask)
		
		box = convert_to_pillow.getbbox()

		if box is not None: ### this line make the bot identify the object with a bonding box 
			x1, y1, x2, y2 = box
			cv_image = cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 5) ## this line rap the object with a bounding box of color yellow with depth 5

		global camera_captures
		camera_captures[camera_number] = cv_image
		
	def simulation_camera_0_listener_callback(self, msg):
		self.convert_to_cv_image(msg,0)

	def simulation_camera_1_listener_callback(self, msg):
		self.convert_to_cv_image(msg,1)
	
	def simulation_camera_2_listener_callback(self, msg):
		self.convert_to_cv_image(msg,2)

	def simulation_camera_3_listener_callback(self, msg):
		self.convert_to_cv_image(msg,3)
		

		
        

class CamHandler(BaseHTTPRequestHandler):
	def do_GET(self):

		#Each server will needs to determine which element from the camera_captures global dictionary to use for camera footage
		self.port_number = self.server.server_address[1] #Will either return 8880, 8881, 8882, or 8883
		self.camera_number = self.server.server_address[1] - 8880 #Will either return 0, 1, 2, or 3

		if self.path.endswith('.mjpg'):
			self.send_response(200)
			self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
			self.end_headers()
			while True:
				try:
					imgRGB=cv2.cvtColor(camera_captures[self.camera_number],cv2.COLOR_BGR2RGB)
					ret, jpg = cv2.imencode('.jpg', imgRGB)
					self.wfile.write("--jpgboundary".encode("utf-8"))
					self.send_header('Content-type','image/jpeg')
					self.send_header('Content-length',str(jpg.size))
					self.end_headers()
					self.wfile.write(jpg.tostring())
				except KeyboardInterrupt:
					break
			return
		if self.path.endswith('.html'):
			self.send_response(200)
			self.send_header('Content-type','text/html')
			self.end_headers()
			self.wfile.write('<html><head></head><body>')
			self.wfile.write(f'<img src="http://{ip_address}:{self.port_number}/cam.mjpg"/>')
			self.wfile.write('</body></html>')
			return

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	"""Handle requests in a separate thread."""

def runserver(port):
	'''Runs a server on the specified ip address and port'''
	try:
		server = ThreadedHTTPServer((ip_address, port), CamHandler)
		server.serve_forever()
	except KeyboardInterrupt:
		server.socket.close()

def main(args=None):
	rclpy.init(args=args)

	simulation_camera_streamer = SimulationCameraStreamer()

	#Create a server to publish each camera's footage
	camera_0_server = threading.Thread(target=runserver, args=(8880,), daemon=True)
	camera_1_server = threading.Thread(target=runserver, args=(8881,), daemon=True)
	camera_2_server = threading.Thread(target=runserver, args=(8882,), daemon=True)
	camera_3_server = threading.Thread(target=runserver, args=(8883,), daemon=True)
	
	#Start all servers
	camera_0_server.start()
	camera_1_server.start()
	camera_2_server.start()
	camera_3_server.start()

	rclpy.spin(simulation_camera_streamer)
	
		
	simulation_camera_streamer.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
    main()
