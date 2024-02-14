import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn
import time

camera_captures = {0: None, 1: None, 2: None, 3: None}

class SimulationCameraSubscriber(Node):
	
	def __init__(self):
		super().__init__('simulation_camera_subscriber')
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera0/image_raw', self.simulation_camera_0_listener_callback, 10)
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera1/image_raw', self.simulation_camera_1_listener_callback, 10)
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera2/image_raw', self.simulation_camera_2_listener_callback, 10)
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera3/image_raw', self.simulation_camera_3_listener_callback, 10)
		self.bridge = CvBridge()

	def convert_to_cv_image(self, msg, camera_number):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
		except CvBridgeError as e:
			print(e)
		
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
					#time.sleep(0.05)
				except KeyboardInterrupt:
					break
			return
		if self.path.endswith('.html'):
			self.send_response(200)
			self.send_header('Content-type','text/html')
			self.end_headers()
			self.wfile.write('<html><head></head><body>')
			self.wfile.write(f'<img src="http://192.168.2.199:{self.port_number}/cam.mjpg"/>')
			self.wfile.write('</body></html>')
			return

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	"""Handle requests in a separate thread."""

def runserver(port):
	try:
		server = ThreadedHTTPServer(('192.168.2.199', port), CamHandler)
		server.serve_forever()
	except KeyboardInterrupt:
		server.socket.close()

def main(args=None):
	rclpy.init(args=args)

	simulation_data_subscriber = SimulationCameraSubscriber()

	camera_0_server = threading.Thread(target=runserver, args=(8880,), daemon=True)
	camera_1_server = threading.Thread(target=runserver, args=(8881,), daemon=True)
	camera_2_server = threading.Thread(target=runserver, args=(8882,), daemon=True)
	camera_3_server = threading.Thread(target=runserver, args=(8883,), daemon=True)
	

	camera_0_server.start()
	camera_1_server.start()
	camera_2_server.start()
	camera_3_server.start()

	rclpy.spin(simulation_data_subscriber)
	
		
	simulation_data_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
    main()
