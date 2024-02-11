import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2 
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

camera_0_capture = None
camera_1_capture = None

class SimulationCameraSubscriber(Node):
	
	def __init__(self):
		super().__init__('simulation_camera_subscriber')
		self.simulation_camera_listener = self.create_subscription(Image, '/demo_cam/camera0/image_raw', self.simulation_camera_listener_callback, 10)
		self.bridge = CvBridge()
		
	def simulation_camera_listener_callback(self, msg):
		
		try:
			cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
		except CvBridgeError as e:
			print(e)
		
		global camera_0_capture
		camera_0_capture = cv_image

		
        

class CamHandler(BaseHTTPRequestHandler):
	def do_GET(self):
		if self.path.endswith('.mjpg'):
			self.send_response(200)
			self.send_header('Content-type','multipart/x-mixed-replace; boundary=--jpgboundary')
			self.end_headers()
			while True:
				try:
					imgRGB=cv2.cvtColor(camera_0_capture,cv2.COLOR_BGR2RGB)
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
			self.wfile.write('<img src="http://192.168.2.199:8888/cam.mjpg"/>')
			self.wfile.write('</body></html>')
			return

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
	"""Handle requests in a separate thread."""

def runserver():
	try:
		server = ThreadedHTTPServer(('192.168.2.199', 8888), CamHandler)
		print("Server otg")
		server.serve_forever()
	except KeyboardInterrupt:
		server.socket.close()

def main(args=None):
	rclpy.init(args=args)

	simulation_data_subscriber = SimulationCameraSubscriber()

	server = threading.Thread(target=runserver, daemon=True)
	server.start()

	rclpy.spin(simulation_data_subscriber)
	
		
	simulation_data_subscriber.destroy_node()
	rclpy.shutdown()


if __name__ == "__main__":
    main()
