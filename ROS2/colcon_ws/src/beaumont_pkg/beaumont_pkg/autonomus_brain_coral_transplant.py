# Imports needed to initiate a complex ROS2 action node
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
import threading

# Import the custom ROS2 interfaces (messages) used by EER
from eer_messages.action import AutoMode 
from eer_messages.msg import ThrusterMultipliers, PilotInput
from eer_messages.srv import Config

# Import libraries for MJPEG stream viewing and colour filtering
import cv2 
from PIL import Image as PIL_Image
import numpy as np
import urllib.request

# Other libraries used
from time import time, sleep
import json

# Out of the 4 cameras on Beaumont, the brain coral transplant action node will use the downward-looking camera
BOTTOM_CAMERA_INDEX = 2 

# The required heave time after picking up the brain coral may differ between simulation and real life
HEAVE_TIME_SIMULATION = 6 
HEAVE_TIME = 5

class AutonomusBrainCoralTransplant(Node):
    """
    Action node used for the autonomus brain coral transplant task.

    This node recieves a goal from an action server to deliver the brain coral to the desired location. It achieves 
    the goal using colour filtering and notifies the pilot of the progress by publishing regular feedback.

    At all times, the pilot has the ability to cancel the goal and take over manually.
    """

    def __init__(self):
        super().__init__('autonomus_brain_coral_transplant')

        # Initiate the ROS2 action server
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            AutoMode, # Uses the custom EER AutoMode interface
            'autonomus_brain_coral_transplant', # Topic on which the action server and client communicate 
            execute_callback=self.execute_callback, 
            goal_callback=self.goal_callback, 
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup()) # Allows for the goal to be cancelled while being executed by making the action server multi-threaded
        
        # The following publishers publish inputs to which both the simulation and real ROV respond 
        self.pilot_publisher = self.create_publisher(PilotInput, "controller_input", 1)
        self.copilot_publisher = self.create_publisher(ThrusterMultipliers, "thruster_multipliers", 1)

        # Client to fetch the bottom camera MJPEG stream url saved the database
        self.camera_urls_client = self.create_client(Config, 'camera_urls')

        self.fetch_tooling_camera_stream_url()
    
    def fetch_tooling_camera_stream_url(self):
        
        while True:
            camera_url_config_request = Config.Request()

            # State == 1 means GET urls from database
            camera_url_config_request.state = 1 

            # This field is not used for fetch requests
            camera_url_config_request.data = ""

            # Ensure the database server is up before continuing 
            self.camera_urls_client.wait_for_service()
            
            future = self.camera_urls_client.call_async(camera_url_config_request)
            
            rclpy.spin_until_future_complete(self,future) # Wait for result

            # If the length of the MJPEG stream urls list is 0, the client should loop until the URLS are assigned (done in the topsides browser-based graphical user interface) 
            if len(json.loads(future.result().result)) > 0:
                self.bottom_camera_stream_url = json.loads(future.result().result)[BOTTOM_CAMERA_INDEX]
                self.get_logger().info(f"Autonomus Brain Coral Transplant Node Fetched {self.bottom_camera_stream_url}")
                break
            else:
                self.get_logger().info(f"Cannot get bottom camera MJPEG stream url, will try again in 1 second")

            sleep(1)

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
        """
        Accept or reject a client request to cancel an action.
        Note that cancel requests are handeled explicitly in the enter_autonomus_mode method.
        """
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        self.enter_autonomus_mode(goal_handle)

        # If execution gets here, it means that the action is either done or cancelled
        result = AutoMode.Result()

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
        elif goal_handle.is_active:
            goal_handle.succeed()

            # Populate result message
            result.success = True

        return result
    # Since pilot will have no control, this open claw command will not timeout
    def enter_autonomus_mode(self, goal_handle):
         
        # Initialize the feedback message 
        feedback_msg = AutoMode.Feedback()

        # Set Beaumont to full power
        full_power_multipliers = ThrusterMultipliers()
        full_power_multipliers.power = 100
        full_power_multipliers.surge = 100
        full_power_multipliers.sway = 100
        full_power_multipliers.heave = 100
        full_power_multipliers.pitch = 100
        full_power_multipliers.yaw = 100
        self.copilot_publisher.publish(full_power_multipliers)

        # Open the MJPEG camera stream
        stream = urllib.request.urlopen(self.bottom_camera_stream_url)
        bytes = b''

        # Predefine variables before entering loop
        heave_stage = True
        time_since_action_start = time()

        # Action loop
        while True:
            
            current_time = time()

            # The following boolean will be set to True if the region is found
            brain_coral_area_found = False

            # Exit action loop as soon as pilot cancels the goal
            # Can be re-entered later
            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                self.get_logger().info('Goal aborted or canceled')
                break

            # Initialize the controller_input message 
            controller_input = PilotInput()

            # Convert the MJPEG stream into a format recognizable by OpenCV
            bytes += stream.read(1024)

            # Check if feed is valid
            a = bytes.find(b'\xff\xd8')
            b = bytes.find(b'\xff\xd9')

            if a != -1 and b != -1:

                # Grab the JPG encoded binary string
                jpg = bytes[a:b+2]
                bytes = bytes[b+2:]
                
                # Convert to an rgb (red, green, blue) image
                rgb_image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

                # Use the rgb image to convert to an hsv (hue, shade, value) image
                hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)

                # Apply a mask on the image based on the lower and upper boards of the brain coral area colour.
                # This is passed in by the action client when requesting the goal. 
                mask = cv2.inRange(hsv_image, np.array(goal_handle.request.lower_hsv_bound), np.array(goal_handle.request.upper_hsv_bound))

                # Grab the contours of the filtered region
                contours = self.get_contours(mask)

                # Filter out all red regions for the one with the biggest area
                biggest_contour = np.ndarray(shape= (1,1,1))
                biggest_contour_area = 0
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > biggest_contour_area:
                        biggest_contour_area = area
                        biggest_contour = contour

                # If there is one decently sized contour, proceed to centered atop it. 
                # This is currently defined as taking up more than 8% of the image
                if (len(contours) > 0) and (biggest_contour_area > mask.shape[0] * mask.shape[1] *0.08): 
                    ((x,y),radius) = cv2.minEnclosingCircle(biggest_contour)
                    brain_coral_area_center_location_x, brain_coral_area_center_location_y = self.get_contour_center(biggest_contour)
                    brain_coral_area_found = True
                else:
                    brain_coral_area_center_location_x, brain_coral_area_center_location_y = 0, 0
                    brain_coral_area_found = False
                    
                    
                # The following algoritm depends on whether or not the brain coral area was found
                if brain_coral_area_found:
                    
                    heave_stage = False
                        
                    controller_input = PilotInput()

                    feedback_msg.status = "Found brain coral area: "

                    # Attempt to center atop region such that the brain coral held by the claw will hover atop brain coral region 
                    desired_x_location = mask.shape[0]*0.5 
                    desired_y_location = mask.shape[1]*0.2 

                    aligned_x = False
                    aligned_y = False

                    # If the brain coral region is not within 5% of the desired area, continue centering on it
                    if abs(brain_coral_area_center_location_x - desired_x_location) > mask.shape[0] * 0.05: 
                        aligned_x = False
                        if brain_coral_area_center_location_x < desired_x_location:
                            controller_input.sway = -10
                            feedback_msg.status += "Swaying left "
                        else:
                            controller_input.sway = 10
                            feedback_msg.status += "Swaying right "
                    else: 
                        controller_input.sway = 0
                        aligned_x = True

                    # Also center in the y
                    if abs(brain_coral_area_center_location_y - desired_y_location) > mask.shape[1] * 0.05: 
                        aligned_y = False
                        if brain_coral_area_center_location_y < desired_y_location:
                            controller_input.surge = 10
                            feedback_msg.status += "Surging forward"
                        else:
                            controller_input.surge = -10
                            feedback_msg.status += "Surging back"
                    else:
                        controller_input.surge = 0
                        aligned_y = True

                    # Once aligned, open the claw to drop the object
                    if aligned_x and aligned_y:

                        controller_input.open_claw  

                        self.pilot_publisher.publish(controller_input)

                        feedback_msg.status += "Opening Claw"

                        goal_handle.publish_feedback(feedback_msg) 

                        break 

                    else:

                        # Ensure brain coral is being gripped until it is time to let go 
                        controller_input.close_claw

                    self.pilot_publisher.publish(controller_input)

                    goal_handle.publish_feedback(feedback_msg) 

                else: 
                    
                    # Initially, Beaumont will heave after the pilot initates autonomus mode
                    if heave_stage:

                        controller_input = PilotInput()
                        controller_input.heave = 100
                        controller_input.close_claw = True
                        self.pilot_publisher.publish(controller_input)

                        feedback_msg.status = "Heaving up"
                        goal_handle.publish_feedback(feedback_msg)

                        if current_time - time_since_action_start > (HEAVE_TIME_SIMULATION if goal_handle.request.is_for_sim else HEAVE_TIME):
                            heave_stage = False

                    # Surge forward until the brain coral region is detected. Pilot will be facing the coral reef when autonomus mode is initalized
                    else:

                        controller_input = PilotInput()
                        controller_input.surge = 100
                        controller_input.close_claw = True
                        controller_input.heave = 0
                        self.pilot_publisher.publish(controller_input)

                        feedback_msg.status = "Surging forward"
                        goal_handle.publish_feedback(feedback_msg)


        # Action loop is exited. Ensure to switch over control to pilot by resetting the multipliers to their inital value and publishing empty pilot input
        controller_input = PilotInput()

        restored_power_multipliers = ThrusterMultipliers()
        restored_power_multipliers.power = goal_handle.starting_power
        restored_power_multipliers.surge = goal_handle.starting_surge
        restored_power_multipliers.sway = goal_handle.starting_sway
        restored_power_multipliers.heave = goal_handle.starting_heave
        restored_power_multipliers.pitch = goal_handle.starting_pitch
        restored_power_multipliers.yaw = goal_handle.starting_yaw


        feedback_msg.status = "Giving control back to the pilot"

        goal_handle.publish_feedback(feedback_msg)
        self.copilot_publisher.publish(restored_power_multipliers)
        self.pilot_publisher.publish(controller_input)
    
    def get_contours(self, mask):
        """
        Given a black and white image where white represents pixels that have met a threshold, this function
        uses OpenCV's findContours to locate the contours or 'edges' of the white areas and it returns these OpenCV
        contour objects in a list.
        """
        contours, hirearchy = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        return contours
    
    def get_contour_center(self, contour):
        """
        Given a contour, this function uses the OpenCV moments function to
        find the center of area of that contour.
        """
        M = cv2.moments(contour)
        cx = -1
        cy = -1
        if (M['m00'] != 0):
            cx= round(M['m10']/M['m00'])
            cy= round(M['m01']/M['m00'])
        return cx, cy



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