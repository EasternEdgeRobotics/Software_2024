import threading

from eer_messages.action import AutoMode 
from eer_messages.msg import ThrusterMultipliers, PilotInput
from eer_messages.srv import Config

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import cv2 
from PIL import Image as PIL_Image
import numpy as np
import urllib.request

from time import time, sleep
import json

TOOLING_CAMERA_INDEX = 1

TIMEOUT = 20
HEAVE_TIME_SIMULATION = 15
HEAVE_TIME = 5

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

        self.fetch_tooling_camera_stream_url()
    
    def fetch_tooling_camera_stream_url(self):
        
        while True:
            camera_url_config_request = Config.Request()
            camera_url_config_request.state = 1 # GET urls from database
            camera_url_config_request.data = "" # Not used
            
            self.camera_urls_client.wait_for_service()
            
            future = self.camera_urls_client.call_async(camera_url_config_request)
            
            rclpy.spin_until_future_complete(self,future)

            if len(json.loads(future.result().result)) > 0:
                self.tooling_camera_stream_url = json.loads(future.result().result)[TOOLING_CAMERA_INDEX]
                self.get_logger().info(f"Autonomus Brain Coral Transplant Node Fetched {self.tooling_camera_stream_url}")
                break
            else:
                self.get_logger().info(f"Cannot get tooling camera url")

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
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT
    
    def enter_autonomus_mode(self, goal_handle):
        # Append the seeds for the Fibonacci sequence
        feedback_msg = AutoMode.Feedback()

        full_power_multipliers = ThrusterMultipliers()
        full_power_multipliers.power = 100
        full_power_multipliers.surge = 100
        full_power_multipliers.sway = 100
        full_power_multipliers.heave = 100
        full_power_multipliers.pitch = 100
        full_power_multipliers.yaw = 100
        self.copilot_publisher.publish(full_power_multipliers)

        stream = urllib.request.urlopen(self.tooling_camera_stream_url)
        bytes = b''

        heave_stage = True
        loop_10Hz_start_time = time()
        time_since_action_start = time()

        # Start executing the action
        while True:
            
            current_time = time()

            if not goal_handle.is_active or goal_handle.is_cancel_requested:
                self.get_logger().info('Goal aborted or canceled')
                break
            
            publish_input = False

            if current_time-loop_10Hz_start_time > 0.01:
                publish_input = True
                loop_10Hz_start_time = current_time

            controller_input = PilotInput()

            bytes += stream.read(1024)
            a = bytes.find(b'\xff\xd8')
            b = bytes.find(b'\xff\xd9')
            if a != -1 and b != -1:
                jpg = bytes[a:b+2]
                bytes = bytes[b+2:]
                rgb_image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv_image, np.array(goal_handle.request.lower_hsv_bound), np.array(goal_handle.request.upper_hsv_bound))
                PIL_image = PIL_Image.fromarray(mask)

                box = PIL_image.getbbox()

                if box is not None: ### this line make the bot identify the object with a bonding box 
                    x1, y1, x2, y2 = box
                else:
                    x1, y1, x2, y2 = 0, 0, 0, 0
                    
                if ((y2-y1) * (x2-x1)) > mask.shape[0] * mask.shape[1] *0.08:

                    heave_stage = False
                        
                    controller_input = PilotInput()

                    feedback_msg.status = "Found brain coral area: "

                    center_point_x = int((x2-x1)/2)
                    center_point_y = int((y2-y1)/2)

                    desired_x_location = mask.shape[0]*0.5 # i.e. under claw horizontally
                    desired_y_location = mask.shape[1]*0.3 # i.e. under claw vertically

                    aligned_x = False
                    aligned_y = False

                    if abs(center_point_x - desired_x_location) > mask.shape[0] * 0.05: # Needs alignment in x
                        aligned_x = False
                        if center_point_x > desired_x_location:
                            controller_input.sway = -10
                            feedback_msg.status += "Swaying left "
                        else:
                            controller_input.sway = 10
                            feedback_msg.status += "Swaying right "
                    else: 
                        aligned_x = True

                    if abs(center_point_y - desired_y_location) > mask.shape[1] * 0.05: # Needs alignment in y
                        aligned_y = False
                        if center_point_y > desired_y_location:
                            controller_input.surge = 10
                            feedback_msg.status += "Surging forward"
                        else:
                            controller_input.surge = -10
                            feedback_msg.status += "Surging back"
                    else:
                        aligned_y = True

                    if aligned_x and aligned_y:

                        controller_input.open_claw # Since pilot will have no control, this open claw command will not timeout 

                        self.pilot_publisher.publish(controller_input)

                        feedback_msg.status += "Opening Claw"

                        goal_handle.publish_feedback(feedback_msg) 

                        break 

                    else:

                        controller_input.close_claw

                    self.pilot_publisher.publish(controller_input)

                    goal_handle.publish_feedback(feedback_msg) 

                else: 

                    if publish_input and heave_stage:

                        controller_input = PilotInput()
                        controller_input.heave = 100
                        controller_input.close_claw = True
                        self.pilot_publisher.publish(controller_input)

                        feedback_msg.status = "Heaving up"
                        goal_handle.publish_feedback(feedback_msg)

                        if current_time - time_since_action_start > (HEAVE_TIME_SIMULATION if goal_handle.request.is_for_sim else HEAVE_TIME):
                            heave_stage = False

                    elif publish_input:

                        controller_input = PilotInput()
                        controller_input.surge = 100
                        controller_input.close_claw = True
                        self.pilot_publisher.publish(controller_input)

                        feedback_msg.status = "Surging forward"
                        goal_handle.publish_feedback(feedback_msg)

        controller_input = PilotInput()

        restored_power_multipliers = ThrusterMultipliers()
        restored_power_multipliers.power = goal_handle.starting_power
        restored_power_multipliers.surge = goal_handle.starting_surge
        restored_power_multipliers.sway = goal_handle.starting_sway
        restored_power_multipliers.heave = goal_handle.starting_heave
        restored_power_multipliers.pitch = goal_handle.starting_pitch
        restored_power_multipliers.yaw = goal_handle.starting_yaw
        self.copilot_publisher.publish(restored_power_multipliers)




        feedback_msg.status = "Resetting"
        goal_handle.publish_feedback(feedback_msg)
        self.pilot_publisher.publish(controller_input)

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