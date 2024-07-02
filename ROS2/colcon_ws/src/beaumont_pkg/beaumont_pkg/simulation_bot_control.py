import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from eer_messages.msg import ThrusterMultipliers, PilotInput
from eer_messages.action import AutoMode 
from eer_messages.srv import HSVColours

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Wrench, Twist
from std_msgs.msg import String

from math import sqrt, cos, pi

SIMULAITON_PERCISION = 0.1 # 10 Hz

MAX_VALUE = {
    "for-port-top": 0,
    "for-star-top": 0,
    "aft-port-top": 0,
    "aft-star-top": 0,
    "for-port-bot": 0,
    "for-star-bot": 0,
    "aft-port-bot": 0,
    "aft-star-bot": 0
}

class SimulationBotControl(Node):

    def __init__(self):
        super().__init__('simulation_bot_control')

        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 10)
        self.pilot_listener = self.create_subscription(PilotInput, 'controller_input', self.pilot_listener_callback, 1)
        self.autonomous_mode_publisher = self.create_publisher(String, "/autonomous_mode_status", 10)

        # For xy movement, the model is using the Gazebo Planar Move Plugin, which allows for movement relative to itself rather than the world
        # It also uses this plugin for yaw 
        self.simulation_velocity_publisher_xy_yaw = self.create_publisher(Twist, "/demo/simulation_model_linear_velocity_xy", 10) 

        # For movement in the z axis, the model uses the Gazebo Force Plugin
        self.simulation_velocity_publisher_z = self.create_publisher(Wrench, "/demo/link/simulation_model_linear_velocity_z", 10) 

        # For Pitch, the model also uses the Gazebo Force Plugin, applied at a specifc spot in order to cause pitch
        self.simulation_velocity_publisher_pitch = self.create_publisher(Wrench, "/demo/link/force_demo_pitch", 10)

        # For claw movement, the Gazebo Planar Move Plugin is used
        self.left_claw_publisher = self.create_publisher(Twist, "/demo/left_claw", 10) 
        self.right_claw_publisher = self.create_publisher(Twist, "/demo/right_claw", 10)

        # Autonomus movement node
        self._action_client = ActionClient(self, AutoMode, 'autonomus_brain_coral_transplant')
        self.autonomus_mode_active = False

        # Client to fetch the hsv colour values camera saved on the task_manager database
        self.brain_coral_hsv_colour_bounds_client = self.create_client(HSVColours, 'set_color', callback_group=ReentrantCallbackGroup())

        # The default bounds filter for the colour RED
        self.brain_coral_hsv_colour_bounds = {
            "upper_hsv":[10,255,140],
            "lower_hsv":[0,242,60]
        }

        # Debugger publisher
        # from std_msgs.msg import String
        # self.debugger = self.create_publisher(String, 'debugger', 10) 

        self.power_multiplier = 0
        self.surge_multiplier = 0
        self.sway_multiplier = 0
        self.heave_multiplier = 0
        self.pitch_multiplier = 0
        self.yaw_multiplier = 0


        # The differnce between the planar move plugin and the force plugin is that the planar move plugin acts relative to the bot, while the force plugin acts relative to the world
        
        self.net_force_array = {"surge":0,
                                       "sway":0,
                                       "heave":0,
                                       "pitch":0,
                                       "yaw":0}

        self.velocity_array = {"surge":0,
                                    "sway":0,
                                    "heave":0,
                                    "pitch":0,
                                    "yaw":0} 

        self.surface_area_for_drag = {"surge":0.1813,
                                        "sway":0.2035,
                                        "heave":0.2695,
                                        "pitch":0.2,
                                        "yaw":0.2} 
        

        self.bot_mass = 23 #23kg

        # The max thruster force is estimated to be 110 Newtons
        self.max_thruster_force = 110 

        # Thruster distance from center of mass is assumed to be around 0.35m
        self.thruster_distance_from_COM = 0.35

        self.fluid_mass_density = 1000 # 1000kg/m^3
        
        # This value has been tuned. The higher it is, the faster the bot reaches terminal velocity
        self.drag_coefficient = 0.1 

        # Assuming the terminal velocity of the bot in water is 0.25 m/s... the force adjustment factor should be 150
        self.gazebo_simulation_velocity_z_adjustment_factor = 150 

        self.gazebo_simulation_velocity_xy_adjustment_factor = 0.04

        self.gazebo_simulation_velocity_pitch_adjustment_factor = 30

        self.gazebo_simulation_velocity_yaw_adjustment_factor = 0.05

        self.bot_yaw_to_claw_yaw_factor = 90

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

    def copilot_listener_callback(self, msg):
        self.power_multiplier = float(msg.power/100)
        self.surge_multiplier = float(msg.surge/100)
        self.sway_multiplier = float(msg.sway/100)
        self.heave_multiplier = float(msg.heave/100)
        self.pitch_multiplier = float(msg.pitch/100)
        self.yaw_multiplier = float(msg.yaw/100)
    

    def pilot_listener_callback(self, msg):  
        '''Called when new controller input from pilot is recieved'''
        
        if not self.autonomus_mode_active or (msg.is_autonomous and self.autonomus_mode_active):
            thruster_forces = self.simulation_rov_math(msg)
            
            for direction in thruster_forces: # Apply drag force 

                # The pitch and yaw the velocity below is actually angular velocity, the the net force is net torque.
                # Drag is still applied in the same way as it helps maintain smooth bot movement 

                self.velocity_array[direction] = self.velocity_array[direction]+((SIMULAITON_PERCISION*self.net_force_array[direction])/self.bot_mass) 

                drag_force = 0.5*self.fluid_mass_density*((self.velocity_array[direction])**2)*self.drag_coefficient*self.surface_area_for_drag[direction] * (-1 if self.velocity_array[direction]>0 else 1)

                self.net_force_array[direction] = thruster_forces[direction] + drag_force


            velocity_xy_yaw = Twist()
            velocity_z = Wrench()
            velocity_pitch = Wrench()

            velocity_xy_yaw.linear.x = float((-1)*self.velocity_array["sway"] * self.gazebo_simulation_velocity_xy_adjustment_factor)  

            velocity_xy_yaw.linear.y = float((-1)*self.velocity_array["surge"] * self.gazebo_simulation_velocity_xy_adjustment_factor)

            velocity_z.force.z = float(self.velocity_array["heave"] * self.gazebo_simulation_velocity_z_adjustment_factor)

            velocity_xy_yaw.angular.z = float((-1)*self.velocity_array["yaw"] * self.gazebo_simulation_velocity_yaw_adjustment_factor)

            velocity_pitch.force.z = float(self.velocity_array["pitch"] * self.gazebo_simulation_velocity_pitch_adjustment_factor)

            self.simulation_velocity_publisher_xy_yaw.publish(velocity_xy_yaw)
            self.simulation_velocity_publisher_z.publish(velocity_z)
            self.simulation_velocity_publisher_pitch.publish(velocity_pitch)

            # from std_msgs.msg import String
            # velocity = String()
            # velocity.data = str(self.velocity_array)
            # self.debugger.publish(velocity)

            self.simulation_tooling(msg)

        if msg.enter_auto_mode:
            if not self.autonomus_mode_active:
                self.autonomus_mode_active = True
                self.send_autonomus_mode_goal()
            else:
                # self._action_client.wait_for_server()

                future = self.goal_handle.cancel_goal_async()
                future.add_done_callback(self.cancel_done)

    def simulation_tooling(self, controller_inputs):
        """Controls the movement of the simulation claw. """

        if not (controller_inputs.open_claw ^ controller_inputs.close_claw):

            left_claw_velocity = Twist()
            right_claw_velocity = Twist()

            left_claw_velocity.angular.z = float(0)
            right_claw_velocity.angular.z = float(0)

            self.left_claw_publisher.publish(left_claw_velocity)
            self.right_claw_publisher.publish(right_claw_velocity)
        
        elif controller_inputs.open_claw:

            left_claw_velocity = Twist()
            right_claw_velocity = Twist()

            left_claw_velocity.angular.z = 0.5
            right_claw_velocity.angular.z = -0.5

            self.left_claw_publisher.publish(left_claw_velocity)
            self.right_claw_publisher.publish(right_claw_velocity)

        elif controller_inputs.close_claw:

            left_claw_velocity = Twist()
            right_claw_velocity = Twist()

            left_claw_velocity.angular.z = -0.5
            right_claw_velocity.angular.z = 0.5

            self.left_claw_publisher.publish(left_claw_velocity)
            self.right_claw_publisher.publish(right_claw_velocity)

    def send_autonomus_mode_goal(self):

        self.fetch_brain_coral_hsv_colour_bounds()
        goal_msg = AutoMode.Goal()
        goal_msg.is_for_sim = True

        # HSV (hue,shade,value) bounds for filtering brain coral area
        goal_msg.lower_hsv_bound = self.brain_coral_hsv_colour_bounds["lower_hsv"]
        goal_msg.upper_hsv_bound = self.brain_coral_hsv_colour_bounds["upper_hsv"]

        goal_msg.starting_power = int(self.power_multiplier * 100)
        goal_msg.starting_surge = int(self.surge_multiplier * 100)
        goal_msg.starting_sway = int(self.sway_multiplier * 100)
        goal_msg.starting_heave = int(self.heave_multiplier * 100)
        goal_msg.starting_pitch = int(self.pitch_multiplier * 100)
        goal_msg.starting_yaw = int(self.yaw_multiplier * 100) 

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result
        autonomous_mode_status = String()
        autonomous_mode_status.data = "Autonomous Mode off, {0}".format("Mission Success" if result.success else "Mission Failed")
        self.autonomous_mode_publisher.publish(autonomous_mode_status)
        self.autonomus_mode_active = False

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        autonomous_mode_status = String()
        autonomous_mode_status.data = "Autonomous Mode on, {0}".format(feedback.status)
        self.autonomous_mode_publisher.publish(autonomous_mode_status)
    
    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            autonomous_mode_status = String()
            autonomous_mode_status.data = 'Auto mode successfully canceled'
            self.autonomous_mode_publisher.publish(autonomous_mode_status)
        else:
            autonomous_mode_status = String()
            autonomous_mode_status.data = 'Auto mode failed to cancel'
            self.autonomous_mode_publisher.publish(autonomous_mode_status)

    def fetch_brain_coral_hsv_colour_bounds(self):

        hsv_colour_bounds_request = HSVColours.Request()

        # load_to_database = False indicates loading FROM database
        hsv_colour_bounds_request.load_to_database = False 

        # Ensure the database server is up before continuing 
        self.brain_coral_hsv_colour_bounds_client.wait_for_service()
            
        future = self.brain_coral_hsv_colour_bounds_client.call(hsv_colour_bounds_request)
        
        if future.success: # Means that HSV colours were stored in the database at this time
            self.brain_coral_hsv_colour_bounds["upper_hsv"] = future.upper_hsv
            self.brain_coral_hsv_colour_bounds["lower_hsv"] = future.lower_hsv
        else:
            self.get_logger().info("No HSV colour bounds stored in database. Will keep using default.")



    def simulation_rov_math(self, controller_inputs):
        """Builds upon the real Rov Math Method in i2c_master.py and calculates net forces on Bot."""

        thruster_values = {}

        surge = controller_inputs.surge * self.power_multiplier * self.surge_multiplier * 0.01
        sway = controller_inputs.sway * self.power_multiplier * self.sway_multiplier * 0.01
        yaw = controller_inputs.yaw * self.power_multiplier * self.yaw_multiplier * 0.01

        if controller_inputs.heave_up or controller_inputs.heave_down:
            controller_inputs.heave = (100 if controller_inputs.heave_up else 0) + (-100 if controller_inputs.heave_down else 0)
            
        heave = controller_inputs.heave * self.power_multiplier * self.heave_multiplier * 0.01   

        if controller_inputs.pitch_up or controller_inputs.pitch_down:
            controller_inputs.pitch = (100 if controller_inputs.pitch_up else 0) + (-100 if controller_inputs.pitch_down else 0)
        
        pitch = controller_inputs.pitch * self.power_multiplier * self.pitch_multiplier * 0.01  

        sum_of_magnitudes_of_pilot_input = abs(surge) + abs(sway) + abs(heave) + abs(pitch) + abs(yaw)

        # These adjustment factors determine how much to decrease power in each thruster due to multipliers.
        # The first mutliplication term determines the total % to remove of the inital thruster direction,
        # The second term scales that thruster direction down based on what it makes up of the sum of pilot input, and also applies a sign
        surge_adjustment = (1 - (self.power_multiplier * self.surge_multiplier * abs(controller_inputs.surge) * 0.01)) * (surge/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0)  
        sway_adjustment = (1 - (self.power_multiplier * self.sway_multiplier * abs(controller_inputs.sway) * 0.01)) * (sway/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0) 
        heave_adjustment = (1 - (self.power_multiplier * self.heave_multiplier * abs(controller_inputs.heave) * 0.01)) * (heave/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0) 
        pitch_adjustment = (1 - (self.power_multiplier * self.pitch_multiplier * abs(controller_inputs.pitch) * 0.01)) * (pitch/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0) 
        yaw_adjustment = (1 - (self.power_multiplier * self.yaw_multiplier * abs(controller_inputs.yaw) * 0.01)) * (yaw/sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0) 

        thruster_scaling_coefficient = 1 / sum_of_magnitudes_of_pilot_input if sum_of_magnitudes_of_pilot_input else 0

        # Calculations below are based on thruster positions:

        # First term:
        # The net pilot input based on how it applies to the specifc thruster (some are reveresed) is calculated.
        # Then, this is scaled down by the thruster scaling coefficent such that the max absolute value it can attain is 1.
        # This will properly activate distribute load and direction among thrusters such that the desired movement is reached,
        # but the first term cancels out the effect of the thruster multipliers 

        # Directional adjustment factors:
        # These adjustment factors will never increase the power going to a single thruster.
        # They will only serve to proportionally decrease it in order to reduce power in a certain direction. 
         
        thruster_values["for-port-bot"] = (((-surge)+(sway)+(heave)+(pitch)+(yaw)) * thruster_scaling_coefficient) + surge_adjustment - sway_adjustment - heave_adjustment - pitch_adjustment - yaw_adjustment
        thruster_values["for-star-bot"] = (((-surge)+(-sway)+(heave)+(pitch)+(-yaw)) * thruster_scaling_coefficient) + surge_adjustment + sway_adjustment - heave_adjustment - pitch_adjustment + yaw_adjustment
        thruster_values["aft-port-bot"] = (((surge)+(sway)+(heave)+ (-pitch)+(-yaw)) * thruster_scaling_coefficient) - surge_adjustment - sway_adjustment - heave_adjustment + pitch_adjustment + yaw_adjustment
        thruster_values["aft-star-bot"] = (((surge)+(-sway)+(heave)+(-pitch)+(yaw)) * thruster_scaling_coefficient) - surge_adjustment + sway_adjustment - heave_adjustment + pitch_adjustment - yaw_adjustment
        thruster_values["for-port-top"] = (((-surge)+(sway)+(-heave)+(-pitch)+(yaw)) * thruster_scaling_coefficient) + surge_adjustment - sway_adjustment + heave_adjustment + pitch_adjustment - yaw_adjustment
        thruster_values["for-star-top"] = (((-surge)+(-sway)+(-heave)+(-pitch)+(-yaw)) * thruster_scaling_coefficient) + surge_adjustment + sway_adjustment + heave_adjustment + pitch_adjustment + yaw_adjustment
        thruster_values["aft-port-top"] = (((surge)+(sway)+(-heave)+(pitch)+(-yaw)) * thruster_scaling_coefficient) - surge_adjustment - sway_adjustment + heave_adjustment - pitch_adjustment + yaw_adjustment
        thruster_values["aft-star-top"] = (((surge)+(-sway)+(-heave)+(pitch)+(yaw)) * thruster_scaling_coefficient) - surge_adjustment + sway_adjustment + heave_adjustment - pitch_adjustment - yaw_adjustment

        ####################################################################
        ############################## DEBUG ###############################
        ####################################################################

        # for thruster_position in MAX_VALUE:
        #     if MAX_VALUE[thruster_position] < thruster_values[thruster_position]:
        #         MAX_VALUE[thruster_position] = thruster_values[thruster_position]

        
        # from std_msgs.msg import String
        # msg = String()
        # msg.data = str(thruster_values)
        # self.debugger.publish(msg) 

        #################################################################################
        ############################## SIMULATION PORTION ###############################
        #################################################################################

        net_surge = self.max_thruster_force * cos(pi/3)*((-thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
        net_sway = self.max_thruster_force * cos(pi/3)*((thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (-thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))
        net_heave = self.max_thruster_force * cos(pi/3)*((thruster_values["for-port-bot"]) + (thruster_values["for-star-bot"]) + (thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (-thruster_values["aft-star-top"]))
        
        net_pitch = self.max_thruster_force * self.thruster_distance_from_COM * cos(pi/4)*((thruster_values["for-port-bot"]) + (thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (-thruster_values["aft-star-bot"]) + (-thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
        net_yaw = self.max_thruster_force * self.thruster_distance_from_COM * cos(pi/4)*((thruster_values["for-port-bot"]) + (-thruster_values["for-star-bot"]) + (-thruster_values["aft-port-bot"]) + (thruster_values["aft-star-bot"]) + (thruster_values["for-port-top"]) + (-thruster_values["for-star-top"]) + (-thruster_values["aft-port-top"]) + (thruster_values["aft-star-top"]))
    
        return {
            "surge": net_surge,
            "sway": net_sway,
            "heave":net_heave,
            "pitch": net_pitch,
            "yaw": net_yaw
        }
        
        ##################################################################################
        ##################################################################################
        ##################################################################################



def main(args=None):
    rclpy.init(args=args)

    simulation_bot_control = SimulationBotControl()

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(simulation_bot_control, executor=executor)

    rclpy.spin(simulation_bot_control)

    simulation_bot_control.destroy_node()


if __name__ == "__main__":
    main()
