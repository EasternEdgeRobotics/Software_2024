import rclpy
from rclpy.node import Node 
from eer_messages.msg import ThrusterMultipliers
from std_msgs.msg import String
from geometry_msgs.msg import Wrench, Twist
import json
import threading
from time import time, sleep

class ThrusterDataSubscriber(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 10)
        self.pilot_listener = self.create_subscription(String, 'controller_input', self.pilot_listener_callback, 10)
        self.simulation_velocity_publisher_xy = self.create_publisher(Twist, "/demo/simulation_model_linear_velocity_xy", 10) 
        self.simulation_velocity_publisher_z = self.create_publisher(Wrench, "/demo/link/simulation_model_linear_velocity_z", 10) 
        self.simulation_angular_velocity_publishers = {"pitch":self.create_publisher(Wrench, "/demo/link/force_demo_pitch", 10),
                                                    "roll":self.create_publisher(Wrench, "/demo/link/force_demo_roll", 10),
                                                    "yaw":self.simulation_velocity_publisher_xy}

        self.lock = threading.Lock() #Used as a synchronization tool

        self.force_input_array = {"surge":0,
                                    "sway":0,
                                    "heave":0}

        self.velocity_array = {"surge":0,
                                    "sway":0,
                                    "heave":0}
        
        self.angular_velocity_array = {"pitch":0,
                                       "roll":0,
                                       "yaw":0}

        self.force_input_time_array = {"surge":0,
                                                "sway":0,
                                                "heave":0,
                                                "pitch":0,
                                                "roll":0,
                                                "yaw":0}

        self.previous_force_input_array = {"surge":0,
                                    "sway":0,
                                    "heave":0}

        self.net_force_input_time_array = {"surge":0,
                                                "sway":0,
                                                "heave":0}

        self.surface_area_for_drag = {"surge":0.1813,
                                        "sway":0.2035,
                                        "heave":0.2695}
        
        self.thruster_force_multipliers = {"power":0,
                                        "surge":0,
                                        "sway":0,
                                        "heave":0,
                                        "pitch":0,
                                        "roll":0,
                                        "yaw":0}

        self.bot_mass = 23 #23kg
        self.fluid_mass_density = 1000 #1000kg/m^3
        self.drag_coefficient = 0.025 #This value has been tuned
        self.max_thruster_force = 116
        self.gazebo_simulation_velocity_z_adjustment_factor = 150
        self.gazebo_simulation_velocity_xy_adjustment_factor = 0.05
        self.gazebo_simulation_velocity_pitch_roll_adjustment_factor = 75
        self.gazebo_simulation_velocity_yaw_adjustment_factor = 0.1
        self.surge_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("surge",) , daemon=True)
        self.sway_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("sway",) , daemon=True)
        self.heave_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("heave",) , daemon=True)

        self.surge_velocity_thread.start()
        self.sway_velocity_thread.start()
        self.heave_velocity_thread.start()

        
        #Should be 4775 to match real velocity... but this is too fast 
        #I persume the the parameters in the calculations are off or that the Python time modules is off
        #Assuming the terminal velocity of the bot in water is 0.25 m/s... the force adjustment factor should be 150

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

    def copilot_listener_callback(self, msg):
        self.get_logger().info(f"{msg} recieved in ThrusterControl")
        self.lock.acquire()
        self.thruster_force_multipliers["power"] = msg.power/100
        self.thruster_force_multipliers["surge"] = msg.surge/100
        self.thruster_force_multipliers["sway"] = msg.sway/100
        self.thruster_force_multipliers["heave"] = msg.heave/100
        self.thruster_force_multipliers["pitch"] = msg.pitch/100
        self.thruster_force_multipliers["roll"] = msg.roll/100
        self.thruster_force_multipliers["yaw"] = msg.yaw/100
        self.lock.release()

    def reset_bot_movement(self):
        angular_velocity = Wrench()
        velocity = Twist()
        self.lock.acquire()
        self.simulation_angular_velocity_publishers["pitch"].publish(angular_velocity)
        self.simulation_angular_velocity_publishers["roll"].publish(angular_velocity)
        self.simulation_angular_velocity_publishers["yaw"].publish(velocity) # Yaw uses the same topic as xy linear velocity, and therefore uses the Twist object
        self.simulation_velocity_publisher_xy.publish(velocity)
        self.simulation_velocity_publisher_z.publish(angular_velocity) #We just need to set it to a Wrench object with empty values
        self.lock.release()      

    def pilot_listener_callback(self, msg):  
        controller_input = json.loads(msg.data)

        if controller_input[1] == "backflip":
            self.reset_bot_movement()

        if (len(controller_input[1].split("_")) == 2): #We may get inputs such as "heave-up" or "pitch-down", where the button is being used as an axis
            input = 1 if controller_input[1].split("_")[1] == "up" else -1 
            controller_input[1] = controller_input[1].split("_")[0]
            controller_input[0] = input        

        accepted_thruster_directions = ("surge", "sway", "heave", "pitch", "roll", "yaw") #This script is only responsible for thruster control related actions
        if (controller_input[1] in accepted_thruster_directions[0:3]):
            self.lock.acquire()
            self.force_input_array[controller_input[1]] = float(controller_input[0])*self.max_thruster_force
            self.lock.release()
        elif (controller_input[1] in accepted_thruster_directions[3:]):
            #Will not do drag calculations for angular velocity
            self.lock.acquire()

            self.angular_velocity_array[controller_input[1]] = float(controller_input[0] * self.gazebo_simulation_velocity_pitch_roll_adjustment_factor * self.thruster_force_multipliers["power"]*self.thruster_force_multipliers[controller_input[1]])

            pitch_velocity = Wrench()
            roll_velocity = Wrench()
            yaw_velocity = self.get_current_velocity_xy_twist_object()
            #The problem with the force topic is it applies absoloute and not relative force. 
            #This is not a problem for pitch and roll, but it is why yaw must use the same topic as xy velocity (planar move) 


            pitch_velocity.force.z = float(self.angular_velocity_array["pitch"] * -1)
            roll_velocity.force.z = float(self.angular_velocity_array["roll"])
            yaw_velocity.angular.z = float(-1*(self.angular_velocity_array["yaw"]/self.gazebo_simulation_velocity_pitch_roll_adjustment_factor)*self.gazebo_simulation_velocity_yaw_adjustment_factor) 
            #We don't want the pitch_roll adjustment factor

            

            self.simulation_angular_velocity_publishers["pitch"].publish(pitch_velocity)
            self.simulation_angular_velocity_publishers["roll"].publish(roll_velocity)
            self.simulation_angular_velocity_publishers["yaw"].publish(yaw_velocity)

            self.reset_unused_angular_velocity()
            self.lock.release()

    def get_current_velocity_xy_twist_object(self):
        twist_object = Twist()

        twist_object.linear.x = float((-1)*self.velocity_array["sway"] * self.gazebo_simulation_velocity_xy_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["sway"])  
        twist_object.linear.y = float(self.velocity_array["surge"] * self.gazebo_simulation_velocity_xy_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["surge"])

        return twist_object

    def reset_unused_angular_velocity(self):
        #Reset angular movement after 0.5 seconds of inactivity
        pitch_velocity = Wrench()
        roll_velocity = Wrench()
        yaw_velocity = self.get_current_velocity_xy_twist_object()

        if (time() - self.force_input_time_array["pitch"] > 1):
            self.simulation_angular_velocity_publishers["pitch"].publish(pitch_velocity)
            self.angular_velocity_array["pitch"] = 0
            self.force_input_time_array["pitch"] = time()
            
        if (time() - self.force_input_time_array["roll"] > 1):
            self.simulation_angular_velocity_publishers["roll"].publish(roll_velocity)
            self.angular_velocity_array["roll"] = 0
            self.force_input_time_array["roll"] = time()


        if (time() - self.force_input_time_array["yaw"] > 1):
            yaw_velocity.angular.z = float(0)
            self.simulation_angular_velocity_publishers["yaw"].publish(yaw_velocity)
            self.angular_velocity_array["yaw"] = 0
            self.force_input_time_array["yaw"] = time()

    def calculate_velocity(self, direction):
        i = 0
        while True:
            self.lock.acquire()
            if self.force_input_time_array[direction] == 0:
                self.force_input_time_array[direction] = time()
            else:
                drag_force = 0.5*self.fluid_mass_density*((self.velocity_array[direction])**2)*self.drag_coefficient*self.surface_area_for_drag[direction]

                thruster_force = self.force_input_array[direction]

                if thruster_force != self.previous_force_input_array[direction]:
                    self.net_force_input_time_array[direction] = time()
                    self.previous_force_input_array[direction] = thruster_force

                for _direction in ("surge", "sway", "heave"):
                    if ((time() - self.net_force_input_time_array[_direction]) > 1 and self.force_input_array[_direction] != 0): #Simulate thrusters turn off after 1 second of no input
                        self.force_input_array[_direction] = 0
                        self.net_force_input_time_array[_direction] = time()
                        self.previous_force_input_array[_direction] = 0

                if (self.velocity_array[direction]>0):
                    drag_force=drag_force*(-1) #Drag force acts such that it opposes motion.

                net_force = thruster_force + drag_force

                self.velocity_array[direction] = self.velocity_array[direction]+(((time()-self.force_input_time_array[direction])*net_force)/self.bot_mass) #Momentum divided by mass

                if (time()-self.force_input_time_array[direction] >= 1): #No input for 1 seconds
                    self.force_input_array[direction] = 0

                self.force_input_time_array[direction] = time()

                velocity_xy = Twist()
                velocity_z = Wrench()

                velocity_xy.linear.x = float((-1)*self.velocity_array["sway"] * self.gazebo_simulation_velocity_xy_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["sway"])  

                velocity_xy.linear.y = float(self.velocity_array["surge"] * self.gazebo_simulation_velocity_xy_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["surge"])

                velocity_z.force.z = float((-1)*self.velocity_array["heave"] * self.gazebo_simulation_velocity_z_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["heave"])

                #print(self.velocity_array)
                #print(self.angular_velocity_array)

                self.simulation_velocity_publisher_xy.publish(velocity_xy)
                self.simulation_velocity_publisher_z.publish(velocity_z)
                self.reset_unused_angular_velocity()
                
            i = i + 1
            self.lock.release()
            sleep(0.2)


def main(args=None):
    rclpy.init(args=args)

    thruster_data_subscriber = ThrusterDataSubscriber()

    rclpy.spin(thruster_data_subscriber)

    thruster_data_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
