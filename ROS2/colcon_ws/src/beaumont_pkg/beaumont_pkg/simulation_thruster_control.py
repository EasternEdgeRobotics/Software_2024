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

        #For xy movement, the model is using the Gazebo Planar Move Plugin, which allows for movement relative to itself rather than the world
        #It also uses this plugin for yaw for the same reason
        self.simulation_velocity_publisher_xy = self.create_publisher(Twist, "/demo/simulation_model_linear_velocity_xy", 10) 

        #For movement in the z axis, the model uses the Gazebo Force Plugin
        self.simulation_velocity_publisher_z = self.create_publisher(Wrench, "/demo/link/simulation_model_linear_velocity_z", 10) 

        #For Roll and Pitch, the model also uses the Gazebo Force Plugin, applied at specifc spots in order to cause pitch or roll
        #For yaw, the model uses the same topic and plugin as the xy movement
        self.simulation_angular_velocity_publishers = {"pitch":self.create_publisher(Wrench, "/demo/link/force_demo_pitch", 10),
                                                    "roll":self.create_publisher(Wrench, "/demo/link/force_demo_roll", 10),
                                                    "yaw":self.simulation_velocity_publisher_xy}

        # The differnce between the planar move plugin and the force plugin is that the palanor move plugin acts relative to the bot, while the force plugin acts relative to the world
        # All forces applied with the Gazebo Force plugin apply up in the z axis. This leads to a problem where the pitch and roll force get weaker the more that the bot it tilted

        self.lock = threading.Lock() #Used to synchronize all the threads (concurrent processes) which are accessing and modifing the same variables

        self.force_input_array = {"surge":0, 
                                    "sway":0,
                                    "heave":0} #Array storing the force being applied by the thrusters

        self.velocity_array = {"surge":0,
                                    "sway":0,
                                    "heave":0} #Array storing the velocity that should be getting applied to the BOT (comes out an order of magnitude higher in calculations... fixed by offsets)
        
        self.angular_velocity_array = {"pitch":0,
                                       "roll":0,
                                       "yaw":0} #Array storing the pesudo angular velocity. This is not a real velocity but is applied straight as a "force" on the model... which Gazebo treats as a velocity

        self.force_input_time_array = {"surge":0,
                                                "sway":0,
                                                "heave":0,
                                                "pitch":0,
                                                "roll":0,
                                                "yaw":0} #The time for which the force has been held for since the thread checks. Used to calculate momentum then velocity.

        self.previous_force_input_array = {"surge":0,
                                    "sway":0,
                                    "heave":0} #The force before the current force that is being applied

        self.net_force_input_time_array = {"surge":0,
                                                "sway":0,
                                                "heave":0} #The net time for which a force has been continously held for. Used to rest force on bot.

        self.surface_area_for_drag = {"surge":0.1813,
                                        "sway":0.2035,
                                        "heave":0.2695} #Used for drag calculations. The bot is treated as a box
        
        self.thruster_force_multipliers = {"power":0,
                                        "surge":0,
                                        "sway":0,
                                        "heave":0,
                                        "pitch":0,
                                        "roll":0,
                                        "yaw":0} #Force multiplier values coming from copilot

        self.bot_mass = 23 #23kg

        self.fluid_mass_density = 1000 #1000kg/m^3

        self.drag_coefficient = 0.025 #This value has been tuned. The higher it is, the faster the bot reaches terminal velocity

        self.max_thruster_force = 116 #Thruster force in Newtons

        self.gazebo_simulation_velocity_z_adjustment_factor = 150 
        #Should be 4775 to match real velocity... but this is too fast 
        #I persume the the parameters in the calculations are off or that the Python time modules is off
        #Assuming the terminal velocity of the bot in water is 0.25 m/s... the force adjustment factor should be 150
        
        self.gazebo_simulation_velocity_xy_adjustment_factor = 0.04
        #The velocity applied by the Gazebo Planar Move plugin is exact
        #The calculations using a thruster force of 116 Newtons and a Bot mass of 23kg call for a terminal velocity
        #Of around 7m/s. This adjustment factor takes that down to 0.28m/s

        self.gazebo_simulation_velocity_pitch_roll_adjustment_factor = 75
        #No real reason for this adjustment factor other than that it seems reasonable in practice
        #There is no attempt to simulate torque and drag with pitch, roll, and yaw with angular velocity calculations

        self.gazebo_simulation_velocity_yaw_adjustment_factor = 0.075
        #This uses the Gazebo Planar Move Plugin. Adjustment factor is reasonable

        #The bottom 3 threads are used for calculating velocity based on thruster force and drag
        self.surge_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("surge",) , daemon=True)
        self.sway_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("sway",) , daemon=True)
        self.heave_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("heave",) , daemon=True)

        #Start each thread
        self.surge_velocity_thread.start()
        self.sway_velocity_thread.start()
        self.heave_velocity_thread.start()

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

    def stop_bot(self):
        '''Stop All Movement. This is not equivalent to stopping thrusters, as this function sets velocity (and not force) to zero.'''

        #Create an empty Wrench and Twist object
        angular_velocity = Wrench()
        velocity = Twist()

        self.lock.acquire()
        self.simulation_angular_velocity_publishers["pitch"].publish(angular_velocity)
        self.simulation_angular_velocity_publishers["roll"].publish(angular_velocity)
        self.simulation_angular_velocity_publishers["yaw"].publish(velocity) # 
        self.simulation_velocity_publisher_xy.publish(velocity)
        self.simulation_velocity_publisher_z.publish(angular_velocity) 
        self.lock.release()      

    def pilot_listener_callback(self, msg):  
        '''Called when new controller input from pilot is recieved'''
        controller_input = json.loads(msg.data)

        if controller_input[1] == "backflip": #Perhaps this should be named better
            self.stop_bot()

        if (len(controller_input[1].split("_")) == 2): #We may get inputs such as "heave_up" or "pitch_down", where a button is being used instead of an axis
            input = 1 if controller_input[1].split("_")[1] == "up" else -1 
            controller_input[1] = controller_input[1].split("_")[0]
            controller_input[0] = input        

        accepted_thruster_directions = ("surge", "sway", "heave", "pitch", "roll", "yaw") #This script is only responsible for thruster control related actions

        if (controller_input[1] in accepted_thruster_directions[0:3]): 
            self.lock.acquire()
            self.force_input_array[controller_input[1]] = float(controller_input[0])*self.max_thruster_force
            self.lock.release()
        elif (controller_input[1] in accepted_thruster_directions[3:]):
            #Will not do force and drag calculations for angular movement
            self.lock.acquire()

            #Immediately multiply the controller input by the Gazebo pitch roll adjustment factor which causes reasonable movement for the bot
            self.angular_velocity_array[controller_input[1]] = float(controller_input[0] * self.gazebo_simulation_velocity_pitch_roll_adjustment_factor * self.thruster_force_multipliers["power"]*self.thruster_force_multipliers[controller_input[1]])

            pitch_velocity = Wrench()
            roll_velocity = Wrench()
            yaw_velocity = self.get_current_velocity_xy_twist_object() #Since yaw velocity publishes to the same topic as surge and sway, it should be ensured that it does not overwrite them to 0


            pitch_velocity.force.z = float(self.angular_velocity_array["pitch"] * -1)
            roll_velocity.force.z = float(self.angular_velocity_array["roll"])

            #For yaw, divide by the pitch_roll_adjustment_factor (to return it to the raw controller input * copilot power multiplier), then multiply by the yaw adjustment factor
            yaw_velocity.angular.z = float(-1*(self.angular_velocity_array["yaw"]/self.gazebo_simulation_velocity_pitch_roll_adjustment_factor)*self.gazebo_simulation_velocity_yaw_adjustment_factor) 
            
            #Publish angular velocities
            self.simulation_angular_velocity_publishers["pitch"].publish(pitch_velocity)
            self.simulation_angular_velocity_publishers["roll"].publish(roll_velocity)
            self.simulation_angular_velocity_publishers["yaw"].publish(yaw_velocity)
            
            self.reset_unused_angular_velocity()
            self.lock.release()

    def get_current_velocity_xy_twist_object(self):
        '''Returns a Twist Object that Contains the Current Linear x and y velocity'''
        twist_object = Twist()

        twist_object.linear.x = float((-1)*self.velocity_array["sway"] * self.gazebo_simulation_velocity_xy_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["sway"])  
        twist_object.linear.y = float(self.velocity_array["surge"] * self.gazebo_simulation_velocity_xy_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["surge"])

        return twist_object

    def reset_unused_angular_velocity(self):
        '''Reset Angular Velocity to 0 After Certain Amount of Inactivity'''
        #Reset angular movement after cetain amount of inactivity

        inactivity_threshold = 1 # 1 second

        pitch_velocity = Wrench()
        roll_velocity = Wrench()
        yaw_velocity = self.get_current_velocity_xy_twist_object() #Since yaw velocity publishes to the same topic as surge and sway, it should be ensured that it does not overwrite them to 0
        
        if (time() - self.force_input_time_array["pitch"] > inactivity_threshold ):
            self.simulation_angular_velocity_publishers["pitch"].publish(pitch_velocity)
            self.angular_velocity_array["pitch"] = 0
            self.force_input_time_array["pitch"] = time()
            
        if (time() - self.force_input_time_array["roll"] > inactivity_threshold ):
            self.simulation_angular_velocity_publishers["roll"].publish(roll_velocity)
            self.angular_velocity_array["roll"] = 0
            self.force_input_time_array["roll"] = time()

        if (time() - self.force_input_time_array["yaw"] > inactivity_threshold ):
            yaw_velocity.angular.z = float(0)
            self.simulation_angular_velocity_publishers["yaw"].publish(yaw_velocity)
            self.angular_velocity_array["yaw"] = 0
            self.force_input_time_array["yaw"] = time()

    def calculate_velocity(self, direction):
        '''Perform Velocity Calculations with Thruster Force and Drag'''
        while True:
            self.lock.acquire()
            if self.force_input_time_array[direction] == 0: #i.e. This is the very first iteration of this function
                self.force_input_time_array[direction] = time()
            else:
                #Calculate drag force based on drag force formula for a cube
                drag_force = 0.5*self.fluid_mass_density*((self.velocity_array[direction])**2)*self.drag_coefficient*self.surface_area_for_drag[direction]

                #Get the thruster force
                thruster_force = self.force_input_array[direction]

                if thruster_force != self.previous_force_input_array[direction]: #i.e. A new force is being applied
                    self.net_force_input_time_array[direction] = time()
                    self.previous_force_input_array[direction] = thruster_force

                #No matter if this function is for the surge, sway, or heave thread, this for loop will be executed
                for _direction in ("surge", "sway", "heave"):
                    if ((time() - self.net_force_input_time_array[_direction]) > 1 and self.force_input_array[_direction] != 0): #Simulate thrusters turn off after 1 second of no input
                        self.force_input_array[_direction] = 0
                        self.net_force_input_time_array[_direction] = time()
                        self.previous_force_input_array[_direction] = 0

                if (self.velocity_array[direction]>0):
                    drag_force=drag_force*(-1) #Drag force acts such that it opposes motion.

                net_force = thruster_force + drag_force 
                #Drag force will usually be the opposite sign of the thruster force, but this is not necessarily the case

                self.velocity_array[direction] = self.velocity_array[direction]+(((time()-self.force_input_time_array[direction])*net_force)/self.bot_mass) #Momentum divided by mass = velocity

                self.force_input_time_array[direction] = time()

                velocity_xy = Twist()
                velocity_z = Wrench()

                velocity_xy.linear.x = float((-1)*self.velocity_array["sway"] * self.gazebo_simulation_velocity_xy_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["sway"])  

                velocity_xy.linear.y = float(self.velocity_array["surge"] * self.gazebo_simulation_velocity_xy_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["surge"])

                velocity_z.force.z = float((-1)*self.velocity_array["heave"] * self.gazebo_simulation_velocity_z_adjustment_factor * self.thruster_force_multipliers["power"] * self.thruster_force_multipliers["heave"])

                #=================DEBUG==================
                #print(self.velocity_array)
                #print(self.angular_velocity_array)
                #========================================

                self.simulation_velocity_publisher_xy.publish(velocity_xy)
                self.simulation_velocity_publisher_z.publish(velocity_z)
                self.reset_unused_angular_velocity()
                
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
