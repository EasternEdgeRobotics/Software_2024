import rclpy
from rclpy.node import Node 
from eer_messages.msg import ThrusterMultipliers
from std_msgs.msg import String
from geometry_msgs.msg import Wrench
import json
import threading
from time import time, sleep

class ThrusterDataSubscriber(Node):

    def __init__(self):
        super().__init__('thruster_data_subscriber')
        self.copilot_listener = self.create_subscription(ThrusterMultipliers, 'thruster_multipliers', self.copilot_listener_callback, 10)
        self.pilot_listener = self.create_subscription(String, 'controller_input', self.pilot_listener_callback, 10)
        self.simulation_velocity_publisher = self.create_publisher(Wrench, "/demo/link/simulation_model_linear_velocity", 10) 

        self.lock = threading.Lock() #Used as a synchronization tool

        #From the simulation, it appears that a force of 750N (not really a force) is just equivalent to giving it a velocity of ~0.2m/s. Tune this number later.

        self.force_input_array = {"surge":0,
                                    "sway":0,
                                    "heave":0}

        self.velocity_array = {"surge":0,
                                    "sway":0,
                                    "heave":0}

        self.force_input_time_array = {"surge":0,
                                                "sway":0,
                                                "heave":0}

        self.previous_force_input_array = {"surge":0,
                                    "sway":0,
                                    "heave":0}

        self.net_force_input_time_array = {"surge":0,
                                                "sway":0,
                                                "heave":0}

        self.surface_area_for_drag = {"surge":0.1813,
                                        "sway":0.2035,
                                        "heave":0.2695}

        self.thread_lock = {"surge":True,
                            "sway":True,
                            "heave":True}

        self.bot_mass = 23 #23kg
        self.fluid_mass_density = 1000 #1000kg/m^3
        self.drag_coefficient = 0.025 #This value has been tuned
        self.force_adjustment_factor = 0.075 #It seems to me like the momentum calculations are giving something an order of magnitude higher than what they should. 
                                            #Maybe because of the the python time module. This adjustment factor should be tuned to be more realistic 
        self.max_thruster_force = 116
        self.surge_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("surge",) , daemon=True)
        self.sway_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("sway",) , daemon=True)
        self.heave_velocity_thread = threading.Thread(target=self.calculate_velocity,args=("heave",) , daemon=True)

        self.surge_velocity_thread.start()
        self.sway_velocity_thread.start()
        self.heave_velocity_thread.start()

        # prevent unused variable warning
        self.copilot_listener 
        self.pilot_listener

    def copilot_listener_callback(self, msg):
        self.get_logger().info(f"{msg} recieved in ThrusterControl")
        #TODO ROV Math

    def pilot_listener_callback(self, msg):  
        controller_input = json.loads(msg.data)
        accepted_thruster_directions = ("surge", "sway", "heave", "pitch", "roll", "yaw") #This script is only responsible for thruster control related actions
        if (controller_input[1] in accepted_thruster_directions[0:3]):
            self.lock.acquire()
            self.force_input_array[controller_input[1]] = float(controller_input[0])*self.max_thruster_force
            #print(self.force_input_array)
            self.lock.release()

    def calculate_velocity(self, direction):
        i = 0
        while True:
            self.lock.acquire()
            if self.force_input_time_array[direction] == 0:
                self.force_input_time_array[direction] = time()
                #print("A time reading: ", self.force_input_time_array[direction])
            else:
                drag_force = 0.5*self.fluid_mass_density*((self.velocity_array[direction]/self.force_adjustment_factor)**2)*self.drag_coefficient*self.surface_area_for_drag[direction]

                thruster_force = self.force_input_array[direction]

                if thruster_force != self.previous_force_input_array[direction]:
                    self.net_force_input_time_array[direction] = time()
                    self.previous_force_input_array[direction] = thruster_force

                if ((time() - self.net_force_input_time_array[direction]) > 1 and self.force_input_array[direction] != 0): #Simulate thrusters turn off after 1 second of no input
                    self.force_input_array[direction] = 0
                    self.net_force_input_time_array[direction] = time()
                    self.previous_force_input_array[direction] = 0

                if (self.velocity_array[direction]>0):
                    drag_force=drag_force*(-1) #Drag force acts such that it opposes motion. Technically it should oppose velocity and not force... but it's too jank

                net_force = thruster_force + drag_force

                self.velocity_array[direction] = self.velocity_array[direction]+(((time()-self.force_input_time_array[direction])*net_force*self.force_adjustment_factor)/self.bot_mass) #Momentum divided by mass

                if (time()-self.force_input_time_array[direction] >= 1): #No input for 1 seconds
                    self.force_input_array[direction] = 0

                self.force_input_time_array[direction] = time()

                velocity = Wrench()

                velocity.force.x = float((-1)*self.velocity_array["sway"] * 750*2) #Gazebo is weird, 750*2 is an adjustment factor to give actual velocity 

                velocity.force.y = float(self.velocity_array["surge"] * 750*2)

                velocity.force.z = float((-1)*self.velocity_array["heave"] * 750*2)

                self.simulation_velocity_publisher.publish(velocity)
                
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
