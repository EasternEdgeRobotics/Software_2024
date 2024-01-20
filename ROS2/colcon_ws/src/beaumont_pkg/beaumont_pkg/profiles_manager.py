from eer_messages.srv import Config

import rclpy
from rclpy.node import Node
import json
import os

class ProfilesManager(Node):

    def __init__(self):
        super().__init__("profiles_manager")
        self.srv = self.create_service(Config, "profiles_config", self.profiles_config_callback)

    def profiles_config_callback(self, request, response):
        file_path = "/home/colcon_ws/src/beaumont_pkg/beaumont_pkg"
        if request.state==0:
            with open(f"{file_path}/json/profiles.json", "w") as file:
                json.dump(json.loads(request.data), file) #Overwrite the profiles.json file with the data coming from the GUI
            response.result = "success"    
            return response
        elif request.state==1:
            with open(f"{file_path}/json/profiles.json", "r") as file:
                response.result = str(file.read())
            return response #Return the stored data to the GUI

def main(args=None):
    rclpy.init(args=args)

    profiles_manager = ProfilesManager()

    rclpy.spin(profiles_manager)

    rclpy.shutdown()

if __name__ == "__main__":
    main()

