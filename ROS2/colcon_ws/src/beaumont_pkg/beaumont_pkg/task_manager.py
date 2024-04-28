from eer_messages.srv import Config

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sqlalchemy import ForeignKey, create_engine
from sqlalchemy.orm import DeclarativeBase, Mapped, Session, mapped_column 
import json

class Base(DeclarativeBase):
    pass

engine = create_engine("sqlite:///config.db")

class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        self.get_logger().info("TASK MANAGE NODE ALIVE")
        #The ROS services below communicate straight with the GUI, as long as rosbridge_server is running
        #The service names ("profile_config" and "profiles_list") MUST match those defined in the GUI
        # self.srv1 = self.create_service(Config, "task_list", self.profile_config_callback)
        # self.srv2 = self.create_service(Config, "task_status", self.profiles_list_callback)

        self.subscription = self.create_subscription(
            String,
            'task_updates',
            self.task_listener_callback,
            10)


    def task_listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        
        print("I heardddd: ", msg.data)
        # taskID:str = msg.data.id
        # taskStatus:bool = msg.data.status

        pass

    # def profile_config_callback(self, request, response):
    #     if request.state == 0: #We are looking to load mappings into database from GUI

    #         message = json.loads(request.data) #Turn the recieved string into a JSON object (i.e. Python dictionary)

    #         query = session.query(Profile).filter(Profile.name == message["profileName"]) # Remove the profile if it exists as it will be recreated
    #         if query.count() == 1: #i.e. profile exists
    #             query.delete()
    #             session.commit()

    #         if (message["controller2"] == "null"):
    #             new_profile = Profile(name=message["profileName"], controller1 = message["controller1"])
    #         else:
    #             new_profile = Profile(name=message["profileName"], controller1 = message["controller1"], controller2 = message["controller2"])

    #         session.add(new_profile)
    #         session.commit()

    #         mappings_json_to_mappings_list(message["profileName"], message["controller1"], message["associated_mappings"]["0"], 0) #This function does not return a value, it directly modifies the database

    #         if (message["controller2"] != "null"):
    #             mappings_json_to_mappings_list(message["profileName"], message["controller2"], message["associated_mappings"]["1"], 1)

    #         response.result = "Success"

    #         #==========================DEBUG===========================
    #         # for i in range(session.query(Mapping).filter(Mapping.name == message["profileName"]).count()):
    #         #    self.get_logger().info(f"{session.query(Mapping).filter_by(name = message['profileName']).all()[i].dict()} recieved")
    #         #==========================================================

    #         return response

    #     elif request.state == 1: #We are looking to load mapping into GUI from database
    #         mappings_list = []
    #         for row in session.query(Mapping).all():
    #             if (row.dict()["name"]==request.data): #request.data in this case stores the name of the profile for which mappings are being requested
    #                 mappings_list.append(row.dict())
    #         mappings_json = mappings_list_to_mappings_json(mappings_list)
    #         response.result = json.dumps(mappings_json) #Turn the JSON object into a string
    #         return response

    # def profiles_list_callback(self, request, response):
    #     if request.state == 0:
    #         query = session.query(Profile).filter(Profile.name == request.data) #In this case, the request data is expected to only be a string
    #         if query.count() == 1: #i.e. profile exists
    #             query.delete()
    #             session.commit()
    #             response.result = "Profile Deleted"
    #         else:
    #             response.result = "Profile not found"
    #         return response
    #     elif request.state == 1: #We only want to read the profiles
    #         output = []
    #         for row in session.query(Profile).all():
    #             output.append(row.dict())
    #         response.result = json.dumps(output) #Turn the JSON object into a string
    #         return response
        
    # def camera_urls_callback(self, request, response):

    #     if request.state == 0: # We are looking to load camera URLs into database from GUI

    #         message = json.loads(request.data) #Turn the recieved string into a List

    #         query = session.query(Camera) # Remove the any saved camera URLs
    #         if query.count() >= 1: 
    #             query.delete()
    #             session.commit()

    #         while True:
    #             if len(message) < 4:
    #                 message.append("http://")
    #             else:
    #                 break

    #         new_camera_urls = Camera(url1=message[0], url2=message[1], url3=message[2], url4=message[3])

    #         session.add(new_camera_urls)
    #         session.commit()

    #         response.result = "Success"

    #         #==========================DEBUG===========================
    #         # for i in range(session.query(Camera).count()):
    #         #     self.get_logger().info(f"{session.query(Camera).all()[i].dict()} recieved")
    #         #==========================================================

    #         return response
        
    #     elif request.state == 1: # We are looking to fetch camera URLs form database into GUI

    #         for row in range(session.query(Camera).count()): # There will only be one "row"
    #             stored_camera_urls = session.query(Camera).all()[row].dict()
    #             outgoing_camera_urls = [stored_camera_urls["url1"], stored_camera_urls["url2"], stored_camera_urls["url3"], stored_camera_urls["url4"]]

    #         response.result = json.dumps(outgoing_camera_urls) #Turn the list into a string

    #         return response

            

def main(args=None):
    global session
    Base.metadata.create_all(engine)
    session = Session(engine)
    rclpy.init(args=args)
    task_manager = TaskManager()
    rclpy.spin(task_manager)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

