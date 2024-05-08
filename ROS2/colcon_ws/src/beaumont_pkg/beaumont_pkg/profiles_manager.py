from eer_messages.srv import Config

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sqlalchemy import ForeignKey, create_engine
from sqlalchemy.orm import DeclarativeBase, Mapped, Session, mapped_column 
import json

class Base(DeclarativeBase):
    pass

#Define the Profiles database schema
class Profile(Base):
    __tablename__ = "profiles"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    name: Mapped[str] = mapped_column(unique=True)
    controller1: Mapped[str] = mapped_column()
    controller2: Mapped[str] = mapped_column(nullable=True)
    
    def dict(self):
        return {"id": self.id, "name": self.name, "controller1": self.controller1, "controller2": self.controller2}

#Define the Mappings database schema
class Mapping(Base):
    __tablename__ = "mappings"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    name: Mapped[str] = mapped_column(ForeignKey("profiles.name", ondelete="cascade"))
    controller: Mapped[str] = mapped_column()
    controllerNumber: Mapped[int] = mapped_column()
    button: Mapped[int] = mapped_column()
    action: Mapped[str] = mapped_column()
    isAxis: Mapped[bool] = mapped_column()
    deadzone: Mapped[float] = mapped_column(nullable=True)
    def dict(self):
        return {"id": self.id, "name": self.name, "controller": self.controller,"controllerNumber": self.controllerNumber , "button": self.button, "action": self.action , "isAxis": self.isAxis, "deadzone": self.deadzone}

class PowerPreset(Base):
    __tablename__ = "powerPresets"
    name: Mapped[str] = mapped_column(primary_key=True)
    power: Mapped[int] = mapped_column()
    surge: Mapped[int] = mapped_column()
    sway: Mapped[int] = mapped_column()
    heave: Mapped[int] = mapped_column()
    pitch: Mapped[int] = mapped_column()
    roll: Mapped[int] = mapped_column()
    yaw: Mapped[int] = mapped_column()

class Camera(Base):
    __tablename__ = "cameras"
    id: Mapped[int] = mapped_column(primary_key=True)
    url1: Mapped[str] = mapped_column()
    url2: Mapped[str] = mapped_column()
    url3: Mapped[str] = mapped_column()
    url4: Mapped[str] = mapped_column()
    def dict(self):
        return {"url1":self.url1, "url2":self.url2, "url3":self.url3, "url4":self.url4}

engine = create_engine("sqlite:///config.db")

def mappings_list_to_mappings_json(mappings_list):
    """Takes in a list of mappings for a certain profile from the database and turns in into a JSON"""

    controller_1_json_mappings = {"axes": {}, "buttons": {}, "deadzones": {}}
    controller_2_json_mappings = {"axes": {}, "buttons": {}, "deadzones": {}}

    def obtain_bindings(binding, _json_mappings):
        if binding["isAxis"] == True:
            _json_mappings["axes"][binding["button"]] = binding["action"]
            if binding["deadzone"]==None:
                binding["deadzone"] = 0
            _json_mappings["deadzones"][binding["button"]] = str(binding["deadzone"])
        else:
            _json_mappings["buttons"][binding["button"]] = binding["action"]
        return _json_mappings
            
    for mapping in mappings_list:

        if mapping["controllerNumber"] == 0:
            controller_1_json_mappings = obtain_bindings(mapping, controller_1_json_mappings)
        elif mapping["controllerNumber"] == 1:
            controller_2_json_mappings = obtain_bindings(mapping, controller_2_json_mappings)

    return {0: controller_1_json_mappings, 1: controller_2_json_mappings}

def mappings_json_to_mappings_list(profile_name, controller_name, mappings_json, controller_number):
    """Takes in a JSON dictionary for a certain profile and turns in into a mappings list to store in database"""

    buttons_dict = mappings_json["buttons"]
    for i, key in enumerate(list(buttons_dict.keys())):
        new_mapping = Mapping(name = profile_name, controller = controller_name, controllerNumber = controller_number, button = i, action = buttons_dict[str(i)], isAxis = False) 
        session.add(new_mapping)

        session.commit()
    
    axes_dict = mappings_json["axes"]
    deadzones_dict = mappings_json["deadzones"]
    for i, key in enumerate(list(axes_dict.keys())):
        if (float(deadzones_dict[str(i)]) == 0): 
            new_mapping = Mapping(name = profile_name, controller = controller_name, controllerNumber = controller_number, button = i, action = axes_dict[str(i)], isAxis = True)
        else:
            new_mapping = Mapping(name = profile_name, controller = controller_name, controllerNumber = controller_number, button = i, action = axes_dict[str(i)], isAxis = True, deadzone = float(deadzones_dict[str(i)]))
            
        session.add(new_mapping)

        session.commit()



class ProfilesManager(Node):

    def __init__(self):
        super().__init__('profiles_manager')

        #The ROS services below communicate straight with the GUI, as long as rosbridge_server is running
        #The service names ("profile_config" and "profiles_list") MUST match those defined in the GUI
        self.srv1 = self.create_service(Config, "profile_config", self.profile_config_callback)
        self.srv2 = self.create_service(Config, "profiles_list", self.profiles_list_callback)
        self.srv3 = self.create_service(Config, "camera_urls", self.camera_urls_callback)

    def profile_config_callback(self, request, response):
        if request.state == 0: #We are looking to load mappings into database from GUI

            message = json.loads(request.data) #Turn the recieved string into a JSON object (i.e. Python dictionary)

            query = session.query(Profile).filter(Profile.name == message["profileName"]) # Remove the profile if it exists as it will be recreated
            if query.count() == 1: #i.e. profile exists
                query.delete()
                session.commit()

            if (message["controller2"] == "null"):
                new_profile = Profile(name=message["profileName"], controller1 = message["controller1"])
            else:
                new_profile = Profile(name=message["profileName"], controller1 = message["controller1"], controller2 = message["controller2"])

            session.add(new_profile)
            session.commit()

            mappings_json_to_mappings_list(message["profileName"], message["controller1"], message["associated_mappings"]["0"], 0) #This function does not return a value, it directly modifies the database

            if (message["controller2"] != "null"):
                mappings_json_to_mappings_list(message["profileName"], message["controller2"], message["associated_mappings"]["1"], 1)

            response.result = "Success"

            #==========================DEBUG===========================
            # for i in range(session.query(Mapping).filter(Mapping.name == message["profileName"]).count()):
            #    self.get_logger().info(f"{session.query(Mapping).filter_by(name = message['profileName']).all()[i].dict()} recieved")
            #==========================================================

            return response

        elif request.state == 1: #We are looking to load mapping into GUI from database
            mappings_list = []
            for row in session.query(Mapping).all():
                if (row.dict()["name"]==request.data): #request.data in this case stores the name of the profile for which mappings are being requested
                    mappings_list.append(row.dict())
            mappings_json = mappings_list_to_mappings_json(mappings_list)
            response.result = json.dumps(mappings_json) #Turn the JSON object into a string
            return response

    def profiles_list_callback(self, request, response):
        if request.state == 0:
            query = session.query(Profile).filter(Profile.name == request.data) #In this case, the request data is expected to only be a string
            if query.count() == 1: #i.e. profile exists
                query.delete()
                session.commit()
                response.result = "Profile Deleted"
            else:
                response.result = "Profile not found"
            return response
        elif request.state == 1: #We only want to read the profiles
            output = []
            for row in session.query(Profile).all():
                output.append(row.dict())
            response.result = json.dumps(output) #Turn the JSON object into a string
            return response
        
    def camera_urls_callback(self, request, response):

        if request.state == 0: # We are looking to load camera URLs into database from GUI

            message = json.loads(request.data) #Turn the recieved string into a List

            query = session.query(Camera) # Remove the any saved camera URLs
            if query.count() >= 1: 
                query.delete()
                session.commit()

            while True:
                if len(message) < 4:
                    message.append("http://")
                else:
                    break

            new_camera_urls = Camera(url1=message[0], url2=message[1], url3=message[2], url4=message[3])

            session.add(new_camera_urls)
            session.commit()

            response.result = "Success"

            #==========================DEBUG===========================
            # for i in range(session.query(Camera).count()):
            #     self.get_logger().info(f"{session.query(Camera).all()[i].dict()} recieved")
            #==========================================================

            return response
        
        elif request.state == 1: # We are looking to fetch camera URLs form database into GUI  
            outgoing_camera_urls = []
            for row in range(session.query(Camera).count()): # There will only be one "row"
                stored_camera_urls = session.query(Camera).all()[row].dict()
                outgoing_camera_urls = [stored_camera_urls["url1"], stored_camera_urls["url2"], stored_camera_urls["url3"], stored_camera_urls["url4"]]

            response.result = json.dumps(outgoing_camera_urls) #Turn the list into a string

            return response

            

def main(args=None):
    global session
    Base.metadata.create_all(engine)
    session = Session(engine)
    rclpy.init(args=args)
    profiles_manager = ProfilesManager()
    rclpy.spin(profiles_manager)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

