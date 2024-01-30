from eer_messages.srv import Config

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sqlalchemy import ForeignKey, create_engine
from sqlalchemy.orm import DeclarativeBase, Mapped, Session, mapped_column 
import json

class Base(DeclarativeBase):
    pass

class Profile(Base):
    __tablename__ = "profiles"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    name: Mapped[str] = mapped_column(unique=True)
    controller1: Mapped[str] = mapped_column()
    controller2: Mapped[str] = mapped_column(nullable=True)
    
    def dict(self):
        return {"id": self.id, "name": self.name, "controller1": self.controller1, "controller2": self.controller2}

class Mapping(Base):
    __tablename__ = "mappings"
    id: Mapped[int] = mapped_column(primary_key=True, autoincrement=True)
    name: Mapped[str] = mapped_column(ForeignKey("profiles.name", ondelete="cascade"))
    controller: Mapped[str] = mapped_column()
    button: Mapped[int] = mapped_column()
    action: Mapped[str] = mapped_column()
    isAxis: Mapped[bool] = mapped_column()
    deadzone: Mapped[float] = mapped_column(nullable=True)
    def dict(self):
        return {"id": self.id, "name": self.name, "controller": self.controller, "button": self.button, "action": self.action , "isAxis": self.isAxis, "deadzone": self.deadzone}
    
class Camera(Base):
    __tablename__ = "cameras"
    id: Mapped[int] = mapped_column(primary_key=True)
    ip: Mapped[str] = mapped_column()
    def dict(self):
        return {"id": self.id, "ip": self.ip}

engine = create_engine("sqlite:///config.db")




# [{"id": 0, "name": zaid, "controller": Logi, "button": 3, "isAxis": False, "deadzone": 0.2},
#    {"id": 18, "name": zaid, "controller": xbox, "button": 1, "isAxis": True, "deadzone": 0.1}]


def mappings_list_to_mappings_json(mappings_list):
    """Takes in a list of mappings for a certain profile from the database and turns in into a JSON"""
    i = 0
    controller1 = ""
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
        if i==0:
            controller1 = mapping["controller"]
        
        if mapping["controller"] == controller1:
            controller_1_json_mappings = obtain_bindings(mapping, controller_1_json_mappings)
        else:
            controller_2_json_mappings = obtain_bindings(mapping, controller_2_json_mappings)
        i = i+1

    return {0: controller_1_json_mappings, 1: controller_2_json_mappings}



def mappings_json_to_mappings_list(profile_name, controller_name, mappings_json):
    """Takes in a JSON dictionary for a certain profile and turns in into a mappings list to store in database"""

    buttons_dict = mappings_json["buttons"]
    for i, key in enumerate(list(buttons_dict.keys())):
        query = session.query(Mapping).filter(Mapping.name == profile_name, Mapping.controller == controller_name, Mapping.button == i) 
        if query.count() > 0: #Mapping exists
            query.update({"action":buttons_dict[str(i)]})
            session.commit()
        else:
            new_mapping = Mapping(name = profile_name, controller = controller_name, button = i, action = buttons_dict[str(i)], isAxis = False)
            session.add(new_mapping)

        session.commit()
    
    axes_dict = mappings_json["axes"]
    deadzones_dict = mappings_json["deadzones"]
    for i, key in enumerate(list(axes_dict.keys())):
        query = session.query(Mapping).filter(Mapping.name == profile_name, Mapping.controller == controller_name, Mapping.button == i , Mapping.isAxis == True) 
        if query.count() > 0: #Mapping exists
            query.update({"action":axes_dict[str(i)]})
            if (float(deadzones_dict[str(i)]) != 0):
                query.update({"deadzone":float(deadzones_dict[str(i)])})
            else:
                query.update({"deadzone":None})
            session.commit()
        else:
            if (float(deadzones_dict[str(i)]) == 0):
                new_mapping = Mapping(name = profile_name, controller = controller_name, button = i, action = axes_dict[str(i)], isAxis = True)
            else:
                new_mapping = Mapping(name = profile_name, controller = controller_name, button = i, action = axes_dict[str(i)], isAxis = True, deadzone = float(deadzones_dict[str(i)]))
            
            session.add(new_mapping)

        session.commit()



class ProfilesManager(Node):

    def __init__(self):
        super().__init__('profiles_manager')
        self.srv1 = self.create_service(Config, "profile_config", self.profile_config_callback)
        self.srv2 = self.create_service(Config, "profiles_list", self.profiles_list_callback)

    def profile_config_callback(self, request, response):
        print("Request Recieved")
        if request.state == 0:

            message = json.loads(request.data)

            if session.query(Profile).filter(Profile.name == message["profileName"]).count() == 0: #i.e. profile does not exist
                if (message["controller2"] == "null"):
                    new_profile = Profile(name=message["profileName"], controller1 = message["controller1"])
                else:
                    new_profile = Profile(name=message["profileName"], controller1 = message["controller1"], controller2 = message["controller2"])

                session.add(new_profile)
                session.commit()

            mappings_json_to_mappings_list(message["profileName"], message["controller1"], message["associated_mappings"]["0"]) #This function does not return a value, it directly modifies the database

            if (message["controller2"] != "null"):
                mappings_json_to_mappings_list(message["profileName"], message["controller2"], message["associated_mappings"]["1"])

            response.result = "Success"

            for i in range(session.query(Mapping).filter(Mapping.name == message["profileName"]).count()):
                print(f"{session.query(Mapping).filter_by(name = message['profileName']).all()[i].dict()} recieved in ThrusterControl")

            return response

        elif request.state == 1:

            mappings_list = []
            for row in session.query(Mapping).all():
                if (row.dict()["name"]==request.data): #request.data in this case stores the desired profile
                    mappings_list.append(row.dict())
            mappings_json = mappings_list_to_mappings_json(mappings_list)
            response.result = json.dumps(mappings_json)
            return response

    def profiles_list_callback(self, request, response):
        print("Request Recieved")
        if request.state == 1: #We only want to read the profiles
            output = []
            for row in session.query(Profile).all():
                output.append(row.dict())
            response.result = json.dumps(output)
            return response
        else:
            response.result = "Only loading works for this service"
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

