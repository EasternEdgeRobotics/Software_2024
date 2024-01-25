from eer_messages.srv import Config

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sqlalchemy import ForeignKey, create_engine
from sqlalchemy.orm import DeclarativeBase, Mapped, Session, mapped_column

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
    isAxis: Mapped[bool] = mapped_column()
    deadzone: Mapped[float] = mapped_column(nullable=True)
    def dict(self):
        return {"id": self.id, "name": self.name, "controller": self.controller, "button": self.button, "isAxis": self.isAxis, "deadzone": self.deadzone}
    
class Camera(Base):
    __tablename__ = "cameras"
    id: Mapped[int] = mapped_column(primary_key=True)
    ip: Mapped[str] = mapped_column()
    def dict(self):
        return {"id": self.id, "ip": self.ip}

engine = create_engine("sqlite:///config.db")

class ProfilesManager(Node):

    def __init__(self):
        super().__init__('profiles_manager')
        self.subscription = self.create_subscription(String, "requestconfig", self.sendConfig, 10)
        self.publisher_ = self.create_publisher(String, "getconfig", 10)

    def sendConfig(self, msg):
        output = {"cameras": [], "profiles": [], "mappings": []}
        for row in session.query(Camera).all():
            output["cameras"].append(row.dict())
        for row in session.query(Profile).all():
            output["profiles"].append(row.dict())
        for row in session.query(Mapping).all():
            output["mappings"].append(row.dict())
        msg = String()
        msg.data = str(output)
        self.publisher_.publish(msg)

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

