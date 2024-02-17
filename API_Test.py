from sqlalchemy import ForeignKey, create_engine, select
from sqlalchemy.orm import DeclarativeBase, Mapped, Session, mapped_column
from flask import Flask

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
        return {"id": self.id, "name": self.name, "controller": self.controller, "button": self.button, "isAxis": self.isAxis, "deadzone": self.deadzone}
    
class Camera(Base):
    __tablename__ = "cameras"
    id: Mapped[int] = mapped_column(primary_key=True)
    ip: Mapped[str] = mapped_column()
    def dict(self):
        return {"id": self.id, "ip": self.ip}

engine = create_engine("sqlite:///config.db")
app = Flask(__name__)

@app.route("/")
def test():
    output = {"cameras": [], "profiles": [], "mappings": []}
    for row in session.query(Camera).all():
        output["cameras"].append(row.dict())
    for row in session.query(Profile).all():
        output["profiles"].append(row.dict())
    for row in session.query(Mapping).all():
        output["mappings"].append(row.dict())
    return output

def main():
    global session
    Base.metadata.create_all(engine)
    session = Session(engine)
    app.run()


if __name__ == "__main__":
    main()