from eer_messages.srv import Config

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sqlalchemy import ForeignKey, create_engine
from sqlalchemy.orm import DeclarativeBase, Mapped, Session, mapped_column 
from sqlalchemy import Column, Integer, Boolean, create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy import String as SQLString
import json
from sqlalchemy.orm import sessionmaker

class Base(DeclarativeBase):
    pass
class Task(Base):
    __tablename__ = 'tasks'
    id = Column(SQLString, primary_key=True)
    status = Column(Boolean)

engine = create_engine('sqlite:///tasks.db')  # Use an SQLite database file named tasks.db
Session = sessionmaker(bind=engine)

Base.metadata.create_all(engine)  # Create the tasks table

class TaskManager(Node):

    def __init__(self):
        super().__init__('task_manager')
        self.get_logger().info("TASK MANAGE NODE ALIVE")
        self.srv1 = self.create_service(Config, "all_tasks", self.get_all_tasks)
        self.srv2 = self.create_service(Config, "reset_tasks", self.delete_all_tasks)

        self.subscription = self.create_subscription(
            String,
            'task_updates',
            self.task_listener_callback,
            10)

    def task_listener_callback(self, msg):
        self.get_logger().info(msg.data)
        data = json.loads(msg.data)
        taskID:str = data['id']
        taskStatus:bool = data['status']

        session = Session()
        try:
            task = session.query(Task).get(taskID)
            if task is None:
                # The task doesn't exist, so create a new one
                task = Task(id=taskID, status=taskStatus)
                session.add(task)
            else:
                # The task exists, so update its status
                task.status = taskStatus
            session.commit()
        finally:
            session.close()

    def get_all_tasks(self, request, response):
        try:
            session = Session()
            tasks = session.query(Task).all()
            result_dict = {task.id: task.status for task in tasks}
            response.result = json.dumps(result_dict)  # Convert the dictionary to a JSON string
            self.get_logger().info(response.result)
        except Exception as e:
            self.get_logger().error(f"Error getting tasks: {e}")
        return response

    def delete_all_tasks(self, request, response):
        session = Session()
        session.query(Task).delete()
        session.commit()
        response.result = "All tasks deleted"
        return response

def main(args=None):
    # global session
    Base.metadata.create_all(engine)
    # session = Session(engine)
    rclpy.init(args=args)
    task_manager = TaskManager()
    rclpy.spin(task_manager)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

