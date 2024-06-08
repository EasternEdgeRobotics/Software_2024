FROM ros:humble-ros-base-jammy

RUN apt-get update && apt-get install -y python3 python3-pip ros-humble-rosbridge-server ffmpeg libsm6 libxext6
RUN pip3 install sqlalchemy RPi.GPIO smbus2 adafruit-circuitpython-bno055 adafruit-circuitpython-pca9685 adafruit-circuitpython-servokit opencv-python

WORKDIR /app
COPY colcon_ws/src /app/src
COPY colcon_ws/docker_entrypoint.sh /app

RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build

ENTRYPOINT ["bash", "/app/docker_entrypoint.sh" ]
