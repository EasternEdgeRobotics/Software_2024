FROM arm64v8/ros:humble-ros-base-jammy

WORKDIR /root/colcon_ws

RUN apt-get update -y
RUN apt-get upgrade -y

# Install Python and PIP
RUN apt-get install python3 -y
RUN apt-get install python3-pip -y

# Install rosbridge server to allow ROS instance to communicate with frontend
RUN apt-get install ros-humble-rosbridge-server -y

# Install any required Python libraries
RUN pip3 install sqlalchemy 
RUN pip3 install RPi.GPIO
RUN pip3 install smbus2
RUN pip3 install adafruit-circuitpython-pca9685
RUN pip3 install adafruit-circuitpython-bno055 

# Modify the bash.bashrc file such that sourcing happens automatically 
RUN echo ". /opt/ros/humble/setup.sh" >> /etc/bash.bashrc
RUN echo ". /root/colcon_ws/install/setup.sh" >> /etc/bash.bashrc

# Launch ROS on startup OR colcon build again if needed (see docker_entrypoint.sh script in colcon_ws)
ENTRYPOINT ["/root/colcon_ws/docker_entrypoint.sh" ]