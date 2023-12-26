# ROS Workspace
This workspace will be loaded into a docker container that runs ROS2 Humble and is meant to be run on Beaumont's onboard computing unit


For development purposes, you can follow the installation guide below to get ROS2 running and communicating with the GUI in your computer's docker container

## Installation

### Step 1
Intall Docker at https://www.docker.com/get-started/ and get it running

### Step 2
Create a volume in Docker, which is a file directory in your computer that your docker container can access.
```
docker volume create colcon_ws
```
Copy the contents on the colcon_ws folder into the _data directory of the new volume. To find the docker volume in windows, type the following into File Explorer:
```
\\wsl$\docker-desktop-data\data\docker\volumes
```

### Step 3
Download the latest ROS2 Humble image at osrf/ros:latest using Docker desktop or the following command:
```
docker pull osrf/ros:humble-desktop-full
```
Then, run a docker container based on this image:
```
docker run -p 9090:9090 --mount source=colcon_ws,destination=/home/colcon_ws -it osrf/ros:humble-desktop-full
```
Above, we are mapping port 9090 of the container to the same port on the host. This is so that we can later listen for ROS2 on port 9090 in the GUI.

### Step 4
To open new terminals in the running container, type:
```
docker exec -it <CONTAINER_ID> bash
```
Navigate to the /home/colcon_ws directory of this container and build the packages.
```
cd /home/colcon_ws
colcon build
```

### Step 5
Finally, make sure to source this ros workspace in every new terminal window you open of this container.
```
source /home/colcon_ws/install/setup.bash
```
Alternatively, you can copy the above command into the bash.bashrc file, which is automatically run at every new terminal window.
When working with these containers, you can use VIM as a file editor.
```
sudo apt-get update
sudo apt-get install vim
cd /etc
vim bash.bashrc
```
Press insert on the keyboard to enter edit mode. When done, press escape, type :wq, and press enter. 

### Step 6
To allow the running ROS2 instance of this container to communicate with the GUI, install rosbridge.
```
sudo apt install ros-humble-rosbridge-server
```
Then, run the launch file which starts the server.
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
Rosbridge starts litening on port 9090 by default. Run the GUI in your browser, both sides should indicate that they are connected.

## What next?
Check the Trello board for the list of available tasks and don't hesitate to ask for guidance! 

Join the Trello board: https://trello.com/invite/b/aAfHeoZS/ATTI800afd588965f89371ccbbb2fee6b0153732980B/2024 