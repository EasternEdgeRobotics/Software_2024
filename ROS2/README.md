# Introduction
This ROS2 (Robotics Operating System 2) workspace contains the "Backend" of the software package, such as thruster math, profiles database, simulation environment, and tooling interface. It is meant to run on Beaumont's onboard Raspberry Pi 4.

The workspace uses the ROS2 "Humble" distribution.

You can follow the installation guide below to get ROS2 running and communicating with the GUI in a docker container on your own computer or a Raspberry Pi.

To run the simulation, navigate to the simulation install guide below.

## Docker Installation

### Step 1
Intall Docker at https://www.docker.com/get-started/ and get it running

### Step 2
Download the latest ROS2 Humble image at osrf/ros:latest using Docker desktop or the following command:
```
docker pull osrf/ros:humble-desktop
```
Then, run a docker container based on this image:
```
docker run -v "<PATH TO ROS2 directory on your computer>":/home/ROS2  -p 9090:9090 -it osrf/ros:humble-desktop
```
Above, we are mapping port 9090 of the container to the same port on the host. This is so that we can later listen for ROS2 on port 9090 in the GUI.
We are also mounting a volume using the -v. Any mapped folders between your host and the docker container will be shared. When you change the files in that 
folder on your computer, they will change in your docker container. 

To find out where the path to the ROS2 directory is on your computer, navigate to the ROS2 folder in Software 2024 in the terminal then type the following:
 ```
pwd
```
Copy and paste the result into the docker run command.

### Step 3
To open new terminals in the running container, type:
```
docker exec -it <CONTAINER_ID> bash
```
To find out your container id, type:
```
docker ps -a
```
Navigate to the /home/colcon_ws directory of this container and build the packages, sourcing the main ROS2 workspace first.
```
cd /home/colcon_ws
source /opt/ros/humble/setup.bash
colcon build
```

### Step 4
Finally, make sure to source this ros workspace in every new terminal window you open of this container.
```
source /home/colcon_ws/install/setup.bash
```
Alternatively, you can copy the above command into the bash.bashrc file, which is automatically run at every new terminal window.
When working with these containers, you can use VIM or nano as a file editor.
```
cd /etc
sudo nano bash.bashrc
```
Press CONTROL+C then Y then ENTER to exit and save the file using the nano editor.

### Step 5
To allow the running ROS2 instance of this container to communicate with the GUI, install rosbridge.
```
sudo apt install ros-humble-rosbridge-server
```
Then, run the launch file which starts the server.
```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```
Rosbridge starts litening on port 9090 by default. Run the GUI in your browser, both sides should indicate that they are connected.

### Step 6
Note that the Docker Installation will not be able to run the simulation environment (which requires a Beefy computer and a Gazebo install) or interface with any outside components. It will, however, contain the profiles database.

If you run into an issue with a dependancy in one of the Python script, you can install that specific library using pip
```
pip install <library name (ex. sqlalchemy)>
```
To run any script (referred to as a ROS node), ensure to source the workspace's setup.bash file as described above then run:
```
ros2 run beaumont_pkg <node name>
```
If you wanted to see a list of executable nodes, run:
```
ros2 pkg executables beaumont_pkg
```

## Raspberry Pi Installation
Note that this installation guide is not exclusive to Raspberry Pi's and can work for other Linux distros that support ROS2. If installing on an operating system such as Windows and Mac, steps will vary (refer to documentation).

### Step 1
Install a Linux Distro that supports ROS2 (such as Ubuntu). [A guide can be followed here](https://ubuntu.com/tutorials/how-to-install-ubuntu-on-your-raspberry-pi#1-overview). Ensure that you will have access to the terminal on the Pi, either through SSH on the network or by plugging in a mouse, keyboard, and monitor. Also, ensure the Pi has internet access.

### Step 2
Most Linux distros come with Python pre-installed. If this is not the case install Python and pip (pip allows you to further install Python libraries).
```
sudo apt update
sudo apt upgrade
sudo apt install python3
sudo apt install python3-pip
```

### Step 3
Install ROS2 Humble using [This tutorial](https://docs.ros.org/en/humble/Installation.html)

### Step 4
Move the ROS2 directory of the Software package into the Pi. You can either setup git and create an access token for your account in order to pull the code straight from the Software 2024 repository. Alternatively, you can opt for the easier route of using SCP (secure copy) to copy the directory into the Pi through the network.

On your own computer which contains the Software package, run the following command:
```
scp -r <PATH TO colcon_ws FOLDER ON YOUR COMPUTER> <RASPBERRY PI USERNAME>@<RASPBERRY PI IP ADDRESS>:/home/colcon_ws
```

Then, on the pi, run:
```
cd /home/colcon_ws
source /opt/ros/humble/setup.bash
colcon build
```

### Step 5
Follow steps 4-6 of the Docker Installation Section of this README

# Simulation Environment

The simulation environment is mainly for use in developing and testing code for any autonomus tasks (science tasks) in the MATE competition. It may also be used for testing features in the GUI or demo run practice.

## Explanation

The Open Source Robotics Foundation (osrf) resposible for writing ROS also created a simulation software called Gazebo. The version used for this simulation is Gazebo Classic (which has an end of life at 2025). It is to be superceeded by Modern Gazebo (foremally called Ignition).

Gazebo allows for creating robot models with visuals, collision, physics, plugins, and sensors. 

For visuals and collision, models can be made using the normal Gazebo model editor (which has powerful features such as mating), or they can be imported as STL. An open source custom Onshape API called [Onshape to Robot](https://onshape-to-robot.readthedocs.io/en/latest/) allows for downloading Onshape models as STL files (TODO: Experiment around trying to lower quality of models, currently causing a strong performance hit).

Plugins allow for the bot to listen to and be controlled by traditional ROS topics. Sensors grab information from the simulation environment and publish their data as ROS topics. This means that the ROS2 workspace can have "simulation" versions of nodes that control the actual Bot. These simulation nodes can listen to the same topics as the real ones, and the sensors can publish to the same topics aswell (or in the case of the cameras, interface with the GUI the same way as real sensors). This means that, in the point of view of the GUI and test code, there will be no difference between the simulation and the actual bot.

TODO:
- Implement Claw
- Setup a simulation version of mission tasks
- Look into the [simulated IMU sensor plugin](https://classic.gazebosim.org/tutorials?tut=ros_gzplugins)

## Installation

### Step 1
[Install Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install). Note that the version used for the simulation at the time of writing is Gazebo Classic 11.10.2

### Step 2
[Install the ROS2 gazebo_ros_pkgs](https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros). No custom plugins were written for this simulation, all plugins used are repurposed from here.

### Step 3
Navigate to the worlds folder of the ROS2 directory in the Software package and open a world
```
gazebo playground.world
```
Worlds may open with models already in them. Also, worlds can be edited on the fly then saved as a .world file. 

Note that you should source the ROS workspace (as described in the ROS installation guides) in order for the Bot in the simulation to listen to the ROS topics.

### Step 4
Navigate to the insert on the top left and click "Add Path". Navigate to the models folder of the ROS2 directory of the Software package and select it.

Then, select a model such as high_performance_caribou (currently, this is the only working model). Place this model somewhere in the world.

### Step 5
Launch the GUI in the browser. Ensure that the ROS IP in the settings tab is set to the IP of the machine running ROS (can be localhost).

### Step 6
In another window, ensure that the ROS workspace is sourced. Then run the following command:
```
ros2 launch beaumont_pkg simulation_beaumont_startup.xml
```
A launch file is simply a shortcut to running each required node indivisually. This launch file will start the rosbridge_server, the profiles_manager (controller mappings database), the simulation_thruster_controller, and the simulation_camera_subscriber

### Step 7
Ensure that there is a green checkmark next to ROS in the BotTab in the GUI. If it doesn't show green, disconnecting then reconnecting the computer running ROS from the network sometimes works. Once it is green, navigate to the settings tab.

For each of the 4 cameras, type "http://<IP ADDRESS OF MACHINE RUNNING ROS>:<EITHER 8880, 8881, 8882, or 8883>/cam.mjpg"

Plug in a controller and either pick or create a profile.

### Step 8
You should now see the simulated camera feed in the CameraTab. Note that the Bot will not move unless the power multipliers are set to non-zero in the BotTab.  

## What next?
Check the Trello board for the list of available tasks and don't hesitate to ask for guidance! 

Join the Trello board: https://trello.com/invite/b/aAfHeoZS/ATTI800afd588965f89371ccbbb2fee6b0153732980B/2024 

# IMU Setup for Raspberry Pi

To setup the BNO055 IMU, ensure that you've connected the BNO055 to the raspberry Pi using i2c. For this, search up your specific Pi's pinout (ex. google "Pi 3 model B pinout") and refer to the pin names on the BNO055 itself. To check if the IMU is being detected, run:

download i2c tools before this if not done already:
```
sudo apt-get update
sudo apt-get install i2c-tools
```
Then run the following command:
```
i2cdetect -y 1
```
You should see an address appearing for the BNO055 IMU board.

To read from the board using Python (as is done in this project), download the following dependencies:

Python if not done already:
```
sudo apt update
sudo apt install python3
```
Pip package manager
```
sudo apt install python3-pip
```
BNO055 Adafruit Python Library
```
pip3 install adafruit-circuitpython-bno055
```
And finally another dependancy that I noticed was needed while running Ubuntu Server on the Raspberry Pi:
```
pip3 install RPi.GPIO
```

Now, along with the installation + compilation of ROS 2 and the workspace in this repository, you should be able to see data from the IMU on the topic "/imu"
