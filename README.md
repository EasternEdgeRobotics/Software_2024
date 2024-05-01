# Introduction

Welcome to the Eastern Edge Robotics Software 2024. For a production installation guide, navigate to the Docker Installation section (recommended) or Host-Side Installation section. For a Simulation Environment install guide, navigate to the Simulation Section. 

### Frontend

The Software 2024 frontend application, located in the GUI folder, is built using the React Javascript Framework. The GUI folder contains a NodeJS (Node Javascript) workspace featuring, most notably, a package.json file which contains all the dependencies for running the development environment and generating a build/ folder for production.    

To run the Frontend as a development environment (non intended for production but functionally equivalent), [install NodeJS](https://nodejs.org/en/download). Then, navigate to the GUI directory and type the following two commands:
```
npm install
npm start 
```
Note that `npm install` will take up a few Gigabytes as all dependencies are downloaded. However, **all** downloaded files will appear in a folder called node_modules in the GUI directory, so it is easy to track and later delete to free up space. The large install is the only disadvantage with running the frontend using the development environment.  

### Backend

The ROS2 (Robotics Operating System 2) workspace, located in the ROS2 folder, contains the "Backend" of the software package. It handles things such as i2c communications to the various board components, thruster math, profiles database, autonomus mode, simulation environment, and more. It is meant to run on Beaumont's onboard Raspberry Pi 4.

The workspace uses the ROS2 Humble distribution. The development and production installations for the backend **are the same**.

### Camera Streamer

In production, camera feed from the main enclosure Raspberry Pi 4 as well as the 3 mini-enclosure Raspberry Pi Zero 2Ws is streamed over TCP/IP using MJPEG (motion jpeg). This is done using a Github project called [Spyglass](https://github.com/roamingthings/spyglass). To install the streamer on a Raspberry Pi, all that simply has to be done is copy over the Cameras directory and running the install.sh script inside.
```
. install.sh
```
This will configure everything and turn the camera streamer into a daemon service running in the background. To access the camera stream, the following can be typed in a browser: 
```
http://<Raspberry Pi ip>:<Port specified in the spyglass.conf file (defualt 8080)>/stream
```
### Simulator

The simulator is not part of the production environment is is meant to completely replace the physical bot while retaining all elements of the frontend and the majority of elements from the backend (minus i2c board communications for the actual bot). Navigate to the Simulation section below to learn more. 

# Production Installation Guides

## Software 2024 Docker Installation for Raspberry Pi (Recommended)

This guide will allow you to run the backend and frontend application on a Raspberry Pi or any computer running Debian or Ubuntu (as well as likely other Linux flavours).

### Step 1
If you are intending to run docker on a Raspberry Pi running Raspberry Pi OS (tested with Bookworm), copy over the Software 2024 directory using scp to the  type the following into the terminal.
```
. all_in_one_pi4_installer.sh
```
Installation is done, including backend, frontend, and the camera streamer. If this is a Raspberry Pi, it should now be ready to drive the bot provided access to an i2c bus with the correct components. 

## Software 2024 Docker Installation for Windows or Mac (Recommended for backend development on a non-linux machine)

### Step 1

[Intall Docker](https://www.docker.com/get-started/) 

### Step 2

Ensure the docker daemon is running. This may require having docker desktop running on windows.

To check if the daemon is running, type the following into a terminal:
```
docker 
```
This should return a bunch of possible commands assoicated with docker, indicating docker is running.

### Step 3

Navigate to Software_2024 (main directory) in the terminal. 

***Modify the compose.yaml file*** in the Software_2024 based on what you need. ***Do not proceed to step 4 without this because the compose.yaml is intended for Raspberry Pi.*** The file is commented to help with this. 

compose.yaml files are a popular way of defining docker applications because, once they are configured to the user's liking, they only require one command to setup the whole environment.

### Step 4

Run the following in a terminal window: 
```
docker compose up 
```

### General Info Regarding Docker

If you would like a rundown of how to use docker including the definition of containers and images, how docker compose files and Dockerfiles work, how to manage containers, images, volumes, and more, look at the following:

- [This really good and comprehensive guide](https://docker-curriculum.com/)
- Tutorials provided by docker desktop upon install

## Host-Side Installation

This guide will allow you to install the entire software package to production-level ***without docker***. This tutorial is intended for Ubuntu Linux, but may be modified for other Operating Systems. However, in theory, the backend should be able to run host-side on Windows, MacOS, and many linux distros. Refer to the documentation referenced in Step 2.

### Step 1

Ensure you have the Software 2024 repository on your machine.

### Step 2

[Install ROS2 humble (base or complete)](https://docs.ros.org/en/humble/Installation.html)

### Step 3

Navigate to the colcon_ws directory build the backend packages, sourcing the main ROS2 workspace first if this .

```
cd <Path to the Software 2024 repository on your computer>/ROS2/colcon_ws
source /opt/ros/humble/setup.bash
colcon build
```
Sourcing the ROS2 workspace allows your terminal instance to know that you have ROS2 installed, and that it is an executable. 

After a successful build, you should see no error messages and three new folders in colcon_ws (build, install, and log).

### Step 3

Make sure to source ROS2 and this ros workspace in every new terminal window you open.

```
source /opt/ros/humble/setup.bash
source /home/colcon_ws/install/setup.bash
```
Alternatively, you can copy the above command into the /etc/bash.bashrc file, which is automatically run at every new terminal window. 

### Step 4

To allow the running ROS2 instance of this container to communicate with the GUI, install rosbridge.

```
sudo apt install ros-humble-rosbridge-server
```

### Step 5

You should now be ready to launch the backend using the following command:
``
ros2 launch beaumont_pkg beaumont_startup.xml
```
This will run all required nodes. The backend is now fully running and ready to communicate with frontend without further setup.

### Step 6

To run the frontend, refer to the quick development environment installation guide in the Introduction. There is no need to run a production build of the frontend any place other than a docker container.

### Step 7

When you run the open the frontend in a browser window, ensure to put in the IP of the machine running the backend in the ROSIP section of the settings tab. 

# Simulation Environment

The simulation environment is mainly for use in developing and testing code for any autonomus tasks (science tasks) in the [MATE competition](https://20693798.fs1.hubspotusercontent-na1.net/hubfs/20693798/2024%20EXPLORER%20Manual%20FINAL_1_16_2024_with_Cover.pdf). It may also be used for testing features in the GUI or demo run practice.

## Explanation

The Open Source Robotics Foundation (osrf) resposible for writing ROS also created a simulation software called Gazebo. The version used for this simulation is Gazebo Classic (which has an end of life at 2025). It is to be superceeded by Modern Gazebo (used to be called called Ignition).

Gazebo allows for creating robot models with visuals, collision, physics, plugins, and sensors. 

For visuals and collision, models can be made using the normal Gazebo model editor (which has powerful features such as mating), or they can be imported as STL. An open-source custom Onshape API called [Onshape to Robot](https://onshape-to-robot.readthedocs.io/en/latest/) allows for conveniently downloading Onshape models as STL files. Alternatively, the STL files can be downloaded straight from onshape.

Plugins allow for the bot to listen to and be controlled by traditional ROS topics. Sensors grab information from the simulation environment and publish their data as ROS topics. This means that the ROS2 workspace can have "simulation" versions of the same nodes that control the actual Bot. These simulation nodes can listen to the same topics as the real ones, and the sensors can publish to the same topics as well (only sensors currently implemented are cameras, which actually create an MJPEG stream independent of ROS). 

This means that, in the point of view of the frontend and a large part of the backend (SQLite database in profiles_manager.py, autonomus brain coral transplant action node, autonomus coral reef modelling code), the simulation environment is the same as the actual bot. 

## Installation

If the simulation environment is already installed, begin at step 3.

### Step 1
Follow the Host-Side installation section.

### Step 2
[Install Gazebo Classic](https://classic.gazebosim.org/tutorials?tut=install_ubuntu&cat=install). Note that the version used for the simulation at the time of writing is Gazebo Classic 11.10.2

### Step 3
[Install the ROS2 gazebo_ros_pkgs](https://classic.gazebosim.org/tutorials?tut=ros2_installing&cat=connect_ros). No custom plugins were written for this simulation, all plugins used are repurposed from here.

### Step 4
Navigate to the worlds folder of the ROS2 directory in the Software package and open a world
```
gazebo competition_world
```
Worlds may open with models already in them. Also, worlds can be edited on the fly then saved as a .world file. 

Note that you should source the ROS workspace (as described in the ROS installation guides) in order for the Bot in the simulation to listen to the ROS topics.

### Step 5
Navigate to the insert on the top left and click "Add Path". Navigate to the models folder of the ROS2 directory of the Software package and select it (if it's not already there).

You should now see all of the models made by EER under the insert tab. 

### Step 5
Launch the GUI in the browser. Ensure that the ROS IP in the settings tab is set to the IP of the machine running ROS (localhost if it's the same machine).

### Step 6
In another window, ensure that the ROS workspace is sourced as per the step 1 tutorial. Then run the following command:
```
ros2 launch beaumont_pkg simulation_beaumont_startup.xml
```
A launch file is simply a shortcut to running each required node individually.

### Step 7
Ensure that there is a green checkmark next to ROS in the BotTab in the GUI. Once it is green, navigate to the settings tab.

For each of the 4 cameras, type "http://<IP ADDRESS OF MACHINE RUNNING ROS>:<EITHER 8880, 8881, 8882, or 8883>/cam.mjpg"

This may or may not already be saved and fetchable from the database.

Plug in a controller and either pick or create a profile.

### Step 8
You should now see the simulated camera feed in the CameraTab. Note that the Bot will not move unless the power multipliers are set to non-zero in the BotTab.  

