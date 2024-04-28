sudo apt-get update
sudo apt-get upgrade -y
cd ~/Software_2024

# Convert docker_entrypoint.sh (for backend) to unix to fix docker erroring out because of windows (dos-style) line endings
sudo apt-get install dos2unix -y
sudo dos2unix ROS2/colcon_ws/docker_entrypoint.sh
sudo chmod +x ROS2/colcon_ws/docker_entrypoint.sh

# Install docker
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Start up and frontend and backend containers as a daemon (background) recurring process
sudo docker compose up -d 

# Install and setup the camera streamer
cd Cameras
. install.sh

# Get back to the Software_2024 folder
cd ..