# ROS 2
## Setup
```
source /opt/ros/humble/setup.bash
```
```
rosdep install -i --from-path src --rosdistro humble -y
```
```
colcon build
```
```
source install/setup.bash
```