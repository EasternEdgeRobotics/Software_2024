#!/bin/sh

. /opt/ros/humble/setup.sh

if [ -e /root/colcon_ws/install/setup.sh ]
then 
  . /root/colcon_ws/install/setup.sh
  ros2 launch beaumont_pkg beaumont_startup.xml
else
  cd /root/colcon_ws/
  colcon build
  . /root/colcon_ws/install/setup.sh
  ros2 launch beaumont_pkg beaumont_startup.xml
fi
  

