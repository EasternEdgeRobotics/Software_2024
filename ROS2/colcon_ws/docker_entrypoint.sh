source /opt/ros/humble/setup.sh

if [ -e /app/install/setup.sh ]
then 
  source /app/install/setup.sh
  ros2 launch beaumont_pkg beaumont_startup.xml
else
  cd /app
  colcon build
  source /app/install/setup.sh
  ros2 launch beaumont_pkg beaumont_startup.xml
fi
