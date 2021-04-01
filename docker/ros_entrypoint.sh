#!/bin/bash
set -e

source /opt/ros/melodic/setup.bash

cd /root/catkin_ws
rosdep install --from-paths src -r -y
# catkin build
catkin_make

# setup ros environment
source "/opt/ros/melodic/setup.bash"
source "/root/catkin_ws/devel/setup.bash"

echo "------------- PWCNet Action Server Ready ----------------------------"

exec "$@"
