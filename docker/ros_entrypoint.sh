#!/bin/bash
set -e

# source /opt/ros/melodic/setup.bash
# catkin build

# setup ros environment
source "/opt/ros/melodic/setup.bash"
# source "/root/catkin_ws/devel/setup.bash"
exec "$@"
