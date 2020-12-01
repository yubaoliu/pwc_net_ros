cd /root/catkin_ws

source /opt/ros/melodic/setup.bash
rosdep install --from-paths src -r -y

source /opt/ros/melodic/setup.bash
catkin build

source "/opt/ros/melodic/setup.bash"
source "/root/catkin_ws/devel/setup.bash"
