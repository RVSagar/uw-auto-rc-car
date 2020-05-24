source /opt/ros/kinetic/setup.bash
source /workspace/OpenNi/OpenNI-Linux-x64-2.3/OpenNIDevEnvironment
source /home/${USER}/catkin_ws/devel/setup.bash
alias killros='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roslaunch & killall -9 roscore & killall -9 rosmaster & killall -9 rviz'

source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/wesley/catkin_ws/src/auto_rc_car_worlds/

echo ".bashrc sourced"