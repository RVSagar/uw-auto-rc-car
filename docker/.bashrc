source /opt/ros/melodic/setup.bash
# source /workspace/OpenNi/OpenNI-Linux-x64-2.3/OpenNIDevEnvironment
source /home/${USER}/catkin_ws/devel/setup.bash
alias killros='killall -9 gazebo & killall -9 gzserver  & killall -9 gzclient & killall -9 roslaunch & killall -9 roscore & killall -9 rosmaster & killall -9 rviz & killall python & killall python2'

source /usr/share/gazebo/setup.sh
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/${USER}/catkin_ws/src/auto_rc_car_worlds/
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/${USER}/catkin_ws/src/auto_rc_car_worlds/media
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/${USER}/catkin_ws/src/auto_rc_car_worlds/media/materials

alias ls="ls --color=auto"
alias grep="grep --color=auto"
. /usr/share/bash-completion/bash_completion

echo ".bashrc sourced"
