#!/bin/bash
LIST=(`hostname -I`)
MY_IP=${LIST[0]}
echo "My IP is: $MY_IP"
export ROS_MASTER_URI=http://$MY_IP:11311
export ROS_IP=$MY_IP # Own IP
export ROS_HOSTNAME=robotlab22.eng.uwaterloo.ca #It's own name

hostip=$(ip route show | awk '/default/ {print $3}')
echo $hostip