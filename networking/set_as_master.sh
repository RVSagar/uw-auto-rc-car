#!/bin/bash
LIST=(`hostname -I`)
MY_IP=${LIST[0]}
echo "My IP is: $MY_IP"
export ROS_MASTER_URI=http://$MY_IP:11311
export ROS_IP=0.0.0.0