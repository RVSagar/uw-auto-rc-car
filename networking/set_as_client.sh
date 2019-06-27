#!/bin/bash

if [ -z "$1" ]
then
      echo "Please call with the MASTER URI as an argument: ./set_as_client.sh xxx.yyy.zzz.www"
      #return
fi

#MASTER_IP=$1
MASTER_IP=129.97.229.135
export ROS_MASTER_URI=http://$MASTER_IP:11311
#export ROS_IP=172.27.0.1

LIST=(`hostname -I`)
MY_IP=${LIST[0]}

MY_IP=172.17.0.1
#Y_IP=129.97.229.135

echo "Setting ROS_IP=${MY_IP}"
export ROS_IP=$MY_IP
export ROS_HOSTNAME=$MY_IP
