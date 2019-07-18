#!/bin/bash

if [ -z "$1" ]
then
      echo "Please call with the MASTER URI as an argument: ./set_as_client.sh xxx.yyy.zzz.www"
      echo "Use IP used as ROS_IP when running set_as_master.sh on master"
      return 0
fi

MASTER_IP=$1
echo "Using ROS_MASTER_URI -> ${MASTER_IP}"
export ROS_MASTER_URI=http://$MASTER_IP:11311

LIST=(`hostname -I`)
echo "Full hostname IP list: $LIST"
MY_IP=${LIST[0]}
echo "My IP is ${MY_IP}"
echo "My hostname is $(host $MY_IP)"
HOSTNAME=$MY_IP

echo "Setting ROS_IP=${MY_IP}"
export ROS_IP=$MY_IP

echo "Setting ROS_HOSTNAME=${HOSTNAME}"
export ROS_HOSTNAME=$HOSTNAME
