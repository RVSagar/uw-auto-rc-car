#!/bin/bash

if [ -z "$1" ]
then
      echo "Please call with the MASTER URI as an argument: ./set_as_client.sh xxx.yyy.zzz.www"
else
fi

export ROS_MASTER_URI=http://$1:11311
export ROS_IP=0.0.0.0