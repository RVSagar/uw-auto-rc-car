#!/bin/bash

if [ -z "$1" ]
then
      echo "Please call with the MASTER URI as an argument: ./set_as_master.sh xxx.yyy.zzz.www"
      echo "Use list_ips.sh to find a list of ip addresses"
      return 0
fi

MY_IP=$1
echo "My IP is: $MY_IP"

export ROS_MASTER_URI=http://$MY_IP:11311

echo "Setting ROS_IP to $MY_IP"
export ROS_IP=$MY_IP # Own IP

HOSTNAME=$(host $MY_IP) # Get info on host
HOSTNAME=$(echo $HOSTNAME | awk 'NF{ print $NF }') # Need last word on line
HOSTNAME=${HOSTNAME::-1} # Remove trailing period
echo "Setting ROS_HOSTNAME to ${HOSTNAME}"

export ROS_HOSTNAME=$HOSTNAME #It's own name
