#!/bin/bash
LIST=(`hostname -I`)
echo "Full hostname IP list: $LIST"

hostip=$(ip route show | awk '/default/ {print $3}')
MY_IP=$(echo $hostip | awk '{print $1;}')
echo "My IP is: $MY_IP"

export ROS_MASTER_URI=http://$MY_IP:11311

echo "Setting ROS_IP to $MY_IP"
export ROS_IP=$MY_IP # Own IP

HOSTNAME=$(host $MY_IP)
echo "Setting ROS_HOSTNAME to ${HOSTNAME}"

export ROS_HOSTNAME=$HOSTNAME #It's own name
