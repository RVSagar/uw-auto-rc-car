#!/bin/bash

if [ -z "$1" ]
then
      echo "Please call with remote name as argument"
fi

IP=$(dig +short $1)
echo "IP of remote ($1) is ${IP}"