#!/bin/bash

echo "ifconfig results:"
ifconfig | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1'


PUBLIC_IP=$(dig @resolver1.opendns.com ANY myip.opendns.com +short -4)
echo "Using dig, my public IP is: ${PUBLIC_IP}"