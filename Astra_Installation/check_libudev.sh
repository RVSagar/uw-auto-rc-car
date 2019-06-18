#!/bin/bash
echo "The following output should return some form of libudev.so.1"
ldconfig -p | grep libudev.so.1