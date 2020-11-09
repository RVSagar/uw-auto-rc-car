#!/bin/bash

export LD_LIBRARY_PATH=/opt/ros/melodic/lib:/usr/local/cuda/lib64
export DISPLAY=:3
Xvfb $DISPLAY -screen 0 1880x1040x24 &
xfce4-session &
mkdir ~/.vnc
x11vnc -storepasswd 'ideas' ~/.vnc/passwd
x11vnc -forever -shared -repeat -usepw -display $DISPLAY -rfbport 6080 -noxrecord -noxdamage -xkb &

bash
