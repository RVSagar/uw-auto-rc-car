#!/bin/bash

export DISPLAY=:3
Xvfb $DISPLAY -screen 0 1880x1040x24 &
xfce4-session &
mkdir ~/.vnc
x11vnc -storepasswd ideas ~/.vnc/passwd
x11vnc -forever -shared -repeat -display $DISPLAY -usepw &

bash
