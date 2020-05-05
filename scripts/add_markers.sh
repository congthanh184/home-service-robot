#!/bin/sh

xterm -e " roslaunch my_robot simple.launch " &
sleep 5

xterm -e " rosrun add_markers add_markers _env:=test " &
sleep 1

xterm -e " roslaunch my_robot amcl.launch "

