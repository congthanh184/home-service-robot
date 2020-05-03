#!/bin/sh

gnome-terminal -- roslaunch my_robot simple.launch &
sleep 5

gnome-terminal -- roslaunch my_robot amcl.launch &
sleep 5

gnome-terminal -- rosrun add_markers add_markers _env:=test &

