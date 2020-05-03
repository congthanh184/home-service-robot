#!/bin/sh

gnome-terminal -- roslaunch my_robot simple.launch &
sleep 5

gnome-terminal -- roslaunch my_robot teleop.launch &
sleep 5

gnome-terminal -- roslaunch my_robot mapping.launch &
sleep 5

