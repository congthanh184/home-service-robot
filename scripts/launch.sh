#!/bin/sh

gnome-terminal -- gazebo  &
sleep 5

gnome-terminal -- roscore &
sleep 5

gnome-terminal -- rosrun rviz rviz
