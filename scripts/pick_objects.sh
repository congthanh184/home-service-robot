#!/bin/sh

xterm -e " roslaunch my_robot simple.launch " &
sleep 5

xterm -e " roslaunch my_robot amcl.launch " &
sleep 5

xterm -e " rosrun pick_objects simple_navigation_goals " &

