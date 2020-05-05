#!/bin/sh

xterm -e " roslaunch my_robot simple.launch " &
sleep 5

xterm -e " roslaunch my_robot teleop.launch " &
sleep 5

xterm -e " roslaunch my_robot mapping.launch " &
sleep 5

