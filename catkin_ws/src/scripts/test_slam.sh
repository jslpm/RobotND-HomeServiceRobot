#!/bin/sh
xterm -e "roslaunch my_robot  world.launch" &
sleep 5
xterm -e "roslaunch my_robot teleop.launch" &
sleep 5
xterm -e "roslaunch gmapping slam_gmapping_pr2.launch" &
sleep 5
xterm  -e "roslaunch turtlebot_rviz_launchers view_navigation.launch"