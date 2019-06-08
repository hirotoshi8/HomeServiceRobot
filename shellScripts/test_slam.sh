#!/bin/sh
xterm -e " gazebo" &
sleep 5
xterm -e " source /opt/ros/kinetic/setup.bash; roscore" &
sleep 5

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch worldfile:=($ my_robot)/worlds/my_world.world"

sleep 3
xterm " roslaunch turtlebot_gazebo gmapping.launch"

sleep 3
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch"

sleep 3
xterm " roslaunch turtlebot_teleop keyboard_teleop.launch"

