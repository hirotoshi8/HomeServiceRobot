#!/bin/sh
xterm -e " source /opt/ros/kinetic/setup.bash; source /home/workspace/catkin_ws/devel/setup.bash; roscore" &
sleep 5

xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/HomeServiceRobot/my_robot/worlds/my_world.world" &
sleep 5

xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/workspace/catkin_ws/src/HomeServiceRobot/my_robot/maps/my_world.yaml" &
sleep 5

xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5
xterm -e " rosrun pick_objects pick_objects"

