#!/bin/sh
xterm -e " source /opt/ros/kinetic/setup.bash; source /home/workspace/catkin_ws/devel/setup.bash; roscore" &
sleep 5

#xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/workspace/catkin_ws/src/HomeServiceRobot/my_robot/worlds/my_world.world" &
xterm -e " roslaunch my_robot world.launch base_frabe:=robot_footprint odom_frame:=/odom"  &
sleep 5

xterm -e " rosrun gmapping slam_gmapping " &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " rosrun teleop_twist_keyboard teleop_twist_keyboard.py"

