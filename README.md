# Map my world project

## Overview
This project is the "Map my world" project in Udacity Robotics nano degree program.

This project is developed with ROS and Gazebo.
A robot create a map for navigation as database file(Gazebo environment).     
You control a robot with teleoperation, map the world in gazebo environment and create a map as database file.


## Build instruction
1. Create workspace 

```
$mkdir -p {$ your workspace}project_workspace/src
```

2. Initial setting for catkin_workspace
```
$cd {$ your workspace}/project_workspace/src
$catkin_init_workspace
$cd {$ your workspace}/project_workspace
$catkin_make
```

<<<<<<< HEAD
3.Clone official packages
- git clone https://github.com/ros-perception/slam_gmapping
- rosdep install gmapping
- cd ..
- cakin_make
=======

>>>>>>> 98a9471d598fb6a651dd7a1df7cc246d78cd1782

3. clone this repository
```
$cd {$ your workspace}/project_workspace/src
$git clone {$ this_repositpry}.git
```

4. Build
```
$cd {$ your workspace}/project_workspace
$catkin_make
```

## Simulation Instruction
1. Launch the gazebo world
```
$roslaunch my_robot world.launch
```

2. Launch manual controller with teleoperaion
```
$roslaunch my_robot teleop.launch
```

3. Launch mapping node
```
$roslaunch my_robot mappping.launch
```

4. You cnotrol the robot with teleoperaion and map the world in gazebo world.

## Result
- As sample of the result, I save the database file below
