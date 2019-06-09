# Home Service Robot project

## Overview
This project is the "Home Service Robot" project in Udacity Robotics nano degree program.

This project is developed with ROS and Gazebo.
A autonomous robot go to the first destination for picking up a object, which is green marker and after a few second, go to the next destination for dropping it off.

# Autonomous robot algorithm explanation
## Creating the MAP for localization
- For autonomous driving, it's neccessary to localize the position(and pose) of the robot and goals in the world. For this localization, creating MAP is important.
- In this project, I use gmapping package in ROS for making MAP.
- This package is based on the Grid-based Fast SLAM algorithm using the particle filter and occupancy grid mapping.

## Localization with MAP
- In this project, I use AMCL package in ROS for the localization.
- This package is the probabilistic localization sysytem based on the Monte Carlo lozalization, using particle filter to track the robot positon and pose against a known MAP.
- This function is used through the turtlebot package.

## Navigation
- In this project, I use move_base package in ROS as Navigation stack.
- This package is based on Dijkstra's algorithm, a variety of the uniform cost search algorithm.
- Especially, this function is used by "pick_objects" node in this project.

# Build instruction
## 1. Create workspace 
```
$mkdir -p {$ your workspace}/project_workspace/src
```

## 2. Initial setting for catkin_workspace
```
$cd {$ your workspace}/project_workspace/src
$catkin_init_workspace
$cd {$ your workspace}/project_workspace
$catkin_make
```

## 3. Pre-requistics (Clone ROS official packages) 

### 3-1. gmapping   (for creating MAP)
```
$cd {$ your workspace}/project_workspace/src
$git clone https://github.com/ros-perception/slam_gmapping
$rosdep install gmapping
$cd ..
$cakin_make
``` 

###  3-2. turtlebot( including AMCL)   
you need some packages about turtlebot below
- turtlebot_teleop
- turtlebot_rviz_launchers
- turtlebot_gazebo
```
$cd {$ your workspace}/project_workspace/src
$git clone https://github.com/turtlebot/turtlebot.git
$git clone https://github.com/turtlebot/turtlebot_interactions.git
$git clone https://github.com/turtlebot/turtlebot_simulator.git
```

### 4. Clone this project repository
```
$cd {$ your workspace}/project_workspace/src
$git clone {$ this_repositpry}.git
```

### 5. Build
```
$cd {$ your workspace}/project_workspace
$catkin_make
```

# Simulation Instruction
- For simulation of the home service robot demo,
you do the command bellow.
-  Home service robot demo is launched with Gazebo, Rviz and the other application nodes.
```
$cd {$ your workspace}/project_workspace/src/HomeServiceRobot/src/shellScripts
$./home_service.sh
```

# Test Simulation
- For making this project, I make the some test simulation sets as shell scripts.

1. Test SLAM to create the map with gmapping
```
$./test_slam.sh
```

2. Test Navigation with AMCL
```
$./test_navigation.sh
```

3. Test for the setting some markers in Rviz
```
$./add_markers.sh
```

4. Test for the picking up and dropping off the target
```
$./pick_objects.sh
```
