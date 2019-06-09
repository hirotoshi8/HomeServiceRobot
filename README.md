# Home Service Robot project

## Overview
This project is the "Home Service Robot" project in Udacity Robotics nano degree program.

This project is developed with ROS and Gazebo.
A self-driving robot go to the first destination for picking up the target, which is green marker and pick it up for a few second and then go to the next target for dropping it off.



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

### 3-1. gmapping
```
$cd {$ your workspace}/project_workspace/src
$git clone https://github.com/ros-perception/slam_gmapping
$rosdep install gmapping
$cd ..
$cakin_make
``` 

###  3-2. turtlebot   
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
