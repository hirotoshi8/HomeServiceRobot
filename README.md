# Where I am project

## Overview
This project is the "Where I am" project in Udacity Robotics nano degree program.

This project is developed with ROS and Gazebo.
A robot localizes self-position in Map(Gazebo environment).     
You set a navigation goal as "2D NavGoal" in Rviz and the robot move to the goal with identify self-positon.

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
