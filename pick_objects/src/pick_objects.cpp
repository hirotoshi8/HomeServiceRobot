#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_object");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();


  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending 1st goal in Pick-up zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    //ROS_INFO("Hooray, the base moved 1 meter forward");
    ROS_INFO("Successfully to reach 1st Goalin Pich-up zone");
  }else{
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }

  // Wait next action
  ros::Rate rate(10);
  ros::Time start = ros::Time::now();
  ros::Time end = ros::Time::now();
  while(ros::ok()){
    end = ros::Time::now();
    if((start - end) > ros::Duration(5.0f)){
      break;
    }
    ros::spinOnce();
  }

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.orientation.w = 0.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending 2nd goal in Drop-Off zone");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
    //ROS_INFO("Hooray, the base moved 1 meter forward");
    ROS_INFO("Successfully to reach 2nd Goal in Drop-Off zone");
  }else{
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }
  
  return 0;
}
