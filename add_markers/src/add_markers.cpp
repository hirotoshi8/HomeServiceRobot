#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

static int is_arrive_goal = false;
static float goal_pos_x;
static float goal_pos_y;

const float FirstGoalPos_x = 1.0;
const float FirstGoalPos_y = 2.0;

const float SecondGoalPos_x = 5.0;
const float SecondGoalPos_y = 2.0;

const float ThreGoalArrive = 0.5;

static int cycle_count = 1;



//void callback_odometry(const nav_msgs::Odometry::ConstPtr& msg){
void callback_odometry(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
  float pos_x = msg->pose.pose.position.x;
  float pos_y = msg->pose.pose.position.y;
  //  ROS_INFO("Positoin: x[%f], y[%f]", pos_x, pos_y);

  // Detect the robot is arrived at goals
  float diff_x = fabs(goal_pos_x - pos_x);
  float diff_y = fabs(goal_pos_y - pos_y);

  //  ROS_INFO("Positoin: diff_x[%f], diff_y[%f]", diff_x, diff_y);

  float distance = diff_x*diff_x + diff_y*diff_y;
  ROS_INFO("distance: dist[%f]", distance);
  
  if(distance < ThreGoalArrive){
    ROS_INFO("!!!!!!!!!! Gaol !!!!!!!!!!");
    is_arrive_goal = true;
  }
  return;
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  //  ros::Subscriber odom_sub = n.subscribe("/odom", 11, callback_odometry);
  ros::Subscriber odom_sub = n.subscribe("/amcl_pose", 1, callback_odometry);
    
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;


  while(ros::ok()){
  
    visualization_msgs::Marker marker;

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the Timer for the publish data
    marker.header.stamp = ros::Time::now();

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

  
    // Main markers state
    switch (cycle_count)
      {
	// Initial Goal setting as 1st Goal
      case 1:

	ROS_INFO("Set the 1st marker");

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;
	goal_pos_x = FirstGoalPos_x;
	goal_pos_y = FirstGoalPos_y;

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = goal_pos_x;
	marker.pose.position.y = goal_pos_y;

	// Publish the marker
	while (marker_pub.getNumSubscribers() < 1){
	  if (!ros::ok()){
	    ROS_INFO("case1");
	    return 0;
	  }
	  ROS_WARN_ONCE("Please create a subscriber to the marker");
	  sleep(1);
	}
	// publish
	marker_pub.publish(marker);
      
	// update the marker state
	cycle_count = cycle_count + 1;
	break;

	// Wait for arriving at 1st goal
      case 2:
	// Process when the robot arrives at the goals
	if(is_arrive_goal == true){
	  ROS_INFO("Delete the 1st marker");
	  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	  marker.action = visualization_msgs::Marker::DELETE;

	  // Publish the marker
	  while (marker_pub.getNumSubscribers() < 1){
	    if (!ros::ok()){
	      ROS_INFO("case1");
	      return 0;
	    }
	    ROS_WARN_ONCE("Please create a subscriber to the marker");
	    sleep(1);
	  }
	  // publish
	  marker_pub.publish(marker);

	  // Reset
	  is_arrive_goal = false;

	  // Update the Goal Positoin(1st -> 2nd)
	  goal_pos_x = SecondGoalPos_x;
	  goal_pos_y = SecondGoalPos_y;

	  // update the marker state
	  cycle_count = cycle_count + 1;
	
	}else{
	  ROS_INFO("Waiting for 1st Goal");
	}
	break;

	// Wait
      case 3:
	ROS_INFO("Wait for the pick up");
	sleep(5);      
	// update the marker state
	cycle_count = cycle_count + 1;

	break;

	// Waiting for arriving at 2nd Goal and set the marker
      case 4:

	if(is_arrive_goal == true){
	  ROS_INFO("Set the 2nd marker");

	  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	  marker.action = visualization_msgs::Marker::ADD;
	  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	  marker.pose.position.x = goal_pos_x;
	  marker.pose.position.y = goal_pos_y;

	  // Publish the marker
	  while (marker_pub.getNumSubscribers() < 1){
	    if (!ros::ok()){
	      ROS_INFO("case1");
	      return 0;
	    }
	    ROS_WARN_ONCE("Please create a subscriber to the marker");
	    sleep(1);
	  }
	  // publish
	  marker_pub.publish(marker);

	  // update the marker state
	  cycle_count = cycle_count + 1;
	}else{
	  ROS_INFO("Waiting for 2nd Goal");
	}
	break;
      
	// After completing marker task
      case 5:
	ROS_INFO("Sleep");
	sleep(1);
#if 0
	ROS_INFO("Delete the 2nd marker");
	marker.action = visualization_msgs::Marker::DELETE;

	// update the marker state
	cycle_count = cycle_count + 1;
#endif
	break;

      default:
	ROS_INFO("case2");
	return 0; 
      }
    
    // wait for the Odometry as Triger to change the goal positoin
    ros::spinOnce();
    loop_rate.sleep();
  }
}
