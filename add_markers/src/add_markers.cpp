#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(0.2);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  int cycle_count = 1;
  
  while (ros::ok())
  {
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

    // Cycle process
    int marker_pose_x = 0;
    int marker_pose_y = 0;
  
    switch (cycle_count)
    {
    case 1:
      // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
      marker.action = visualization_msgs::Marker::ADD;
      marker_pose_x = 2;
      marker_pose_y = 3; 
      cycle_count = cycle_count+1;
      ROS_INFO("Set the 1st marker");
      break;
    case 2:
      marker.action = visualization_msgs::Marker::DELETE;
      //      marker_pose_x = 1;
      //      marker_pose_y = 2;
      cycle_count = cycle_count+1;
      ROS_INFO("Delete the 1st marker");
      break;
    case 3:
      marker.action = visualization_msgs::Marker::ADD;
      marker_pose_x = 4;
      marker_pose_y = 8;
      cycle_count = cycle_count+1;
      ROS_INFO("Set the 2nd marker");
      break;
    case 4:
      marker.action = visualization_msgs::Marker::DELETE;
      //      marker_pose_x = 1;
      //      marker_pose_y = 2;
      cycle_count = cycle_count+1;
      ROS_INFO("Delete the 2nd marker");
      break;
    default:
      return 0;
      
    }

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = marker_pose_x;
    marker.pose.position.y = marker_pose_y;
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

#if 1
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
#endif
    // Publish
    marker_pub.publish(marker);
    ROS_INFO("Publish the marker");
    // Wait for a few second
    sleep(5);

#if 0
    // Cycle between different shapes
    switch (shape)
    {
    case visualization_msgs::Marker::CUBE:
      shape = visualization_msgs::Marker::SPHERE;
      break;
    case visualization_msgs::Marker::SPHERE:
      shape = visualization_msgs::Marker::ARROW;
      break;
    case visualization_msgs::Marker::ARROW:
      shape = visualization_msgs::Marker::CYLINDER;
      break;
    case visualization_msgs::Marker::CYLINDER:
      shape = visualization_msgs::Marker::CUBE;
      break;
    }
#endif
    
    r.sleep();
  }
}
