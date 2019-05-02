#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
  // TODO: Request a service and pass the velocities to it to drive the robot
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  // Call the drive to target service and pass the requested command
  if(!client.call(srv)){
    ROS_ERROR("Failed to call service");
  }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    const int white_pixel_threshold = 200;

    float vel = 0.0f;
    float omega = 0.0f;
    
    int left_threshold = img.step*1/4;
    int right_threshold = img.step*3/4;
    
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int img_size = img.height * img.step;
    const int num_channel = 3;
    
    for(int i=0; i < img_size; i+=num_channel){

      int r_color = img.data[i];
      int g_color = img.data[i+1];
      int b_color = img.data[i+2];
      
      // is this white ball ?
      if( (white_pixel_threshold > r_color)
	  || (white_pixel_threshold > g_color)
	  || (white_pixel_threshold > b_color) ) continue;

      //Debug if it success to get white ball, all channels values are near 255
      ROS_INFO("%d",r_color);
      ROS_INFO("%d",g_color);
      ROS_INFO("%d",b_color);

      // Get the positoin og white ball in the image
      int position = i % img.step;

      // is white ball face-to-face ?
      if( (left_threshold < position) && (position < right_threshold) ){
	vel = 0.5f;
	omega = 0.0f;
	ROS_INFO("========== Straight ==========");
	break;
      }
      // is white ball in the left side ?
      if( position < left_threshold){
	vel = 0.01f;
	omega = 0.2f;
	ROS_INFO("---------- Left side ----------");
	break;
      }
      // is white ball in the right side ?
      if( right_threshold < position){
	vel = 0.01f;
	omega = -0.2f;
	ROS_INFO("********** Right side **********");
	break;
      }
    }

    // Send command as a service
    drive_robot(vel, omega);  
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}

