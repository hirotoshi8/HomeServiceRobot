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

    const int white_pixel = 255;

    float vel = 0.0f;
    float omega = 0.0f;
    
    int left_threshold = img.step*1/3;
    int right_threshold = img.step*2/3;
    
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    int img_size = img.height * img.step;
    for(int i=0; i < img_size; i++){

      int color = img.data[i];

      // is this white ball ?
      if(white_pixel != color) continue;

      int position = i % img.height;

      // is white ball face-to-face ?
      if( (left_threshold < position) && (position < right_threshold) ){
	vel = 1.0f;
	omega = 0.0f;
	break;
      }
      // is white ball in the left side ?
      if( position < left_threshold){
	vel = 0.2f;
	omega = 0.5f;
	break;
      }
      // is white ball in the right side ?
      if( right_threshold < position){
	vel = 0.2f;
	omega = 0-.5f;
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

