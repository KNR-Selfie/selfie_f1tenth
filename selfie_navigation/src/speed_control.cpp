#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "ros/console.h"

float max_speed = 0;
float min_speed = 1;

void speedCallback(const ackermann_msgs::AckermannDriveStamped &msg )
{
  float actual_speed = msg.drive.speed;

  if(actual_speed > max_speed)
    max_speed = actual_speed;
  if(actual_speed < min_speed)
      min_speed = actual_speed;
  ROS_INFO("\nSpeed = %f\n", msg.drive.speed);
}

int main(int argc, char **argv)
{
  ROS_INFO("entered main \n");
  ros::init(argc, argv, "speed_control");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("drive", 100, speedCallback);


  ros::spin();

  if(ros::isShuttingDown())
  {
    std::printf("\nmax speed = %f \tmin_speed = %f\n", max_speed, min_speed);
    std::printf("closing speed_control\n");
  }

  return 0;
}
