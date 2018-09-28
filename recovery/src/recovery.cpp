#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Bool.h"

#include<iostream>
#include<iostream>


bool obstacle = 0;

void scanCallback(const sensor_msgs::LaserScan &msg)
{
  float sc_time = msg.scan_time;
//  ROS_INFO("\nscan time: %f\n", sc_time);
  std::cout << "range_min: " << msg.range_min << std::endl;
}
void is_stuckedCall(const std_msgs::Bool &info)
{
  obstacle = info;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "recovery");
  ros::NodeHandle n;

  ros::Publisher drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 50);
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  ros::Subscriber obstacle_sub = n.subscribe("/is_stucked",, 1000, is_stuckedCall);


  ackermann_msgs::AckermannDriveStamped drive_slow;
  drive_msg.drive.speed = 0.1;
  drive_msg.drive.acceleration = 0;
  drive_msg.drive.jerk = 0;
  drive_msg.drive.steering_angle = 0;
  drive_msg.drive.steering_angle_velocity = 0;

  ackermann_msgs::AckermannDriveStamped stop;
  drive_msg.drive.speed = 0;
  drive_msg.drive.acceleration = 0;
  drive_msg.drive.jerk = 0;
  drive_msg.drive.steering_angle = 0;
  drive_msg.drive.steering_angle_velocity = 0;
  while(ros::ok())
  {
    if(obstacle == 0)
    {
      drive_pub.publish(drive_msg);
    }
    if(obstacle == 1)
    {
      drive_pub.publish(stop);
    }
  }
  ros::spin();
  return 0;
}
