#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "std_msgs/Bool.h"

#include<iostream>
#include<iostream>
#include<time.h>

#define SLEEP_TIME 1
#define DRIVE_SPEED 0.1

bool obstacle = 0;


void scanCallback(const sensor_msgs::LaserScan &);
void is_stuckedCall(const std_msgs::Bool &);
void dodge_obstacle();
void go_back(double distance);
void go_forward(double distance);



int main(int argc, char **argv)
{
  ros::init(argc, argv, "recovery");
  ros::NodeHandle n;

  ros::Publisher drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 50);
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  ros::Subscriber obstacle_sub = n.subscribe("/is_stucked",, 1000, is_stuckedCall);


  ackermann_msgs::AckermannDriveStamped drive_slow;
  drive_slow.drive.speed = 0.1;
  drive_slow.drive.acceleration = 0;
  drive_slow.drive.jerk = 0;
  drive_slow.drive.steering_angle = 0;
  drive_slow.drive.steering_angle_velocity = 0;

  ackermann_msgs::AckermannDriveStamped stop;
  stop.drive.speed = 0;
  stop.drive.acceleration = 0;
  stop.drive.jerk = 0;
  stop.drive.steering_angle = 0;
  stop.drive.steering_angle_velocity = 0;

  ackermann_msgs::AckermannDriveStamped retreat;
  retreat.drive.speed = -0.2;
  retreat.drive.acceleration = 0;
  retreat.drive.jerk = 0;
  retreat.drive.steering_angle = 0;
  retreat.drive.steering_angle_velocity = 0;

  ackermann_msgs::AckermannDriveStamped left;
  left.drive.speed = -0.2;
  left.drive.acceleration = 0;
  left.drive.jerk = 0;
  left.drive.steering_angle = 0;
  left.drive.steering_angle_velocity = 0;

  ackermann_msgs::AckermannDriveStamped right;
  right.drive.speed = -0.2;
  right.drive.acceleration = 0;
  right.drive.jerk = 0;
  right.drive.steering_angle = ;
  right.drive.steering_angle_velocity = 0;

  clock_t start;
  while(ros::ok())
  {
    if(obstacle == 0)
    {
      drive_pub.publish(drive_slow);
    }
    if(obstacle == 1)
    {
      dodge_obstacle();
    }
  }
  ros::spin();
  return 0;
}

void dodge_obstacle()
{
  drive_pub.publish(stop);
  ros::Duration(0.5).sleep();   //check if the obstacle is a driving car
  go_back(0.5);
  

}

void go_back(double distance) //[meters]
{
  ackermann_msgs::AckermannDriveStamped retreat;
  retreat.drive.speed = -DRIVE_SPEED;
  retreat.drive.acceleration = 0;
  retreat.drive.jerk = 0;
  retreat.drive.steering_angle = 0;
  retreat.drive.steering_angle_velocity = 0;
  drive_pub.publish(retreat)
  ros::Duration(distance / DRIVE_SPEED).sleep();
}

void go_forward(double distance)
{
  ackermann_msgs::AckermannDriveStamped a;
  a.drive.speed = DRIVE_SPEED;
  a.drive.acceleration = 0;
  a.drive.jerk = 0;
  a.drive.steering_angle = 0;
  a.drive.steering_angle_velocity = 0;
  drive_pub.publish(a)
  ros::Duration(distance / DRIVE_SPEED).sleep();
}

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
