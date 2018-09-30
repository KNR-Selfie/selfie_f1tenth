#include<iostream>
#include"ros/ros.h"
#include"ackermann_msgs/AckermannDriveStamped.h"


#define DRIVE_SPEED 0.4
#define MAX_ANGLE 0.7


void move_forward(const ros::Publisher &n, double speed)
{
  ackermann_msgs::AckermannDriveStamped drive_slow;
  drive_slow.drive.speed = speed;
  drive_slow.drive.acceleration = 0;
  drive_slow.drive.jerk = 0;
  drive_slow.drive.steering_angle = 0;
  drive_slow.drive.steering_angle_velocity = 0;
  n.publish(drive_slow);
//  ros::Duration(dist/DRIVE_SPEED).sleep();
}

void retreat(const ros::Publisher &n, double dist, double stop_time, double angle)
{

  std::cout << "retreating\n";
  ackermann_msgs::AckermannDriveStamped retreat;
  retreat.drive.speed = 0;
  retreat.drive.acceleration = 0;
  retreat.drive.jerk = 0;
  retreat.drive.steering_angle = 0;
  retreat.drive.steering_angle_velocity = 0;
  n.publish(retreat); //stoping engine and reseting steering angle
  ros::Duration(stop_time).sleep();

  retreat.drive.speed = -DRIVE_SPEED;
  retreat.drive.acceleration = 0;
  retreat.drive.jerk = 0;
  retreat.drive.steering_angle = angle;
  retreat.drive.steering_angle_velocity = 0;
  n.publish(retreat);
  ros::Duration(dist/DRIVE_SPEED).sleep();
}

void turn_left(const ros::Publisher &n, double dist, bool forward)
{
  ackermann_msgs::AckermannDriveStamped left;
  if(forward == true)
    left.drive.speed = DRIVE_SPEED;
  else
    left.drive.speed = -DRIVE_SPEED;
  left.drive.acceleration = 0;
  left.drive.jerk = 0;
  left.drive.steering_angle = MAX_ANGLE;
  left.drive.steering_angle_velocity = 0;
  n.publish(left);
  ros::Duration(DRIVE_SPEED/dist).sleep();
}

void turn_right(const ros::Publisher &n, double dist, bool forward)
{
  ackermann_msgs::AckermannDriveStamped right;
  if(forward == true)
    right.drive.speed = 0.2;
  else
    right.drive.speed = -0.2;
  right.drive.acceleration = 0;
  right.drive.jerk = 0;
  right.drive.steering_angle = -MAX_ANGLE;
  right.drive.steering_angle_velocity = 0;
  n.publish(right);
  ros::Duration(DRIVE_SPEED/dist).sleep();
}
