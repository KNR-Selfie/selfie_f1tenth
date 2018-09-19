#pragma once
#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "nav_msgs/Path.h"
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <stdbool.h>

class Ackermann_control
{
public:
  float steering_angle;
  float steering_angle_velocity;
  float speed;
  float acceleration;
  float jerk;
};

class path_control
{
public:
  float position_x;
  float position_y;
  float position_z;
  float orientation_x;
  float orientation_y;
  float orientation_z;
  float orientation_w;
};

class localization_data
{
public:
  float position_x;
  float position_y;
  float position_z;
  float orientation_x;
  float orientation_y;
  float orientation_z;
  float orientation_w;
  float yaw;
};

class drive_control
{
private:

public:
  double base_yaw = 0.0;
  uint8_t yaw_initialized = false;
  

  Ackermann_control ackermann;
  path_control path;
  localization_data localization;

  double convert_quaternion_to_yaw(float orientation_x, float orientation_y, float orientation_z, float orientation_w);
  float get_theta(float position_x, float position_y, float target_x, float target_y, float yaw);

};



#endif // USB_HPP