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
#include <vector>

#define PATH_BUFFER 10

class Ackermann_control
{
public:
  float steering_angle;
  float steering_angle_velocity;
  float speed;
  float acceleration;
  float jerk;
  Ackermann_control(){};
};

class path_control
{
public:
  std::vector<float> position_x;
  std::vector<float> position_y;
  float target_x;
  float target_y;

  //float position_z[PATH_BUFFER];
  //float orientation_x[PATH_BUFFER];
  //float orientation_y[PATH_BUFFER];
  //float orientation_z[PATH_BUFFER];
  //float orientation_w[PATH_BUFFER];
  path_control(){};
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
  localization_data(){};
};

class PID
{
public:
  float Kp = 15.f;
  float Ki = 0.f;
  float Kd = 0.09;
  float theta;
  PID(){};
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
  PID pid;

  double convert_quaternion_to_yaw(float orientation_x, float orientation_y, float orientation_z, float orientation_w);
  float get_theta(float position_x, float position_y, float target_x, float target_y, float yaw);
  void check_target(float position_x, float position_y, std::vector<float> path_position_x, std::vector<float> path_position_y, float& target_x, float& target_y);

  drive_control(){};
};



#endif // USB_HPP