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
#include <math.h>

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
  float kp = 15.f;
  float ki = 0.f;
  float kd = 0.09;
  float theta;
  float d = 0.1;
  float l = 0.2;
  float error =0;
  float e_i = 0;
  float e_prev = 0;
  PID(){};
};

class ackermann_limits
{
public:
  float v_max = 10;
  float alfa_max = 40*3.14/180;
};

class drive_control
{
private:

public:
  double base_yaw = 0.0;
  uint8_t yaw_initialized = false;
  

  Ackermann_control ackermann;
  ackermann_limits ack_limit;
  path_control path;
  localization_data localization;
  PID pid;

double convert_quaternion_to_yaw(float orientation_x, float orientation_y, float orientation_z, float orientation_w);
float get_theta(float position_x, float position_y, float target_x, float target_y);
float get_distance(float position_x, float position_y, float target_x, float target_y);
float check_target(float position_x, float position_y, std::vector<float> path_position_x, std::vector<float> path_position_y, float yaw);
float calc_path_line(float pos_start_x, float pos_start_y, float pos_end_x, float pos_end_y, float pos_now_x, float pos_now_y, float yaw);
float calc_error(float localization_position_x, float localization_position_y, std::vector<float> path_position_x, std::vector<float> path_position_y, float yaw);
float calc_PID(float& error, float& e_cum, float& e_prev,float kp, float ki, float kd);

   
  drive_control(){};
};



#endif // USB_HPP