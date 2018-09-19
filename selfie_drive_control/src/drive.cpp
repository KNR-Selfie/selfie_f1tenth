#include "drive.hpp"
#include <tf2/LinearMath/Matrix3x3.h>

double drive_control::convert_quaternion_to_yaw(float orientation_x, float orientation_y, float orientation_z, float orientation_w){
  tf2::Quaternion q(
    orientation_x,
    orientation_y,
    orientation_z,
    orientation_w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  if (yaw_initialized == 0) {
    
    base_yaw = yaw;
    if (base_yaw==base_yaw){
        yaw_initialized = 1;
    }
    
  }
  ROS_INFO("BASE:%lf %lf",yaw, base_yaw);
  double yaw_now;
  if (yaw_initialized==1){
    yaw_now =  yaw - base_yaw;
  }
  else{
    yaw_now = 0;
  }

  return yaw_now;
}

float drive_control::get_theta(float position_x, float position_y, float target_x, float target_y, float yaw){
    float delta_x = target_x - position_x;
    float delta_y = target_y - position_y;
    float theta = atan(delta_y/delta_x)-yaw;
    return theta;
}

