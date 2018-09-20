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

  return yaw;
}

float drive_control::get_theta(float position_x, float position_y, float target_x, float target_y, float yaw){
  float delta_x = target_x - position_x;
  float delta_y = target_y - position_y;
  float theta = atan(delta_y/delta_x);

  return theta-yaw;
}

void drive_control::check_target(float position_x, float position_y, std::vector<float> path_position_x, std::vector<float> path_position_y, float& target_x, float& target_y){
  float delta_x, delta_y, r_2;
  for (int i=0; i<path_position_x.size(); ++i){
    delta_x = abs(position_x - path_position_x[i]);
    delta_y = abs(position_y - path_position_y[i]);
    r_2 = delta_x*delta_x+delta_y*delta_y;
    if (r_2>4){
      target_x = path_position_x[i];
      target_y = path_position_y[i];
      break;
    }
  }
  
  if (r_2<4){
      ROS_INFO("PATH ERROR");
  }
   
}
