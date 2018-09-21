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

float drive_control::get_theta(float position_x, float position_y, float target_x, float target_y){
  float delta_x = target_x - position_x;
  float delta_y = target_y - position_y;
  float theta = atan(delta_y/delta_x);
  return theta;
}

float drive_control::get_distance(float position_x, float position_y, float target_x, float target_y){
  float delta_x = target_x - position_x;
  float delta_y = target_y - position_y;
  float dist = sqrt(delta_x*delta_x+delta_y+delta_y);
  return dist;
}

float drive_control::check_target(float position_x, float position_y, std::vector<float> path_position_x, std::vector<float> path_position_y, float yaw){
  float delta_x, delta_y;
  float r_2 = 100.f;
  float r_2_cmp;
  int idx = 0;
  //find min distance from points
  for (int i=0; i<path_position_x.size(); ++i){
    delta_x = abs(position_x - path_position_x[i]);
    delta_y = abs(position_y - path_position_y[i]);
    r_2_cmp = delta_x*delta_x+delta_y*delta_y;
    if (r_2_cmp<r_2){
      r_2 = r_2_cmp;
      idx = i;
    }
  }
  
  float pos_start_x, pos_start_y, pos_end_x, pos_end_y;
  if (idx==0){
    pos_start_x = path_position_x[0];
    pos_start_y = path_position_y[0];
    pos_end_x = path_position_x[1];
    pos_end_y = path_position_y[1];
  }
  else if(idx<path_position_x.size()){
    ROS_INFO("2");
    pos_start_x = path_position_x[idx-1];
    pos_start_y = path_position_y[idx-1];
    pos_end_x = path_position_x[idx+1];
    pos_end_y = path_position_y[idx+1];
  }
  else{
    ROS_INFO("ERROR: Too late path");
  }
  ROS_INFO("Target %f %f",pos_end_x, pos_end_y);
  //ROS_INFO("%f %f %f %f %f %f %f", pos_start_x, pos_start_x, pos_end_x, pos_end_y, position_x, position_y, yaw);
  return calc_path_line(pos_start_x, pos_start_y,pos_end_x, pos_end_y,position_x, position_y, yaw );
}

float drive_control::calc_path_line(float pos_start_x, float pos_start_y, float pos_end_x, float pos_end_y, float pos_now_x, float pos_now_y, float yaw){
  float A = (pos_end_x-pos_start_x);
  float B = (pos_start_y - pos_end_y);
  float C = (pos_start_x*pos_end_y - pos_start_y*pos_end_x);
  float y = -(A*pos_now_x+B*pos_now_y+C)/(sqrt(A*A+B*B));
  
  float theta_path = get_theta(pos_now_x, pos_now_y, pos_end_x, pos_end_y);
  float delta_theta = theta_path - (yaw);
  //ROS_INFO("SIn %f %f %f",y, theta_path, delta_theta);
  y = y + pid.l*sin(delta_theta);
  return y;
}

float drive_control::calc_error(float localization_position_x, float localization_position_y, std::vector<float> path_position_x, std::vector<float> path_position_y, float yaw){
    //check point that we have to go next
    //return the distance 
    float y = check_target(localization_position_x, localization_position_y, path_position_x, path_position_y, yaw);
    return y;
}

float drive_control::calc_PID(float& error, float& e_cum, float& e_prev, float kp, float ki, float kd){
  error = pid.d*error;
  e_cum = e_cum+error;
  float out_pid = -( kp*(error)+ki*(e_cum)+kd*(error-e_prev));
  e_prev = error;
  ROS_INFO("OUT_PID: %f",out_pid);
  if (out_pid>ack_limit.alfa_max){
    out_pid = ack_limit.alfa_max;
  }
  else if (out_pid<-ack_limit.alfa_max){
    out_pid = -ack_limit.alfa_max;
  }
  return out_pid;
}