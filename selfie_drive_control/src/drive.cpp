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
  if (idx<path_position_x.size()){
    pos_start_x = path_position_x[idx];
    pos_start_y = path_position_y[idx];
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
  float A,B,C,y;
  if (pos_start_x != pos_end_x && pos_start_y != pos_end_y){
    A = -(pos_end_y - pos_start_y)/(pos_end_x-pos_start_x);
    C = (-A*pos_start_x-pos_start_y);
    y = -(A*pos_now_x+pos_now_y+C)/(sqrt(A*A+1));
    ROS_INFO("1, A: %f, C: %f, pos_start: %f %f", A,C, pos_start_x, pos_start_y);
  }
  else if (pos_start_x == pos_end_x && pos_start_y != pos_end_y){
    A = -(pos_end_y - pos_start_y);
    C = pos_start_x*(pos_end_y-pos_start_y);
    y = -(A*pos_now_x+C)/(sqrt(A*A));
    ROS_INFO("2, A: %f, C: %f, pos_start: %f %f", A,C, pos_start_x, pos_start_y);
  }
  else if (pos_start_x != pos_end_x && pos_start_y == pos_end_y){
    B = pos_end_x - pos_start_x;
    C = -pos_start_y*B;
    y = -(B*pos_now_y+C)/(sqrt(B*B));
    ROS_INFO("3, B: %f, C: %f, pos_start: %f %f", B,C, pos_start_x, pos_start_y);
  }
  else{
    ROS_INFO("PATH ERROR - same points");
  }
  
  float theta_path = get_theta(pos_start_x, pos_start_y, pos_end_x, pos_end_y);
  float delta_theta = theta_path - yaw;
  ROS_INFO("y %f, theta_path: %f, delta: %f",y, theta_path, delta_theta);
  //ROS_INFO("SIn %f %f %f",y, theta_path, delta_theta);
  ROS_INFO("y: %f, kat: %f", y, pid.l*sin(delta_theta));
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
  float out_pd = kp*error + kd*(error-e_prev);
  float out_i = ki*e_cum;
  if (out_i > 0.1*out_pd){
    out_i = 0.1*out_pd;
  }
  //float out_pid = out_pd;
  float out_pid = out_pd+out_i;
  e_prev = error;
  if (out_pid>ack_limit.alfa_max){
    out_pid = ack_limit.alfa_max;
  }
  else if (out_pid<-ack_limit.alfa_max){
    out_pid = -ack_limit.alfa_max;
  }
  return out_pid;
}

