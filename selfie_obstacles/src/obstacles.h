#pragma once
#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <vector>

using namespace std;

void getLaserScan (const sensor_msgs::LaserScan &sens_msg);
void categorize(float angle_min, float angle_max, float angle_increment, vector<float> ranges,
 float pos_x, float pos_y, vector<float> path_x, vector<float> path_y, vector<float>path_width, float *offset);

    #endif // OBSTACLES_H