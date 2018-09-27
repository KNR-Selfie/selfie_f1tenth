#pragma once
#ifndef OBSTACLES_H
#define OBSTACLES_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Imu.h>

float angle_min = 0;
float angle_max = 0;
float angle_increment = 0;
//?
float ranges[8];

/// categorize variables ///
    // right sectors //
    float RR[2];
    int RR_count = 0;
    float RS[2];
    int RS_count = 0;
    float RF[2];
    int RF_count = 0;
    // middle sectors //
    float MF[2];
    int MF_count = 0;
    // left sectors //
    float LS[2];
    int LS_count = 0;
    float LF[2];
    int LF_count = 0;
    float LR[2];
    int LR_count = 0;



void getLaserScan (const sensor_msgs::LaserScan &sens_msg);
bool categorizeLaserScan(float angle_min,float angle_max,float angle_increment,float *ranges);



    #endif // OBSTACLES_H