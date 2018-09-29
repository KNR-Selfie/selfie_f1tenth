/*
**detecting walls in front of car's face. by  avaraging lidar reads 20 cm in frot of the car
**
**TODO: poodzielc pole widzenia na 3 czesci, jakby przszkoda byla nie dokladnie przed callym autem
**
**
*/
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"

#include<iostream>
#include<time.h>

#define SLEEP_TIME 1
#define DRIVE_SPEED 0.1
#define MAX_ANGLE 0.7
#define LEFT 0
#define RIGHT 1
#define PI 3.1415926
#define OBSERVED_AREA 60 //in degrees
#define DEAD_LINE 0.2 //in meters from LIDAR

bool obstacle = 0;
bool info = 0;
double deg_per_angle = 0;

void move_forward(const ros::Publisher &, double);
void turn_left(const ros::Publisher &, double, bool dist);
void turn_right(const ros::Publisher &, double, bool dist);
void retreat(const ros::Publisher &, double, double stop_time);
bool check_lidar_data(const sensor_msgs::LaserScan &ms);


void scanCallback(const sensor_msgs::LaserScan &ms)
{  //TODO!!!! detecting wall in front of my face

  if(check_lidar_data(ms) == 1)
    obstacle = 1;
}
void recoveryCallback(const std_msgs::Bool &ms)
{
  obstacle = ms.data;
  if(obstacle == 0)
    std::cout  << " normal_mode info\n";
  else
    std::cout << " recover info\n";
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "recovery_behaviour");
  ros::NodeHandle n;

  ros::Publisher drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 50);
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  ros::Subscriber obstacle_sub = n.subscribe("/recovery_mode", 1000, recoveryCallback);
  while(ros::ok())
  {
    ros::spinOnce();
    if(obstacle == 0)
    {
     std::cout << "in forward move\n";
     move_forward(drive_pub, 0);
    }else
    {
      std::cout << "in recovery mode\n";
      retreat(drive_pub, 0.4, 1);
    }
  }
  ros::spinOnce();
  return 0;
}

void move_forward(const ros::Publisher &n, double dist)
{
  ackermann_msgs::AckermannDriveStamped drive_slow;
  drive_slow.drive.speed = 0.2;
  drive_slow.drive.acceleration = 0;
  drive_slow.drive.jerk = 0;
  drive_slow.drive.steering_angle = 0;
  drive_slow.drive.steering_angle_velocity = 0;
  n.publish(drive_slow);
//  ros::Duration(dist/DRIVE_SPEED).sleep();
}

void retreat(const ros::Publisher &n, double dist, double stop_time)
{

  std::cout << "retreating\n";
  ackermann_msgs::AckermannDriveStamped drive_slow;
  drive_slow.drive.speed = 0;
  drive_slow.drive.acceleration = 0;
  drive_slow.drive.jerk = 0;
  drive_slow.drive.steering_angle = 0;
  drive_slow.drive.steering_angle_velocity = 0;
  n.publish(drive_slow);
  ros::Duration(stop_time).sleep();

  drive_slow.drive.speed = -0.2;
  drive_slow.drive.acceleration = 0;
  drive_slow.drive.jerk = 0;
  drive_slow.drive.steering_angle = 0;
  drive_slow.drive.steering_angle_velocity = 0;
  n.publish(drive_slow);
  ros::Duration(dist/DRIVE_SPEED).sleep();
}

void turn_left(const ros::Publisher &n, double dist, bool forward)
{
  ackermann_msgs::AckermannDriveStamped left;
  if(forward == true)
    left.drive.speed = 0.2;
  else
    left.drive.speed = -0.2;
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

bool check_lidar_data(const sensor_msgs::LaserScan &ms)
{
//  std::cout <<"ok\n";
  deg_per_angle = ms.angle_increment * 180/PI;
//std::cout << deg_per_angle<<std::endl;  //0.00581
  int max_angle_nr = (ms.angle_max - ms.angle_min)/ms.angle_increment;
//  std::cout << "max_angle_nr: " << max_angle_nr<<std::endl; //510
  int center_angle_nr = max_angle_nr/2;
//  std::cout << "center_angle__nr: " << center_angle_nr << std::endl;
  int observed_angles = OBSERVED_AREA/deg_per_angle;
  int n = center_angle_nr - observed_angles/2;
  //OK
//  std::cout << observed_angles <<std::endl;
  double  min = 100;
  int min_nr = 0;
  double max = 0;
  int max_nr = 0;
  double sum = 0;
  double range_value = 0;
  double avg_range_value = 0;
  for(int i=n;  i<n+observed_angles;  i++)
  {
    range_value = ms.ranges[i];
    sum +=range_value;
    if(range_value < min)
    {
      min = range_value;
      min_nr = i;
    }
    if(range_value > max)
    {
      max = range_value;
      max_nr = i;
    }
  }
  avg_range_value = sum/observed_angles;
/*  std::cout <<  "min_number: " << min_nr <<"\t\t " << min << std::endl;
  std::cout << "max_nr" << max_nr << "\t\t" <<max <<std::endl;
  std::cout << "avg value in our observation range(" << OBSERVED_AREA << "): " << avg_range_value << std::endl;
*/
  if(avg_range_value <= DEAD_LINE)
  {
    ROS_INFO("WE ARE IN A DEAD SPOT!!!!");
    return 1;
  }
  return 0;
}
