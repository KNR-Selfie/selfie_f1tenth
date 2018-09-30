/*
**detecting walls in front of car's face. by  avaraging lidar reads 20 cm in frot of the car
**
**TODO: poodzielc pole widzenia na 3 czesci, jakby przszkoda byla nie dokladnie przed callym autem
**
**IMPROTANT:: usage: first param = observed_area[meters]
      second param = dead_line distance
*/
#include "ros/ros.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include <math.h>

#include<iostream>
#include<time.h>

#define SLEEP_TIME 1
#define LEFT 0
#define RIGHT 1
#define PI 3.1415926
#define MAX_ANGLE 0.7
#define OBSERVED_SIDE_ANGLE 0.
//#define OBSERVED_AREA 60 //in degrees
//#define DEAD_LINE 0.2 //in meters from LIDAR

double DEAD_LINE = 0.2;     //meters counted from LIDAR
double OBSERVED_AREA = 0.3; // in meters
double OBSERVED_ANGLE = 2 * std::atan(OBSERVED_AREA/(2*DEAD_LINE));

bool obstacle = 0;
bool info = 0;
bool recovery_mode = 0;
double deg_per_angle = 0;
long double loop_rate = 10;

void move_forward(const ros::Publisher &, double);
void turn_left(const ros::Publisher &, double, bool dist);
void turn_right(const ros::Publisher &, double, bool dist);
void retreat(const ros::Publisher &, double, double stop_time, double angle);
bool check_lidar_data(const sensor_msgs::LaserScan &ms, int observed_angles, int observation_center);
double check_sides(const sensor_msgs::LaserScan &ms, int observed_angles);
float diff = 0;



void scanCallback(const sensor_msgs::LaserScan &ms)
{
  if(check_lidar_data(ms, OBSERVED_ANGLE, 90) == 1)
    recovery_mode = 1;

//  diff = check_sides(ms, OBSERVED_SIDE_ANGLE);
//  lidar_rate = ms.time_increment;
}

int main(int argc, char **argv)
{
  using namespace std;
  double speed = 0;
  if(argc != 1 && argc != 2 && argc != 3)
  {
    std:: cout << "use one, two or zero parameters\n";
    return 0;
  }

  ros::init(argc, argv, "recovery_behaviour");
  ros::NodeHandle n;

  ros::Publisher drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 50);
  ros::Publisher center_dist = n.advertise<std_msgs::Float32>("/from_right", 50);
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
//  ros::Subscriber obstacle_sub = n.subscribe("/recovery_mode", 1000, recoveryCallback);
  if(argc == 1)
  {
  /*  cout << "default angle and Dead_line values ( 60* and 20cm)\none your parameter is set to be speed\n";
    char nr = argv[1][0];;
    int desired_speed = (int)nr - 48;
    std::cout << desired_speed << std::endl;
  */
    cout << "put desired observed area value[meters]:\n";
    cin >> OBSERVED_AREA;
    cout << "put desired DEAD_LINE distance[meters]:\n";
    cin >> DEAD_LINE;
    OBSERVED_ANGLE = 2*(180/PI) * std::atan(OBSERVED_AREA/(2*DEAD_LINE));
    cout << "observed_angle [degrees] = " << OBSERVED_ANGLE<<endl;
    cout << "dead_line [meters/10] = " << DEAD_LINE <<endl;

    ros::Rate r(loop_rate);
    while(ros::ok())
    {
      if(recovery_mode == 0)
      {
      //  std::cout << "in forward move\n";
        move_forward(drive_pub, 0); //sending drive commands from selfie_control
      }else
      {
      //  std::cout << "in recovery mode\n";
        retreat(drive_pub, 0.4, 1, 0);
        recovery_mode = 0;
      }
      ros::spinOnce();
    }
  }
  if(argc == 3)
  {
    cout << "put desired observed area value[meters]:\n";
    cin >> OBSERVED_AREA;
    cout << "put desired DEAD_LINE distance[meters]:\n";
    cin >> DEAD_LINE;
    cout << "put desired speed\n";
    cin >> speed;
    OBSERVED_ANGLE = 2*(180/PI) * std::atan(OBSERVED_AREA/(2*DEAD_LINE));

    cout << "observed_angle [degrees] = " << OBSERVED_ANGLE<<endl;
    cout << "dead_line [meters] = " << DEAD_LINE <<endl;

    ros::Rate r(loop_rate);
    while(ros::ok())
    {
//      cout << lidar_rate << endl;
      if(recovery_mode == 0)
      {
      //  std::cout << "in forward move\n";
        std_msgs::Float32 left_right_diff;
        left_right_diff.data = diff;
        center_dist.publish(left_right_diff);
        move_forward(drive_pub, speed); //sending drive commands from selfie_control

      }else
      {
      //  std::cout << "in recovery mode\n";
        retreat(drive_pub, 0.4, 1, MAX_ANGLE);
        recovery_mode = 0;
      }
      ros::spinOnce();
    }
  }
  return 0;
}


bool check_lidar_data(const sensor_msgs::LaserScan &ms, int observed_angles, int observation_center)
{
  using namespace std;
  deg_per_angle = ms.angle_increment * 180/PI;
  int max_angle_nr = (ms.angle_max - ms.angle_min)/ms.angle_increment;
  cout << "mr of angles: " << max_angle_nr<<endl;

  int center_angle_nr = observation_center/2;
  observed_angles = observed_angles/deg_per_angle;
  cout << "observed_angles: " << observed_angles<<endl;
  int n = center_angle_nr - observed_angles/2;
  cout << "observed_angle[nr na 510]: " <<  n<<endl;
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
