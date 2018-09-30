#include "ros/ros.h"
#include "ros/console.h"
#include "sensor_msgs/LaserScan.h"

using namespace std;

void scanCallback(const sensor_msgs::LaserScan &ms);
void speedCallback(const std_msgs::Float32 &ms);

int main(void)
{
  ros::init(argc, argv, "map_maker");
  ros::NodeHandle n;
  ros::Subscriber scan_sub = n.subscribe("/scan", 1000, scanCallback);
  ros::Subscriber speed_sub = n.subscrbe("/speed", 500, speedCallback)

/*
1  collect track width
2  when speed == 0
    stop collecting + read distance
  back to 1





  */
}
