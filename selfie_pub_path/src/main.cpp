#include "ros/ros.h"
#include "nav_msgs/Path.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "selfie_pub_path");
  ros::NodeHandle n;
  ros::Publisher path_publisher = n.advertise<nav_msgs::Path>("path", 100);
  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped pose;
  ros::Rate loop_rate(1000);

  path_msg.header.frame_id = 'path';
  //path_msg.header.stamp = plan[0].header.stamp;
  
  int fi = 0;
  
  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    
    pose.pose.position.x = 10*sin(fi);
    pose.pose.position.y = 10*cos(fi)-10;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    path_msg.poses.push_back(pose);
    fi = fi + 15*3.14/180;
    //publishing msg
    path_publisher.publish(path_msg);

    loop_rate.sleep();
  }
}