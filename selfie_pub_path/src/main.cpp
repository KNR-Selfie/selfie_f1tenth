#include "ros/ros.h"
#include "nav_msgs/Path.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "selfie_pub_path");
  ros::NodeHandle n;
  ros::Publisher path_publisher = n.advertise<nav_msgs::Path>("path", 100);
  nav_msgs::Path path_msg;
  geometry_msgs::PoseStamped pose;
  ros::Rate loop_rate(10000000);

  path_msg.header.frame_id = 'path';
  //path_msg.header.stamp = plan[0].header.stamp;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  float fi = 0;
  float add_x = 0.5;
  float add_y = 0.7;
  float add_c_x = 0.1;
  float add_c_y = 0.2;

  while (ros::ok())
  {
    ros::spinOnce();
    ros::Time now = ros::Time::now();
    path_msg.poses.clear();
    pose.pose.position.x = 0.1;
    pose.pose.position.y = 0.1;
    float fi = 0;
    float add_x = 0.5;
    float add_y = 0.7;
    float add_c_x = 0.1;
    float add_c_y = 0.2;
    for(int i = 0; i<10;i++){
    pose.pose.position.x += add_x*add_c_x;
    pose.pose.position.y += add_y*add_c_y;

    add_c_x +=add_c_x;
    add_c_y +=add_c_y;
    if (add_c_x>1){
      add_c_x = 0.1;
    }
    if (add_c_y>1){
      add_c_y = 0.2;
    }
    path_msg.poses.push_back(pose);
    }
    /*fi = 0;
    for(int i=0;i<10;i++){
      
      pose.pose.position.x = 10*sin(fi)+10;
      pose.pose.position.y = 10*cos(fi);
      pose.pose.position.z = 0.0;
      pose.pose.orientation.x = 0.0;
      pose.pose.orientation.y = 0.0;
      pose.pose.orientation.z = 0.0;
      pose.pose.orientation.w = 1.0;
      path_msg.poses.push_back(pose);
      fi = 36*3.14/180;
      //publishing msg
      //path_publisher.publish(path_msg);
    }*/
    path_publisher.publish(path_msg);
    loop_rate.sleep();
  }
}