#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sstream>

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom);

int main(int argc, char **argv){

   ros::init(argc, argv, "listen_odometry");
   ros::NodeHandle n;
   ros::Subscriber odom_subscriber = n.subscribe("odom", 1000, odomCallback);
   ROS_INFO("Loop");
   ros::spin();
   return 0;
}
  

void odomCallback(const nav_msgs::Odometry::ConstPtr& odom){
    float pos_x = odom->pose.pose.position.x;
    float pos_y = odom->pose.pose.position.y;
    float pos_z = odom->pose.pose.position.z;
    float orient = odom->pose.pose.orientation.z;
       
    ROS_INFO("I heard: [X: %.1f Y: %.1f Z: %.1f Orient: %.1f]", pos_x, pos_y, pos_z, orient);
}

