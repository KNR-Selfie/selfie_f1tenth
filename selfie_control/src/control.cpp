#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <sstream>

int main(int argc, char **argv){

   ros::init(argc, argv, "control");
   ros::NodeHandle n;
   ros::Publisher control_publisher = n.advertise<ackermann_msgs::AckermannDrive>("drive", 1000);
 
   ros::Rate loop_rate(10);
   int count = 0;
   ros::Time begin = ros::Time::now();
   ros::Time begin_angle = ros::Time::now();

   ackermann_msgs::AckermannDrive msg;
   msg.steering_angle = 0;
   msg.steering_angle_velocity=0;
   msg.speed=0;
   msg.acceleration=0.2;
   msg.jerk=0;
   int8_t direction_speed = 1;
   int8_t direction_angle = 1; 

   while (ros::ok()){
  
      ros::Time now = ros::Time::now();
      
      if (now.sec - begin.sec > 1){
        begin = ros::Time::now();
        msg.speed = msg.speed + direction_speed*0.1;
      }

      if (now.nsec - begin_angle.nsec > 1000000){
        begin_angle = ros::Time::now();
        msg.steering_angle = msg.steering_angle + direction_angle*0.1;
      }

      if (msg.speed>1){
        direction_speed = -1;
      }
      else if (msg.speed<0.2){
        direction_speed = 1;
      }
      if (msg.steering_angle > 40){
        direction_angle = -1;
      }
      if (msg.steering_angle < -40){
        direction_angle = 1;
      }
      

      //ROS_INFO("petla",);
  
      control_publisher.publish(msg);
  
      ros::spinOnce();
      ++count;
    }
    return 0;
}
  

