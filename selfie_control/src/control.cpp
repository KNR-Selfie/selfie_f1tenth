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
   ros::Time start= ros::Time::now();
   uint32_t start_time = (uint32_t)(start.sec*1000 + start.nsec/1000);
   ros::Time begin = ros::Time::now();
   ros::Time begin_angle = ros::Time::now();
   uint32_t begin_time = (uint32_t)(begin_angle.sec*1000 + begin_angle.nsec/1000000);
   uint32_t begin_time_angle = (uint32_t)(begin_angle.sec*1000 + begin_angle.nsec/100000);

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
      uint32_t now_time = (uint32_t)(now.sec*1000 + now.nsec/1000000);

      if (now_time - begin_time > 1000){
        begin = ros::Time::now();
        begin_time = (uint32_t)(begin.sec*1000 + begin.nsec/1000000);
        msg.speed = msg.speed + direction_speed*0.1;
      }

      if (now_time - begin_time_angle> 100){
        begin_angle = ros::Time::now();
        begin_time_angle = (uint32_t)(begin_angle.sec*1000 + begin_angle.nsec/1000000);
        msg.steering_angle = msg.steering_angle + direction_angle*1*3.14/180;
      }

      if (msg.speed>2){
        direction_speed = -1;
      }
      else if (msg.speed<0.2){
        direction_speed = 1;
      }

      if (msg.steering_angle > 40*3.14/180){
        direction_angle = -1;
      }
      if (msg.steering_angle < -40*3.14/180){
        direction_angle = 1;
      }
      

      control_publisher.publish(msg);
  
      ros::spinOnce();
      ++count;
    }
    return 0;
}
  

