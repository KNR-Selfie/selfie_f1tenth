#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include "usb.hpp"
#include <sstream>

USB_STM Usb;

int main(int argc, char **argv){

   ros::init(argc, argv, "selfie_stm32_bridge");
   ros::NodeHandle n;
   ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu>("data_raw", 100);
   ros::Publisher velo_publisher = n.advertise<std_msgs::Float32>("float32", 50);
   //ros::Subscriber ackerman_subscriber = n.subscribe("chatter", 1000, Usb.ackermanCallback);
   ros::Rate loop_rate(10);
      
   int count = 0;

   //usb communication
   //usb_read 
   uint32_t timestamp=1;
   float velocity=1;
   float quaternion_x=1; 
   float quaternion_y=1;
   float quaternion_z=1; 
   float quaternion_w=1;
   float ang_vel_x=1; 
   float ang_vel_y=1;
   float ang_vel_z=1;
   float lin_acc_x=1; 
   float lin_acc_y=1;
   float lin_acc_z=1;
   uint16_t tf_mini=1;
   uint8_t taranis_3_pos=1;
   uint8_t taranis_reset_gear=1;
   uint8_t stm_reset=1;
   uint8_t lights=1;

   //usb receive data
   Usb.init();
  
   while (ros::ok()){
      //reading from usb
      
      Usb.usb_read_buffer(128, timestamp, velocity, quaternion_x, quaternion_y, quaternion_z,quaternion_w,ang_vel_x,  ang_vel_y, ang_vel_z, lin_acc_x, lin_acc_y, lin_acc_z, taranis_3_pos, taranis_reset_gear,stm_reset);
      Usb.usb_send_buffer();
      
      //send to imu
      sensor_msgs::Imu imu_msg;

      imu_msg.header.frame_id = "imu";
      imu_msg.header.stamp = ros::Time::now();
      
      imu_msg.orientation.x = quaternion_x;
      imu_msg.orientation.y = quaternion_y;
      imu_msg.orientation.z = quaternion_z;
      imu_msg.orientation.w = quaternion_w;

      imu_msg.linear_acceleration.x = lin_acc_x;
      imu_msg.linear_acceleration.y = lin_acc_y;
      imu_msg.linear_acceleration.z = lin_acc_z;

      imu_msg.angular_velocity.x = ang_vel_x;
      imu_msg.angular_velocity.y = ang_vel_y;
      imu_msg.angular_velocity.z = ang_vel_z;

      imu_publisher.publish(imu_msg);
      //send to float32
      std_msgs::Float32 velo_msg;
      velo_msg.data = velocity;
      velo_publisher.publish(velo_msg);


      
      ros::spinOnce();
  
      ++count;
    }
    return 0;
}
  

