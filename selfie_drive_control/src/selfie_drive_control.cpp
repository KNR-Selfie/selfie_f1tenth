#include <ros/ros.h>
#include "drive.hpp"

drive_control drive;

void pathCallback(const nav_msgs::Path::ConstPtr& msg);
void listen_tf(geometry_msgs::TransformStamped transformStamped, float& position_x, float& position_y, float& position_z, float& orientation_x,float& orientation_y,float& orientation_z, float& orientation_w, float& yaw);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "selfie_drive_control");
  ros::NodeHandle n;
  ros::Publisher ackermann_publisher = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 100);
  ros::Subscriber path_subscriber = n.subscribe("path", 1, pathCallback);
  
  ackermann_msgs::AckermannDriveStamped ack_msg;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Rate rate(10.0);
  drive.ackermann.steering_angle = 0;//10*3.14/180;
  drive.ackermann.steering_angle_velocity = 0;// 4*3.14/180;
  drive.ackermann.speed = 0;
  drive.ackermann.acceleration = 0;
  drive.ackermann.jerk = 0;
  float pid_out;
  uint32_t counter= 0;
  while (ros::ok()){
    ros::spinOnce();

    //listen localization data 
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_link","map",ros::Time(0));
    }
    catch (tf2::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    
    //get tf information
    
    listen_tf(transformStamped,drive.localization.position_x,drive.localization.position_y,drive.localization.position_z,drive.localization.orientation_x,drive.localization.orientation_y, drive.localization.orientation_z, drive.localization.orientation_w, drive.localization.yaw);
    ROS_INFO("Posx %f Posy: %f", drive.localization.position_x, drive.localization.position_y);
    //check if we have path
    if (drive.path.position_x.size()==10 and counter >3){    
      drive.pid.error = drive.calc_error(drive.localization.position_x, drive.localization.position_y, drive.path.position_x, drive.path.position_y, drive.localization.yaw);
      //ROS_INFO("PID_Error %f", drive.pid.error);
      
      pid_out = drive.calc_PID(drive.pid.error, drive.pid.e_i, drive.pid.e_prev,drive.pid.kp, drive.pid.ki, drive.pid.kd);
      //ROS_INFO("PID_OUD %f", pid_out);
      //pack data into msg
      ROS_INFO("PID error: %f, PID_Out %f Yaw: %f", drive.pid.error, pid_out, drive.localization.yaw);
      ack_msg.drive.steering_angle = pid_out;
      ack_msg.drive.steering_angle_velocity = pid_out*0.1;
      ack_msg.drive.speed = 0.2;
      ack_msg.drive.acceleration = 0.1;
      ack_msg.drive.jerk =0.1;
      ackermann_publisher.publish(ack_msg);
    }

    
    
    

    //ROS_INFO("%f, %f", drive.path.position_x, drive.path.position_y);
    //ackermann_publisher.publish(ack_msg);
    rate.sleep();
    counter ++;
  }
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  std::vector<geometry_msgs::PoseStamped> data = msg->poses;
  drive.path.position_x.clear();
  drive.path.position_y.clear();
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = data.begin(); it !=data.end(); ++it){
    drive.path.position_x.push_back(it->pose.position.x);
    drive.path.position_y.push_back(it->pose.position.y);
  }
  if (drive.path.position_x.size()!=10){
    ROS_INFO("Path callback errror");
  }

}


void listen_tf(geometry_msgs::TransformStamped transformStamped, float& position_x, float& position_y, float& position_z, float& orientation_x,float& orientation_y,float& orientation_z, float& orientation_w, float& yaw){
  //błąd symulatora:
  position_x = transformStamped.transform.translation.x;

  position_y = transformStamped.transform.translation.y;
  position_z = transformStamped.transform.translation.z;
  orientation_x = transformStamped.transform.rotation.x;
  orientation_y = transformStamped.transform.rotation.y;
  orientation_z = transformStamped.transform.rotation.z;
  orientation_w = transformStamped.transform.rotation.w;
  yaw = drive.convert_quaternion_to_yaw(orientation_x, orientation_y, orientation_z, orientation_w);
  //ROS_INFO("%f %f %f %f", orientation_x, orientation_y, orientation_z, orientation_w);
}
