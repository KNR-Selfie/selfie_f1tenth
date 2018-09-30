
#include "std_msgs/Float64.h"
#include "sensor_msgs/LaserScan.h"

#include "include/process.hpp"
#include "sensor_msgs/PointCloud.h"

#define left_and_right_pub

Process process;
SteerData data_to_send;

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& message);

ros::Publisher steering_publisher;

std_msgs::Float64 state_msg;

#ifdef left_and_right_pub
//LEFT AND RIGHT
ros::Publisher left_publisher;
ros::Publisher right_publisher;
sensor_msgs::PointCloud left_msg;
sensor_msgs::PointCloud right_msg;
  
#endif (left_and_right_pub)
void pub_left_and_right_data(LidarReading points_to_send, ros::Publisher publisher_msg, sensor_msgs::PointCloud msg);
  
int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_steering");
  ros::NodeHandle node;
  steering_publisher = node.advertise<std_msgs::Float64>("steering_state", 100);
  
  #ifdef left_and_right_pub
  left_publisher = node.advertise<sensor_msgs::PointCloud>("left_laser",100);
  right_publisher = node.advertise<sensor_msgs::PointCloud>("right_laser",100);
  #endif (left_and_right_pub)

  ros::Subscriber lidar_subscriber = node.subscribe("scan", 1, lidarCallback);
  std_msgs::Float64 state_msg;
  

  while (ros::ok())
  {
  
     // Next step
    ros::spinOnce();
  }

  return 0;
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& message)
{
    // Copy data
    process.angle_min = -(message->angle_max);
    process.raw_data.clear();
    for(uint32_t i = 0; i<message->ranges.size(); i++){
        process.raw_data.push_back (message->ranges[i]);
    }
    // Process data
    process.polar_to_cartesian();
    process.simplify_data();
    process.split_poins_equally();
    process.search_points();
    process.calc_mid();
    process.calc_offsets();
    process.calc_slopes();
    
    // Publish data
    process.pack_data(data_to_send);
    state_msg.data = data_to_send.offset[1]*0.8 + data_to_send.offset[0]*0.2;
    steering_publisher.publish(state_msg);
    ROS_INFO("Output %d",state_msg.data);
    pub_left_and_right_data(process.right_points, right_publisher, right_msg);

    pub_left_and_right_data(process.left_points, left_publisher, left_msg);
}

void pub_left_and_right_data(LidarReading points_to_send, ros::Publisher publisher_msg, sensor_msgs::PointCloud msg)
{   
    geometry_msgs::Point32 point;
    msg.points.clear();
    for (int32_t i=0; i<points_to_send.pos.size(); i++){
       
        
        point.x = points_to_send.pos[i].x;

        point.y = points_to_send.pos[i].y;

        point.z = 0;
        msg.header.frame_id = "laser";
        msg.points.push_back(point);
        

    }
    publisher_msg.publish(msg);
}