#include <ros/ros.h>
#include "drive.hpp"

/*
double speed = 0;

double roll = 0;
double pitch = 0;
double yaw = 0;

double base_yaw = 0;
bool yaw_initialized = false;

double x = 0;
double y = 0;

double vx = 0;
double vy = 0;
double vyaw = 0;

double currentDistance = 0;
double lastDistance = 0;
double deltaDistance = 0;
double dt = 0;
double dx = 0;
double dy = 0;

ros::Time current_time, last_distance_update;
bool distance_initialized = false;
geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
std::string rear_axis_frame;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "selfie_drive_control");
  ros::NodeHandle n;
  ros::Publisher ackerman_publisher = n.advertise<AckermannDrive::Imu>("drive", 100);
  ros::Subscriber path_subscriber = n.subscribe("path", 1, pathCallback);
  ros::Subscriber localization_subscriber = n.subscribe("localization", 1, localizationCallback);

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);


  odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  ros::Subscriber sub_distance = n.subscribe("/distance", 50, distanceCallback);
  ros::Subscriber sub_imu = n.subscribe("/imu", 50, imuCallback);

  rear_axis_frame = n.param<std::string>("rear_axis_frame", "base_link");
  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate loop_rate(10);

  while (n.ok())
  {

    // check for incoming messages
    ros::spinOnce();
    current_time = ros::Time::now();

    // publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = ODOM_FRAME;
    odom_trans.child_frame_id = rear_axis_frame;

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    // send the transform
    odom_broadcaster.sendTransform(odom_trans);

    // publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = ODOM_FRAME;

    // set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    // set the velocity
    odom.child_frame_id = rear_axis_frame;
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vyaw;

    // publish the message
    odom_pub.publish(odom);

    loop_rate.sleep();
  }
}

void distanceCallback(const std_msgs::Float32 &msg)
{

  current_time = ros::Time::now();
  currentDistance = msg.data / 1000;

  if (distance_initialized)
  {
    deltaDistance = currentDistance - lastDistance;
    dt = (current_time - last_distance_update).toSec();
    if (dt != 0)
    {
      speed = deltaDistance / dt;
      vx = speed * cos(yaw);
      vy = speed * sin(yaw);
    }

    dx = deltaDistance * cos(yaw);
    dy = deltaDistance * sin(yaw);

    x += dx;
    y += dy;
  }

  distance_initialized = true;
  last_distance_update = current_time;
  lastDistance = currentDistance;
}

void localizationCallback(const nav_msgs::PoseStamped &msg)
{
  pos_x = msg.pose.position.x
  pox_y = msg.pose.position.y
  pos.z = msg.pose.position.x


  tf::Quaternion q(
    msg.orientation.x,
    msg.orientation.y,
    msg.orientation.z,
    msg.orientation.w);

  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);

  if (!yaw_initialized) {
    base_yaw = yaw;
    yaw_initialized = true;
  }

  yaw -= base_yaw;

  // quaternion created from yaw
  odom_quat = tf::createQuaternionMsgFromYaw(yaw);
}
*/


drive_control drive;

void pathCallback(const nav_msgs::Path::ConstPtr& msg);
float listen_tf(geometry_msgs::TransformStamped transformStamped, float position_x, float position_y, float position_z, float orientation_x,float orientation_y,float orientation_z, float orientation_w );

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
  drive.ackermann.steering_angle = 10*3.14/180;
  drive.ackermann.steering_angle_velocity = 4*3.14/180;
  drive.ackermann.speed = 10;
  drive.ackermann.acceleration = 0.1;
  drive.ackermann.jerk = 0;

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

    drive.localization.yaw = listen_tf(transformStamped,drive.localization.position_x,drive.localization.position_y,drive.localization.position_z,drive.localization.orientation_x,drive.localization.orientation_y, drive.localization.orientation_z, drive.localization.orientation_w);
    ROS_INFO("%f",drive.localization.yaw);


    //pack data into msg
    ack_msg.drive.steering_angle = drive.ackermann.steering_angle;
    ack_msg.drive.steering_angle_velocity = drive.ackermann.steering_angle_velocity;
    ack_msg.drive.speed = drive.ackermann.speed;
    ack_msg.drive.acceleration = drive.ackermann.acceleration;
    ack_msg.drive.jerk = drive.ackermann.jerk;

    //ROS_INFO("%f, %f", drive.path.position_x, drive.path.position_y);
    ackermann_publisher.publish(ack_msg);
    rate.sleep();
  }
}

void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  
  std::vector<geometry_msgs::PoseStamped> data = msg->poses;
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = data.begin(); it !=data.end(); ++it){
    drive.path.position_x = it->pose.position.x;
    drive.path.position_y = it->pose.position.y;
  }

}


float listen_tf(geometry_msgs::TransformStamped transformStamped, float position_x, float position_y, float position_z, float orientation_x,float orientation_y,float orientation_z, float orientation_w ){
  position_x = transformStamped.transform.translation.x;
  position_y = transformStamped.transform.translation.y;
  position_z = transformStamped.transform.translation.z;
  orientation_x = transformStamped.transform.rotation.x;
  orientation_y = transformStamped.transform.rotation.y;
  orientation_z = transformStamped.transform.rotation.z;
  orientation_w = transformStamped.transform.rotation.w;
  double yaw = drive.convert_quaternion_to_yaw(orientation_x, orientation_y, orientation_z, orientation_w);
  return float(yaw);
}
