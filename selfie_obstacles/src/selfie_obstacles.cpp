#include "obstacles.h"

//jesli chcesz to odkomentuj
void pathCallback(const nav_msgs::Path::ConstPtr& msg);
//void listen_tf(geometry_msgs::TransformStamped transformStamped, float& position_x, float& position_y, float& position_z, float& orientation_x,float& orientation_y,float& orientation_z, float& orientation_w, float& yaw);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "selfie_obstacles");
    ros::NodeHandle n;
    //zmienic nazwe msgs
    //ros::Publisher obstackles_publisher = n.advertise<ackermann_msgs::AckermannDriveStamped>("drive", 100);
    ros::Subscriber path_subscriber = n.subscribe("path", 1, pathCallback);

    //sluchanie tf2
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);
    
    // obst_pub = n.advertise<>();
    // ros::Subscriber sub_distance = n.subscribe();
    // ros::Subscriber sub_imu = n.subscribe();
    while (ros::ok()){
    ros::spinOnce();
    /*
    //listen localization data 
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("map","base_link",ros::Time(0));
    }
    catch (tf2::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    */
    //get tf information
    //listen_tf(transformStamped,drive.localization.position_x,drive.localization.position_y,drive.localization.position_z,drive.localization.orientation_x,drive.localization.orientation_y, drive.localization.orientation_z, drive.localization.orientation_w, drive.localization.yaw);
    
    //zmienic nazwe obs_msg na ta ktora bedziesz uzywal
    //obstackles_publisher.publish(obs_msg);
    rate.sleep();
  }
}

    


//callback from path 
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  /*
  std::vector<geometry_msgs::PoseStamped> data = msg->poses;
  //ja w tej klasie przechowuję pozycje z path, typ vector
  drive.path.position_x.clear();
  drive.path.position_y.clear();
  //sczytywanie z path
  for (std::vector<geometry_msgs::PoseStamped>::iterator it = data.begin(); it !=data.end(); ++it){
    drive.path.position_x.push_back(it->pose.position.x);
    drive.path.position_y.push_back(it->pose.position.y);
  }
  //sprawdzam czy jest taka dlugosc jaka chce w wiadomosci
  if (drive.path.position_x.size()!=10){
    ROS_INFO("Path callback errror");
  }
  */
}

/*
//pomocnicza do listen_tf, wrzuc do obstacles.cpp jak bedzie potrzebowal
double convert_quaternion_to_yaw(float orientation_x, float orientation_y, float orientation_z, float orientation_w){
  tf2::Quaternion q(
    orientation_x,
    orientation_y,
    orientation_z,
    orientation_w);

  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}

//listening TF 
//nie wiem czy potrzebujesz
//jak tak to w argumentach wrzucasz 
void listen_tf(geometry_msgs::TransformStamped transformStamped, float& position_x, float& position_y, float& position_z, float& orientation_x,float& orientation_y,float& orientation_z, float& orientation_w, float& yaw){

  position_x = transformStamped.transform.translation.x;
  position_y = transformStamped.transform.translation.y;
  ROS_INFO("Posx %f Posy: %f",position_x, position_y);
   
  position_z = transformStamped.transform.translation.z;
  orientation_x = transformStamped.transform.rotation.x;
  orientation_y = transformStamped.transform.rotation.y;
  orientation_z = transformStamped.transform.rotation.z;
  orientation_w = transformStamped.transform.rotation.w;
  yaw = convert_quaternion_to_yaw(orientation_x, orientation_y, orientation_z, orientation_w);
  //yaw = -90*3.14/180;
  //ROS_INFO("%f %f %f %f", orientation_x, orientation_y, orientation_z, orientation_w);
}
*/