#include "obstacles.h"
#include "selfie_map_processing/PathWithMeta.h"
#include <iostream>

using namespace std;

void pathCallback(const nav_msgs::Path::ConstPtr& msg);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void wallDistanceCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void positionCallback(const geometry_msgs::Pose::ConstPtr& msg);
void obstaclesExist(const std_msgs::Bool::ConstPtr& exist);
void widthCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);

void path_with_metaCallback(const selfie_map_processing::PathWithMeta::ConstPtr& msg);


int path_MF_count = 0;
int path_RF_count = 0;
int path_LF_count = 0;

//tresholds
int path_treshold = 5;
int obstacle_treshold = 3;
int range_treshold = 1000;

int REAR_VAGUE = 20;
int SIDE_VAGUE = 15;
int FRONT_VAGUE = 10;
int MIDDLE_VAGUE = 10;

float RR[2];
int RR_count = 0;

float RS[2];
int RS_count = 0;

float RF[2];
int RF_count = 0;

float MF[2];
int MF_count = 0;

float LF[2];
int LF_count = 0;

float LS[2];
int LS_count = 0;

float LR[2];
int LR_count = 0;

float car_width = 30;

// categorizes global variables
float Angle_min = 0;            // minimal angle
float Angle_max = 0;            // maximum angle
float Angle_increment = 0;      // angle increment
float *Ranges;                  // pointer to ranges array
float Pos_x = 0;                // position x value
float Pos_y = 0;                // position y value
vector<float> Path_x;           // path x values
vector<float> Path_y;           // path y values
float Offset = 0;               // path offset
vector<float> Path_width;       // track width

int main(int argc, char** argv)
{   ROS_INFO("main begin\r\n");
    ros::init(argc, argv, "selfie_obstacles");
    ros::NodeHandle n;
    ROS_INFO("definitions of callbacks \r\n");
    ros::Publisher offset_publisher = n.advertise<std_msgs::Float32>("/offset", 50);
    ros::Subscriber path_with_meta_subscriber = n.subscribe("/path_with_meta", 50, path_with_metaCallback);
    ros::Subscriber laserScan_subscriber = n.subscribe("/scan", 50, laserScanCallback);
    //ros::Subscriber position_subscriber = n.subscribe("/move_base_simple/goal", 1, positionCallback);
    //ros::Subscriber width_subscriber = n.subscribe("width", 1, widthCallback);
    ROS_INFO("loop rate \r\n");
    ros::Time current_time;
    ros::Rate loop_rate(10);

    ROS_INFO("before while \r\n");
    while (ros::ok())
    {
    ROS_INFO("SPIN");
    ros::spinOnce();
    ROS_INFO("TIME");
    current_time = ros::Time::now();

    float offset = 1;

    //categorize(Angle_min, Angle_max, Angle_increment, Ranges, Pos_x, Pos_y, Path_x, Path_y, Path_width, &offset);
    std_msgs::Float32 offset_msg;
    ROS_INFO("Przypisanie");
    offset_msg.data = offset;
    ROS_INFO("Publisher");
    offset_publisher.publish(offset_msg);
    ROS_INFO("rate sleep");
    loop_rate.sleep();
  }

}

// callback from path with meta data 
void path_with_metaCallback(const selfie_map_processing::PathWithMeta::ConstPtr& msg)
{   ROS_INFO("metaCll");
  nav_msgs::Path pth = msg->path;
  
  std::vector<geometry_msgs::PoseStamped> pos = pth.poses;

  Path_x.clear();
  Path_y.clear();

  // read path
  for(std::vector<geometry_msgs::PoseStamped>::iterator it = pos.begin(); it !=pos.end(); ++it)
  {
    Path_x.push_back(it->pose.position.x);
    Path_y.push_back(it->pose.position.y);
    //Path_width.push_back(it->width);
  }
  for(int i = 0;i<pos.size();i++)
  {
      float width_mes = msg->track_width[i];
      Path_width.push_back(width_mes);
  }
  ROS_INFO("meta_end");
}

// callback from laser scan
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  float laser_scan_array[scan->ranges.size()];
  for(int i = 0; i < scan->ranges.size(); i++)
  {
    laser_scan_array[i] = scan->ranges[i];
  }
  *Ranges = laser_scan_array[0];
  Angle_min = scan->angle_min;
  Angle_max = scan->angle_max;
  Angle_increment= scan->angle_increment;
}

// callback from position
void positionCallback(const geometry_msgs::Pose::ConstPtr& position)
{
  Pos_x = position->position.x;
  Pos_y = position->position.y;
}

// categorize obstacles
void categorize(float angle_min, float angle_max, float angle_increment, float *ranges, float pos_x,
 float pos_y, vector<float> path_x, vector<float> path_y, vector<float>path_width, float *offset)
{
    vector<float> path_range;
    vector<float> path_fi;
    vector <float> left_wall_pos;
    vector <float> right_wall_pos;

    int path_size = 1;
    
    path_MF_count = 0;
    path_RF_count = 0;
    path_LF_count = 0;

    for(int i = 0;i<path_x.size();i++)
    {
       //zamiana x z y
       path_x[i] = - (path_y[i] - pos_y);
       path_y[i] = (path_x[i] - pos_x);

       //zmienne biegunowe (promień i kąt w radianach)
       path_range.push_back(sqrt(path_x[i]*path_x[i]+path_y[i]*path_y[i]));
       path_fi.push_back(atan(path_x[i]/path_y[i]));
    }

    //wyzerowanie układu współrzędnych
    pos_x = 0;
    pos_y = 0;

    //jeden procent kątowy
    float percent = (angle_max - angle_min)/100;

    //zdefiniowanie sektorów
    LR_count = 0;
    LR[0] = angle_min;
    LR[1] = angle_min + REAR_VAGUE * percent;

    LS_count = 0;
    LS[0] = LR[1];
    LS[1] = LS[0] + SIDE_VAGUE * percent;

    LF_count = 0;
    LF[0] = LS[1];
    LF[1] = LF[0] + FRONT_VAGUE * percent;

    //middle sector
    MF_count = 0;
    MF[0] = LF[1];
    MF[1] = MF[0] + MIDDLE_VAGUE * percent;

    //left sector
    RF_count = 0;
    RF[0] = MF[1];
    RF[1] = RF[0] + FRONT_VAGUE * percent;

    RS_count = 0;
    RS[0] = RF[1];
    RS[1] = RS[0] + SIDE_VAGUE * percent;

    RR_count = 0;
    RR[0] = RS[1];
    RR[1] = RR[0] + REAR_VAGUE * percent;

    for(int i = 0;i<path_fi.size();i++)
    {
        if(path_fi[i]>RF[0] && path_fi[i]<RF[1])
            path_RF_count++;
        if(path_fi[i]>MF[0] && path_fi[i]<MF[1])
            path_MF_count++;
        if(path_fi[i]>LF[0] && path_fi[i]<LF[1])
            path_LF_count++;
    }

    //sprawdź czy path jest w sektorach LF,MF,RF
    if(!(path_RF_count>path_treshold || path_LF_count>path_treshold || path_MF_count>path_treshold))
    {
        for(int i = 0;i<path_size;i++)
        {
            offset[i] = 0;
        }
        return; //wyjdż z funkcji
    }

    //filtrowanie i przypisanie danych z lidaru do sektorów
    for(int i = LS[0]/angle_increment;i<RS[1]/angle_increment;i++)
    {
        float range = ranges[i];
        float angle = i*angle_increment;

        //filtracja
        if(range>range_treshold)
            continue;

        //przypisanie punktów do sektorów
        if(angle>0)
        {
            if(angle>RS[0] && angle<RS[1])
                RS_count++;
            if(angle>RF[0] && angle<RF[1])
                RF_count++;
            if(angle>MF[0] && angle<MF[1])
                MF_count++;
        }
        else
        {
            if(angle>LS[0] && angle<LS[1])
                LS_count++;
            if(angle>LF[0] && angle<LF[1])
                LF_count++;
            if(angle>MF[0] && angle<MF[1])
                MF_count++;
        }
    }

    //czy wykryto przeszkodode
    if((LF_count>obstacle_treshold || RF_count>obstacle_treshold) && MF_count>obstacle_treshold)
    {
        for(int i = 0;i<path_size;i++)
        {
            right_wall_pos.push_back(path_x[i] + path_width[i]/2);
            left_wall_pos.push_back(path_x[i] - path_width[i]/2);
        }
    }
    else
    {
        for(int i = 0;i<path_size;i++)
        {
            offset[i] = 0;
        }
        return; //wyjdż z funkcji - brak przeszkod
    }

    //przeszkoda z lewej - omiń z prawej
    if(LF_count>RF_count)
    {
        if(LS_count<obstacle_treshold) // lewa strona wolna
        {   //oblicz offset dla wszystkich punktów patha
            for(int i = 0;i<path_size;i++)
            {
                offset[i] = path_width[i]/2 - (right_wall_pos[i] - car_width);
            }
        }
        else //Kombinator help
        {
            for(int i = 0;i<path_size;i++)
            {
                offset[i] = 0;
            }
        }
    }
    else if(RF_count>LF_count)//przeszkoda z prawej - omiń z lewej
    {    //sprawdź czy wolne z lewej
        if(RS_count<obstacle_treshold)
        {
            for(int i = 0;i<path_size;i++)
            {
                offset[i] = path_width[i]/2 - (left_wall_pos[i] + car_width);
            }
        }
        else
        {
            for(int i = 0;i<path_size;i++)
            {
                offset[i] = 0;
            }
        }
    }
}



