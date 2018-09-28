#include "obstacles.h"

using namespace std;

void pathCallback(const nav_msgs::Path::ConstPtr& msg);
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
void wallDistanceCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
void positionCallback(const geometry_msgs::Pose::ConstPtr& msg);
void obstaclesExist(const std_msgs::Bool::ConstPtr& exist);

// categorize variables //
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

// sectors variables
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

// categorizes global variables
float Angle_min = 0;
float Angle_max = 0;
float Angle_increment = 0;
float *Ranges;
float Left = 0;
float Right = 0;
float Pos_x = 0;
float Pos_y = 0;
vector<float> Path_x;
vector<float> Path_y;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "selfie_obstacles");
    ros::NodeHandle n;

    ros::Publisher obstackles_publisher = n.advertise<std_msgs::Bool>("obstacles_exist", 100);
    ros::Subscriber path_subscriber = n.subscribe("path", 1, pathCallback);
    ros::Subscriber laserScan_subscriber = n.subscribe("laser_scan", 1, laserScanCallback);
    ros::Subscriber wallDist_subscriber = n.subscribe("wall_dist", 1, wallDistanceCallback);
    ros::Subscriber position_subscriber = n.subscribe("position", 1, positionCallback);
    
    ros::Time current_time;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
    ros::spinOnce();
    current_time = ros::Time::now();

    std_msgs::Bool obst_exist;
    obst_exist.data = categorizeLaserScan(Angle_min, Angle_max, Angle_increment, Ranges, Left, Right, Pos_x, Pos_y, Path_x, Path_y);
    obstackles_publisher.publish(obst_exist);
    loop_rate.sleep();
  }
}

//callback from path 
void pathCallback(const nav_msgs::Path::ConstPtr& msg)
{
  // declare variables
  std::vector<geometry_msgs::PoseStamped> pos = msg->poses;

  // clear vectors
  Path_x.clear();
  Path_y.clear();

  // read path
  for(std::vector<geometry_msgs::PoseStamped>::iterator it = pos.begin(); it !=pos.end(); ++it)
  {
    Path_x.push_back(it->pose.position.x);
    Path_y.push_back(it->pose.position.y);
  }  
}

// callback from laser scan
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  float laser_scan_array[100];
  for(int i = 0; i < 100; i++)
  {
    laser_scan_array[i] = scan->ranges[i];
  }
  *Ranges = laser_scan_array[0];
  Angle_min = scan->angle_min;
  Angle_max = scan->angle_max;
  Angle_increment= scan->angle_increment;
}

// callback from wall distances
void wallDistanceCallback(const std_msgs::Float32MultiArray::ConstPtr& wall)
{
  float data[2];
  for (int i = 0; i < 2; i++)
  {
    data[i] = wall-> data[i];
  }
  float left_dist = data[0];
  float right_dist = data [1];
  }

// callback from position
void positionCallback(const geometry_msgs::Pose::ConstPtr& position)
{
  Pos_x = position->position.x;
  Pos_y = position->position.y;
}

// categorize obstacles
bool categorizeLaserScan(float angle_min,float angle_max,float angle_increment,float *ranges,float left,float right, float pos_x, float pos_y, vector<float> path_x, vector<float> path_y)
{
    vector<float> path_range;
    vector<float> path_fi;

    path_MF_count = 0;
    path_RF_count = 0;
    path_LF_count = 0;

    for(int i = 0;i<path_x.size();i++)
    {
        //zamiana x z y? dobrze to rozumiem?
       path_x[i] = (path_y[i] - pos_y);
       path_y[i] = (path_x[i] - pos_x);

       //zmienne biegunowe (promień i kąt w stopniach)
       path_range.push_back(sqrt(path_x[i]*path_x[i]+path_y[i]*path_y[i]));
       path_fi.push_back(atan(path_x[i]/path_y[i])*180/M_PI);

    }
    //wyzerowanie układu współrzędnych
    pos_x = 0;
    pos_y = 0;

    //przelicz zmienne lidaru na bieugnowe
    angle_increment = angle_increment*180/M_PI;
    angle_min = angle_min * 180/M_PI;
    angle_max = angle_max * 180/M_PI;

    int point_number = (angle_max - angle_min)/angle_increment;

    //jeden procent kątowy
    float percent = (angle_max - angle_min)/100;

    //zdefiniowanie sektorów
    RR_count = 0;
    RR[0] = angle_min;
    RR[1] = angle_min + REAR_VAGUE * percent;

    RS_count = 0;
    RS[0] = RR[1];
    RS[1] = RS[0] + SIDE_VAGUE * percent;

    RF_count = 0;
    RF[0] = RS[1];
    RF[1] = RF[0] + FRONT_VAGUE * percent;

    //middle sector
    MF_count = 0;
    MF[0] = RF[1];
    MF[1] = MF[0] + MIDDLE_VAGUE * percent;

    //left sector
    LF_count = 0;
    LF[0] = MF[1];
    LF[1] = LF[0] + FRONT_VAGUE * percent;

    LS_count = 0;
    LS[0] = LF[1];
    LS[1] = LS[0] + SIDE_VAGUE * percent;

    LR_count = 0;
    LR[0] = LS[1];
    LR[1] = LR[0] + REAR_VAGUE * percent;

    //sprawdź czy path jest w sektorach LF,MF,RF
    for(int i = 0;i<path_fi.size();i++)
    {
        if(path_fi[i]>RF[0] && path_fi[i]<RF[1])
            path_RF_count++;
        if(path_fi[i]>MF[0] && path_fi[i]<MF[1])
            path_MF_count++;
        if(path_fi[i]>LF[0] && path_fi[i]<LF[1])
            path_LF_count++;
    }

    if(path_RF_count>path_treshold || path_LF_count>path_treshold || path_MF_count>path_treshold)
    {
        //ścieżka w sektorach - szukamy przeszkody
    }
    else
    {
        return false; // ścieżka jest poza przodem - olewamy przeszkody
    }

    //filtrowanie i przypisanie danych z lidaru do sektorów
    for(int i = LS[0]/angle_increment;i<RS[1]/angle_increment;i++)
    {
        float range = ranges[i];
        float angle = i*angle_increment;

        //punkty które sa bliżej niż treshold
        if(range<range_treshold)
        {
           //jest git - idź dalej
        }
        else
        {
            continue; //jeżeli punkt za daleko przejdź do kolejnej iteracji
        }

        //przypisanie punktów do sektorów

        //połowa zakresu 0-135
        if(angle<(angle_max-angle_min)/2)
        {
//            if(angle>RR[0] && angle<RR[1])
//                RR_count++;
            if(angle>RS[0] && angle<RS[1])
                RS_count++;
            if(angle>RF[0] && angle<RF[1])
                RF_count++;
            if(angle>MF[0] && angle<MF[1])
                MF_count++;
        }
        else //zakres 135-270
        {
//            if(angle>LR[0] && angle<LR[1])
//                LR_count++;
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
//        //sprawdź sektory boczne
//        if(LS_count<5) //wolny lewy
//        {
//            //offset na minus
//        }
//        else if(RS_count<5)//wolny prawy
//        {
//            //offset na plus
//        }
        return true;
    }

    //nie ma przeszkody
    return false;

}



