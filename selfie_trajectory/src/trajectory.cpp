#include "trajectory.h"
#include <iostream>
#include <math.h>
#include <vector>

#define ANGLE_OFFSET 0
#define THRESH_SIMPLIFY 20
#define THRESH_CONTINOUS 50

// Namespace symulujący OpenCV
// namespace cv
// {
//     struct Point
//     {
//         int x;
//         int y;
//     };
// }

// ROS and trajectory variables
sensor_msgs::LaserScan scan;
vector<float> input_scan;
vector<Point> left_points, right_points, rejected_points;

// Struktura do trzymania odczytów z Sicka
struct LidarReading
{
    std::vector<Point> pos; // Wektor kolejnych pozycji obliczonych z promieni
    std::vector<double> angle;  // Wektor kątów odpowiadających kolejnym punktom powyżej
}lidarReading;

// Funkcja zamieniająca wsp biegunowe na kartezjańskie
void polar_to_cartesian(std::vector<float> &input, LidarReading &output);

// Funkcja ustala stałą gęstość punktów, na wejście dajemy wektor punktów z Sicka
void simplify_data(LidarReading &input_output);

// Funkcja dziląca na prawe i lewe linie
void split_poins_equally(LidarReading &input, std::vector<Point> &left_points, std::vector<Point> &right_points, std::vector<Point> &rejected_points);

void laserScanCallback(const sensor_msgs::LaserScan &msg);
void marczuk_spline(vector<Point> right_lidar_vec,vector<Point> left_lidar_vec,float &angle, int velocity);

spline_t left_lidar_spline;
spline_t right_lidar_spline;
spline_t trajectory_path_spline;

tangent middle_tangent;
tangent trajectory_tangent;

bool left_line_detect = 0;
bool right_line_detect = 0;

float servo_weight;
float ang_div;
float far_tg_fac;
float velocity;
float previous_velocity;
float angle;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "selfie_trajectory");
    ros::NodeHandle n;
    ros::Publisher out_publisher = n.advertise<ackermann_msgs::AckermannDrive>("/ack_output", 50);
    ros::Subscriber laserScan_subscriber = n.subscribe("/scan", 50, laserScanCallback);

    ros::Time current_time;
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        //("spin");
        ros::spinOnce();
        //("time");
        current_time = ros::Time::now();
        //("tr_f");

        ackermann_msgs::AckermannDrive output_ackermann;
        output_ackermann.steering_angle = angle;
        output_ackermann.speed = velocity;
        out_publisher.publish(output_ackermann);

        //("sleep");
        loop_rate.sleep();
    }
}

// ROS 
void laserScanCallback(const sensor_msgs::LaserScan& msg)
{
    //("laser scan");
    scan = msg;

    input_scan.clear();
    for(int i = 0; i < scan.ranges.size(); i++)
    {
        input_scan.push_back(scan.ranges[i]);
    }
    //("%d", scan.ranges.size());
    //("laser scan end");

        if(scan.ranges.size() != 0)
        {
            polar_to_cartesian(input_scan, lidarReading);
            //("lidarReading");
            simplify_data(lidarReading);
            //("split_points");
            split_poins_equally(lidarReading, left_points, right_points, rejected_points);
            //("m_f");
            ROS_INFO("r size %d r size %d ",right_points.size(),rejected_points.size());
            ROS_INFO("r vector; x: %d y %d",right_points[0].x,right_points[0].y);
            ROS_INFO("l vector; x: %d y %d",left_points[0].x,left_points[0].y);
            marczuk_spline(right_points,left_points,angle,velocity);
            //("pub");
        }

}

void marczuk_spline(vector<Point> right_lidar_vec,vector<Point> left_lidar_vec,float &angle, int velocity)
{
    if(right_lidar_vec.size()>5)
    {
        right_lidar_spline.set_spline(right_lidar_vec,false);
        right_line_detect = 1;
    }
    if(left_lidar_vec.size()>5)
    {
        left_lidar_spline.set_spline(left_lidar_vec,false);
        left_line_detect = 1;
    }

    //set path according to lines
    if(left_line_detect == 1 && right_line_detect == 1)
    {
        two_wall_planner(left_lidar_spline,right_lidar_spline,trajectory_path_spline);

        trajectory_tangent.calculate(trajectory_path_spline,0);
        trajectory_tangent.angle();

        middle_tangent.calculate(trajectory_path_spline,trajectory_path_spline.X.back()/2);
        middle_tangent.angle();
    }
    else if(right_line_detect)
    {
        one_wall_planner(right_lidar_spline,-100,trajectory_path_spline);

        trajectory_tangent.calculate(trajectory_path_spline,0);
        trajectory_tangent.angle();

        middle_tangent.calculate(trajectory_path_spline,trajectory_path_spline.X.back()/2);
        middle_tangent.angle();
    }
    else if(left_line_detect)
    {
        one_wall_planner(left_lidar_spline,0,trajectory_path_spline);

        trajectory_tangent.calculate(trajectory_path_spline,0);
        trajectory_tangent.angle();

        middle_tangent.calculate(trajectory_path_spline,trajectory_path_spline.X.back()/2);
        middle_tangent.angle();
    }

        servo_weight = 0.9;
        ang_div = abs(middle_tangent.angle_deg - trajectory_tangent.angle_deg);
        far_tg_fac = 0.8;

    //first condition
    if(ang_div>10)
    {
        velocity = 10/ang_div*2500; //*velocity
        servo_weight = 1;
        far_tg_fac = 0.9;
    }
    else
    {
        uint32_t acceleration = 20;
        velocity = previous_velocity + acceleration;

        if(velocity>5000)
            velocity = 5000;
    }

    previous_velocity = velocity;
    angle = (float)servo_weight*(far_tg_fac*middle_tangent.angle_rad + (1.0-far_tg_fac)*trajectory_tangent.angle_rad);
}
// Funkcja dziląca na prawe i lewe linie
void split_poins_equally(LidarReading &input, std::vector<Point> &left_points, std::vector<Point> &right_points, std::vector<Point> &rejected_points)
{
    left_points.clear();
    right_points.clear();
    rejected_points.clear();

    uint32_t i = 0;
    uint32_t j = input.pos.size();

    bool left_continous = true;
    bool right_continous = true;

    if(input.pos.size() >= 3)
    {
        if(input.pos[input.pos.size()-1].y < - 50)
        {
            left_points.push_back(input.pos[input.pos.size()-1]);
        }
        else
        {
            left_continous = false;
        }

        if(input.pos[0].y < - 50)
        {
            right_points.push_back(input.pos[0]);
        }
        else
        {
            right_continous = false;
        }

        for(i = 1; i < input.pos.size()-1; i++)
        {
            if(right_continous)
            {
                if(sqrt((input.pos[i].x - input.pos[i-1].x)*(input.pos[i].x - input.pos[i-1].x) + (input.pos[i].y - input.pos[i-1].y)*(input.pos[i].y - input.pos[i-1].y)) < THRESH_CONTINOUS)
                    right_points.push_back(input.pos[i]);
                else
                    right_continous = false;
            }

            if(left_continous)
            {
                j = input.pos.size()-1-i;
                if(sqrt((input.pos[j].x - input.pos[j+1].x)*(input.pos[j].x - input.pos[j+1].x) + (input.pos[j].y - input.pos[j+1].y)*(input.pos[j].y - input.pos[j+1].y)) < THRESH_CONTINOUS)
                    left_points.push_back(input.pos[j]);
                else
                    left_continous = false;
            }

            if(input.pos.size()-1-i < i || (!left_continous && !right_continous))
                break;
        }

        int i_int = i;
        int j_int= j;

        if(j_int - i_int >= 0)
        {
            for(uint32_t k = i; k <= j; k++)
                rejected_points.push_back(input.pos[k]);
        }
    }
}

void simplify_data(LidarReading &input_output)
{
    Point tmp, last;
    std::vector<Point> tmp_vec_pos;
    std::vector<double> tmp_vec_angle;
    //("push_vec");
    //("%d", input_output.pos.size());
    tmp_vec_pos.push_back(input_output.pos[0]);
    //("push_angle");
    tmp_vec_angle.push_back(input_output.angle[0]);
    last = input_output.pos[0];
    //("for");
    for(uint32_t i = 1; i < input_output.pos.size() - 1; i++)
    {
        tmp.x = input_output.pos[i].x - last.x;
        tmp.y = input_output.pos[i].y - last.y;
        if(sqrt(tmp.x*tmp.x + tmp.y*tmp.y) > THRESH_SIMPLIFY)
        {
            tmp_vec_pos.push_back(input_output.pos[i]);
            tmp_vec_angle.push_back(input_output.angle[i]);
            last = input_output.pos[i];
        }
    }
    //("input");
    input_output.pos.clear();
    input_output.angle.clear();
    input_output.pos = tmp_vec_pos;
    input_output.angle = tmp_vec_angle;
}
// Funkcja zamieniająca wsp biegunowe na kartezjańskie
void polar_to_cartesian(std::vector<float> &input, LidarReading &output)
{
    Point new_data;
    output.pos.clear();
    output.angle.clear();

    for(uint32_t i = 0; i < input.size(); i++)
    {
        // Obliczenie kąta odczytu
        output.angle.push_back((i * 0.36 + ANGLE_OFFSET));
        // Obliczenie pozycji X, Y
        new_data.x = -cos(output.angle[i]  * (3.14159/180)) * input[i];
        new_data.y = sin(output.angle[i]  * (3.14159/180)) * input[i];

        output.pos.push_back(new_data);
        ROS_INFO("I: %d, A: %f, X;Y: %d ; %d", i, output.angle[i]  * (3.14159/180), output.pos[i].x, output.pos[i].y);
    }
}