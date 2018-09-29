#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

geometry_msgs::Pose act_pose;
sensor_msgs::LaserScan scan;
nav_msgs::OccupancyGrid map;
geometry_msgs::TransformStamped transformStamped;
bool map_initialized = false;
double roll,pitch,yaw;

float wall_obstacle_error = 0.4; //m
int wall_obstacle_error_iterations;

ros::Publisher obstacles_pub;
tf2_ros::Buffer tfBuffer;

int get_index(int x, int y);
void get_act_pose();

void scanCallback(const sensor_msgs::LaserScan &msg){
    if(map_initialized){
        scan = msg;
        float read_angle = scan.angle_max;
        int i = 0;
        for(read_angle; read_angle >= scan.angle_min; read_angle -= scan.angle_increment){
            if(scan.ranges[i] >= scan.range_min && scan.ranges[i] <= scan.range_max){

                float temp_scan_range = scan.ranges[i] - wall_obstacle_error / 2;
                bool wall = false;
                for(int j = 0;j <= wall_obstacle_error_iterations; j++){

                    float x = act_pose.position.x + temp_scan_range * cos(yaw + read_angle);
                    float y = act_pose.position.y + temp_scan_range * sin(yaw + read_angle);
                    int x_index, y_index;
 
                    if(x<0){
                        x_index = -1 * (map.info.origin.position.x - x) / map.info.resolution;
                    }
                    else{
                        x_index = -1 * map.info.origin.position.x / map.info.resolution + x / map.info.resolution;
                    }
                    if(y<0){
                        y_index = -1 * (map.info.origin.position.y - y) / map.info.resolution;
                    }
                    else{
                        y_index = -1 * map.info.origin.position.y / map.info.resolution + y / map.info.resolution;
                    }
                    if(map.data[get_index(x_index,y_index)] == 100){
                        wall = true;
                        break;
                    }
                    temp_scan_range += map.info.resolution;
                }
                if(wall == true){
                    scan.ranges[i] = scan.range_max + 1;
                }
            }
            i++;
        }
        obstacles_pub.publish(scan);
    }
}

void mapCallback(const nav_msgs::OccupancyGrid &msg){
    map = msg;
    map_initialized = true;
    wall_obstacle_error_iterations = int(wall_obstacle_error / map.info.resolution);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "selfie_obstacles_filter");

    ros::NodeHandle n("~");

    obstacles_pub = n.advertise<sensor_msgs::LaserScan>("/obstacles_scan", 50);
    ros::Subscriber sub_scan = n.subscribe("/scan", 50, scanCallback);
    ros::Subscriber sub_map = n.subscribe("/map", 50, mapCallback);

    n.param<float>("error", wall_obstacle_error, 0.4);
    ROS_INFO("error is: %.2f", wall_obstacle_error);

    tf2_ros::TransformListener tfListener(tfBuffer);
    get_act_pose();
    
    ros::Rate loop_rate(15);
    while (n.ok())
    {
        ros::spinOnce();
        get_act_pose();
        loop_rate.sleep();
    }

    return 0;
}

int get_index(int x, int y){
    return y * map.info.height + x;
}

void get_act_pose(){
    while(ros::ok()){
        try{
        transformStamped = tfBuffer.lookupTransform("map", "base_link",ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            continue;
        }
    break;
    }

    act_pose.position.x = transformStamped.transform.translation.x;
    act_pose.position.y = transformStamped.transform.translation.y;
    act_pose.position.z = transformStamped.transform.translation.z;
    act_pose.orientation.x = transformStamped.transform.rotation.x;
    act_pose.orientation.y = transformStamped.transform.rotation.y;
    act_pose.orientation.z = transformStamped.transform.rotation.z;
    act_pose.orientation.w = transformStamped.transform.rotation.w;

    tf::Quaternion q(
        act_pose.orientation.x,
        act_pose.orientation.y,
        act_pose.orientation.z,
        act_pose.orientation.w);
    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}