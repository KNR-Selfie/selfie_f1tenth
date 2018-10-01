#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <math.h>

sensor_msgs::LaserScan scan;

ros::Publisher offset_pub;
float min_angle = M_PI/2;
float read_angle_range = M_PI/4;
std_msgs::Float64 offset;
float last_offset = 0;

void scanCallback(const sensor_msgs::LaserScan &msg){
    scan = msg;
    last_offset = offset.data;
    float right_read_angle = scan.angle_max;
    int left_read_angle_index = 0;
    int right_read_angle_index = 0;
    for(right_read_angle; right_read_angle >= scan.angle_min; right_read_angle -= scan.angle_increment){
        right_read_angle_index++;
    }
    float middle_sum = 0;
    float front_sum = 0;
    int counter = 0;
    right_read_angle_index = right_read_angle_index - ((scan.angle_max - min_angle) / scan.angle_increment);
    left_read_angle_index = left_read_angle_index - ((scan.angle_min - min_angle )/ scan.angle_increment);
    right_read_angle = min_angle;
    float left_read_angle = -1 * min_angle;
    for (float angle_counter = 0; angle_counter <= read_angle_range; angle_counter += scan.angle_increment){
        if((scan.ranges[right_read_angle_index] >= scan.range_min && scan.ranges[right_read_angle_index] <= scan.range_max) 
        && (scan.ranges[left_read_angle_index] >= scan.range_min && scan.ranges[left_read_angle_index] <= scan.range_max)){
            float xPLUSy = sqrt(scan.ranges[left_read_angle_index]*scan.ranges[left_read_angle_index]+scan.ranges[right_read_angle_index]*scan.ranges[right_read_angle_index]
            -2*scan.ranges[left_read_angle_index]*scan.ranges[right_read_angle_index]*cos(2*right_read_angle));
            float temp_angle = asin(scan.ranges[left_read_angle_index]*sin(2*right_read_angle)/xPLUSy);
            float temo_angle2 = M_PI - temp_angle - right_read_angle;
            float x = scan.ranges[right_read_angle_index] * sin(right_read_angle) / sin(temo_angle2);
            front_sum += x;
            middle_sum += xPLUSy / 2;
            counter++;
        }
        right_read_angle_index--;
        left_read_angle_index++;
        right_read_angle -= scan.angle_increment;
        left_read_angle += scan.angle_increment;
    }
    
    float avr_front = front_sum / counter;
    float avr_middle = middle_sum / counter;
    offset.data = avr_front - avr_middle;
    if(!(offset.data > -10 && offset.data < 10))
        offset.data = last_offset;  
    offset_pub.publish(offset);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "selfie_simple_laser_control");

    ros::NodeHandle n("~");
    offset.data = 0;

    offset_pub = n.advertise<std_msgs::Float64>("/steering_state", 50);
    ros::Subscriber sub_scan = n.subscribe("/scan", 50, scanCallback);
    
    ros::Rate loop_rate(15);
    while (n.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}