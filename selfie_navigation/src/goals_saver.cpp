#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <iostream>

int goals_amount = 7;
geometry_msgs::Pose goal;
int goals_received = 0;
std::ofstream file("src/selfie_f1tenth/selfie_navigation/goals/goals.txt");

void goalReceiverCallback(const geometry_msgs::PoseStamped &msg)
{
    goal=msg.pose;
    goals_received++;
    ROS_INFO("I've already saved %i goals", goals_received);
    file<<goal.position.x<<std::endl;
    file<<goal.position.y<<std::endl;
    file<<goal.position.z<<std::endl;
    file<<goal.orientation.x<<std::endl;
    file<<goal.orientation.y<<std::endl;
    file<<goal.orientation.z<<std::endl;
    file<<goal.orientation.w<<std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "goals_saver");
    ros::NodeHandle n("~");

    ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 50, goalReceiverCallback);


    file<<goals_amount<<std::endl;

    //save goals
    ROS_INFO("Waiting for %d goals",goals_amount);
    while(goals_amount > goals_received && n.ok()){
      ros::spinOnce();
    }
    file.close();
    return 0;
}
