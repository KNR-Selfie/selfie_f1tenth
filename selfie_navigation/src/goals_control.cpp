#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <move_base_msgs/MoveBaseActionFeedback.h>
#include <vector>

#include <fstream>
#include <iostream>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int goals_amount = 5;
std::vector < geometry_msgs::Pose > goals_vector;
int goals_received = 0;
geometry_msgs::Pose act_pose;
float distanceToSendNewGoal = 0.5; //m

void feedbackCallback(const move_base_msgs::MoveBaseActionFeedback &msg){
  act_pose.position.x = msg.feedback.base_position.pose.position.x;
  act_pose.position.y = msg.feedback.base_position.pose.position.y;
}

void goalReceiverCallback(const geometry_msgs::PoseStamped &msg)
{
  goals_vector.push_back(msg.pose);
  goals_received++;
  ROS_INFO("I've already saved %i goals", goals_received);
}

float getDistance(int goalID){
  return sqrt(pow((goals_vector[goalID].position.x - act_pose.position.x),2)
     + pow((goals_vector[goalID].position.y - act_pose.position.y),2));
}

int findNearestGoal(){
  float min = 99999999;
  int index=0;

  for(int  i = 0; i < goals_amount; i++){
     float distance = getDistance(i);
     if(distance<min){
       min=distance;
       index=i;
     }
  }
  return index;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "goals_control");

  ros::NodeHandle n("~");

  //ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 50, goalReceiverCallback);
  ros::Subscriber feedback_goal = n.subscribe("/move_base/feedback", 50, feedbackCallback);

  //save goals
  /*
  ROS_INFO("Waiting for %d goals",goals_amount);
  while(goals_amount > goals_received && n.ok()){
    ros::spinOnce();
  }*/

  //save goals from file
  
  std::fstream file;
  file.open("src/selfie_f1tenth/selfie_navigation/goals/goals.txt", std::ios::in);
  if(file.good() == true){
      file >> goals_amount;
      for(int i = 0; i< goals_amount; i++)
      {
          geometry_msgs::Pose temp;
          file >> temp.position.x;
          file >> temp.position.y;
          file >> temp.position.z;
          file >> temp.orientation.x;
          file >> temp.orientation.y;
          file >> temp.orientation.z;
          file >> temp.orientation.w;
          goals_vector.push_back(temp);
      }
      ROS_INFO("Goals sent");
  }
  else{
      ROS_INFO("Couldn't load file");
  }
  file.close();

  //get actual pose
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;


  while(ros::ok()){
    try{
    transformStamped = tfBuffer.lookupTransform("map", "base_link",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
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
  
  int act_goalToCheckDistance;
  int act_goal = (findNearestGoal() + 2) % goals_amount;

  //info printing
  ROS_INFO("actPOSE IS\n X: %.2f \n Y: %.2f", act_pose.position.x, act_pose.position.y);
  ROS_INFO("nearestGOAL IS\n id: %d\nX: %.2f \n Y: %.2f",act_goal, goals_vector[act_goal].position.x, goals_vector[act_goal].position.y);

  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
   }

  //send goals
  float distanceToGoal=0;
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  while(ros::ok()){
    act_goal = act_goal % goals_amount;
    act_goalToCheckDistance = (act_goal + goals_amount - 1) % goals_amount;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goals_vector[act_goal];
    ROS_INFO("Sending goal %d",act_goal);
    ac.sendGoal(goal);
    ros::Rate loop_rate(1);
    while(n.ok())
    {
      ros::spinOnce();
      distanceToGoal=getDistance(act_goalToCheckDistance);
      if(distanceToGoal < distanceToSendNewGoal)
        break;
      loop_rate.sleep();
    }
    act_goal++;
  }

  return 0;
}
