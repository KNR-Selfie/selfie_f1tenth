#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

const int goals_amount = 4;
geometry_msgs::Pose goals_tab[goals_amount];
int goals_received = 0;
int actual_goal_number = 0;

void goalReceiverCallback(const geometry_msgs::PoseStamped &msg)
{
  goals_tab[goals_received]=msg.pose;
  goals_received++;
  ROS_INFO("I've already saved %i goals", goals_received);
}

int findNearestGoal(geometry_msgs::Pose act_pose){
  float min = 99999999;
  int index=0;

  for(int  i = 0; i < goals_amount; i++){
     float distance = sqrt(pow((goals_tab[i].position.x - act_pose.position.x),2)
     + pow((goals_tab[i].position.y - act_pose.position.y),2));
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

  ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 50, goalReceiverCallback);

  //save goals
  while(goals_amount > goals_received && n.ok()){
    ros::spinOnce();
  }

  //get actual pose
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::Pose act_pose;

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

  int act_goal = findNearestGoal(act_pose);

  //info printing
  ROS_INFO("actPOSE IS\n X: %.2f \n Y: %.2f", act_pose.position.x, act_pose.position.y);
  ROS_INFO("nearestGOAL IS\n id: %d\nX: %.2f \n Y: %.2f",act_goal, goals_tab[act_goal].position.x, goals_tab[act_goal].position.y);

  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
   }

  //send goals
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  while(ros::ok()){
    act_goal = act_goal%goals_amount;
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = goals_tab[act_goal];
    ROS_INFO("Sending goal %d",act_goal);
    ac.sendGoal(goal);
    ac.waitForResult();
    act_goal++;
  }
  
  return 0;
}
