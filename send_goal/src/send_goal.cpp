#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <actionlib/client/simple_action_client.h>
#include<iostream>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){

  ros::init(argc, argv, "send_goal_node");
  double goal_position_x;
  double goal_position_y;
  double goal_position_yaw;
  ros::NodeHandle nh;
  nh.getParam("position_x", goal_position_x);
  nh.getParam("position_y", goal_position_y);
  nh.getParam("position_yaw", goal_position_yaw);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  
  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  
  //send a goal to the robot

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = goal_position_x;
  goal.target_pose.pose.position.y = goal_position_y;
  goal.target_pose.pose.position.z = 0.0;
  goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_position_yaw);
  ROS_INFO("Sending goal");

  ac.sendGoal(goal);
  ac.waitForResult();
  
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){

    ROS_INFO("Hooray, goal reached");
  }
  else{
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  }
  return 0;
}
