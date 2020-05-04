#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <string>
#include "add_markers/AddMarker.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

ros::ServiceClient markerClient;

void requestMarker(float x, float y, std::string action) {
  add_markers::AddMarker srv;
  srv.request.x = x;
  srv.request.y = y;
  srv.request.action = action;

  if (!markerClient.call(srv)) {
    ROS_INFO("Error calling service");
  }
}

void sendGoalToMoveBase(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal,
                        char* goalName) {
  ROS_INFO("Drive Robot to %s", goalName);


  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Successfully reach %s", goalName);
  else
    ROS_INFO("Failed to reach %s", goalName);

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_goal");
  ros::NodeHandle n("~");

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  markerClient = n.serviceClient<add_markers::AddMarker>("/add_markers/add_marker");

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // we'll send a goal to the robot to move 1 meter forward

  goal.target_pose.pose.position.x = -6;
  goal.target_pose.pose.position.y = -1.5;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = -0.8778;
  goal.target_pose.pose.orientation.w = 0.4789;

  requestMarker(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, "PLACE");
  sendGoalToMoveBase(ac, goal, (char*)"pick up zone");
  requestMarker(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, "PICK");

  ROS_INFO("Wait for 5 seconds");
  ros::Duration(5).sleep();

  goal.target_pose.pose.position.x = 1.0;
  goal.target_pose.pose.position.y = 2.5;
  goal.target_pose.pose.orientation.x = 0;
  goal.target_pose.pose.orientation.y = 0;
  goal.target_pose.pose.orientation.z = 0;
  goal.target_pose.pose.orientation.w = 1.0;
  sendGoalToMoveBase(ac, goal, (char*)"drop off zone");
  requestMarker(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, "PLACE");

  ros::spin();
  return 0;
}
