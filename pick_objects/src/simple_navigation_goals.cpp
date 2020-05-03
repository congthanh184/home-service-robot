#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <string>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
    MoveBaseClient;

void sendGoalToMoveBase(MoveBaseClient& ac, move_base_msgs::MoveBaseGoal& goal,
                        char* goalName) {
  ROS_INFO("Sending %s", goalName);
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

  // tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // wait for the action server to come up
  while (!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // we'll send a goal to the robot to move 1 meter forward

  goal.target_pose.pose.position.x = -2.0;
  goal.target_pose.pose.orientation.w = 1.0;

  sendGoalToMoveBase(ac, goal, (char*)"pick up zone");

  ROS_INFO("Wait for 5 seconds");
  ros::Duration(5).sleep();

  goal.target_pose.pose.position.x = -3.0;
  goal.target_pose.pose.position.y = 3.0;
  goal.target_pose.pose.orientation.w = 1.0;
  sendGoalToMoveBase(ac, goal, (char*)"drop off zone");

  ros::spin();
  return 0;
}
