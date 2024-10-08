#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <move_base_msgs/MoveBaseAction.h>

int main (int argc, char **argv)
{
  ros::init(argc, argv, "waypoint_test_node");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("/move_base", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 1.53126275539;
  goal.target_pose.pose.position.y = 4.52530193329;

  goal.target_pose.pose.orientation.z = -0.51;
  goal.target_pose.pose.orientation.w = 0.85;
  
  ac.sendGoal(goal);

  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  //exit
  return 0;
}

