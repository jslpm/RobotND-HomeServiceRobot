#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <vector>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// define goal points
std::vector<std::vector<double> > goals = {{4.0,-6.0,0.0,1.0}, {1.3,-7,0.707,0.707}};

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  for (int i = 0; i < goals.size(); i++) {
    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = goals[i][0];
    goal.target_pose.pose.position.y = goals[i][1];
    goal.target_pose.pose.orientation.z = goals[i][2];
    goal.target_pose.pose.orientation.w = goals[i][3];
  
   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, the base moved to the goal");
  else
    ROS_INFO("The base failed to move to the goal for some reason");

  ROS_INFO("Waiting 5 seconds...");
  ros::Duration(5).sleep(); // sellep for 5 second

  }
  
  ROS_INFO("The cycle is over");
  
  return 0;
}