#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <vector>
#include <math.h>

// position variables
float robot_x, robot_y;
// define goal points
std::vector<std::vector<double>> goals = {{4.0,-6.0}, {1.3,-7.0}};
float delta = 0.5;
enum state {goal_1, goal_hide, goal_2};

void get_pos_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) 
{
    robot_x = odom_msg->pose.pose.position.x;
    robot_y = odom_msg->pose.pose.position.y;
}

float error(float x1, float y1, float x2, float y2)
{
    return sqrt( pow( x1 - x2, 2 ) + pow( y1 - y2, 2 ) );
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("odom", 10, get_pos_callback);
	state object_state = state::goal_1;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

	visualization_msgs::Marker marker;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();

  while (ros::ok())
  {  
    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    switch (object_state)
    {
			ROS_INFO("Pose x: %f, Pose y: %f", robot_x, robot_y);
			case state::goal_1:
				ROS_INFO("Moving to goal 1");
			  // Set first pose of the marker
  			marker.pose.position.x = goals[0][0];
  			marker.pose.position.y = goals[0][1];
				marker_pub.publish(marker); // publish marker	
				// compare robot pose with first goal
				if (error(goals[0][0], goals[0][1], robot_x, robot_y) < delta)
				{
					ROS_INFO("Goal 1 reached");
					object_state = state::goal_hide;
				}
				break;
			case state::goal_hide:
				ROS_INFO("Moving to goal 2");
				marker.color.a = 0.0;       // hide marker
    		marker_pub.publish(marker); // publish marker
				// compare robot pose with second goal
				if (error(goals[1][0], goals[1][1], robot_x, robot_y) < delta)
				{
					ROS_INFO("Goal 2 reached");
					object_state = state::goal_2;
				}
				break;
			case state::goal_2:
				ROS_INFO("Goal 2 reached");
			  marker.pose.position.x = goals[1][0];
    		marker.pose.position.y = goals[1][1];
    		marker.color.a = 1.0;       // hide marker
    		marker_pub.publish(marker); // publish marker
				break;
		}
		ros::spinOnce();
		r.sleep();
  }
}