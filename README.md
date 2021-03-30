# Final Project - Home Service Robot

This repo contains the final project of the Robotics Software Engineer Nanodegree Program from Udacity. The program shows a home service robot which can perform SLAM (Simultaneous Localization and Mapping) using [gmapping](http://wiki.ros.org/gmapping). The mobile robot navigates autonomously from its initial position to the pickup point and then to a drop off point.

Below you can see the simulation in Rviz:

![rviz_example](media/rviz.gif)

The image below shows the robot in gazebo simulator:

![gazebo_example](media/gazebo.gif)


## Dependencies for Running
* cmake >= 3.5.1
  * Linux: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac)
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
* gazebo simulator >= 11.0.0
  * Linux: [gazebo download](http://gazebosim.org/download)
* ROS >= Kinetic
  *  Linux: [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

## Basic Running Instructions

1. Clone this repo.
2. Open top level directory: `cd RobotND-HomeServiceRobot/catkin_ws`
3. Build the files: `catkin_make`
4. Execute in terminal: `source devel/setup.bash`
5. Run the simulation file located in the scripts folder: `./src/script/home_service.sh` (this file must have execution permission)

There are other options for testing:
* `test_slam.sh`: Allows to generate the map through navigation of the robot with keyboard commands.`RobotND-HomeServiceRobot/catkin_ws/src/teleop_twist_keyboard/teleop_twist_keyboard.py` must have execution permission.
* `test_navigation.sh`: The robot navigates in the map with a goal indicated in Rviz (2D Nav Goal tool).
* `test_navigation_to_goal.sh`: The robot navigates through the map to a pickup point and then to a drop off point.
* `test_markers.sh`: A cube objects appears in the pickup zone for 5 seconds, disappear for 5 seconds and then it appears again in the drop off zone.

## Project Packages
* **Localization:** [amcl](http://wiki.ros.org/amcl) is a probabilistic localization system for a robot moving in 2D. It implements the adaptive Monte Carlo localization approach, which uses a particle filter to track the pose of a robot against a known map.
* **Mapping:** The [gmapping](http://wiki.ros.org/gmapping) package provides laser-based SLAM (Simultaneous Localization and Mapping), as a ROS node called slam_gmapping. Using slam_gmapping, you can create a 2-D occupancy grid map (like a building floorplan) from laser and pose data collected by a mobile robot.
* **Navigation:** The [ROS navigation stack](http://wiki.ros.org/navigation/Tutorials/SendingSimpleGoals) creates a path for the robot based on Dijkstra's algorithm, a variant of the Uniform Cost Search algorithm, while avoiding obstacles on its path.
* **Teleoperation:** [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard) is Generic Keyboard Teleop for ROS.
* **Rviz configuration:** Launchers for (visualizing TurtleBot)[http://wiki.ros.org/turtlebot_rviz_launchers]
* **Markers in Rviz:** [Basic Shapes (C++)](http://wiki.ros.org/rviz/Tutorials/Markers%3A%20Basic%20Shapes) shows how to use `visualization_msgs/Marker` messages to send basic shapes (cube, sphere, cylinder, arrow) to rviz.
