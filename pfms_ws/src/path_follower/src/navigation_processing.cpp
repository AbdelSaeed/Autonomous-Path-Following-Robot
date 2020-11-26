/**
 * @file navigation_processing.h
 * @author Abdelghani Saeed (12747150@uts.student.uts.edu.au)
 * @brief Helper functions for the follow trajectory project
 * @version 1.0
 * @date 07-11-2020
 *
 */

#include "navigation_processing.h"

NavigationProcessing::NavigationProcessing(){ }

// Utility Function to compute distance between two poses
double
NavigationProcessing::compute_dist_between_poses(nav_msgs::Odometry pose1, nav_msgs::Odometry pose2)
{
  double x1 = pose1.pose.pose.position.x;
  double y1 = pose1.pose.pose.position.y;
  double x2 = pose2.pose.pose.position.x;
  double y2 = pose2.pose.pose.position.y;
  return compute_dist(x1, y1, x2, y2);
}


// Utility Function to compute scalar distance
double
NavigationProcessing::compute_dist(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}


// Utility Function to convert poses between frames with the assistance of Eigen library
geometry_msgs::PoseStamped
NavigationProcessing::transform_goal_frame(nav_msgs::Odometry pose1, geometry_msgs::PoseStamped pose_shift)
{
  geometry_msgs::PoseStamped odom_T_robot;                  // Input poses to PoseStamped
  geometry_msgs::PoseStamped odom_T_shifted;
  geometry_msgs::PoseStamped robot_T_shift;
  Eigen::Affine3d odom_T_robot_3d;                          // 4x4 robot pose
  Eigen::Affine3d robot_T_shift_3d;                         // 4x4 shift pose
  Eigen::Affine3d odom_T_shifted_3d;                        // 4x4 shifted desired pose

  odom_T_robot.pose = pose1.pose.pose;                      // Input pose
  robot_T_shift.pose.position = pose_shift.pose.position;   // Input shift pose

  tf::poseMsgToEigen(odom_T_robot.pose, odom_T_robot_3d);   // Convert robot pose to 4x4 transform matrix
  tf::poseMsgToEigen(robot_T_shift.pose, robot_T_shift_3d); // Convert shifted pose to 4x4 transform matrix

  odom_T_shifted_3d = odom_T_robot_3d * robot_T_shift_3d;   // Matrix multiplication to obtain desired shifted pose in world frame

  tf::poseEigenToMsg(odom_T_shifted_3d, odom_T_shifted.pose); // Convert back to normal pose consisting of position and orientation components
  return odom_T_shifted;
}


// Utility Function to compute assistance mode poses for robot_0
std::vector<geometry_msgs::PoseStamped>
NavigationProcessing::computeAssitModePoses(nav_msgs::Odometry robot1_stationary_odom)
{
  geometry_msgs::PoseStamped shift_pose1;
  geometry_msgs::PoseStamped shift_pose2;

  shift_pose1.pose.position.x = -0.50;  // Pose thats behind R1 and offset 0.5m in y-axis
  shift_pose1.pose.position.y = 0.50;

  shift_pose2.pose.position.x = 0.00;   // Pose thats in same location as R1 in x-axis but offset by 0.5m in y-axis
  shift_pose2.pose.position.y = 0.50;

  std::vector<geometry_msgs::PoseStamped> assit_mode_poses;                               // Container to hold assist mode poses
  assit_mode_poses.push_back(transform_goal_frame(robot1_stationary_odom, shift_pose1));  // Pass in R1 pose and the desired XY shift to obtain the goal pose
  assit_mode_poses.push_back(transform_goal_frame(robot1_stationary_odom, shift_pose2));

  return assit_mode_poses; // Return poses for execution
}
