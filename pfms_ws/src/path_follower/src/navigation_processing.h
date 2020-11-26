/**
 * @file navigation_processing.h
 * @author Abdelghani Saeed (12747150@uts.student.uts.edu.au)
 * @brief Helper functions for the follow trajectory project
 * @version 1.0
 * @date 07-11-2020
 *
 */


#ifndef NAVIGATION_PROCESSING_H
#define NAVIGATION_PROCESSING_H

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include "nav_msgs/Odometry.h"


class NavigationProcessing
{
public:
  NavigationProcessing();

public:

  /**
   * @brief Utility Function to compute distance between two poses
   *
   * @param nav_msgs::Odometry pose1
   * @param nav_msgs::Odometry pose2
   * @return double
   */
  double compute_dist_between_poses(nav_msgs::Odometry pose1, nav_msgs::Odometry pose2);

  /**
   * @brief Utility Function to compute scalar distance
   *
   * @param x1
   * @param y1
   * @param x2
   * @param y2
   * @return double
   */
  double compute_dist(double x1, double y1, double x2, double y2);

  /**
   * @brief Utility Function to convert poses between frames and transform manipulation
   *
   * @param nav_msgs::Odometry pose1
   * @param geometry_msgs::PoseStamped pose_shift
   * @return geometry_msgs::PoseStamped
   */
  geometry_msgs::PoseStamped transform_goal_frame(nav_msgs::Odometry pose1, geometry_msgs::PoseStamped pose_shift);

  /**
   * @brief Utility Function to compute assistance mode poses for R0
   *
   * @param nav_msgs::Odometry robot1_stationary_odom
   * @return std::vector<geometry_msgs::PoseStamped>
   */
  std::vector<geometry_msgs::PoseStamped> computeAssitModePoses(nav_msgs::Odometry robot1_stationary_odom);
};

#endif
