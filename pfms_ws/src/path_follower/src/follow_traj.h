/**
 * @file follow_traj.h
 * @author Abdelghani Saeed (12747150@uts.student.uts.edu.au)
 * @brief follow_traj headerfile
 * @version 1.0
 * @date 07-11-2020
 *
 */

#ifndef FOLLOW_TRAJ_H_
#define FOLLOW_TRAJ_H_

#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <mutex>
#include <thread>
#include "a4_setup/RequestGoal.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "navigation_processing.h"



class FollowTrajectory
{
public:
  FollowTrajectory(ros::NodeHandle &nodehandle);
  ~FollowTrajectory();

  /**
   * @brief Main loop
   *
   */
  void seperateThread();
  /**
   * @brief Function to publish goals to robot for navigation
   *
   */
  void pub_goals_as_path();


  // Robot callback functions

  /**
   * @brief Robot 0's odom callback function
   *
   * @param const nav_msgs::Odometry::ConstPtr &msg
   */
  void robot0PoseCB(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief Robot 1's odom callback function
   * This callback will also track and add robot_1 pose as a intermediate checkpoint
   *
   * @param const nav_msgs::Odometry::ConstPtr &msg
   */
  void robot1PoseCB(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief Robot 0's Laser Scanner callback function
   *
   * @param const sensor_msgs::LaserScan::ConstPtr &msgLsr
   */
  void robot0ScanCB(const sensor_msgs::LaserScan::ConstPtr &msgLsr);


  // Functions for checking line of sight

  /**
   * @brief This function uses the service "face_goal" to determine if two poses are in line of sight
   *
   * @param nav_msgs::Odometry start_pose
   * @param nav_msgs::Odometry target_pose
   * @return bool
   */
  bool goal_in_sight(nav_msgs::Odometry start_pose, nav_msgs::Odometry target_pose);


  /**
   * @brief This function will generate searching points on eitherside of target with predefined offset
   *
   * @param nav_msgs::Odometry input_pose
   * @return std::vector<geometry_msgs::PoseStamped>
   */
  std::vector<geometry_msgs::PoseStamped> computeSearchPoints(nav_msgs::Odometry input_pose);


  // Navigation functions
  /**
   * @brief To set a given target pose as a navigation target
   *
   * @param nav_msgs::Odometry target_pose
   */
  void set_pose_as_goal(nav_msgs::Odometry target_pose);

  /**
   * @brief To determine whether or not R0 reached the first pose in the goal_list "visualization_goal_"
   *
   * @return bool
   */
  bool is_robot_at_goal();

  /**
   * @brief Function to send empty path list to stop robot motion
   */
  void pub_empty_goal_as_path();

  /**
   * @brief Function to remove first element (oldest goal) from the goal list
   *
   */
  void rm_1st_goal_in_list();

  /**
   * @brief Function to convert poses to pose array
   *
   * @param std::vector<geometry_msgs::PoseStamped> pose
   * @return geometry_msgs::PoseArray
   */
  geometry_msgs::PoseArray producePoseArray(std::vector<geometry_msgs::PoseStamped> pose);

  // Visualization functions

  /**
   * @brief Function to convert poses to marker array
   *
   * @param std::vector<geometry_msgs::PoseStamped> pose
   * @return visualization_msgs::MarkerArray
   */
  visualization_msgs::MarkerArray produceMarkerArray(std::vector<geometry_msgs::PoseStamped> pose);

  /**
   * @brief Function to publish goal list as markers for visualization
   *
   */
  void pub_goals_as_marker();


  /**
   * @brief Function to add tracked R1 poses into goal list
   *
   */
  void append_goals_as_marker();

  // Utility function
  /**
   * @brief To check the received_goal_ vector length
   *
   * @return int
   */
  int target_goal_length();

private:

  /**
   * @brief Shared resources struct
   *
   */
  struct OdomDataBuffer
  {
    nav_msgs::Odometry odom;  //! Robot Odometry
    std::mutex mtx;           //! Mutex
  };

  OdomDataBuffer robot0_odom_buff_;              //! To store R0 pose
  OdomDataBuffer robot1_odom_buff_;              //! To store R1 pose
  nav_msgs::Odometry robot1_odom_shifted_pose_;  //! To store shifted R1 pose
  nav_msgs::Odometry prev_communicated_pose_;    //! To store last goal pose
  nav_msgs::Odometry r1_prev_recorded_pose_;     //! To store R1 previous pose for checking purposes

  ros::NodeHandle nh_, pnh_;
  NavigationProcessing *nav_proc_;  //! Pointer to NavigationProcessing class
  ros::Publisher goal_path_pub_, marker_viz_pub_;  // Publishers
  ros::Subscriber robot1_odom_sub_, robot0_odom_sub_, robot0_basescan_, follow_goal_subs_, local_map_sub_; // Subscribers
  ros::ServiceClient check_goal_client_;  // Service client
  std::vector<geometry_msgs::PoseStamped> received_goal_;       //! To store intermediate pose
  std::vector<geometry_msgs::PoseStamped> visualization_goal_;  //! Actual navigation goal poses

  bool start_following_;              //! Flag to determine when to start following R1
  bool r0_wait_for_first_sight_;      //! Flag to indicate when R1 is first seen
  bool advance_mode_;                 //! Flag to determine follow mode for R0 (Normal/Advance)
  bool robot1_is_stationary_;         //! Flag to determine if R1 is in stationary mode
  bool robot0_in_assit_mode_;         //! Flag to determine if R0 is in asistance mode
  bool send_goal_now;                 //! Flag to send saved intermediate poses
  ros::Time stationary_start_stamp_;  //! To store and record the timestamp
};

#endif
