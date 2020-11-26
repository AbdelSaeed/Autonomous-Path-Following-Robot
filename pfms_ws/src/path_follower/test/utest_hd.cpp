/**
 * @file utest_hd.cpp
 * @author Abdelghani Saeed (12747150@uts.student.uts.edu.au)
 * @brief Unit testing NavigationProcessing::computeAssitModePoses(nav_msgs::Odometry) member function from NavigationProcessing class
 * @version 1.0
 * @date 07-11-2020
 *
 */

#include <gtest/gtest.h>
#include <climits>
#include <ros/ros.h>

// This tool allows to identify the path of the package on your system
#include <ros/package.h>

// These allow us to inspect bags of data
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include "../src/navigation_processing.h"

TEST(GenerateAssitPose, AssitPoseTest)
{
  NavigationProcessing* nav_proc_ = new NavigationProcessing();

  // Vector to store poses to navigate R0 to the left side of R1
  std::vector<geometry_msgs::PoseStamped> assit_mode_poses;

  // Predefined pose
  nav_msgs::Odometry target_odom;
  target_odom.pose.pose.position.x = 0.0;
  target_odom.pose.pose.position.y = 0.0;
  target_odom.pose.pose.position.z = 0.0;

  target_odom.pose.pose.orientation.x = 0.0;
  target_odom.pose.pose.orientation.y = 0.0;
  target_odom.pose.pose.orientation.z = 0.0;
  target_odom.pose.pose.orientation.w = 1.0;

  // Expecting the computed pose to be to the left of the input pose (+0.5m in y-axis)
  nav_msgs::Odometry expected_odom_pose;
  expected_odom_pose.pose.pose.position.x = 0.0;
  expected_odom_pose.pose.pose.position.y = 0.5;
  expected_odom_pose.pose.pose.position.z = 0.0;

  // Orientation expected to remain the same
  expected_odom_pose.pose.pose.orientation.x = 0.0;
  expected_odom_pose.pose.pose.orientation.y = 0.0;
  expected_odom_pose.pose.pose.orientation.z = 0.0;
  expected_odom_pose.pose.pose.orientation.w = 1.0;

  // Vector consist of 2 pose : index[0] is an intermediate pose , index[1] is the final pose
  assit_mode_poses = nav_proc_->computeAssitModePoses(target_odom);

  // ASSERT_EQ(expected value, computed value) << "The computed value x's failed";
  ASSERT_EQ(expected_odom_pose.pose.pose.position.x, assit_mode_poses[1].pose.position.x) << "Assist pose.position.x's "
                                                                                             "failed";
  ASSERT_EQ(expected_odom_pose.pose.pose.position.y, assit_mode_poses[1].pose.position.y) << "Assist pose.position.y's "
                                                                                             "failed";
  ASSERT_EQ(expected_odom_pose.pose.pose.position.z, assit_mode_poses[1].pose.position.z) << "Assist pose.position.z's "
                                                                                             "failed";
  ASSERT_EQ(expected_odom_pose.pose.pose.orientation.x, assit_mode_poses[1].pose.orientation.x)
      << "Assist pose.orientation.x's failed";
  ASSERT_EQ(expected_odom_pose.pose.pose.orientation.y, assit_mode_poses[1].pose.orientation.y)
      << "Assist pose.orientation.y's failed";
  ASSERT_EQ(expected_odom_pose.pose.pose.orientation.z, assit_mode_poses[1].pose.orientation.z)
      << "Assist pose.orientation.z's failed";
  ASSERT_EQ(expected_odom_pose.pose.pose.orientation.z, assit_mode_poses[1].pose.orientation.z)
      << "Assist pose.orientation.w's failed";
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
