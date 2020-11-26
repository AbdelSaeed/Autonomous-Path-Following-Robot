
/**
 * @file follow_traj.cpp
 * @author Abdelghani Saeed ((12747150@uts.student.uts.edu.au)
 * @brief follow_traj implementation file
 * @version 1.0
 * *
 */

#include "follow_traj.h"
using namespace std;


FollowTrajectory::FollowTrajectory(ros::NodeHandle &nodehandle) :
  nh_(nodehandle),
  pnh_("~"),
  start_following_(false),
  r0_wait_for_first_sight_(true),
  advance_mode_(false),
  robot1_is_stationary_(false),
  robot0_in_assit_mode_(false),
  send_goal_now(false)

{
  robot1_odom_sub_ = nh_.subscribe("/robot_1/odom", 100, &FollowTrajectory::robot1PoseCB, this);
  robot0_odom_sub_ = nh_.subscribe("/robot_0/odom", 100, &FollowTrajectory::robot0PoseCB, this);
  robot0_basescan_ = nh_.subscribe("/robot_0/base_scan", 1, &FollowTrajectory::robot0ScanCB, this);

  goal_path_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/robot_0/path", 3, false);
  marker_viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/robot_0/following", 3, false);

  check_goal_client_ = nh_.serviceClient<a4_setup::RequestGoal>("face_goal");

  r1_prev_recorded_pose_.pose.pose.position.x = 1000;  // to make conditions invalid at start (r1 is moving)
  r1_prev_recorded_pose_.pose.pose.position.y = 1000;

  //  Advance Mode
  pnh_.getParam("advance", advance_mode_);

  if (advance_mode_)
  {
    ROS_INFO("Robot_0 Mode: Advance ");
  }
  else
  {
    ROS_INFO("Robot_0 Mode: Normal ");
  }
}


FollowTrajectory::~FollowTrajectory(){  }


void
FollowTrajectory::robot0PoseCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  std::unique_lock<std::mutex> r0_lck(robot0_odom_buff_.mtx);
  robot0_odom_buff_.odom.pose.pose.position = msg->pose.pose.position;
  robot0_odom_buff_.odom.pose.pose.orientation = msg->pose.pose.orientation;
}


void
FollowTrajectory::robot1PoseCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  // Store R1 pose
  std::unique_lock<std::mutex> r1_lck(robot1_odom_buff_.mtx);
  robot1_odom_buff_.odom.pose.pose.position = msg->pose.pose.position;
  robot1_odom_buff_.odom.pose.pose.orientation = msg->pose.pose.orientation;
  r1_lck.unlock();

  std::unique_lock<std::mutex> r1_lck2(robot1_odom_buff_.mtx);
  nav_msgs::Odometry robot1_curr_odom = robot1_odom_buff_.odom;
  robot1_odom_shifted_pose_ = robot1_curr_odom;
  r1_lck2.unlock();

  if (advance_mode_)
  {
    // Check if R1 is stationary
    double pose_dist_diff_thres = 0.01;
    double stationary_confirmed_time = 10.0;

    if (start_following_)   // If in follow mode
    {
      if (nav_proc_->compute_dist_between_poses(r1_prev_recorded_pose_, robot1_curr_odom) > pose_dist_diff_thres)
      {
        robot0_in_assit_mode_ = false;
        stationary_start_stamp_ = ros::Time::now();
        robot1_is_stationary_ = false;
      }
      else
      {
        if ((ros::Time::now() - stationary_start_stamp_) > ros::Duration(stationary_confirmed_time))
        {
          ROS_INFO("Robot 1 is stationary for more than 10 seconds !");
          robot1_is_stationary_ = true;
        }
      }
    }
    else
    {
      stationary_start_stamp_ = ros::Time::now();
    }

    // Save curr pose for checking if R1 is in stationary later
    r1_prev_recorded_pose_ = robot1_curr_odom;
  }

  geometry_msgs::PoseStamped shift_pose;
  shift_pose.pose.position.x = -0.5;

  geometry_msgs::PoseStamped robot1_shifted_pose = nav_proc_->transform_goal_frame(robot1_curr_odom, shift_pose);   // Obtain the shifted pose that is 0.5m behind R1

  robot1_odom_shifted_pose_.pose.pose.position.x = robot1_shifted_pose.pose.position.x;   // Store it as a R1 position shifted
  robot1_odom_shifted_pose_.pose.pose.position.y = robot1_shifted_pose.pose.position.y;
}


void FollowTrajectory::robot0ScanCB(const sensor_msgs::LaserScan::ConstPtr &msgLsr){  }


// This function uses the service "face_goal" to check if two poses are in sight
bool
FollowTrajectory::goal_in_sight(nav_msgs::Odometry start_pose, nav_msgs::Odometry target_pose)
{
  a4_setup::RequestGoal srv;

  std::vector<geometry_msgs::PoseStamped> start_search_poses, dst_search_poses;
  geometry_msgs::PoseStamped in_pose;
  in_pose.pose = start_pose.pose.pose;
  start_search_poses.push_back(in_pose);  // Include the start pose only

  dst_search_poses = computeSearchPoints(target_pose);  // Obtains left and right shifted poses given the target pose

  for (int i = 0; i < int(start_search_poses.size()); i++)
  {
    for (int j = 0; j < int(dst_search_poses.size()); j++)
    {
      srv.request.pose_start.x = start_search_poses[i].pose.position.x; // Service call start pose
      srv.request.pose_start.y = start_search_poses[i].pose.position.y;

      srv.request.pose.x = dst_search_poses[j].pose.position.x;         // Service call target pose
      srv.request.pose.y = dst_search_poses[j].pose.position.y;

      if ((srv.request.pose_start.x == 0) && (srv.request.pose_start.y == 0) && (srv.request.pose.x == 0) &&
          (srv.request.pose.y == 0))
      {
        return false;
      }

      if (check_goal_client_.call(srv)) // Check if service call is succesfull
      {
        if (srv.response.ack)           // If response is true, then the poses are in sight
        {
          return true;
        }
      }
      else
      {
        ROS_ERROR("Failed to call service");
        return false;
      }
    }
  }
  return false;
}


// This function will generate searching points on eitherside of target with predefined offset
std::vector<geometry_msgs::PoseStamped>
FollowTrajectory::computeSearchPoints(nav_msgs::Odometry input_pose)
{
  std::vector<geometry_msgs::PoseStamped> search_poses;
  std::vector<geometry_msgs::PoseStamped> shift_poses;  // vec to hold shifted poses
  geometry_msgs::PoseStamped shift_pose_temp;           // Temporary shifted pose

  double offset_y_dist = 0.50;

  // Left side shift
  shift_pose_temp.pose.position.x = 0.0;
  shift_pose_temp.pose.position.y = offset_y_dist;
  shift_poses.push_back(shift_pose_temp);

  // Right side shift
  shift_pose_temp.pose.position.x = 0.0;
  shift_pose_temp.pose.position.y = -offset_y_dist;
  shift_poses.push_back(shift_pose_temp);

  geometry_msgs::PoseStamped in_pose;
  in_pose.pose = input_pose.pose.pose;
  search_poses.push_back(in_pose);    // Input pose

  //  Loop to insert shifted poses into search poses vec
  for (int i = 0; i < int(shift_poses.size()); i++)
  {
    geometry_msgs::PoseStamped shifted_pose = nav_proc_->transform_goal_frame(input_pose, shift_poses[i]);
    search_poses.push_back(shifted_pose);
  }
  return search_poses;
}


//  Function to set a given target pose as a navigation target
void
FollowTrajectory::set_pose_as_goal(nav_msgs::Odometry target_pose)
{
  geometry_msgs::PoseStamped new_goal;

  new_goal.header.stamp = ros::Time::now();
  new_goal.header.frame_id = "robot_0/base_link";
  new_goal.pose.position = target_pose.pose.pose.position;
  new_goal.pose.orientation = target_pose.pose.pose.orientation;

  received_goal_.push_back(new_goal);

  ROS_INFO("Sending new pose goal [%f , %f] ", new_goal.pose.position.x, new_goal.pose.position.y);
  ROS_INFO("There are a total of %d target(s) to go.", int(received_goal_.size()));
}


// Function to add tracked R1 poses into goal list
void FollowTrajectory::append_goals_as_marker()
{
  if (received_goal_.size() > 0)
  {
    for (int i = 0; i < int(received_goal_.size()); i++)
    {
      visualization_goal_.push_back(received_goal_[i]);
    }
  }
}


//  Function to publish goal list as markers for visualization
void FollowTrajectory::pub_goals_as_marker()
{
  // publish the pose in goal list as marker
  if (visualization_goal_.size() > 0)
  {
    visualization_msgs::MarkerArray marker_array = produceMarkerArray(visualization_goal_);
    marker_viz_pub_.publish(marker_array);
  }
}


//  Function to publish goal to robot for navigation
void FollowTrajectory::pub_goals_as_path()
{
  // Publish the pose in goal list as path
  geometry_msgs::PoseArray pose_array = producePoseArray(visualization_goal_);
  goal_path_pub_.publish(pose_array);
}


// To check the received_goal_ vector length
int FollowTrajectory::target_goal_length()
{
  return int(received_goal_.size());
}



// To determine whether R0  has reached the first pose in the goal_list "visualization_goal_".
bool FollowTrajectory::is_robot_at_goal()
{
  if (visualization_goal_.size() == 0)  // If there are no more goals remaining, return true to indicate R0 has reached the target
  {
    return true;
  }

  double curr_goal_x = visualization_goal_[0].pose.position.x;
  double curr_goal_y = visualization_goal_[0].pose.position.y;

  std::unique_lock<std::mutex> lck(robot0_odom_buff_.mtx);
  double curr_r0_x = robot0_odom_buff_.odom.pose.pose.position.x;
  double curr_r0_y = robot0_odom_buff_.odom.pose.pose.position.y;
  lck.unlock();
  double robot_goal_dist = nav_proc_->compute_dist(curr_goal_x, curr_goal_y, curr_r0_x, curr_r0_y);    // Compute dist between R0's current position and first pose in goal list

  double reach_goal_thres = 0.2;  //  Threshfold for dist computed

  if (robot_goal_dist < reach_goal_thres) // If R0 is within threshold then R0 has reached the first goal pose, then remove first element from list
  {
    ROS_INFO("Robot reached goal [%f, %f] ", curr_goal_x, curr_goal_y);
    rm_1st_goal_in_list();                                                  // Remove goal reached
    ROS_INFO("Goal length %d ", int(visualization_goal_.size()));

    if (visualization_goal_.size() > 0) // If there are still goals remaining, print their x,y positions
    {
      for (int i = 0; i < int(visualization_goal_.size()); i++)
      {
        ROS_INFO("Goal %d [%f , %f] ", i, visualization_goal_[i].pose.position.x,
                 visualization_goal_[i].pose.position.y);
      }
    }
    return true;
  }
  else
  {
    return false;   // R0 has not reached the target pose
  }
}



// Function to remove first element (oldest goal) from the goal list
void FollowTrajectory::rm_1st_goal_in_list()
{
  if (visualization_goal_.size() == 0)
  {
    ROS_INFO("No goal in list !");
  }
  else if (visualization_goal_.size() == 1)
  {
    visualization_goal_.pop_back();   // Removes last element
  }
  else
  {  // Visualization_goal_ size is greater than 1
    std::reverse(visualization_goal_.begin(), visualization_goal_.end());  // Reverses the vector to utilise pop_back() feature
    visualization_goal_.pop_back();                                        // Remove last element in vector
    std::reverse(visualization_goal_.begin(), visualization_goal_.end());  // Reverses it again, so the elements are in the same order as before
  }
}


// Function to send empty path to stop robot motion
void FollowTrajectory::pub_empty_goal_as_path()
{
  received_goal_.clear();               // Publish empty goal to stop the robot
  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = "robot_0/base_link";
  pose_array.header.stamp = ros::Time::now();
  goal_path_pub_.publish(pose_array);
}


// Function to convert poses to pose array
geometry_msgs::PoseArray FollowTrajectory::producePoseArray(std::vector<geometry_msgs::PoseStamped> pose)
{
  geometry_msgs::PoseArray pose_array;
  int goal_length = pose.size();
  pose_array.header.frame_id = "robot_0/base_link";
  pose_array.header.stamp = ros::Time::now();

  for (int i = 0; i < goal_length; i++)
  {
    geometry_msgs::Pose pose_temp;
    pose_array.poses.push_back(pose[i].pose);
  }
  return pose_array;
}


// Function to convert poses to marker array
visualization_msgs::MarkerArray FollowTrajectory::produceMarkerArray(std::vector<geometry_msgs::PoseStamped> pose)
{
  visualization_msgs::MarkerArray marker_array;
  int goal_length = pose.size();
  int marker_counter = 0;

  for (int i = 0; i < goal_length; i++)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "robot_0/odom";

    marker.header.stamp = ros::Time::now();

    marker.lifetime = ros::Duration(2.0);
    marker.ns = "test";
    marker.id = marker_counter++;

    marker.type = visualization_msgs::Marker::ARROW;

    marker.action = visualization_msgs::Marker::ADD;

    // Set position and orientation
    marker.pose.position.x = pose[i].pose.position.x;
    marker.pose.position.y = pose[i].pose.position.y;
    marker.pose.position.z = 0.0;

    marker.pose.orientation.x = pose[i].pose.orientation.x;
    marker.pose.orientation.y = pose[i].pose.orientation.y;
    marker.pose.orientation.z = pose[i].pose.orientation.z;
    marker.pose.orientation.w = pose[i].pose.orientation.w;

    // Set the scale of the marker
    marker.scale.x = 0.5;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    if (i == (goal_length - 1))
    {
      // The final pose's color is green
      std_msgs::ColorRGBA green_color;
      green_color.a = 0.5;
      green_color.r = 0.0;
      green_color.g = 1.0;
      green_color.b = 0.0;
      marker.color = green_color;
    }
    else
    {
      // The interim pose's color is blue
      std_msgs::ColorRGBA blue_color;
      blue_color.a = 0.5;
      blue_color.r = 0.0;
      blue_color.g = 0.0;
      blue_color.b = 1.0;
      marker.color = blue_color;
    }

    marker_array.markers.push_back(marker);
  }
  return marker_array;
}



// Main loop

void FollowTrajectory::seperateThread()
{
  ros::Rate rate_limiter(1.0);
  while (ros::ok())
  {
    // [P/C] R0 will stay stationary until it first sees R1
    if (r0_wait_for_first_sight_)
    {
      // Fetch R0 pose
      std::unique_lock<std::mutex> lck(robot0_odom_buff_.mtx);
      nav_msgs::Odometry r0_odom = robot0_odom_buff_.odom;
      lck.unlock();

      // Determine whether R1 is in sight of R0
      if (goal_in_sight(r0_odom, robot1_odom_shifted_pose_))
      {
        ROS_INFO("First sight of Robot 1 ...");
        r0_wait_for_first_sight_ = false;             // No longer waiting
        start_following_ = true;                      // Follow mode activated
        prev_communicated_pose_ = robot1_odom_shifted_pose_;
        set_pose_as_goal(robot1_odom_shifted_pose_);  // Set R1 pose as a navigation goal
        append_goals_as_marker();                     // Append received_goal_ to markers array
        pub_goals_as_marker();                        // Publish to markers array for visualization
        pub_goals_as_path();                          // Send received_goal_ to navigate robot
        received_goal_.clear();                       // Clear current received goals
      }
    }
    else
    {
      if (!advance_mode_) //  If not in advance mode
      {
        // [P/C]: If R0 currently in follow mode - will track and save R1 poses into a vector [received_goal_]
        if (start_following_)
        {
          // Threshold for setting new goals
          double tracking_dist = 1.0;  // meters

          // If R1 distance to previously communicated pose is greater than 1.0m
          if (nav_proc_->compute_dist_between_poses(prev_communicated_pose_, robot1_odom_shifted_pose_) > tracking_dist)
          {
            set_pose_as_goal(robot1_odom_shifted_pose_);          // Set R1's current position as a goal
            prev_communicated_pose_ = robot1_odom_shifted_pose_;  // Thereafter this pose becomes the previously communicated pose
          }
          // If R1 is not in sight of last goal && distance is greater than 1.0m; need to command R0 to start navigating
          else if (!goal_in_sight(prev_communicated_pose_, robot1_odom_shifted_pose_) &&
                   nav_proc_->compute_dist_between_poses(prev_communicated_pose_, robot1_odom_shifted_pose_) > 1.0)
          {
            set_pose_as_goal(robot1_odom_shifted_pose_);
            prev_communicated_pose_ = robot1_odom_shifted_pose_;
            send_goal_now = true;
          }

          // R0 to start navigating to the tracked poses
          if (received_goal_.size() > 3 || send_goal_now)
          {
            send_goal_now = false;
            append_goals_as_marker();  // Append received_goal_ to markers array
            pub_goals_as_marker();     // Publish to markers array
            pub_goals_as_path();       // Send received_goal_ to robot
            received_goal_.clear();    // Clear away current received goals
          }
        }
      }
      else
      {
        // [D/HD]: Active follower / Advance mode

        double advance_tracking_dist = 0.2;

        // If R1 is not in sight - request R0 to start navigating
        if ((!goal_in_sight(prev_communicated_pose_, robot1_odom_shifted_pose_) ||
             nav_proc_->compute_dist_between_poses(prev_communicated_pose_, robot1_odom_shifted_pose_) >   // If last pose goal and R1 are not in sight && also not in assist mode
                 advance_tracking_dist) && start_following_ && !robot0_in_assit_mode_)
        {
          pub_empty_goal_as_path();                             // Send empty goal to stop current motion
          prev_communicated_pose_ = robot1_odom_shifted_pose_;  // Resend latest goal to follow
          set_pose_as_goal(prev_communicated_pose_);
          append_goals_as_marker();     // Append received_goal_ to markers array
          pub_goals_as_marker();        // Publish to markers array
          pub_goals_as_path();          // Send received_goal_ to robot
          received_goal_.clear();       // Clear current received goals
          visualization_goal_.clear();
        }
        // If R1 is stationary for > 10 seconds , R0 will go to the side of R1 (assistance mode)
        if (robot1_is_stationary_ && (robot0_in_assit_mode_ == false))
        {
          ROS_INFO("Robot 1 has been stationary for > 10 seconds");
          ROS_INFO("Robot 0 is going into assistance mode (move to the left side (0.5m away) from Robot 1 ");
          robot1_is_stationary_ = false;
          robot0_in_assit_mode_ = true;

          std::unique_lock<std::mutex> r1_lck(robot1_odom_buff_.mtx);
          nav_msgs::Odometry robot1_stationary_odom = robot1_odom_buff_.odom;
          r1_lck.unlock();

          std::vector<geometry_msgs::PoseStamped> computed_assit_mode_poses;      // Container to hold poses taking R0 to assist mode
          computed_assit_mode_poses = nav_proc_->computeAssitModePoses(robot1_stationary_odom); // Pass in target pose and assist mode shifting will be computed and returned

          nav_msgs::Odometry interim_pose1;   // To hold the 2 poses returned leading to R0 being in assist mode
          nav_msgs::Odometry interim_pose2;
          interim_pose1.pose.pose = computed_assit_mode_poses[0].pose;
          interim_pose2.pose.pose = computed_assit_mode_poses[1].pose;

          set_pose_as_goal(interim_pose1);
          set_pose_as_goal(interim_pose2);

          append_goals_as_marker();  // Append received_goal_ to markers array
          pub_goals_as_marker();     // Publish to markers array
          pub_goals_as_path();       // Send received_goal_ to robot
          received_goal_.clear();    // Clear current received goals
          visualization_goal_.clear();
        }
      }
    }

    if (!advance_mode_)
    {
      is_robot_at_goal();     // Check if R0 is near the first pose on the list
      pub_goals_as_marker();  // Keep publishing the latest goal list for visualization
    }
    rate_limiter.sleep();
  }
}
