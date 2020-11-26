/**
 * @file sample.cpp
 * @author Abdelghani Saeed (12747150@uts.student.uts.edu.au)
 * @brief Sample implementation file
 * @version 1.0
 * @date 07-11-2020
 *
 */

#include "sample.h"

#define USE_GIVEN_START_POSE
#define UPDT_CHECK_CONNECTIVITY_INPUT

/**
 * This sample code is provided to illustrate
 * - Subscribing to standard topics (Odometry, Laser and Grid)
 * - Respond to an incoming service call
 *
 */

PfmsSample::PfmsSample(ros::NodeHandle nh) : nh_(nh)
{
  // Subscribing to odometry
  sub1_ = nh_.subscribe("robot_0/odom", 1000, &PfmsSample::odomCallback, this);
  // Subscribing to laser
  sub2_ = nh_.subscribe("robot_0/base_scan", 10, &PfmsSample::laserCallback, this);
  // Subscribing to occupnacy grid
  sub3_ = nh_.subscribe("local_map/local_map", 1, &PfmsSample::occupancyGridCallback, this);

  // Publishing markers
  viz_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("visualization_marker", 3, false);

  // Below is how to get parameters from command line, on command line they need to be _param:=value
  // For example _example:=0.1
  // ROS will obtain the configuration from command line, or assign a default value 0.1
  ros::NodeHandle pn("~");
  double example;
  pn.param<double>("example", example, 0.1);
  ROS_INFO_STREAM("example:" << example);

  goalReceived_ = false;  // We will use this atomic bool to let us know when we have new data

  // Allowing an incoming service on /face_goal
  service_ = nh_.advertiseService("face_goal", &PfmsSample::faceGoal, this);
}

PfmsSample::~PfmsSample()
{
}

bool PfmsSample::faceGoal(a4_setup::RequestGoal::Request &req, a4_setup::RequestGoal::Response &res)
{
#ifdef USE_GIVEN_START_POSE
  {
    ROS_INFO_STREAM("start at: [x,y]=[" << req.pose_start.x << "," << req.pose_start.y << "]");

    std::unique_lock<std::mutex> lck_3(poseDataBuffer_.mtx);
    poseDataBuffer_.pose.position.x = req.pose_start.x;
    poseDataBuffer_.pose.position.y = req.pose_start.y;
  }
#endif
  // When an incoming call arrives, we can respond to it here
  ROS_INFO_STREAM("request to: [x,y]=[" << req.pose.x << "," << req.pose.y << "]");

  // We make a copy of the pose request to share it in other threads of processing (Ex5 for instance)
  {
    std::unique_lock<std::mutex> lck(goalPoseBuffer_.mtx);
    goalPoseBuffer_.pose.position.x = req.pose.x;
    goalPoseBuffer_.pose.position.y = req.pose.y;
  }

  // Let's get the Grid
  std::unique_lock<std::mutex> lck(ogMapBuffer_.mtx);
  nav_msgs::OccupancyGrid grid = ogMapBuffer_.grid;

  // We now construct gridPorcessing, which takes a grid and internally makes a copy of the grid
  // Which is why we unlock here immediately therafter
  GridProcessing gridProcessing(grid);
  lck.unlock();

  //! @todo Ex04 : Adjust the code so it returns a `true` in the acknowledgment of the service call if the point
  //! can be reached in a straight line from current robot pose, only traversing free space.
  //! Check the service message via "rossrv info a4_setup/RequestGoal"
  // We can transform either way (global to local) or (local to global)
  // Here we go global to local
  geometry_msgs::Point local;

  std::unique_lock<std::mutex> lck2(poseDataBuffer_.mtx);
  geometry_msgs::Pose pose = poseDataBuffer_.pose;
  lck2.unlock();

  local.x = (req.pose.x - pose.position.x);
  local.y = (req.pose.y - pose.position.y);

  geometry_msgs::Point zero;
  zero.x = 0;
  zero.y = 0;
// We store the result of checking connectivity
// in ack field of the responce (if you look at the service message, the ack is just a boolean
// res.ack = gridProcessing.checkConnectivity(zero, local);

#ifdef UPDT_CHECK_CONNECTIVITY_INPUT
  zero.x = req.pose_start.x;
  zero.y = req.pose_start.y;
  local.x = req.pose.x;
  local.y = req.pose.y;
#endif
  bool result = gridProcessing.checkConnectivity(zero, local);
  res.ack = result;
  if (result)
  {
    ROS_INFO_STREAM("Goal can be reached");
  }
  goalReceived_ = true;

  return true;  // We retrun true to indicate the service call sucseeded (your responce should indicate a value)
}

// A callback for odometry
void PfmsSample::odomCallback(const nav_msgs::OdometryConstPtr &msg)
{
// We store a copy of the pose and lock a mutex when updating
#ifndef USE_GIVEN_START_POSE
  {
    std::unique_lock<std::mutex> lck(poseDataBuffer_.mtx);
    poseDataBuffer_.pose = msg->pose.pose;
  }
#endif
}

void PfmsSample::occupancyGridCallback(const nav_msgs::OccupancyGridPtr &msg)
{
  std::unique_lock<std::mutex> lck(ogMapBuffer_.mtx);
  ogMapBuffer_.grid = *msg;
}

void PfmsSample::laserCallback(const sensor_msgs::LaserScanConstPtr &msg)
{
}

void PfmsSample::seperateThread()
{
  /**
    * The below loop runs until ros is shutdown
    */

  //! What rate shoudl we run this at?
  ros::Rate rate_limiter(1.0);
  while (ros::ok())
  {
    //! @todo Ex05 : Check wether the requested goal can be reached every 5 seconds.
    //!
    //! To do this check, what information do we need?
    //!
    if (goalReceived_)
    {
      // Let's check which way we should face
      std::unique_lock<std::mutex> lck(poseDataBuffer_.mtx);
      geometry_msgs::Pose robotPose = poseDataBuffer_.pose;
      lck.unlock();

      // Below is how to get yaw ... if your still wondering
      // double robotYaw = tf::getYaw(robotPose.orientation);

      std::unique_lock<std::mutex> lck2(goalPoseBuffer_.mtx);
      geometry_msgs::Pose goalPose = goalPoseBuffer_.pose;
      lck2.unlock();

#ifdef UPDT_CHECK_CONNECTIVITY_INPUT
      geometry_msgs::Point local;
      local.x = goalPose.position.x;
      local.y = goalPose.position.y;

      geometry_msgs::Point zero;
      zero.x = robotPose.position.x;
      zero.y = robotPose.position.y;
#else
      geometry_msgs::Point local;
      local.x = goalPose.position.x - robotPose.position.x;
      // local.y = goalPose.position.x - robotPose.position.x;

      local.y = goalPose.position.y - robotPose.position.y;

      geometry_msgs::Point zero;
      zero.x = 0;
      zero.y = 0;
#endif
      // Let's get the Grid
      std::unique_lock<std::mutex> lck3(ogMapBuffer_.mtx);
      nav_msgs::OccupancyGrid grid = ogMapBuffer_.grid;
      GridProcessing gridProcessing(grid);
      lck3.unlock();

      // Why would a goal reachability change?
      // It could have been in uknown space first and then free OR
      // It could have been behind a wall and your robot now has line of sight to it!
      bool reachable = gridProcessing.checkConnectivity(zero, local);
      // ROS_INFO("local point %f %f", local.x, local.y);

      // Let's send a marker with color (green for reachable, red for now)
      std_msgs::ColorRGBA color;
      color.a = 0.5;  // a is alpha - transparency 0.5 is 50%;
      // the colors r,g,b are floats 0 - 1.

      if (reachable)
      {
        // ROS_INFO_STREAM("Goal can be reached");
        color.r = 0;
        color.g = 1.0;
        color.b = 0;
      }
      else
      {
        // ROS_INFO_STREAM("Goal CAN NOT be reached");
        color.r = 1.0;
        color.g = 0;
        color.b = 0;
      }

      // Let's also publish the marker here
      visualization_msgs::MarkerArray marker_array = produceMarkerArray(goalPose, color);
      viz_pub_.publish(marker_array);
    }

    rate_limiter.sleep();
  }
}

visualization_msgs::MarkerArray PfmsSample::produceMarkerArray(geometry_msgs::Pose pose, std_msgs::ColorRGBA color)
{
  visualization_msgs::MarkerArray marker_array;

  int marker_counter = 0;
  visualization_msgs::Marker marker;

  // We need to set the frame
  // Set the frame ID and time stamp.
  // marker.header.frame_id = "/world"; // this is wrong reference frame
  marker.header.frame_id = "/local_map/local_map";  // fixed
  // single_marker_person.header.stamp = ros::Time();
  marker.header.stamp = ros::Time::now();

  // We set lifetime (it will dissapear in this many seconds)
  marker.lifetime = ros::Duration(2.0);
  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "test";
  marker.id = marker_counter++;

  // The marker type, we use a cylinder in this example
  marker.type = visualization_msgs::Marker::CYLINDER;

  // Set the marker action.  Options are ADD and DELETE (we ADD it to the screen)
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pose.position.x;
  marker.pose.position.y = pose.position.y;
  marker.pose.position.z = 0.0;

  // Orientation, we are not going to orientate it
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 0.5x0.5x0.5 here means 0.5m side
  marker.scale.x = 0.5;
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Alpha is stransparency (50% transparent)
  marker.color = color;

  // We push the marker back on our array of markers
  marker_array.markers.push_back(marker);

  return marker_array;
}
