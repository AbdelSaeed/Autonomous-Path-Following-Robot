/**
 * @file follow_traj_main.h
 * @author Abdelghani Saeed (12747150@uts.student.uts.edu.au)
 * @brief Project main point of entry
 * @version 1.0
 * @date 07-11-2020
 *
 */


#include "follow_traj.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "follow_traj_main_node");

  ros::NodeHandle nh;

  std::shared_ptr<FollowTrajectory> gc(new FollowTrajectory(nh));
  std::thread t(&FollowTrajectory::seperateThread, gc);

  ros::spin();

  ros::shutdown();
  t.join();

  return 0;
}
