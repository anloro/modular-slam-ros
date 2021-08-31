#include "ros/ros.h"
#include "modular_slam/SavePosesRaw.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "save_poses_raw_client");
  if (argc != 1)
  {
    ROS_INFO("usage: save_poses_raw_client name.txt");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<modular_slam::SavePosesRaw>("save_poses_raw");
  modular_slam::SavePosesRaw srv;
  srv.request.name = argv[1];
  if (client.call(srv))
  {
    // std::string response = ;
    // ROS_INFO(srv.response.response);
  }
  else
  {
    ROS_ERROR("Failed to call service save_poses_raw");
    return 1;
  }

  return 0;
}