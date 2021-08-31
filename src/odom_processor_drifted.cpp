#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OdometrySub_driftedv2.h"
// #include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_processor");

  ros::NodeHandle n;

// Create the interface to the modular slam framework
  anloro::OdometrySub_driftedv2 odomSub;

  ros::spin();

  return 0;
}