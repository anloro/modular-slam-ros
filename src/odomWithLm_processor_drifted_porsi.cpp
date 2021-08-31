#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OdometrySub_drifted.h"
#include "LandMarkSub.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_processor");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  // Create the interface to the modular slam framework
  anloro::OdometrySub_drifted odomSub;
  anloro::LandMarkSub landMarkSub;

  ros::Subscriber subOdom = n.subscribe("odom", 1000, odomSub.ProcessOdom_cb);
  ros::Subscriber subLandMark = n.subscribe("tag_detections", 1000, landMarkSub.ProcessLandMark_cb);

  ros::ServiceServer service = n.advertiseService("save_poses_raw", landMarkSub.SavePosesRaw);

  ros::spin();

  return 0;
}