#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OdometrySub.h"
#include "LandMarkSub.h"
// #include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_processor");

  ros::NodeHandle n;

// Create the interface to the modular slam framework
    anloro::OdometrySub odomSub;
    anloro::LandMarkSub landMarkSub;

  ros::Subscriber subOdom = n.subscribe("odom", 1000, odomSub.ProcessOdom_cb);
  ros::Subscriber subLandMark = n.subscribe("tag_detections", 1000, landMarkSub.ProcessLandMark_cb);
  
  ros::ServiceServer service = n.advertiseService("save_poses_raw", landMarkSub.SavePosesRaw);


  ros::spin();

  return 0;
}