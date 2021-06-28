#include "ros/ros.h"
#include "std_msgs/String.h"
#include "OdometrySub_drifted.h"
#include "LandMarkSub.h"
// #include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
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


  // tf::TransformListener listener;
  // tf::StampedTransform transform;
  // try{
  //   listener.lookupTransform("/camera", "/base_link",  
  //                             ros::Time(0), transform);
  //   float x, y, z, qx, qy, qz, qw;
  //   x = transform.getOrigin().x();
  //   y = transform.getOrigin().y();
  //   z = transform.getOrigin().z();
  //   qx = transform.getRotation().x();
  //   qy = transform.getRotation().y();
  //   qz = transform.getRotation().z();
  //   qw = transform.getRotation().getW();
  //   anloro::LandMarkSub::_CameraToBaseTf = anloro::Transform(x, y, z, qx, qy, qz, qw);

  // }
  // catch (tf::TransformException ex){
  //   ROS_ERROR("%s",ex.what());
  // }

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber subOdom = n.subscribe("odom", 1000, odomSub.ProcessOdom_cb);
  ros::Subscriber subLandMark = n.subscribe("tag_detections", 1000, landMarkSub.ProcessLandMark_cb);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}