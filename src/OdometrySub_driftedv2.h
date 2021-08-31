/**
 * @file   OdometrySub_driftedv2.h
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#pragma once

#include "nav_msgs/Odometry.h"
#include "/usr/local/include/modularslam/WorldModelInterface.h"
#include "modular_slam/SavePosesRaw.h"
#include <ros/ros.h>

namespace anloro{

class OdometrySub_driftedv2
{
public:
    // Constructor
    OdometrySub_driftedv2();

    // Member functions
    void UpdateId();
    void ProcessOdom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    bool SavePosesRaw(modular_slam::SavePosesRaw::Request  &req,
                             modular_slam::SavePosesRaw::Response &res);

protected:
    WorldModelInterface _interface;
    int _currentId;
    int _previousId;
    Transform _lastKeyFramePose, _lastKeyFramePoseDrifted, _lastOdomPose, _lastOdomPoseDrifted;
    float _lastKeyFrameTime;
    float _xBias, _yawBias;

private:
    ros::Publisher pub_;
    ros::NodeHandle n_; 
    ros::Subscriber sub_; 
    ros::ServiceServer serv_;

};

} // namespace anloro