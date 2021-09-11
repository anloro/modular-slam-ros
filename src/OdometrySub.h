/**
 * @file   OdometrySub.h
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#pragma once

#include "nav_msgs/Odometry.h"
#include "/usr/local/include/modularslam/WorldModelInterface.h"
#include "modular_slam/SavePosesRaw.h"

namespace anloro{

class OdometrySub
{
public:
    // Constructor
    OdometrySub() = default;

    // Member functions
    static void UpdateId();
    static void ProcessOdom_cb(const nav_msgs::Odometry::ConstPtr& msg);
    static bool SavePosesRaw(modular_slam::SavePosesRaw::Request  &req,
                             modular_slam::SavePosesRaw::Response &res);

protected:
    static WorldModelInterface _interface;
    static int _currentId;
    static int _previousId;
    static Transform _lastKeyFramePose;
    static float _lastKeyFrameTime;
};

} // namespace anloro