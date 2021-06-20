/**
 * @file   LandMarkSub.h
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#pragma once

#include "apriltag_ros/AprilTagDetectionArray.h"
#include "/usr/local/include/modularslam/WorldModelInterface.h"

namespace anloro{

class LandMarkSub
{
public:
    // Constructor
    LandMarkSub() = default;

    // Member functions
    static void ProcessLandMark_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    // static anloro::Transform _CameraToBaseTf;

protected:
    static anloro::WorldModelInterface _interface;
};

} // namespace anloro