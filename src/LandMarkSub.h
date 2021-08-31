/**
 * @file   LandMarkSub.h
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#pragma once

#include "apriltag_ros/AprilTagDetectionArray.h"
#include "/usr/local/include/modularslam/WorldModelInterface.h"
#include "modular_slam/SavePosesRaw.h"

namespace anloro{

class LandMarkSub
{
public:
    // Constructor
    LandMarkSub() = default;

    // Member functions
    static void TestWithAngle(Transform t, float angle);
    static void ProcessLandMark_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg);
    static bool SavePosesRaw(modular_slam::SavePosesRaw::Request  &req,
                             modular_slam::SavePosesRaw::Response &res);
    // static anloro::Transform _CameraToBaseTf;

protected:
    static anloro::WorldModelInterface _interface;
};

} // namespace anloro