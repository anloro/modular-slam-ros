/**
 * @file   OdometrySub.h
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#pragma once

#include "nav_msgs/Odometry.h"
#include "/usr/local/include/modularslam/WorldModelInterface.h"

namespace anloro{

class OdometrySub
{
public:
    // Constructor
    OdometrySub() = default;

    // Member functions
    static void UpdateId();
    static void ProcessOdom_cb(const nav_msgs::Odometry::ConstPtr& msg);

protected:
    static anloro::WorldModelInterface _interface;
    static int _currentId;
    static int _previousId;
};

} // namespace anloro