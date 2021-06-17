/**
 * @file   OdometrySub.cpp
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#include "OdometrySub.h"
#include <string>


int anloro::OdometrySub::_previousId = 0;
int anloro::OdometrySub::_currentId = 0;
anloro::WorldModelInterface anloro::OdometrySub::_interface = anloro::WorldModelInterface();

void anloro::OdometrySub::UpdateId()
{
    _previousId = _currentId;
    _currentId++;
}

void anloro::OdometrySub::ProcessOdom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{

    // std::cout << "Odometry message received!" << std::endl;

    // Update the node ID
    UpdateId();

    float x, y, z, qx, qy, qz, qw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    // Get the pose information 
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.x;
    qx = msg->pose.pose.orientation.x;
    qy = msg->pose.pose.orientation.y;
    qz = msg->pose.pose.orientation.z;
    qw = msg->pose.pose.orientation.w;
    // Get the covariance information [row*6 + row]
    sigmaX = msg->pose.covariance[0];
    sigmaY = msg->pose.covariance[7];
    sigmaZ = msg->pose.covariance[14];
    sigmaRoll = msg->pose.covariance[21];
    sigmaPitch = msg->pose.covariance[28];
    sigmaYaw = msg->pose.covariance[35];

    Transform transform = Transform(x, y, z, qx, qy, qz, qw);

    _interface.AddKeyFrame(_currentId, transform);
    _interface.AddPoseConstraint(_previousId, _currentId, transform, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);

}
