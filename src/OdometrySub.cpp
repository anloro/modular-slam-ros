/**
 * @file   OdometrySub.cpp
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#include "OdometrySub.h"
#include <string>


int anloro::OdometrySub::_previousId;
int anloro::OdometrySub::_currentId = 0;
float anloro::OdometrySub::_lastKeyFrameTime = 0;
anloro::Transform anloro::OdometrySub::_lastKeyFramePose;
anloro::WorldModelInterface anloro::OdometrySub::_interface = anloro::WorldModelInterface();

void anloro::OdometrySub::UpdateId()
{
    _previousId = _currentId;
    _currentId++;
}

void anloro::OdometrySub::ProcessOdom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{

    float currentTimeStamp = msg->header.stamp.toSec();
    float timeThresh = 1;

    float x, y, z, qx, qy, qz, qw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
    // Get the pose information in global coordinates
    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;
    z = msg->pose.pose.position.z;
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

    // _interface.SetCurrentState(transform);

    if (_currentId == 0) // Just for the inizialization
    {
        _interface.AddKeyFrame(currentTimeStamp, _currentId, transform);
        _lastKeyFramePose = transform.Clone();
        _lastKeyFrameTime = currentTimeStamp;
        UpdateId();

    }else 
    {     
        // Compute the relative transform between odom poses without the drift
        Transform relTransform = Transform(_lastKeyFramePose.inverse().ToMatrix4f() * transform.ToMatrix4f());
        float relRoll, relPitch, relYaw;
        relTransform.GetEulerAngles(relRoll, relPitch, relYaw);
        // std::cout << "INFO: Current relYaw is " << relYaw << " [rad]" << std::endl;

        // Check if the robot moved a certain distance from the previous odom node
        // if (distance > 0.2 && abs(_lastKeyFrameTime - currentTimeStamp) > timeThresh)
        // Check condition of a certain time difference between odom nodes
        if(abs(_lastKeyFrameTime - currentTimeStamp) > timeThresh)
        {
            std::cout << "INFO: Added node with ID " << _currentId << " GT: \n" << transform.ToMatrix4f() << std::endl;

            Transform keyFramePose = _lastKeyFramePose * relTransform;

            float fixedCov = 0.00001;
            _interface.AddPoseConstraint(_previousId, _currentId, relTransform, 
                                            fixedCov/10, fixedCov/10, fixedCov/10, fixedCov, fixedCov, fixedCov);
            _interface.AddKeyFrame(currentTimeStamp, _currentId, keyFramePose);

            _lastKeyFramePose = Transform(x, y, z, qx, qy, qz, qw);
            _lastKeyFrameTime = currentTimeStamp;
            UpdateId();
        }
    }

}

bool anloro::OdometrySub::SavePosesRaw(modular_slam::SavePosesRaw::Request  &req,
                                       modular_slam::SavePosesRaw::Response &res)
{
    std::string name = req.name; 
    _interface.SavePosesRaw(name);
    res.response = "Raw poses saved to " + name;
    return true;
}