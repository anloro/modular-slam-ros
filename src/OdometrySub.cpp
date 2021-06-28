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
float anloro::OdometrySub::_lastOdomTime = 0;
anloro::Transform anloro::OdometrySub::_lastOdomPose;
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
    // Get the pose information 
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

    _interface.SetCurrentState(transform);
    // Check condition of a certain time difference between odom nodes
    if(abs(_lastOdomTime - currentTimeStamp) > timeThresh)
    {
        
        // std::cout << "Current time stamp: " << currentTimeStamp << std::endl;
        // std::cout << "Last time stamp: " << _lastOdomTime << std::endl;

        // float x, y, z, qx, qy, qz, qw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
        // // Get the pose information 
        // x = msg->pose.pose.position.x;
        // y = msg->pose.pose.position.y;
        // z = msg->pose.pose.position.z;
        // qx = msg->pose.pose.orientation.x;
        // qy = msg->pose.pose.orientation.y;
        // qz = msg->pose.pose.orientation.z;
        // qw = msg->pose.pose.orientation.w;
        // // Get the covariance information [row*6 + row]
        // sigmaX = msg->pose.covariance[0];
        // sigmaY = msg->pose.covariance[7];
        // sigmaZ = msg->pose.covariance[14];
        // sigmaRoll = msg->pose.covariance[21];
        // sigmaPitch = msg->pose.covariance[28];
        // sigmaYaw = msg->pose.covariance[35];

        // Transform transform = Transform(x, y, z, qx, qy, qz, qw);

        // std::cout << "Previous id: " << _previousId << std::endl;
        // std::cout << "Current id: " << _currentId << std::endl;

        if (_currentId == 0) // Just for the inizialization
        {
            _interface.AddKeyFrame(_currentId, transform);
            _lastOdomPose = transform;
            _lastOdomTime = currentTimeStamp;
            UpdateId();

        }else 
        {
            // Transform relTransform = Transform(_lastOdomPose.inverse().ToMatrix4f() * transform.ToMatrix4f());
            Transform relTransform = Transform(transform.ToMatrix4f() * _lastOdomPose.inverse().ToMatrix4f());

            float x, y, dummyData;
            relTransform.GetTranslationalAndEulerAngles(x, y ,dummyData, dummyData, dummyData, dummyData);
            float distance = std::sqrt(x*x + y*y);

            // Check if the robot moved a certain distance from the previous odom node
            if (distance > 0.2)
            {
                // std::cout << "Previous Transform: \n" << _lastOdomPose.ToMatrix4f() << std::endl;
                // std::cout << "Transform: \n" << transform.ToMatrix4f() << std::endl;

                // std::cout << "Relative transform: \n" << std::endl;
                // relTransform = Transform(transform.ToMatrix4f() * _lastOdomPose.inverse().ToMatrix4f());
                // std::cout << "transform * lastOdom.inv(): \n" << relTransform.ToMatrix4f() << std::endl;
                // std::cout << " " << std::endl;

                _interface.AddPoseConstraint(_previousId, _currentId, relTransform, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
                // _interface.AddPoseConstraint(_previousId, _currentId, transform, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
                _interface.AddKeyFrame(_currentId, transform);
                _lastOdomPose = transform;
                _lastOdomTime = currentTimeStamp;
                UpdateId();
            }
        }
    }

}
