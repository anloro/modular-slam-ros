/**
 * @file   OdometrySub_drifted.cpp
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#include "OdometrySub_drifted.h"
#include <string>


int anloro::OdometrySub_drifted::_previousId;
int anloro::OdometrySub_drifted::_currentId = 0;
float anloro::OdometrySub_drifted::_lastKeyFrameTime = 0;
float anloro::OdometrySub_drifted::_xBias = 0;
float anloro::OdometrySub_drifted::_yawBias = 0;
anloro::Transform anloro::OdometrySub_drifted::_lastKeyFramePose;
anloro::Transform anloro::OdometrySub_drifted::_lastKeyFramePoseDrifted;
anloro::Transform anloro::OdometrySub_drifted::_lastOdomPose;
anloro::Transform anloro::OdometrySub_drifted::_lastOdomPoseDrifted;
anloro::WorldModelInterface anloro::OdometrySub_drifted::_interface = anloro::WorldModelInterface();

void anloro::OdometrySub_drifted::UpdateId()
{
    _previousId = _currentId;
    _currentId++;
}

void anloro::OdometrySub_drifted::ProcessOdom_cb(const nav_msgs::Odometry::ConstPtr& msg)
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

    _interface.SetCurrentState(transform);

    if (_currentId == 0) // Just for the inizialization
    {
        _interface.AddKeyFrame(_currentId, transform);
        _lastKeyFramePose = transform.Clone();
        _lastKeyFramePoseDrifted = transform.Clone();
        _lastOdomPose = transform.Clone();
        _lastOdomPoseDrifted = transform.Clone();
        _lastKeyFrameTime = currentTimeStamp;
        UpdateId();

    }else 
    {
        // Now compute the movement vector in absolute coordinates 
        // (not very useful)
        // float dx, dy, xLast, yLast, zLast;
        // _lastKeyFramePose.GetTranslationalVector(xLast, yLast, zLast);
        // dx = x - xLast;
        // dy = y - yLast;
     
        Transform dtransform = Transform(_lastOdomPose.inverse().ToMatrix4f() * transform.ToMatrix4f());

        float droll, dpitch, dyaw;
        dtransform.GetEulerAngles(droll, dpitch, dyaw);
        // std::cout << "INFO: Current dyaw is " << dyaw << " [rad]" << std::endl;

        // Compute the drift proportional to the differential displacement
        float xBias = 0;
        float yawBias = 0;
        Transform odomPoseDrifted;
        if (dyaw > 0)
        {
            yawBias = dyaw * 0.01;
            // _yawBias = _yawBias + yawBias;
            // Apply the drift to the relative transform
            dtransform.SetEulerAngles(droll, dpitch, dyaw + yawBias);
        }

        // Apply the relative transform to the last modified transform
        // to obtain the new drifted odom pose
        odomPoseDrifted = _lastOdomPoseDrifted * dtransform;
        _lastOdomPoseDrifted = odomPoseDrifted.Clone();
        _lastOdomPose = transform.Clone();

        // -----------------------------------------
        // Do the same but with keyframes
        // Compute the relative transform between odom poses without the drift
        Transform relTransform = Transform(_lastKeyFramePose.inverse().ToMatrix4f() * transform.ToMatrix4f());

        float relRoll, relPitch, relYaw;
        relTransform.GetEulerAngles(relRoll, relPitch, relYaw);
        // std::cout << "INFO: Current relYaw is " << relYaw << " [rad]" << std::endl;

        // Compute the drift proportional to the differential displacement
        float yawRelBias = 0;
        Transform keyFramePoseDrifted;
        if (relYaw > 0)
        {
            yawRelBias = relYaw * 0.01;
            // Apply the drift to the relative transform
            relTransform.SetEulerAngles(relRoll, relPitch, relYaw + yawRelBias);

            // float xRel, yRel, zRel;
            // relTransform.GetTranslationalVector(xRel, yRel ,zRel);
            // relTransform.SetTranslationalVector(xRel -0.01, yRel -0.035 ,zRel);
        }

        float xRel, yRel, zRel;
        relTransform.GetTranslationalVector(xRel, yRel ,zRel);
        float distance = std::sqrt(xRel*xRel + yRel*yRel);

        // // Get the relative transform to the prevoius KeyFrame
        // Transform relTransform = Transform(_lastKeyFramePose.inverse().ToMatrix4f() * transform.ToMatrix4f());
        // // Transform relTransform = Transform(transform.ToMatrix4f() * _lastKeyFramePose.inverse().ToMatrix4f());
        // float xRel, yRel, zRel;
        // relTransform.GetTranslationalVector(xRel, yRel ,zRel);
        // float distance = std::sqrt(xRel*xRel + yRel*yRel);

        // // Add drift to the absolute pose
        // _xBias = _xBias + xBias;
        // transform.SetTranslationalVector(x + _xBias, y, z);
        // relTransform.SetTranslationalVector(xRel + xBias, yRel ,zRel);

        // // std::cout << "INFO: Current bias is " << _xBias << " [m]" << std::endl;
        // std::cout << "INFO: Current relative transform \n " << relTransform.ToMatrix4f() << std::endl;

        // Check if the robot moved a certain distance from the previous odom node
        // Check condition of a certain time difference between odom nodes
        if (distance > 0.2 && abs(_lastKeyFrameTime - currentTimeStamp) > timeThresh)
        {
            // std::cout << "Previous Transform: \n" << _lastKeyFramePose.ToMatrix4f() << std::endl;
            // std::cout << "Transform: \n" << transform.ToMatrix4f() << std::endl;

            // std::cout << "Relative transform: \n" << std::endl;
            // relTransform = Transform(transform.ToMatrix4f() * _lastKeyFramePose.inverse().ToMatrix4f());
            // std::cout << "transform * lastOdom.inv(): \n" << relTransform.ToMatrix4f() << std::endl;
            // std::cout << " " << std::endl;

            // Apply the relative transform to the last modified transform
            // to obtain the new drifted odom pose
            Transform keyFramePoseDrifted = _lastKeyFramePoseDrifted * relTransform;

            // if(abs(relYaw + yawRelBias) > 0.1)
            // {
            //     _interface.AddPoseConstraint(_previousId, _currentId, relTransform, 1, 1, 1, 1, 1, 1);
            //     std::cout << "INFO: Variances set to " << 1 << std::endl;
            // }
            // else{
            //     _interface.AddPoseConstraint(_previousId, _currentId, relTransform, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001);
            //     // _interface.AddPoseConstraint(_previousId, _currentId, relTransform, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
            //     std::cout << "INFO: Variances set to DEFAULT" << std::endl;
            // }

            _interface.AddPoseConstraint(_previousId, _currentId, relTransform, 0.0001, 0.0001, 0.0001, 0.001, 0.001, 0.001);

            // std::cout << "INFO: yaw odom relative rotation " << relYaw + yawRelBias << std::endl;
            // std::cout << "Variances: " << sigmaX << ", " << sigmaY << ", " << sigmaZ << ", " << sigmaRoll << ", " << sigmaPitch << ", " << sigmaYaw + yawRelBias*4 << std::endl;
            // _interface.AddPoseConstraint(_previousId, _currentId, transform, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
            _interface.AddKeyFrame(_currentId, keyFramePoseDrifted);
            _lastKeyFramePose = Transform(x, y, z, qx, qy, qz, qw);
            _lastKeyFramePoseDrifted = keyFramePoseDrifted.Clone(); 
            _lastKeyFrameTime = currentTimeStamp;
            UpdateId();
        }
    }
}
