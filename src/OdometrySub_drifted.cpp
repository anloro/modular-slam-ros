/**
 * @file   OdometrySub_drifted.cpp
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#include "OdometrySub_drifted.h"
#include <string>


// int anloro::OdometrySub_drifted::_previousId;
// int anloro::OdometrySub_drifted::_currentId = 0;
// float anloro::OdometrySub_drifted::_lastKeyFrameTime = 0;
// float anloro::OdometrySub_drifted::_xBias = 0;
// float anloro::OdometrySub_drifted::_yawBias = 0;
// anloro::Transform anloro::OdometrySub_drifted::_lastKeyFramePose;
// anloro::Transform anloro::OdometrySub_drifted::_lastKeyFramePoseDrifted;
// anloro::Transform anloro::OdometrySub_drifted::_lastOdomPose;
// anloro::Transform anloro::OdometrySub_drifted::_lastOdomPoseDrifted;
// anloro::WorldModelInterface anloro::OdometrySub_drifted::_interface = anloro::WorldModelInterface();
// ros::Publisher anloro::OdometrySub_drifted::_pub;
// ros::NodeHandle anloro::OdometrySub_drifted::_n;

anloro::OdometrySub_drifted::OdometrySub_drifted(){
    _currentId = 0;
    _lastKeyFrameTime = 0;
    _xBias = 0;
    _yawBias = 0;
    // to publish the drifted odometry
    pub_ = n_.advertise<nav_msgs::Odometry>("/odom_drifted", 1000);

    sub_ = n_.subscribe("/odom", 1000, &anloro::OdometrySub_drifted::ProcessOdom_cb, this);
    serv_ = n_.advertiseService("save_poses_raw", &anloro::OdometrySub_drifted::SavePosesRaw, this);
}

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

    // _interface.SetCurrentState(transform);

    if (_currentId == 0) // Just for the inizialization
    {
        _interface.AddKeyFrame(currentTimeStamp, _currentId, transform);
        _lastKeyFramePose = transform.Clone();
        _lastKeyFramePoseDrifted = transform.Clone();
        _lastOdomPose = transform.Clone();
        _lastOdomPoseDrifted = transform.Clone();
        _lastKeyFrameTime = currentTimeStamp;
        UpdateId();

    }else 
    {     
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
            // yawRelBias = relYaw * 0.4;
            yawRelBias = 0.01;
            // Apply the drift to the relative transform
            relTransform.SetEulerAngles(relRoll, relPitch, relYaw + yawRelBias);
        }

        float xRel, yRel, zRel;
        relTransform.GetTranslationalVector(xRel, yRel ,zRel);
        float distance = std::sqrt(xRel*xRel + yRel*yRel);

        // Check if the robot moved a certain distance from the previous odom node
        // if (distance > 0.2 && abs(_lastKeyFrameTime - currentTimeStamp) > timeThresh)
        // Check condition of a certain time difference between odom nodes
        if(abs(_lastKeyFrameTime - currentTimeStamp) > timeThresh)
        {
            std::cout << "INFO: Added node with ID " << _currentId << " GT: \n" << transform.ToMatrix4f() << std::endl;

            // Apply the relative transform to the last modified transform
            // to obtain the new drifted odom pose
            Transform keyFramePoseDrifted = _lastKeyFramePoseDrifted * relTransform;

            float fixedCov = 0.01;
            _interface.AddPoseConstraint(_previousId, _currentId, relTransform, 
                                            fixedCov/10, fixedCov/10, fixedCov/10, fixedCov, fixedCov, fixedCov);
            _interface.AddKeyFrame(currentTimeStamp, _currentId, keyFramePoseDrifted);

            nav_msgs::Odometry odom_output;
            odom_output.header.stamp = msg->header.stamp;
            odom_output.header.frame_id = "odom_drifted";
            // odom_output.header.frame_id = msg->header.frame_id;
            odom_output.pose.pose.position.x = keyFramePoseDrifted.X();
            odom_output.pose.pose.position.y = keyFramePoseDrifted.Y();
            odom_output.pose.pose.position.z = keyFramePoseDrifted.Z();
            geometry_msgs::Quaternion quaternion;
            Eigen::Quaternionf q_out = Eigen::Quaternionf(keyFramePoseDrifted.GetAffineTransform().linear()).normalized();
            quaternion.x = q_out.x();
            quaternion.y = q_out.y();
            quaternion.z = q_out.z();
            quaternion.w = q_out.w();
            odom_output.pose.pose.orientation = quaternion;
            odom_output.pose.covariance[0] = fixedCov/10;
            odom_output.pose.covariance[7] = fixedCov/10;
            odom_output.pose.covariance[14] = fixedCov/10;
            odom_output.pose.covariance[21] = fixedCov;
            odom_output.pose.covariance[28] = fixedCov;
            odom_output.pose.covariance[35] = fixedCov;

            odom_output.child_frame_id = msg->child_frame_id;
            odom_output.twist.twist.linear.x = 0;
            odom_output.twist.twist.linear.y = 0;
            odom_output.twist.twist.angular.z = 0;
            odom_output.twist.covariance = msg->twist.covariance;

            pub_.publish(odom_output);

            _lastKeyFramePose = Transform(x, y, z, qx, qy, qz, qw);
            _lastKeyFramePoseDrifted = keyFramePoseDrifted.Clone(); 
            _lastKeyFrameTime = currentTimeStamp;
            UpdateId();
        }
    }
}

bool anloro::OdometrySub_drifted::SavePosesRaw(modular_slam::SavePosesRaw::Request  &req,
                                       modular_slam::SavePosesRaw::Response &res)
{
    std::string name = req.name; 
    _interface.SavePosesRaw(name);
    res.response = "Raw poses saved to " + name;
    return true;
}