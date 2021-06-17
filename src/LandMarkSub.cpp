/**
 * @file   LandMarkSub.cpp
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#include "LandMarkSub.h"
#include <string>


anloro::WorldModelInterface anloro::LandMarkSub::_interface = anloro::WorldModelInterface();

void anloro::LandMarkSub::ProcessLandMark_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{

    // std::cout << "Odometry message received!" << std::endl;


    int length = msg->detections.size(); 
    for (int i = 0; i < length; i++) 
    {    
        int id = msg->detections[i].id[0];

        float x, y, z, qx, qy, qz, qw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
        // Get the pose information 
        x = msg->detections[i].pose.pose.pose.position.x;
        y = msg->detections[i].pose.pose.pose.position.y;
        z = msg->detections[i].pose.pose.pose.position.x;
        qx = msg->detections[i].pose.pose.pose.orientation.x;
        qy = msg->detections[i].pose.pose.pose.orientation.y;
        qz = msg->detections[i].pose.pose.pose.orientation.z;
        qw = msg->detections[i].pose.pose.pose.orientation.w;
        // Get the covariance information [row*6 + row]
        sigmaX = msg->detections[i].pose.pose.covariance[0];
        sigmaY = msg->detections[i].pose.pose.covariance[7];
        sigmaZ = msg->detections[i].pose.pose.covariance[14];
        sigmaRoll = msg->detections[i].pose.pose.covariance[21];
        sigmaPitch = msg->detections[i].pose.pose.covariance[28];
        sigmaYaw = msg->detections[i].pose.pose.covariance[35];

        Transform transform = Transform(x, y, z, qx, qy, qz, qw);

        _interface.AddLandMark(id, transform, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);

    }   

}
