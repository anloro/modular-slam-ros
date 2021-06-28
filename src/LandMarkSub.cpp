/**
 * @file   LandMarkSub.cpp
 * @brief  Defines the Odometry ros interface with the modular slam framework.
 * @author Ángel Lorente Rogel
 * @date   15/06/2021
 */

#include "LandMarkSub.h"
#include <string>


anloro::WorldModelInterface anloro::LandMarkSub::_interface = anloro::WorldModelInterface();
// anloro::Transform anloro::LandMarkSub::_CameraToBaseTf;

void anloro::LandMarkSub::ProcessLandMark_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr& msg)
{

    int length = msg->detections.size(); 
    for (int i = 0; i < length; i++) 
    {    
        int id = msg->detections[i].id[0];

        float x, y, z, qx, qy, qz, qw, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw;
        // Get the pose information 
        x = msg->detections[i].pose.pose.pose.position.x;
        y = msg->detections[i].pose.pose.pose.position.y;
        z = msg->detections[i].pose.pose.pose.position.z;
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

        // std::cout << "Obtained transform from april: " << x << " " << y << " " << z << std::endl;
        // std::cout << "Relative transform: " << std::endl;
        Transform CameraToBaseTf = Transform(-0.047, 0.107, -0.069, 0.500, -0.500, 0.500, 0.500);
        // CameraToBaseTf.GetTranslationalAndEulerAngles(x, y, z, qx, qy, qz);
        // std::cout << "In quaternions: " << x << " " << y << " " << z << " " << qx << " " << qy << " " << qz << std::endl;
        // std::cout << "//////////////////////////////////////////////" << std::endl;


        Transform t = Transform(CameraToBaseTf.inverse() * transform);
        // std::cout << "AprilTag relative pose base frame custom: \n" << t.ToMatrix4f() << std::endl;

        // std::cout << "---------------------------------------------" << std::endl;

        t.GetTranslationalAndEulerAngles(x, y, z, qx, qy, qz);
        float distance = std::sqrt(x*x + y*y);
        // std::cout << "INFO: distance to april tag " << distance << std::endl;
        // std::cout << "INFO: transform \n" << t.ToMatrix4f() << std::endl;

        // if(abs(y) < 0.5)
        // if (true)
        if (distance < 10)
        {
            _interface.AddLandMark(id, t, sigmaX, sigmaY, sigmaZ, sigmaRoll, sigmaPitch, sigmaYaw);
        }

    }   

}
