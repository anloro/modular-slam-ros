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

        // std::cout << "INFO: Obtained transform from april: " << x << ", " << y << ", " << z << std::endl;
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
        std::cout << "INFO: distance to april tag " << id << ": " << distance << std::endl;
        // std::cout << "INFO: Tag id " << id << " x, y: (" << x << ", " << y << ")" << std::endl;
        // std::cout << "INFO: transform \n" << t.ToMatrix4f() << std::endl;

        // to test april tag
        // -30 degrees
        TestWithAngle(t, -0.5236);
        // -25 degrees
        TestWithAngle(t, -0.4363);
        // -20 degrees
        TestWithAngle(t, -0.3491);
        // -15 degrees
        TestWithAngle(t, -0.2618);
        // -10 degrees
        TestWithAngle(t, -0.1745);
        // -5 degrees
        TestWithAngle(t, -0.0873);
        // 0 degrees
        TestWithAngle(t, 0.0000);
        // 5 degrees
        TestWithAngle(t, 0.0873);
        // 10 degrees
        TestWithAngle(t, 0.1745);
        // 15 degrees
        TestWithAngle(t, 0.2618);
        // 20 degrees
        TestWithAngle(t, 0.3491);
        // 25 degrees
        TestWithAngle(t, 0.4363);
        // 30 degrees
        TestWithAngle(t, 0.5236);

        // if (true)
        // if(abs(y) < 0.5 && distance < 8)
        if (id != 3)
        {
            float fixedCov = 0.1;
            _interface.AddLandMark(id, t, fixedCov/10, fixedCov/10, fixedCov/10, fixedCov, fixedCov, fixedCov);
        }
        else if (distance < 7 && id == 3)
        {
            float fixedCov = 0.1;
            _interface.AddLandMark(id, t, fixedCov/10, fixedCov/10, fixedCov/10, fixedCov, fixedCov, fixedCov);
        }

    }   

}

void anloro::LandMarkSub::TestWithAngle(Transform t, float angle)
{
    Transform rot, tNorm;
    rot = Transform(0, 0, 0, 0, 0, angle); 
    tNorm = rot.inverse()*t;
    float xx, yy, bb;
    tNorm.GetTranslationalAndEulerAngles(xx, yy, bb, bb, bb, bb);
    // std::cout << "INFO: x, y angle " << angle*180/3.141592 << ": (" << xx << ", " << yy << ")" << std::endl;
}

bool anloro::LandMarkSub::SavePosesRaw(modular_slam::SavePosesRaw::Request  &req,
                                       modular_slam::SavePosesRaw::Response &res)
{
    std::string name = req.name; 
    _interface.SavePosesRaw(name);
    res.response = "Raw poses saved to " + name;
    return true;
}