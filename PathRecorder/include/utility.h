#ifndef PATHRECORDER_INCLUDE_UTILITY_H_
#define PATHRECORDER_INCLUDE_UTILITY_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

namespace path_recorder
{
    enum class Algorithm
    {
        A_LOAM,
        LeGO_LOAM,
        LIO_SAM
    };

    inline Algorithm GetAlgorithmType(std::string algorithm_name)
    {
        if (algorithm_name == "aloam")
            return Algorithm::A_LOAM;
        else if (algorithm_name == "lego_loam")
            return Algorithm::LeGO_LOAM;
        else if (algorithm_name == "lio_sam")
            return Algorithm::LIO_SAM;
        else
            std::cerr << "Unsupported algorithm type." << std::endl;
    }
}

namespace ros_conversion
{
    inline geometry_msgs::PoseStamped Odom2PoseStamped(const nav_msgs::Odometry &odom)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = odom.header;
        pose_stamped.pose.position = odom.pose.pose.position;
        return pose_stamped;
    }
}

#endif //PATHRECORDER_INCLUDE_UTILITY_H_
