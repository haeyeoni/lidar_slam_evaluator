#ifndef PATH_RECORDER_SRC_ALOAM_PATH_RECORDER_H_
#define PATH_RECORDER_SRC_ALOAM_PATH_RECORDER_H_

#include <iostream>
#include <mutex>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <rosbag/bag.h>

#include "utility.h"

class PathRecorder
{
public:
    PathRecorder()
    {
        nh_.getParam("algorithm_name", algorithm_name_);
        algorithm_type_ = path_recorder::GetAlgorithmType(algorithm_name_);
        nh_.getParam("path_topic", path_topic_);
        nh_.getParam("save_to", save_to_);

        path_rcvd_ = boost::make_shared<nav_msgs::Path>();
    }
    ~PathRecorder() = default;
    void Record();

private:
    void AloamHandler(const nav_msgs::Path::ConstPtr &path);
    void LegoloamHandler(const nav_msgs::Odometry::ConstPtr &odom);
    void LiosamHandler(const nav_msgs::Path::ConstPtr &path);
    ros::NodeHandle nh_ = ros::NodeHandle("~");
    nav_msgs::PathPtr path_rcvd_;
    std::string algorithm_name_;
    path_recorder::Algorithm algorithm_type_;
    std::string path_topic_;
    std::string save_to_;

    bool updated_ = false;
    std::mutex path_mtx_; // for further development
};

#endif //PATH_RECORDER_SRC_ALOAM_PATH_RECORDER_H_
