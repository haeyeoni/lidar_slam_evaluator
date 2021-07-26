#include "path_recorder.h"

void PathRecorder::Record()
{
    boost::filesystem::create_directory(save_to_);

    rosbag::Bag bag;
    ros::Subscriber sub_path;
    if (algorithm_type_ == path_recorder::Algorithm::A_LOAM)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::AloamHandler, this,
                                                 ros::TransportHints().tcpNoDelay());
    else if (algorithm_type_ == path_recorder::Algorithm::LeGO_LOAM)
        sub_path = nh_.subscribe<nav_msgs::Odometry>(path_topic_,
                                                     100,
                                                     &PathRecorder::LegoloamHandler, this,
                                                     ros::TransportHints().tcpNoDelay());
    else if(algorithm_type_ == path_recorder::Algorithm::LIO_SAM)
        sub_path = nh_.subscribe<nav_msgs::Path>(path_topic_,
                                                 100,
                                                 &PathRecorder::LiosamHandler, this,
                                                 ros::TransportHints().tcpNoDelay());

    std::string bag_fn = save_to_ + "/" + algorithm_name_ + "_path.bag";
    bag.open(bag_fn, rosbag::bagmode::Write);

    while (ros::ok())
    {
        if (updated_)
        {
            std::unique_lock<std::mutex> lock(path_mtx_);
            bag.write(algorithm_name_ + "_path", ros::Time::now(), path_rcvd_);
            updated_ = false;
        }
        ros::spinOnce();
    }

    bag.close();
    std::cout << "Recorded path bag file is saved in: " << bag_fn  << std::endl;
}

void PathRecorder::AloamHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";
    path_rcvd_->poses.push_back(path->poses.back());
    updated_ = true;
}

void PathRecorder::LegoloamHandler(const nav_msgs::Odometry::ConstPtr &odom)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = odom->header.seq;
    path_rcvd_->header.frame_id = "/map";

    auto pose = ros_conversion::Odom2PoseStamped(*odom);
    geometry_msgs::PoseStamped pose_lego_loam;
    pose_lego_loam.header = pose.header;
    pose_lego_loam.pose.position.x = pose.pose.position.z;
    pose_lego_loam.pose.position.y = pose.pose.position.x;
    pose_lego_loam.pose.position.z = pose.pose.position.y;
    path_rcvd_->poses.push_back(pose_lego_loam);
    updated_ = true;
}

void PathRecorder::LiosamHandler(const nav_msgs::Path::ConstPtr &path)
{
    std::unique_lock<std::mutex> lock(path_mtx_);
    path_rcvd_->header.seq = path->header.seq;
    path_rcvd_->header.frame_id = "/map";

    geometry_msgs::PoseStamped liosam_pose;
    liosam_pose.header = path->poses.back().header;
    liosam_pose.pose.position.x = path->poses.back().pose.position.x;
    liosam_pose.pose.position.y = path->poses.back().pose.position.y;
    liosam_pose.pose.position.z = path->poses.back().pose.position.z;
    path_rcvd_->poses.push_back(liosam_pose);
    updated_ = true;
}
