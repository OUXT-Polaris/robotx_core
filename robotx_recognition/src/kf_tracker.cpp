#include <kf_tracker.h>

kf_tracker::kf_tracker()
{
    nh_.param<int>(ros::this_node::getName()+"/num_tracking_frames", num_tracking_frames_, 3);
    nh_.param<double>(ros::this_node::getName()+"/matching_distance_threashold", matching_distance_threashold_, 1.0);
    nh_.param<double>(ros::this_node::getName()+"/max_target_height", max_target_height_, 3.0);
    nh_.param<double>(ros::this_node::getName()+"/min_target_height", min_target_height_, 0.0);
    nh_.param<std::string>(ros::this_node::getName()+"/euclidean_cluster_topic", euclidean_cluster_topic_, ros::this_node::getName()+"/input_clusters");
    tracking_targets_ = boost::circular_buffer<jsk_recognition_msgs::BoundingBoxArray>(num_tracking_frames_);
    euclidean_cluster_sub_ = nh_.subscribe(euclidean_cluster_topic_, 10, &kf_tracker::clusters_callback_, this);
}

kf_tracker::~kf_tracker()
{
    
}

void kf_tracker::clusters_callback_(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr msg)
{
    tracking_targets_.push_back(*msg);
    return;
}