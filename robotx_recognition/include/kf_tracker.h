#ifndef KF_TRACKER_H_INCLUDED
#define KF_TRACKER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

//headers in opencv
#include <opencv2/video/tracking.hpp>

//headers in boost
#include <boost/optional.hpp>
#include <boost/shared_ptr.hpp>

//headers in robotx_packages
#include <robotx_msgs/TrackedClusterArray.h>
#include <tracking_module.h>

class kf_tracker
{
public:
    kf_tracker();
    ~kf_tracker();
private:
    void publish_marker_(ros::Time stamp);
    void publish_clusters_(ros::Time stamp);
    void clusters_callback_(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr msg);
    void reset_callback_(const std_msgs::Empty::ConstPtr msg);
    void track_clusters_();
    void reset_();
    boost::optional<int> get_tracker_index_(jsk_recognition_msgs::BoundingBox bbox);
    double min_target_height_;
    double max_target_height_;
    double min_bbox_size_;
    double max_bbox_size_;
    int num_tracking_frames_;
    double matching_distance_threashold_;
    volatile bool recieved_first_time_;
    std::string euclidean_cluster_topic_;
    std::string map_frame_;
    ros::NodeHandle nh_;
    ros::Subscriber euclidean_cluster_sub_;
    ros::Subscriber reset_sub_;
    ros::Publisher tracked_clusters_pub_;
    ros::Publisher marker_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<boost::shared_ptr<tracking_module> > trackers_;
    jsk_recognition_msgs::BoundingBoxArray bbox_data_;
};
#endif  //KF_TRACKER_H_INCLUDED