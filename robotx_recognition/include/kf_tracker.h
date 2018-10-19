#ifndef KF_TRACKER_H_INCLUDED
#define KF_TRACKER_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

//headers in opencv
#include <opencv2/video/tracking.hpp>

//headers in boost
#include <boost/circular_buffer.hpp>
#include <boost/optional.hpp>

//headers in robotx_packages
#include <robotx_msgs/TrackedClusterArray.h>

class kf_tracker
{
public:
    kf_tracker();
    ~kf_tracker();
private:
    void clusters_callback_(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr msg);
    boost::optional<int> get_nearest_bbox_index_(jsk_recognition_msgs::BoundingBox target_bbox,jsk_recognition_msgs::BoundingBoxArray query_targets);
    void track_clusters_();
    double min_target_height_;
    double max_target_height_;
    int num_tracking_frames_;
    double matching_distance_threashold_;
    std::string euclidean_cluster_topic_;
    std::string map_frame_;
    ros::NodeHandle nh_;
    ros::Subscriber euclidean_cluster_sub_;
    ros::Publisher tracked_clusters_pub_;
    ros::Publisher marker_pub_;
    //boost::circular_buffer<jsk_recognition_msgs::BoundingBoxArray> tracking_targets_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
#endif  //KF_TRACKER_H_INCLUDED