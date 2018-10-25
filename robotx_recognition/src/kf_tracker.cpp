#include <kf_tracker.h>

kf_tracker::kf_tracker() : tf_listener_(tf_buffer_)
{
    nh_.param<double>(ros::this_node::getName()+"/matching_distance_threashold", matching_distance_threashold_, 1.0);
    nh_.param<double>(ros::this_node::getName()+"/max_target_height", max_target_height_, 3.0);
    nh_.param<double>(ros::this_node::getName()+"/min_target_height", min_target_height_, 0.0);
    nh_.param<std::string>(ros::this_node::getName()+"/euclidean_cluster_topic", euclidean_cluster_topic_, ros::this_node::getName()+"/input_clusters");
    nh_.param<std::string>(ros::this_node::getName()+"/map_frame", map_frame_, "map");
    reset_();
    tracked_clusters_pub_ = nh_.advertise<robotx_msgs::TrackedClusterArray>(ros::this_node::getName()+"/tracked_clusters",1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(ros::this_node::getName()+"/tracked_clusters/marker",1);
    euclidean_cluster_sub_ = nh_.subscribe(euclidean_cluster_topic_, 10, &kf_tracker::clusters_callback_, this);
    reset_sub_ = nh_.subscribe("/reset", 1, &kf_tracker::reset_callback_, this);
}

kf_tracker::~kf_tracker()
{
    
}

void kf_tracker::reset_callback_(const std_msgs::Empty::ConstPtr msg)
{
    reset_();
    return;
}

void kf_tracker::reset_()
{
    tracklers_.clear();
    recieved_fast_time_ = false;
    return;
}

void kf_tracker::track_clusters_()
{
    for(auto bbox_itr = bbox_data_.boxes.begin(); bbox_itr != bbox_data_.boxes.end(); bbox_itr++)
    {
        boost::shared_ptr<tracking_module> module_ptr = boost::make_shared<tracking_module>(map_frame_, tracklers_.size());
        module_ptr->input_measurement(*bbox_itr, bbox_data_.header.stamp);
    }
    return;
}

void kf_tracker::clusters_callback_(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr msg)
{
    jsk_recognition_msgs::BoundingBoxArray transformed_clusters;
    transformed_clusters.header.stamp = msg->header.stamp;
    transformed_clusters.header.frame_id = map_frame_;
    for(auto cluster_itr = msg->boxes.begin(); cluster_itr != msg->boxes.end(); cluster_itr++)
    {
        jsk_recognition_msgs::BoundingBox bbox;
        geometry_msgs::PoseStamped pose;
        pose.pose = cluster_itr->pose;
        pose.header = cluster_itr->header;
        if(cluster_itr->header.frame_id == map_frame_)
        {
            geometry_msgs::TransformStamped transform_stamped;
            try
            {
                transform_stamped = tf_buffer_.lookupTransform(map_frame_, cluster_itr->header.frame_id, ros::Time(0));
                tf2::doTransform(pose, pose, transform_stamped);
            }
            catch (tf2::TransformException &ex)
            {
                ROS_WARN("%s",ex.what());
                continue;
            }
        }
        bbox.header.frame_id = map_frame_;
        bbox.header.stamp = cluster_itr->header.stamp;
        bbox.pose = pose.pose;
        bbox.dimensions = cluster_itr->dimensions;
        bbox.label = cluster_itr->label;
        bbox.value = cluster_itr->value;
        if(min_target_height_ < pose.pose.position.z && pose.pose.position.z < max_target_height_)
        {
            transformed_clusters.boxes.push_back(bbox);
        }
    }
    bbox_data_ = transformed_clusters;
    track_clusters_();
    return;
}