#include <kf_tracker.h>

kf_tracker::kf_tracker() : tf_listener_(tf_buffer_)
{
    //nh_.param<int>(ros::this_node::getName()+"/num_tracking_frames", num_tracking_frames_, 3);
    nh_.param<double>(ros::this_node::getName()+"/matching_distance_threashold", matching_distance_threashold_, 1.0);
    nh_.param<double>(ros::this_node::getName()+"/max_target_height", max_target_height_, 3.0);
    nh_.param<double>(ros::this_node::getName()+"/min_target_height", min_target_height_, 0.0);
    nh_.param<std::string>(ros::this_node::getName()+"/euclidean_cluster_topic", euclidean_cluster_topic_, ros::this_node::getName()+"/input_clusters");
    nh_.param<std::string>(ros::this_node::getName()+"/map_frame", map_frame_, "map");
    //tracking_targets_ = boost::circular_buffer<jsk_recognition_msgs::BoundingBoxArray>(num_tracking_frames_+1);
    tracked_clusters_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>(ros::this_node::getName()+"/tracked_clusters",1);
    euclidean_cluster_sub_ = nh_.subscribe(euclidean_cluster_topic_, 10, &kf_tracker::clusters_callback_, this);
}

kf_tracker::~kf_tracker()
{
    
}

void kf_tracker::track_clusters_()
{
    /*
    if(tracking_targets_.size() > 1)
    {
        for(int i=0; i<tracking_targets_.size()-1;i++)
        {
            jsk_recognition_msgs::BoundingBoxArray bbox_array;
            for(auto target_bbox_itr = tracking_targets_[i].boxes.begin(); target_bbox_itr != tracking_targets_[i].boxes.end(); target_bbox_itr++)
            {
                boost::optional<int> idx = get_nearest_bbox_index_(*target_bbox_itr, tracking_targets_[i+1]);
            }
            cv::KalmanFilter KF(2, 2, 0, CV_32F);
            cv::Mat state(2, 1, CV_32F);
            cv::Mat processNoise(2, 1, CV_32F);
            cv::Mat measurement = cv::Mat::zeros(1, 1, CV_32F);
        }
    }
    */
    return;
}

boost::optional<int> kf_tracker::get_nearest_bbox_index_(jsk_recognition_msgs::BoundingBox target_bbox, jsk_recognition_msgs::BoundingBoxArray query_targets)
{
    if(query_targets.boxes.size() == 0)
    {
        return boost::none;
    }
    std::vector<double> ranges(query_targets.boxes.size());
    int i = 0;
    for(auto query_targets_itr = query_targets.boxes.begin(); query_targets_itr != query_targets.boxes.end(); query_targets_itr++)
    {
        ranges[i] = std::sqrt(std::pow(query_targets_itr->pose.position.x-target_bbox.pose.position.x,2) 
            + std::pow(query_targets_itr->pose.position.y-target_bbox.pose.position.y,2));
        i++;
    }
    if(ranges[*std::min_element(ranges.begin(), ranges.end())] < matching_distance_threashold_)
    {
        return *std::min_element(ranges.begin(), ranges.end());
    }
    return boost::none;
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
    //tracking_targets_.push_back(transformed_clusters);
    return;
}