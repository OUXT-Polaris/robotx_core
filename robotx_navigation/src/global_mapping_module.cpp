#include <global_mapping_module.h>

global_mapping_module::global_mapping_module() : tf_listener_(tf_buffer_)
{
    nh_.param<std::string>(ros::this_node::getName()+"/objects_topic_name", objects_topic_name_, ros::this_node::getName()+"/object_roi");
    nh_.param<std::string>(ros::this_node::getName()+"/odom_frame", odom_frame_, "odom");
    nh_.param<std::string>(ros::this_node::getName()+"/map_frame", map_frame_, "map");
    nh_.param<std::string>(ros::this_node::getName()+"/object_roi_frame", object_roi_frame_, "velodyne");
    nh_.param<double>(ros::this_node::getName()+"/matching_distance_threashold", matching_distance_threashold_, 1.0);
    nh_.param<int>(ros::this_node::getName()+"/local_mapping_buffer_length", local_mapping_buffer_length_,10);
    objects_sub_ = nh_.subscribe(objects_topic_name_,3,&global_mapping_module::objects_callback_,this);
}

global_mapping_module::~global_mapping_module()
{

}

void global_mapping_module::objects_callback_(const robotx_msgs::ObjectRegionOfInterestArray msg)
{
    robotx_msgs::ObjectRegionOfInterestArray transformed_rois;
    transformed_rois.header = msg.header;
    geometry_msgs::TransformStamped transform_stamped;
    try
    {
        transform_stamped = tf_buffer_.lookupTransform(odom_frame_, object_roi_frame_, ros::Time(0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return;
    }
    for(auto roi_itr = msg.object_rois.begin(); roi_itr != msg.object_rois.end(); roi_itr++)
    {
        robotx_msgs::ObjectRegionOfInterest transformed_roi;
        transformed_roi.header = roi_itr->header;
        transformed_roi.roi_2d = roi_itr->roi_2d;
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = roi_itr->roi_3d.header;
        pose_stamped.pose = roi_itr->roi_3d.pose;
        geometry_msgs::Vector3Stamped dimensions_stamped;
        dimensions_stamped.header = roi_itr->roi_3d.header;
        dimensions_stamped.vector = roi_itr->roi_3d.dimensions;
        tf2::doTransform(pose_stamped, pose_stamped, transform_stamped);
        tf2::doTransform(dimensions_stamped, dimensions_stamped, transform_stamped);
        transformed_roi.roi_3d.header.stamp = roi_itr->roi_3d.header.stamp;
        transformed_roi.roi_3d.header.frame_id = odom_frame_;
        transformed_roi.roi_3d.dimensions = dimensions_stamped.vector;
        transformed_roi.roi_3d.pose = pose_stamped.pose;
        transformed_rois.object_rois.push_back(transformed_roi);
    }
    return;
}