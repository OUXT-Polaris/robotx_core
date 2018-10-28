#ifndef GLOBAL_MAPPING_MODULE_H_INCLUDED
#define GLOBAL_MAPPING_MODULE_H_INCLUDED

//headers in ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/PoseStamped.h>

//headers in this package
#include <local_mapping_module.h>
#include <robotx_msgs/ObjectRegionOfInterestArray.h>

class global_mapping_module
{
public:
    global_mapping_module();
    ~global_mapping_module();
private:
    void objects_callback_(const robotx_msgs::ObjectRegionOfInterestArray msg);
    std::string objects_topic_name_;
    std::string odom_frame_;
    std::string map_frame_;
    std::string object_roi_frame_;
    double matching_distance_threashold_;
    int local_mapping_buffer_length_;
    ros::NodeHandle nh_;
    ros::Subscriber objects_sub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};
#endif  //GLOBAL_MAPPING_MODULE_H_INCLUDED