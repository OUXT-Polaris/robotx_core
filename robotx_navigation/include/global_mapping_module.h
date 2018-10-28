#ifndef GLOBAL_MAPPING_MODULE_H_INCLUDED
#define GLOBAL_MAPPING_MODULE_H_INCLUDED

//headers in ROS
#include <ros/ros.h>

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
    double matching_distance_threashold_;
    ros::NodeHandle nh_;
    ros::Subscriber objects_sub_;
};
#endif  //GLOBAL_MAPPING_MODULE_H_INCLUDED