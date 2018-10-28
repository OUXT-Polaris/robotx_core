#include <global_mapping_module.h>

global_mapping_module::global_mapping_module()
{
    nh_.param<std::string>(ros::this_node::getName()+"/objects_topic_name", objects_topic_name_,ros::this_node::getName()+"/object_roi");
    nh_.param<double>(ros::this_node::getName()+"matching_distance_threashold", matching_distance_threashold_, 1.0);
    objects_sub_ = nh_.subscribe(objects_topic_name_,3,&global_mapping_module::objects_callback_,this);
}

global_mapping_module::~global_mapping_module()
{

}

void global_mapping_module::objects_callback_(const robotx_msgs::ObjectRegionOfInterestArray msg)
{
    return;
}