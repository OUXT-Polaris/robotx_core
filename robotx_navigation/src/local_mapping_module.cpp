#include <local_mapping_module.h>

local_mapping_module::local_mapping_module(int buffer_length, double matching_distance_threashold)
{
    buffer_length_ = buffer_length;
    matching_distance_threashold_ = matching_distance_threashold;
    buf_ = boost::circular_buffer<robotx_msgs::ObjectRegionOfInterestArray>(buffer_length_);
}

local_mapping_module::~local_mapping_module()
{

}

bool local_mapping_module::add_measurement(robotx_msgs::ObjectRegionOfInterestArray measurement)
{
    buf_.push_back(measurement);
    if(buf_.size() == buffer_length_)
    {
        build_();
        return false;
    }
    return true;
}

void local_mapping_module::build_()
{
    return;
}