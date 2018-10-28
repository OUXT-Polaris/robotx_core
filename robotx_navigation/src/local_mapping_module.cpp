#include <local_mapping_module.h>

local_mapping_module::local_mapping_module(int buffer_length)
{
    buf_ = boost::circular_buffer<robotx_msgs::ObjectRegionOfInterestArray>(buffer_length);
}

local_mapping_module::~local_mapping_module()
{

}

void local_mapping_module::add_measurement(robotx_msgs::ObjectRegionOfInterestArray measurement)
{
    return;
}