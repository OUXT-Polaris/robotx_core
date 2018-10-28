#ifndef LOCAL_MAPPING_MODULE_H_INCLUDED
#define LOCAL_MAPPING_MODULE_H_INCLUDED

//headers in boost
#include <boost/circular_buffer.hpp>

//headers in this package
#include <robotx_msgs/ObjectRegionOfInterestArray.h>

class local_mapping_module
{
public:
    local_mapping_module(int buffer_length, double matching_distance_threashold);
    ~local_mapping_module();
    bool add_measurement(robotx_msgs::ObjectRegionOfInterestArray measurement);
private:
    void build_();
    boost::circular_buffer<robotx_msgs::ObjectRegionOfInterestArray> buf_;
    int buffer_length_;
    double matching_distance_threashold_;
};
#endif //LOCAL_MAPPING_MODULE_H_INCLUDED