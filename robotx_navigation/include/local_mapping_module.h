#ifndef LOCAL_MAPPING_MODULE_H_INCLUDED
#define LOCAL_MAPPING_MODULE_H_INCLUDED

//headers in boost
#include <boost/circular_buffer.hpp>

//headers in this package
#include <robotx_msgs/ObjectRegionOfInterestArray.h>

class local_mapping_module
{
public:
    local_mapping_module(int buffer_length);
    ~local_mapping_module();
    void add_measurement(robotx_msgs::ObjectRegionOfInterestArray measurement);
private:
    boost::circular_buffer<robotx_msgs::ObjectRegionOfInterestArray> buf_;
};
#endif //LOCAL_MAPPING_MODULE_H_INCLUDED