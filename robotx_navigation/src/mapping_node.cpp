// headers for ros
#include <ros/ros.h>

//headers in this package
#include <global_mapping_module.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mapping_node");
    global_mapping_module mapper;
    ros::spin();
    return 0;
}