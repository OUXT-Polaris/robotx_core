// headers for ros
#include <ros/ros.h>

//headers in this package
#include <field_map_server.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "field_map_server_node");
    ros::spin();
    return 0;
}