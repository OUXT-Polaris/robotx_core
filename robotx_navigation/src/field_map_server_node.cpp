// headers for ros
#include <ros/ros.h>

//headers in this package
#include <field_map_server.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "field_map_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    field_map_server server(nh,pnh);
    server.run();
    ros::spin();
    return 0;
}