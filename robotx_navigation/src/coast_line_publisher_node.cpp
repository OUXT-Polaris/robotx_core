// headers for ros
#include <ros/ros.h>

// headers in this package
#include <coast_line_publisher.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "coast_line_publisher_node");
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_("~");
    coast_line_publisher publisher(nh_,pnh_);
    publisher.run();
    return 0;
}
