// headers in this package
#include <world_pose_publisher.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "world_pose_publisher_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    world_pose_publisher publisher(nh,pnh);
    ros::spin();
    return 0;
}