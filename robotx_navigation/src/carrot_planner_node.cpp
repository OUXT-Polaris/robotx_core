// headers for ros
#include <ros/ros.h>

//headers in this package
#include <carrot_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "costmap_clear_request_sender_node");
    carrot_planner planner;
    planner.run();
    ros::spin();
    return 0;
}
