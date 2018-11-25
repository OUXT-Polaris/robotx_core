// headers for ros
#include <ros/ros.h>

//headers in this package
#include <robotx_navigation_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotx_navigation_planner_node");
    robotx_navigation_planner planner;
    planner.run();
    ros::spin();
    return 0;
}