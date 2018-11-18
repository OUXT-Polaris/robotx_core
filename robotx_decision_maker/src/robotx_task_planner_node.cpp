// headers for ros
#include <ros/ros.h>

//headers in this package
#include <robotx_task_planner.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robotx_task_planner_node");
    robotx_task_planner planner;
    ros::spin();
    return 0;
}