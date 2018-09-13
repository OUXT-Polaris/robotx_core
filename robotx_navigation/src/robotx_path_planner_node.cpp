// headers in this package
#include <robotx_path_planner.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "robotx_path_planner_node");
  robotx_path_planner planner;
  ros::spin();
  return 0;
}