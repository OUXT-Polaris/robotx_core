// headers in this package
#include <waypoint_logger.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "waypoint_logger_node");
  waypoint_logger logger_;
  ros::spin();
  return 0;
}