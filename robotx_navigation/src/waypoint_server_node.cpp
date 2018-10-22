// headers in this package
#include <waypoint_server.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "waypoint_server_node");
  waypoint_server server;
  ros::spin();
  return 0;
}