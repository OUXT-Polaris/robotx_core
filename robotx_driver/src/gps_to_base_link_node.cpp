// headers in this package
#include <gps_to_base_link.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "gps_to_base_link_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  gps_to_base_link publisher(nh,pnh);
  ros::spin();
  return 0;
}