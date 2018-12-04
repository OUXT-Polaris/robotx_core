// headers in this package
#include <battery_monitor_driver.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "cuda_diagnostic_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  battery_monitor_driver driver(nh,pnh);
  ros::spin();
  return 0;
}