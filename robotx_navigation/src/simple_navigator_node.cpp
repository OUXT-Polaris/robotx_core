//headers in this package
#include <simple_navigator.h>
//headers in ROS
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "simple_navigator_node");
  simple_navigator nav;
  ros::spin();
  return 0;
}