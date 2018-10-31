// headers in this package
#include <obstacle_avoid.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "obstacle_avoid");
  obstacle_avoid avoider;
  ros::spin();
  return 0;
}