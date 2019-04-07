// headers in this package
#include <imu_gravity_removal.h>

// headers for ros
#include <ros/ros.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "imu_gravity_removal_node");
  imu_gravity_removal i_g_r;
  ros::spin();
  return 0;
}